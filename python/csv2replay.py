#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
csv2replay.py
=============
Generic CSV (any logger: ROS bag export, motion capture, custom FC, ...) -> Unity replay JSON/CSV.
Same output format as bin2replay.py, so the same Unity scripts play it.

You tell the script which columns hold time / position / attitude and which coordinate
convention they use. Everything is converted to NED/FRD internally, then to Unity.

Position conventions (--pos-frame)
    ned     x=North, y=East, z=Down        (ArduPilot, PX4 local)
    enu     x=East,  y=North, z=Up         (ROS REP-103 'map'/'odom')
    unity   x=East,  y=Up,    z=North      (already Unity, left-handed)
    geo     latitude [deg], longitude [deg], altitude [m, up]  -> local NED (flat earth)

Attitude conventions (--att-frame)
    ned_frd body x=Forward, y=Right, z=Down in NED world  (aerospace; ArduPilot/PX4)
    enu_flu body x=Forward, y=Left,  z=Up   in ENU world  (ROS REP-103; yaw 0 = East, CCW+)
    unity   Unity rotation (quaternion only)

Attitude given either as Euler (--euler-cols roll,pitch,yaw ; intrinsic Z-Y-X, i.e.
R = Rz(yaw) Ry(pitch) Rx(roll)) or as quaternion (--quat-cols a,b,c,d with --quat-order wxyz|xyzw).

Examples
--------
  # PX4/ArduPilot-style CSV, time in microseconds, Euler in degrees
  python csv2replay.py log.csv --time-col TimeUS --time-unit us \
      --pos-cols PN,PE,PD --pos-frame ned \
      --euler-cols Roll,Pitch,Yaw --angle-unit deg --att-frame ned_frd

  # ROS odometry export (ENU / FLU, quaternion x,y,z,w), no arm
  python csv2replay.py odom.csv --time-col stamp \
      --pos-cols x,y,z --pos-frame enu \
      --quat-cols qx,qy,qz,qw --quat-order xyzw --att-frame enu_flu --arm-length 0

  # inspect the header first
  python csv2replay.py log.csv --dump-columns
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # find replaylib.py next to this script
from replaylib import (A_ENU_TO_NED, P_NED_TO_UNITY, R_enu_flu_to_R_ned_frd, R_unity_to_R_ned,
                       add_common_args, clean_series, euler_zyx_to_R, export, geodetic_to_local_ned,
                       make_meta, process, quat_xyzw_to_R, geo_of_replay_origin)

TIME_SCALE = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}


def read_csv_columns(path: str, delimiter: Optional[str]) -> Dict[str, np.ndarray]:
    with open(path, newline="", encoding="utf-8-sig") as f:
        sample = f.read(4096)
        f.seek(0)
        if delimiter is None:
            try:
                delimiter = csv.Sniffer().sniff(sample, delimiters=",;\t ").delimiter
            except csv.Error:
                delimiter = ","
        rd = csv.reader(f, delimiter=delimiter)
        header = [h.strip() for h in next(rd)]
        cols: Dict[str, list] = {h: [] for h in header}
        for row in rd:
            if not row or all(not c.strip() for c in row):
                continue
            for h, v in zip(header, row + [""] * (len(header) - len(row))):
                v = v.strip()
                try:
                    cols[h].append(float(v) if v != "" else np.nan)
                except ValueError:
                    cols[h].append(np.nan)   # non-numeric cell -> treated as missing
    return {h: np.asarray(v, float) for h, v in cols.items()}


def dump_columns(path: str, delimiter: Optional[str]) -> None:
    cols = read_csv_columns(path, delimiter)
    n = len(next(iter(cols.values()))) if cols else 0
    print(f"{n} rows, {len(cols)} columns")
    for h, v in cols.items():
        fin = np.isfinite(v)
        rng = f"{np.nanmin(v):.6g} .. {np.nanmax(v):.6g}" if fin.any() else "(no numeric data)"
        print(f"  {h:24s} valid={int(fin.sum()):7d}  range={rng}")


def _pick(cols: Dict[str, np.ndarray], spec: str, n: int) -> List[np.ndarray]:
    names = [s.strip() for s in spec.split(",")]
    if len(names) != n:
        raise SystemExit(f"expected {n} column names, got '{spec}'")
    missing = [c for c in names if c not in cols]
    if missing:
        raise SystemExit(f"column(s) not found: {missing}. Available: {list(cols)}")
    return [cols[c] for c in names]


def position_to_ned(p: np.ndarray, frame: str) -> np.ndarray:
    if frame == "ned":
        return p
    if frame == "enu":
        return p @ A_ENU_TO_NED.T
    if frame == "unity":
        return p @ P_NED_TO_UNITY          # inverse of v_u = P v_ned  (P orthogonal: P^-1 = P^T)
    if frame == "geo":
        return geodetic_to_local_ned(p[:, 0], p[:, 1], p[:, 2])
    raise ValueError(frame)


def attitude_to_R_ned(kind: str, vals: np.ndarray, frame: str, angle_unit: str, quat_order: str) -> np.ndarray:
    if kind == "euler":
        if frame == "unity":
            raise SystemExit("--att-frame unity is supported for quaternions only")
        e = np.radians(vals) if angle_unit == "deg" else vals
        R = euler_zyx_to_R(e[:, 0], e[:, 1], e[:, 2])
    else:
        q = vals if quat_order == "xyzw" else vals[:, [1, 2, 3, 0]]
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        R = quat_xyzw_to_R(q)
    if frame == "ned_frd":
        return R
    if frame == "enu_flu":
        return R_enu_flu_to_R_ned_frd(R)
    if frame == "unity":
        return R_unity_to_R_ned(R)
    raise ValueError(frame)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="generic CSV -> Unity replay JSON/CSV",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("csv", help="input CSV with a header row")
    ap.add_argument("--dump-columns", action="store_true", help="print columns and value ranges, then exit")
    ap.add_argument("--delimiter", default=None, help="field delimiter (default: auto-detect)")
    ap.add_argument("--time-col", help="time column name")
    ap.add_argument("--time-unit", choices=list(TIME_SCALE), default="s")
    ap.add_argument("--pos-cols", help="3 position column names, e.g. x,y,z or lat,lon,alt")
    ap.add_argument("--pos-frame", choices=["ned", "enu", "unity", "geo"], default="ned")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--euler-cols", help="roll,pitch,yaw column names (Z-Y-X)")
    g.add_argument("--quat-cols", help="4 quaternion column names, order given by --quat-order")
    ap.add_argument("--quat-order", choices=["wxyz", "xyzw"], default="wxyz")
    ap.add_argument("--att-frame", choices=["ned_frd", "enu_flu", "unity"], default="ned_frd")
    ap.add_argument("--angle-unit", choices=["deg", "rad"], default="deg")
    ap.add_argument("--des-euler-cols", default=None,
                    help="optional desired roll,pitch,yaw (same unit/frame) -> attitude-error colouring")
    ap.add_argument("--geo-origin", default=None, metavar="LAT,LON,ALT",
                    help="geodetic position of the position-frame origin (for ned/enu/unity input), "
                         "used to place real-world scenery. Not needed with --pos-frame geo")
    ap.add_argument("--motor-cols", default=None,
                    help="optional motor output columns for motor 1..n (raw PWM or any unit; converted in Unity)")
    ap.add_argument("--motor-range", default="1000,2000", help="min,max of the motor column values (meta only)")
    add_common_args(ap)
    a = ap.parse_args(argv)

    if a.dump_columns:
        dump_columns(a.csv, a.delimiter)
        return 0
    if not (a.time_col and a.pos_cols and (a.euler_cols or a.quat_cols)):
        ap.error("--time-col, --pos-cols and one of --euler-cols / --quat-cols are required")

    cols = read_csv_columns(a.csv, a.delimiter)
    t = cols.get(a.time_col)
    if t is None:
        raise SystemExit(f"time column '{a.time_col}' not found. Available: {list(cols)}")
    t = t * TIME_SCALE[a.time_unit]

    # position and attitude may come from different rows (different sensors); clean each separately
    t_pos, pcols = clean_series(t, *_pick(cols, a.pos_cols, 3))
    pos_ned = position_to_ned(np.column_stack(pcols), a.pos_frame)

    if a.euler_cols:
        kind, acols = "euler", _pick(cols, a.euler_cols, 3)
    else:
        kind, acols = "quat", _pick(cols, a.quat_cols, 4)
    dcols = _pick(cols, a.des_euler_cols, 3) if a.des_euler_cols else []
    t_att, ac = clean_series(t, *acols, *dcols)
    nA = len(acols)
    R_att = attitude_to_R_ned(kind, np.column_stack(ac[:nA]), a.att_frame, a.angle_unit, a.quat_order)
    R_des = attitude_to_R_ned("euler", np.column_stack(ac[nA:]), a.att_frame, a.angle_unit, a.quat_order) if dcols else None

    motors, motor_meta = None, {}
    if a.motor_cols:
        names = [c.strip() for c in a.motor_cols.split(",")]
        t_m, mc = clean_series(t, *_pick(cols, a.motor_cols, len(names)))
        motors = (t_m, np.column_stack(mc))
        lo, hi = (float(v) for v in a.motor_range.split(","))
        motor_meta = {"motor_count": len(names), "motor_channels": list(range(1, len(names) + 1)),
                      "motor_channel_source": "--motor-cols " + a.motor_cols, "pwm_min": lo, "pwm_max": hi,
                      "frame_class": -1.0, "frame_type": -1.0, "motor_source": "csv"}
    print(f"[csv ] {a.csv}: pos samples={len(t_pos)} ({a.pos_frame}), att samples={len(t_att)} ({kind}, {a.att_frame})")
    res = process(t_att, R_att, R_des, t_pos, pos_ned, rate_hz=a.rate, max_gap=a.max_gap,
                  arm_length=a.arm_length, origin_shift=not a.no_origin_shift,
                  t_start=a.t_start, t_end=a.t_end,
                  skip_gaps=a.skip_gaps, min_segment_s=a.min_segment,
                  arm_mount_frd=a.arm_mount, motors=motors)
    out = a.out or os.path.splitext(a.csv)[0] + "_replay"
    meta = make_meta(os.path.basename(a.csv), f"csv:{a.pos_frame}/{a.att_frame}", a, float(res["t_abs"][0]))
    meta.update(motor_meta)
    # geodetic origin of the replay (for scenery): source origin geo + shift applied by origin_shift
    geo0 = None
    if a.pos_frame == "geo":
        raw = np.column_stack(pcols)
        geo0 = (float(raw[0, 0]), float(raw[0, 1]), float(raw[0, 2]))
    elif a.geo_origin:
        geo0 = tuple(float(v) for v in a.geo_origin.split(","))
    if geo0 is not None:
        o = geo_of_replay_origin(*geo0, -np.asarray(res["origin_unity"]))
        meta.update({"geo_origin": [o[0], o[1], o[2]],
                     "geo_origin_note": "lat [deg], lon [deg], alt [m] of Unity (0,0,0)"})
    export(res, out, meta)
    print(f"[done] {out}.json / .csv   frames={len(res['t'])}  valid={int(res['valid'].sum())}  duration={res['t'][-1]:.1f}s  segments={len(res['segments'])}")
    for sg in res["segments"]:
        print(f"       seg {sg['seg']}: replay {sg['t_start']:8.2f}-{sg['t_end']:8.2f} s   source {sg['t_src_start']:8.2f}-{sg['t_src_end']:8.2f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
