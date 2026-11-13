#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bin2replay.py
=============
ArduPilot DataFlash log (.bin) -> Unity replay data (JSON + CSV).

The source-independent part (frame conversion, resampling, export) is in replaylib.py.
For other loggers (ROS, motion capture, ...) use csv2replay.py.

Pipeline
--------
1. Read ATT (attitude) and XKF1 (EKF3 local NED position) messages with pymavlink.
   Fallbacks: NKF1 (EKF2) or POS (lat/lon/alt -> local NED, flat-earth).
2. Clean: drop non-finite rows, drop non-monotonic / duplicate timestamps.
3. Convert frames  NED (world) / FRD (body)  ->  Unity left-handed, Y-up.
4. Resample to a uniform rate (position: linear, attitude: quaternion SLERP).
   Samples inside a source gap longer than --max-gap are flagged valid=0.
5. Compute arm tip position (rigid, massless arm of length L along body +X / forward).
6. Export JSON (for Unity JsonUtility) and CSV (for inspection / plotting).

Coordinate conventions (see README "Coordinate frames")
-------------------------------------------------------
ArduPilot world : NED  (x=North, y=East, z=Down), right-handed
ArduPilot body  : FRD  (x=Forward, y=Right, z=Down), right-handed
Unity world     : (x=East, y=Up, z=North), left-handed
Unity body      : (x=Right, y=Up, z=Forward), left-handed

Both world and body use the same permutation matrix P:
    v_unity = P @ v_ned,   P = [[0,1,0],[0,0,-1],[1,0,0]]   (det P = -1 -> handedness flip)
Rotation (body->world):
    R_unity = P @ R_ned @ P^T   (det = +1, a proper rotation in Unity space)
Equivalent Unity Euler angles (Unity applies Z, then X, then Y):
    unity_euler = (-pitch, yaw, -roll)   [deg]

Usage
-----
    python bin2replay.py flight.bin -o out/flight_replay --rate 50
    python bin2replay.py flight.bin --dump-fields      # list available fields only
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # find replaylib.py next to this script
from replaylib import (add_common_args, euler_zyx_to_R, export, geodetic_to_local_ned,  # noqa: F401
                       make_meta, process, clean_series, geo_of_replay_origin)

# ---------------------------------------------------------------------------
# 2. Log reading (pymavlink DFReader)
# ---------------------------------------------------------------------------
WANTED = ["ATT", "XKF1", "NKF1", "POS", "RCOU"]
READ_TYPES = WANTED + ["PARM"]


def read_bin(path: str, types: List[str]) -> Dict[str, List[dict]]:
    from pymavlink import mavutil  # imported here so the math part is testable without pymavlink
    mlog = mavutil.mavlink_connection(path)
    out: Dict[str, List[dict]] = {t: [] for t in types}
    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break
        t = m.get_type()
        if t in out:
            out[t].append(m.to_dict())
    return out


def dump_fields(path: str) -> None:
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path)
    seen = {}
    while True:
        m = mlog.recv_match(type=WANTED)
        if m is None:
            break
        if m.get_type() not in seen:
            seen[m.get_type()] = list(m.get_fieldnames())
    for k in WANTED:
        print(f"{k:5s}: {seen.get(k, '(not found)')}")


def _col(rows: List[dict], key: str) -> np.ndarray:
    return np.array([r.get(key, np.nan) for r in rows], dtype=float)


def extract_attitude(logs):
    """ATT: Roll/Pitch/Yaw [deg] and DesRoll/DesPitch/DesYaw [deg].
    [要出典確認: ATT のフィールド名は ArduPilot の版で異なる場合がある。--dump-fields で確認すること]"""
    rows = logs["ATT"]
    if not rows:
        raise RuntimeError("ATT message not found in log. Check LOG_BITMASK (attitude logging).")
    t = _col(rows, "TimeUS") * 1e-6
    r, p, y = _col(rows, "Roll"), _col(rows, "Pitch"), _col(rows, "Yaw")
    has_des = all(k in rows[0] for k in ("DesRoll", "DesPitch", "DesYaw"))
    if has_des:
        dr, dp, dy = _col(rows, "DesRoll"), _col(rows, "DesPitch"), _col(rows, "DesYaw")
        t, (r, p, y, dr, dp, dy) = clean_series(t, r, p, y, dr, dp, dy)
        des = np.radians(np.column_stack([dr, dp, dy]))
    else:
        t, (r, p, y) = clean_series(t, r, p, y)
        des = None
    return t, np.radians(np.column_stack([r, p, y])), des


def extract_position(logs, source: str, ekf_core: int):
    """Returns t [s], pos_ned (N,3) [m], source name actually used."""
    order = {"auto": ["XKF1", "NKF1", "POS"], "xkf1": ["XKF1"], "nkf1": ["NKF1"], "pos": ["POS"]}[source]
    for name in order:
        rows = logs.get(name, [])
        if not rows:
            continue
        if name in ("XKF1", "NKF1"):
            # [要出典確認: XKF1/NKF1 の PN/PE/PD は EKF 原点基準の NED 位置 [m]、C はコア番号]
            if "C" in rows[0]:
                rows = [r for r in rows if int(r.get("C", -1)) == ekf_core]
                if not rows:
                    continue
            t = _col(rows, "TimeUS") * 1e-6
            t, (n, e, d) = clean_series(t, _col(rows, "PN"), _col(rows, "PE"), _col(rows, "PD"))
            return t, np.column_stack([n, e, d]), name
        if name == "POS":
            # [要出典確認: POS の Lat/Lng は pymavlink で度単位に換算済み、Alt は海抜 m]
            t = _col(rows, "TimeUS") * 1e-6
            t, (lat, lng, alt) = clean_series(t, _col(rows, "Lat"), _col(rows, "Lng"), _col(rows, "Alt"))
            return t, geodetic_to_local_ned(lat, lng, alt), name
    raise RuntimeError(f"No usable position message found (tried {order}).")


def read_params(logs) -> Dict[str, float]:
    return {r["Name"]: float(r["Value"]) for r in logs.get("PARM", []) if "Name" in r}


def extract_motors(logs, params: Dict[str, float], channels_arg: Optional[str]):
    """RCOU raw outputs [PWM us] of motor 1..n.
    Motor -> output channel from SERVOn_FUNCTION (33 = Motor1, 34 = Motor2, ...)
    [要出典確認: ArduPilot SERVOn_FUNCTION の Motor1=33 ...]. Override with --motor-channels."""
    rows = logs.get("RCOU", [])
    if not rows:
        return None, {}
    if channels_arg:
        chans = [int(c) for c in channels_arg.split(",")]
        how = "--motor-channels"
    else:
        fn = {int(k[5:-9]): int(v) for k, v in params.items()
              if k.startswith("SERVO") and k.endswith("_FUNCTION") and k[5:-9].isdigit()}
        motor_to_ch = {f - 32: ch for ch, f in fn.items() if 33 <= f <= 40}
        if motor_to_ch:
            chans = [motor_to_ch[m] for m in sorted(motor_to_ch)]
            how = "SERVOn_FUNCTION"
        else:
            chans, how = [1, 2, 3, 4], "default C1-C4"
    t = _col(rows, "TimeUS") * 1e-6
    t, cols = clean_series(t, *[_col(rows, f"C{c}") for c in chans])
    pmin, pmax = params.get("MOT_PWM_MIN", 0.0), params.get("MOT_PWM_MAX", 0.0)
    if not (pmin > 0 and pmax > pmin):   # 0 = use the throttle channel range
        pmin, pmax = params.get("RC3_MIN", 1000.0), params.get("RC3_MAX", 2000.0)
    meta = {"motor_count": len(chans), "motor_channels": chans, "motor_channel_source": how,
            "pwm_min": pmin, "pwm_max": pmax,
            "frame_class": params.get("FRAME_CLASS", -1.0), "frame_type": params.get("FRAME_TYPE", -1.0),
            "motor_source": "RCOU (commanded PWM, not measured RPM)"}
    return (t, np.column_stack(cols)), meta


def geo_origin_from_pos(logs, res):
    """Latitude/longitude/altitude of the replay origin, from POS interpolated at the first replay frame.
    POS.Alt is used as is [要出典確認: ArduPilot POS.Alt は海抜高度 (AMSL)]."""
    rows = logs.get("POS", [])
    if not rows:
        return None
    t = _col(rows, "TimeUS") * 1e-6
    t, (lat, lng, alt) = clean_series(t, _col(rows, "Lat"), _col(rows, "Lng"), _col(rows, "Alt"))
    if len(t) < 2:
        return None
    t_ref = float(res["t_abs"][0])
    lat_r, lon_r, alt_r = (float(np.interp(t_ref, t, v)) for v in (lat, lng, alt))
    o = geo_of_replay_origin(lat_r, lon_r, alt_r, res["pos"][0])
    return {"geo_origin": [o[0], o[1], o[2]],
            "geo_origin_note": "lat [deg], lon [deg], alt [m] of Unity (0,0,0); from POS at the first frame; alt datum = POS.Alt"}


# ---------------------------------------------------------------------------
# 5. CLI
# ---------------------------------------------------------------------------
def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="ArduPilot .bin -> Unity replay JSON/CSV")
    ap.add_argument("bin", help="ArduPilot DataFlash log (.bin)")
    add_common_args(ap)
    ap.add_argument("--pos-source", choices=["auto", "xkf1", "nkf1", "pos"], default="auto")
    ap.add_argument("--ekf-core", type=int, default=0, help="EKF core index for XKF1/NKF1 (default 0)")
    ap.add_argument("--motor-channels", default=None,
                    help="output channels of motor 1..n, e.g. 1,2,3,4 (default: from SERVOn_FUNCTION in the log)")
    ap.add_argument("--no-motors", action="store_true", help="do not export motor outputs")
    ap.add_argument("--dump-fields", action="store_true", help="print available fields and exit")
    a = ap.parse_args(argv)

    if a.dump_fields:
        dump_fields(a.bin)
        return 0

    out = a.out or os.path.splitext(a.bin)[0] + "_replay"
    print(f"[read] {a.bin}")
    logs = read_bin(a.bin, READ_TYPES)
    print("[read] counts: " + ", ".join(f"{k}={len(v)}" for k, v in logs.items()))
    t_att, eul, des = extract_attitude(logs)
    t_pos, pos_ned, src = extract_position(logs, a.pos_source, a.ekf_core)
    print(f"[pos ] source={src}  samples={len(t_pos)}   [att] samples={len(t_att)}  desired={'yes' if des is not None else 'no'}")

    motors, motor_meta = (None, {}) if a.no_motors else extract_motors(logs, read_params(logs), a.motor_channels)
    if motors is not None:
        print(f"[mot ] {motor_meta['motor_count']} motors, channels {motor_meta['motor_channels']} "
              f"({motor_meta['motor_channel_source']}), PWM {motor_meta['pwm_min']:.0f}-{motor_meta['pwm_max']:.0f}, "
              f"FRAME_CLASS={motor_meta['frame_class']:.0f} FRAME_TYPE={motor_meta['frame_type']:.0f}")

    R_att = euler_zyx_to_R(eul[:, 0], eul[:, 1], eul[:, 2])
    R_des = euler_zyx_to_R(des[:, 0], des[:, 1], des[:, 2]) if des is not None else None
    res = process(t_att, R_att, R_des, t_pos, pos_ned, rate_hz=a.rate, max_gap=a.max_gap,
                  arm_length=a.arm_length, origin_shift=not a.no_origin_shift,
                  t_start=a.t_start, t_end=a.t_end,
                  skip_gaps=a.skip_gaps, min_segment_s=a.min_segment,
                  arm_mount_frd=a.arm_mount, motors=motors)
    meta = make_meta(os.path.basename(a.bin), src, a, float(res["t_abs"][0]))
    meta.update(motor_meta)
    geo = geo_origin_from_pos(logs, res)
    if geo is not None:
        meta.update(geo)
        print(f"[geo ] replay origin  lat {geo['geo_origin'][0]:.7f}  lon {geo['geo_origin'][1]:.7f}  alt {geo['geo_origin'][2]:.1f} m (POS.Alt datum)")
    export(res, out, meta)
    nv = int(res["valid"].sum())
    print(f"[done] {out}.json / .csv   frames={len(res['t'])}  valid={nv}  duration={res['t'][-1]:.1f}s  segments={len(res['segments'])}")
    for sg in res["segments"]:
        print(f"       seg {sg['seg']}: replay {sg['t_start']:8.2f}-{sg['t_end']:8.2f} s   source {sg['t_src_start']:8.2f}-{sg['t_src_end']:8.2f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
