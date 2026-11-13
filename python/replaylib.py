# -*- coding: utf-8 -*-
"""
replaylib.py -- source-independent part of the replay pipeline.

Input to `process()` is always:   time [s], rotation body(FRD)->world(NED), position NED [m].
Readers (bin2replay.py, csv2replay.py) convert their own formats into that first.
Output frame: Unity left-handed Y-up  (x=East, y=Up, z=North), body (x=Right, y=Up, z=Forward).
"""
from __future__ import annotations

import csv
import json
import math
import os
from typing import Dict, List, Optional

import numpy as np

# ---------------------------------------------------------------------------
# 1. Frame conversion (NED/FRD -> Unity left-handed Y-up)
# ---------------------------------------------------------------------------
P_NED_TO_UNITY = np.array([[0.0, 1.0, 0.0],
                           [0.0, 0.0, -1.0],
                           [1.0, 0.0, 0.0]])


def ned_to_unity_vec(v: np.ndarray) -> np.ndarray:
    """(...,3) NED (or FRD) vectors -> Unity (or Unity-body) vectors."""
    return np.asarray(v, float) @ P_NED_TO_UNITY.T


def euler_zyx_to_R(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Aerospace ZYX Euler (rad) -> rotation matrices body(FRD)->world(NED), shape (N,3,3)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.empty((len(roll), 3, 3))
    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return R


def R_ned_to_R_unity(R_ned: np.ndarray) -> np.ndarray:
    P = P_NED_TO_UNITY
    return P @ R_ned @ P.T


def R_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrices (N,3,3) -> quaternions (N,4) in (x,y,z,w), same convention as Unity Quaternion."""
    N = R.shape[0]
    q = np.empty((N, 4))
    tr = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]
    for i in range(N):
        m = R[i]
        if tr[i] > 0:
            s = math.sqrt(tr[i] + 1.0) * 2
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        q[i] = (x, y, z, w)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q


def quat_xyzw_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def make_quat_continuous(q: np.ndarray) -> np.ndarray:
    """Flip signs so consecutive quaternions lie in the same hemisphere (q and -q are the same rotation)."""
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] = -q[i]
    return q


def slerp(q0: np.ndarray, q1: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Vectorised SLERP. q0,q1: (N,4), u: (N,) in [0,1]."""
    dot = np.sum(q0 * q1, axis=1)
    q1 = np.where(dot[:, None] < 0, -q1, q1)
    dot = np.abs(dot)
    out = np.empty_like(q0)
    lin = dot > 0.9995
    # nearly identical -> normalised lerp
    out[lin] = q0[lin] + u[lin, None] * (q1[lin] - q0[lin])
    th0 = np.arccos(np.clip(dot[~lin], -1, 1))
    s0 = np.sin(th0)
    a = np.sin((1 - u[~lin]) * th0) / s0
    b = np.sin(u[~lin] * th0) / s0
    out[~lin] = a[:, None] * q0[~lin] + b[:, None] * q1[~lin]
    return out / np.linalg.norm(out, axis=1, keepdims=True)



# --- other common frames -> NED/FRD -------------------------------------------
# ENU world (x=East, y=North, z=Up)  ->  NED
A_ENU_TO_NED = np.array([[0.0, 1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, -1.0]])
# FLU body (x=Forward, y=Left, z=Up)  <->  FRD (self-inverse)
B_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def enu_to_ned_vec(v):
    return np.asarray(v, float) @ A_ENU_TO_NED.T


def R_enu_flu_to_R_ned_frd(R_enu: np.ndarray) -> np.ndarray:
    """Rotation body(FLU)->world(ENU)  (ROS REP-103 convention)  ->  body(FRD)->world(NED)."""
    return A_ENU_TO_NED @ R_enu @ B_FLU_FRD


def R_unity_to_R_ned(R_u: np.ndarray) -> np.ndarray:
    P = P_NED_TO_UNITY
    return P.T @ R_u @ P


def geodetic_to_local_ned(lat_deg, lon_deg, alt_m):
    """Flat-earth approximation around the first sample (adequate for a few hundred metres)."""
    R_E = 6378137.0
    lat0, lon0, alt0 = lat_deg[0], lon_deg[0], alt_m[0]
    n = np.radians(lat_deg - lat0) * R_E
    e = np.radians(lon_deg - lon0) * R_E * math.cos(math.radians(lat0))
    d = -(alt_m - alt0)
    return np.column_stack([n, e, d])


def local_ned_to_geodetic(lat0, lon0, alt0, n, e, d):
    """Inverse of geodetic_to_local_ned (flat earth around lat0/lon0)."""
    R_E = 6378137.0
    lat = lat0 + math.degrees(n / R_E)
    lon = lon0 + math.degrees(e / (R_E * math.cos(math.radians(lat0))))
    return lat, lon, alt0 - d


def geo_of_replay_origin(lat_ref, lon_ref, alt_ref, ref_unity):
    """Geodetic position of the replay origin (Unity 0,0,0), given a reference point whose
    geodetic position (lat_ref, lon_ref, alt_ref) and Unity position ref_unity=(E, Up, N) are both known."""
    e, up, n = (float(v) for v in ref_unity)
    return local_ned_to_geodetic(lat_ref, lon_ref, alt_ref, -n, -e, up)


def clean_series(t: np.ndarray, *cols: np.ndarray):
    """Drop rows with any non-finite value, sort by time, drop duplicate/non-increasing timestamps."""
    data = np.column_stack([t, *cols])
    ok = np.all(np.isfinite(data), axis=1)
    data = data[ok]
    data = data[np.argsort(data[:, 0], kind="stable")]
    if len(data) > 1:
        keep = np.concatenate([[True], np.diff(data[:, 0]) > 0])
        data = data[keep]
    return data[:, 0], [data[:, i + 1] for i in range(len(cols))]


# ---------------------------------------------------------------------------
# 3. Processing
# ---------------------------------------------------------------------------
def gap_mask(t_src: np.ndarray, t_grid: np.ndarray, max_gap: float) -> np.ndarray:
    """True where the grid sample lies between two source samples that are <= max_gap apart."""
    idx = np.clip(np.searchsorted(t_src, t_grid, side="right") - 1, 0, len(t_src) - 2)
    span = t_src[idx + 1] - t_src[idx]
    return span <= max_gap


def resample_quat(t_src, q_src, t_grid):
    idx = np.clip(np.searchsorted(t_src, t_grid, side="right") - 1, 0, len(t_src) - 2)
    t0, t1 = t_src[idx], t_src[idx + 1]
    u = np.clip((t_grid - t0) / (t1 - t0), 0.0, 1.0)
    return slerp(q_src[idx], q_src[idx + 1], u)


def rotation_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> np.ndarray:
    """Angle of the relative rotation Ra^T Rb (deg)."""
    Rrel = np.einsum("nji,njk->nik", Ra, Rb)
    c = (np.trace(Rrel, axis1=1, axis2=2) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def find_segments(valid: np.ndarray, min_len: int = 1):
    """Runs of valid==1 as (start, end_exclusive) index pairs, keeping runs with >= min_len samples."""
    v = np.concatenate([[0], valid.astype(int), [0]])
    starts = np.where(np.diff(v) == 1)[0]
    ends = np.where(np.diff(v) == -1)[0]
    return [(a, b) for a, b in zip(starts, ends) if b - a >= min_len]


def process(t_att, R_ned, R_des, t_pos, pos_ned, *, rate_hz, max_gap, arm_length,
            origin_shift=True, t_start=None, t_end=None, skip_gaps=False, min_segment_s=1.0,
            arm_mount_frd=(0.0, 0.0, 0.0), motors=None):
    """motors: optional (t_m (K,), pwm (K, n_motors)) raw motor outputs, resampled like the pose."""
    """t_att: (N,) s, R_ned: (N,3,3) body(FRD)->world(NED), R_des: same shape or None,
    t_pos: (M,) s, pos_ned: (M,3) m.  Everything source-specific is converted to NED/FRD before this."""
    # Attitude -> Unity quaternions at source rate
    q_u = make_quat_continuous(R_to_quat_xyzw(R_ned_to_R_unity(R_ned)))

    att_err = rotation_angle_deg(R_des, R_ned) if R_des is not None else None

    # common time window
    lo = max(t_att[0], t_pos[0])
    hi = min(t_att[-1], t_pos[-1])
    if t_start is not None:
        lo = max(lo, t_start)
    if t_end is not None:
        hi = min(hi, t_end)
    if hi <= lo:
        raise RuntimeError("ATT and position logs do not overlap in time.")
    dt = 1.0 / rate_hz
    t_grid = np.arange(lo, hi, dt)

    # position (linear) & attitude (slerp)
    pos_u_src = ned_to_unity_vec(pos_ned)
    pos_u = np.column_stack([np.interp(t_grid, t_pos, pos_u_src[:, k]) for k in range(3)])
    q_grid = make_quat_continuous(resample_quat(t_att, q_u, t_grid))
    err_grid = np.interp(t_grid, t_att, att_err) if att_err is not None else np.full(len(t_grid), np.nan)

    valid = gap_mask(t_att, t_grid, max_gap) & gap_mask(t_pos, t_grid, max_gap)

    origin = pos_u[np.argmax(valid)].copy() if origin_shift and valid.any() else np.zeros(3)
    pos_u = pos_u - origin

    # arm tip: mount point (FRD, from body centre) + L along body forward (FRD +X), then -> Unity body -> world.
    # mount = (0,0,0) reproduces the thesis kinematics  p_tip = p + R (L,0,0).
    arm_body_u = ned_to_unity_vec(np.asarray(arm_mount_frd, float) + np.array([arm_length, 0.0, 0.0]))
    R_grid = quat_xyzw_to_R(q_grid)
    tip_u = pos_u + R_grid @ arm_body_u

    # segments = continuous runs of valid samples (e.g. armed periods of an ArduPilot log)
    segs = find_segments(valid, max(1, int(round(min_segment_s * rate_hz))))
    seg_id = np.full(len(t_grid), -1, dtype=int)
    for k, (a, b) in enumerate(segs):
        seg_id[a:b] = k

    if skip_gaps:
        if not segs:
            raise RuntimeError("no valid segment left (check --max-gap / --min-segment)")
        keep = np.concatenate([np.arange(a, b) for a, b in segs])
        t_out = np.arange(len(keep)) * dt          # gaps removed: segments are joined back-to-back
    else:
        keep = np.arange(len(t_grid))
        t_out = t_grid - t_grid[0]

    seg_list = []
    for k in range(len(segs)):
        idx = np.where(seg_id[keep] == k)[0]
        seg_list.append({"seg": k,
                         "t_start": float(t_out[idx[0]]), "t_end": float(t_out[idx[-1]]),
                         "t_src_start": float(t_grid[keep][idx[0]]), "t_src_end": float(t_grid[keep][idx[-1]])})

    pwm_out = None
    if motors is not None:
        t_m, pwm = motors
        tg = t_grid[keep]
        pwm_out = np.column_stack([np.interp(tg, t_m, pwm[:, k]) for k in range(pwm.shape[1])])

    return {
        "pwm": pwm_out,
        "t": t_out,
        "t_abs": t_grid[keep],
        "pos": pos_u[keep],
        "quat": q_grid[keep],
        "tip": tip_u[keep],
        "att_err_deg": err_grid[keep],
        "valid": valid.astype(int)[keep],
        "seg": seg_id[keep],
        "segments": seg_list,
        "origin_unity": origin,
    }


def make_meta(source_file: str, position_source: str, a, t0: float) -> dict:
    return {
        "format_version": "1.0",
        "source_file": source_file,
        "position_source": position_source,
        "rate_hz": a.rate,
        "max_gap_s": a.max_gap,
        "arm_length_m": a.arm_length,
        "arm_mount_frd": [float(v) for v in a.arm_mount],
        "t0_boot_s": t0,
        "skip_gaps": bool(getattr(a, "skip_gaps", False)),
        "frame": "unity_left_handed_y_up (x=East, y=Up, z=North)",
        "body_frame": "unity_body (x=Right, y=Up, z=Forward)",
        "mapping": "v_unity = (E, -D, N); R_unity = P R_ned P^T",
    }


def _triple(text: str):
    v = [float(x) for x in text.split(",")]
    if len(v) != 3:
        raise ValueError("expected three comma-separated numbers")
    return tuple(v)


def add_common_args(ap) -> None:
    ap.add_argument("-o", "--out", default=None, help="output base path without extension")
    ap.add_argument("--rate", type=float, default=50.0, help="resample rate [Hz] (default 50)")
    ap.add_argument("--max-gap", type=float, default=0.5, help="source gap [s] above which samples are flagged invalid")
    ap.add_argument("--arm-length", type=float, default=1.5,
                    help="arm length L [m] along body forward (default 1.5; 0 = no arm)")
    ap.add_argument("--arm-mount", type=_triple, default=(0.0, 0.0, 0.0), metavar="F,R,D",
                    help="arm root position from the body centre [m]: forward,right,down "
                         "(e.g. 0,0,-0.08 = 8 cm above the centre; write --arm-mount=-0.1,0,0 if the first value is negative). Default 0,0,0 (on the roll axis)")
    ap.add_argument("--no-origin-shift", action="store_true", help="keep source origin instead of first valid sample")
    ap.add_argument("--skip-gaps", action="store_true",
                    help="remove gaps longer than --max-gap and join the valid segments back-to-back")
    ap.add_argument("--min-segment", type=float, default=1.0,
                    help="segments shorter than this [s] are treated as gaps (default 1.0)")
    ap.add_argument("--t-start", type=float, default=None, help="crop start [s, source time]")
    ap.add_argument("--t-end", type=float, default=None, help="crop end [s, source time]")


def export(res: dict, out_base: str, meta: dict) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(out_base)), exist_ok=True)
    n = len(res["t"])
    frames = []
    for i in range(n):
        e = res["att_err_deg"][i]
        frames.append({
            "t": round(float(res["t"][i]), 4),
            "px": round(float(res["pos"][i, 0]), 4),
            "py": round(float(res["pos"][i, 1]), 4),
            "pz": round(float(res["pos"][i, 2]), 4),
            "qx": round(float(res["quat"][i, 0]), 6),
            "qy": round(float(res["quat"][i, 1]), 6),
            "qz": round(float(res["quat"][i, 2]), 6),
            "qw": round(float(res["quat"][i, 3]), 6),
            "tx": round(float(res["tip"][i, 0]), 4),
            "ty": round(float(res["tip"][i, 1]), 4),
            "tz": round(float(res["tip"][i, 2]), 4),
            "att_err_deg": round(float(e), 3) if np.isfinite(e) else -1.0,  # -1 = not available
            "valid": int(res["valid"][i]),
            "seg": int(res["seg"][i]),                    # segment index, -1 = inside a gap
            "t_src": round(float(res["t_abs"][i]), 4),    # source time [s] (time since boot for .bin)
        })
        if res.get("pwm") is not None:
            frames[-1]["pwm"] = [round(float(v), 1) for v in res["pwm"][i]]   # raw motor output per motor (motor 1..n)
    meta = dict(meta)
    meta.update({"n_frames": n, "duration_s": float(res["t"][-1]) if n else 0.0,
                 "origin_unity": [float(v) for v in res["origin_unity"]],
                 "n_segments": len(res["segments"]), "segments": res["segments"]})
    with open(out_base + ".json", "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "frames": frames}, f, ensure_ascii=False, separators=(",", ":"))
    with open(out_base + ".csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if frames:
            keys = [k for k in frames[0] if k != "pwm"]
            n_m = len(frames[0].get("pwm", []))
            w.writerow(keys + [f"pwm{k + 1}" for k in range(n_m)])
            for fr in frames:
                w.writerow([fr[k] for k in keys] + list(fr.get("pwm", [])))


