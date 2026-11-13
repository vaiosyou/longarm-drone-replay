"""Frame-conversion and processing tests. Run: python -m pytest tests/ (or python tests/test_frames.py)"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))
import replaylib as b  # noqa: E402
import csv2replay as c2r  # noqa: E402


def unity_R_from_euler(ex, ey, ez):
    """Unity convention: rotate Z, then X, then Y  ->  R = Ry @ Rx @ Rz (deg)."""
    x, y, z = map(math.radians, (ex, ey, ez))
    Rx = np.array([[1, 0, 0], [0, math.cos(x), -math.sin(x)], [0, math.sin(x), math.cos(x)]])
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    Rz = np.array([[math.cos(z), -math.sin(z), 0], [math.sin(z), math.cos(z), 0], [0, 0, 1]])
    return Ry @ Rx @ Rz


def Rn(eul_deg):
    e = np.radians(np.atleast_2d(eul_deg))
    return b.euler_zyx_to_R(e[:, 0], e[:, 1], e[:, 2])


def R_unity(roll, pitch, yaw):
    R = b.euler_zyx_to_R(np.radians([roll]), np.radians([pitch]), np.radians([yaw]))
    return b.R_ned_to_R_unity(R)[0]


def test_handedness():
    assert np.isclose(np.linalg.det(b.P_NED_TO_UNITY), -1.0)
    assert np.isclose(np.linalg.det(R_unity(10, 20, 30)), 1.0)


def test_position_mapping():
    v = b.ned_to_unity_vec(np.array([1.0, 2.0, -3.0]))  # N=1, E=2, D=-3 (3 m up)
    assert np.allclose(v, [2.0, 3.0, 1.0])


def test_yaw90_points_east():
    fwd = R_unity(0, 0, 90) @ np.array([0, 0, 1.0])
    assert np.allclose(fwd, [1, 0, 0], atol=1e-9)


def test_pitch_up_raises_nose():
    fwd = R_unity(0, 10, 0) @ np.array([0, 0, 1.0])
    assert fwd[1] > 0


def test_roll_right_lowers_right_side():
    right = R_unity(10, 0, 0) @ np.array([1.0, 0, 0])
    assert right[1] < 0


def test_unity_euler_equivalence():
    for r, p, y in [(5, -12, 37), (-20, 15, 250), (0, 0, 0), (30, 40, -100)]:
        assert np.allclose(R_unity(r, p, y), unity_R_from_euler(-p, y, -r), atol=1e-9)


def test_quat_roundtrip():
    R = b.R_ned_to_R_unity(b.euler_zyx_to_R(np.radians([12, -170]), np.radians([-5, 80]), np.radians([300, 10])))
    q = b.R_to_quat_xyzw(R)
    assert np.allclose(b.quat_xyzw_to_R(q), R, atol=1e-9)


def test_tip_matches_thesis_formula():
    """Thesis: p_tip = p + (L cos(psi)cos(theta), L sin(psi)cos(theta), L sin(theta)) in (N, E, Up)."""
    L, th, psi = 1.5, math.radians(12), math.radians(40)
    t = np.array([0.0, 1.0])
    eul = np.radians([[3, 12, 40], [3, 12, 40]])
    pos = np.array([[0, 0, 0], [0, 0, 0]], float)
    res = b.process(t, Rn(np.degrees(eul)), None, t, pos, rate_hz=10, max_gap=2, arm_length=L, origin_shift=False)
    n, e, up = L * math.cos(psi) * math.cos(th), L * math.sin(psi) * math.cos(th), L * math.sin(th)
    # Unity = (E, Up, N). Roll 3 deg moves the tip slightly only if arm off roll axis -> here exactly 0.
    assert np.allclose(res["tip"][0], [e, up, n], atol=1e-9)


def test_resample_and_gap():
    t_att = np.array([0.0, 0.1, 0.2, 2.0, 2.1])
    eul = np.radians(np.column_stack([np.zeros(5), np.zeros(5), [0, 10, 20, 30, 40]]))
    t_pos = np.linspace(0, 2.1, 22)
    pos = np.column_stack([t_pos, np.zeros(22), np.zeros(22)])
    res = b.process(t_att, Rn(np.degrees(eul)), None, t_pos, pos, rate_hz=20, max_gap=0.5, arm_length=1.5)
    assert np.allclose(np.diff(res["t"]), 0.05)
    assert res["valid"][0] == 1
    assert res["valid"][10] == 0          # t=0.5 s is inside the 0.2-2.0 s gap
    # yaw at t=0.05 s should be 5 deg (SLERP midpoint)
    fwd = b.quat_xyzw_to_R(res["quat"][1:2])[0] @ np.array([0, 0, 1.0])
    assert math.isclose(math.degrees(math.atan2(fwd[0], fwd[2])), 5.0, abs_tol=1e-6)


def test_yaw_wrap_slerp():
    """Yaw 359 -> 1 deg must interpolate through 0, not through 180."""
    t = np.array([0.0, 1.0])
    eul = np.radians([[0, 0, 359], [0, 0, 1]])
    pos = np.zeros((2, 3))
    res = b.process(t, Rn(np.degrees(eul)), None, t, pos, rate_hz=2, max_gap=2, arm_length=1.5)
    fwd = b.quat_xyzw_to_R(res["quat"][1:2])[0] @ np.array([0, 0, 1.0])
    yaw = math.degrees(math.atan2(fwd[0], fwd[2]))
    assert abs(yaw) < 1e-6, yaw


def test_att_error():
    t = np.array([0.0, 1.0])
    eul = np.radians([[0, 5, 0], [0, 5, 0]])
    des = np.radians([[0, 0, 0], [0, 0, 0]])
    res = b.process(t, Rn(np.degrees(eul)), Rn(np.degrees(des)), t, np.zeros((2, 3)), rate_hz=2, max_gap=2, arm_length=1.5)
    assert np.allclose(res["att_err_deg"], 5.0)


def test_clean_series():
    t = np.array([0.0, 0.2, 0.1, 0.1, np.nan, 0.3])
    x = np.array([0, 2, 1, 9, 5, np.nan])
    tc, (xc,) = b.clean_series(t, x)
    assert np.allclose(tc, [0.0, 0.1, 0.2]) and np.allclose(xc, [0, 1, 2])


def _write(path, header, rows):
    with open(path, "w") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(",".join("" if v is None else repr(float(v)) for v in r) + "\n")


def test_csv_frames_agree(tmp_path=None):
    """The same flight written as NED/Euler, ROS ENU/FLU quaternion, and Unity quaternion must give identical output."""
    import tempfile
    d = tempfile.mkdtemp() if tmp_path is None else str(tmp_path)
    t = np.linspace(0, 4, 41)
    roll, pitch, yaw = 5 * np.sin(t), 10 * np.cos(t), 30 + 40 * t          # deg, NED/FRD
    N, E, D = 2 * t, np.sin(t), -1 - 0.5 * t
    R = Rn(np.column_stack([roll, pitch, yaw]))

    # 1) NED + Euler deg, time in ms, one missing cell
    rows = [[ti * 1000, n, e, dd, r, p, y] for ti, n, e, dd, r, p, y in zip(t, N, E, D, roll, pitch, yaw)]
    rows[7][3] = None
    _write(f"{d}/ned.csv", ["ms", "n", "e", "d", "r", "p", "y"], rows)
    c2r.main([f"{d}/ned.csv", "--time-col", "ms", "--time-unit", "ms", "--pos-cols", "n,e,d", "--pos-frame", "ned",
              "--euler-cols", "r,p,y", "-o", f"{d}/o1", "--rate", "10"])

    # 2) ROS: ENU position, FLU->ENU quaternion xyzw
    A, B = b.A_ENU_TO_NED, b.B_FLU_FRD
    R_enu = np.einsum("ij,njk,kl->nil", A.T, R, B)          # inverse of R_ned = A R_enu B
    q = b.R_to_quat_xyzw(R_enu)
    rows = [[ti, e, n, -dd, *qq] for ti, n, e, dd, qq in zip(t, N, E, D, q)]
    _write(f"{d}/ros.csv", ["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"], rows)
    c2r.main([f"{d}/ros.csv", "--time-col", "stamp", "--pos-cols", "x,y,z", "--pos-frame", "enu",
              "--quat-cols", "qx,qy,qz,qw", "--quat-order", "xyzw", "--att-frame", "enu_flu", "-o", f"{d}/o2", "--rate", "10"])

    # 3) Unity quaternion wxyz
    qu = b.R_to_quat_xyzw(b.R_ned_to_R_unity(R))
    rows = [[ti, e, -dd, n, qq[3], qq[0], qq[1], qq[2]] for ti, n, e, dd, qq in zip(t, N, E, D, qu)]
    _write(f"{d}/u.csv", ["t", "x", "y", "z", "w", "i", "j", "k"], rows)
    c2r.main([f"{d}/u.csv", "--time-col", "t", "--pos-cols", "x,y,z", "--pos-frame", "unity",
              "--quat-cols", "w,i,j,k", "--att-frame", "unity", "-o", f"{d}/o3", "--rate", "10"])

    import json
    outs = [json.load(open(f"{d}/o{i}.json"))["frames"] for i in (1, 2, 3)]
    keys = ["px", "py", "pz", "tx", "ty", "tz"]
    for i in (1, 2):
        assert len(outs[i]) == len(outs[0])
        for fa, fb in zip(outs[0], outs[i]):
            if 0.6 < fa["t"] < 0.8:   # row 7 of ned.csv has a blank cell -> linearly bridged there
                continue
            assert all(abs(fa[k] - fb[k]) < 2e-3 for k in keys), (i, fa, fb)
            qa = np.array([fa[k] for k in ("qx", "qy", "qz", "qw")])
            qb = np.array([fb[k] for k in ("qx", "qy", "qz", "qw")])
            assert abs(abs(qa @ qb) - 1) < 1e-5


    f7 = next(f for f in outs[0] if abs(f["t"] - 0.7) < 1e-6)
    assert f7["valid"] == 1 and abs(f7["px"] - (np.sin(0.6) + np.sin(0.8)) / 2) < 2e-3


def test_ros_yaw_zero_points_east():
    """ROS yaw 0 = facing East -> Unity nose +X."""
    R = b.R_enu_flu_to_R_ned_frd(Rn([0, 0, 0]))
    fwd = b.R_ned_to_R_unity(R)[0] @ np.array([0, 0, 1.0])
    assert np.allclose(fwd, [1, 0, 0], atol=1e-9)


def test_skip_gaps():
    """Two 2-s segments separated by a 10-s gap -> joined, uniform time, segment ids 0/1, source time kept."""
    t_att = np.concatenate([np.arange(0, 2.01, 0.1), np.arange(12, 14.01, 0.1)])
    R = Rn(np.zeros((len(t_att), 3)))
    pos = np.column_stack([t_att, np.zeros_like(t_att), np.zeros_like(t_att)])
    full = b.process(t_att, R, None, t_att, pos, rate_hz=10, max_gap=0.5, arm_length=1.5)
    res = b.process(t_att, R, None, t_att, pos, rate_hz=10, max_gap=0.5, arm_length=1.5, skip_gaps=True)
    assert len(full["segments"]) == 2 and full["seg"][50] == -1
    assert np.all(res["valid"] == 1) and set(res["seg"]) == {0, 1}
    assert np.allclose(np.diff(res["t"]), 0.1)
    assert res["t"][-1] < 4.5                       # gap of ~10 s removed
    s1 = res["segments"][1]
    assert abs(s1["t_src_start"] - 12.0) < 0.11 and abs(res["pos"][res["seg"] == 1][0, 2] - (12.0 - 0.0)) < 0.11


def test_arm_mount_offset():
    """Arm root 0.1 m above the centre (FRD down = -0.1): level -> tip 0.1 m higher; pitch-up 30 deg -> rotated with the body."""
    L, h = 1.5, 0.1
    t = np.array([0.0, 1.0])
    lvl = b.process(t, Rn([[0, 0, 0], [0, 0, 0]]), None, t, np.zeros((2, 3)), rate_hz=2, max_gap=2,
                    arm_length=L, origin_shift=False, arm_mount_frd=(0, 0, -h))
    assert np.allclose(lvl["tip"][0], [0, h, L], atol=1e-9)          # Unity (E, Up, N)
    th = math.radians(30)
    pit = b.process(t, Rn([[0, 30, 0], [0, 30, 0]]), None, t, np.zeros((2, 3)), rate_hz=2, max_gap=2,
                    arm_length=L, origin_shift=False, arm_mount_frd=(0, 0, -h))
    north = L * math.cos(th) - h * math.sin(th)
    up = L * math.sin(th) + h * math.cos(th)
    assert np.allclose(pit["tip"][0], [0, up, north], atol=1e-9)


def test_motor_resample_and_skip():
    t = np.concatenate([np.arange(0, 2.01, 0.1), np.arange(12, 14.01, 0.1)])
    R = Rn(np.zeros((len(t), 3)))
    pos = np.zeros((len(t), 3))
    pwm = np.column_stack([1000 + 100 * t, 1500 + 0 * t])
    res = b.process(t, R, None, t, pos, rate_hz=20, max_gap=0.5, arm_length=1.5, skip_gaps=True, motors=(t, pwm))
    assert res["pwm"].shape == (len(res["t"]), 2)
    assert np.allclose(res["pwm"][:, 0], 1000 + 100 * res["t_abs"])      # linear, aligned with source time
    assert np.allclose(res["pwm"][:, 1], 1500)


def test_unity_telemetry_inverse():
    """Formulas used in FlightReplayTelemetry.cs recover roll/pitch/yaw from the Unity rotation."""
    for r, p, y in [(10, 20, 30), (-35, -15, 250), (5, 60, 359), (-80, 5, 90)]:
        Ru = R_unity(r, p, y)
        f, rt, u = Ru @ [0, 0, 1.0], Ru @ [1.0, 0, 0], Ru @ [0, 1.0, 0]
        yaw = math.degrees(math.atan2(f[0], f[2])) % 360
        pitch = math.degrees(math.asin(f[1]))
        roll = math.degrees(math.atan2(-rt[1], u[1]))
        assert np.allclose([roll, pitch, yaw], [r, p, y % 360], atol=1e-6), (r, p, y, roll, pitch, yaw)


def test_geo_origin_roundtrip():
    """Point 30 m north / 40 m east / 5 m up of the reference, expressed back and forth."""
    lat0, lon0, alt0 = 43.77, 142.36, 120.0
    # replay origin at ref + (E=40, Up=5, N=30)  ->  ref seen from the origin is (-40, -5, -30)
    o = b.geo_of_replay_origin(lat0, lon0, alt0, (-40.0, -5.0, -30.0))
    ned = b.geodetic_to_local_ned(np.array([lat0, o[0]]), np.array([lon0, o[1]]), np.array([alt0, o[2]]))
    assert np.allclose(ned[1], [30.0, 40.0, -5.0], atol=1e-3)


def test_csv_geo_origin():
    import json, tempfile
    d = tempfile.mkdtemp()
    t = np.arange(0, 3.01, 0.1)
    lat = 43.77 + 1e-5 * t; lon = 142.36 + 0 * t; alt = 120 + t
    _write(f"{d}/g.csv", ["t", "lat", "lon", "alt", "r", "p", "y"], [[a, b_, c, e, 0, 0, 0] for a, b_, c, e in zip(t, lat, lon, alt)])
    c2r.main([f"{d}/g.csv", "--time-col", "t", "--pos-cols", "lat,lon,alt", "--pos-frame", "geo",
              "--euler-cols", "r,p,y", "-o", f"{d}/go", "--rate", "10"])
    m = json.load(open(f"{d}/go.json"))["meta"]
    assert np.allclose(m["geo_origin"], [43.77, 142.36, 120.0], atol=1e-7)   # first sample = replay origin


def test_gsi_terrain_offline():
    """gsi_terrain.build with a fake tile server: elevation = 100 m + 0.01 m * global pixel x (z15), 128,0,0 hole."""
    import io, json, re, tempfile
    from PIL import Image
    import gsi_terrain as g
    assert np.allclose(g.decode_gsi_dem_png(np.array([[[0, 39, 16], [128, 0, 0], [255, 255, 156]]], np.uint8)),
                       [[100.0, np.nan, -1.0]], equal_nan=True)

    def enc(h):
        x = np.rint(h / 0.01).astype(np.int64) & 0xFFFFFF
        return np.stack([(x >> 16) & 255, (x >> 8) & 255, x & 255], -1).astype(np.uint8)

    def fetch(url):
        m = re.search(r"/(\w+)/(\d+)/(\d+)/(\d+)\.(png|jpg)$", url)
        layer, z, tx, ty = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
        if layer == "dem5a_png":
            gx = tx * 256 + np.arange(256)[None, :] + 0.5 + 0 * np.arange(256)[:, None]
            img = enc(100.0 + 0.01 * (gx - 29179 * 256))
            img[0, 0] = (128, 0, 0)
            bio = io.BytesIO(); Image.fromarray(img).save(bio, "PNG"); return bio.getvalue()
        if layer == "seamlessphoto":
            bio = io.BytesIO(); Image.fromarray(np.full((256, 256, 3), 90, np.uint8)).save(bio, "JPEG"); return bio.getvalue()
        return None

    d = tempfile.mkdtemp()
    t = np.array([0.0, 1.0]); R = Rn(np.zeros((2, 3)))
    res = b.process(t, R, None, t, np.array([[0, 0, 0], [20.0, 30.0, -5]]), rate_hz=2, max_gap=2, arm_length=1.5)
    meta = {"geo_origin": [43.81, 142.355, 150.0]}
    b.export(res, f"{d}/r", meta)
    ter = g.build(f"{d}/r.json", f"{d}/gsi", margin=20, grid=5, photo_zoom=15, fetch=fetch)
    H = np.array(ter["heights"]).reshape(ter["nz"], ter["nx"])
    assert np.isfinite(H).all() and ter["dem_layer"] == "dem5a_png"
    # elevation increases eastwards by 0.01 m per z15 pixel (~3.4 m at this latitude -> ~0.0147 m per 5 m)
    px_per_m = 256 * 2 ** 15 / (2 * math.pi * 6378137 * math.cos(math.radians(43.81)))
    assert np.allclose(np.diff(H, axis=1).mean(), 0.01 * px_per_m * 5 / math.cos(math.radians(43.81)) * math.cos(math.radians(43.81)), rtol=0.05)
    assert abs(H[ter["nz"] // 2, 0] - (100.0 + 0.01 * (g.lonlat_to_global_px(142.355 + math.degrees(ter["x0"] / (6378137 * math.cos(math.radians(43.81)))), 43.81, 15)[0] - 29179 * 256) - 150.0)) < 0.1
    assert os.path.exists(f"{d}/gsi/gsi_photo.jpg")


def test_no_arm():
    t = np.array([0.0, 1.0])
    res = b.process(t, Rn([[0, 20, 50], [0, 20, 50]]), None, t, np.zeros((2, 3)), rate_hz=2, max_gap=2, arm_length=0)
    assert np.allclose(res["tip"], res["pos"])


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f()
            print("ok", k)
