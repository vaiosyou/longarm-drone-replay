#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gsi_terrain.py
==============
Build a small terrain (height grid + aerial photo) around a replay from GSI tiles
(Geospatial Information Authority of Japan, 地理院タイル), for FlightReplayEnvironment (mode GSI).

Input : replay JSON made by bin2replay.py / csv2replay.py (needs meta.geo_origin)
Output: <out-dir>/gsi_terrain.json  (grid of heights, Unity axes x=East, z=North, y=Up, relative to the replay origin)
        <out-dir>/gsi_photo.jpg     (aerial photo covering exactly the same rectangle, north up)
        <out-dir>/ATTRIBUTION.txt

Tiles used (XYZ, Web Mercator) [要出典確認: 地理院タイル一覧 https://maps.gsi.go.jp/development/ichiran.html]
  elevation : dem5a_png (z15, 5 m)  -> falls back to dem5b_png (z15) -> dem_png (z14, 10 m)
  photo     : seamlessphoto (jpg, up to z18)
PNG elevation encoding (地理院 標高タイル仕様):
  x = 2^16 R + 2^8 G + B ;  h = x*0.01 (x < 2^23),  NA (x = 2^23),  (x - 2^24)*0.01 (x > 2^23)
Heights are orthometric (標高, above the geoid). They are stored relative to meta.geo_origin[2]
(the log's altitude of the replay origin); use the ground snap / manual offset in Unity to fix any datum gap.

Needs internet access to cyberjapandata.gsi.go.jp. Downloaded tiles are cached in <out-dir>/_tile_cache.
Please follow the GSI terms of use (出典の明示) when publishing images or data made from these tiles.

Usage
-----
  python gsi_terrain.py out/flight_replay.json --out-dir <UnityProject>/Assets/StreamingAssets/gsi
  python gsi_terrain.py out/flight_replay.json --margin 150 --grid 5 --photo-zoom 17
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Callable, Dict, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replaylib import geodetic_to_local_ned, local_ned_to_geodetic  # noqa: E402

BASE = "https://cyberjapandata.gsi.go.jp/xyz"
DEM_LAYERS = [("dem5a_png", 15), ("dem5b_png", 15), ("dem_png", 14)]
PHOTO_LAYER = "seamlessphoto"
ATTRIBUTION = "出典：国土地理院（地理院タイル：標高タイル、全国最新写真（シームレス））を加工して作成"

Fetcher = Callable[[str], Optional[bytes]]


# ---------------------------------------------------------------------------
# tile math (Web Mercator / slippy map)
# ---------------------------------------------------------------------------
def lonlat_to_global_px(lon: float, lat: float, z: int) -> Tuple[float, float]:
    """Global pixel coordinates (256 px tiles) at zoom z."""
    n = 256.0 * (2 ** z)
    x = (lon + 180.0) / 360.0 * n
    s = math.sin(math.radians(lat))
    y = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n
    return x, y


def decode_gsi_dem_png(rgb: np.ndarray) -> np.ndarray:
    """(H,W,3) uint8 -> heights [m], NaN for no-data."""
    r = rgb[..., 0].astype(np.int64)
    g = rgb[..., 1].astype(np.int64)
    b = rgb[..., 2].astype(np.int64)
    x = (r << 16) + (g << 8) + b
    h = np.where(x < 2 ** 23, x, x - 2 ** 24).astype(float) * 0.01
    h[x == 2 ** 23] = np.nan
    return h


# ---------------------------------------------------------------------------
# download with cache
# ---------------------------------------------------------------------------
def make_fetcher(cache_dir: str, sleep_s: float = 0.05) -> Fetcher:
    os.makedirs(cache_dir, exist_ok=True)

    def fetch(url: str) -> Optional[bytes]:
        key = url.replace(BASE + "/", "").replace("/", "_")
        path = os.path.join(cache_dir, key)
        if os.path.exists(path):
            data = open(path, "rb").read()
            return data or None             # empty file = cached 404
        req = urllib.request.Request(url, headers={"User-Agent": "longarm-drone-replay/gsi_terrain"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                data = r.read()
        except urllib.error.HTTPError as e:
            if e.code == 404:                # outside coverage
                open(path, "wb").close()
                return None
            raise
        open(path, "wb").write(data)
        time.sleep(sleep_s)                  # be gentle with the server
        return data
    return fetch


def _open_image(data: bytes) -> np.ndarray:
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(data)).convert("RGB"))


def mosaic(layer: str, z: int, ext: str, px0: float, py0: float, px1: float, py1: float,
           fetch: Fetcher, decode: Callable[[bytes], np.ndarray], fill) -> Tuple[np.ndarray, int, int]:
    """Stitch the tiles covering global pixel box [px0,px1) x [py0,py1). Returns (array, origin_px, origin_py)."""
    tx0, ty0 = int(px0 // 256), int(py0 // 256)
    tx1, ty1 = int(px1 // 256), int(py1 // 256)
    tiles: Dict[Tuple[int, int], np.ndarray] = {}
    found = 0
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            data = fetch(f"{BASE}/{layer}/{z}/{tx}/{ty}.{ext}")
            if data is not None:
                tiles[(tx, ty)] = decode(data)
                found += 1
    if found == 0:
        raise LookupError(f"no {layer} tiles at z{z} for this area")
    sample = next(iter(tiles.values()))
    shape = (256 * (ty1 - ty0 + 1), 256 * (tx1 - tx0 + 1)) + sample.shape[2:]
    out = np.full(shape, fill, dtype=sample.dtype)
    for (tx, ty), a in tiles.items():
        out[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = a
    return out, tx0 * 256, ty0 * 256


def bilinear(img: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Sample a 2D array at fractional pixel coords (pixel centres at i + 0.5); NaN-aware (nearest valid fallback)."""
    x = np.clip(x - 0.5, 0, img.shape[1] - 1.001)
    y = np.clip(y - 0.5, 0, img.shape[0] - 1.001)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    a, b = img[y0, x0], img[y0, x0 + 1]
    c, d = img[y0 + 1, x0], img[y0 + 1, x0 + 1]
    v = (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy
    bad = np.isnan(v)
    if bad.any():
        v[bad] = img[np.rint(y[bad]).astype(int), np.rint(x[bad]).astype(int)]
    return v


def fill_nan(h: np.ndarray) -> np.ndarray:
    """Fill no-data cells (water etc.) with the nearest valid value along rows/cols, then the mean."""
    h = h.copy()
    if np.isnan(h).all():
        raise ValueError("no valid elevation in the area")
    for _ in range(4):
        if not np.isnan(h).any():
            break
        for axis in (0, 1):
            m = np.isnan(h)
            shifted = [np.roll(h, s, axis=axis) for s in (1, -1)]
            for sh in shifted:
                h[m] = np.where(np.isnan(h[m]), sh[m], h[m])
    h[np.isnan(h)] = np.nanmean(h)
    return h


# ---------------------------------------------------------------------------
# main build
# ---------------------------------------------------------------------------
def build(replay_json: str, out_dir: str, margin: float = 100.0, grid: float = 5.0, photo_zoom: int = 17,
          fetch: Optional[Fetcher] = None) -> dict:
    d = json.load(open(replay_json, encoding="utf-8"))
    meta = d["meta"]
    if "geo_origin" not in meta:
        raise SystemExit("meta.geo_origin is missing: re-run bin2replay.py (or csv2replay.py with --geo-origin)")
    lat0, lon0, alt0 = meta["geo_origin"]
    xs = np.array([f["px"] for f in d["frames"]] + [f["tx"] for f in d["frames"]])   # East
    zs = np.array([f["pz"] for f in d["frames"]] + [f["tz"] for f in d["frames"]])   # North
    e_min, e_max = float(xs.min()) - margin, float(xs.max()) + margin
    n_min, n_max = float(zs.min()) - margin, float(zs.max()) + margin
    nx = int(math.ceil((e_max - e_min) / grid)) + 1
    nz = int(math.ceil((n_max - n_min) / grid)) + 1
    e_max, n_max = e_min + (nx - 1) * grid, n_min + (nz - 1) * grid

    os.makedirs(out_dir, exist_ok=True)
    fetch = fetch or make_fetcher(os.path.join(out_dir, "_tile_cache"))

    # geodetic corners of the rectangle
    lat_s, lon_w, _ = local_ned_to_geodetic(lat0, lon0, alt0, n_min, e_min, 0.0)
    lat_n, lon_e, _ = local_ned_to_geodetic(lat0, lon0, alt0, n_max, e_max, 0.0)

    # --- elevation grid
    E, N = np.meshgrid(np.linspace(e_min, e_max, nx), np.linspace(n_min, n_max, nz))   # rows = north index
    lat_g = lat0 + np.degrees(N / 6378137.0)
    lon_g = lon0 + np.degrees(E / (6378137.0 * math.cos(math.radians(lat0))))
    used = None
    for layer, z in DEM_LAYERS:
        px = np.vectorize(lambda lo, la: lonlat_to_global_px(lo, la, z)[0])(lon_g, lat_g)
        py = np.vectorize(lambda lo, la: lonlat_to_global_px(lo, la, z)[1])(lon_g, lat_g)
        try:
            dem, ox, oy = mosaic(layer, z, "png", px.min() - 1, py.min() - 1, px.max() + 1, py.max() + 1,
                                 fetch, lambda b: decode_gsi_dem_png(_open_image(b)), np.nan)
        except LookupError:
            continue
        h = bilinear(dem, px - ox, py - oy)
        if np.isfinite(h).mean() < 0.5:      # mostly no-data (e.g. dem5a not surveyed) -> coarser layer
            continue
        used = (layer, z)
        break
    if used is None:
        raise SystemExit("no GSI elevation tiles found for this area")
    h = fill_nan(h)

    # --- photo, cropped to the same rectangle (north up)
    from PIL import Image
    p0 = lonlat_to_global_px(lon_w, lat_n, photo_zoom)    # north-west corner
    p1 = lonlat_to_global_px(lon_e, lat_s, photo_zoom)    # south-east corner
    photo, ox, oy = mosaic(PHOTO_LAYER, photo_zoom, "jpg", p0[0], p0[1], p1[0], p1[1],
                           fetch, _open_image, 128)
    crop = photo[int(p0[1] - oy):int(math.ceil(p1[1] - oy)), int(p0[0] - ox):int(math.ceil(p1[0] - ox))]
    Image.fromarray(crop).save(os.path.join(out_dir, "gsi_photo.jpg"), quality=90)

    terrain = {
        "format_version": "1.0",
        "nx": nx, "nz": nz, "dx": grid, "dz": grid,
        "x0": e_min, "z0": n_min,                     # Unity x (East) / z (North) of heights[0]
        "heights": [round(float(v - alt0), 3) for v in h.ravel()],   # row-major, rows = North index
        "height_reference": "geo_origin[2] of the replay (log altitude); GSI heights are orthometric",
        "geo_origin": [lat0, lon0, alt0],
        "dem_layer": used[0], "dem_zoom": used[1],
        "photo": "gsi_photo.jpg", "photo_layer": PHOTO_LAYER, "photo_zoom": photo_zoom,
        "attribution": ATTRIBUTION,
    }
    json.dump(terrain, open(os.path.join(out_dir, "gsi_terrain.json"), "w", encoding="utf-8"),
              ensure_ascii=False, separators=(",", ":"))
    open(os.path.join(out_dir, "ATTRIBUTION.txt"), "w", encoding="utf-8").write(
        ATTRIBUTION + "\nhttps://maps.gsi.go.jp/development/ichiran.html\n")
    print(f"[gsi ] {nx}x{nz} grid ({grid} m), area E {e_min:.0f}..{e_max:.0f} m, N {n_min:.0f}..{n_max:.0f} m")
    print(f"[gsi ] elevation {used[0]} z{used[1]}: {np.nanmin(h):.1f}..{np.nanmax(h):.1f} m (origin alt {alt0:.1f} m)")
    print(f"[gsi ] photo {PHOTO_LAYER} z{photo_zoom}: {crop.shape[1]}x{crop.shape[0]} px -> {out_dir}")
    return terrain


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="GSI terrain + photo around a replay (for Unity)")
    ap.add_argument("replay_json")
    ap.add_argument("--out-dir", default="gsi", help="output folder (put it under Assets/StreamingAssets/)")
    ap.add_argument("--margin", type=float, default=100.0, help="extra area around the flight [m]")
    ap.add_argument("--grid", type=float, default=5.0, help="height grid spacing [m]")
    ap.add_argument("--photo-zoom", type=int, default=17, help="photo zoom level (17 ~ 1 m/px, 18 ~ 0.5 m/px)")
    a = ap.parse_args(argv)
    build(a.replay_json, a.out_dir, a.margin, a.grid, a.photo_zoom)
    return 0


if __name__ == "__main__":
    sys.exit(main())
