#!/usr/bin/env python3
"""Stage 1: OpenAerialMap maps + Copernicus DEM for the Tuniu flight, and per-photo map coverage.

  .venv/bin/python experiments/x2_tuniu_map.py fetch      # OAM query, every usable scene at 1 m, DEM
  .venv/bin/python experiments/x2_tuniu_map.py coverage   # % of each photo footprint covered; picks main/y2021
  .venv/bin/python experiments/x2_tuniu_map.py resample   # chosen maps at 0.1/0.25/0.5/1.0 m/px (EPSG:3826)
  .venv/bin/python experiments/x2_tuniu_map.py calibrate  # pre-cut boresight + map georeferencing offsets

Only COG windows are read (GDAL /vsicurl range requests, overview-aware decimated reads).
The 2019-04-11 OAM orthophotos are excluded (same day as the photos; the 5.7K one is likely built from
these very photos). Map roles:
  main   2019-12-12 '5.7K' orthophoto
  y2021  the 2021 orthophoto with the best footprint coverage (harder: two years later, works on banks)
  negs   2019-07-09 'miaoli_sanwan' scene (~700 m south) used only for negative windows
Coverage uses RTK positions + DJI attitude (evaluator-side report, not an estimator input).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import urllib.request

import numpy as np
import pandas as pd

os.environ.setdefault("GDAL_DISABLE_READDIR_ON_OPEN", "EMPTY_DIR")
os.environ.setdefault("GDAL_HTTP_MULTIPLEX", "YES")
os.environ.setdefault("GDAL_HTTP_MULTIRANGE", "YES")
os.environ.setdefault("VSI_CACHE", "TRUE")

import rasterio  # noqa: E402
from rasterio.enums import Resampling  # noqa: E402
from rasterio.transform import from_origin  # noqa: E402
from rasterio.warp import reproject  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
import x_tuniu_geo as G  # noqa: E402

FLIGHT_BBOX = (120.9485, 24.6797, 120.9526, 24.6822)   # lon/lat
MARGIN_M = 300.0
API = "https://api.openaerialmap.org/meta"
UA = {"User-Agent": "TaipeiDrift-research/1.0 (civil navigation research)"}
EXCLUDE_DATES = {"2019-04-11"}
NEG_SCENE_ID = "5e903766c6bbac0005e30fab"     # 20190709_miaoli_sanwan, OAM upload 2020-04-09


def query_bbox():
    lat0 = (FLIGHT_BBOX[1] + FLIGHT_BBOX[3]) / 2
    dlon = MARGIN_M / (111320 * math.cos(math.radians(lat0)))
    dlat = MARGIN_M / 110570
    return (FLIGHT_BBOX[0] - dlon, FLIGHT_BBOX[1] - dlat, FLIGHT_BBOX[2] + dlon, FLIGHT_BBOX[3] + dlat)


def grid():
    """Target EPSG:3826 extent: flight bbox + 300 m margin, snapped to 1 m."""
    b = query_bbox()
    xs, ys = G.TO_3826.transform([b[0], b[2], b[0], b[2]], [b[1], b[1], b[3], b[3]])
    return math.floor(min(xs)), math.floor(min(ys)), math.ceil(max(xs)), math.ceil(max(ys))


def oam_query() -> list[dict]:
    b = query_bbox()
    url = f"{API}?bbox={b[0]:.6f},{b[1]:.6f},{b[2]:.6f},{b[3]:.6f}&limit=200"
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=60) as r:
        d = json.loads(r.read())
    return d["results"]


def scene_record(rec):
    return {"id": rec["_id"], "date": (rec.get("acquisition_start") or "")[:10], "gsd_m": rec.get("gsd"),
            "title": rec.get("title"), "provider": rec.get("provider"),
            "licence": (rec.get("properties") or {}).get("license") or rec.get("license"),
            "bbox": rec.get("bbox"), "url": rec["uuid"]}


def read_cog_to_grid(url: str, res: float, extent, tile_px: int = 1024, threads: int = 8) -> np.ndarray:
    """RGBA uint8 on the EPSG:3826 grid at `res` (alpha = valid). The grid is split into tiles read
    concurrently (GDAL releases the GIL); the white collar is removed on the mosaic."""
    from concurrent.futures import ThreadPoolExecutor
    x0, y0, x1, y1 = extent
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    out = np.zeros((4, H, W), np.uint8)
    jobs = []
    for r in range(0, H, tile_px):
        for c in range(0, W, tile_px):
            w, h = min(tile_px, W - c), min(tile_px, H - r)
            ext = (x0 + c * res, y1 - (r + h) * res, x0 + (c + w) * res, y1 - r * res)
            jobs.append((r, c, ext))
    with ThreadPoolExecutor(threads) as ex:
        for (r, c, _), arr in zip(jobs, ex.map(lambda j: _read_tile(url, res, j[2]), jobs)):
            out[:, r:r + arr.shape[1], c:c + arr.shape[2]] = arr
    out[3] = np.where(white_collar(out[:3], out[3] > 0), 0, out[3])
    return out


def _read_tile(url: str, res: float, extent) -> np.ndarray:
    """Decimated in-source window read (honours COG overviews) then in-memory reprojection."""
    x0, y0, x1, y1 = extent
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    dst_t = from_origin(x0, y1, res, res)
    out = np.zeros((4, H, W), np.uint8)
    with rasterio.open("/vsicurl/" + url) as src:
        from pyproj import Transformer
        tr = Transformer.from_crs("EPSG:3826", src.crs, always_xy=True)
        xs, ys = tr.transform([x0, x1, x0, x1], [y1, y1, y0, y0])
        if src.crs.is_geographic:
            mpp = abs(src.transform.a) * 111320.0 * math.cos(math.radians(np.mean(ys)))
        else:
            mpp = abs(src.transform.a)
        win = rasterio.windows.from_bounds(min(xs), min(ys), max(xs), max(ys), src.transform)
        c0, r0 = max(0, int(math.floor(win.col_off))), max(0, int(math.floor(win.row_off)))
        c1 = min(src.width, int(math.ceil(win.col_off + win.width)))
        r1 = min(src.height, int(math.ceil(win.row_off + win.height)))
        if c1 - c0 < 2 or r1 - r0 < 2:
            return out
        sub = rasterio.windows.Window(c0, r0, c1 - c0, r1 - r0)
        dec = max(1.0, (res / 2.0) / mpp)          # source sampled at <= res/2 before reprojection
        ow, oh = max(1, int(round(sub.width / dec))), max(1, int(round(sub.height / dec)))
        arr = src.read([1, 2, 3], out_shape=(3, oh, ow), window=sub, resampling=Resampling.average)
        mask = src.read_masks(1, out_shape=(oh, ow), window=sub, resampling=Resampling.nearest)
        tr0 = src.window_transform(sub)
        stf = rasterio.Affine(tr0.a * sub.width / ow, tr0.b, tr0.c, tr0.d, tr0.e * sub.height / oh, tr0.f)
        for i in range(3):
            reproject(arr[i], out[i], src_transform=stf, src_crs=src.crs, dst_transform=dst_t,
                      dst_crs="EPSG:3826", resampling=Resampling.average)
        m = np.zeros((H, W), np.uint8)
        reproject((mask > 0).astype(np.uint8) * 255, m, src_transform=stf, src_crs=src.crs, dst_transform=dst_t,
                  dst_crs="EPSG:3826", resampling=Resampling.min, src_nodata=None, dst_nodata=0)
        out[3] = np.where(out[:3].max(0) > 0, m, 0)
    return out


def white_collar(rgb: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """The OAM uploads carry a near-white collar inside the dataset mask (photogrammetry background).
    Near-white pixels connected to the no-data area are no-data; isolated white roofs are kept."""
    import cv2
    near_white = (rgb.min(0) >= 225) & ((rgb.max(0).astype(int) - rgb.min(0)) <= 12)
    cand = (near_white | ~valid).astype(np.uint8)
    n, lab = cv2.connectedComponents(cand, connectivity=8)
    bad = np.zeros(n, bool)
    bad[np.unique(lab[~valid])] = True
    return bad[lab] & near_white


def write_rgba(path, arr, res, extent):
    path.parent.mkdir(parents=True, exist_ok=True)
    x0, _, _, y1 = extent
    with rasterio.open(path, "w", driver="GTiff", width=arr.shape[2], height=arr.shape[1], count=4,
                       dtype="uint8", crs="EPSG:3826", transform=from_origin(x0, y1, res, res),
                       compress="deflate", tiled=True, photometric="RGB", alpha="YES") as ds:
        ds.write(arr)


def fetch_dem(extent):
    if G.DEM_TIF.exists():
        return
    x0, y0, x1, y1 = extent
    lon, lat = G.TO_LONLAT.transform([x0, x1, x0, x1], [y0, y0, y1, y1])
    w, e, s, n = min(lon) - 0.01, max(lon) + 0.01, min(lat) - 0.01, max(lat) + 0.01
    tile = f"Copernicus_DSM_COG_10_N{int(math.floor(s)):02d}_00_E{int(math.floor(w)):03d}_00_DEM"
    with rasterio.open(f"/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif") as ds:
        win = rasterio.windows.from_bounds(w, s, e, n, ds.transform)
        data = ds.read(1, window=win)
        t = ds.window_transform(win)
        prof = dict(driver="GTiff", width=data.shape[1], height=data.shape[0], count=1, dtype=data.dtype,
                    crs=ds.crs, transform=t)
    G.DEM_TIF.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(G.DEM_TIF, "w", **prof) as o:
        o.write(data, 1)
    print(f"DEM {tile}: {data.shape} px, {data.min():.1f}-{data.max():.1f} m (EGM2008)")


def cmd_fetch(args):
    recs = [scene_record(r) for r in oam_query()]
    extent = grid()
    print(f"OAM query bbox (flight + {MARGIN_M:.0f} m): {query_bbox()}; {len(recs)} scenes")
    for r in sorted(recs, key=lambda r: r["date"]):
        tag = "EXCLUDED (same day as photos)" if r["date"] in EXCLUDE_DATES else ""
        print(f"  {r['date']} {r['id']} gsd {r['gsd_m']:.3f} m  {r['title'][:48]}  {r['licence']} {tag}")
    neg = scene_record(json.loads(urllib.request.urlopen(urllib.request.Request(
        f"{API}/{NEG_SCENE_ID}", headers=UA), timeout=60).read())["results"])
    usable = [r for r in recs if r["date"] not in EXCLUDE_DATES]
    G.XOUT.mkdir(parents=True, exist_ok=True)
    (G.XOUT / "stage1_oam_scenes.json").write_text(json.dumps(
        {"query_bbox": query_bbox(), "grid_3826": extent, "scenes": recs, "negative_scene": neg,
         "excluded_dates": sorted(EXCLUDE_DATES)}, indent=1, ensure_ascii=False))
    fetch_dem(extent)
    # every usable scene at 1 m first (coverage decides the 2021 map), then the chosen maps at all res
    for r in usable + [neg]:
        p = G.MAPDIR / f"scene_{r['id']}_1m.tif"
        if not p.exists():
            ext = extent
            if r["id"] == NEG_SCENE_ID:
                bx = r["bbox"]
                xs, ys = G.TO_3826.transform([bx[0], bx[2]], [bx[1], bx[3]])
                ext = (math.floor(xs[0]) - 20, math.floor(ys[0]) - 20, math.ceil(xs[1]) + 20, math.ceil(ys[1]) + 20)
            write_rgba(p, read_cog_to_grid(r["url"], 1.0, ext), 1.0, ext)
            print(f"  wrote {p.name}")


def footprints(far_m=100.0):
    """Per photo: footprint polygon (EPSG:3826) from RTK pose + DJI attitude, flat ground at DEM."""
    cam = G.Camera()
    ph, tr = G.photos(), G.truth_xy()
    out = []
    for a, t in zip(ph.itertuples(), tr.itertuples()):
        R = G.rot_enu_cam(a.gimbal_yaw_deg, a.gimbal_pitch_deg, a.gimbal_roll_deg)
        h = t.alt_ell - float(G.dem_ellipsoidal(t.x, t.y))
        poly = G.footprint_offsets(cam, R, h, far_m) + [t.x, t.y]
        out.append(poly)
    return out


def coverage_table(paths: dict, polys) -> pd.DataFrame:
    from rasterio.features import rasterize
    from shapely.geometry import Polygon
    rows = []
    for key, path in paths.items():
        with rasterio.open(path) as ds:
            valid = ds.read(4) > 0
            t = ds.transform
            for i, poly in enumerate(polys):
                pg = Polygon(poly)
                m = rasterize([(pg, 1)], out_shape=valid.shape, transform=t, fill=0, dtype="uint8").astype(bool)
                rows.append(dict(map=key, frame=i + 1, footprint_m2=pg.area,
                                 covered_pct=100.0 * (valid & m).sum() / max(1, pg.area / t.a ** 2)))
    return pd.DataFrame(rows)


def cmd_coverage(args):
    info = json.loads((G.XOUT / "stage1_oam_scenes.json").read_text())
    usable = [r for r in info["scenes"] if r["date"] not in EXCLUDE_DATES]
    polys = footprints()
    paths = {f"{r['date']}_{r['id'][-6:]}": G.MAPDIR / f"scene_{r['id']}_1m.tif" for r in usable}
    cov = coverage_table(paths, polys)
    prot = G.protocol()
    test = set(prot["test_frames"])
    cov["test"] = cov.frame.isin(test)
    summ = cov.groupby("map").agg(mean_cov_all=("covered_pct", "mean"),
                                  photos_ge90=("covered_pct", lambda s: int((s >= 90).sum())),
                                  photos_ge50=("covered_pct", lambda s: int((s >= 50).sum())))
    summ["mean_cov_test"] = cov[cov.test].groupby("map").covered_pct.mean()
    print("footprint = image area up to 100 m ahead of the nadir, flat ground at the DEM under the RTK nadir")
    print(f"photos: {len(polys)} (test {len(test)}); median footprint area {np.median([cov.footprint_m2.iloc[i] for i in range(len(polys))]):.0f} m2")
    print(summ.round(1).to_string())
    cov.to_csv(G.XOUT / "stage1_coverage_per_photo.csv", index=False)
    y2021 = summ[summ.index.str.startswith("2021")].mean_cov_test.idxmax()
    main_key = [k for k in summ.index if k.startswith("2019-12-12")][0]
    choice = {"main": {"key": main_key, "id": [r["id"] for r in usable if f"{r['date']}_{r['id'][-6:]}" == main_key][0]},
              "y2021": {"key": y2021, "id": [r["id"] for r in usable if f"{r['date']}_{r['id'][-6:]}" == y2021][0]},
              "negs": {"key": "2019-07-09_sanwan", "id": NEG_SCENE_ID}}
    (G.XOUT / "stage1_maps.json").write_text(json.dumps(choice, indent=1))
    print("chosen:", json.dumps(choice))


def cmd_resample(args):
    info = json.loads((G.XOUT / "stage1_oam_scenes.json").read_text())
    choice = json.loads((G.XOUT / "stage1_maps.json").read_text())
    byid = {r["id"]: r for r in info["scenes"] + [info["negative_scene"]]}
    extent = tuple(info["grid_3826"])
    for role, c in choice.items():
        r = byid[c["id"]]
        ext = extent
        if role == "negs":
            with rasterio.open(G.MAPDIR / f"scene_{r['id']}_1m.tif") as ds:
                b = ds.bounds
                ext = (b.left, b.bottom, b.right, b.top)
        for res in G.RESOLUTIONS:
            p = G.map_path(role, res)
            if p.exists():
                continue
            arr = read_cog_to_grid(r["url"], res, ext)
            write_rgba(p, arr, res, ext)
            print(f"  {role} {r['date']} {res:g} m/px: {arr.shape[2]}x{arr.shape[1]} px, "
                  f"valid {100 * (arr[3] > 0).mean():.1f} %")


def _precut_matches(maps: dict, res: float, bs: dict, cam, imgs, search_m=30.0, angles=(0.0,), scales=(1.0,),
                    lifted=False):
    """ZNCC of every pre-cut photo with the RTK pose (allowed before the cut): offsets fix - RTK."""
    import x_tuniu_match as M
    ph, tr = G.photos(), G.truth_xy()
    rows = []
    for f in G.protocol()["precut_frames"]:
        a, t = ph.iloc[f - 1], tr.iloc[f - 1]
        R = G.rot_enu_cam(a.gimbal_yaw_deg + bs["yaw"], a.gimbal_pitch_deg + bs["pitch"], a.gimbal_roll_deg + bs["roll"])
        z0 = float(G.dem_ellipsoidal(t.x, t.y))
        h = t.alt_ell - z0
        ground = (lambda X, Y, t=t, z0=z0: G.dem_ellipsoidal(t.x + X, t.y + Y) - z0) if lifted else None
        q = G.rectify(imgs[f][0], imgs[f][1], cam, R, h, res, ground=ground)
        for key, m in maps.items():
            ref = M.reference_for(m, q, (t.x, t.y), search_m)
            r = M.zncc_fix(q, ref, angles, scales)
            d = (r["fix"] - [t.x, t.y]) if r["fix"] is not None else np.array([np.nan, np.nan])
            rows.append(dict(map=key, frame=f, leg=G.protocol()["leg_per_frame"][f - 1], score=r["score"],
                             quad_n=r["quad_n"], de=d[0], dn=d[1]))
    return pd.DataFrame(rows)


def cmd_calibrate(args):
    """Pre-cut only: camera boresight (pitch/yaw/roll offsets of the DJI gimbal angles) maximising the
    median ZNCC peak on the main map at 0.5 m/px (coordinate descent, 0.5 deg grid), then each map's
    georeferencing offset (median of fix - RTK over pre-cut photos whose quad >= 3) at 0.25 m/px.
    Also checks the DJI distortion model and the leg-to-leg consistency (flat plane vs DEM-lifted)."""
    cam = G.Camera()
    ph = G.photos()
    pre = G.protocol()["precut_frames"]
    maps = {k: G.MapRaster(G.map_path(k, 0.5)) for k in ("main",)}
    f05 = G.reduce_factor(0.5)
    imgs = {f: (G.load_gray(ph.path.iloc[f - 1], f05), f05) for f in pre}
    bs = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}

    def objective(b):
        d = _precut_matches(maps, 0.5, b, cam, imgs)
        return float(d.score.median()), d

    best, _ = objective(bs)
    print(f"boresight search on {len(pre)} pre-cut photos, main map 0.5 m/px; start median ZNCC {best:.3f}")
    grids = {"pitch": np.arange(-3, 3.01, 0.5), "yaw": np.arange(-3, 3.01, 0.5), "roll": np.arange(-2, 2.01, 0.5)}
    for it in range(2):
        for axis, grid_ in grids.items():
            scores = {}
            for v in grid_:
                b = dict(bs, **{axis: float(v)})
                scores[float(v)] = objective(b)[0]
            v = max(scores, key=scores.get)
            bs[axis] = v
            best = scores[v]
            print(f"  iter {it} {axis}: best {v:+.1f} deg, median ZNCC {best:.3f} "
                  f"(at 0: {scores.get(0.0, np.nan):.3f})")
    cam_nodist = G.Camera()
    cam_nodist.dist = np.zeros(5)
    s_nodist = float(_precut_matches(maps, 0.5, bs, cam_nodist, imgs).score.median())
    print(f"distortion check: median ZNCC with DJI DewarpData {best:.3f}, without distortion {s_nodist:.3f}")

    f025 = G.reduce_factor(0.25)
    imgs = {f: (G.load_gray(ph.path.iloc[f - 1], f025), f025) for f in pre}
    hyp = dict(angles=(-2.0, 0.0, 2.0), scales=(0.97, 1.0, 1.03))
    # Diagnostic (not used to choose the boresight): a pitch/roll boresight error flips sign between the
    # opposite legs 0 and 1, a map offset does not. But on this hillside a sloped ground under a flat-plane
    # rectification also flips sign with heading, so the leg gap mixes boresight and relief. Compare the gap
    # with flat-plane and DEM-lifted rectification at the score-based boresight.
    main025 = {"main": G.MapRaster(G.map_path("main", 0.25))}

    def leg_gap(b, lifted):
        d = _precut_matches(main025, 0.25, b, cam, imgs, lifted=lifted, **hyp)
        ok = d[(d.quad_n >= 3) & d.leg.isin([0, 1])]
        n = ok.groupby("leg").size()
        if len(n) < 2 or n.min() < 3:
            return np.nan, n.to_dict(), float(d.score.median())
        med = ok.groupby("leg")[["de", "dn"]].median()
        return float(np.hypot(*(med.loc[0] - med.loc[1]))), n.to_dict(), float(d.score.median())

    diag = {}
    for lifted in (False, True):
        gap, nleg, med = leg_gap(bs, lifted)
        diag["dem_lifted" if lifted else "flat_plane"] = {"leg_gap_m": gap, "quad3_per_leg": nleg, "median_zncc": med}
        print(f"leg0-leg1 gap of median offsets ({'DEM-lifted' if lifted else 'flat plane'}): {gap:.2f} m, "
              f"quad>=3 photos per leg {nleg}, median ZNCC {med:.3f}")
    maps = {k: G.MapRaster(G.map_path(k, 0.25)) for k in ("main", "y2021")}
    d = _precut_matches(maps, 0.25, bs, cam, imgs, **hyp)
    d.to_csv(G.XOUT / "stage1_georef_precut.csv", index=False)
    offsets = {}
    for key, g in d.groupby("map"):
        ok = g[g.quad_n >= 3]
        per_leg = ok.groupby("leg")[["de", "dn"]].median()
        offsets[key] = {"de_m": float(ok.de.median()), "dn_m": float(ok.dn.median()), "n_used": int(len(ok)),
                        "n_precut": int(len(g)), "mad_e_m": float((ok.de - ok.de.median()).abs().median()),
                        "mad_n_m": float((ok.dn - ok.dn.median()).abs().median()),
                        "per_leg": {int(k): {"de_m": float(v.de), "dn_m": float(v.dn)} for k, v in per_leg.iterrows()}}
        print(f"map {key}: georef offset (map - RTK) dE {offsets[key]['de_m']:+.2f} m, dN {offsets[key]['dn_m']:+.2f} m "
              f"from {len(ok)}/{len(g)} pre-cut photos with quad>=3; MAD {offsets[key]['mad_e_m']:.2f}/"
              f"{offsets[key]['mad_n_m']:.2f} m; per leg " +
              ", ".join(f"leg{k}: ({v['de_m']:+.2f},{v['dn_m']:+.2f})" for k, v in offsets[key]["per_leg"].items()))
    out = {"boresight_deg": bs, "median_zncc_precut_0.5m": best, "median_zncc_no_distortion": s_nodist,
           "leg_gap_diagnostic_0.25m": diag, "map_offsets": offsets,
           "takeoff_alt_ell_m": float(G.truth_xy().alt_ell.iloc[[f - 1 for f in pre]].mean() - 100.0),
           "note": "pre-cut frames only; RTK pose + RTK height above DEM used; takeoff = mean pre-cut RTK "
                   "altitude minus the planned 100 m"}
    (G.XOUT / "stage1_calibration.json").write_text(json.dumps(out, indent=1))
    print(f"wrote {G.XOUT / 'stage1_calibration.json'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["fetch", "coverage", "resample", "calibrate"])
    args = ap.parse_args()
    {"fetch": cmd_fetch, "coverage": cmd_coverage, "resample": cmd_resample, "calibrate": cmd_calibrate}[args.cmd](args)


if __name__ == "__main__":
    main()
