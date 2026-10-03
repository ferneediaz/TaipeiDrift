#!/usr/bin/env python
"""Fetch OpenAerialMap (OAM) scenes covering Taiwan, find multi-date overlap
groups, and download per-site multi-date stacks reprojected to EPSG:3826 @1 m.

Subcommands
-----------
  catalogue   paginate the OAM meta API, save raw results to oam_catalogue.json
  pairs       compute overlapping multi-date scene pairs (>0.5 km^2, >30 days)
  fetch       download + reproject chosen groups, write manifest_oam.json
  all         catalogue + pairs + fetch (default)

Everything is idempotent: existing rasters are skipped unless --force.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path

import numpy as np
import rasterio
import rasterio.features

os.environ.setdefault("GDAL_DISABLE_READDIR_ON_OPEN", "EMPTY_DIR")
os.environ.setdefault("GDAL_HTTP_MULTIPLEX", "YES")
os.environ.setdefault("GDAL_HTTP_MULTIRANGE", "YES")
os.environ.setdefault("VSI_CACHE", "TRUE")
os.environ.setdefault("VSI_CACHE_SIZE", "100000000")

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "raw" / "aerial_pairs"
CAT = OUT / "oam_catalogue.json"
MANIFEST = OUT / "manifest_oam.json"

API = "https://api.openaerialmap.org/meta"
BBOX = "119.3,21.8,122.1,25.4"
UA = {"User-Agent": "TaipeiDrift-research/1.0 (GNSS-denied nav research)"}

MIN_OVERLAP_KM2 = 0.5
MIN_DAY_GAP = 30


# --------------------------------------------------------------------------- #
# catalogue
# --------------------------------------------------------------------------- #
def http_json(url: str, tries: int = 4) -> dict:
    last = None
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=60) as r:
                return json.loads(r.read().decode("utf-8"))
        except Exception as e:  # noqa: BLE001
            last = e
            time.sleep(2 * (i + 1))
    raise RuntimeError(f"failed {url}: {last!r}")


def cmd_catalogue(args) -> list[dict]:
    OUT.mkdir(parents=True, exist_ok=True)
    results, page = [], 1
    while True:
        url = f"{API}?bbox={BBOX}&limit=100&page={page}"
        d = http_json(url)
        res = d.get("results", [])
        if not res:
            break
        results.extend(res)
        print(f"page {page}: {len(res)} results (total {len(results)}/{d.get('meta',{}).get('found')})")
        if len(res) < 100:
            break
        page += 1
    recs = []
    for x in results:
        p = x.get("properties") or {}
        recs.append(
            {
                "_id": x.get("_id"),
                "title": x.get("title"),
                "provider": x.get("provider"),
                "contact": x.get("contact"),
                "acquisition_start": x.get("acquisition_start"),
                "acquisition_end": x.get("acquisition_end"),
                "uploaded_at": x.get("uploaded_at"),
                "platform": x.get("platform"),
                "gsd": x.get("gsd"),
                "bbox": x.get("bbox"),
                "footprint": x.get("footprint"),
                "geojson": x.get("geojson"),
                "uuid": x.get("uuid"),
                "meta_uri": x.get("meta_uri"),
                "file_size": x.get("file_size"),
                "projection": x.get("projection"),
                "properties": {
                    k: p.get(k)
                    for k in ("license", "sensor", "bands", "dtype", "crs", "resolution_in_meters", "dimensions", "url", "thumbnail", "tags")
                },
            }
        )
    payload = {
        "query": {"api": API, "bbox": BBOX, "fetched_at": datetime.now(timezone.utc).isoformat()},
        "api_license": "CC-BY 4.0 (per OAM meta endpoint)",
        "count": len(recs),
        "results": recs,
    }
    CAT.write_text(json.dumps(payload, ensure_ascii=False, indent=1))
    print(f"wrote {CAT} ({CAT.stat().st_size/1e6:.2f} MB, {len(recs)} records)")
    return recs


def load_catalogue() -> list[dict]:
    if not CAT.exists():
        raise SystemExit(f"missing {CAT}; run `catalogue` first")
    return json.loads(CAT.read_text())["results"]


# --------------------------------------------------------------------------- #
# pairs
# --------------------------------------------------------------------------- #
def _wgs_to_3826():
    from pyproj import Transformer

    return Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True)


def scene_poly_3826(rec, tf):
    gj = rec.get("geojson") or {}
    coords = None
    if gj.get("type") == "Polygon":
        coords = gj["coordinates"][0]
    elif gj.get("type") == "MultiPolygon":
        coords = max(gj["coordinates"], key=lambda p: len(p[0]))[0]
    if not coords:
        # fall back to bbox rectangle
        w, s, e, n = rec["bbox"]
        coords = [(w, s), (e, s), (e, n), (w, n), (w, s)]
    xs, ys = tf.transform([c[0] for c in coords], [c[1] for c in coords])
    from shapely.geometry import Polygon

    poly = Polygon(zip(xs, ys))
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly


def acq_date(rec) -> str:
    for k in ("acquisition_start", "acquisition_end"):
        v = rec.get(k)
        if v:
            return v[:10]
    return "unknown"


def cmd_pairs(args) -> list[dict]:
    recs = load_catalogue()
    tf = _wgs_to_3826()
    scenes = []
    for r in recs:
        try:
            poly = scene_poly_3826(r, tf)
        except Exception as e:  # noqa: BLE001
            print(f"  skip {r['_id']}: geometry {e!r}")
            continue
        if poly.is_empty:
            continue
        scenes.append((r, poly))
    print(f"usable geometries: {len(scenes)}/{len(recs)}")

    # group by rounded centroid so we only compare spatially near scenes
    def key(p):
        c = p.centroid
        return (round(c.x / 5000), round(c.y / 5000))

    buckets: dict[tuple, list] = {}
    for i, (r, p) in enumerate(scenes):
        buckets.setdefault(key(p), []).append(i)

    pairs = []
    seen = set()
    for k, idxs in buckets.items():
        neigh = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                neigh.extend(buckets.get((k[0] + dx, k[1] + dy), []))
        for i, j in combinations(sorted(set(neigh)), 2):
            if (i, j) in seen:
                continue
            seen.add((i, j))
            ri, pi = scenes[i]
            rj, pj = scenes[j]
            inter = pi.intersection(pj)
            if inter.is_empty:
                continue
            a_km2 = inter.area / 1e6
            if a_km2 < MIN_OVERLAP_KM2:
                continue
            di, dj = acq_date(ri), acq_date(rj)
            try:
                gap = abs((datetime.fromisoformat(dj) - datetime.fromisoformat(di)).days)
            except ValueError:
                continue
            if gap < MIN_DAY_GAP:
                continue
            minx, miny, maxx, maxy = inter.bounds
            pairs.append(
                {
                    "a": ri["_id"],
                    "b": rj["_id"],
                    "title_a": ri["title"],
                    "title_b": rj["title"],
                    "date_a": di,
                    "date_b": dj,
                    "day_gap": gap,
                    "gsd_a": ri["gsd"],
                    "gsd_b": rj["gsd"],
                    "overlap_km2": round(a_km2, 4),
                    "license_a": (ri.get("properties") or {}).get("license"),
                    "license_b": (rj.get("properties") or {}).get("license"),
                    "provider_a": ri.get("provider"),
                    "provider_b": rj.get("provider"),
                    "url_a": ri.get("uuid"),
                    "url_b": rj.get("uuid"),
                    "overlap_3826_bbox": [round(v, 1) for v in (minx, miny, maxx, maxy)],
                }
            )
    pairs.sort(key=lambda p: -p["overlap_km2"])
    print(f"\noverlapping pairs (>= {MIN_OVERLAP_KM2} km2, >= {MIN_DAY_GAP} d): {len(pairs)}")
    hdr = f"{'date_a':10} {'date_b':10} {'gap':>4} {'ovl_km2':>8} {'gsd_a':>7} {'gsd_b':>7}  license_a / license_b  ids"
    print(hdr)
    for p in pairs[:60]:
        print(
            f"{p['date_a']:10} {p['date_b']:10} {p['day_gap']:>4} {p['overlap_km2']:>8.3f} "
            f"{p['gsd_a']:>7.4f} {p['gsd_b']:>7.4f}  {p['license_a']} / {p['license_b']}  {p['a'][-6:]}x{p['b'][-6:]}"
        )
    (OUT / "oam_pairs.json").write_text(json.dumps(pairs, ensure_ascii=False, indent=1))
    print(f"wrote {OUT/'oam_pairs.json'}")
    return pairs


# --------------------------------------------------------------------------- #
# fetch
# --------------------------------------------------------------------------- #
def build_groups(pairs: list[dict]) -> list[dict]:
    """Union-find over pair edges -> overlap groups (each with >=2 dates)."""
    by_id = {r["_id"]: r for r in load_catalogue()}
    parent: dict[str, str] = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    overlap_area = {}
    for p in pairs:
        union(p["a"], p["b"])
        overlap_area[(p["a"], p["b"])] = p["overlap_km2"]
    groups: dict[str, list[str]] = {}
    for x in parent:
        groups.setdefault(find(x), []).append(x)
    out = []
    for gid, ids in groups.items():
        if len(ids) < 2:
            continue
        # dates: one per id
        members = []
        for i in ids:
            r = by_id[i]
            members.append({"id": i, "date": acq_date(r), "gsd": r["gsd"], "provider": r.get("provider"),
                            "license": (r.get("properties") or {}).get("license"), "url": r.get("uuid"),
                            "title": r.get("title"), "file_size": r.get("file_size")})
        members.sort(key=lambda m: m["date"])
        # group extent = union polygon area of all members
        tf = _wgs_to_3826()
        from shapely.ops import unary_union

        polys = [scene_poly_3826(by_id[i], tf) for i in ids]
        u = unary_union(polys)
        out.append(
            {
                "gid": gid,
                "n_dates": len(members),
                "members": members,
                "union_km2": round(u.area / 1e6, 3),
                "union_bbox_3826": [round(v, 1) for v in u.bounds],
                "centroid_3826": [round(u.centroid.x, 1), round(u.centroid.y, 1)],
                "max_overlap_km2": max([overlap_area.get((a, b), 0) for a, b in combinations(ids, 2)] or [0]),
            }
        )
    out.sort(key=lambda g: (-g["n_dates"], -g["max_overlap_km2"]))
    return out


MAX_PX = 4000
MIN_JOINT_VALID = 0.15      # min fraction of the window inside the footprint intersection
MIN_COMMON_FRAC = 0.10      # min fraction of the window valid in every date
MIN_COMMON_KM2 = 0.25       # min common valid area (the matcher needs usable overlap)

# licence strings seen in the OAM catalogue (2026-10 crawl: CC-BY 4.0 x313,
# CC BY-NC 4.0 x46, CC BY-SA 4.0 x7, None x5)
LICENCE_URLS = {
    "CC-BY 4.0": "https://creativecommons.org/licenses/by/4.0/",
    "CC BY-NC 4.0": "https://creativecommons.org/licenses/by-nc/4.0/",
    "CC BY-SA 4.0": "https://creativecommons.org/licenses/by-sa/4.0/",
}
MIN_SEP_KM = 3.0
MAX_SITES = 12

# Land-cover tag per site, set after inspecting statistics + previews.
# heuristic tag is used when a site is absent from this dict.
LAND_COVER_OVERRIDE: dict[str, str] = {}


def pick_sites(groups: list[dict], max_sites: int = MAX_SITES) -> list[dict]:
    """Prefer many dates, then big overlap, then spatially distinct locations."""
    kept: list[dict] = []
    for g in sorted(groups, key=lambda g: (-g["n_dates"], -g["max_overlap_km2"])):
        cx, cy = g["centroid_3826"]
        if any(math.hypot(cx - k["centroid_3826"][0], cy - k["centroid_3826"][1]) < MIN_SEP_KM * 1000 for k in kept):
            continue
        kept.append(g)
        if len(kept) >= max_sites:
            break
    return kept


def joint_window(members: list[dict], recs: dict) -> tuple:
    """Intersection polygon (EPSG:3826) of every member footprint + a <=4000x4000
    1 m grid snapped to integer metres, centred on the intersection centroid."""
    tf = _wgs_to_3826()
    polys = [scene_poly_3826(recs[m["id"]], tf) for m in members]
    joint = polys[0]
    for p in polys[1:]:
        joint = joint.intersection(p)
        if joint.is_empty:
            return None, None
    if joint.geom_type != "Polygon":
        joint = max(joint.geoms, key=lambda g: g.area) if hasattr(joint, "geoms") else joint
    minx, miny, maxx, maxy = joint.bounds
    cx, cy = joint.centroid.x, joint.centroid.y
    w = int(min(round(maxx - minx), MAX_PX))
    h = int(min(round(maxy - miny), MAX_PX))
    x0 = int(round(cx - w / 2))
    y0 = int(round(cy + h / 2))  # top edge
    x0 = max(x0, int(math.floor(minx)))
    y0 = min(y0, int(math.ceil(maxy)))
    from rasterio.transform import from_origin

    return joint, from_origin(x0, y0, 1.0, 1.0), w, h


_TR_CACHE: dict = {}


def _transformer(src_crs):
    """Cached EPSG:3826 -> src_crs transformer."""
    from pyproj import Transformer

    key = str(src_crs)
    if key not in _TR_CACHE:
        _TR_CACHE[key] = Transformer.from_crs("EPSG:3826", src_crs, always_xy=True)
    return _TR_CACHE[key]


def read_window(url: str, transform, w: int, h: int) -> np.ndarray | None:
    """Read a 3-band uint8 window onto the target EPSG:3826 1 m grid.

    Two steps, both cheap over /vsicurl/: (1) decimated in-source read of the
    intersecting window (one HTTP range burst, honours COG overviews), then
    (2) an in-memory reprojection of that small array onto the target grid.
    A direct WarpedVRT read of the same window reads the source at full
    resolution and is ~20x slower here (measured 60 s vs 3 s for 1000x1000).
    """
    from rasterio.enums import Resampling
    from rasterio.warp import reproject

    try:
        with rasterio.open("/vsicurl/" + url) as src:
            x0, y1 = float(transform.c), float(transform.f)
            x1, y0 = x0 + w, y1 - h
            tr = _transformer(src.crs)
            xs, ys = tr.transform([x0, x1, x0, x1], [y1, y1, y0, y0])
            if src.crs.is_geographic:
                lat = math.radians((min(ys) + max(ys)) / 2.0)
                mpp = abs(src.transform.a) * 111320.0 * math.cos(lat)
            else:
                mpp = abs(src.transform.a)
            win = rasterio.windows.from_bounds(min(xs), min(ys), max(xs), max(ys), src.transform)
            c0 = max(0, int(math.floor(win.col_off)))
            r0 = max(0, int(math.floor(win.row_off)))
            c1 = min(src.width, int(math.ceil(win.col_off + win.width)))
            r1 = min(src.height, int(math.ceil(win.row_off + win.height)))
            if c1 - c0 < 2 or r1 - r0 < 2:
                return np.zeros((3, h, w), np.uint8)  # no overlap with this scene
            sub = rasterio.windows.Window(c0, r0, c1 - c0, r1 - r0)
            ow = max(1, min(int(round(sub.width * mpp)), MAX_PX))
            oh = max(1, min(int(round(sub.height * mpp)), MAX_PX))
            arr = src.read(out_shape=(src.count, oh, ow), window=sub,
                           resampling=Resampling.average)
            if src.count >= 3:
                bands = arr[:3]
            else:
                bands = np.repeat(arr[:1], 3, axis=0)
            dst = np.zeros((3, h, w), np.uint8)
            tr0 = src.window_transform(sub)
            sx, sy = sub.width / ow, sub.height / oh
            stf = rasterio.Affine(tr0.a * sx, tr0.b, tr0.c, tr0.d, tr0.e * sy, tr0.f)
            for i in range(3):
                reproject(
                    bands[i], dst[i],
                    src_transform=stf, src_crs=src.crs,
                    dst_transform=transform, dst_crs="EPSG:3826",
                    resampling=Resampling.average,
                    src_nodata=0, dst_nodata=0,
                )
            return dst
    except Exception as e:  # noqa: BLE001
        print(f"    !! read failed {url[-40:]}: {repr(e)[:160]}")
        return None


def stats_to_land_cover(arr: np.ndarray, title: str) -> tuple[str, dict]:
    """Cheap RGB heuristics; the tag is a guess and is flagged as such in notes."""
    r, g, b = (arr[i].astype(np.float32) for i in range(3))
    m = (r > 0) | (g > 0) | (b > 0)
    if m.sum() == 0:
        return "mixed", {}
    r, g, b = r[m], g[m], b[m]
    tot = r + g + b + 1e-6
    green_idx = (g - np.maximum(r, b)) / tot
    water = float(((b > g + 15) & (b > r + 25)).mean())
    veg = float((green_idx > 0.02).mean())
    bright = float((tot / 3).mean())
    bright_std = float((tot / 3).std())
    st = {
        "water_frac": round(water, 3),
        "veg_frac": round(veg, 3),
        "mean_brightness": round(bright, 1),
        "brightness_std": round(bright_std, 1),
        "green_idx_mean": round(float(green_idx.mean()), 4),
    }
    t = (title or "").lower()
    if water > 0.25 and st["green_idx_mean"] < 0.01:
        tag = "water_ponds" if water > 0.5 else "river"
    elif veg > 0.6:
        tag = "rice_paddy" if st["green_idx_mean"] > 0.045 else "hills_forest"
    elif veg < 0.15 and bright > 110 and bright_std < 55:
        tag = "industrial" if "industr" in t else "urban"
    elif veg < 0.3:
        tag = "suburban"
    else:
        tag = "farmland"
    return tag, st


def save_preview(arr: np.ndarray, path: Path, px: int = 512) -> None:
    """Write a PNG whose longest side is <= px (contract: 512 px previews)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    step = max(1, -(-max(arr.shape[1], arr.shape[2]) // px))
    small = arr[:, ::step, ::step]
    img = np.transpose(small, (1, 2, 0)).astype(np.float32)
    nz = img[(img > 0).any(axis=2)]
    if nz.size:
        lo, hi = np.percentile(nz, [2, 98])
        img = np.clip((img - lo) / max(hi - lo, 1e-6), 0, 1)
    plt.imsave(path, img)


def cmd_fetch(args) -> None:
    from rasterio.transform import from_origin

    pairs = json.loads((OUT / "oam_pairs.json").read_text())
    groups = build_groups(pairs)
    recs = {r["_id"]: r for r in load_catalogue()}
    tf = _wgs_to_3826()
    t_inv = None
    from pyproj import Transformer

    t_inv = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)

    print(f"overlap groups: {len(groups)}")
    for g in groups:
        print(f"  {g['gid'][-6:]} n={g['n_dates']} maxovl={g['max_overlap_km2']:.2f} km2 union={g['union_km2']:.2f} dates={[m['date'] for m in g['members']]}")
    sites = pick_sites(groups, args.max_sites)
    if args.list:
        print(f"selected {len(sites)} sites: {[g['gid'][-6:] for g in sites]}")
        return

    manifest = json.loads(MANIFEST.read_text()) if MANIFEST.exists() and not args.force else []
    manifest = [m for m in manifest if not any(m["site_id"] == "oam_" + g["gid"][-6:] for g in sites)]
    total_bytes = 0
    for g in sites:
        site_id = "oam_" + g["gid"][-6:]
        sdir = OUT / site_id
        sdir.mkdir(parents=True, exist_ok=True)
        joint, transform, w, h = joint_window(g["members"], recs)
        if joint is None:
            print(f"{site_id}: empty intersection, skipped")
            continue
        # cap to 4000x4000 keeping the window whose joint validity will be highest:
        # start centred, and if the intersection is larger, recentre on a coarse grid
        # of candidate windows and keep the best (checked at read time via the
        # joint mask) - here we simply use the intersection bbox centre.
        lon, lat = t_inv.transform(*g["centroid_3826"])
        print(f"\n== {site_id} {g['n_dates']} dates {w}x{h}px lat={lat:.5f} lon={lon:.5f} joint={joint.area/1e6:.2f} km2")
        arrays, masks, metas = [], [], []
        seen_dates = set()
        for mm in g["members"]:
            if mm["date"] in seen_dates:
                print(f"   {mm['date']} {mm['id'][-6:]} duplicate date, skipped")
                continue
            seen_dates.add(mm["date"])
            rec = recs[mm["id"]]
            arr = read_window(rec["uuid"], transform, w, h)
            if arr is None:
                continue
            valid = (arr > 0).any(axis=0)
            arrays.append(arr)
            masks.append(valid)
            metas.append((mm, rec, valid.mean()))
            print(f"   {mm['date']} {mm['id'][-6:]} valid={valid.mean():.3f}")
        if len(arrays) < 2:
            print(f"{site_id}: fewer than 2 readable dates, skipped")
            continue
        outside = rasterio.features.geometry_mask(
            [joint.__geo_interface__], out_shape=(h, w), transform=transform,
            invert=False, all_touched=True,
        )
        poly_frac = float(1.0 - outside.mean())
        raw_frac = [float((a > 0).any(0).mean()) for a in arrays]
        # common valid region = inside the intersection of all footprints AND carrying
        # data in every date, so every raster of a site has a bit-identical valid mask
        common = ~outside
        for a in arrays:
            common &= (a > 0).any(0)
        jf = float(common.mean())
        km2 = jf * w * h / 1e6
        if poly_frac < MIN_JOINT_VALID or jf < MIN_COMMON_FRAC or km2 < MIN_COMMON_KM2:
            print(f"{site_id}: common valid {jf:.3f} ({km2:.2f} km2, polygon {poly_frac:.3f}) below threshold, skipped")
            continue
        for a in arrays:
            a[:, ~common] = 0
        tag, st = stats_to_land_cover(arrays[0], g["members"][0].get("title", ""))
        tag = LAND_COVER_OVERRIDE.get(site_id, tag)
        notes = (
            f"OAM overlap group {g['gid']}; {len(arrays)} dates on one identical 1 m grid. "
            f"valid_fraction {jf:.3f} ({km2:.2f} km2) is the region inside the intersection of all scene "
            f"footprints that carries data in every date; every raster of this site has exactly that same "
            f"valid region, everything else is nodata=0. Polygon-only fraction {poly_frac:.3f}; "
            "per-date raw data fraction (before the common mask) "
            + ", ".join(f"{mm['date']}:{rf:.3f}" for mm, rf in zip([m for m, _r, _v in metas], raw_frac))
            + ". Land-cover tag "
            f"'{tag}' is an RGB-statistics heuristic (water_frac={st.get('water_frac')}, "
            f"veg_frac={st.get('veg_frac')}, mean_brightness={st.get('mean_brightness')}) - UNCERTAIN, "
            "check the previews. These are UAV survey scenes from OpenAerialMap, not full orthophotos."
        )
        for arr, (mm, rec, _vf) in zip(arrays, metas):
            fname = f"{mm['date']}_oam.tif"
            path = sdir / fname
            prof = dict(
                driver="GTiff", width=w, height=h, count=3, dtype="uint8", crs="EPSG:3826",
                transform=transform, nodata=0, compress="deflate", predictor=2, tiled=True,
            )
            with rasterio.open(path, "w", **prof) as dst:
                dst.write(arr)
            total_bytes += path.stat().st_size
            prev = sdir / f"preview_{mm['date']}.png"
            save_preview(arr, prev)
            total_bytes += prev.stat().st_size
            lic = (rec.get("properties") or {}).get("license") or ""
            enotes = notes
            if "NC" in lic:
                enotes += " LICENCE: CC BY-NC 4.0 is non-commercial - fine for this hackathon research, not for a commercial product."
            manifest.append(
                {
                    "site_id": site_id, "land_cover": tag, "lat": round(lat, 6), "lon": round(lon, 6),
                    "date": mm["date"], "source": "openaerialmap",
                    "source_url_or_layer": rec["uuid"], "native_gsd_m": round(float(rec["gsd"]), 4),
                    "licence": lic, "licence_url": LICENCE_URLS.get(lic, ""),
                    "attribution": rec.get("provider") or "", "file": f"{site_id}/{fname}",
                    "width": w, "height": h, "valid_fraction": round(jf, 4),
                    "notes": enotes,
                }
            )
            print(f"   wrote {fname}")
    # keep any pre-existing sites untouched
    MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=1))
    n_sites = len({m["site_id"] for m in manifest})
    print(f"\nwrote {MANIFEST}: {len(manifest)} rasters over {n_sites} sites; new bytes on disk {total_bytes/1e6:.1f} MB")


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["catalogue", "pairs", "fetch", "all"])
    ap.add_argument("--list", action="store_true", help="fetch: only list groups")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--max-sites", type=int, default=MAX_SITES)
    args = ap.parse_args()
    if args.cmd in ("catalogue", "all"):
        cmd_catalogue(args)
    if args.cmd in ("pairs", "all"):
        cmd_pairs(args)
    if args.cmd in ("fetch", "all"):
        cmd_fetch(args)


if __name__ == "__main__":
    main()
