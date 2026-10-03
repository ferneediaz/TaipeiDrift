"""Draw a topographic map of the islands world, and export its elevation grid: sim/maps/.

Reads what make_islands.py generated (models/islands/terrain.npz and layout.json) and writes:

- islands_topo.png and islands_topo.pdf: the map. Land contours every 1 m (labelled every 5 m),
  depth contours every 2 m, hillshade, coastline, helipads, houses, the lighthouse, trees, the
  demo route, a 100 m grid in world metres and latitude and longitude on the outer edges.
- islands_dem.tif: the elevation grid, float32, metres above the sea (negative: depth), 1 m per pixel,
  image top is north. islands_dem.json gives where it lies, in world metres and in latitude and longitude.

Heights on the map are above the sea. World z (and the drone's altitude above the start pad) is
height above the sea minus 4 m.

Usage: python3 sim/scripts/make_topo_map.py [--res M] [--dpi N]
"""
import argparse
import json
import math
import re
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, LightSource  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

SIM = Path(__file__).resolve().parents[1]
MODEL = SIM / "models" / "islands"
OUT = SIM / "maps"
EARTH_R = 6378137.0
MARGIN = 120.0  # m of sea shown around the islands
DEEP = 12.0  # m: deepest colour from here on, as in the world's sea texture
DEPTH_LINE = "#d8ecff"

# Hypsometric tints: depth in blues, land from beach to hilltop
SEA_CMAP = LinearSegmentedColormap.from_list("sea", ["#1d4f86", "#3c7fb8", "#7fbfdc", "#c4e6ef"])
LAND_CMAP = LinearSegmentedColormap.from_list("land", ["#e9dfb8", "#c9d79a", "#9fbf78", "#c8b27a", "#a98d63", "#8a6d4f"])


def world_origin():
    """Latitude, longitude and elevation of the world origin, from worlds/islands.sdf."""
    text = (SIM / "worlds" / "islands.sdf").read_text()
    get = lambda tag: float(re.search(rf"<{tag}>([-\d.]+)</{tag}>", text).group(1))  # noqa: E731
    return get("latitude_deg"), get("longitude_deg"), get("elevation")


def dem(layout, grids, res):
    """Elevation above the sea on a world-aligned grid; returns (grid, x0, y0) with row 0 at y0 (south)."""
    tiles = [(isl["centre"], float(grids[f"tile_{n}"]), grids[f"h_{n}"]) for n, isl in layout["islands"].items()]
    reach = [(i["centre"], 1.3 * i["radius"] + MARGIN) for i in layout["islands"].values()]  # coast lies within 1.3 radii
    x0, x1 = min(c[0] - r for c, r in reach), max(c[0] + r for c, r in reach)
    y0, y1 = min(c[1] - r for c, r in reach), max(c[1] + r for c, r in reach)
    xs, ys = np.arange(x0, x1 + res / 2, res), np.arange(y0, y1 + res / 2, res)
    gx, gy = np.meshgrid(xs, ys)
    out = np.full(gx.shape, -layout["seabed_m"], np.float32)
    for (cx, cy), tile, h in tiles:
        n = h.shape[0]
        col = ((gx - cx) / tile + 0.5) * (n - 1)
        row = ((gy - cy) / tile + 0.5) * (n - 1)
        inside = (col >= 0) & (col <= n - 1) & (row >= 0) & (row <= n - 1)
        val = cv2.remap(h, col.astype(np.float32), row.astype(np.float32), cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REPLICATE)
        out[inside] = val[inside]
    return out, xs, ys


def draw(z, xs, ys, layout, origin, pad_asl, dpi):
    lat0, lon0, _ = origin
    extent = (xs[0], xs[-1], ys[0], ys[-1])
    w, h = extent[1] - extent[0], extent[3] - extent[2]
    fig = plt.figure(figsize=(18, 18 * h / w + 2.6), dpi=dpi)
    ax = fig.add_axes([0.05, 0.16, 0.9, 0.74])
    ax.set_facecolor("#1d4f86")

    # Colour by height, with hillshade from the north-west as in the world. Contours and shading use a
    # smoothed copy, so the half-metre bumps of the ground do not turn every line into a zigzag.
    res = xs[1] - xs[0]
    zs = cv2.GaussianBlur(z, (0, 0), 3.0 / res)
    sea = SEA_CMAP(np.clip((z + DEEP) / DEEP, 0, 1))
    land = LAND_CMAP(np.clip(z / max(z.max(), 1), 0, 1))
    rgb = np.where((z >= 0)[..., None], land, sea)[..., :3]
    shade = LightSource(azdeg=315, altdeg=40).hillshade(zs, vert_exag=3, dx=res, dy=res)
    rgb = np.where((z >= 0)[..., None], rgb * (0.55 + 0.6 * shade[..., None]), rgb)
    ax.imshow(np.clip(rgb, 0, 1), extent=extent, origin="lower", interpolation="bilinear")

    # Contours: land every 1 m (index every 5 m), depth every 2 m, coastline at 0
    top = math.ceil(z.max())
    ax.contour(xs, ys, zs, levels=[l for l in range(1, top) if l % 5], colors="#6b4f32", linewidths=0.4, alpha=0.8)
    cs = ax.contour(xs, ys, zs, levels=list(range(5, top, 5)), colors="#5a3f24", linewidths=1.0)
    ax.clabel(cs, fmt="%d", fontsize=7, inline_spacing=2)
    ds = ax.contour(xs, ys, zs, levels=list(range(-12, 0, 2)), colors=DEPTH_LINE, linewidths=0.5, alpha=0.8)
    ax.clabel(ds, fmt=lambda v: f"{-v:.0f}", fontsize=6, colors="#e8f4ff", inline_spacing=2)
    ax.contour(xs, ys, z, levels=[0], colors="#2a2a2a", linewidths=1.3)

    # Trees, houses, helipads, lighthouse
    if layout.get("trees"):
        t = np.array(layout["trees"])
        ax.scatter(t[:, 0], t[:, 1], s=9, c="#2f6b2a", edgecolors="#1c401a", linewidths=0.3, zorder=4)
    for hs in layout.get("houses", []):
        c, s = math.cos(hs["yaw"]), math.sin(hs["yaw"])
        corners = [(hs["x"] + a * hs["size"][0] / 2 * c - b * hs["size"][1] / 2 * s,
                    hs["y"] + a * hs["size"][0] / 2 * s + b * hs["size"][1] / 2 * c) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        ax.add_patch(Polygon(corners, closed=True, fc="#8b2f22", ec="#2a0f0a", lw=0.6, zorder=5))
    for name, (px, py) in layout["pads"].items():
        ax.add_patch(Rectangle((px - 4, py - 4), 8, 8, fc="#cfcfcf", ec="#e0b000", lw=1.2, zorder=6))
        ax.text(px, py, "H", ha="center", va="center", fontsize=7, weight="bold", zorder=7)
        ax.annotate(f"Helipad {name.upper()}\nx {px:.0f}, y {py:.0f} m\n{pad_asl:.0f} m above sea",
                    (px, py), (px - 20, py - 62), fontsize=8, zorder=8,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#555", alpha=0.85),
                    arrowprops=dict(arrowstyle="-", color="#333", lw=0.7))
    if layout.get("lighthouse"):
        lh = layout["lighthouse"]
        ax.plot(lh["x"], lh["y"], marker="*", ms=16, mfc="#d22", mec="white", mew=1, zorder=8)
        ax.annotate(f"Lighthouse\ntop {lh['top_z'] + pad_asl:.0f} m above sea", (lh["x"], lh["y"]),
                    (lh["x"] + 18, lh["y"] + 22), fontsize=8, zorder=8,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#555", alpha=0.85),
                    arrowprops=dict(arrowstyle="-", color="#333", lw=0.7))

    # Highest point of each island
    for name, isl in layout["islands"].items():
        cx, cy, r = *isl["centre"], isl["radius"] * 1.3
        m = (np.abs(xs - cx) < r)[None, :] & (np.abs(ys - cy) < r)[:, None]
        i = np.argmax(np.where(m, z, -np.inf))
        hy, hx = np.unravel_index(i, z.shape)
        ax.plot(xs[hx], ys[hy], "k^", ms=6, zorder=7)
        ax.text(xs[hx] + 6, ys[hy] + 4, f"{z[hy, hx]:.1f}", fontsize=8, weight="bold", zorder=7)
        ax.text(cx, cy + isl["radius"] * 1.08 + 10, f"ISLAND {name.upper()}", ha="center", fontsize=13,
                weight="bold", color="#1a1a1a", zorder=7,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.6))

    # Demo route between the helipads, with distance and bearing
    (ax_, ay), (bx, by) = layout["pads"]["a"], layout["pads"]["b"]
    ax.plot([ax_, bx], [ay, by], ls=(0, (6, 4)), color="#ffd23a", lw=1.6, zorder=3)
    dist = math.hypot(bx - ax_, by - ay)
    bearing = math.degrees(math.atan2(bx - ax_, by - ay)) % 360
    mx, my = (ax_ + bx) / 2, (ay + by) / 2
    ax.text(mx, my + 14, f"Demo route  {dist:.0f} m  bearing {bearing:03.0f}°  (open sea, no landmarks)",
            ha="center", fontsize=9, color="white", rotation=math.degrees(math.atan2(by - ay, bx - ax_)),
            rotation_mode="anchor", zorder=8)

    # 100 m grid in world metres; latitude and longitude on the top and right edges
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(math.ceil(extent[0] / 100) * 100, extent[1], 100))
    ax.set_yticks(np.arange(math.ceil(extent[2] / 100) * 100, extent[3], 100))
    ax.grid(color="white", lw=0.4, alpha=0.35)
    ax.tick_params(labelsize=8)
    ax.set_xlabel("x, east of helipad A (m)", fontsize=9)
    ax.set_ylabel("y, north of helipad A (m)", fontsize=9)
    m_per_deg_lat = math.pi * EARTH_R / 180
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(lat0))
    top_ax = ax.secondary_xaxis("top", functions=(lambda x: lon0 + x / m_per_deg_lon, lambda l: (l - lon0) * m_per_deg_lon))
    right_ax = ax.secondary_yaxis("right", functions=(lambda y: lat0 + y / m_per_deg_lat, lambda l: (l - lat0) * m_per_deg_lat))
    for sec, label in ((top_ax, "longitude (° E)"), (right_ax, "latitude (° N)")):
        sec.tick_params(labelsize=8)
        sec.set_xlabel(label, fontsize=9) if sec is top_ax else sec.set_ylabel(label, fontsize=9)
    top_ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.4f}"))
    right_ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.4f}"))

    # North arrow and scale bar
    nx, ny = extent[1] - 45, extent[3] - 95
    ax.annotate("", (nx, ny + 60), (nx, ny), arrowprops=dict(arrowstyle="-|>", lw=2, color="black", mutation_scale=18), zorder=9)
    ax.text(nx, ny + 66, "N", ha="center", fontsize=14, weight="bold", zorder=9)
    sx, sy = extent[0] + 30, extent[2] + 30
    for k in range(4):
        ax.add_patch(Rectangle((sx + k * 50, sy), 50, 7, fc="black" if k % 2 == 0 else "white", ec="black", lw=0.8, zorder=9))
    for k in range(5):
        ax.text(sx + k * 50, sy + 11, f"{k * 50}", ha="center", fontsize=8, zorder=9, color="white")
    ax.text(sx + 210, sy + 1, "m", fontsize=8, color="white", zorder=9)

    fig.text(0.05, 0.955, "TaipeiDrift islands world: topographic map", fontsize=18, weight="bold")
    fig.text(0.05, 0.93, f"Taiwan Strait, origin (helipad A) {lat0:.4f}° N, {lon0:.4f}° E. Heights and depths in metres "
             f"relative to the sea surface. World z = height above sea − {pad_asl:.0f} m.", fontsize=10)

    # Legend: colour bars and symbols
    cax = fig.add_axes([0.05, 0.07, 0.3, 0.018])
    grad = np.linspace(-layout["seabed_m"], z.max(), 512)[None]
    cols = np.where((grad >= 0)[..., None], LAND_CMAP(np.clip(grad / z.max(), 0, 1)),
                    SEA_CMAP(np.clip((grad + DEEP) / DEEP, 0, 1)))
    cax.imshow(cols, aspect="auto", extent=(grad.min(), grad.max(), 0, 1))
    cax.set_yticks([])
    cax.set_xticks(list(range(-14, int(z.max()) + 1, 2)))
    cax.tick_params(labelsize=8)
    cax.set_xlabel("height above sea (m); negative is depth", fontsize=9)
    items = [("Coastline (0 m)", dict(color="#2a2a2a", lw=1.3)), ("Land contour, 1 m (bold every 5 m)", dict(color="#6b4f32", lw=0.8)),
             ("Depth contour, 2 m", dict(color="#3c7fb8", lw=0.8)), ("Demo route", dict(color="#e0b000", lw=1.6, ls="--"))]
    lax = fig.add_axes([0.42, 0.03, 0.55, 0.09])
    lax.axis("off")
    for k, (label, style) in enumerate(items):
        y = 0.85 - k * 0.25
        lax.plot([0, 0.05], [y, y], **style)
        lax.text(0.065, y, label, va="center", fontsize=9)
    marks = [("Helipad (8 m)", dict(marker="s", ms=9, mfc="#cfcfcf", mec="#e0b000")), ("House", dict(marker="s", ms=8, mfc="#8b2f22", mec="#2a0f0a")),
             ("Tree", dict(marker="o", ms=5, mfc="#2f6b2a", mec="#1c401a")), ("Lighthouse", dict(marker="*", ms=13, mfc="#d22", mec="#555")),
             ("Highest point (m above sea)", dict(marker="^", ms=7, mfc="k", mec="k"))]
    for k, (label, style) in enumerate(marks):
        y = 0.85 - (k % 4) * 0.25
        x = 0.45 + (k // 4) * 0.3
        lax.plot([x + 0.02], [y], ls="none", **style)
        lax.text(x + 0.045, y, label, va="center", fontsize=9)
    lax.set_xlim(0, 1)
    lax.set_ylim(0, 1)
    return fig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--res", type=float, default=1.0, help="elevation grid spacing, m")
    ap.add_argument("--dpi", type=int, default=200)
    args = ap.parse_args()
    if not (MODEL / "terrain.npz").exists():
        raise SystemExit("No islands terrain yet. Run: python3 sim/scripts/make_islands.py")
    layout = json.loads((MODEL / "layout.json").read_text())
    pad_asl = -layout["sea_z"]
    origin = world_origin()
    z, xs, ys = dem(layout, np.load(MODEL / "terrain.npz"), args.res)
    OUT.mkdir(exist_ok=True)

    fig = draw(z, xs, ys, layout, origin, pad_asl, args.dpi)
    fig.savefig(OUT / "islands_topo.png", dpi=args.dpi)
    fig.savefig(OUT / "islands_topo.pdf")

    cv2.imwrite(str(OUT / "islands_dem.tif"), z[::-1])  # image top is north
    lat0, lon0, _ = origin
    (OUT / "islands_dem.json").write_text(json.dumps({
        "file": "islands_dem.tif",
        "values": "float32, metres above the sea surface (negative: depth below it)",
        "world_z": f"value - {pad_asl:g}",
        "resolution_m": args.res,
        "size_px": [len(xs), len(ys)],
        "image_top": "north (+y)",
        "pixel_centres_world_m": {"x_first": float(xs[0]), "x_last": float(xs[-1]),
                                  "y_top": float(ys[-1]), "y_bottom": float(ys[0])},
        "world_origin": {"latitude_deg": lat0, "longitude_deg": lon0, "note": "helipad A, x east, y north (ENU)"},
    }, indent=2) + "\n")
    print(f"map: {len(xs)} x {len(ys)} m, land up to {z.max():.1f} m, sea down to {-z.min():.0f} m -> {OUT}")


if __name__ == "__main__":
    main()
