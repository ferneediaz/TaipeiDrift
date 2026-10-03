"""Build the islands world's scenery: sim/models/islands/.

Two small, low islands with open sea between them, in the style of the Penghu islands in the
Taiwan Strait. Island A holds the start pad on its west side; island B lies about 230 m of open water to
the east (moved closer on Saturday 3 October for the demo flight: about 190 m of land after the pad, then
30 s of water at 8 m/s, then island B).

- Islands: 3D terrain meshes with irregular coastlines, beaches, dry grass, scrub and rock, up to
  about 14 m above the sea. They are solid.
- On land: 3D trees (from make_trees.py), a few houses, a lighthouse on island B, and a helipad on
  each island. Both helipad tops are at z = 0.
- Sea: a flat, opaque water surface, PAD_ASL metres below the helipads, coloured by depth
  (turquoise shallows and surf at the shore, deep blue offshore). Offshore it has almost no
  texture, which is what makes navigating over it hard.

The world origin is the helipad on island A, so the drone starts at z = 0 and the sea is at
z = -PAD_ASL. layout.json lists the helipads, houses, lighthouse and trees; terrain.npz holds the
height grids. make_topo_map.py draws the map from both.

Usage: python3 sim/scripts/make_islands.py [--seed S] [--if-missing]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_trees import tree  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "models" / "islands"
VERSION = "3"  # bump when the scenery changes, so --if-missing rebuilds old copies
PAD_ASL = 4.0  # m: helipad tops above the sea; the sea surface is at z = -PAD_ASL

# name, centre (x, y) in m, mean radius in m, highest ground in m above the sea,
# helipad (x, y), tree count, house count, lighthouse
ISLANDS = [
    dict(name="a", centre=(40.0, 0.0), radius=150.0, top=14.0, pad=(0.0, 0.0), trees=160, houses=7, lighthouse=False),
    dict(name="b", centre=(530.0, 120.0), radius=110.0, top=11.0, pad=(480.0, 110.0), trees=90, houses=3, lighthouse=True),
]
GRID = 257          # terrain vertices per side
COLLISION_STEP = 4  # every 4th vertex for the collision mesh
TEX = 2048          # island texture side in px
SEA_SIZE = 3000.0   # m, side of the depth-coloured sea
SEA_PX = 4096


def noise(rng, px, octaves=7, persistence=0.55):
    """Multi-scale value noise in [0, 1]."""
    acc = np.zeros((px, px), np.float32)
    for o in range(octaves):
        cells = max(2, px >> (octaves - o))
        layer = rng.random((cells, cells), dtype=np.float32)
        acc += cv2.resize(layer, (px, px), interpolation=cv2.INTER_CUBIC) * persistence ** (octaves - 1 - o)
    acc -= acc.min()
    return acc / acc.max()


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def terrain(rng, isl):
    """Ground height above the sea on a GRID x GRID grid around the island centre; returns (h, tile side)."""
    r0, top = isl["radius"], isl["top"]
    tile = 2 * 1.7 * r0
    u = np.linspace(-tile / 2, tile / 2, GRID, dtype=np.float32)
    x, y = np.meshgrid(u, u)  # row = +y (north), column = +x (east)
    theta, r = np.arctan2(y, x), np.hypot(x, y)
    # Irregular coastline: a few lobes and bays, plus a noisy warp
    shape = np.ones_like(r)
    for k in range(2, 8):
        shape += rng.uniform(0.04, 0.16) / k**0.5 * np.cos(k * theta + rng.uniform(0, 2 * np.pi))
    rho = r / (r0 * shape) + 0.12 * (noise(rng, GRID) - 0.5)
    hills = noise(rng, GRID, octaves=6, persistence=0.5)
    bumps = noise(rng, GRID, octaves=6, persistence=0.6)
    land = top * np.clip(1 - rho**2, 0, None) ** 0.9 * (0.45 + 0.55 * hills) + 0.5 * (bumps - 0.5) * (rho < 1)
    sea = -14.0 * (1 - np.exp(-np.clip(rho - 1, 0, None) / 0.25))  # shallows, then the seabed drops off
    h = np.where(rho < 1, np.maximum(land, 0.6 * (1 - rho)), sea)
    # Flatten the ground under the helipad, blending into the hills around it
    px, py = isl["pad"][0] - isl["centre"][0], isl["pad"][1] - isl["centre"][1]
    w = 1 - smoothstep(9, 28, np.hypot(x - px, y - py))
    return (h * (1 - w) + (PAD_ASL - 0.15) * w).astype(np.float32), tile


def sample(h, tile, x, y):
    """Bilinear ground height at island-local (x, y)."""
    c = (np.asarray(x) / tile + 0.5) * (GRID - 1)
    r = (np.asarray(y) / tile + 0.5) * (GRID - 1)
    return cv2.remap(h, np.float32(np.atleast_1d(c))[None], np.float32(np.atleast_1d(r))[None],
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)[0]


def texture(rng, h, tile):
    """Island colours from height and slope: surf line, sand, dry grass, scrub, rock; with baked relief shading."""
    e = cv2.resize(h, (TEX, TEX), interpolation=cv2.INTER_CUBIC)
    gy, gx = np.gradient(e, tile / TEX)
    slope = np.hypot(gx, gy)
    mix, patch, grain = noise(rng, TEX, 8), noise(rng, TEX, 9, 0.65), rng.random((TEX, TEX), dtype=np.float32)
    sand = np.array([214, 198, 160], np.float32)
    grass = np.array([158, 152, 92], np.float32)
    scrub = np.array([84, 108, 58], np.float32)
    rock = np.array([118, 108, 96], np.float32)
    wet = np.array([150, 140, 115], np.float32)
    veg = grass + (scrub - grass) * smoothstep(0.45, 0.65, mix)[..., None]
    rgb = veg + (sand - veg) * (1 - smoothstep(1.2, 2.4, e + 0.8 * (patch - 0.5)))[..., None]
    rgb = rgb + (rock - rgb) * smoothstep(0.35, 0.7, slope + 0.2 * (patch - 0.5))[..., None]
    rgb = rgb + (wet - rgb) * (1 - smoothstep(-0.4, 0.3, e))[..., None]
    shade = np.clip(1.0 + 0.8 * (-gx * 0.5 + gy * 0.35), 0.75, 1.15)  # sun from the north-west, as in the world
    rgb = rgb * (0.82 + 0.3 * patch)[..., None] * shade[..., None] + (grain[..., None] - 0.5) * 30
    # Dirt tracks from the helipad side of the island
    for _ in range(4):
        pts, heading = [rng.uniform(0.4, 0.6, 2) * TEX], rng.uniform(0, 2 * np.pi)
        for _ in range(60):
            heading += rng.normal(0, 0.15)
            pts.append(pts[-1] + 8 * np.array([np.cos(heading), np.sin(heading)]))
        cv2.polylines(rgb, [np.int32(pts)], False, (172, 152, 118), int(rng.integers(5, 9)))
    return np.clip(rgb, 0, 255).astype(np.uint8)[::-1]  # image top is north


def write_obj(path, h, tile, step, mtl=None):
    """Grid mesh in island-local coordinates (z = height above the sea), only where it reaches near the surface."""
    g = h[::step, ::step]
    n = g.shape[0]
    u = np.linspace(-tile / 2, tile / 2, n)
    x, y = np.meshgrid(u, u)
    gy, gx = np.gradient(g, tile / (n - 1))
    nrm = np.dstack([-gx, -gy, np.ones_like(g)])
    nrm /= np.linalg.norm(nrm, axis=2, keepdims=True)
    lines = [f"mtllib {mtl}\nusemtl ground\n" if mtl else ""]
    lines += [f"v {a:.2f} {b:.2f} {c:.2f}\n" for a, b, c in zip(x.ravel(), y.ravel(), g.ravel())]
    lines += [f"vt {a:.5f} {b:.5f}\n" for a, b in zip(((x / tile) + 0.5).ravel(), ((y / tile) + 0.5).ravel())]
    lines += [f"vn {a:.3f} {b:.3f} {c:.3f}\n" for a, b, c in nrm.reshape(-1, 3)]
    keep = np.maximum.reduce([g[:-1, :-1], g[1:, :-1], g[:-1, 1:], g[1:, 1:]]) > -3.0
    for i, j in zip(*np.nonzero(keep)):
        a, b, c, d = i * n + j + 1, i * n + j + 2, (i + 1) * n + j + 1, (i + 1) * n + j + 2
        lines.append(f"f {a}/{a}/{a} {b}/{b}/{b} {d}/{d}/{d}\nf {a}/{a}/{a} {d}/{d}/{d} {c}/{c}/{c}\n")
    path.write_text("".join(lines))


def sea_texture(rng, islands):
    """Water colour by depth over SEA_SIZE metres around the midpoint between the islands."""
    cx = np.mean([i["centre"][0] for i in islands])
    cy = np.mean([i["centre"][1] for i in islands])
    depth = np.full((SEA_PX, SEA_PX), 30.0, np.float32)  # deep water
    m_per_px = SEA_SIZE / SEA_PX
    for isl in islands:
        size = int(round(isl["tile"] / m_per_px))
        tile_h = cv2.resize(isl["h"], (size, size), interpolation=cv2.INTER_CUBIC)
        c0 = int(round((isl["centre"][0] - isl["tile"] / 2 - cx + SEA_SIZE / 2) / m_per_px))
        r0 = int(round((isl["centre"][1] - isl["tile"] / 2 - cy + SEA_SIZE / 2) / m_per_px))
        view = depth[r0:r0 + size, c0:c0 + size]
        np.minimum(view, -tile_h[:view.shape[0], :view.shape[1]], out=view)
    d = depth + 1.2 * (noise(rng, SEA_PX, 8) - 0.5)
    stops = [-0.5, 0.25, 1.0, 3.0, 6.0, 12.0]  # deep colour from 12 m on, so the island tiles have no edge
    colours = np.array([[200, 205, 185], [222, 236, 232], [96, 196, 192], [52, 160, 178],
                        [26, 100, 148], [14, 56, 108]], np.float32)
    rgb = np.dstack([np.interp(d, stops, colours[:, k]) for k in range(3)]).astype(np.float32)
    # Faint swell and ripple: real open water gives a camera very little to hold on to
    rgb *= (0.97 + 0.06 * noise(rng, SEA_PX, 10, 0.6))[..., None]
    rgb += (rng.random((SEA_PX, SEA_PX, 1), dtype=np.float32) - 0.5) * 4
    return np.clip(rgb, 0, 255).astype(np.uint8)[::-1], (cx, cy)


def box(name, x, y, z, sx, sy, sz, colour, yaw=0.0, collide=True):
    pose = f"<pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.3f}</pose>"
    geo = f"<geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>"
    out = (f'\n      <visual name="{name}">{pose}{geo}'
           f"<material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>")
    if collide:
        out += f'\n      <collision name="{name}">{pose}{geo}</collision>'
    return out


def helipad(tag, x, y):
    """8 m concrete pad, top at z = 0, with a white H and border."""
    white, out = "0.95 0.95 0.95 1", box(f"pad_{tag}", x, y, -0.25, 8, 8, 0.5, "0.45 0.45 0.47 1")
    out += box(f"pad_{tag}_h1", x - 1.2, y, 0.005, 0.5, 3.2, 0.01, white, collide=False)
    out += box(f"pad_{tag}_h2", x + 1.2, y, 0.005, 0.5, 3.2, 0.01, white, collide=False)
    out += box(f"pad_{tag}_h3", x, y, 0.005, 2.4, 0.5, 0.01, white, collide=False)
    for k, (dx, dy, sx, sy) in enumerate([(0, 3.6, 7.6, 0.3), (0, -3.6, 7.6, 0.3), (3.6, 0, 0.3, 7.6), (-3.6, 0, 0.3, 7.6)]):
        out += box(f"pad_{tag}_edge{k}", x + dx, y + dy, 0.005, sx, sy, 0.01, "0.95 0.8 0.1 1", collide=False)
    return out


def house(rng, tag, x, y, ground):
    """Low stone house with a flat or tiled roof, set into the ground. `ground` gives height above the sea."""
    sx, sy, yaw = rng.uniform(6, 11), rng.uniform(5, 8), rng.uniform(0, math.pi)
    corners = [(x + a * sx / 2 * math.cos(yaw) - b * sy / 2 * math.sin(yaw),
                y + a * sx / 2 * math.sin(yaw) + b * sy / 2 * math.cos(yaw)) for a in (-1, 1) for b in (-1, 1)]
    hs = [ground(cx, cy) - PAD_ASL for cx, cy in corners]
    base, wall = min(hs) - 1.0, max(hs) + rng.uniform(3.0, 4.5)
    walls = rng.choice(["0.86 0.82 0.74 1", "0.93 0.92 0.88 1", "0.72 0.66 0.58 1"])
    roof = rng.choice(["0.68 0.30 0.22 1", "0.62 0.60 0.58 1", "0.30 0.42 0.55 1"])
    sdf = (box(f"house_{tag}", x, y, (base + wall) / 2, sx, sy, wall - base, walls, yaw)
           + box(f"house_{tag}_roof", x, y, wall + 0.2, sx + 0.6, sy + 0.6, 0.4, roof, yaw))
    return sdf, dict(x=x, y=y, size=[sx + 0.6, sy + 0.6], yaw=yaw, roof_z=wall + 0.4)


def lighthouse(x, y, z):
    out = ""
    for k, (z0, z1, r, c) in enumerate([(-1, 4, 2.2, "0.95 0.95 0.93 1"), (4, 8, 1.9, "0.80 0.15 0.12 1"),
                                        (8, 12, 1.7, "0.95 0.95 0.93 1"), (12, 14, 1.9, "0.20 0.20 0.22 1")]):
        pose = f"<pose>{x:.2f} {y:.2f} {z + (z0 + z1) / 2:.2f} 0 0 0</pose>"
        geo = f"<geometry><cylinder><radius>{r}</radius><length>{z1 - z0}</length></cylinder></geometry>"
        out += (f'\n      <visual name="lighthouse_{k}">{pose}{geo}<material><ambient>{c}</ambient>'
                f"<diffuse>{c}</diffuse></material></visual>\n      <collision name=\"lighthouse_{k}\">{pose}{geo}</collision>")
    return out


MODEL = """<?xml version="1.0"?>
<!-- Generated by sim/scripts/make_islands.py (seed {seed}). +X east, +Y north; helipad tops at z = 0. -->
<sdf version="1.11">
  <model name="islands">
    <static>true</static>
    <link name="link">
      <visual name="sea">
        <pose>{sx:.1f} {sy:.1f} {sea_z:.2f} 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>{sea:.0f} {sea:.0f}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.3 0.3 0.3 1</specular>
          <pbr><metal>
            <albedo_map>model://islands/materials/textures/sea.jpg</albedo_map>
            <roughness>0.55</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
      <visual name="ocean">
        <pose>{sx:.1f} {sy:.1f} {ocean_z:.2f} 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>40000 40000</size></plane></geometry>
        <material><ambient>0.055 0.22 0.42 1</ambient><diffuse>0.055 0.22 0.42 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>{body}
    </link>
  </model>
</sdf>
"""

ISLAND = """
      <visual name="island_{name}">
        <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
        <geometry><mesh><uri>model://islands/meshes/island_{name}.obj</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>model://islands/materials/textures/island_{name}.jpg</albedo_map>
            <roughness>1.0</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
      <collision name="island_{name}">
        <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
        <geometry><mesh><uri>model://islands/meshes/island_{name}_collision.obj</uri></mesh></geometry>
      </collision>"""


def place(rng, isl, count, min_gap, keep_out, lo=1.8):
    """Random spots on land between `lo` m above the sea and the hilltops, on gentle slopes."""
    h, tile, (cx, cy) = isl["h"], isl["tile"], isl["centre"]
    pts, tries = [], 0
    while len(pts) < count and tries < count * 400:
        tries += 1
        x, y = cx + rng.uniform(-tile / 2, tile / 2), cy + rng.uniform(-tile / 2, tile / 2)
        e = isl["ground"](x, y)
        slope = abs(isl["ground"](x + 2, y) - isl["ground"](x - 2, y)) / 4 + abs(isl["ground"](x, y + 2) - isl["ground"](x, y - 2)) / 4
        if e < lo or slope > 0.3:
            continue
        if any(math.hypot(x - a, y - b) < r for a, b, r in keep_out):
            continue
        if pts and min(math.hypot(x - a, y - b) for a, b in pts) < min_gap:
            continue
        pts.append((x, y))
    return pts


def build(seed):
    rng = np.random.default_rng(seed)
    (OUT / "meshes").mkdir(parents=True, exist_ok=True)
    (OUT / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    body, tree_i = [], 0
    features = dict(houses=[], lighthouse=None, trees=[])
    for isl in ISLANDS:
        isl["h"], isl["tile"] = terrain(rng, isl)
        cx, cy = isl["centre"]
        isl["ground"] = lambda x, y, isl=isl: float(sample(isl["h"], isl["tile"], x - isl["centre"][0], y - isl["centre"][1])[0])
        name = isl["name"]
        write_obj(OUT / "meshes" / f"island_{name}.obj", isl["h"], isl["tile"], 1)
        write_obj(OUT / "meshes" / f"island_{name}_collision.obj", isl["h"], isl["tile"], COLLISION_STEP)
        cv2.imwrite(str(OUT / "materials" / "textures" / f"island_{name}.jpg"),
                    cv2.cvtColor(texture(rng, isl["h"], isl["tile"]), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
        body.append(ISLAND.format(name=name, x=cx, y=cy, z=-PAD_ASL))
        body.append(helipad(name, *isl["pad"]))
        keep_out = [(*isl["pad"], 16.0)]
        if isl["lighthouse"]:
            # On the highest gentle ground, as a landmark visible from the sea
            spots = place(rng, isl, 40, 5, keep_out, lo=isl["top"] * 0.4)
            lx, ly = max(spots, key=lambda p: isl["ground"](*p))
            body.append(lighthouse(lx, ly, isl["ground"](lx, ly) - PAD_ASL))
            features["lighthouse"] = dict(x=lx, y=ly, top_z=isl["ground"](lx, ly) - PAD_ASL + 14)
            keep_out.append((lx, ly, 10.0))
        homes = place(rng, isl, isl["houses"], 16, keep_out)
        for k, (x, y) in enumerate(homes):
            sdf, footprint = house(rng, f"{name}{k}", x, y, isl["ground"])
            body.append(sdf)
            features["houses"].append(footprint)
        keep_out += [(x, y, 9.0) for x, y in homes]
        for x, y in place(rng, isl, isl["trees"], 4, keep_out, lo=2.2):
            body.append(tree(rng, tree_i, x, y, isl["ground"](x, y) - PAD_ASL - 0.3))
            features["trees"].append([round(x, 2), round(y, 2)])
            tree_i += 1
    sea, (sx, sy) = sea_texture(rng, ISLANDS)
    cv2.imwrite(str(OUT / "materials" / "textures" / "sea.jpg"), cv2.cvtColor(sea, cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, 90])
    (OUT / "model.sdf").write_text(MODEL.format(seed=seed, sx=sx, sy=sy, sea_z=-PAD_ASL, ocean_z=-PAD_ASL - 0.05,
                                                sea=SEA_SIZE, body="".join(body)))
    (OUT / "model.config").write_text(
        '<?xml version="1.0"?>\n<model><name>islands</name><version>1.0</version>'
        '<sdf version="1.11">model.sdf</sdf></model>\n')
    # Height grids for maps: h[row, col] in m above the sea, row = +y, over a tile centred on the island
    np.savez_compressed(OUT / "terrain.npz", **{f"h_{i['name']}": i["h"] for i in ISLANDS},
                        **{f"tile_{i['name']}": i["tile"] for i in ISLANDS})
    (OUT / "layout.json").write_text(json.dumps({
        "sea_z": -PAD_ASL,
        "pads": {i["name"]: list(i["pad"]) for i in ISLANDS},
        "islands": {i["name"]: {"centre": list(i["centre"]), "radius": i["radius"]} for i in ISLANDS},
        "seabed_m": 14.0,  # depth away from the islands
        **features,
    }, indent=2) + "\n")
    return tree_i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--if-missing", action="store_true",
                    help="do nothing if this version of the islands already exists")
    args = ap.parse_args()
    stamp = OUT / "VERSION"
    if args.if_missing and stamp.exists() and stamp.read_text().strip() == VERSION:
        return
    trees = build(args.seed)
    stamp.write_text(VERSION + "\n")
    a, b = ISLANDS
    gap = math.dist(a["centre"], b["centre"]) - a["radius"] - b["radius"]
    print(f"islands: 2 islands, about {gap:.0f} m of open sea between them, {trees} trees, seed {args.seed} -> {OUT}")


if __name__ == "__main__":
    main()
