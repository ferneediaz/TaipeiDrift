"""Generate a compact, varied city block model for the Gazebo city world."""
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "models" / "city"


def generate(force=False):
    sdf = OUT / "model.sdf"
    config = OUT / "model.config"
    if sdf.exists() and not force:
        return
    OUT.mkdir(parents=True, exist_ok=True)
    links = []

    def box(name, x, y, z, sx, sy, sz, color):
        links.append(f'''<link name="{name}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
          <visual name="visual"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
          <collision name="collision"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry></collision></link>''')

    box("ground", 0, 0, -0.15, 500, 500, 0.3, "0.34 0.36 0.33 1")
    # Main roads and cross streets; pale narrow lines provide visible lane markings.
    for i, x in enumerate((-120, -40, 40, 120)):
        box(f"road_x_{i}", x, 0, 0.005, 16, 500, 0.01, "0.16 0.17 0.18 1")
    for i, y in enumerate((-120, -40, 40, 120)):
        box(f"road_y_{i}", 0, y, 0.005, 500, 16, 0.01, "0.16 0.17 0.18 1")
    for i, x in enumerate((-120, -40, 40, 120)):
        for j, y in enumerate((-120, -40, 40, 120)):
            box(f"crosswalk_{i}_{j}", x, y, 0.006, 20, 20, 0.012, "0.68 0.66 0.58 1")
    # Irregular blocks with varied footprints and heights, deliberately avoiding a repeated grid.
    footprints = [(25, 25), (34, 20), (20, 38), (28, 31), (42, 22), (22, 22), (36, 28)]
    colors = ["0.68 0.52 0.39 1", "0.48 0.55 0.62 1", "0.74 0.70 0.58 1", "0.54 0.60 0.48 1"]
    idx = 0
    for bx in (-80, 0, 80):
        for by in (-80, 0, 80):
            if (bx, by) in ((0, 0), (80, -80)):
                continue  # square/plaza and open park
            for ox, oy in ((-19, -17), (20, 19)):
                w, d = footprints[(idx * 3 + (idx // 3)) % len(footprints)]
                h = (18, 30, 45, 24, 58, 35, 16)[idx % 7]
                x, y = bx + ox, by + oy
                box(f"building_{idx}", x, y, h / 2 + 0.08, w, d, h, colors[idx % len(colors)])
                # Slightly smaller contrasting rooftop cap creates distinct silhouettes.
                if idx % 3 != 1:
                    cap_h = 3.0 + (idx % 3)
                    box(f"roof_{idx}", x, y, h + cap_h / 2 + 0.08, w * 0.72, d * 0.72, cap_h,
                        "0.36 0.39 0.42 1")
                idx += 1
    # Small vegetation clumps in the open park, using solid trunks and simple green crowns.
    for i, (x, y) in enumerate(((-30, 25), (-18, 36), (-7, 22), (14, 35), (28, 21), (8, 5), (-22, 5))):
        box(f"tree_trunk_{i}", x, y, 3.2, 1.2, 1.2, 6.4, "0.30 0.20 0.10 1")
        # Square foliage blocks are lightweight and still provide strong image features.
        box(f"tree_crown_{i}", x, y, 7.0, 7.0, 7.0, 4.0, "0.16 0.38 0.18 1")
    sdf.write_text('<?xml version="1.0"?><sdf version="1.9"><model name="city"><static>true</static>' +
                   ''.join(links) + '</model></sdf>')
    config.write_text('''<?xml version="1.0"?><model><name>TaipeiDrift compact city</name>
      <version>1.0</version><sdf version="1.9">model.sdf</sdf><description>Generated city navigation environment</description></model>''')


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--if-missing", action="store_true")
    p.add_argument("--force", action="store_true")
    a = p.parse_args()
    generate(force=a.force or not a.if_missing)
