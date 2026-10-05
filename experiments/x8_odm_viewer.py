#!/usr/bin/env python3
"""Look around the OpenDroneMap 3D model of the Tuniu reach (survey flight 2019-09-16).

Default model: OpenDroneMap's own textured 2.5D mesh (stage mvs_texturing):
  data/processed/x_tuniu_survey_odm/odm_texturing_25d/odm_textured_model_geo.{obj,mtl} + 30 textures 8192 px.
Other model, same OpenDroneMap layout: `--src <project>/odm_texturing`, e.g. the full 3D high-quality mesh
data/processed/x_tuniu_survey_odm_server/tuniu/odm_texturing (5.8 M faces, 725 textures 8192 px: use --tex-px 1024).
Nothing is rebuilt here. `prepare` only makes a display copy (in <project>/<--dir>) whose textures are downscaled
(default 2048 px), because 8192^2 textures do not fit comfortably in memory; the geometry and texture coordinates
are unchanged.

The April 2019 test flight (RTK) is drawn on top as a white line through the camera positions.

    uv run --with pyvista python experiments/x8_odm_viewer.py prepare
    uv run --with pyvista python experiments/x8_odm_viewer.py view            # interactive window
    uv run --with pyvista python experiments/x8_odm_viewer.py view --shot out.png --off-screen
    S=data/processed/x_tuniu_survey_odm_server/tuniu/odm_texturing            # full 3D model
    uv run --with pyvista python experiments/x8_odm_viewer.py prepare --src $S --dir viewer_1024 --tex-px 1024
    uv run --with pyvista python experiments/x8_odm_viewer.py view --src $S --dir viewer_1024

Mouse in the window: left drag = rotate around the focus, right drag or wheel = zoom, middle drag / shift +
left = pan; key `f` with the mouse over a point = fly to it; `r` = reset view; `q` = quit.
Textures come from Yu-Huang Wang's photos (licence unknown): keep screenshots private.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image
from pyproj import Transformer

ROOT = Path(__file__).resolve().parents[1]
ODM = ROOT / "data/processed/x_tuniu_survey_odm"
SRC = ODM / "odm_texturing_25d"
OBJ = "odm_textured_model_geo.obj"
MTL = "odm_textured_model_geo.mtl"
OUT = ODM / "viewer"          # set from --dir in main(): one folder per texture size
TRUTH = ROOT / "data/processed/t_replay/tuniu_tw_1/truth.csv"
ATT = ROOT / "data/processed/t_replay/tuniu_tw_1/attitude.csv"


def offset() -> tuple[float, float]:
    lines = (ODM / "odm_georeferencing/coords.txt").read_text().splitlines()
    if lines[0].strip() != "WGS84 UTM 51N":
        raise ValueError(f"unexpected CRS line in coords.txt: {lines[0]!r}")
    e, n = map(float, lines[1].split())
    return e, n


def cmd_prepare(args) -> None:
    Image.MAX_IMAGE_PIXELS = None
    OUT.mkdir(parents=True, exist_ok=True)
    mtl_out = []
    for line in (SRC / MTL).read_text().splitlines():
        if line.startswith("map_Kd "):
            src = SRC / line.split(maxsplit=1)[1].strip()
            dst = OUT / (src.stem + f"_{args.tex_px}.jpg")
            if not dst.exists():
                with Image.open(src) as im:
                    im.convert("RGB").resize((args.tex_px, args.tex_px), Image.Resampling.LANCZOS).save(dst, quality=90)
                print("texture", dst.name)
            line = f"map_Kd {dst.name}"
        mtl_out.append(line)
    (OUT / MTL).write_text("\n".join(mtl_out) + "\n")
    if not (OUT / OBJ).exists():
        shutil.copyfile(SRC / OBJ, OUT / OBJ)
    e0, n0 = offset()
    tr, att = pd.read_csv(TRUTH), pd.read_csv(ATT)
    x, y = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True).transform(tr.lon_deg.values, tr.lat_deg.values)
    out = pd.DataFrame({"x": x - e0, "y": y - n0, "z": tr.alt_m.values,
                        "yaw_deg": np.interp(tr.t_s, att.t_s, att.gimbal_yaw_deg),
                        "pitch_deg": np.interp(tr.t_s, att.t_s, att.gimbal_pitch_deg)})
    out.to_csv(OUT / "april_flight_mesh_frame.csv", index=False)
    print("ready:", OUT)


def cmd_view(args) -> None:
    import pyvista as pv
    if not (OUT / MTL).exists():
        raise SystemExit("run `prepare` first")
    pl = pv.Plotter(off_screen=args.off_screen, window_size=(1600, 1000))
    pl.import_obj(str(OUT / OBJ), str(OUT / MTL))
    actors = pl.renderer.GetActors()   # VTK collection: also holds the actors made by the OBJ importer
    actors.InitTraversal()
    for _ in range(actors.GetNumberOfItems()):   # photo colours as they are: no shading, MTL ambient (Ka 1) off
        prop = actors.GetNextActor().GetProperty()
        prop.LightingOff()
        prop.SetAmbient(0.0)
        prop.SetDiffuse(1.0)
        prop.SetSpecular(0.0)
    pl.set_background("#c9dcef", top="#4f7fb5")   # sky gradient
    pl.enable_anti_aliasing("ssaa")
    fl = pd.read_csv(OUT / "april_flight_mesh_frame.csv")
    cams = fl[["x", "y", "z"]].to_numpy()
    if args.no_flight:
        _finish(pl, cams, args)
        return
    pl.add_mesh(pv.lines_from_points(cams), color="white", line_width=3, label="April 2019 flight (RTK)")
    pl.add_text(f"Tuniu River, OpenDroneMap 3D (survey 2019-09-16): {SRC.parent.name}/{SRC.name}\n"
                "white line = April 2019 flight, camera positions from the drone's RTK log\n"
                "left drag: rotate   wheel: zoom   shift+drag: pan   f: fly to point   r: reset   q: quit",
                font_size=10, color="white")
    _finish(pl, cams, args)


def _finish(pl, cams, args) -> None:
    pl.enable_terrain_style(mouse_wheel_zooms=True)
    c = cams.mean(0)
    pl.camera_position = [(c[0] - 250, c[1] - 450, c[2] + 250), (c[0], c[1], c[2] - 90), (0, 0, 1)]
    if args.shot:
        pl.show(screenshot=args.shot, auto_close=True)
        print("screenshot", args.shot)
    else:
        pl.show()


def main() -> None:
    global SRC, ODM, OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("prepare", "view"):
        s = sub.add_parser(name)
        s.add_argument("--src", default=str(SRC), help="OpenDroneMap texturing folder (OBJ + MTL + textures)")
        s.add_argument("--dir", default="viewer",
                       help="display folder, next to the --src folder (e.g. viewer_4096)")
        if name == "prepare":
            s.add_argument("--tex-px", type=int, default=2048)
        else:
            s.add_argument("--shot", help="save a screenshot to this path")
            s.add_argument("--off-screen", action="store_true")
            s.add_argument("--no-flight", action="store_true", help="hide the April flight overlay")
    a = ap.parse_args()
    SRC = Path(a.src).resolve()
    ODM = SRC.parent
    OUT = ODM / a.dir
    {"prepare": cmd_prepare, "view": cmd_view}[a.cmd](a)


if __name__ == "__main__":
    main()
