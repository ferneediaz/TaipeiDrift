#!/usr/bin/env python3
"""Digital twin of the April 2019 Tuniu flight: a virtual camera inside OpenDroneMap's 3D model of the reach.

World: OpenDroneMap's own textured 2.5D mesh of the 2019-09-16 survey (odm_texturing_25d). `prepare` makes
texture copies at 4096 px (Lanczos) and an MTL pointing to them; the OBJ is a symlink to OpenDroneMap's file
(geometry and texture coordinates never rebuilt or edited). Mesh frame = UTM 51N minus the offset in
odm_georeferencing/coords.txt, z = ellipsoidal height. Placement: the camera is moved by a constant offset
(twin_calibration.json, estimated by `placement` on PRE-CUT photos only) that absorbs the mesh georeferencing
error (~1.9 m).

Camera: for each photo pose (RTK position + DJI gimbal yaw/pitch/roll + pre-cut boresight, stage1_calibration)
an ideal pinhole with the real intrinsics scaled to the variant size, principal point honoured through an
explicit OpenGL projection matrix (fx and fy kept separately), no distortion, no lighting (texture colours as
they are), trilinear mipmapped textures. Pixels where no mesh is seen are black and marked invalid in masks/*.png.

Variants (one replay folder each, data/processed/t_replay/tuniu_tw_1_twin[_<name>]):
  twin      real poses, quarter size 1368x912 (rendered at 2736x1824 and area-averaged)
  twin_att  diagnostic: + per-leg gimbal errors fitted to the real photos (`attitude`); not used by the sweep
  half      half size 2736x1824
  tilt0 / tilt15 / tilt45   camera tilt from nadir 0/15/45 deg instead of the real 30 deg (gimbal pitch -60)
  dt1 / dt05                photo every 1 s / 0.5 s (RTK track and attitude interpolated) instead of 2.8 s
  alt50                     50 m above the real track
  + suffix _rc: the same images through the realistic camera (degrade)

Commands (uv run --with pyvista python experiments/x9_tuniu_twin.py <cmd>):
  prepare           4096 px texture copies + MTL (data/processed/x9_tuniu_twin/mesh_4096)
  texel             texture ground sampling of the mesh (why 4096 px textures)
  projtest          checks the projection: markers at known 3D points land on the predicted pixels
  render <variant>  writes the twin replay folder (resumes: existing frames are kept)
  fidelity          real vs twin on the rule-fixed sample (every 20th photo + 47, 100, 185), or --all
  placement         constant camera offset from the pre-cut fidelity of an unplaced render
  attitude          per-leg gimbal errors from the fidelity of the placed twin (twin_att diagnostic)
  degrade <variant> realistic camera (TaipeiDrift-sim camera_model.py, adapted) -> <variant>_rc
  quality           sharpness / contrast / noise of real vs twin vs twin_rc rectified patches

Rendered images stay in data/processed (photo licence unknown).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from pyproj import Transformer  # noqa: E402

import x_tuniu_geo as G  # noqa: E402

ROOT = G.ROOT
ODM = ROOT / "data/processed/x_tuniu_survey_odm"
SRC = ODM / "odm_texturing_25d"                  # OpenDroneMap output, read only
OBJ, MTL = "odm_textured_model_geo.obj", "odm_textured_model_geo.mtl"
REAL = ROOT / "data/processed/t_replay/tuniu_tw_1"
WORK = ROOT / "data/processed/x9_tuniu_twin"
TEX_PX = 4096                                    # see `texel`: median texel 7 cm, 90 % of the surface <= 15 cm
MESH = WORK / f"mesh_{TEX_PX}"                   # MTL pointing to downscaled texture copies; OBJ = symlink to SRC
CAL = json.loads((G.XOUT / "stage1_calibration.json").read_text())
FULL_W, FULL_H = 5472, 3648
NEAR_M, FAR_M = 1.0, 5000.0
SAMPLE = sorted(set(range(1, 272, 20)) | {47, 100, 185})

# att: render with gimbal errors fitted to the real photos (attitude_errors.csv, see `attitude`); the navigation
# side always gets the nominal DJI angles. Diagnostic only: those errors (mostly roll / yaw) raise the twin's
# pre-cut odometry drift from 0.89 m (ideal twin; real photos 0.87 m) to 1.5 m, i.e. they are mostly mesh
# geometry, not camera attitude. Sweep variants therefore use the ideal attitude, like `twin`.
VARIANTS = {
    "twin": dict(att=False), "twin_att": dict(att=True),
    "half": dict(scale=2),
    "tilt0": dict(tilt=0.0), "tilt15": dict(tilt=15.0), "tilt45": dict(tilt=45.0),
    "dt1": dict(interval=1.0), "dt05": dict(interval=0.5),
    "alt50": dict(dalt=50.0),
}
# Effect of a 1 deg gimbal error on the real-vs-twin alignment of a rectified 0.5 m patch (measured by
# perturbing renders by 0.5 deg at photos 30, 141, 200, 265: pitch +1 deg -> +2.16 m along track; roll +1 deg ->
# -1.0 m cross track (to the right) and -1.0 deg rotation; yaw +1 deg -> -1.0 deg rotation)
SENS_ALONG_PER_PITCH = 2.16
SENS_CROSS_PER_ROLL = -1.0
SENS_ROT_PER_ROLL = -1.0
SENS_ROT_PER_YAW = -1.0


def replay_dir(name: str) -> Path:
    """twin -> tuniu_tw_1_twin, twin_rc -> tuniu_tw_1_twin_rc, tilt0 -> tuniu_tw_1_twin_tilt0, ..."""
    return ROOT / "data/processed/t_replay" / (f"tuniu_tw_1_{name}" if name.startswith("twin") else f"tuniu_tw_1_twin_{name}")


def spec_of(name: str) -> dict:
    base = name[:-3] if name.endswith("_rc") else name
    return dict(scale=4, tilt=None, interval=None, dalt=0.0, att=False) | VARIANTS[base]


def render_attitude(spec: dict, P: pd.DataFrame, A: pd.DataFrame) -> pd.DataFrame:
    """Attitude used to RENDER: nominal (= what the navigation reads) + the real flight's errors if att."""
    R = A.copy()
    if spec["att"]:
        e = pd.read_csv(WORK / "attitude_errors.csv")
        for k in ("pitch", "roll", "yaw"):
            R[f"gimbal_{k}_deg"] = R[f"gimbal_{k}_deg"] + np.interp(P.t_s, e.t_s, e[f"d_{k}_deg"])
    return R


# ----------------------------------------------------------------------------- frames

def mesh_offset() -> tuple[float, float]:
    lines = (ODM / "odm_georeferencing/coords.txt").read_text().splitlines()
    if lines[0].strip() != "WGS84 UTM 51N":
        raise ValueError(lines[0])
    e, n = map(float, lines[1].split())
    return e, n


_TO_UTM = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)


def to_mesh(lon, lat, alt) -> np.ndarray:
    e0, n0 = mesh_offset()
    x, y = _TO_UTM.transform(np.asarray(lon, float), np.asarray(lat, float))
    return np.stack([np.asarray(x) - e0, np.asarray(y) - n0, np.asarray(alt, float)], -1)


def enu_to_mesh_rot(lon: float, lat: float) -> np.ndarray:
    """3x3 rotation taking true east/north/up vectors to mesh (UTM 51N grid) axes: UTM grid convergence."""
    a = to_mesh([lon, lon], [lat, lat + 1e-3], [0, 0])
    d = a[1, :2] - a[0, :2]
    g = math.atan2(d[0], d[1])          # grid azimuth of true north (rad, clockwise from grid north)
    c, s = math.cos(g), math.sin(g)
    # true east (1,0) -> grid (cos g, -sin g); true north (0,1) -> grid (sin g, cos g)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.]])


def poses(spec: dict) -> pd.DataFrame:
    """Per output frame: time, RTK lat/lon/alt (interpolated for interval variants), DJI attitude columns."""
    tr = pd.read_csv(REAL / "truth.csv")
    att = pd.read_csv(REAL / "attitude.csv")
    t = tr.t_s.to_numpy()
    if spec["interval"]:
        tt = np.arange(0.0, t[-1] + 1e-9, spec["interval"])
        x, y = G.TO_3826.transform(tr.lon_deg.to_numpy(), tr.lat_deg.to_numpy())
        xi, yi = np.interp(tt, t, x), np.interp(tt, t, y)
        lon, lat = G.TO_LONLAT.transform(xi, yi)
        out = pd.DataFrame({"t_s": tt, "lat_deg": lat, "lon_deg": lon})
        for c in ("e_m", "n_m", "u_m", "alt_m"):
            out[c] = np.interp(tt, t, tr[c].to_numpy())
        a = pd.DataFrame({"t_s": tt, "frame": np.arange(1, len(tt) + 1)})
        for c in att.columns:
            if c in ("t_s", "frame"):
                continue
            v = att[c].to_numpy(float)
            if "yaw" in c:
                a[c] = (np.interp(tt, t, np.degrees(np.unwrap(np.radians(v)))) + 180.0) % 360.0 - 180.0
            else:
                a[c] = np.interp(tt, t, v)
        src_frame = np.clip(np.searchsorted(t, tt, side="right"), 1, len(t))   # real photo at or before
    else:
        out = tr[["t_s", "lat_deg", "lon_deg", "e_m", "n_m", "u_m", "alt_m"]].copy()
        a = att.copy()
        src_frame = np.arange(1, len(t) + 1)
    if spec["tilt"] is not None:
        a["gimbal_pitch_deg"] = -(90.0 - spec["tilt"])
    out["alt_m"] = out["alt_m"] + spec["dalt"]
    out["u_m"] = out["u_m"] + spec["dalt"]
    out["frame"] = np.arange(1, len(out) + 1)
    out["src_frame"] = src_frame
    return out.reset_index(drop=True), a.reset_index(drop=True)


def camera_dict(scale: float) -> dict:
    cam = json.loads((REAL / "meta.json").read_text())["camera"]
    return dict(model=cam["model"] + f" (ideal pinhole, 1/{scale:g} size, twin)",
                image_width_px=int(round(FULL_W / scale)), image_height_px=int(round(FULL_H / scale)),
                fx_px=cam["fx_px"] / scale, fy_px=cam["fy_px"] / scale,
                cx_px=(cam["cx_px"] + 0.5) / scale - 0.5, cy_px=(cam["cy_px"] + 0.5) / scale - 0.5,
                distortion_k1_k2_p1_p2_k3=[0.0] * 5,
                distortion_model="none (ideal pinhole rendered by VTK)")


def cam_rotation(att_row, spec) -> np.ndarray:
    bs = CAL["boresight_deg"]
    return G.rot_enu_cam(att_row.gimbal_yaw_deg + bs["yaw"], att_row.gimbal_pitch_deg + bs["pitch"],
                         att_row.gimbal_roll_deg + bs["roll"])


# ----------------------------------------------------------------------------- VTK

def make_plotter(w: int, h: int):
    import pyvista as pv
    if not (MESH / MTL).exists():
        raise SystemExit("run `uv run --with pyvista python experiments/x9_tuniu_twin.py prepare` first")
    pl = pv.Plotter(off_screen=True, window_size=(w, h))
    pl.import_obj(str(MESH / OBJ), str(MESH / MTL))
    actors = pl.renderer.GetActors()
    actors.InitTraversal()
    for _ in range(actors.GetNumberOfItems()):
        act = actors.GetNextActor()
        prop = act.GetProperty()
        prop.LightingOff()           # photo colours as they are (see x8_odm_viewer.py)
        prop.SetAmbient(0.0)
        prop.SetDiffuse(1.0)
        prop.SetSpecular(0.0)
        tex = act.GetTexture()
        if tex is not None:          # trilinear mipmaps: minified texels are averaged, not aliased
            tex.InterpolateOn()
            tex.MipmapOn()
            tex.SetMaximumAnisotropicFiltering(16.0)
    pl.set_background("black")
    return pl


def set_camera(pl, C: np.ndarray, R_mesh: np.ndarray, K: np.ndarray, w: int, h: int) -> None:
    """C camera centre (mesh frame); R_mesh columns = OpenCV camera x, y, z axes in mesh frame; K for w x h."""
    import vtk
    cam = pl.renderer.GetActiveCamera()
    cam.SetPosition(*C)
    cam.SetFocalPoint(*(C + 100.0 * R_mesh[:, 2]))
    cam.SetViewUp(*(-R_mesh[:, 1]))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    n, f = NEAR_M, FAR_M
    P = np.array([[2 * fx / w, 0, -(2 * (cx + 0.5) / w - 1), 0],
                  [0, 2 * fy / h, -(1 - 2 * (cy + 0.5) / h), 0],
                  [0, 0, -(f + n) / (f - n), -2 * f * n / (f - n)],
                  [0, 0, -1, 0]])
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, float(P[i, j]))
    cam.SetExplicitProjectionTransformMatrix(m)
    cam.SetUseExplicitProjectionTransformMatrix(True)
    cam.SetClippingRange(n, f)
    pl.renderer.camera_set = True    # otherwise pyvista resets (moves) the camera on the first render


def grab(pl) -> np.ndarray:
    pl.render()
    return np.asarray(pl.screenshot(transparent_background=True, return_img=True))


def K_of(cam: dict) -> np.ndarray:
    return np.array([[cam["fx_px"], 0, cam["cx_px"]], [0, cam["fy_px"], cam["cy_px"]], [0, 0, 1.]])


# ----------------------------------------------------------------------------- commands

def cmd_prepare(args) -> None:
    """Texture copies at TEX_PX (Lanczos) + MTL pointing to them; the OBJ is a symlink to OpenDroneMap's file."""
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    MESH.mkdir(parents=True, exist_ok=True)
    lines = []
    for line in (SRC / MTL).read_text().splitlines():
        if line.startswith("map_Kd "):
            src = SRC / line.split(maxsplit=1)[1].strip()
            dst = MESH / f"{src.stem}_{TEX_PX}.jpg"
            if not dst.exists():
                with Image.open(src) as im:
                    im.convert("RGB").resize((TEX_PX, TEX_PX), Image.Resampling.LANCZOS).save(dst, quality=92)
                print("texture", dst.name, flush=True)
            line = f"map_Kd {dst.name}"
        lines.append(line)
    (MESH / MTL).write_text("\n".join(lines) + "\n")
    if not (MESH / OBJ).exists():
        (MESH / OBJ).symlink_to(SRC / OBJ)
    print("ready:", MESH)


def cmd_texel(args) -> None:
    """Ground size of one texel per face (area-weighted quantiles) for 2048 / 4096 / 8192 px textures,
    from the OBJ texture coordinates."""
    V, VT, faces, mats = [], [], [], []
    cur = -1
    names = {}
    with open(SRC / OBJ) as fh:
        for line in fh:
            if line.startswith("v "):
                V.append([float(v) for v in line.split()[1:4]])
            elif line.startswith("vt "):
                VT.append([float(v) for v in line.split()[1:3]])
            elif line.startswith("usemtl"):
                cur = names.setdefault(line.split()[1], len(names))
            elif line.startswith("f "):
                p = [s.split("/") for s in line.split()[1:4]]
                faces.append([int(a[0]) - 1 for a in p] + [int(a[1]) - 1 for a in p])
                mats.append(cur)
    V, VT, F = np.array(V), np.array(VT), np.array(faces)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    area3 = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    ta, tb, tc = VT[F[:, 3]], VT[F[:, 4]], VT[F[:, 5]]
    uv = 0.5 * np.abs((tb[:, 0] - ta[:, 0]) * (tc[:, 1] - ta[:, 1]) - (tc[:, 0] - ta[:, 0]) * (tb[:, 1] - ta[:, 1]))
    ok = (uv > 0) & (area3 > 0)
    unit = np.sqrt(area3[ok] / uv[ok])        # metres per texture-coordinate unit; texel = unit / tex_px
    w = area3[ok]
    order = np.argsort(unit)
    cw = np.cumsum(w[order]) / w.sum()
    q = {p: float(unit[order][np.searchsorted(cw, p)]) for p in (0.1, 0.5, 0.9, 0.99)}
    share = {px: {f"le_{lim}m": float(w[unit / px <= lim].sum() / w.sum()) for lim in (0.15, 0.25, 0.5)}
             for px in (2048, 4096, 8192)}
    # flight footprint check: mesh extent vs the April track
    tr = pd.read_csv(REAL / "truth.csv")
    cams = to_mesh(tr.lon_deg, tr.lat_deg, tr.alt_m)
    out = dict(tex_px_used=TEX_PX, faces=int(len(F)), materials=len(names), surface_m2=float(area3.sum()),
               texel_m_quantiles={px: {f"p{int(p * 100)}": v / px for p, v in q.items()} for px in (2048, 4096, 8192)},
               surface_share_with_texel=share,
               mesh_extent=dict(x=[float(V[:, 0].min()), float(V[:, 0].max())],
                                y=[float(V[:, 1].min()), float(V[:, 1].max())],
                                z=[float(V[:, 2].min()), float(V[:, 2].max())]),
               flight_extent=dict(x=[float(cams[:, 0].min()), float(cams[:, 0].max())],
                                  y=[float(cams[:, 1].min()), float(cams[:, 1].max())]))
    WORK.mkdir(parents=True, exist_ok=True)
    (WORK / "texel.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


def cmd_projtest(args) -> None:
    """Markers (small spheres) at known mesh-frame points ahead of a real pose must land on K-projected pixels."""
    import pyvista as pv
    spec = spec_of("twin")
    P, A = poses(spec)
    cam = camera_dict(2)
    w, h = cam["image_width_px"], cam["image_height_px"]
    K = K_of(cam)
    pl = pv.Plotter(off_screen=True, window_size=(w, h))
    pl.set_background("black")
    i = 99
    C = to_mesh(P.lon_deg[i], P.lat_deg[i], P.alt_m[i])
    Rm = enu_to_mesh_rot(P.lon_deg[i], P.lat_deg[i]) @ cam_rotation(A.iloc[i], spec)
    targets = np.array([[300, 200], [2400, 300], [1368, 912], [500, 1600], [2500, 1700]], float)
    pts = []
    for u, v in targets:
        ray = np.linalg.solve(K, [u, v, 1.0])
        pts.append(C + Rm @ (ray * 120.0))
    for p_ in pts:
        pl.add_mesh(pv.Sphere(radius=0.6, center=p_), color="white", lighting=False)
    set_camera(pl, C, Rm, K, w, h)
    img = grab(pl)
    lab_n, lab = cv2.connectedComponents((img[..., 3] > 0).astype(np.uint8))
    found = []
    for k in range(1, lab_n):
        yy, xx = np.nonzero(lab == k)
        found.append((xx.mean(), yy.mean()))
    found = np.array(found)
    err = [float(np.min(np.hypot(*(found - t).T))) for t in targets]
    print("targets", targets.tolist())
    print("found", np.round(found, 2).tolist())
    print("pixel error per marker", np.round(err, 3).tolist())
    (WORK / "projtest.json").write_text(json.dumps(dict(size=[w, h], pixel_err=err), indent=1))


def cmd_render(args) -> None:
    name = args.variant
    spec = spec_of(name)
    P, A = poses(spec)
    out = replay_dir(name)
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "masks").mkdir(parents=True, exist_ok=True)
    scale = spec["scale"]
    ss = 2 if scale >= 4 else 1                       # supersampling (render size never above 2736 px)
    cam = camera_dict(scale)
    w, h = cam["image_width_px"], cam["image_height_px"]
    camr = camera_dict(scale / ss)
    Kr = K_of(camr)
    W, H = camr["image_width_px"], camr["image_height_px"]
    off = np.array(CAL_TWIN.get("camera_offset_en_m", [0.0, 0.0]) + [0.0]) if args.placement else np.zeros(3)
    RA = render_attitude(spec, P, A)
    frames = range(len(P)) if not args.frames else [f - 1 for f in args.frames]
    pl = None
    t0 = time.time()
    n_done = 0
    for i in frames:
        jpg = out / "images" / f"frame_{i + 1:04d}.jpg"
        if jpg.exists() and not args.force:
            continue
        if pl is None:
            pl = make_plotter(W, H)
        Renu = enu_to_mesh_rot(P.lon_deg[i], P.lat_deg[i])
        C = to_mesh(P.lon_deg[i], P.lat_deg[i], P.alt_m[i]) + Renu @ off
        Rm = Renu @ cam_rotation(RA.iloc[i], spec)
        set_camera(pl, C, Rm, Kr, W, H)
        rgba = grab(pl)
        if ss > 1:
            rgba = cv2.resize(rgba, (w, h), interpolation=cv2.INTER_AREA)
        valid = rgba[..., 3] == 255
        bgr = cv2.cvtColor(np.ascontiguousarray(rgba[..., :3]), cv2.COLOR_RGB2BGR)
        bgr[~valid] = 0
        cv2.imwrite(str(jpg), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        cv2.imwrite(str(out / "masks" / f"frame_{i + 1:04d}.png"), valid.astype(np.uint8) * 255)
        n_done += 1
        if n_done % 25 == 0:
            print(f"  {name}: {n_done} frames, {time.time() - t0:.0f} s", flush=True)
    write_replay_tables(name, spec, P, A, cam, off)
    print(f"{name}: rendered {n_done} frames in {time.time() - t0:.0f} s -> {out}")


def write_replay_tables(name, spec, P, A, cam, off) -> None:
    out = replay_dir(name)
    n = len(P)
    paths = [f"images/frame_{i:04d}.jpg" for i in range(1, n + 1)]
    pd.DataFrame({"t_s": P.t_s, "cam": "twin", "path": paths}).to_csv(out / "images.csv", index=False)
    A.assign(t_s=P.t_s, frame=P.frame).to_csv(out / "attitude.csv", index=False)
    tr = P[["t_s", "e_m", "n_m", "u_m", "lat_deg", "lon_deg", "alt_m"]].copy()
    for c in ("qw", "qx", "qy", "qz"):
        tr[c] = np.nan
    tr.to_csv(out / "truth.csv", index=False)
    meta = json.loads((REAL / "meta.json").read_text())
    cut = meta["gnss_cut_s"]
    tr[tr.t_s <= cut][["t_s", "lat_deg", "lon_deg", "alt_m"]].to_csv(out / "gnss.csv", index=False)
    meta.update(sequence=out.name, evidence_label="SIMULATED (rendered from the OpenDroneMap 2019-09-16 mesh)",
                source=f"x9_tuniu_twin.py render {name}: virtual camera at the RTK poses of tuniu_tw_1",
                camera=cam, licence="Renders of Yu-Huang Wang's survey photos (texture), licence unknown: local only")
    meta["sensors"]["camera"]["rate_hz"] = float(1.0 / np.median(np.diff(P.t_s)))
    meta["twin"] = dict(variant=name, spec=spec, mesh=str((SRC / OBJ).relative_to(ROOT)), texture_px=TEX_PX,
                        camera_offset_enu_m=[float(v) for v in off], boresight_deg=CAL["boresight_deg"],
                        attitude_errors="attitude_errors.csv added to the render attitude" if spec["att"] else "none",
                        invalid="masks/*.png: 0 = no mesh seen (image pixel set to black)")
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    prot = json.loads((G.XOUT / "stage0_protocol.json").read_text())
    legs = np.array(prot["leg_per_frame"])
    prot.update(precut_frames=[int(f) for f in P.frame[P.t_s < cut]],
                test_frames=[int(f) for f in P.frame[P.t_s > cut]],
                leg_per_frame=[int(legs[s - 1]) for s in P.src_frame],
                note=prot["note"] + f"; twin {name}: frames split at the same cut time")
    (out / "twin_protocol.json").write_text(json.dumps(prot, indent=1))


# ----------------------------------------------------------------------------- calibration of the twin

def good_rows(d: pd.DataFrame) -> pd.Series:
    return (d.zncc_best >= 0.6) & ~d.edge.astype(bool)


def cmd_placement(args) -> None:
    """Mesh placement offset from PRE-CUT photos only: mean of the per-leg median twin-vs-real shift over the
    two pre-cut legs (one flown east, one west, so along-track attitude effects cancel). The render camera
    is moved by minus that shift (moving the camera 2 m east raises the measured shift by 1.95 m)."""
    d = pd.read_csv(WORK / "fidelity" / args.source / "all.csv")
    prot = json.loads((G.XOUT / "stage0_protocol.json").read_text())
    d["leg"] = np.array(prot["leg_per_frame"])[d.frame - 1]
    pre = d[d.frame.isin(prot["precut_frames"]) & d.leg.isin([0, 1]) & good_rows(d)]
    legs = pre.groupby("leg")[["dx_m", "dy_m"]].median()
    dx, dy = legs.mean()
    CAL_TWIN.update(camera_offset_en_m=[float(-dx), float(-dy)],
                    placement=dict(source=f"fidelity/{args.source}/all.csv", frames="pre-cut legs 0 and 1",
                                   n=len(pre), per_leg=legs.round(3).to_dict(orient="index")))
    CAL_TWIN_PATH.write_text(json.dumps(CAL_TWIN, indent=1))
    print(json.dumps(CAL_TWIN, indent=1))


def cmd_attitude(args) -> None:
    """Per-photo gimbal errors of the REAL flight relative to the nominal pose (DJI + boresight), from the
    twin-vs-real alignment of the placed twin: along-track shift -> pitch, cross-track shift -> roll, the
    remaining rotation -> yaw (sensitivities SENS_*). Photos with a weak alignment (ZNCC < 0.6 or search
    edge) are ignored. Model (--smooth leg, default): one constant error per straight leg (median), linear in
    time through the turns between legs: slowly varying gimbal biases. The ideal twin's pre-cut odometry
    drift already equals the real one (0.89 vs 0.87 m), so per-photo jitter would be alignment noise.
    --smooth N: N-photo running median instead. Used only to RENDER the twin_att variants."""
    d = pd.read_csv(WORK / "fidelity" / args.source / "all.csv").sort_values("frame").reset_index(drop=True)
    att = pd.read_csv(REAL / "attitude.csv")
    psi = np.radians(att.gimbal_yaw_deg.to_numpy()[d.frame - 1])
    along = d.dx_m * np.sin(psi) + d.dy_m * np.cos(psi)
    cross = d.dx_m * np.cos(psi) - d.dy_m * np.sin(psi)
    raw = pd.DataFrame({"frame": d.frame, "along_m": along, "cross_m": cross, "rot_deg": d.rot_deg})
    raw["d_pitch_deg"] = -along / SENS_ALONG_PER_PITCH
    raw["d_roll_deg"] = -cross / SENS_CROSS_PER_ROLL
    raw["d_yaw_deg"] = -(raw.rot_deg + SENS_ROT_PER_ROLL * raw.d_roll_deg) / SENS_ROT_PER_YAW
    ok = good_rows(d).to_numpy()
    out = pd.DataFrame({"t_s": att.t_s.to_numpy()[d.frame - 1], "frame": d.frame, "aligned": ok})
    leg = np.array(json.loads((G.XOUT / "stage0_protocol.json").read_text())["leg_per_frame"])[d.frame - 1]
    out["leg"] = leg
    for k in ("pitch", "roll", "yaw"):
        s = raw[f"d_{k}_deg"].where(ok)
        if args.smooth == "leg":
            med = s.groupby(leg).median()
            s = pd.Series(np.where(leg >= 0, med.reindex(leg).to_numpy(), np.nan))
        else:
            s = s.rolling(int(args.smooth), center=True, min_periods=3).median()
        out[f"d_{k}_deg"] = s.interpolate(limit_direction="both").to_numpy()
        out[f"raw_{k}_deg"] = raw[f"d_{k}_deg"]
    out.to_csv(WORK / "attitude_errors.csv", index=False)
    s = out[["d_pitch_deg", "d_roll_deg", "d_yaw_deg"]].describe().round(3)
    print(s.to_string())
    CAL_TWIN["attitude_errors"] = dict(source=f"fidelity/{args.source}/all.csv", aligned=int(ok.sum()), smooth=args.smooth,
                                       sd_deg={k: float(out[f"d_{k}_deg"].std()) for k in ("pitch", "roll", "yaw")},
                                       median_deg={k: float(out[f"d_{k}_deg"].median()) for k in ("pitch", "roll", "yaw")})
    CAL_TWIN_PATH.write_text(json.dumps(CAL_TWIN, indent=1))


# ----------------------------------------------------------------------------- fidelity

CAL_TWIN_PATH = WORK / "twin_calibration.json"
CAL_TWIN = json.loads(CAL_TWIN_PATH.read_text()) if CAL_TWIN_PATH.exists() else {}


def twin_gray(path: str, factor: int, mask_path: str | None = None):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    mpath = mask_path or path.replace("/images/", "/masks/").replace(".jpg", ".png")
    m = cv2.imread(mpath, cv2.IMREAD_GRAYSCALE)
    if factor > 1:
        size = (img.shape[1] // factor, img.shape[0] // factor)
        img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
        m = cv2.resize(m, size, interpolation=cv2.INTER_AREA)
    return img, m


def ground_fn(xy):
    z0 = float(G.dem_ellipsoidal(xy[0], xy[1]))

    def g(X, Y):
        return G.dem_ellipsoidal(xy[0] + X, xy[1] + Y) - z0
    return z0, g


def rectify_pair(f: int, twin_dir: Path, res: float = 0.5, real_factor: int = 8):
    """Real photo and twin render of frame f rectified on the same 0.5 m grid (RTK pose, DJI attitude +
    boresight, Copernicus terrain lifted, as the dem_lifted pipeline). Returns aligned patches + valid."""
    tr = G.truth_xy().set_index("frame")
    att = pd.read_csv(REAL / "attitude.csv").iloc[f - 1]
    bs = CAL["boresight_deg"]
    R = G.rot_enu_cam(att.gimbal_yaw_deg + bs["yaw"], att.gimbal_pitch_deg + bs["pitch"], att.gimbal_roll_deg + bs["roll"])
    xy = (tr.loc[f, "x"], tr.loc[f, "y"])
    z0, g = ground_fn(xy)
    H = tr.loc[f, "alt_ell"] - z0
    cam_r = G.Camera(json.loads((REAL / "meta.json").read_text())["camera"])
    img_r = G.load_gray(str(REAL / pd.read_csv(REAL / "images.csv").path[f - 1]), real_factor)
    a = G.rectify(img_r, real_factor, cam_r, R, H, res, ground=g)
    tmeta = json.loads((twin_dir / "meta.json").read_text())
    cam_t = G.Camera(tmeta["camera"])
    tf = max(1, int(round(real_factor * cam_t.w / FULL_W)))
    img_t, m_t = twin_gray(str(twin_dir / pd.read_csv(twin_dir / "images.csv").path[f - 1]), tf)
    b = G.rectify(img_t, tf, cam_t, R, H, res, ground=g)
    bm = G.rectify(m_t, tf, cam_t, R, H, res, ground=g)
    b["valid"] &= bm["patch"] >= 254
    # common grid: both grids are multiples of res relative to the nadir
    x0 = max(a["x0"], b["x0"])
    y0 = min(a["y0"], b["y0"])
    x1 = min(a["x0"] + (a["patch"].shape[1] - 1) * res, b["x0"] + (b["patch"].shape[1] - 1) * res)
    y1 = max(a["y0"] - (a["patch"].shape[0] - 1) * res, b["y0"] - (b["patch"].shape[0] - 1) * res)
    cols, rows = int(round((x1 - x0) / res)) + 1, int(round((y0 - y1) / res)) + 1

    def crop(p):
        c0 = int(round((x0 - p["x0"]) / res))
        r0 = int(round((p["y0"] - y0) / res))
        return p["patch"][r0:r0 + rows, c0:c0 + cols], p["valid"][r0:r0 + rows, c0:c0 + cols]
    pa, va = crop(a)
    pb, vb = crop(b)
    return dict(real=pa, twin=pb, valid_real=va, valid_twin=vb, x0=x0, y0=y0, res=res,
                img_real=img_r, img_twin=img_t, mask_twin=m_t)


def zncc(a, b, m):
    a, b = a[m].astype(np.float64), b[m].astype(np.float64)
    if len(a) < 100 or a.std() < 1e-6 or b.std() < 1e-6:
        return np.nan
    return float(np.corrcoef(a, b)[0, 1])


def align(pair, max_shift_m: float = 12.0, angles=np.arange(-3.0, 3.01, 0.25)):
    """Rotation about the camera nadir (deg, counter-clockwise on the north-up patch) and then shift (east,
    north metres) that bring the twin patch onto the real patch (best ZNCC). A camera yaw difference shows as
    a pure rotation, a position / map placement difference as a pure shift. Template = largest rectangle valid
    in both patches; real pixels outside the real footprint are filled with the real mean."""
    import x_tuniu_match as M
    res = pair["res"]
    both = pair["valid_real"] & pair["valid_twin"]
    out = dict(valid_frac=float(both.mean()), zncc0=zncc(pair["real"], pair["twin"], both))
    m = int(round(max_shift_m / res))
    r0, c0, r1, c1 = M.max_rect(cv2.erode(both.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0)
    if r1 - r0 < 30 or c1 - c0 < 30:
        return out | dict(dx_m=np.nan, dy_m=np.nan, rot_deg=np.nan, zncc_best=np.nan, tmpl_m2=0.0)
    real = pair["real"].astype(np.float32)
    real = np.where(pair["valid_real"], real, real[pair["valid_real"]].mean())
    search = cv2.copyMakeBorder(real, m, m, m, m, cv2.BORDER_CONSTANT, value=float(real.mean()))
    search = search[r0:r1 + 2 * m, c0:c1 + 2 * m]
    nadir = (-pair["x0"] / res, pair["y0"] / res)
    twin = pair["twin"].astype(np.float32)
    best = None
    for a in angles:
        Mrot = cv2.getRotationMatrix2D(nadir, float(a), 1.0)    # positive = counter-clockwise as displayed
        tw = cv2.warpAffine(twin, Mrot, (real.shape[1], real.shape[0]), flags=cv2.INTER_LINEAR)
        s = cv2.matchTemplate(search, tw[r0:r1, c0:c1], cv2.TM_CCOEFF_NORMED)
        _, mx, _, loc = cv2.minMaxLoc(s)
        if best is None or mx > best[0]:
            best = (mx, loc, s, a)
    mx, (lx, ly), s, a = best

    def sub(v_m, v0, v_p):
        d = v_m - 2 * v0 + v_p
        return 0.0 if d >= 0 else 0.5 * (v_m - v_p) / d
    dx = lx + (sub(s[ly, lx - 1], s[ly, lx], s[ly, lx + 1]) if 0 < lx < s.shape[1] - 1 else 0) - m
    dy = ly + (sub(s[ly - 1, lx], s[ly, lx], s[ly + 1, lx]) if 0 < ly < s.shape[0] - 1 else 0) - m
    # best template position (dx, dy) px from where it sits in the twin: the real ground there is shifted
    # by +dx px east and +dy px south relative to the twin
    return out | dict(dx_m=float(dx * res), dy_m=float(-dy * res), rot_deg=float(a), zncc_best=float(mx),
                      tmpl_m2=float((r1 - r0) * (c1 - c0) * res * res),
                      edge=bool(lx in (0, s.shape[1] - 1) or ly in (0, s.shape[0] - 1) or abs(a) >= angles.max()))


def cmd_fidelity(args) -> None:
    twin_dir = replay_dir(args.variant)
    frames = list(range(1, 272)) if args.all else SAMPLE
    odir = WORK / "fidelity" / (args.tag or args.variant)
    odir.mkdir(parents=True, exist_ok=True)
    rows = []
    for f in frames:
        pair = rectify_pair(f, twin_dir)
        r = dict(frame=f) | align(pair)
        rows.append(r)
        if not args.all:
            print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)
        if f in SAMPLE:
            save_side_by_side(pair, odir / f"frame_{f:04d}.jpg", r)
    df = pd.DataFrame(rows)
    df.to_csv(odir / ("all.csv" if args.all else "sample.csv"), index=False)
    s = df.dropna(subset=["dx_m"])
    summ = dict(variant=args.variant, frames=len(df), aligned=len(s),
                zncc0_median=float(df.zncc0.median()), zncc_best_median=float(s.zncc_best.median()),
                shift_m_median=float(np.hypot(s.dx_m, s.dy_m).median()),
                shift_m_p90=float(np.hypot(s.dx_m, s.dy_m).quantile(0.9)),
                dx_median=float(s.dx_m.median()), dy_median=float(s.dy_m.median()),
                rot_deg_median=float(s.rot_deg.median()), rot_abs_max=float(s.rot_deg.abs().max()))
    pre = s[s.frame <= 46]
    if len(pre):
        summ |= dict(precut_dx_median=float(pre.dx_m.median()), precut_dy_median=float(pre.dy_m.median()),
                     precut_n=len(pre))
    (odir / ("summary_all.json" if args.all else "summary_sample.json")).write_text(json.dumps(summ, indent=1))
    print(json.dumps(summ, indent=1))


def save_side_by_side(pair, path: Path, r: dict) -> None:
    ir, it = pair["img_real"], pair["img_twin"]
    it = cv2.resize(it, (ir.shape[1], ir.shape[0]), interpolation=cv2.INTER_AREA)
    top = np.hstack([ir, it])
    pr, pt = pair["real"].copy(), pair["twin"].copy()
    pr[~pair["valid_real"]] = 0
    pt[~pair["valid_twin"]] = 0
    # checkerboard of 20 m squares: edges continuous when aligned
    hh, ww = pr.shape
    yy, xx = np.mgrid[0:hh, 0:ww]
    chk = np.where(((yy // 40) + (xx // 40)) % 2 == 0, pr, pt)
    bot = np.hstack([pr, pt, chk])
    scale = top.shape[1] / bot.shape[1]
    bot = cv2.resize(bot, (top.shape[1], int(bot.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    canvas = np.vstack([top, bot])
    txt = (f"frame {r['frame']}  left real / right twin; below: rectified 0.5 m real | twin | checker  "
           f"zncc0 {r['zncc0']:.2f} best {r.get('zncc_best', np.nan):.2f} shift E {r.get('dx_m', np.nan):+.1f} "
           f"N {r.get('dy_m', np.nan):+.1f} m rot {r.get('rot_deg', np.nan):+.1f}")
    canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    cv2.putText(canvas, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])


# ----------------------------------------------------------------------------- realistic camera

SIM_CAMERA = Path(os.environ.get("TAIPEIDRIFT_SIM", ROOT.parent / "TaipeiDrift-sim")) / "baseline"


def cmd_degrade(args) -> None:
    """TaipeiDrift-sim RealisticCamera (configs/sim_navigator.yaml cameras.realistic), adapted to an oblique
    rectangular frame: clouds are drawn where each pixel's ray meets flat ground (terrain at the nadir), the
    vignetting / residual distortion radius is normalised as in the original (1 at mid-edge of the long side,
    corners ~ 2), everything else (haze, blur px, auto-exposure, shot noise, JPEG 80) is the original code."""
    import yaml
    sys.path.insert(0, str(SIM_CAMERA))
    from src.data import camera_model as CM   # noqa: E402
    from src.data.degrade import AIRLIGHT, less_light  # noqa: E402
    cfg = yaml.safe_load((SIM_CAMERA / "configs/sim_navigator.yaml").read_text())["cameras"]["realistic"]
    model = CM.CameraModel(**cfg)
    src, dst = replay_dir(args.variant), replay_dir(args.variant + "_rc")
    spec = spec_of(args.variant)
    (dst / "images").mkdir(parents=True, exist_ok=True)
    for fn in ("attitude.csv", "truth.csv", "gnss.csv", "twin_protocol.json", "images.csv"):
        shutil.copyfile(src / fn, dst / fn)
    if (dst / "masks").is_symlink():
        (dst / "masks").unlink()
    (dst / "masks").mkdir(exist_ok=True)
    # residual distortion (<= 3 px) and blur move the mesh border: shrink the valid mask by 3 px
    for mp in sorted((src / "masks").glob("*.png")):
        m = cv2.imread(str(mp), cv2.IMREAD_GRAYSCALE)
        cv2.imwrite(str(dst / "masks" / mp.name), cv2.erode(m, np.ones((7, 7), np.uint8)))
    meta = json.loads((src / "meta.json").read_text())
    meta["sequence"] = dst.name
    meta["twin"]["realistic_camera"] = dict(model=cfg, source="TaipeiDrift-sim baseline/src/data/camera_model.py")
    (dst / "meta.json").write_text(json.dumps(meta, indent=2))
    cam = meta["camera"]
    w, h = cam["image_width_px"], cam["image_height_px"]
    K = K_of(cam)
    P = pd.read_csv(src / "truth.csv")
    A = pd.read_csv(src / "attitude.csv")
    clouds = CM.cloud_field(model)
    ncl = clouds.shape[0]
    v, u = np.mgrid[0:h, 0:w].astype(np.float32)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    half = max(cx, cy)
    r2 = ((u - cx) ** 2 + (v - cy) ** 2) / (half * half)
    vignette = (1.0 - model.vignetting * r2 / 2.0).astype(np.float32)
    k1 = model.distortion_k1                          # on the radius normalised to 1 at mid long edge, as the original
    sc = 1.0 / (1.0 + k1 * r2)
    map_x = (cx + (u - cx) * sc).astype(np.float32)
    map_y = (cy + (v - cy) * sc).astype(np.float32)
    rays = np.linalg.solve(K, np.stack([u.ravel(), v.ravel(), np.ones(u.size, np.float32)]))
    x3826, y3826 = G.TO_3826.transform(P.lon_deg.to_numpy(), P.lat_deg.to_numpy())
    # cloud field anchored on the first photo (local metres east/north)
    ox, oy = x3826[0], y3826[0]
    t0 = time.time()
    for i in range(len(P)):
        out = dst / "images" / f"frame_{i + 1:04d}.jpg"
        if out.exists() and not args.force:
            continue
        ideal = cv2.imread(str(src / "images" / f"frame_{i + 1:04d}.jpg"), cv2.IMREAD_GRAYSCALE).astype(np.float32)
        rng = np.random.default_rng((model.seed, 2, i))
        R = cam_rotation(A.iloc[i], spec)
        d = R @ rays
        z0 = float(G.dem_ellipsoidal(x3826[i], y3826[i]))
        hgt = P.alt_m[i] - z0
        down = np.where(d[2] < -1e-3, -d[2], np.nan)
        t = hgt / down
        east = (x3826[i] - ox) + d[0] * t
        north = (y3826[i] - oy) + d[1] * t
        east = np.nan_to_num(east, nan=0.0) - model.wind_east_mps * P.t_s[i]
        north = np.nan_to_num(north, nan=0.0) - model.wind_north_mps * P.t_s[i]
        col = np.mod(east / CM.CLOUD_GRID_M, ncl).astype(np.float32).reshape(h, w)
        row = np.mod(-north / CM.CLOUD_GRID_M, ncl).astype(np.float32).reshape(h, w)
        shadow = cv2.remap(clouds, col, row, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        light = ideal * (1.0 - shadow) * CM.RAIN_WET[model.rain]
        light = model.haze_contrast * light + (1.0 - model.haze_contrast) * AIRLIGHT
        light = cv2.remap(light, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        light *= vignette
        if model.blur_px > 0:
            light = cv2.GaussianBlur(light, (0, 0), model.blur_px)
        gain = 115.0 / max(float(np.mean(light)), 1.0) * float(np.exp(rng.normal(0.0, model.gain_wobble)))
        exposed = np.clip(light * gain, 0, 255).astype(np.uint8)
        noisy = less_light(exposed, model.light * CM.RAIN_LIGHT[model.rain], rng)
        cv2.imwrite(str(out), noisy, [cv2.IMWRITE_JPEG_QUALITY, model.jpeg_quality])
        if (i + 1) % 50 == 0:
            print(f"  {dst.name}: {i + 1}/{len(P)} frames, {time.time() - t0:.0f} s", flush=True)
    print(f"wrote {dst}")


# ----------------------------------------------------------------------------- image quality

def quality_stats(p: np.ndarray, v: np.ndarray) -> dict:
    """Sharpness and noise of a rectified 0.5 m patch: gradient energy, contrast, and the high-frequency
    residual (patch minus its 3x3 median), on valid pixels only."""
    p = p.astype(np.float32)
    v = cv2.erode(v.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    gx, gy = cv2.Sobel(p, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(p, cv2.CV_32F, 0, 1, ksize=3)
    hf = p - cv2.medianBlur(p.astype(np.uint8), 3).astype(np.float32)
    return dict(contrast_sd=float(p[v].std()), grad_mean=float(np.hypot(gx, gy)[v].mean()),
                grad_rel=float(np.hypot(gx, gy)[v].mean() / max(p[v].std(), 1e-3)),
                hf_mad=float(1.4826 * np.median(np.abs(hf[v]))))


def cmd_quality(args) -> None:
    rows = []
    for f in SAMPLE:
        for name in args.variants:
            pair = rectify_pair(f, replay_dir(name))
            both = pair["valid_real"] & pair["valid_twin"]
            if name == args.variants[0]:
                rows.append(dict(frame=f, image="real") | quality_stats(pair["real"], both))
            rows.append(dict(frame=f, image=name) | quality_stats(pair["twin"], both)
                        | dict(zncc0=zncc(pair["real"], pair["twin"], both)))
    df = pd.DataFrame(rows)
    df.to_csv(WORK / "quality_sample.csv", index=False)
    s = df.groupby("image").median(numeric_only=True).drop(columns="frame")
    print(s.round(3).to_string())
    (WORK / "quality_summary.json").write_text(s.to_json(indent=1))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("prepare")
    sub.add_parser("texel")
    sub.add_parser("projtest")
    r = sub.add_parser("render")
    r.add_argument("variant", choices=list(VARIANTS))
    r.add_argument("--frames", nargs="*", type=int)
    r.add_argument("--force", action="store_true")
    r.add_argument("--no-placement", dest="placement", action="store_false",
                   help="ignore the pre-cut mesh placement offset in twin_calibration.json")
    f = sub.add_parser("fidelity")
    f.add_argument("--variant", default="twin")
    f.add_argument("--tag", help="output folder name under fidelity/ (default: the variant)")
    f.add_argument("--all", action="store_true")
    pc = sub.add_parser("placement")
    pc.add_argument("--source", default="twin_unplaced", help="fidelity/<source>/all.csv of a render without placement")
    at = sub.add_parser("attitude")
    at.add_argument("--source", default="twin", help="fidelity/<source>/all.csv of the placed twin")
    at.add_argument("--smooth", default="leg", help="leg (one value per straight leg) or N (running median)")
    d = sub.add_parser("degrade")
    d.add_argument("variant", choices=list(VARIANTS))
    d.add_argument("--force", action="store_true")
    q = sub.add_parser("quality")
    q.add_argument("--variants", nargs="+", default=["twin", "twin_rc"])
    a = ap.parse_args()
    WORK.mkdir(parents=True, exist_ok=True)
    {"prepare": cmd_prepare, "texel": cmd_texel, "projtest": cmd_projtest, "render": cmd_render, "fidelity": cmd_fidelity,
     "placement": cmd_placement, "attitude": cmd_attitude, "degrade": cmd_degrade, "quality": cmd_quality}[a.cmd](a)


if __name__ == "__main__":
    main()
