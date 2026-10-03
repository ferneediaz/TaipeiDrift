"""Replay of a navigator run as a video, for the demo.

Left: the map around the drone, with the true path, the navigator's estimate, the circle of its
stated uncertainty (3 sigma) and every attempted fix: used in green, refused in red with the reason.
Right: what the camera sees, the status the navigator reports, the numbers, and the error so far
against the distance flown.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FFMpegWriter  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

from src.data.camera_flight import CameraFlight  # noqa: E402
from src.estimation.camera_navigator import NavigatorResult  # noqa: E402
from src.evaluation.navigation_metrics import navigation_errors  # noqa: E402

STATUS_COLOURS = {"TRACKING": "#1baf7a", "DEGRADED": "#d6a12a", "LOST": "#d64a4a"}
REASON_TEXT = {
    "OK": "used",
    "LOW_SCORE": "refused: weak match",
    "DISAGREES_WITH_ESTIMATE": "refused: too far from the estimate",
    "UNCONFIRMED": "held: large jump, waits for the next fix",
    "QUARTERS_DISAGREE": "refused: parts of the image disagree",
    "FRAMES_DISAGREE": "refused: frames disagree",
    "OFF_MAP": "no map here",
}
ESTIMATE = "#eb6834"
TRUE_PATH = "#ffffff"


def _map_window(flight: CameraFlight, centre: np.ndarray, half_m: float, aspect: float = 1.0, out_px: int = 900):
    """The map around a point, cut and shrunk to about ``out_px`` wide; with its extent in metres (east, north).

    ``aspect`` is height over width of the window, to match the panel it is drawn in.
    """
    g = flight.ground_map
    x, y = g.to_pixel(centre)
    half_px = half_m / g.metres_per_pixel
    left, right = int(max(0, x - half_px)), int(min(g.shape[1], x + half_px))
    top, bottom = int(max(0, y - half_px * aspect)), int(min(g.shape[0], y + half_px * aspect))
    crop = g.image[top:bottom, left:right]
    step = max(1, int(crop.shape[1] / out_px)) if crop.size else 1
    crop = crop[::step, ::step]
    west, north = g.to_position(left, top)[1], g.to_position(left, top)[0]
    east, south = g.to_position(right, bottom)[1], g.to_position(right, bottom)[0]
    return crop, (west, east, south, north)


def make_replay(
    flight: CameraFlight,
    result: NavigatorResult,
    path: Path,
    title: str,
    every: int = 2,
    fps: int = 20,
    window_m: float = 1500.0,
    stills: tuple[float, ...] = (),
) -> list[Path]:
    """Write the replay as an MP4 video. Returns the paths of the still images written.

    Args:
        every: draw every n-th frame of the flight.
        window_m: width of the map window around the drone, in metres.
        stills: fractions of the run (0 to 1) at which a PNG is also saved, for slides.
    """
    if flight.ground_map is None:
        raise ValueError("the replay draws the map; the flight has none")
    errors = navigation_errors(result, flight)
    start = result.start_index
    truth = flight.position_gt[start : start + len(result.position)]
    est = result.position
    three_sigma = 3 * result.sigma
    km = errors.distance_since_jam / 1000.0
    fixes = sorted(result.fixes, key=lambda f: f.frame)

    fig = plt.figure(figsize=(16, 9), dpi=100, facecolor="#111418")
    grid = fig.add_gridspec(3, 3, width_ratios=[1, 1, 0.95], height_ratios=[1.15, 0.55, 0.8], left=0.02, right=0.98, top=0.93, bottom=0.06, wspace=0.08, hspace=0.25)
    ax_map = fig.add_subplot(grid[:, :2])
    ax_cam = fig.add_subplot(grid[0, 2])
    ax_txt = fig.add_subplot(grid[1, 2])
    ax_err = fig.add_subplot(grid[2, 2])
    for ax in (ax_map, ax_cam, ax_txt):
        ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
    ax_txt.set_facecolor("#111418")
    fig.suptitle(title, color="white", fontsize=15, x=0.02, ha="left")

    box = ax_map.get_position()
    aspect = (box.height * fig.get_figheight()) / (box.width * fig.get_figwidth())
    crop, extent = _map_window(flight, truth[0], window_m / 2, aspect)
    background = ax_map.imshow(crop, cmap="gray", extent=extent, origin="upper", vmin=0, vmax=255, zorder=0)
    true_line, = ax_map.plot([], [], color=TRUE_PATH, lw=2.0, alpha=0.9, zorder=2, label="true path")
    est_line, = ax_map.plot([], [], color=ESTIMATE, lw=2.5, zorder=3, label="navigator's estimate")
    used_dots = ax_map.scatter([], [], s=70, color="#1baf7a", edgecolor="white", lw=0.8, zorder=5, label="fix used")
    refused_dots = ax_map.scatter([], [], s=80, color="#d64a4a", marker="X", edgecolor="white", lw=0.6, zorder=5, label="fix refused")
    true_dot, = ax_map.plot([], [], "o", color=TRUE_PATH, ms=9, mec="black", zorder=6)
    est_dot, = ax_map.plot([], [], "o", color=ESTIMATE, ms=10, mec="black", zorder=6)
    circle = Circle((0, 0), 1.0, facecolor=ESTIMATE, alpha=0.18, edgecolor=ESTIMATE, lw=1.5, zorder=4)
    ax_map.add_patch(circle)
    search = Circle((0, 0), 1.0, fill=False, edgecolor="#9ecbff", lw=1.2, ls="--", zorder=4, visible=False)
    ax_map.add_patch(search)
    fix_label = ax_map.text(0.02, 0.03, "", transform=ax_map.transAxes, color="white", fontsize=13,
                            bbox=dict(facecolor="#111418", alpha=0.75, edgecolor="none"), zorder=7)
    ax_map.legend(loc="upper right", fontsize=10, facecolor="#111418", labelcolor="white", framealpha=0.8)

    camera = ax_cam.imshow(flight.frame(start), cmap="gray", vmin=0, vmax=255)
    ax_cam.set_title("what the camera sees", color="white", fontsize=11)

    status_text = ax_txt.text(0.0, 0.85, "", fontsize=24, weight="bold", transform=ax_txt.transAxes, va="top")
    numbers = ax_txt.text(0.0, 0.42, "", fontsize=12, color="white", transform=ax_txt.transAxes, va="top", family="monospace")

    ax_err.set_facecolor("#111418")
    err_line, = ax_err.plot([], [], color=ESTIMATE, lw=2, label="true error")
    sig_line, = ax_err.plot([], [], color="#9ecbff", lw=1.5, ls="--", label="stated bound (3 sigma)")
    ax_err.set_yscale("log")
    ax_err.set_ylim(max(1.0, float(np.nanmin(errors.error[1:])) * 0.8 if len(errors.error) > 1 else 1.0), float(max(np.nanmax(errors.error), np.nanmax(three_sigma))) * 1.3)
    ax_err.set_xlim(0, km[-1])
    ax_err.set_xlabel("km since GNSS was lost", color="white")
    ax_err.set_ylabel("metres", color="white")
    ax_err.tick_params(colors="white")
    for s in ax_err.spines.values():
        s.set_color("#555")
    ax_err.legend(loc="upper left", fontsize=9, facecolor="#111418", labelcolor="white", framealpha=0.8)

    rows = list(range(0, len(est), every))
    if rows[-1] != len(est) - 1:
        rows.append(len(est) - 1)
    still_rows = {rows[min(len(rows) - 1, int(f * (len(rows) - 1)))]: f for f in stills}
    written: list[Path] = []
    last_fix = None
    writer = FFMpegWriter(fps=fps, bitrate=6000)
    with writer.saving(fig, str(path), dpi=100):
        for row in rows:
            frame = start + row
            here = truth[row]
            gap = float(np.linalg.norm(est[row] - here))
            half = max(window_m / 2, gap + 250.0)
            crop, extent = _map_window(flight, (est[row] + here) / 2, half, aspect)
            background.set_data(crop)
            background.set_extent(extent)
            ax_map.set_xlim(extent[0], extent[1])
            ax_map.set_ylim(extent[2], extent[3])

            true_line.set_data(truth[: row + 1, 1], truth[: row + 1, 0])
            est_line.set_data(est[: row + 1, 1], est[: row + 1, 0])
            true_dot.set_data([here[1]], [here[0]])
            est_dot.set_data([est[row, 1]], [est[row, 0]])
            circle.center = (est[row, 1], est[row, 0])
            circle.set_radius(three_sigma[row])

            done = [f for f in fixes if f.frame <= frame and np.all(np.isfinite(f.position))]
            used = np.array([[f.position[1], f.position[0]] for f in done if f.used]).reshape(-1, 2)
            refused = np.array([[f.position[1], f.position[0]] for f in done if not f.used]).reshape(-1, 2)
            used_dots.set_offsets(used)
            refused_dots.set_offsets(refused)
            recent = [f for f in fixes if frame - 3 * every <= f.frame <= frame]
            if recent:
                last_fix = recent[-1]
                search.center = (est[max(0, last_fix.frame - start - 1), 1], est[max(0, last_fix.frame - start - 1), 0])
                search.set_radius(max(last_fix.search_radius_m, 1.0))
                search.set_visible(last_fix.search_radius_m > 0)
            else:
                search.set_visible(False)
            if last_fix is not None:
                fix_label.set_text(f"last fix: {REASON_TEXT.get(last_fix.reason, last_fix.reason)}")

            camera.set_data(flight.frame(frame))
            state = result.status[row]
            status_text.set_text(state)
            status_text.set_color(STATUS_COLOURS.get(state, "white"))
            n_used = sum(f.used for f in fixes if f.frame <= frame)
            n_refused = sum((not f.used) for f in fixes if f.frame <= frame)
            numbers.set_text(
                f"since GNSS lost   {km[row]:6.2f} km\n"
                f"true error        {errors.error[row]:6.0f} m\n"
                f"stated bound      {three_sigma[row]:6.0f} m\n"
                f"fixes used        {n_used:6d}\n"
                f"fixes refused     {n_refused:6d}"
            )
            err_line.set_data(km[: row + 1], errors.error[: row + 1])
            sig_line.set_data(km[: row + 1], three_sigma[: row + 1])
            writer.grab_frame(facecolor=fig.get_facecolor())
            if row in still_rows:
                still = path.with_name(f"{path.stem}_{int(100 * still_rows[row]):02d}.png")
                fig.savefig(still, facecolor=fig.get_facecolor())
                written.append(still)
    plt.close(fig)
    return written
