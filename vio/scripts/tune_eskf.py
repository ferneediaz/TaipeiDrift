"""Tune the ESKF measurement noise on the TUNING flight only (config: tuning_trajectory).

Rule fixed before looking at the evaluation flights:
1. Forward-rotation sigma: the smallest value whose mean NIS of accepted updates
   is at most 1.2 x its 3 degrees of freedom (no overconfidence), with forward_rotation alone.
2. Flow settings: the grid point with the lowest ATE of the full filter ("both"),
   among those whose 9-state NEES mean is below 50 (not grossly overconfident).

    python vio/scripts/tune_eskf.py --data-root data/MidAir
"""
from __future__ import annotations

import argparse
import copy
import itertools
import json
import sys
from pathlib import Path

import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))

from src.data.midair import MidAirConfig, load_midair_trajectory  # noqa: E402
from vio.eskf_pipeline import evaluate, prepare_visual_inputs, run_ablation  # noqa: E402

ROT_SIGMAS = [0.15, 0.2, 0.25, 0.3, 0.4]
FLOW_GRID = {"dir_sigma_deg": [1.0, 1.5, 2.0], "rel_height_std": [0.3, 0.6, 1.0], "every_n_frames": [1, 5]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=VIO_DIR / "configs" / "midair_eskf.yaml")
    ap.add_argument("--data-root", default="data/MidAir")
    args = ap.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    vio_cfg = yaml.safe_load((REPO_ROOT / cfg["vio_config"]).read_text())
    t = cfg["tuning_trajectory"]
    traj = load_midair_trajectory(MidAirConfig.from_dict({"condition": t["condition"], "trajectory": t["trajectory"]}),
                                  data_root=args.data_root)
    k0 = traj.index_at(cfg["gnss_cutoff_s"])
    vis = prepare_visual_inputs(traj, cfg, vio_cfg, k0, 100.0, REPO_ROOT / cfg["output_dir"] / "cache")
    log = {"tuning_flight": t, "rotation": [], "flow": []}

    chosen_rot = ROT_SIGMAS[-1]
    for s in ROT_SIGMAS:
        c = copy.deepcopy(cfg)
        c["forward_camera"]["sigma_deg"] = s
        m = evaluate(run_ablation(traj, k0, vis, c, "forward_rotation"), traj, c, None)
        nis = m["updates"]["rotation"]["nis_accepted"]["mean"]
        log["rotation"].append({"sigma_deg": s, "nis_mean": nis, "ate_m": m["position"]["ate_rmse_m"]})
        print(f"rotation sigma {s:.2f} deg: NIS mean {nis:.2f} (dof 3), ATE {m['position']['ate_rmse_m']:.1f} m")
        if nis <= 1.2 * 3 and chosen_rot == ROT_SIGMAS[-1]:
            chosen_rot = s
    cfg["forward_camera"]["sigma_deg"] = chosen_rot

    best = None
    for vals in itertools.product(*FLOW_GRID.values()):
        c = copy.deepcopy(cfg)
        c["down_camera"].update(dict(zip(FLOW_GRID, vals)))
        m = evaluate(run_ablation(traj, k0, vis, c, "both"), traj, c, None)
        row = {**dict(zip(FLOW_GRID, vals)), "ate_m": m["position"]["ate_rmse_m"], "nees9": m["nees"]["nav9"]["mean"],
               "flow_nis": (m["updates"].get("flow", {}).get("nis_accepted") or {}).get("mean")}
        log["flow"].append(row)
        print(f"flow {dict(zip(FLOW_GRID, vals))}: ATE {row['ate_m']:.1f} m, NEES9 {row['nees9']:.1f}, flow NIS {row['flow_nis']}")
        if row["nees9"] < 50 and (best is None or row["ate_m"] < best["ate_m"]):
            best = row
    log["chosen"] = {"forward_sigma_deg": chosen_rot, "flow": best}
    out = REPO_ROOT / cfg["output_dir"] / "tuning.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(log, indent=2))
    print(f"chosen: forward sigma {chosen_rot} deg, flow {best}\nlog: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
