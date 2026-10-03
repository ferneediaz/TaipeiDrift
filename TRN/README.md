# Obscuration-aware laser TRN simulator — Taiwan

Simulates a fixed-wing drone that has lost GNSS and navigates by matching a multi-echo laser altimeter
profile against an onboard elevation map. The proposed filter models **ground**, **cloud / fog / canopy**
and **missing** returns in its likelihood. It is compared with TERCOM and with a standard marginalized
particle filter (MPF) that uses only the last echo.

* Start here: **[TUTORIAL.md](TUTORIAL.md)**. It covers what the simulator does, how to run it, and how to read the results.
* Results: **[REPORT.md](REPORT.md)**. Design and data inspection: **[PLAN.md](PLAN.md)**.

![animation](docs/figures/animation_A_clouds50.gif)

## Install (macOS, Python 3.11)

```bash
cd TRN
uv venv --python 3.11 .venv          # or: python3.11 -m venv .venv
uv pip install --python .venv/bin/python -r requirements.txt
uv pip install --python .venv/bin/python -e .
.venv/bin/python -m pytest -q        # 38 tests
```

Set the raster paths in `configs/data.yaml` (the only file with absolute paths). The raw rasters in
`Data/` are read-only and are never committed.

## Run the milestones

| Milestone | Command | Output |
|---|---|---|
| M0 tiles | `python -m trn.terrain.tiles` | `data/processed/<route>/` |
| M1 maps | `python -m trn.experiments.m1_maps` | `results/m1_maps/`, `docs/figures/m1_*` |
| M2 laser | `python -m trn.experiments.m2_laser` | `results/m2_laser/` |
| M3 INS | `python -m trn.experiments.m3_ins` | `results/m3_ins/` |
| M4–M6 Monte Carlo | `python -m trn.experiments.monte_carlo configs/experiments/<exp>.yaml [--quick]` | `results/<exp>/` |
| all of M4–M6 | `scripts/run_all_experiments.sh [--quick]` | |
| M7 figures + tables | `python -m trn.experiments.m7_report` | `docs/figures/`, `docs/results_tables.md` |
| animation | `python -m trn.analysis.animate --route A_mountain_crossing --clouds 0.5` | GIF |

All commands run from `TRN/` with `.venv/bin/python`. Every run is reproducible from
`montecarlo.base_seed`, the run index and the config. Each result folder stores the resolved config.

## Layout

```
configs/        all parameters (YAML); experiments/ = Monte Carlo definitions
trn/common      config loading, frames (EPSG:3826), seeded RNG streams
trn/terrain     Grid (numba interpolation + ray casting), tiles, OnboardMap (+ builder), metrics
trn/truth       TruthMap (DSM top surface + DEM ground)  — simulator only
trn/trajectory  fixed-wing truth trajectory
trn/ins         strapdown INS, IMU errors, barometer, 16-state error model
trn/atmosphere  cloud layer + valley fog random fields
trn/laser       multi-echo altimeter (simulator) + shared sensor model
trn/filters     TERCOM, MPF (baseline / gated / proposed), likelihoods, resampling
trn/experiments runner, Monte Carlo, milestone scripts
trn/analysis    metrics, summaries, plots, animation
tests/          pytest (interpolation, ray casting, INS, likelihoods, truth isolation, toy filter)
```
