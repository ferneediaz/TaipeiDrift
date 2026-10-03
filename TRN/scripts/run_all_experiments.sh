#!/usr/bin/env bash
# Runs the Monte Carlo milestones M4-M6 in sequence (several hours on 7 cores). Pass --quick for a smoke test.
set -euo pipefail
cd "$(dirname "$0")/.."
for e in m5_clouds m6_beams m6_imu m6_baro m6_altitude m6_map m4_baselines; do
  .venv/bin/python -m trn.experiments.monte_carlo "configs/experiments/$e.yaml" "$@"
done
echo ALL_EXPERIMENTS_DONE
