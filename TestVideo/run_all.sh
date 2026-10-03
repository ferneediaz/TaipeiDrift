#!/usr/bin/env bash
# Full pipeline: calibration -> tracking -> positioning -> tape scale check -> visualisation
# usage: ./run_all.sh [height_in_metres]   (default 0.641)
set -euo pipefail
cd "$(dirname "$0")"
H="${1:-0.641}"
if [ ! -x .venv/bin/python ]; then
  uv venv .venv -p 3.11 -q
  uv pip install -p .venv/bin/python -q -r requirements.txt
fi
PY="$(pwd)/.venv/bin/python -u"
cd vidnav
$PY calibrate.py                       # ~2 min   -> output/calibration.json
$PY track.py                           # ~8 min   -> output/cache/tracks.npz, keyframes.npz
$PY navigate.py --height "$H"          # ~7 min   -> output/trajectory.csv, summary.json
$PY validate_scale.py                  # ~8 min   -> output/scale_check.json
$PY render.py                          # ~6 min   -> floor_map*.jpg, trajectory.png, position_video.mp4
