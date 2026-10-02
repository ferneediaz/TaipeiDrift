#!/usr/bin/env bash
# Fetch the public audio datasets for Challenge 7 into data/raw/ (about 1.8 GB).
# Run from the repo root: bash scripts/fetch_data.sh
set -euo pipefail

mkdir -p data/raw
cd data/raw

if [ ! -d drone_audio ]; then
  git clone --depth 1 https://github.com/saraalemadi/DroneAudioDataset.git drone_audio
  rm -rf drone_audio/.git
fi

if [ ! -d esc50 ]; then
  curl -L -o esc50.zip https://github.com/karolpiczak/ESC-50/archive/refs/heads/master.zip
  unzip -q esc50.zip
  mv ESC-50-master esc50
  rm esc50.zip
fi

if [ ! -d outdoor_audio ]; then
  git clone --depth 1 --filter=blob:none --sparse https://github.com/DroneDetectionThesis/Drone-detection-dataset.git _ddt
  (cd _ddt && git sparse-checkout set Data/Audio)
  mkdir -p outdoor_audio
  mv _ddt/Data/Audio/* outdoor_audio/
  rm -rf _ddt
fi

du -sh drone_audio esc50 outdoor_audio
