#!/usr/bin/env bash
# Fetch the public datasets for Challenges 7 and 4 into data/raw/ (about 2 GB).
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

if [ ! -d drone_video ]; then
  git clone --depth 1 --filter=blob:none --sparse https://github.com/DroneDetectionThesis/Drone-detection-dataset.git _ddt
  (cd _ddt && git sparse-checkout set Data/Video_V Data/Video_IR)
  mkdir -p drone_video
  mv _ddt/Data/Video_V _ddt/Data/Video_IR drone_video/
  rm -rf _ddt
fi

du -sh drone_audio esc50 outdoor_audio drone_video
