# Data

Datasets live in `data/raw/` and are not committed. Fetch them with `bash scripts/fetch_data.sh`.

## Downloaded (Challenge 7, acoustic drone detection)

About 1.8 GB in total.

### Drone audio (`data/raw/drone_audio`, 803 MB)

- Source: https://github.com/saraalemadi/DroneAudioDataset (Al-Emadi et al., 2019)
- Format: 16 kHz, mono, clips of about 1 second
- `Binary_Drone_Audio/yes_drone`: 1,332 clips
- `Binary_Drone_Audio/unknown`: 10,372 clips (background and noise)
- `Multiclass_Drone_Audio`: 666 clips each for two drone models (`bebop_1`, `membo_1`), plus the same 10,372 `unknown` clips
- Licence: the repository states none. Cite the paper and confirm with the organisers that it counts as "public audio".
- Known weakness: the drone clips were recorded indoors at close range and then mixed with noise. A model can score well on a held-out split of this set and still fail on a real outdoor recording. Test on audio from another source before trusting any accuracy figure.

### ESC-50 (`data/raw/esc50`, 846 MB)

- Source: https://github.com/karolpiczak/ESC-50
- Format: 44.1 kHz, mono, 2,000 clips of 5 seconds, 50 classes with 40 clips each, labels in `meta/esc50.csv`
- Useful negative classes: `engine`, `helicopter`, `airplane`, `wind`, `rain`, `train`, `chainsaw`, `vacuum_cleaner`, `insects`, `siren`
- Licence: CC BY-NC (non-commercial). The ESC-10 subset is CC BY.
- The sample rate differs from the drone set, so resample everything to 16 kHz before training.

### Outdoor drone, helicopter and background audio (`data/raw/outdoor_audio`, 152 MB)

- Source: https://github.com/DroneDetectionThesis/Drone-detection-dataset, `Data/Audio` only (Svanström et al., 2021)
- Format: 44.1 kHz, two channels, 90 clips of 10 seconds, 30 each of `BACKGROUND`, `DRONE`, `HELICOPTER`, class in the file name
- Licence: CC0
- Use: keep this set out of training and use it as the final test. It comes from a different microphone and setting than the drone audio set above, so it shows whether the detector generalises. Helsing used this same data for its acoustic challenge at the London hackathon in May 2025.

## Not downloaded

### Challenge 2, navigation

The brief says the organisers suggest a dataset with IMU, speed, heading and reference position. Ask for it at kickoff. Alternatives if it is not provided:

- Record a short walk with a smartphone IMU logger (the brief allows this).
- Simulate a trajectory and add sensor noise, which also gives exact ground truth.
- KITTI raw GPS/IMU records. These sit inside 20 GB archives per drive, so only fetch them on a fast connection.

### Challenge 3, task allocation

No dataset needed. The brief mentions a common JSON scenario "if available"; otherwise write and document one.
