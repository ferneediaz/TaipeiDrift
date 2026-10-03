# Data

Datasets live in `data/raw/` and `data/processed/` and are not committed. Keep downloads inside `data/raw/`; a large file anywhere else in the repository would be picked up by git.

## Mid-Air (`data/raw/midair`)

- Source: [Mid-Air](https://midair.ulg.ac.be/), licence CC BY-NC-SA 4.0
- Get the links: fill in the form on the [download page](https://midair.ulg.ac.be/download.html) with image type "Down RGB". It sends a text file of links. That file contains a personal download ID, so keep it out of git.
- Download our subset, about 10 GB: `scripts/fetch_midair.sh path/to/links.txt`. It needs `wget` (`brew install wget`).
- Do not download the whole list. All downward-camera data is 100.6 GB.
- What the subset contains:
  - `MidAir/<set>/<condition>/sensor_records.zip` for every flight in every condition. Each holds one `sensor_records.hdf5` with, per flight, the IMU at 100 Hz, the simulated GNSS at 1 Hz and the ground truth at 100 Hz.
  - `MidAir/Kite_training/sunny/color_down/trajectory_0000` to `0005` and the same six flights in fog, `trajectory_2000` to `2005`.
  - `MidAir/PLE_training/<season>/color_down/`, three flights each in spring (`5000` to `5002`), fall (`4000` to `4002`) and winter (`6000` to `6002`).
- Flight numbers: the last three digits name the flight, the first digit the condition. `0003` and `2003` are the same flight in sun and in fog.
- Reading the sensor file needs `h5py`, which is in the project setup.

## ALTO (`data/raw/alto`)

- Source: the ICRA 2022 competition sample of [ALTO](https://github.com/MetaSLAM/ALTO), described in the [competition repository](https://github.com/MetaSLAM/GPR_Competition)
- Download in a browser from the [Round 1 folder](https://www.dropbox.com/sh/q1w5dmghbkut553/AAAOCMaELmfHE4NN5cw06QBba?dl=0), subfolder `UAV`: `readme.txt` and `Val.zip` (1.73 GB). The links do not work from the command line.
- `Val.zip` contains:
  - `Val/query_images/`: 1,684 frames from the helicopter's downward camera, 500 by 500 pixels
  - `Val/reference_images/offset_0_None/`: 459 aerial images along the route, one every 10 m. Four more folders hold copies shifted 20 and 40 m north and south.
  - `Val/query.csv`: position, altitude and orientation for every camera frame
  - `Val/reference.csv`: position of every reference image
  - `Val/gt_matches.csv`: the correct reference image for every camera frame
- `Train.zip` (9.93 GB) and `Test.zip` are not needed. The test section has no positions.

## Aerial images (`data/raw/aerial`)

- Source: [OpenAerialMap](https://openaerialmap.org/), licence CC BY 4.0, credit the image providers named there
- Two images of the same site in Wufeng, Taichung, map projection EPSG:3826 (TWD97): 3 May 2018 and 23 March 2020
- Each covers a corridor along a motorway, about 3 km long and 300 to 600 m wide
- Fetch with `python scripts/fetch_aerial.py`. It reads a quarter-resolution version over HTTP (about 20 and 14 cm per pixel, 27 MB together) instead of the full 115 MB and 385 MB files. Pass a different factor as the first argument for more or less detail.
- Use: the simulator on branch `simulations` can take the 2020 image as its ground. The 2018 image can then serve as the on-board map.

## Elevation (`data/processed/dem_strip.npy`)

- Source: Copernicus DEM GLO-30, a 30 m surface model, free to use with attribution, read from the public AWS bucket `copernicus-dem-30m`
- A strip at 23.90 to 24.20 N, 120 to 121 E: from the Taiwan Strait across the coast into the mountains
- Fetched automatically on the first run of `python experiments/c_terrain_matching.py`
- Used only by the earlier experiments in `docs/experiments.md`.
