# Data

Datasets live in `data/raw/` and `data/processed/` and are not committed.

## Aerial images (`data/raw/aerial`)

- Source: [OpenAerialMap](https://openaerialmap.org/), licence CC BY 4.0, credit the image providers named there
- Two images of the same site in Wufeng, Taichung, map projection EPSG:3826 (TWD97): 3 May 2018 and 23 March 2020
- Each covers a corridor along a motorway, about 3 km long and 300 to 600 m wide
- Fetch with `python scripts/fetch_aerial.py`. It reads a quarter-resolution version over HTTP (about 20 and 14 cm per pixel, 27 MB together) instead of the full 115 MB and 385 MB files. Pass a different factor as the first argument for more or less detail.
- Use: the 2020 image is what the drone sees, the 2018 image is the on-board map.

## Elevation (`data/processed/dem_strip.npy`)

- Source: Copernicus DEM GLO-30, a 30 m surface model, free to use with attribution, read from the public AWS bucket `copernicus-dem-30m`
- A strip at 23.90 to 24.20 N, 120 to 121 E: from the Taiwan Strait across the coast into the mountains
- Fetched automatically on the first run of `python experiments/c_terrain_matching.py`
- It is a surface model, so it includes treetops and roofs, which is what a downward laser or radar would measure.

## Not yet available

- The organisers' suggested dataset with IMU, speed, heading and reference position. Ask for it.
- Real flight footage. [UAV-VisLoc](https://github.com/IntelliSensing/UAV-VisLoc) has a 2 GB sample on Google Drive; the repository states no licence.
