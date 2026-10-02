# First experiments: laser, camera and the sea

These experiments were made before the team chose the Mid-Air dataset and a sensor set of camera, IMU and barometer. They used Taiwan imagery and elevation, a laser and an airspeed sensor. The particle filter and the camera-matching findings still apply; the terrain and wind results do not belong to the current plan in [PLAN.md](PLAN.md).

Four quick experiments on real data of Taiwan, run on Friday evening to settle one question: does a camera cover the gaps of terrain matching with a laser, and what is left uncovered?

The scripts are in `experiments/`. Run them from the repository root after `uv sync` and `python scripts/fetch_aerial.py`.

## Short version

- On flat land the laser is useless and the camera is accurate. On Taiwan's coastal plain, terrain matching ended 1.2 km off, the same as doing nothing. Camera fixes brought that to 7 to 16 m.
- In mountains the laser needs no camera. It held 9 m.
- Over the sea neither works. What decides the error there is the wind.
- A filter that learns the wind while it still has fixes cuts the drift over water about five times.
- A constant compass error does no harm while fixes are available, because the filter absorbs it together with the wind.

## Setup

- Elevation: Copernicus 30 m surface model, a strip 102 km east to west and 22 km north to south at 24.05 N, from the Strait across the coast into the mountains.
- Imagery: the two aerial images of Wufeng, Taichung, from 2018 and 2020, resampled to 1 m per pixel.
- Drone: 20 m/s airspeed, flying straight, heading known.
- Wind: 1.8 m/s, unknown to the drone.
- Laser plus map error: 4 m per measurement, one measurement every 30 m of flight.
- Filter: 4000 particles over position and wind.

## A. Camera fix when the filter already knows roughly where it is

Script: `experiments/a_camera_with_prior.py`

A 96 m patch of the 2020 image is searched in the 2018 image within 150 m of the true position.

| Patches | Median error | Within 5 m | Wrong by more than 50 m |
|---|---|---|---|
| 103 | 3.6 m | 99% | 1% |

The 3.6 m is nearly constant, so it is most likely an offset between how the two images were georeferenced.

## B. Camera fix with no idea where it is

Script: `experiments/b_camera_blind.py`

The same patches are searched over the whole 3 km corridor.

| Ground seen by the camera | Within 10 m | Wrong by more than 50 m |
|---|---|---|
| 64 m wide | 77% | 21% |
| 96 m wide | 86% | 14% |
| 160 m wide | 97% | 3% |
| 96 m wide, blurred, darker, noisy | 70% | 30% |

Fields repeat. A single camera match without prior knowledge is wrong one time in seven, and more often with a poor image. Seeing more ground, which means flying higher or using a wider lens, helps a great deal. A filter that carries a rough position is what makes the camera reliable.

## C. Terrain matching along a real flight line

Script: `experiments/c_terrain_matching.py`

| Leg | Elevation range | Dead reckoning ends | Terrain matching, median | Worst of 6 runs |
|---|---|---|---|---|
| Open sea, 30 km | 0 m | 2515 m off | 2004 m | 2085 m |
| Coastal plain, 16 km | 0 to 23 m | 1341 m off | 1213 m | 1238 m |
| Foothills, 16 km | 30 to 224 m | 1341 m off | 78 m | 187 m |
| Mountains, 20 km | 274 to 996 m | 1677 m off | 9 m | 11 m |

With a laser and map error of 10 m in place of 4 m: foothills 126 m, mountains 16 m, plain unchanged.

On the plain with terrain matching switched off and a camera fix of 10 m accuracy:

| Camera fix | Error |
|---|---|
| Every 100 m of flight | 7 m |
| Every 1 km of flight | 16 m |

In this strip, 15 percent of the land has less than 2 m of relief per kilometre and another 29 percent has 2 to 10 m. That is 44 percent of the land where terrain matching has little to work with, and it is where the coast, ports, airfields and towns are.

## D. Crossing water

Script: `experiments/d_water_crossing.py`

The drone flies 16 km with a camera fix every kilometre, then 30 km with no fixes. The filter keeps the wind it learned.

| Wind during the crossing | Distance after last fix | With learned wind | Dead reckoning from last fix |
|---|---|---|---|
| Steady | 30 km | 109 m | 2515 m |
| Changes by about 0.5 m/s | 10 km | 111 m | 844 m |
| Changes by about 0.5 m/s | 30 km | 552 m | 2702 m |
| Changes by about 1.5 m/s | 30 km | 1642 m | 3223 m |

Without any wind knowledge the drift is about 84 m per kilometre flown, which is the ratio of wind speed to airspeed.

Adding a constant compass error of 1 or 5 degrees changed none of these numbers. On a straight course a constant heading error looks exactly like a crosswind, and the filter learns both together.

## What follows for the design

1. The filter should carry the wind as part of what it estimates. That is the piece that helps over water, and it costs a few lines.
2. Camera and laser are both needed around Taiwan, for different ground: camera on the plain, laser in the hills and at night.
3. A camera fix must be checked against the filter's own position. Alone it is wrong too often.
4. The sun compass matters for the cold start, before any fix exists, and for heading errors that change over time. It matters less than we assumed once fixes are flowing.
5. Over open water the honest claim is a drift of a few hundred metres after 30 km in steady weather, growing faster when the wind shifts.

## Limits of these experiments

- The simulation is two-dimensional with constant airspeed, a perfect airspeed sensor and a straight course.
- The first version of the terrain filter diverged: it had no wind state, lagged behind the true position and then became confident in a wrong place. The numbers above are from the corrected filter.
- The camera test uses two corrected aerial images. A real camera adds tilt, blur and perspective.
- 103 to 150 patches from one 3 km corridor of farmland beside a motorway. Other ground will differ.
- Night, fog and rain were not tested. Both sensors are optical, so both should degrade in fog and rain, and a visible-light camera fails at night.
- The wind model is a slow random change. Real wind over a strait has gusts and changes with altitude.
