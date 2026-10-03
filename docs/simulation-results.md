# The simulated flights: what we measured

Saturday 3 October 2026, 17:30. Every number here comes from the recordings in `recordings/` and the scripts
named next to it; `python scripts/sim_progress.py` and `python scripts/sim_figures.py` redo the tables and figures.

## The setup

![The six simulated flights over Wufeng and their roles](figures/sim_flights.png)

- **The world:** Gazebo Harmonic in Docker; the ground is the real aerial photo of Wufeng, Taichung, from March 2020
  (OpenAerialMap, CC BY 4.0). The drone is a Mid-Air-like quadcopter with a down camera (512 px, 90 degrees, 5
  frames per second), an IMU, a barometer and GNSS, each with noise.
- **The navigator's map:** the photo of the same place from May 2018, two years older than the ground.
- **The test:** GNSS is lost after 450 m. Before that the navigator learns how image motion turns into ground motion;
  after it, it counts the ground moving past the camera and fixes its position on the map every 300 m.
- **Six flights,** their roles fixed before most of them were recorded: three for development, one held out, two sealed.

## Step by step

![Error and wrong fixes per development flight, stage by stage](figures/sim_progress.png)

One change per stage, each kept only if every development flight works. "Works" was fixed before the loop
started: with the 2018 map, on every flight and every draw of the heading sensor, no wrong fix used (a used fix
more than 50 m from the truth), the true error within the stated 3 sigma at least 99 percent of the time, and
never "within 50 m" while further off.

Median error with the 2018 map, over three draws (`scripts/sim_progress.py`):

| Stage | Flight 1, 100 m | Flight 3, 120 m | Flight 4, 65 m | Save point |
|---|---|---|---|---|
| 1. The navigator of save point 1 | 26.5 m, fails (bound 98.3%) | 30.9 m, 3 wrong fixes, fails | 39.8 m, 1 wrong fix, fails | `sp1-merged-camera-navigator` |
| 2. Keep the camera's scale learned before the jam | 18.5 m, works | 28.6 m, works | 26.6 m, 1 wrong fix, fails | `sp2-scale-kept` |
| 3. Search the zooms of the heights flown; confirm jumps from 100 m | 22.0 m, works | 34.4 m, works | 30.3 m, works | `sp3-all-dev-flights-work` |
| 4. Heading from a sun sensor instead of a compass | **16.5 m, works** | **32.3 m, works** | **18.0 m, works** | `sp4-sun-heading` |

The camera alone, without map fixes, ends at a median of 69, 94 and 64 m with the sun heading (76, 104 and 75 m
with the compass), and keeps growing with the distance flown.

What each change fixed:

- **Stage 2.** After every fix the navigator re-read the drone's height from the zoom of the match, in steps of 0.05,
  so one step changed the distance flown by 6 to 13 percent until the next fix. The drone holds its height, so the
  scale learned before the jam is kept.
- **Stage 3.** The zooms searched after the jam were set for 100 m and missed the 65 m flight's scale, so after a
  few lost fixes it searched only wrong scales and one look-alike, 74 m off, was accepted. Searching the zooms of
  the heights flown and asking a second fix to confirm any large jump from a search wider than 100 m removed every
  wrong fix; the price is fewer fixes used and a median a few metres higher.
- **Stage 4.** A simulated sun sensor (Fan, Peng and Gao 2016: 0.1 degree, 65-degree view) turned into a heading
  with the drone's tilt. Its heading error on flight 1: 0.9 degree median, no fixed offset; the simulated compass:
  1.8 degrees median with fixed offsets up to 8 degrees per draw. A sensor of five photodiodes, a few dollars, was
  about as good as the compass and failed the 120 m flight; it stays an option, not the default.

## The held-out flight

Flight 2 (80 m, 8 m/s, south first, 5.0 km) was recorded after save point 1 and run once with its settings (only
the calibration zooms widened, because 80 m lay outside them). With the 2018 map: median 20.7 m, 90 percent below
47 m, worst 79 m, no wrong fix used, the stated bound held 99.4 to 100 percent of the time. With the 2020 map (the
ground itself, an ideal map), the stated bound failed 2 to 5 percent of the time.

## Conditions measured so far

On flight 1, at the settings of stage 2 (compass), median / 90 percent / worst in metres with the 2018 map
(`scripts/sim_dev_check.py --camera realistic --camera-set ...`):

| Condition | Error | Bound held | Note |
|---|---|---|---|
| The simulator's ideal camera | 18.5 / 44 / 66 | 100% | |
| A realistic camera: cloud shadows, haze, lens, vibration, exposure, noise, JPEG | 18.0 / 40 / 68 | 100% | costs little |
| Fog, visibility 1 km | 75.5 / 171 / 216 | 82% | the error grows four times, and the bound fails |
| Fog, visibility 500 m | 56.2 / 129 / 186 | 86% | one wrong fix |
| Fog, visibility 300 m | 608 / 2,807 / 2,958 | 25% | the camera loses the ground |
| Heavy rain (set by eye) | 40.1 / 118 / 187 | 84% | one wrong fix |

Fog is the weak spot: no camera sees through it, and today the navigator does not notice that its picture has
gone flat, so its stated bound fails. The next step makes it notice.

## Still to come

1. The navigator notices fog (picture detail falls) and widens its uncertainty; kept if the fog flights then hold
   the bound and the clear flights do not change.
2. Freeze; the two sealed flights run once. Those are the numbers we quote.
