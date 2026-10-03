# The simulated flights: what we measured

Saturday 3 October 2026, 18:46. Every number here comes from the recordings in `recordings/` and the scripts
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

At save point 4, on all three development flights (100 m / 120 m / 65 m, over three draws):

| Condition | Median error | Stated bound held (lowest draw) | Wrong fixes used |
|---|---|---|---|
| The simulator's ideal camera | 16.5 / 32.3 / 18.0 m | 100% / 100% / 100% | none |
| The realistic camera, clear weather | 18.2 / 31.3 / 16.7 m | 100% / 100% / 97.8% | one on the 65 m flight, in 2 of 3 draws |
| The realistic camera, fog with 1 km of visibility | 61.3 / 93.6 / 22.3 m | 88.9% / 75.2% / 97.8% | one on the 65 m flight, in 1 of 3 draws |

Two limits, stated plainly:

- **Fog.** No camera sees through it. At 1 km of visibility the median error grows up to three and a half times,
  and the stated bound fails 2 to 25 percent of the time: the navigator does not notice that its picture has gone flat.
- **Turns, with a realistic camera.** The wrong fix on the 65 m flight lands 54 m from the truth (our line is 50 m).
  It is taken while the drone turns on the spot at the end of a leg, at about 30 degrees per second and tilted up to
  20 degrees, so the camera does not look straight down. A navigator with only a camera also cannot tell that the
  drone has stopped to turn: it counts on at cruising speed and, on that flight, is about 50 m off after the turn.
  An IMU can tell; that is what fusing with the team's filter (`vio/`) would add.

## Tried after save point 4, and not kept

Each was measured on the three development flights with both cameras. The logs are in `outputs/` and the code is
kept as patches in `outputs/experiments/`; neither is in git.

| Change | What it did | Why it is not in |
|---|---|---|
| Noticing fog: widen the uncertainty when the picture loses fine detail, compared with the pictures before the jam | Nothing in fog: the bound held 75 to 98 percent with it and without it | With fog from take-off the pictures before the jam are as dull as the rest; in clear weather it cost flight 1 5 m |
| The same, compared with a clear day | Flying on camera motion alone, the bound holds again at 1 km (99.6 to 100 percent, from 92) | With map fixes, wrong fixes remain (the 120 m flight, 2 of 3 draws) |
| Search only the scale learned before the jam, one step either side | Removes one of the realistic camera's two wrong fixes | Not both |
| No fix while the heading turns faster than 10 degrees per second | Worse: wrong fixes on two of three flights with either camera | It moves every later attempt, and look-alikes at other scales get through |
| Those two together | No wrong fix on any flight with either camera | 4 to 10 m worse with the ideal camera (23.9 / 35.9 / 27.5 m) |
| Those two, and believing the camera when the drone stops to turn | No wrong fix; the camera alone improves on the 65 m flight (64 to 54 m) | 4 to 6 m worse with the ideal camera (20.5 / 35.8 / 24.2 m); with the realistic camera flight 1 is for a moment "within 50 m" while further off |

The rule was set before these tests: a change stays only if every development flight works and none gets worse.
None of them met it, so the navigator of save point 4 is the one we freeze. The last row is where we would start
again: with an IMU telling the navigator when the drone turns or stops.

## The freeze and the sealed flights

**Frozen on Saturday 3 October 2026 at 18:46** (tag `frozen-navigator`): the navigator of save point 4, its code
unchanged since.

Declared before the run:

- The two sealed flights are run once, with `baseline/configs/sim_navigator.yaml` as it stands and three draws of
  the heading sensor: `wufeng_north_90m` (90 m, 9 m/s, 4.8 km) and `wufeng_south_110m` (110 m, 8 m/s, 5.1 km).
  Neither was run or looked at before.
- The numbers we quote are those with the camera as configured, the simulator's pictures. Next to them, also run
  once: the realistic camera, because of the weakness above.
- Known beforehand: on `wufeng_north_90m` the recorder dropped pictures under CPU load (3,326 instead of about
  3,600). It is run as recorded.
- "Works" means what it meant in the loop. Whatever comes out is reported here.

```bash
python baseline/scripts/run_sim_navigator.py --recording recordings/wufeng_north_90m --route sim/scenarios/wufeng_north_90m.json [--camera realistic]
python baseline/scripts/run_sim_navigator.py --recording recordings/wufeng_south_110m --route sim/scenarios/wufeng_south_110m.json [--camera realistic]
```

### The result

Run once on Saturday 3 October 2026, 18:46 to 18:52, at tag `frozen-navigator` (log: `outputs/sealed_run.log`).
Three draws of the heading sensor; medians over the draws, in metres.

| Sealed flight | Camera alone | With the 2018 map: median / 90 percent / worst | Fixes used per draw | Wrong fixes | Stated bound held | With the 2020 map |
|---|---|---|---|---|---|---|
| North first, 90 m, 4.3 km without GNSS | 70.6 | **36.3 / 70 / 90** | 12, 7, 7 | none | 100% in every draw | 18.4 |
| South first, 110 m, 4.5 km without GNSS | 57.6 | **16.2 / 38 / 58** | 14, 15, 15 | none | 100% in every draw | 11.8 |

With the realistic camera: 21.8 / 60 / 85 m and 15.3 / 33 / 48 m, no wrong fix, the bound held 100 percent.
Both flights work, with both cameras: no wrong fix used, the true error within the stated 3 sigma all the time,
never "within 50 m" while further off.

![Sealed flight, 90 m](figures/sealed_north_90m.png)

![Sealed flight, 110 m](figures/sealed_south_110m.png)

How to read it:

- **The map fixes cut the camera's drift** from 71 to 36 m on one flight and from 58 to 16 m on the other, with a map
  two years older than the ground.
- **The 90 m flight is the weaker one.** Its three draws give 20, 36 and 42 m: in two of them the navigator used only
  7 of its 16 fix attempts and held the others back. It stayed honest and became less accurate. The largest error
  in any draw was 128 m (62 m on the 110 m flight), inside the bound it stated at that moment.
- **The bound is honest and wide.** Three sigma is above 50 m for 71 to 81 percent of the time, so with an alert
  limit of 50 m the navigator says "I cannot promise 50 m" most of the time, and is right when it does promise.
- **Two flights, one place, one simulator.** The development flights showed what a turn and fog can do; these two
  flights did not show it, and that is no proof it cannot happen.

## After the freeze: the navigator's measurements in Alessandro's filter

Saturday 3 October 2026, 20:04. Development flights only; the sealed flights were not run again, and the numbers we
quote stay those of the frozen navigator above.

`scripts/fused_replay.py` replays a recorded flight through Alessandro's ESKF (`vio/estimation/eskf.py`), run as his
live adapter runs it: started on the pad from GNSS and the IMU's gravity, then IMU prediction, barometer height, and
GNSS position with his velocity fit until GNSS is lost at the 450 m mark. From there the filter is fed what the camera
navigator measures:

- **the sun sensor's heading,** as a yaw, at every frame;
- **the down camera's ground speed,** corrected for tilt first: a camera fixed to the drone looks at the ground a
  little away from the point below it, so a tilt change shifts the picture (at 100 m, 1 degree is 1.75 m, which
  over 0.2 s reads as 8.7 m/s). The filter knows its tilt and takes that shift out. A reading far from what the
  filter expects is given less weight, not thrown out and not believed outright; a step shorter than 0.3 of the
  cruising step is not fused (stopped, or lost track: the IMU decides);
- **the map fixes the frozen navigator used,** as horizontal positions, the way Dan's code fuses the ships' RF fix:
  behind the filter's gate, with a reset after three rejections in a row;
- **the navigator's uncertainty rule** on the horizontal position: 10 percent of the distance since the last fix. The
  filter's own uncertainty grows far too slowly, because it takes the camera's speed errors for random.

Step by step on the 100 m flight (one draw), error after the GNSS loss:

| Fed to the filter | Median | At the end | Heading error |
|---|---|---|---|
| GNSS all the way (the check that recordings and filter fit) | 1.5 m | 0.4 m | 5.8 degrees |
| IMU and barometer only | 3.0 km | 30.8 km | 55 degrees |
| plus the sun heading | 2.4 km | 24.1 km | 1.3 degrees |
| plus the camera's ground speed | 57 m | 115 m | 1.3 degrees |
| plus the navigator's map fixes | 18 m | 9 m | 1.3 degrees |

On all three development flights (100 m / 120 m / 65 m), median over three draws, median error in metres with the
worst in brackets:

| | The camera's steps alone | Filter with sun and camera speed | Frozen navigator | Filter with its map fixes |
|---|---|---|---|---|
| Ideal camera | 69 / 95 / 54 | 62 / 83 / 26 (147 / 207 / 205) | 16.5 / 32.3 / 18.0 (72 / 118 / 85) | **18.4 / 31.0 / 11.0 (56 / 85 / 94)** |
| Realistic camera | 69 / 98 / 53 | 62 / 81 / 31 (138 / 198 / 235) | 18.2 / 31.3 / 16.7 (57 / 149 / 104) | **14.7 / 29.5 / 9.6 (52 / 96 / 118)** |
| Ideal camera, compass for the sun sensor | 74 / 106 / 68 | 76 / 93 / 31 (175 / 240 / 246) | 22.0 / 34.4 / 30.3 (92 / 133 / 120) | 21.6 / 36.3 / 20.5 (85 / 112 / 131) |

How to read it:

- **Between fixes the filter drifts less than the camera alone,** on every flight. With the map fixes it is about as
  accurate as the frozen navigator, and its worst error is smaller on two of the three flights.
- **The sun sensor:** it keeps the heading within 1.3 degrees, and the filter took every one of its readings. Alone
  it does not hold the position; it is what lets the camera's speed, measured in the picture, be turned into north
  and east. With a compass in its place every number in the last row is worse.
- **The stated error is less reliable than the navigator's.** With the ideal camera the true error is within the
  filter's 3 sigma 100, 98.1 and 99.9 percent of the time (lowest draw); the navigator's holds 100 percent. On the
  realistic 65 m flight the filter fuses the navigator's one wrong fix too, and its bound holds 93 percent.

Limits: three flights of one simulated place, and the filter's two settings (1 m/s for the camera's speed, the
limit for an unexpected reading) were chosen on them. The navigator finds the fixes on its own; the filter does not
steer the map search yet. The tilt correction assumes flat ground. The filter starts on the pad with a heading
reading that is 1 degree off.

```bash
python scripts/fused_replay.py --flight wufeng_corridor_100m                         # GNSS all the way
python scripts/fused_replay.py --flight wufeng_corridor_100m --cut                   # IMU and barometer only
python scripts/fused_replay.py --flight wufeng_corridor_100m --cut --camera          # plus sun heading and camera speed
python scripts/fused_replay.py --flight wufeng_corridor_100m --cut --camera --fixes  # plus the map fixes
python scripts/fused_replay.py --all [--camera-model realistic] [--heading-source compass --yaw-sigma 4.1]
```
