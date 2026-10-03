# Where we stand (the working doc)

**Last updated: Saturday 3 October, 22:10.** **The navigator is frozen** (tag `frozen-navigator`, 18:46) and the two sealed flights have been run, once: 36.3 and 16.2 m median over 4.3 and 4.5 km without GNSS, against 70.6 and 57.6 m for the camera alone, no wrong fix, the stated bound held all the time. What we hand in is a deck with a link to a demo video, by Sunday noon. The one up-to-date document for the team: what we are building, what changed today and why, what is running, and what is left until the code freeze on Sunday at 10:00. It is updated at each milestone. All team work up to 14:49 is in `main` ([pull request 2](https://github.com/dwn97/TaipeiDrift/pull/2)); the afternoon's work (save points 2 to 4, Dan's strait world, Ilhan's plan for the Tuniu photos) is in [pull request 3](https://github.com/dwn97/TaipeiDrift/pull/3), tested and ready to merge. Start new work from `main`; ours goes through branch `integration`. Every number comes from a script in this repository; the simulated results are in [simulation-results.md](simulation-results.md), the rest in [findings.md](findings.md). The code is under the MIT licence (`LICENSE`); the datasets keep their own licences.

## Save points: how we build from here

Since Saturday 14:20 we build in steps that each leave something working, so that nothing new can break what we already have.

- **A save point** is a tagged commit where `python scripts/check_save_point.py --record <name>` passed: every test, plus the key numbers, stored in `scripts/save_points.json`.
- **Everything new is built on the latest save point**, on branch `integration`.
- **Before a change is kept**, `python scripts/check_save_point.py` (about five minutes) must say "same as save point". If a number moves, the change is explained, and if it is better it becomes the next save point.
- **The demo uses the latest save point.** Something new replaces it only when it is itself a save point and measurably better.
- **At the code freeze, Sunday 10:00, the latest save point is the submission.**

| Save point | Tag | What works | Key numbers (flight 1, seed 3, 2018 map, unless said) |
|---|---|---|---|
| 1 | `sp1-merged-camera-navigator` | All team branches merged; camera navigator on ALTO, UAV-VisLoc and the simulated flight | 297 tests; UAV-VisLoc 03 seed 1: 28.0 m, 0 wrong fixes; simulated 26.5 m |
| 2 | `sp2-scale-kept` | The camera's scale learned before the jam is kept | Simulated 15.0 m; real data unchanged |
| 3 | `sp3-all-dev-flights-work` | Every development flight works: zooms of the heights flown, confirmation of jumps from 100 m | Simulated 18.2 m; development medians 22.0, 34.4, 30.3 m, no wrong fix |
| 4 | `sp4-sun-heading` | Heading from the simulated sun sensor instead of a compass | 304 tests; development medians 16.5, 32.3, 18.0 m, no wrong fix |
| Tried, not kept | | Noticing fog; fixes only at the learned scale; no fix while turning | None left every development flight working without making one worse; details in [simulation-results.md](simulation-results.md) |
| **Freeze, 18:46** | `frozen-navigator` | The navigator of save point 4. The two sealed flights, run once | **90 m flight: 36.3 m (camera alone 70.6). 110 m flight: 16.2 m (camera alone 57.6). No wrong fix, bound held 100 percent, with the ideal and the realistic camera** |
| Sunday 09:00 to 10:00 | | The phone's measured sun-compass error in the heading model | Reruns with the measured number |

## Now, in short

**Since 16:00: one solution, step by step** (Dustin: one change at a time, smallest test first, a save point after each). Every development flight now works, and the heading comes from the sun: development medians 16.5, 32.3 and 18.0 m over about 4.5 km without GNSS, with a two-year-old map, against 69, 94 and 64 m for the camera alone; no wrong fix; the stated bound held 100 percent. Stage by stage, with figures: [simulation-results.md](simulation-results.md). **Frozen at 18:46, and the sealed flights are run** (once, 18:46 to 18:52): the 90 m flight 36.3 m median (90 percent below 70 m, worst 90 m; camera alone 70.6 m), the 110 m flight 16.2 m (38 m, 58 m; camera alone 57.6 m); no wrong fix in any draw; the stated bound held all the time; the same with the realistic camera (21.8 and 15.3 m). Two limits we state: fog (at 1 km of visibility the error grows up to three and a half times and the bound fails 2 to 25 percent of the time), and turns with a realistic camera (on one development flight one fix 54 m off, taken while the drone turned on the spot). Noticing fog and two rules for turns were tried and not kept. **In `integration` since 19:27:** Ilhan's results on real photos, Dan's strait world with Alessandro's filter and the ships' radio fix, and Alessandro's filter with its GNSS velocity fit and city world (304 tests and 18 simulator tests pass; our navigator files are unchanged). All of it is in `main` since 20:22 ([pull request 4](https://github.com/dwn97/TaipeiDrift/pull/4)); start new work from `main`. **Since 20:10: the parts work together.** Alessandro's filter, fed with our sun heading, our camera's ground speed and the navigator's map fixes (`scripts/fused_replay.py`), run once on the sealed flights: 32.2 and 11.5 m median, against 36.3 and 16.2 m for the navigator alone; its stated error holds 98.2 and 99.7 percent of the time. Not yet live in the simulator. **In `integration` since 20:49:** Alessandro's second speed measurement, from the downward camera and a new range finder, live in the simulator and switched off by default (his one run without GNSS in the city world diverged: 12 m off after 30 s, 426 m after 136 s; his note `VERY IMPORTANT.md` says to leave it off, and our check in [simulation-results.md](simulation-results.md) traces the error to the buildings: one range for points at very different depths. Over the flat Wufeng ground, at 21:00, the same measurement was 0.16 m/s off in the median, and his filter with it stayed 14.5 m off in the median over 164 s without GNSS on a circle, against 322 m for the same filter without it on the same flight; one run, and the heading drifts 4.5 degrees per minute), Dan's map from the ships' bearings and a fixed four-antenna direction finder on the drone, and Felix's TRN fixes with a new `TestVideo/` demo (a camera on a cart positioned from the floor's texture), pushed straight to `main` at 20:49 (304 tests and 23 simulator tests pass; the strait world was started after the merges). None of our numbers change. **Since 22:10:** a second navigator by Ilhan (his branch `ilhan/sim-demo`, not merged) was checked on our development flights: 2 to 9 m median where ours has 16 to 32 m, also with nothing taken from the truth, but lost on one of the three flights and without a usable uncertainty; it runs in parallel as a second option, and ours stays the main solution. A 3.5-minute live run of the strait world on the current version: nothing crashed; the ships' bearings alone are 55 m off, the ESKF with the ships' fix 83 m and outside its own bound most of the time. Both in [simulation-results.md](simulation-results.md). Everything except Ilhan's new branches is in `main` since 22:15 ([pull request 5](https://github.com/dwn97/TaipeiDrift/pull/5)). Next: the demo video and the deck. Parked: the 3D world from the Tuniu photos, wind, rain, spoofing, the closed loop.

**The simulation, this afternoon (15:00 to 16:00; the demo will be the simulation).**

*The held-out test: flight 2* (80 m, 8 m/s, south first, 5.0 km; recorded after the settings were frozen at save point 1, run once at 15:05). The only change: the zooms tried at calibration were widened, because 80 m lies outside the range set for 100 m (the run stopped at our own check, before any result existed). Median over 3 compass seeds, in metres:

| | Flight 1, 100 m (developed on) | Flight 2, 80 m (held out) |
|---|---|---|
| Camera alone | 76 / 181 / 234 | 71 / 146 / 194 |
| 2018 map (the realistic case) | 26.5 / 75 / 92, 12 fixes | **20.7 / 47 / 79**, 13 fixes |
| 2020 map (the ground itself: an ideal bound) | 17.3 / 36 / 68, 17 fixes | 22.2 / 58 / 90, 15 fixes |

Median / 90 percent / worst. No wrong fix used in any run. With the 2018 map the stated bound held 99.4 to 100 percent of the time and the navigator never said "within 50 m" while being further off. With the ideal 2020 map it did, 0.2 to 1.1 percent of the time. The bound is wide: with the 2018 map it commits to "within 50 m" only 23 to 30 percent of the time.

*From here on: development, held-out and sealed flights* (fixed at 15:20 in `baseline/configs/sim_navigator.yaml`, before the new flights were recorded). Four more flights, all along the same corridor, differing in height, speed and direction: two for development (120 m and 65 m), two sealed (90 m and 110 m) that are neither run nor looked at until the end, then run once; those are the numbers we quote. A change is kept only if it works on every development flight and passes `scripts/check_save_point.py` (the real data). "Works", for the 2018 map on every flight and seed: no wrong fix used, the true error within the stated 3 sigma at least 99 percent of the time, never "within 50 m" while further off. Check: `python scripts/sim_dev_check.py [--camera realistic] [--set name=value]`.

*The first change found this way: keep the camera's scale.* After each fix the navigator re-read the height from the fix's zoom, in steps of 0.05, so one step changed the distance flown by 6 to 13 percent until the next fix. Our drone holds its height, so the scale learned before the jam is kept (`scale_from_fixes: false`). 2018 map, median / 90% / worst in m:

| | Flight 1, 100 m | Flight 3, 120 m |
|---|---|---|
| As before | 26.5 / 75 / 92: fails (bound 98.3%) | 30.9 / 65 / 112: fails (3 wrong fixes, 5.3% "within 50 m" while further off) |
| Scale kept | **18.5 / 44 / 66: works** | **28.6 / 63 / 113: works** |
| Scale kept, realistic camera | **18.0 / 40 / 68: works** | **27.4 / 53 / 79: works** |

Kept once the third development flight (65 m) agrees.

*More realism.*
- **A realistic camera** (`baseline/src/data/camera_model.py`, `--camera realistic`): cloud shadows drifting with the wind, haze, vignetting, a small lens distortion, vibration blur, auto-exposure, sensor noise, JPEG. It costs little (table above): the matcher compares patterns after removing brightness and contrast. The limits test of this morning found the real blind spot: blur of about 4 m and more.
- **Wind** (`wind:=6,20` in the simulator launch: 6 m/s from north-north-east, the October monsoon direction, with gusts): the drone leans into it, so the camera's view moves sideways by about h tan(lean), differently on every leg. Test at 16:30, then a windy set of flights.
- **A 3D world from real drone photos.** Two flights of the same stretch of the Tuniu River in Toufeng, Miaoli, by Yu-Huang Wang (OpenDroneMap's example data; DJI Phantom 4 RTK, 100 m, camera tilted 60 degrees; no licence stated, so used for testing and not redistributed): 11 April and 16 September 2019. OpenDroneMap turns the September photos into a textured 3D model, the simulated world (hills, forest, buildings with real height), and the April photos into an orthophoto, the navigator's map, five months older across a typhoon season. Processing since 15:47; the world in about an hour.

*The plan, each step a save point* (if a step is not measurably better by its time, we stay on the last save point): realism and the scale fix by 18:00; the fused navigator (Alessandro's filter with IMU, barometer, sun sensor and tilt-corrected fixes) by 21:30; closed loop (the drone steers by our estimate), spoofing detection and a live map by 01:00; overnight the sealed flights run once, README, figures, video; Sunday 09:00 to 10:00 the phone sun test and the freeze. Open with Dustin: the pitch story (GNSS spoofed, then jammed; the navigator runs from take-off and takes over), and whether Dan builds 3D trees and buildings for Wufeng.

**Earlier today.** The system works on real data from two countries, and we now know how well. On the flights it was developed on, it holds about 30 m where the camera alone drifts 470 to 820 m. On flights it never saw, the fixes cut the drift by 40 percent (ALTO, 219 to 131 m, Ilhan's test at 13:00) to tenfold (UAV-VisLoc 04, 675 to 60 m); where the ground was rebuilt since the map (UAV-VisLoc 01) they do not help. Fielded systems claim 15 to 20 m. **Section 2 explains in detail what the system does and what it gives a user.** All team branches now sit in one branch, `integration` (297 tests pass), up for merging as pull request 2.

**The simulated flight over Wufeng (new, 14:00).** Dan's simulator with Ilhan's patch; the real Wufeng 2020 aerial photo as the ground, the 2018 photo of the same place as the navigator's map; a 4.8 km route along the motorway corridor at 100 m and 10 m/s, with a 180-degree turn; GNSS lost after 450 m; camera motion is real optical flow on the simulated images, the heading a simulated compass (median over three compass error draws; `python baseline/scripts/run_sim_navigator.py`):

| | Median error | 90% below | Worst | Fixes used (wrong) | Error within the stated 3 sigma |
|---|---|---|---|---|---|
| Camera alone | 76 m | 181 m | 234 m | | 100% |
| Map fixes, 2018 map (two years old) | **26.5 m** | 75 m | 92 m | 12 (0) | 100% |
| Map fixes, fresh 2020 map | **17.3 m** | 36 m | 68 m | 17 (0) | 100% |

- **The price of an old map, measured:** 26.5 against 17.3 m, and 12 against 17 accepted fixes. This is the Eagle Eyes lesson in numbers.
- **A camera fixed to a quadcopter looks backwards when the drone tilts forward** (9.5 degrees in cruise, up to 16 in turns): each fix lands 16 to 22 m behind the drone. Corrected with the drone's true tilt, the fixes are 2 to 5 m off. The IMU knows the tilt, so this is the measured case for fusing with Alessandro's filter; the camera-only navigator absorbs only the steady part, as a learned offset.
- **The same tilt broke the first run** (no fix used, 1 km drift): the banking turns before the jam spoiled the learned motion scale. Fixed with a fit of one turn and one scale from medians (`motion_fit: rotation_scale`, tested); ALTO and UAV-VisLoc keep the old fit, so their numbers do not change.
- Demo clip: `outputs/replay/sim_map_2018_seed3.mp4` (seed 3 is the median of the three), stills next to it.
- The simulated ground is a flat photo: no 3D, no change of light, no wind. The test is easier than a real flight, and the 2018 map carries real changes on the ground.

**Alessandro's push at 13:58:** a new measurement for his filter, the direction of travel seen by the forward camera (direction only, no speed; 1.0 degree median error on Mid-Air), his baselines rerun with Ilhan's measured barometer, and a transfer test on NTU VIRAL (real data, no retuning). Numbers not in the repository yet. **Dan's push at 13:14:** a recorder that writes simulated flights in the Mid-Air format, so Alessandro's code reads them unchanged; his simulator is now in `main`.

**The new checks, on the development flight** (UAV-VisLoc 03, 74 km without GNSS, median over 3 seeds; "all clear while wrong" = the share of the flight where the navigator's stated bound is within 50 m but the true error is above it):

| | Median error | 90% below | Worst | Wrong fixes used, per seed | Error within the stated 3 sigma | All clear while wrong |
|---|---|---|---|---|---|---|
| Dead reckoning only | 822 m | 1,391 m | 1,571 m | | 100% | 0% |
| First version (as tuned on ALTO) | 41 m | 427 m | 1,301 m | 11, 12, 12 | 73% | 2.5% |
| **New: compass, confirmation of large jumps, offset in the drone's frame, search capped at 600 m** | **28 m** | **180 m** | **368 m** | **0, 0, 2** | **97%** | **0.0%** |
| New, with the sun sensor instead of the compass | 33 m | 162 m | 395 m | 0, 2, 2 | 96% | 0.3% |

On ALTO the new checks change nothing at fixes every 300 m (31.1 m). The sun sensor does not help on this flight: here the heading is not the main error, which matches Ilhan's ALTO finding (6 percent). One seed stays poor (median about 200 m) without wrong fixes: long stretches without an accepted fix, which the navigator reports honestly as uncertain.

**The held-out test** (flights 01 and 04, never looked at before, method frozen at commit f66d4b0, run once; median over 3 seeds):

| Flight | | Median error | 90% below | Worst | Wrong fixes used, per seed | Error within the stated 3 sigma | All clear while wrong |
|---|---|---|---|---|---|---|---|
| 01 Changjiang, 66 km, map 5 years newer | Dead reckoning only | 227 m | 596 m | 973 m | | 100% | 0% |
| | First version | 474 m | 1,604 m | 2,051 m | 17, 18, 24 | 48% | 3.1% |
| | New, compass | 306 m | 851 m | 1,493 m | 6, 7, 4 | 97% | 0.9% |
| | New, sun sensor | 177 m | 556 m | 1,514 m | 5, 6, 3 | 98% | 1.0% |
| 04 Taizhou, 83 km, map 4.5 years newer | Dead reckoning only | 675 m | 1,235 m | 1,805 m | | 100% | 0% |
| | First version | 285 m | 1,592 m | 2,244 m | 37, 31, 53 | 39% | 6.5% |
| | New, compass | 60 m | 1,723 m | 2,620 m | 18, 27, 32 | 77% | 3.0% |
| | New, sun sensor | 64 m | 1,206 m | 1,860 m | 17, 29, 35 | 76% | 3.2% |

What it says:

- **The new checks are clearly better than the first version on unseen flights:** wrong fixes down by a third to three quarters, and the stated uncertainty far more honest (97 instead of 48 percent on flight 01, 77 instead of 39 on flight 04).
- **They are not good enough yet.** On flight 04 the typical error drops tenfold against dead reckoning (675 to 60 m), but 18 to 32 wrong fixes still pass and the worst stretches are worse than without fixes. On flight 01, map fixes do not beat dead reckoning at all.
- **Why flight 01 fails:** it runs through a fast-growing area along the Yangtze. Bare construction sites in the 2018 photos are high-rise estates on the 2023 map; no matcher can recognise that ground. Raptor's documentation and the UASTHN paper name the same limit: a map older than the change on the ground.
- **The sun sensor helps here** where the compass is the weak part: median 177 instead of 306 m on flight 01, 90 percent below 1,206 instead of 1,723 m on flight 04.

**Ilhan's held-out test on ALTO, without the leak** (pushed 13:00, branch `research/offline-nav-evidence`, script `experiments/w_dustin_heldout.py`). Our navigator as of this morning (commit 83adc59, before the new checks), frozen, on the 8 ALTO Round 2 sections it never saw (37.4 km). It first reproduced our validation numbers exactly.

| Fixes | Validation section (ours) | 8 unseen sections: median of the section medians (range) | Fixes used, of them wrong | Error within the stated 3 sigma (median section, worst) |
|---|---|---|---|---|
| None (camera alone) | 472 m | 219 m (68 to 420) | | 100%, 75% |
| Every 300 m, score check | 31 m | 131 m (26 to 1,135) | 47, 6 wrong (76 refused) | 70%, 15% |
| Every 1,000 m, score check | 56 m | 98 m (43 to 783) | 16, 3 wrong | 82%, 24% |
| Every 100 m, no check | 25 m | 50 m (17 to 437) | 289, 95 wrong | 39%, 9% |

- Our 31 m on ALTO is the section it was tuned on. Across unseen sections the result runs from 26 to 1,135 m, median 131 m: the fixes cut the error by 40 percent, and the stated uncertainty is too small in some sections. Two thirds of the fix attempts are refused.
- Ilhan's suspect from last night: the zoom, learned from three fixes, goes wrong when the helicopter changes height. A height sensor (barometer) would set it.
- The new checks (confirmation of large jumps, search cap, camera-motion floor) were developed on UAV-VisLoc and have not run on these sections yet; asked of Ilhan.

**How the plan changed this morning**

| Before | Now | Why |
|---|---|---|
| Wait for Dropbox to test on ALTO Train | Test on a second dataset from China (UAV-VisLoc) | The jury scores technical validity; Dropbox was blocked |
| Navigator ignores the heading | Navigator uses the drone's heading (compass, or the mentor's sun sensor) | The brief lists heading, every drone has it, and without it we fail at the first turn |
| Next step: blurred and dark frames | First: make the check against wrong fixes hold on unseen flights | Both held-out tests (ours and Ilhan's) show it is the weak point |
| Sun sensor as a "future" slide | Sun sensor simulated now; the mentor's phone test measures it for real | The mentor's idea; gives a measured number |

**Similar systems in the field** (researched at 13:00; details and sources in [landscape.md](landscape.md#eagle-eyes-and-the-systems-fielded-in-ukraine)):

- Ukraine's special forces fly our idea at scale. Their "Eagle Eyes" matches live video of the ground against a map stitched from recent reconnaissance flights; in use since 2023, no accuracy published. The Tomahawk missile did the same in the 1980s. So we say openly that the idea is proven, and claim only the measured, honest weekend version.
- Fielded systems that publish a number claim 15 to 20 m. We are at 28 to 31 m on the development flights and 60 m on held-out flight 04.
- Their map is fresh; ours is years old, and that is exactly why flight 01 fails. The simulator will show the price of an old map: the same flight with the 2020 image and with the 2018 image as the map.

**Achieved**

- **USA (ALTO, real helicopter flight):** camera alone ends 472 m off (median); with map fixes 31 m. A flaw in our test (the search knew the true path) was found and removed; the numbers did not change. On the 8 unseen ALTO sections (Ilhan): 219 m to 131 m.
- **China (real drone photos against a map 2.5 years older):** the matcher, unchanged, finds 80 percent of the photos within 30 m.
- **74 km without GNSS:** median 27 m, against 1,041 m without fixes. But about 11 wrong fixes slipped through and the drone got lost in stretches: our honest weakness. Ilhan found the same independently on unseen ALTO data (94 m instead of 31 m).
- Built against it: confirmation of large jumps by the next fix, Ilhan's quarters check, the fix offset in the drone's own frame (fix error 19 to 13 m), compass and sun-sensor models, the aviation integrity measure.

**The limits: a worse camera picture** (the brief's "limits when sensors fail or noise rises"; ALTO, frames made darker, blurred or hazy after the jam; `python baseline/scripts/run_limits.py`, chart `outputs/limits_floor_0.3/before_after.png`):

- **It holds** down to 1/64 of the light (31 m, no wrong fixes), blur up to 2 m, haze down to half the contrast.
- **A dangerous failure was found and closed.** When the picture is bad enough that the camera cannot see the motion any more, the estimate used to stand still while its stated uncertainty stopped growing: "tracking, sure to a few metres" while 1 to 2 km off. Now a camera step shorter than 30 percent of the cruising step (learned with GNSS) is not believed: the navigator flies on at cruising speed along the last good direction and its uncertainty grows three times faster. Haze with 10 percent of the contrast left: 1,640 m and a bound that held in 0 percent of frames before; 54 m, status LOST, bound held in 100 percent after. Clean frames: unchanged.
- **One blind spot remains:** moderate blur (4 m), where the camera's motion is wrong but looks plausible; 4 wrong fixes pass. Catching it needs a second motion source to cross-check, which is what Alessandro's IMU filter does. The fallback flies on in the last good direction, which suits a straight flight; through turns it needs the heading sensor.

**Demo clips (ready)**, made with `python baseline/scripts/make_replay.py alto` and `... visloc --flight 04`; the videos are in `outputs/replay/` on Dustin's laptop (not in git, 20 MB each):

- `alto_val.mp4`, 26 s: the real ALTO flight, camera motion and map fixes every 300 m.
- `visloc_04_confirm_body_seed2.mp4`, 30 s: the unseen flight 04 with the frozen method (seed 2, the median of the three), over survey legs with turns; fixes used in green, refused in red, the search circle, the status.
- Still images at 25, 50, 75 and 100 percent of each run, for the slides.

**Asks to the team**

- **Ilhan:** ~~rerun the ALTO Train test with the map search~~ done at 13:00 (131 m). Next: the same with the current navigator, and the zoom from the height.
- **Alessandro:** the heading error of the visual-inertial odometry after 30, 60 and 80 s without GNSS; it becomes our compass model. And what the "graph-map layer" in your architecture sketch is, before anyone builds it.
- **Anyone with an iPhone, today until 14:30 or tomorrow 09:00 to 09:45:** the mentor's sun-compass test (below).
- **Dustin:** the story and slides; ask the organisers what the brief's "suggested dataset" is.

**The mentor's sun-compass test with an iPhone** (code ready: `baseline/scripts/phone_sun_compass.py`). The full plan for the teammate, with steps, timing and how the number feeds into the navigator: [phone-sun-test.md](phone-sun-test.md). Two corrections to the steps written this morning:

- **Use the 0.5x camera, for the chessboard photos too.** Lying flat, the 1x camera only sees about 35° from straight up, so the sun is in the picture only around noon; the 0.5x reaches about 50°.
- **The sun must stand above about 40°:** at NTU today until about 14:30, tomorrow from 09:00. Tomorrow's window ends with the code freeze.
- The script now reports the error of one heading reading (the spread of the eight readings around their 90-degree steps); before, it counted the first photo's error twice.

## 1. The goal for Sunday

**One sentence:** a small drone that loses GNSS keeps its position with its own downward camera and a free satellite map, and it says honestly when it is not sure.

What the jury scores (the challenge brief), and how we answer each point:

| The brief scores | Our answer | State |
|---|---|---|
| Reduction in positioning error | Where developed: 472 → 31 m (ALTO), 822 → 28 m (UAV-VisLoc 03). Never seen: 219 → 131 m (ALTO, 8 sections), 675 → 60 m (UAV-VisLoc 04); no gain where the ground changed (01) | Measured |
| Technical validity | Tested on flights we never tuned on, from another country, camera and map source | Done: UAV-VisLoc 01 and 04 (ours), ALTO Round 2 (Ilhan) |
| Noise tolerance | Error as the picture gets darker, blurred, hazy, and as the heading sensor gets worse | Done: light, blur, haze (top of page); compass against sun sensor |
| Computing and integration | One laptop core: 11 ms per camera frame; one map fix 0.07 s for the smallest search (60 m) up to 2 s for the widest (600 m), against about 30 s between fixes; output is a position with an uncertainty, the form an autopilot takes | Measured (`baseline/scripts/time_navigator.py`) |
| Deployment feasibility | Free maps (Taiwan's government orthophotos), an ordinary camera, no GPU | Slide |
| The user | Operators of small drones near Taiwan's coast and islands, where GNSS is jammed | Slide |

The required parts of the brief: a dead-reckoning baseline, at least one correction, plots of estimated against true path and of error over time, and the limits. We have all four on ALTO; the held-out test adds them on a second dataset.

## 2. The system in detail: what it does, and what it gives the user

Updated at 17:40 for the system at save point 4 (`sp4-sun-heading`), so that everyone on the team can explain it in their own words. Every number below is a setting the code really uses (`baseline/configs/sim_navigator.yaml`, `baseline/src/`) or a measured result; the simulated results are in [simulation-results.md](simulation-results.md). One step is still under test: the navigator noticing fog (marked below).

**In one picture**

```
 sun sensor ────────┐ heading (the gyro carries it while a cloud hides the sun)
                    ▼
 camera motion ──► dead reckoning ──► estimate + stated uncertainty ──► status: TRACKING / DEGRADED / LOST
                    (drifts ~10 %)        ▲            ▲
 picture detail ──── widens the uncertainty in fog (under test)
                                          │ used only if it passes the checks
 camera frame ──► search the map around ──┘   score high enough? close enough to the estimate?
                  the estimate (a fix)         a large jump confirmed by the next fix?
```

### 2.1 The user and the problem

- **The user:** anyone who flies small, cheap drones where GNSS is jammed or spoofed. In Taiwan: around the outlying islands, where interference is reported regularly, and in any conflict. Examples: coast and island surveillance, disaster response, military reconnaissance.
- **The problem:** without GNSS the drone has only its own sensors, and they drift. The camera alone ends up 472 m off on ALTO and 822 m off on UAV-VisLoc flight 03 (median); on our simulated flights 64 to 94 m after about 4.5 km, still growing; the IMU alone 338 to 912 m after 83 s on Mid-Air (Alessandro). The drone cannot hold its route, find its target or come home, and nobody tells the operator how far off it is.
- **What the drone already carries:** a camera, an IMU, a barometer, a small computer, often a compass. What it lacks is a position it can trust.
- **When it runs:** from take-off, quietly. While GNSS works it learns how its camera behaves (below); when GNSS is lost it takes over. A spoofed GNSS does not announce itself, so the right design compares GNSS with the camera and the map all the time and stops trusting GNSS when they disagree. That check is a design point, **not built**.

### 2.2 One flight, step by step

**Before take-off.** The drone stores an aerial or satellite image of the area as its map, in grey. In the simulation it is 0.5 m per pixel: 1 km² is 2,000 × 2,000 pixels, 4 MB. At 1 m per pixel it is 1 MB, and all of Taiwan, about 36,000 km², is 36 GB before compression; it fits on a memory card. Our map is two years older than the ground the drone flies over.

**While GNSS still works** (the first 450 m in the simulation; 300 m on ALTO, 1,000 m on UAV-VisLoc), the system learns by comparing the camera with GNSS. Nothing is calibrated by hand.

1. *How a slide of the picture becomes metres.* If the picture slides 10 pixels while GNSS says the drone flew 5 m north, one pixel of slide is 0.5 m. On a quadcopter the fixed camera tilts in every turn, so the system learns one turn and one scale from medians, which a few tilted frames cannot spoil (`motion_fit: rotation_scale`).
2. *How the map must be zoomed and turned* to look like the camera picture. The zoom follows from the height: 100 m gives a zoom of 0.78 on our 0.5 m map. The drone holds its height, so this scale is kept for the whole flight (save point 2).
3. *The fix offset:* where a matched position lies compared with the GNSS position. A quadcopter leans forward 9.5 degrees in cruise, so its camera looks about 16 m behind it at 100 m. The offset is learned in the drone's own frame (forward, right), so it turns with the drone.
4. *The cruising speed,* for the case "camera blind" below.
5. *How much fine detail its pictures normally hold,* to notice fog later (under test).

**GNSS is lost. Every camera frame: dead reckoning.**

- The system measures how far the picture slid since the last frame, turns that into metres, turns it to north and east with the heading, and adds it to the position.
- **The heading comes from the sun**, not from a compass: the drone's own motors and currents make a magnetic field that changes with the throttle. With the date, the time and a rough position, the sun's direction in the sky is known. A sensor on top of the drone (Fan et al. 2016: 0.1 degree, 35 g, 0.2 W) measures the sun's direction against the airframe; turned level with the tilt from the IMU, the difference is the heading. Example: the sun stands at bearing 215 and the sensor sees it 35 degrees right of the nose: the nose points to 180. Simulated on flight 1: 0.9 degree median heading error and no fixed offset, against 1.8 degrees and offsets up to 8 degrees for a compass.
- **Where the sun heading stops:** the sensor sees the sun only within 65 degrees of straight up, so a sun lower than 25 degrees (early morning, late afternoon) gives no reading. A sun right overhead says nothing about north: in Taiwan, near the Tropic of Cancer, that is around noon from May to July. Under a cloud the gyro carries the heading on until the sun is back.
- Every step carries a small error, and the errors add up: about 10 percent of the distance flown. The system says so. Next to the position it keeps σ (sigma), its own statement of how far off it may be, in metres. If σ is honest, the true error stays below 3σ almost always; our tables check exactly that ("bound held").
- How σ grows, from zero. Independent errors add as squares. Right after a fix σ is about 13 m. After 300 m more, the dead reckoning has added 10 percent of 300 m, which is 30 m. The new σ is √(13² + 30²) = √(169 + 900) = 33 m.

**Every 300 m: a position fix.**

- The camera picture is compared with the map in a circle around the current estimate. The radius is 3σ, at least 60 m and at most 600 m: with σ = 33 m it is 99 m.
- The comparison slides the picture over every position in the circle and scores how well the pattern of light and dark agrees (normalised correlation). The score ignores how bright the picture is and how strong its contrast is. Three pixels in a row as an example: 10, 20, 30 against 110, 120, 130 score 1.0 (the same pattern, only brighter); against 30, 20, 10 they score −1 (the pattern reversed). Zooms for heights of about 58 to 134 m and angles around the learned ones are tried. The best place is the candidate fix.
- **Three checks.** The candidate is used only if
  1. its score is at least 0.33;
  2. it lies within 3σ of the estimate (99 m in the example);
  3. when it comes from a search wider than 100 m and would move the estimate by more than 30 m, the next fix, 300 m later over different ground, agrees with it. A look-alike place rarely has a matching look-alike 300 m further on. (Since save point 3 this check starts at 100 m instead of 150 m: it removed the last wrong fixes on the development flights, at the price of a few metres of median.)

**Using a fix.** A fix has its own uncertainty, σ = 15 m. The estimate moves towards the fix by how unsure it is compared with the fix. Estimate σ = 30 m (squared: 900), fix σ = 15 m (squared: 225): the estimate moves 900 / (900 + 225) = 80 percent of the way, and its new σ is √(0.2 × 900) = 13.4 m. A fix that waited for confirmation is used first, then the one that confirmed it.

**What it reports, every frame:** a position (north and east), σ, and a status.

| Status | When | With a fix every 300 m |
|---|---|---|
| TRACKING | σ up to 30 m | From each fix (13 m) until about 270 m later |
| DEGRADED | σ from 30 to 100 m | The last 30 m before each fix; after one missed fix σ is 62 m |
| LOST | σ above 100 m | After about 1 km without an accepted fix |

A position with its uncertainty is the form in which an autopilot takes a position from an outside source. The link to a real autopilot is not built (a small adapter to the standard MAVLink message is proposed, after the freeze).

**When the camera stops seeing the motion.** If a frame's step is shorter than 30 percent of the cruising step learned with GNSS, the camera is not believed: the system flies on at cruising speed in the last good direction, and σ grows three times faster. It never stands still while claiming to be sure. **Fog (under test):** fog washes out the picture's fine detail (54 percent of it is left at 1 km of visibility, 21 percent at 300 m). With a share s of the usual detail, σ grows 1/s times faster, up to five times; below 25 percent the camera is not believed, as above. Before this step, fog at 1 km made the stated bound fail 18 percent of the time.

### 2.3 What it gives the user, and the evidence

| The user gets | Evidence (median error) | The honest limit |
|---|---|---|
| A bounded error with a two-year-old map, in simulation | Development flights at 100, 120 and 65 m, about 4.5 km each without GNSS: 16.5, 32.3, 18.0 m with the map against 69, 94, 64 m with the camera alone; no wrong fix, bound held 100 percent | Simulated: flat ground, no wind; the settings were tuned on these flights. Worst cases 72 to 118 m |
| The same on a simulated flight it never saw | Flight 2 (80 m), run once: 20.7 m, worst 79 m, no wrong fix | One flight; two more are sealed and run at the freeze |
| A position within tens of metres on real data | Developed on: ALTO 472 → 31 m; UAV-VisLoc 03 (74 km) 822 → 28 m | The settings were chosen on these flights |
| On real flights it never saw, a drift cut by 40 percent to tenfold | ALTO, 8 sections (Ilhan): 219 → 131 m; UAV-VisLoc 04 (83 km): 675 → 60 m | Wrong fixes still passed there: 18 to 32 on flight 04 (before today's checks) |
| Nothing where the ground has changed since the map | UAV-VisLoc 01: no gain over dead reckoning | Needs a fresher map, as Eagle Eyes has |
| A warning when it is unsure | σ and status every frame; on the development flights the true error stayed within 3σ 100 percent of the time | With a 50 m limit it commits to "within 50 m" only about a quarter of the time |
| A heading without a magnetometer | Sun sensor instead of compass: development medians 22.0 → 16.5, 34.4 → 32.3, 30.3 → 18.0 m | No sun below 25 degrees or overhead; a few-dollar photodiode sensor was only as good as a compass |
| A safe reaction when the camera goes blind | Haze with 10 percent of the contrast left (ALTO): 1,640 m and "sure" before this morning's change; 54 m and LOST after it | Fog: under test (above). Blur of 4 m still fools the camera's motion |
| No vendor, nothing to jam or to detect | Free aerial images, an ordinary camera, no radio emission | Daylight and land only |

### 2.4 What it costs the user

- **Sensors:** a downward camera; a sun sensor (35 g, 0.2 W) or, cheaper and less accurate, five photodiodes; the IMU and barometer every drone has; GNSS for the first few hundred metres. No magnetometer needed.
- **Map:** free aerial images, 4 MB per km² at 0.5 m per pixel.
- **Computing:** one laptop core, no graphics processor: 11 ms per camera frame (94 frames per second possible, the camera gives 5 in our simulation); one map fix 0.07 s for the smallest search (60 m), 0.27 s at 150 m, 0.8 s at 300 m, 2 s at the 600 m cap. A fix comes about every 30 s, so even the widest search uses under 7 percent of one core. Measured under Intel emulation on an Apple Silicon Mac; not yet timed on a drone's small computer.
- **Setup:** none by hand; it calibrates itself while GNSS works. That also means a drone jammed from take-off is not covered.

### 2.5 What it does not do yet, and what would close each gap

| Gap | What would close it | State |
|---|---|---|
| Fog and low cloud (measured: at 1 km of visibility the error grows four times; at 300 m the camera loses the ground) | Honesty: σ that grows with lost picture detail (under test). Accuracy: other sensors: thermal camera, radio | Honesty step under test; sensors not started |
| Night | A thermal camera against the same map, as Raptor does; matching needs learned features (the STHN paper) | Not started |
| Changed ground | A fresher map: recent reconnaissance or the drone's own earlier flights (Eagle Eyes) | Shown in simulation: with the 2020 map (the ground itself) the error is about the same, so on this corridor the map's age is not what limits us |
| Height changes | The scale from the barometer's height instead of a fixed scale | The simulated drone holds its height; not needed there |
| Wrong fixes | The confirmation check (done: none on the development flights); two matchers that must agree (Ilhan's plan on the Tuniu photos) | Ilhan's code and results not pushed yet |
| Water, the Strait | Positions from ships' radio (Dan's strait world in the simulator) | In the simulator; not combined with ours |
| Link to the autopilot | MAVLink `GPS_INPUT` with our σ as its accuracy | Proposed, after the freeze |
| Spoofing | Compare GNSS with camera and map from take-off | Design point, not built |
| Jammed from take-off | A start without GNSS | Not covered |

### 2.6 Against what a buyer can get today

Sources in [landscape.md](landscape.md).

| | Ours (weekend prototype) | VNS01 (UAV Navigation) | AIDC AIxVNAV (Vantor's Raptor inside) | Eagle Eyes (Ukraine) | Osiris (Greece) |
|---|---|---|---|---|---|
| Map | Free aerial images, two years old | Preloaded satellite imagery | Vantor's licensed 3D data | Own recent reconnaissance flights | Preloaded satellite images |
| Hardware | Ordinary camera, sun sensor, one CPU core | Dedicated 100 g unit | Camera and a graphics processor | Not public | Module under 300 g, 25 W |
| Accuracy | Simulated: 16 to 32 m median on development flights, 21 m on one unseen flight; real data: 28 to 31 m where developed, 60 to 131 m unseen | About 30 m with satellite map matching (maker, September 2026) | Raptor: 12 m mean in one published test | Not public | 15 m CEP (maker) |
| Says when unsure | σ and status every frame, measured against the truth | Not public | A confidence from 0 to 1 per frame | Not public | Not public |
| Night | No | No claim | With an infrared camera | Not public | Daytime figure |
| Maturity | Recorded and simulated flights | Product | Flight-tested in Taiwan, 2026 | In combat since 2023 | About 3,000 km of trials in Ukraine |

**In one sentence for a customer:** for cheap drones where GNSS is jammed, it keeps the position within a few tens of metres for as long as the drone flies over mapped ground, with the camera it already has, a sun sensor of a few grams and a free map that may be years old, and it says when it is not sure. A buyer who needs accuracy, night or fog today buys a fielded system; ours is the open, low-cost version with measured limits.

## 3. What we did this morning, and why

**A. We checked our own work for mistakes before building more on it.**

- All seven numbers in our docs reproduce exactly, and all tests pass.
- **Mistake found and repaired:** ALTO's reference images are centred on the *true* flight path. Searching them told the navigator where the path was. We now build one map from all reference images and search it around the navigator's own estimate. The results changed by less than half a metre, so they hold (findings 3.8).
- **Gap found:** the ALTO validation flight is a straight line. Our navigator had no heading input and would have failed at the first turn. Every real mission has turns.
- **Weakness known:** all settings were chosen on the same 4.6 km that we reported on.

**B. We added a second, very different dataset to test on: UAV-VisLoc** (Xu et al. 2024, from the Chinese papers we found).

- Real drone photos over China, taken in 2018, matched against a Google Earth satellite map taken 2.5 to 5 years later. Fields, villages and roads have changed in between. This is the realistic case: a map is always older than the flight.
- Each flight is up to 80 km long, with many turns, and gives the heading for every photo.
- Why it matters: "technical validity" means showing that the method works on data it was not tuned on. Dropbox blocks the rest of ALTO today, and this is a stronger test anyway: another country, camera and map source.
- What is real and what is simulated: **the fixes are real** (real photos against the real map). The photos are 95 m apart, too far for our camera motion, so the **dead reckoning between photos is simulated** with the error we measured on ALTO plus the error of the heading sensor. We say this on the slide.

**C. What the first test on that dataset showed (flight 03, 74 km)**

- **The matcher transfers.** Without changing anything, 80 percent of the photos are found within 30 m of the truth, median error 16 m.
- **The navigator works most of the time:** median error 27 m over 73 km without GNSS, against 1,041 m for dead reckoning alone.
- **But the check let about 11 wrong fixes through**, and after each the navigator stayed lost for kilometres (90 percent of the time below 356 m, worst 1,301 m). We traced why: after a few rightly refused fixes, the uncertainty grows, the search widens to 300 to 450 m, and in that wide area a look-alike place with a borderline score is found and believed. This is the most important finding of the day. It is what a held-out test is for.

**D. Two findings that need the heading**

- **The fix offset turns with the drone.** The matched position lies about 13 m ahead of the recorded one in the direction of flight, on every leg. Learned in the drone's own frame (forward, right) instead of north and east, the median fix error falls from 18.6 to 13.1 m.
- **The camera points along the drone's nose, which differs from the direction of travel by the wind's crab angle (up to 13 degrees).** A heading sensor measures the nose direction, which is the one we need.

**E. Ilhan's overnight research** (branch `research/offline-nav-evidence`, pushed at 10:50, summary in French in `docs/research/overnight-synthesis.md`). It reaches the same conclusion by another route:

- **Held-out ALTO test done.** He downloaded ALTO Round 2 Train (37.4 km, with positions) through the second Dropbox link. With our navigator frozen and fixes every 300 m, the median error per section is 94 m (20 to 339 m), against 31 m on the validation section; only 3 of 8 sections reproduce it. His runs still used the search that knew the true path; they need repeating with the map search, which on the validation section changed nothing.
- **Main cause there: the zoom** (image scale), learned from three fixes, goes wrong when the helicopter's height changes. With a better zoom, right fixes rise from 21 to 43 percent to 86 to 100 percent in three sections.
- **His integrity rule "quad ≥ 3":** cut the frame into four quarters; at least three must land where the whole frame landed. On 300 real ALTO frames: 44 accepted, 0 wrong. Learned matchers (XFeat) failed on real aerial images; matching OpenStreetMap roads, magnetic anomalies and a shadow compass were tried and dropped.
- **Taiwan:** forest covers 76 percent of the island, where map fixes are unlikely.

Two independent held-out tests, his on ALTO and ours on UAV-VisLoc, say the same: the first version does not carry over to unseen flights, and the score threshold is its weak point.

## 4. What we are testing now, and what we expect

| Method | What it does | Why | What we expect | State |
|---|---|---|---|---|
| Map search around the estimate | Search one map in a circle of 60 m or 3 sigma around the estimate | Removes the knowledge of the true path | Same results as before | **Done**, confirmed on ALTO |
| Confirmation of large jumps | A fix that would move the estimate far, out of a wide search, is held until the next fix over different ground agrees; then both are used | A single look-alike place rarely repeats 300 m later | On UAV-VisLoc: far fewer wrong fixes used, no more long lost stretches. On ALTO: little or no cost | **Testing now** |
| Offset in the drone's frame | Learn the fix offset in forward/right, turned with the heading | The camera's tilt or trigger delay is fixed to the drone | Fix error about 13 m instead of 19 m | Built, testing now |
| Heading sensor: compass | Recorded heading plus a fixed offset (about 4 degrees) and noise | What every drone has | The baseline for turns | Built |
| Heading sensor: sun (Fan et al. 2016, the mentor's paper) | Heading error from the sun's elevation: 0.1 degree sensor, 1 degree tilt | Better heading, no magnetic disturbance | Less sideways drift between fixes; useless when the sun is overhead | Built |
| Quarters must agree (Ilhan's "quad ≥ 3") | The four quarters of the frame, searched alone, must land with the whole | No tuned threshold; 0 wrong on his benchmark | Fewer wrong fixes, but also fewer fixes | **On ALTO too strict:** at 300 m spacing it refuses 6 right fixes and the worst error rises from 73 to 344 m; at 1,000 m it refuses all |
| Integrity measures (Zhu et al. 2022) | Share of the flight where the navigator says "tracking" while the error is above the alert limit (50 m) | The aviation standard for "knows when it is wrong" | Near zero with the confirmation, clearly above zero without | Built, runs with the next test |

**The sun sensor in numbers** (our own calculation, checked against the standard solar position library):

| When and where | Sun's elevation | Heading error on a drone |
|---|---|---|
| UAV-VisLoc flight, morning in October | 27 degrees | about 0.5 degrees |
| Taipei today at noon | 61 degrees | about 1.7 degrees |
| Taipei at noon on 21 June | 88 degrees | about 29 degrees: useless |

On a drone the sensor's own 0.1 degrees hardly matter; the tilt the IMU reports dominates. In Taiwan the sun stands nearly overhead around midday from May to July.

## 5. The clean test

To keep the held-out test honest, we fixed the test flights **before** looking at them: UAV-VisLoc flights 01 (Changjiang, 817 photos) and 04 (Taizhou, 738 photos), downloaded and set aside. Flight 10 was meant to be the third; Google Drive throttles its download. We develop only on ALTO and flight 03. Once the method is frozen, it runs once on 01 and 04, and we report what comes out, good or bad. Done at 12:20; the result is at the top of this page.

## 5a. Alessandro's visual-inertial odometry and our map fixes: how they fit together

Read on Saturday at 12:40 from branch `mid-air-vio` (commits 4d60f38 and 7773c95, `vio/README.md`).

**What he built.** An error-state Kalman filter on Mid-Air (synthetic flights about 15 m above hilly ground, about 90 s each). The IMU drives it at 100 Hz; three measurements correct it, each behind a 99 percent Mahalanobis gate: the barometer (height), the forward camera (how much the drone turned between two moments, which makes the gyroscope's error observable, including its drift in heading) and the down camera (speed over ground from optical flow, scaled by the height). On three flights it was not tuned on, the position is 13 to 33 m off after 83 s without GNSS, against 114 to 441 m for IMU and barometer and 338 to 912 m for the IMU alone. On Saturday morning he added a benchmark over all flights, a filter for the gyroscope's error alone, and a sun detector (work in progress).

**His own caveats, which matter before his numbers go on a slide:** the filter states an uncertainty 2 to 5 times too small (NEES 15 to 42 against an expected 9), so it cannot serve as an integrity bound yet; and his README notes that the filter's uncertainty propagation misses one term for Mid-Air's world-frame gyroscope, "to be fixed before the ESKF results are trusted".

| | Alessandro: visual-inertial odometry | Ours: map fixes with checks |
|---|---|---|
| Answers | How far and in which direction has the drone moved? | Where is the drone? |
| Error over time | Grows: 13 to 33 m after 83 s | Bounded where the ground matches the map; fails where it changed |
| Sensors | IMU, forward and down camera, barometer | Down camera, a map, a heading |
| Data | Mid-Air: synthetic, low, short flights, no map | ALTO and UAV-VisLoc: real, 400 to 550 m high, 4 to 83 km, no IMU |
| Stated uncertainty | 2 to 5 times too small | Holds in 97 percent of frames on development data, 77 to 98 on held-out UAV-VisLoc, 70 on unseen ALTO (older version) |
| Wrong inputs | 99 percent Mahalanobis gate | Score, distance gate, confirmation by the next fix |

**Where they combine:**

1. **Our fixes go into his filter as position measurements.** His filter has a generic update with a gate; a fix is a measurement of north and east with an accuracy of 15 m. That is about 20 lines. It is how Raptor and VNS01 work too: the fix goes into the drone's own estimator.
2. **His filter is our heading source between fixes.** The forward camera makes the heading drift observable. His final attitude errors after 83 s, 1.9 to 5.3 degrees, are what our compass model assumes (an offset of about 4 degrees), so our compass runs stand for his heading. The sun sensor would be the step beyond.
3. **His barometer height can set our matcher's zoom.** Ilhan found the zoom to be the main cause of failure on ALTO Train; the height from the barometer predicts it.
4. **One sun detector, not two.** His detector (round, blooming, next to the sky) is more careful than ours (largest bright patch); the phone test should use his.
5. **Before fusing:** his filter's stated uncertainty has to be made honest first. A filter that is too sure of itself refuses right fixes as "too far away" and can stay lost.

**What cannot be shown today:** both on one flight. No dataset we have holds an IMU, a down camera and a map together: Mid-Air has no map, ALTO's sample and UAV-VisLoc have no IMU. The places where they can meet: Dan's simulator with a real aerial image as the ground, or a map built from Mid-Air's own down-camera images of another flight over the same terrain.

## 6. Synthesis: what works and what does not

| Works | Evidence |
|---|---|
| Matching camera frames against an aerial map by correlation of brightness patterns | About 14 to 16 m per fix on ALTO (USA) and UAV-VisLoc (China) |
| A stated uncertainty that holds when the system works | Error within 3 sigma in 97 to 100 percent of frames on the ALTO validation section and UAV-VisLoc 03 |
| A search sized by the uncertainty | Recovers after 1,000 m without fixes on ALTO |

| Does not work | Evidence |
|---|---|
| Keypoint matching between camera and map | 2 of 100 accepted on ALTO, both wrong; the papers agree |
| Agreement of frames 14 m apart | They see the same ground and agree on the same wrong place |
| The score threshold alone, on unseen data | About 11 wrong fixes used on UAV-VisLoc flight 03 |
| Any check against a map that is wrong as a whole | Two fixes then agree on the same wrong place; needs a second source (documented as a test) |
| The stated uncertainty on unseen ALTO sections | Holds in 70 percent of frames, 15 in the worst section (Ilhan, version before the new checks) |

## 7. What is done, what is left, and who does it

Code freeze Sunday 10:00, demo 13:00. Updated 14:15.

**Done** (all on branch `integration`):

| What | Where |
|---|---|
| Map search around the estimate; the leak through the reference images removed | findings 3.8 |
| Second dataset UAV-VisLoc; heading sensors (compass, sun); fix offset in the drone's frame | section 3 above |
| Checks against wrong fixes: confirmation of large jumps, Ilhan's quarters rule (option) | section 4 above |
| **Held-out test** on flights 01 and 04, run once | top of this page |
| **Limits test** (light, blur, haze) and the fix for the camera losing track | top of this page |
| **Demo clips**: ALTO and unseen flight 04 | `outputs/replay/` |
| Comparison with Alessandro's visual-inertial odometry | section 5a |
| The mentor's phone sun compass: code ready, waiting for photos | `baseline/scripts/phone_sun_compass.py` |
| Eagle Eyes and the other fielded systems compared with ours | [landscape.md](landscape.md#eagle-eyes-and-the-systems-fielded-in-ukraine), top of page |
| The system in detail: what it does, step by step, and what it gives the user | section 2 |
| **One branch with everything**, `integration`: every team branch merged, 297 tests pass | [pull request 2](https://github.com/dwn97/TaipeiDrift/pull/2) |
| **The simulated flight over Wufeng**: route, aerial ground, recorded flight, camera navigator with the 2018 and the 2020 map, demo clip | top of this page |
| The phone sun test, written up for the teammate | [phone-sun-test.md](phone-sun-test.md) |
| Computing time: 11 ms per camera frame, 0.07 to 2 s per map fix, 4 MB of map per km² | section 2.4, `outputs/timing/timing.json` |

**Left:**

| What | Who | State |
|---|---|---|
| **The fused navigator on the simulated flight:** Alessandro's IMU filter as the core; the down camera's motion with its tilt removed by the IMU; our map fixes as position measurements, each corrected for the drone's tilt (2 to 5 m instead of 16 to 22), behind his 99 percent gate and our confirmation of large jumps; the barometer sets the matcher's zoom; a compass (later the sun) for the heading, since the simulated drone has no forward camera | Claude | Next, about two hours |
| **Merge pull request 2 into `main`** | Dustin and team | Open |
| README for the submission (one sentence, headline number, how to run, limits, each part, data and licences) | Claude | Open |
| Fold today's numbers into `findings.md` | Claude | Open |
| Slides and the one story | Dustin and team | Open; charts from Claude |
| ~~Rerun the ALTO Train test with the map search~~ Done at 13:00: 131 m (top of page). Next: the same test with the current navigator (new checks), and the zoom from the height | Ilhan | Asked |
| The heading drift of the visual-inertial odometry; fix the term he flagged; who adds the fix input to his filter | Alessandro | Asked |
| Phone photos for the sun compass: [phone-sun-test.md](phone-sun-test.md), tomorrow 09:00 to 09:45 | anyone with an iPhone | Asked |

**The story for the slides** (proposal): the drone's camera as a GNSS replacement on cheap hardware and free maps. Dead reckoning drifts (Alessandro's visual-inertial odometry, our camera motion). Map fixes reset it. The check keeps it honest, shown on flights from another country that we never tuned on. Next steps: the sun sensor for heading, terrain navigation for forest and night, a thermal camera for night, the water crossing.

## 8. What we will not claim

- Better accuracy than Raptor or VNS01.
- 31 m as our accuracy in general: it is the ALTO section we tuned on. On unseen flights: 60 to 131 m.
- Night, fog or flight over water.
- Real-time on drone hardware: measured on a laptop only.
- Real dead reckoning on UAV-VisLoc: it is simulated there; real on ALTO.

## 9. A note on the laptop

The project folder on the Desktop syncs to iCloud. With the disk 98 percent full, macOS started moving project files into iCloud this morning (535 of them, including part of `.git`), and Python and git stalled waiting for them. What was done:

- **The working copy is now `~/Projects/DefenseHackathon`**, cloned fresh from GitHub, outside iCloud. Open this folder in VS Code from now on. The Desktop folder is left untouched but is no longer up to date.
- The data stays where it was, in `Desktop/DefenseHackathon/data/raw.nosync` and `processed.nosync` (iCloud does not sync folders ending in `.nosync`); the new copy links to them.
- The Python environment is in `~/.venvs/defensehackathon`; `.venv` in the new copy links to it.
- 18 GB of duplicate downloads were deleted; 21 GB are free.
