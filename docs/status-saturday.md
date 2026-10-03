# Where we stand (the working doc)

**Last updated: Saturday 3 October, 12:20.** The one up-to-date document for the team: what we are building, what changed today and why, what is running, and what is left until the code freeze on Sunday at 10:00. It is updated at each milestone. Every number comes from a script on branch `alto-navigator`; the details are in [findings.md](findings.md).

## Now, in short

The core system works on real data from two countries. The test on data it was never tuned on exposed one weakness: wrong position fixes slip through. The new checks cut them from about 12 per flight to 0 to 2, on the flight where they were developed (UAV-VisLoc 03). The clean final test on two flights never looked at (01 and 04) is the next step.

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

**How the plan changed this morning**

| Before | Now | Why |
|---|---|---|
| Wait for Dropbox to test on ALTO Train | Test on a second dataset from China (UAV-VisLoc) | The jury scores technical validity; Dropbox was blocked |
| Navigator ignores the heading | Navigator uses the drone's heading (compass, or the mentor's sun sensor) | The brief lists heading, every drone has it, and without it we fail at the first turn |
| Next step: blurred and dark frames | First: make the check against wrong fixes hold on unseen flights | Both held-out tests (ours and Ilhan's) show it is the weak point |
| Sun sensor as a "future" slide | Sun sensor simulated now; the mentor's phone test measures it for real | The mentor's idea; gives a measured number |

**Achieved**

- **USA (ALTO, real helicopter flight):** camera alone ends 472 m off (median); with map fixes 31 m. A flaw in our test (the search knew the true path) was found and removed; the numbers did not change.
- **China (real drone photos against a map 2.5 years older):** the matcher, unchanged, finds 80 percent of the photos within 30 m.
- **74 km without GNSS:** median 27 m, against 1,041 m without fixes. But about 11 wrong fixes slipped through and the drone got lost in stretches: our honest weakness. Ilhan found the same independently on unseen ALTO data (94 m instead of 31 m).
- Built against it: confirmation of large jumps by the next fix, Ilhan's quarters check, the fix offset in the drone's own frame (fix error 19 to 13 m), compass and sun-sensor models, the aviation integrity measure.

**Asks to the team**

- **Ilhan:** rerun the ALTO Train test with the map search (`search: area`); your quarters rule is in the shared code.
- **Alessandro:** the heading error of the visual-inertial odometry after 30, 60 and 80 s without GNSS; it becomes our compass model.
- **Felix:** one slide on terrain navigation for forest and night (forest is 76 percent of Taiwan, where camera fixes fail).
- **Dan:** can the simulator show a flight for the demo by 18:00? Yes or no by 14:00.
- **Anyone with an iPhone, before about 13:00 while the sun is high:** the mentor's sun-compass test (steps below).
- **Dustin:** the story and slides; ask the organisers what the brief's "suggested dataset" is.

**The mentor's sun-compass test with an iPhone** (code ready: `baseline/scripts/phone_sun_compass.py`)

1. Settings, Camera: location on. Back camera at 1x.
2. Show `data/raw/phone_sun/chessboard_9x6_inner_corners.png` full-screen on a laptop; take 15 to 20 photos of it from different angles and distances, filling much of the frame. Put them in `data/raw/phone_sun/chessboard/`.
3. Lay the phone screen down on a level table in the sun (check with the Measure app's Level), back camera looking up, exposure turned all the way down. Shoot with the volume button.
4. Take 8 photos, turning the phone in 90-degree steps along a table edge (0, 90, 180, 270, twice). Note what the Compass app shows for each. Put them in `data/raw/phone_sun/sun/`.
5. Run `python baseline/scripts/phone_sun_compass.py`: the scatter of the measured turns around the 90-degree steps is the sun compass's error.

## 1. The goal for Sunday

**One sentence:** a small drone that loses GNSS keeps its position with its own downward camera and a free satellite map, and it says honestly when it is not sure.

What the jury scores (the challenge brief), and how we answer each point:

| The brief scores | Our answer | State |
|---|---|---|
| Reduction in positioning error | Camera alone drifts to 472 m (median) on ALTO; with map fixes 31 m | Measured |
| Technical validity | Tested on flights we never tuned on, from another country, camera and map source | Running now |
| Noise tolerance | Error as the picture gets darker, blurred, hazy, and as the heading sensor gets worse | This afternoon |
| Computing and integration | One fix takes about 0.1 s on one laptop core; output is a position with an uncertainty, the form an autopilot takes | Timing this afternoon |
| Deployment feasibility | Free maps (Taiwan's government orthophotos), an ordinary camera, no GPU | Slide |
| The user | Operators of small drones near Taiwan's coast and islands, where GNSS is jammed | Slide |

The required parts of the brief: a dead-reckoning baseline, at least one correction, plots of estimated against true path and of error over time, and the limits. We have all four on ALTO; the held-out test adds them on a second dataset.

## 2. The system, layer by layer

```
 heading sensor  ─┐
 (compass or sun) │
                  ▼
 camera motion ──► dead reckoning ──► estimate + stated uncertainty ──► status: TRACKING / DEGRADED / LOST
 (or VIO)          (drifts ~10 %)          ▲
                                           │ used only if it passes the check
 camera frame ──► search the map around ───┘
                  the estimate (a fix)      check: score high enough? close enough to the estimate?
                                                   large jump confirmed by the next fix?
```

1. **Dead reckoning.** The image slides through the camera as the drone flies; that motion, added up, carries the position forward. It drifts by about 10 percent of the distance flown.
2. **Heading.** To add up the motion in the right direction through turns, the navigator needs the drone's heading. Every drone has one from its compass (magnetometer); the mentor's sun sensor would give a better one.
3. **Position fixes.** Every few hundred metres the camera frame is compared with the satellite map in a circle around the estimate. The best match is a candidate position.
4. **The check.** A candidate is used only if its match score is high enough and it lies within the stated uncertainty of the estimate. New today: a candidate that would move the estimate far, out of a wide search, must be confirmed by the next fix over different ground.
5. **Stated uncertainty and status.** The navigator always reports how sure it is. Without fixes the uncertainty grows; at a fix it shrinks. The status follows from it.

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
- **Taiwan:** forest covers 76 percent of the island, where map fixes are unlikely; there, terrain navigation (Felix) carries the load.

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

To keep the held-out test honest, we fixed the test flights **before** looking at them: UAV-VisLoc flights 01 (Changjiang, 817 photos) and 04 (Taizhou, 738 photos), downloaded and set aside. Flight 10 was meant to be the third; Google Drive throttles its download. We develop only on ALTO and flight 03. Once the method is frozen, it runs once on 01 and 04, and we report what comes out, good or bad.

## 6. Synthesis: what works and what does not

| Works | Evidence |
|---|---|
| Matching camera frames against an aerial map by correlation of brightness patterns | About 14 to 16 m per fix on ALTO (USA) and UAV-VisLoc (China) |
| A stated uncertainty that holds when the system works | Error within 3 sigma in 97 to 100 percent of frames on ALTO |
| A search sized by the uncertainty | Recovers after 1,000 m without fixes on ALTO |

| Does not work | Evidence |
|---|---|
| Keypoint matching between camera and map | 2 of 100 accepted on ALTO, both wrong; the papers agree |
| Agreement of frames 14 m apart | They see the same ground and agree on the same wrong place |
| The score threshold alone, on unseen data | About 11 wrong fixes used on UAV-VisLoc flight 03 |
| Any check against a map that is wrong as a whole | Two fixes then agree on the same wrong place; needs a second source (documented as a test) |

## 7. What is left, and who does it

Until about 23:00 tonight; code freeze Sunday 10:00, demo 13:00.

| Time | What | Who |
|---|---|---|
| until 12:00 | Finish the confirmation test on ALTO and flight 03; freeze the method | Claude |
| 12:00 to 12:30 | **Held-out run on flights 01 and 04**, the main result table | Claude |
| 12:30 to 13:30 | Compass against sun sensor; integrity measures; Stanford diagram | Claude |
| 13:30 to 14:30 | Noise tolerance: darker, blurred, hazy frames; worse heading | Claude |
| 14:30 to 15:00 | Timing on one CPU core; map storage per square kilometre | Claude |
| 15:00 to 18:00 | Demo: replay of a flight on the map with estimate, uncertainty circle, fixes used and refused, status | Claude |
| 18:00 to 21:00 | Slides, README, findings | Dustin and Claude |
| now | Roles; the one story; ask the organisers what the brief's "suggested dataset" is | Dustin |
| today | Heading error of the visual-inertial odometry after 30, 60 and 80 s without GNSS; it becomes our compass model | Alessandro |
| today | One slide each: terrain navigation (Felix), simulator (Dan), integrity review (Ilhan) | team |

**The story for the slides** (proposal): the drone's camera as a GNSS replacement on cheap hardware and free maps. Dead reckoning drifts (Alessandro's visual-inertial odometry, our camera motion). Map fixes reset it. The check keeps it honest, shown on flights from another country that we never tuned on. Next steps: the sun sensor for heading, terrain navigation for forest and night, a thermal camera for night, the water crossing.

## 8. What we will not claim

- Better accuracy than Raptor or VNS01.
- Night, fog or flight over water.
- Real-time on drone hardware: measured on a laptop only.
- Real dead reckoning on UAV-VisLoc: it is simulated there; real on ALTO.

## 9. A note on the laptop

The project folder on the Desktop syncs to iCloud. With the disk 98 percent full, macOS started moving project files into iCloud this morning (535 of them, including part of `.git`), and Python and git stalled waiting for them. What was done:

- **The working copy is now `~/Projects/DefenseHackathon`**, cloned fresh from GitHub, outside iCloud. Open this folder in VS Code from now on. The Desktop folder is left untouched but is no longer up to date.
- The data stays where it was, in `Desktop/DefenseHackathon/data/raw.nosync` and `processed.nosync` (iCloud does not sync folders ending in `.nosync`); the new copy links to them.
- The Python environment is in `~/.venvs/defensehackathon`; `.venv` in the new copy links to it.
- 18 GB of duplicate downloads were deleted; 21 GB are free.
