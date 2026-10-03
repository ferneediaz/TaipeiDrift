# Where we stand (the working doc)

**Last updated: Saturday 3 October, 13:15.** The one up-to-date document for the team: what we are building, what changed today and why, what is running, and what is left until the code freeze on Sunday at 10:00. It is updated at each milestone. Every number comes from a script on branch `alto-navigator`; the details are in [findings.md](findings.md).

## Now, in short

The system works on real data from two countries, and we now know how well. On the flights it was developed on, it holds about 30 m where the camera alone drifts 470 to 820 m. On flights it never saw, the fixes cut the drift by 40 percent (ALTO, 219 to 131 m, Ilhan's test at 13:00) to tenfold (UAV-VisLoc 04, 675 to 60 m); where the ground was rebuilt since the map (UAV-VisLoc 01) they do not help. Fielded systems claim 15 to 20 m. **Section 2 explains in detail what the system does and what it gives a user.** The three parts of the team (our map fixes, Alessandro's IMU filter, Ilhan's checks and simulator patch) are not combined in code yet; they can only run together in the simulator, which is the next job.

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
| Reduction in positioning error | Where developed: 472 → 31 m (ALTO), 822 → 28 m (UAV-VisLoc 03). Never seen: 219 → 131 m (ALTO, 8 sections), 675 → 60 m (UAV-VisLoc 04); no gain where the ground changed (01) | Measured |
| Technical validity | Tested on flights we never tuned on, from another country, camera and map source | Done: UAV-VisLoc 01 and 04 (ours), ALTO Round 2 (Ilhan) |
| Noise tolerance | Error as the picture gets darker, blurred, hazy, and as the heading sensor gets worse | Done: light, blur, haze (top of page); compass against sun sensor |
| Computing and integration | One fix takes about 0.1 s on one laptop core; output is a position with an uncertainty, the form an autopilot takes | Timing open |
| Deployment feasibility | Free maps (Taiwan's government orthophotos), an ordinary camera, no GPU | Slide |
| The user | Operators of small drones near Taiwan's coast and islands, where GNSS is jammed | Slide |

The required parts of the brief: a dead-reckoning baseline, at least one correction, plots of estimated against true path and of error over time, and the limits. We have all four on ALTO; the held-out test adds them on a second dataset.

## 2. The system in detail: what it does, and what it gives the user

Written at 13:10 so that everyone on the team can explain the system in their own words. Every number below is a setting the code really uses (`baseline/configs/`, `baseline/src/estimation/`) or a measured result from this page.

**In one picture**

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

### 2.1 The user and the problem

- **The user:** anyone who flies small, cheap drones where GNSS is jammed or spoofed. In Taiwan: around the outlying islands, where interference is reported regularly, and in any conflict. Examples: coast and island surveillance, disaster response, military reconnaissance.
- **The problem:** without GNSS the drone has only its own sensors, and they drift. On our data the camera alone ends up 472 m off on ALTO and 822 m off on UAV-VisLoc flight 03 (median); the IMU alone 338 to 912 m after 83 s on Mid-Air (Alessandro). The drone cannot hold its route, find its target or come home, and nobody tells the operator how far off it is.
- **What the drone already carries:** a camera, a compass, a barometer, an IMU, a small computer. What it lacks is a position it can trust.

### 2.2 One flight, step by step

**Before take-off.** The drone stores an aerial or satellite image of the area as its map, in grey, at about 1 m per pixel. The size, counted from zero: 1 km² at 1 m per pixel is 1,000 × 1,000 pixels, which is 1 MB. A route corridor 100 km long and 2 km wide is 200 MB. All of Taiwan, about 36,000 km², is 36 GB before compression; it fits on a memory card.

**While GNSS still works** (the first 300 m on ALTO, 1,000 m on UAV-VisLoc), the system learns four things by comparing the camera with GNSS. Nothing is calibrated by hand.

1. *How a slide of the picture becomes metres.* If the picture slides 10 pixels while GNSS says the drone flew 5 m north, one pixel of slide is 0.5 m. The system learns this as a small 2 × 2 table, which also holds how the camera is turned against the drone.
2. *How the map must be zoomed and turned* to look like the camera picture. The zoom depends on the height.
3. *The fix offset:* where a matched position lies compared with the GNSS position. On UAV-VisLoc it is about 13 m ahead of the drone (camera tilt, trigger timing). It is learned in the drone's own frame (forward, right), so it turns with the drone.
4. *The cruising speed,* for the last case below.

**GNSS is lost. Every camera frame: dead reckoning.**

- The system measures how far the picture slid since the last frame, turns that into metres with the table, turns it to north and east with the heading (compass or sun sensor), and adds it to the position.
- Every step carries a small error, and the errors add up: about 10 percent of the distance flown.
- The system says so. Next to the position it keeps σ (sigma), its own statement of how far off it may be, in metres. If σ is honest, the true error stays below 3σ almost always; our tables check exactly that ("error within the stated 3 sigma").
- How σ grows, from zero. Independent errors add as squares. Right after a fix σ is about 13 m. After 300 m more, the dead reckoning has added 10 percent of 300 m, which is 30 m. The new σ is √(13² + 30²) = √(169 + 900) = 33 m.

**Every 300 m: a position fix.**

- The camera picture is compared with the map in a circle around the current estimate. The radius is 3σ, at least 60 m and at most 600 m: with σ = 33 m it is 99 m.
- The comparison slides the picture over every position in the circle and scores how well the pattern of light and dark agrees (normalised correlation). The score ignores how bright the picture is and how strong its contrast is. Three pixels in a row as an example: 10, 20, 30 against 110, 120, 130 score 1.0 (the same pattern, only brighter); against 30, 20, 10 they score −1 (the pattern reversed). A few zooms and angles around the learned ones are tried. The best place is the candidate fix.
- **Three checks.** The candidate is used only if
  1. its score is at least 0.33;
  2. it lies within 3σ of the estimate (99 m in the example);
  3. when it comes from a wide search (radius over 150 m) and would move the estimate by more than 30 m, the next fix, 300 m later over different ground, agrees with it. "Agrees" means the two fixes lie as far apart as the dead reckoning says, within 3 × √(15² + 15² + 30²) = 110 m: the error of each fix, 15 m, and of the dead reckoning between them, 30 m. A look-alike place rarely has a matching look-alike 300 m further on.

**Using a fix.** A fix has its own uncertainty, σ = 15 m. The estimate moves towards the fix by how unsure it is compared with the fix. Estimate σ = 30 m (squared: 900), fix σ = 15 m (squared: 225): the estimate moves 900 / (900 + 225) = 80 percent of the way, and its new σ is √(0.2 × 900) = 13.4 m. A fix that waited for confirmation is used first, then the one that confirmed it.

**What it reports, every frame:** a position (north and east), σ, and a status.

| Status | When | With a fix every 300 m |
|---|---|---|
| TRACKING | σ up to 30 m | From each fix (13 m) until about 270 m later |
| DEGRADED | σ from 30 to 100 m | The last 30 m before each fix; after one missed fix σ is 62 m |
| LOST | σ above 100 m | After about 1 km without an accepted fix |

A position with its uncertainty is the form in which an autopilot takes a position from an outside source. The link to a real autopilot is not built.

**When the camera stops seeing the motion** (dark, hazy or blurred picture). If a frame's step is shorter than 30 percent of the cruising step learned with GNSS (say the drone cruises 10 m between frames and the camera reports less than 3 m), the camera is not believed. The system flies on at cruising speed in the last good direction, and σ grows three times faster (30 percent of the distance). It never stands still while claiming to be sure.

### 2.3 What it gives the user, and the evidence

| The user gets | Evidence (median error) | The honest limit |
|---|---|---|
| A position within tens of metres where the ground matches the map | Developed on: ALTO 472 → 31 m; UAV-VisLoc 03 (74 km) 822 → 28 m | The settings were chosen on these flights |
| On flights it never saw, a drift cut by 40 percent to tenfold | ALTO, 8 sections (Ilhan): 219 → 131 m; UAV-VisLoc 04 (83 km): 675 → 60 m | Wrong fixes still pass: 18 to 32 on flight 04 |
| Nothing where the ground has changed since the map | UAV-VisLoc 01: no gain over dead reckoning | Needs a fresher map, as Eagle Eyes has |
| A warning when it is unsure | Share of the flight with "all clear" while more than 50 m off: 0.0 percent on flight 03, 0.9 to 3.2 on unseen 01 and 04 | On unseen ALTO the stated bound held in 70 percent of frames (older version, before the new checks) |
| A safe reaction when the camera goes blind | Haze with 10 percent of the contrast left: 1,640 m and "sure" before the change; 54 m and LOST after it | Blur of 4 m still fools the camera's motion |
| A better heading from the sun | Unseen flight 01: 306 → 177 m with the sun sensor instead of the compass | Useless around midday in Taiwan's summer, when the sun is overhead |
| No vendor, nothing to jam or to detect | Free aerial images, an ordinary camera, no radio emission | Daylight and land only |

### 2.4 What it costs the user

- **Sensors:** a downward camera; a heading from the compass every drone has, or a sun sensor (35 g, 0.2 W, Fan et al. 2016); GNSS for the first few hundred metres.
- **Map:** free aerial images, about 1 MB per km² before compression.
- **Computing:** one fix in about 0.1 s on one laptop core, no graphics processor. Not yet timed on a drone's small computer.
- **Setup:** none by hand; it calibrates itself while GNSS works. That also means a drone jammed from take-off is not covered.

### 2.5 What it does not do yet, and what would close each gap

| Gap | What would close it | State |
|---|---|---|
| Changed ground | A fresher map: recent reconnaissance or the drone's own earlier flights (Eagle Eyes) | To show in the simulator: the same flight with the 2020 and the 2018 image as the map |
| Wrong zoom when the height changes | Set the zoom from the barometer's height (Ilhan's finding on ALTO) | The simulator has a barometer |
| Wrong fixes that still pass | A second motion source to cross-check (Alessandro's IMU filter); a forecast of where the map has look-alikes | Open |
| Night | An infrared camera against the same map, as Raptor does | Not started |
| Water, the Strait | Nothing to match; IMU and camera dead reckoning with the sun heading | Alessandro's filter; not combined |
| Forest, 76 percent of Taiwan | Terrain navigation (Felix) | Separate |
| Jammed from take-off | A start without GNSS | Not covered |

### 2.6 Against what a buyer can get today

Sources in [landscape.md](landscape.md).

| | Ours (weekend prototype) | AIDC AIxVNAV (Vantor's Raptor inside) | Eagle Eyes (Ukraine) | Osiris (Greece) |
|---|---|---|---|---|
| Map | Free aerial images | Vantor's licensed 3D data | Own recent reconnaissance flights | Preloaded satellite images |
| Hardware | Ordinary camera, one CPU core | Camera and a graphics processor | Not public | Module under 300 g, 25 W |
| Accuracy | 28 to 31 m where developed; 60 to 131 m on unseen flights | Raptor: 12 m mean in one published test | Not public | 15 m CEP (maker) |
| Says when unsure | σ and status every frame, and we measure how often it is wrong while saying "all clear" | A confidence from 0 to 1 per frame | Not public | Not public |
| Night | No | With an infrared camera | Not public | Daytime figure |
| Maturity | Recorded flights | Flight-tested in Taiwan, 2026 | In combat since 2023 | About 3,000 km of trials in Ukraine |

**In one sentence for a customer:** for cheap drones where GNSS is jammed, it turns "the drone is lost" into "the drone knows where it is, to within tens of metres where the map matches and 60 to 130 m on flights it never saw, and says when it is not sure", with the camera it already has and a free map. A buyer who needs accuracy or night flight today buys a fielded system; ours is the open, low-cost prototype with measured limits.

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

Code freeze Sunday 10:00, demo 13:00. Updated 13:15.

**Done** (all on branch `alto-navigator`):

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

**Left:**

| What | Who | State |
|---|---|---|
| **1. One branch with everything, `integration`:** ours, Alessandro's `mid-air-vio`, Dan's simulator with Ilhan's patch (`sim-demo`), Ilhan's research, Felix's TRN from `main`. A trial merge shows small conflicts only: `.gitignore`, `pyproject.toml`, the handoff note, and the Mid-Air baseline files that Alessandro and we both fixed on Friday. `main` stays untouched for the team to decide | Claude | Started 13:20, locally; pushed once it runs |
| **2. The fused navigator:** Alessandro's IMU filter as the core. Our map fixes go in as position measurements (15 m) behind his 99 percent gate and our confirmation of large jumps; the barometer sets the matcher's zoom (Ilhan's finding); the filter's heading turns the frame for the search; Ilhan's quarters rule stays an option | Claude | After step 1 |
| **3. The simulator demo:** Dan's simulator with Ilhan's patch (GNSS cut, recorder, IMU leak fixed), the real Wufeng 2020 aerial image as the ground, a flight along the motorway corridor. Four runs on the same flight: our camera navigator alone, Alessandro's filter alone, the two fused, and the fused one with the fresh 2020 image as the map (the price of an old map) | Claude | After step 2; branch `sim-demo` started 12:40 |
| Computing time per fix and per frame, map storage per square kilometre | Claude | Open |
| README for the submission (one sentence, headline number, how to run, limits, each part, data and licences) | Claude | Open |
| Fold today's numbers into `findings.md` | Claude | Open |
| Slides and the one story | Dustin and team | Open; charts from Claude |
| Which branches go into `main` | team | Open |
| ~~Rerun the ALTO Train test with the map search~~ Done at 13:00: 131 m (top of page). Next: the same test with the current navigator (new checks), and the zoom from the height | Ilhan | Asked |
| The heading drift of the visual-inertial odometry; fix the term he flagged; who adds the fix input to his filter | Alessandro | Asked |
| One slide on terrain navigation for forest and night | Felix | Asked |
| Phone photos for the sun compass, while the sun is high | anyone with an iPhone | Asked |

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
