# Findings

What we measured on Friday 2 October between 21:00 and 22:40, and on Saturday 3 October (sections 2.6 and 3.6 to 3.8), on the data we have on disk. Every number on this page comes from a script in `experiments/` or from the shared code in `baseline/`; the list is at the end. Read this before the team decides how to go on.

The current [PLAN.md](PLAN.md) was written before these measurements. Section 6 says what they change.

## In short

1. **The data is here.** A 10 GB subset of Mid-Air and the validation section of ALTO, a real helicopter flight, are downloaded and checked. [data/README.md](../data/README.md) says how to get them.
2. **The IMU alone drifts fast on Mid-Air.** After GNSS is cut, the IMU-only position is 485 m off after 78 seconds and passes 50 m after 36 seconds (median of 30 flights). This is the baseline the challenge asks for.
3. **Mid-Air's gyroscope uses an unusual convention.** Its turn rates are given around the map's axes. A textbook filter assumes the drone's own axes and gives nonsense on this data. Anyone writing a filter on Mid-Air needs section 2.2.
4. **Camera speed with a barometer does not work on Mid-Air.** The drone flies about 15 m above hilly ground, so the height above take-off says little about the distance to the ground. The speed error was 37 and 161 percent on two flights.
5. **On the real ALTO flight the whole chain works.** GNSS is cut after 300 m. Camera motion alone ends 608 m off after 4.3 km. With a position fix against the reference images every 100 to 300 m, the median error is 26 to 31 m. The system learns what it needs from the first 300 m of GNSS and uses no camera calibration, no altitude and no IMU.
6. **Fixes have a cliff, and we know where it is.** Once the drift since the last fix is larger than the area that is searched, the match lands in a wrong place and is accepted. This happens between 300 and 400 m of flight, and the run then ends further off than with no fixes at all. Two additions repair it: a minimum match score, and a search area that grows with the uncertainty. With both, fixes 1,000 m apart give a median error of 56 m.
7. **Plain brightness matching is enough to start.** Keypoint matching fails completely on these images. Sliding the camera frame over the reference image works, at about 14 m per fix. The pretrained DenseUAV model is not needed for a first version.
8. **Proposal.** Mid-Air keeps the IMU baseline. Camera speed, position fixes, the integrity check and the drift budget move to ALTO. The simulator is where all sensors run in one flight.

Added on Saturday:

9. **The camera navigator is shared code now** (branch `alto-navigator`, one command, 91 tests). It reproduces every number of section 3.4 exactly, and it states its own uncertainty. When it breaks at the cliff, its error is larger than 3 times the uncertainty it states in 76 percent of frames; when it works, in 0 to 3 percent (section 3.6).
10. **Agreement of nearby frames does not catch wrong fixes here.** Frames 14 m apart see almost the same ground and land on the same wrong place. The score check stays (section 3.6).
11. **Visual-inertial odometry on Mid-Air works** (Alessandro, branch `mid-air-vio`): 13 to 33 m after 83 s without GNSS on three flights it was not tuned on, against 338 to 912 m for the IMU alone (section 2.6). It is the missing layer between our fixes.
12. **The test on the ALTO training section is blocked.** Dropbox has disabled the dataset link for the day. The images are on disk, but the files with the positions are in the part that did not arrive (section 3.7).
13. **A mistake found and repaired: our search knew the true path.** ALTO's reference images are centred on the true path, so every fix landed near it. The navigator now searches one map of the area in a circle around its own estimate. The results stay within half a metre of the earlier ones, so they hold; the cliff at 400 m disappears (section 3.8).
14. **A second dataset, from China, never tuned on** (UAV-VisLoc: real drone photos from 2018 against satellite maps from 2021 to 2023). The matcher, unchanged, finds 80 percent of the photos within 30 m. On the development flight (03) the first version used about 12 wrong fixes per flight; with two new checks, 0 to 2 (section 3.9).
15. **The held-out result is mixed, and we report it as it is.** On flight 04 (83 km) the typical error falls from 675 m without fixes to 60 m with them, but 18 to 32 wrong fixes still pass. On flight 01 (66 km) the ground has changed since the map was made (bare land became high-rise estates) and map fixes do not beat dead reckoning (section 3.9).
16. **The limits of the camera picture, and a dangerous failure closed.** Down to 1/64 of the light the error stays at 31 m. When the picture is so poor that the camera cannot see the motion, the estimate used to stand still while claiming a few metres of uncertainty, 1 to 2 km off; now the navigator flies on at cruising speed and says LOST (section 3.10).

## What the team has to decide

1. Decided on Friday night: we stay with Challenge 2. Section 5 lists what spoke for and against.
2. Confirm the three results in section 6 and put one name on each.
3. Position fixes: start with brightness matching, and treat DenseUAV as an upgrade to compare against it.
4. What the simulator is for: the demo view, a test of all sensors in one flight, or both.
5. Whether to download the ALTO training section (9.93 GB) tonight. It is roughly 30 km of the same flight, estimated from its image count, and would let us report on data we did not tune on.

## Terms used on this page

- **Jam:** the moment GNSS is lost.
- **Dead reckoning:** carrying the position forward from measured motion alone. Its error grows without limit.
- **Fix:** an absolute position, obtained by recognising the ground below in stored reference images that have coordinates.
- **Zoom:** how much a camera frame has to be shrunk so that it shows the ground at the same size as the reference image. A zoom of 0.85 means 85 percent of the original size.
- **Score:** how well the camera frame fits the reference image at the best position. 1 is a perfect fit.
- **Drift:** the error of dead reckoning, given in metres or as a share of the distance flown.

## 1. The data we have

| Dataset | What it is | On disk | What it lacks |
|---|---|---|---|
| Mid-Air | Synthetic drone flights, low over hilly terrain | Sensor records of every flight in every condition (0.8 GB). Downward camera for 21 flights (about 9 GB). All 38 archives arrived and pass the integrity test | Barometer, distance to the ground, any map |
| ALTO validation section | A real helicopter flight over rural Ohio, August 2017 | 1,684 camera frames and 459 reference images with coordinates (1.73 GB) | Raw IMU, height above ground |
| Our simulator (branch `simulations`) | Gazebo in Docker, a drone with camera, IMU, barometer and GNSS | Runs on a MacBook according to its README. A second world with two islands was added at 22:19 | The GNSS cut, recorded flights, export in our format |

Product code so far: branch `mid-air-baseline` holds a first IMU-only baseline for Mid-Air, pushed at 22:06. Everything else on this page is experiment scripts.

## 2. Mid-Air

### 2.1 What a flight looks like

- 30 flights in each weather (sunny, cloudy, foggy, sunset) and 24 in each season (spring, fall, winter).
- A flight lasts 88 to 90 seconds. The 30 sunny flights cover 270 m to 1.5 km, 1.0 km in the median, at 3 to 17 m/s.
- Within one flight the altitude changes by 14 to 242 m, 70 m in the median. The drone follows hilly terrain.
- Per flight the sensor file holds the IMU at 100 Hz, a simulated GNSS at 1 Hz, the true position, speed, acceleration and attitude at 100 Hz, and the file names of the camera frames at 25 Hz.
- There is no distance to the ground below. The depth images in the dataset belong to the forward camera, according to the Mid-Air specifications.

### 2.2 The gyroscope is given around the map's axes

A real IMU reports turn rates around the drone's own axes, and textbook filters assume that. In Mid-Air the gyroscope, and the true angular velocity it is derived from, fit the attitude only as turn rates around the map's axes (north, east, down). The accelerometer follows the usual convention.

How we found it: the first version of the IMU-only script used the textbook rule and ended 8 km off even when fed with the true motion. With the rule below, the same test ends 0.13 m off after 78 seconds.

How to use the data correctly:

```python
from scipy.spatial.transform import Rotation

rot = Rotation.from_quat(attitude[[1, 2, 3, 0]])      # Mid-Air stores w, x, y, z; scipy wants x, y, z, w

rot = Rotation.from_rotvec(gyro * dt) * rot           # Mid-Air: the turn step goes on the LEFT
# textbook rule for rates around the drone's own axes:  rot = rot * Rotation.from_rotvec(gyro * dt)

accel_map = rot.apply(accel) + [0, 0, 9.81]           # the accelerometer is in the drone's axes; z points down
```

The evidence: each signal compared with the change of the true attitude from one sample to the next. Smaller is a better fit.

| Sensor file | Flights | True turn rate, as drone axes | True turn rate, as map axes | Gyroscope, as drone axes | Gyroscope, as map axes |
|---|---|---|---|---|---|
| Kite sunny | 30 | 0.070 | 0.001 | 0.079 | 0.026 |
| Kite foggy | 30 | 0.070 | 0.001 | 0.077 | 0.020 |
| PLE fall | 24 | 0.140 | 0.002 | 0.148 | 0.020 |

Values are the typical difference in rad/s. The 0.02 that remains for the gyroscope in map axes is its noise. The accelerometer fits to 0.06 m/s² in the drone's axes and misses by more than 1 m/s² in map axes.

The documentation says otherwise. The [technical specifications](https://midair.ulg.ac.be/tech_specs.html) of Mid-Air state that the gyroscope and the true angular velocity are expressed in the Body frame. The files we downloaded do not behave that way: in all 84 flights we checked, they fit the attitude only as rates around the world axes. Code written from the documentation alone gives wrong results, which is what happened to the first version of our baseline.

There are two consistent ways to handle this, and they do not give the same drift. Median over the 30 sunny flights, GNSS cut after 10 seconds:

| Reading | Needs the true attitude after the jam | Error at the end | Passes 50 m after |
|---|---|---|---|
| A. Use the rates as given, with the turn step on the left | No | 485 m | 36 s |
| B. Convert the rates to the drone's axes with the true attitude, then use the textbook rule | Yes, at every sample | 719 m | 29 s |
| Textbook rule on the rates as given, which is wrong | No | 7,479 m | 12 s |

We use A, because the baseline must not read ground truth after the jam. A and B agree during the first 10 seconds and then drift apart, more so in flights with many turns: a constant gyroscope offset partly cancels in A when the drone turns, and it adds up in B.

The baseline code on branch `mid-air-baseline` used the textbook rule when it was pushed at 22:06 and ends 3,758 m off on flight 0003. With reading A it ends 247 m off on that flight, and an independent implementation with the method of `experiments/j_midair_imu_only.py` gives the same value to the centimetre. The change is eight lines and is on branch `mid-air-baseline-fix`, together with three tests for it. On all 30 sunny flights that code gives a median of 491 m at the end. Its error includes the height, which is why it differs slightly from the 485 m of the experiment script.

### 2.3 How large the IMU errors are

Measured on the 30 sunny flights, IMU reading minus true motion.

| | Median | Largest |
|---|---|---|
| Accelerometer, constant offset | 0.025 m/s² | 0.100 m/s² |
| Accelerometer, noise per sample | 0.040 m/s² | 0.122 m/s² |
| Accelerometer, change of the offset during a flight | 0.042 m/s² | 0.228 m/s² |
| Gyroscope, constant offset | 0.0010 rad/s | 0.0061 rad/s |
| Gyroscope, noise per sample | 0.021 rad/s | 0.075 rad/s |
| Gyroscope, change of the offset during a flight | 0.0030 rad/s | 0.0158 rad/s |

Each weather and season file has its own noise. The gyroscope noise per sample has a median of 0.014 rad/s in the foggy file and 0.015 rad/s in the fall file.

For the simulator: its accelerometer assumptions fit these values. Its gyroscope noise is assumed at 0.0005 to 0.005 rad/s per sample, so the median in Mid-Air is three to four times the upper end of that range.

### 2.4 IMU-only baseline

Scenario from the plan: GNSS works for the first 10 seconds, then it is cut. From that moment the position comes from the IMU alone, starting from the true position, speed and attitude.

| Time since the jam | Median error | Smallest | Largest |
|---|---|---|---|
| 5 s | 0.4 m | 0.1 m | 1.9 m |
| 10 s | 1.9 m | 0.2 m | 10.5 m |
| 30 s | 38 m | 6.7 m | 262 m |
| 60 s | 237 m | 26 m | 2,145 m |
| End of flight, about 78 s | 485 m | 44 m | 3,831 m |

- The error at the end is 57 percent of the distance flown since the jam (median).
- The error passes 50 m after 36 seconds in the median, between 17 and 74 seconds. One flight of 30 stays below 50 m.
- The same code fed with the true motion ends 0.13 m off (largest 0.64 m), so these numbers come from the sensor errors and not from the code.
- These are horizontal errors. The height drifts as well, by about 200 m on flight 0003. Holding the height is what a barometer is good for.

![IMU-only drift on Mid-Air](figures/midair_imu_only.png)

### 2.5 Camera speed with a barometer does not work on Mid-Air

Camera speed works like this:

```
speed over ground = image motion x distance to the ground / focal length
```

With numbers: the image moves 100 pixels per second, the focal length is 128 pixels and the ground is 13 m below. Then the speed is 100 x 13 / 128 = 10.2 m/s. If the ground is really 20 m below, the true speed is 15.6 m/s. The speed is wrong by exactly as much as the distance to the ground is wrong.

Mid-Air has no sensor for that distance. The test used the scenario of the plan: learn the distance while GNSS still works (first 10 seconds), then follow it with the barometer, which measures changes of altitude.

| | Flight 0000 | Flight 0001 |
|---|---|---|
| Distance to the ground implied by the camera | median 13 m, mostly 5 to 20 m | median 16 m, mostly 10 to 27 m |
| Change of altitude during the flight | 14 m down to 17 m up | 26 m down to 8 m up |
| Speed error after the jam, distance followed by barometer | median 161% | median 37% |
| Speed error after the jam, distance kept constant | median 61% | median 90% |

The drone follows the terrain at about 15 m. The ground rises and falls by as much as the drone's whole distance to it, and the barometer cannot see that. For comparison, the paper we follow assumes a motion error of about 2 percent.

Limits of this test: two flights, a simple measurement of image motion with no correction for the drone's rotation, and about half of the frames left out because the drone was turning or slow. A careful version would do better, but it cannot close a gap from 37 percent to 2 percent, because the missing information is the terrain.

What follows: on Mid-Air, camera speed needs another source of scale, which is the hard part of full visual-inertial odometry. On data where the aircraft flies high above the ground, the problem is much smaller. See section 3.3.

### 2.6 Visual-inertial odometry on Mid-Air (Alessandro, branch `mid-air-vio`)

Pushed on Saturday at 08:14; the numbers below are copied from `vio/README.md` on that branch and were not re-run here. An error-state Kalman filter estimates position, speed, attitude and both IMU offsets. It takes three kinds of updates: a barometer (simulated from the true altitude with noise and a drifting offset), the turn measured by the forward camera, and the speed measured by the downward camera, whose size the filter trusts only loosely because of the height problem of section 2.5. Settings were tuned on flight 0001 only.

GNSS is lost after 5 s, then about 83 s follow without it. Position error at the end of the flight, in metres:

| Flight | IMU only | IMU + barometer | + both cameras |
|---|---|---|---|
| sunny 0000 | 684 | 114 | 12.7 |
| sunny 0001 (used for tuning) | 381 | 145 | 12.6 |
| cloudy 3000 | 912 | 441 | 33.1 |
| cloudy 3001 | 338 | 116 | 17.3 |

- On the three flights not used for tuning, both cameras together bring the error to 13 to 33 m, 4 to 9 times lower than IMU and barometer.
- The downward camera alone helps the position but loses the heading, which flow cannot observe, and the filter then claims far more certainty than it has.
- The error still grows. Absolute fixes stay necessary, which is the part section 3 covers.
- Caveats from the branch itself: the barometer is simulated, four flights, one of them used for tuning.

## 3. ALTO

### 3.1 What the data is

- A section of 4.59 km, flown in 84 seconds at about 56 m/s. One camera frame every 2.8 m, 500 by 500 pixels.
- 459 reference images from an aerial survey years earlier, one every 10 m along the flown route. Each covers 301 m at 0.60 m per pixel, with north at the top.
- For every camera frame: true position, altitude and orientation. The altitude is 376 to 455 m above the Earth model. The ground along the route is at 250 to 323 m according to the open Copernicus elevation model, so the helicopter is very roughly 100 to 200 m above the ground. The two heights use different zero levels, and we have not corrected for that.
- The camera frames are rotated by about 10 to 20 degrees against the reference images, show a little less ground, and are strongly green and overexposed.
- The orientation values describe the aircraft with x forward, y to the right and z down. The heading is 76 degrees in the median, 4 degrees off the course because of wind. The top of the camera frame is the aircraft's left side, so the frame has to be turned by 90 degrees minus the heading, here 14 degrees, to put north at the top. That agrees with the rotation found by matching.
- The camera looks 2.3 degrees away from straight down in the median, and up to 6.9 degrees. At this height 2.3 degrees move the centre of the picture by about 7 m on the ground.
- The [ALTO paper](https://arxiv.org/abs/2207.12317) confirms these axes: x towards the nose of the helicopter, z down. It names the camera (1600 by 1200 pixels, a 3.5 mm lens, 20 frames per second) and says the full dataset also holds an IMU at 200 Hz and a laser altimeter. The public sample contains neither, and the paper says it covers only a few kilometres.
- The reference images form a strip along the route. There is no map of the area to the sides.

### 3.2 Matching a camera frame against the reference images

Two classical methods, same frames.

| Method | Result |
|---|---|
| Keypoints (SIFT, with a geometric check) | Fails. A median of 3 matching points where 12 are needed. 2 of 100 frames accepted, both more than 140 m wrong |
| Brightness pattern: shrink and rotate the camera frame, slide it over the reference image, take the best fit | 21 of 24 frames within 20 m, median error 13.6 m. The other three are 30, 31 and 47 m off |

Keypoint matching works between two reference images, so the method is fine. The camera frames look too different from the survey images for it.

For brightness matching, zoom and rotation have to be searched. The results are consistent with the physics, which is why we trust them: the rotation stays between 10 and 20 degrees on a steady course, and the zoom is smaller where the helicopter flies lower.

**Can the score tell a right match from a wrong one?** 60 frames, each matched once at the right place and once against a reference image 400 m further along the route.

| | Right place | Wrong place |
|---|---|---|
| Score, median | 0.39 | 0.21 |
| Score, range | lowest 0.21 | highest 0.32 |

- The right place scored higher than the wrong place in all 60 frames.
- A threshold of 0.33 rejects all 60 wrong matches and keeps 44 of the 60 right ones.
- The score does not separate small misses. The three frames that were 30 to 47 m off scored 0.43, 0.42 and 0.26, like correct ones.
- A wrong match prefers the smallest zoom it is allowed. 57 of 60 wrong matches chose a zoom of 0.625 or less, and 30 chose the lower limit. A zoom at the edge of the search range is a warning sign.

### 3.3 Camera speed

The image shifts by 4.9 pixels from one frame to the next, and the true step is 2.81 m. That gives 0.57 m per pixel, so a frame covers 283 m. Matching gives about 271 m for the same quantity, so the two methods agree.

The scale changes along the route, between 0.45 and 0.69 m per pixel, because the ground height changes by 73 m under an aircraft that is 100 to 200 m up. So a scale goes stale:

| Scale taken from | Speed error, median | 90% of frames below |
|---|---|---|
| 100 m earlier | 5.7% | 13% |
| 300 m earlier | 8.6% | 18% |
| 1,000 m earlier | 17.2% | 28% |

Every fix renews the scale for free, because the zoom that makes the match fit says how many metres one pixel covers. Computing the image motion takes 9 ms per frame on a laptop.

### 3.4 End to end: GNSS, jam, camera only

This is the main result. Script: `experiments/h_alto_end_to_end.py`.

**What the system gets:** the camera frames, the reference images with their coordinates, and the GNSS position during the first 300 m. It does not use the altitude, the orientation, an IMU or any camera calibration.

**Before the jam, it learns three things from GNSS:**

- How a shift of the image in pixels translates into a step on the ground in metres east and north.
- At which zoom and rotation the camera frame fits the reference images: 0.85 and 10 degrees.
- A constant offset of the fixes: 3.6 m east and 6.6 m south.

**After the jam:**

- Every frame, the position is moved by the image shift, scaled with the zoom of the last fix. This is dead reckoning by camera.
- Every so many metres, the frame is matched against reference images near the current estimate. The result is blended with the estimate. A fix accurate to 15 m gets almost all the weight when the estimate is uncertain by 100 m, and little weight when the estimate is fresh.
- The true position is used only to measure the error.

**Results over the 4.3 km after the jam:**

| Fixes | Search | Score check | Median error | Worst | At the end | Fixes used, rejected | Used but wrong by over 50 m |
|---|---|---|---|---|---|---|---|
| None | | | 472 m | 657 m | 608 m | | |
| Every 100 m | 7 nearest images | no | 26 m | 50 m | 26 m | 39, 0 | 0 |
| Every 200 m | 7 nearest images | no | 30 m | 71 m | 45 m | 19, 0 | 0 |
| Every 300 m | 7 nearest images | no | 31 m | 83 m | 16 m | 13, 0 | 0 |
| Every 400 m | 7 nearest images | no | 285 m | 898 m | 898 m | 7, 0 | 5 |
| Every 500 m | 7 nearest images | no | 401 m | 1,247 m | 1,247 m | 5, 1 | 4 |
| Every 600 m | 7 nearest images | no | 609 m | 1,334 m | 1,334 m | 4, 0 | 4 |
| Every 800 m | 7 nearest images | no | 661 m | 949 m | 949 m | 4, 0 | 4 |
| Every 1,000 m | 7 nearest images | no | 426 m | 805 m | 805 m | 3, 0 | 3 |
| Every 300 m | 7 nearest images | yes | 32 m | 83 m | 38 m | 12, 1 | 0 |
| Every 1,000 m | 7 nearest images | yes | 472 m | 657 m | 608 m | 0, 3 | 0 |
| Every 300 m | sized by uncertainty | yes | 31 m | 73 m | 38 m | 12, 1 | 0 |
| Every 1,000 m | sized by uncertainty | yes | 56 m | 278 m | 10 m | 4, 0 | 0 |
| Every 2,000 m | sized by uncertainty | yes | 116 m | 578 m | 197 m | 1, 0 | 0 |

![End to end on ALTO](figures/alto_end_to_end.png)

**The cliff.** A fix can only be right if the true position lies inside the area that is searched. With the 7 nearest reference images, the frame can be found up to about 40 m from the estimate. Camera dead reckoning is 27 m off after 300 m and 62 m off after 500 m. So fixes work up to 300 m apart and fail from 400 m on, which is what the table shows. As a rule:

```
longest gap between fixes = size of the search / drift per metre flown
                          = 40 m / 0.10 = 400 m
```

**Why a wrong fix is worse than none.** It moves the estimate to a wrong place and tells the filter that the position is now well known. The next search then looks in the wrong area with confidence. All five runs with gaps of 400 m and more end 805 to 1,334 m off, against 608 m with no fixes.

**Repair 1, a minimum score.** A fix is used only if its score is at least 0.33. With gaps of 1,000 m all three wrong fixes are rejected, and the result is exactly the one without fixes. The system is then never worse than dead reckoning. The cost: with gaps of 300 m, one correct fix of 13 is rejected.

**Repair 2, a search sized by the uncertainty.** The filter knows roughly how far off it may be. The search covers all reference images within that distance, and a wider range of zooms after a long gap. With gaps of 1,000 m the four fixes are all correct and the median error is 56 m. With gaps of 2,000 m the one fix is correct.

**The first drift budget.** Camera dead reckoning with no fix is off by 27 m after 300 m, 62 m after 500 m, 193 m after 1,000 m and 465 m after 2,000 m. The drift grows faster than the distance because scale and direction go stale.

**Where that drift comes from.** At the end of the section the error is 575 m along the route and 198 m across it. The first comes from a scale that has become 13 percent too small, the second from a direction that is 3 degrees off. A heading reference, such as a sun sensor, would remove only the second part.

**What a fix costs.** One fix with 105 comparisons takes 285 ms on one processor core of a laptop. With the images shrunk to 250 pixels it takes 69 ms, and at 125 pixels 20 ms. The fix error on four test frames is the same at all three sizes, 4 to 15 m. A drone needs a fix every few seconds at most, which leaves a wide margin for a small board. It has not been run on one.

All 14 runs together take 78 seconds on a laptop.

### 3.5 An idea that did not hold up

The idea: the zoom of a match measures the height above ground. Altitude minus the ground height from an open elevation model predicts that zoom for any claimed position, so a fix at a wrong place could be caught by its zoom.

The test: the zoom does follow the height, but loosely. The correlation is 0.64, and on frames not used for fitting the zoom misses the prediction by 32 m of height in the median. That is too imprecise for a check. The test seemed to work, catching 58 of 60 wrong fixes, but for another reason: wrong matches prefer the smallest zoom (section 3.2). We keep the score check and drop this idea.

### 3.6 The navigator as shared code (Saturday)

The chain of section 3.4 now lives in `baseline/` on branch `alto-navigator`: `python baseline/scripts/run_alto_navigator.py`, about a minute for the validation section. It reproduces all seven runs that section 3.4 reports with the score check or the sized search, and the cliff, to the tenth of a metre; `baseline/tests/test_alto_navigator.py` checks that whenever the data is present.

**The navigator states its own uncertainty.** For every frame it gives sigma: 3 m at the jam, growing by 10 percent of the distance flown since the last fix, shrinking at every fix it uses. A status follows from it: tracking up to 30 m, degraded up to 100 m, lost above. How often the true error stays within 3 sigma:

| Run | Median error | Frames with error within 3 sigma |
|---|---|---|
| Camera alone | 472 m | 89% |
| Fix every 100 m | 26 m | 90% |
| Fix every 300 m | 31 m | 97% |
| Fix every 300 m, sized search, score check | 31 m | 100% |
| Fix every 1,000 m, sized search, score check | 56 m | 100% |
| Fix every 400 m, no check (the cliff) | 285 m | 24% |

When the navigator works, its stated uncertainty holds. When it breaks, it is badly overconfident, and that is exactly the case an integrity check has to catch.

**A limit, found by a test.** We gave the navigator a map whose coordinates are 200 m off, on the generated test flight. The first five fixes were rejected for lying too far from the estimate. After 600 m without a fix the allowed distance had grown to 189 m, and a sixth fix, with a confident score, landed 187 m away and was used. The distance check alone cannot catch a wrong place that lies inside the stated uncertainty. The test stays in the suite to document this.

**Agreement of three frames, the rule of the Tomahawk camera fix** (Irani and Christ 1994, see `docs/reading-notes.md`). Each fix is matched in three frames about 14 m apart; it counts only if at least two land within 10 m of each other after removing the dead-reckoned motion between them. Tomahawk is not a GNSS-denied design: its camera fixes followed GPS or terrain-matching updates, and it checked the frames against an accurate inertial system. Here the motion between frames comes from the camera itself. Tested without the score check, to see whether it could replace the tuned threshold of 0.33:

| Run | Median error, worst | Fixes used, wrong among them |
|---|---|---|
| Fix every 300 m, score check 0.33 | 31 m, 73 m | 12, 0 |
| Fix every 300 m, agreement instead | 31 m, 74 m | 13, 0 |
| Fix every 300 m, 7 nearest images, agreement instead | 139 m, 522 m | 8, 5 |
| Fix every 400 m, 7 nearest images, agreement instead | 241 m, 589 m | 3, 2 |
| Fix every 1,000 m, 7 nearest images, agreement instead | 424 m, 783 m | 3, 3 |

It does not work in this form. Frames 14 m apart share most of their ground, so a wrong place that fits one frame fits the next as well. All wrong fixes that got through had scores between 0.16 and 0.26, so the score check of 0.33 would have stopped them. The agreement check stays in the code as an option, switched off. Frames further apart, 100 m or more, would see different ground; that is untested.

### 3.7 The training section: a first look, without positions

The training section of the same flight was downloaded on Friday night and stopped at 10.25 of about 10.66 GB. The archive stores the three files with the positions at its very end, so they are missing. The images are complete and readable from the partial file:

- 10,436 camera frames and 2,853 reference images on the route: 28.5 km, six times the validation section.
- Ground: fields with forest edges, long stretches of dark forest, single houses, a large industrial site, villages and a dense town centre.
- Exposure swings strongly: average brightness per frame from 20 to 183 (on a 0 to 255 scale). About 16 percent of sampled frames have very low contrast, in stretches of up to roughly 760 m, mostly forest.
- In the town, the streets in the camera frame are rotated against the reference image by much more than the 14 degrees of the validation section. The heading probably changes along the route, which the rotation learned before the jam does not follow.

Dropbox has disabled the dataset link for the day ("downloaded too many times in a day"), and the GitHub pages of the dataset do not carry the position files. The test waits until the link opens again; only the last 0.42 GB are needed.

### 3.8 The search no longer knows the true path (Saturday morning)

**The mistake.** The reference images we searched (folder `offset_0_None`) are centred on the true flight path: image 0 has the same coordinates as camera frame 0, and all 459 lie within 2.8 m of the path. Inside one reference image a template can slide only about 48 m from its centre (a 340-pixel template in a 500-pixel image, at 0.6 m per pixel). So every fix landed within about 48 m of the true path, wherever the estimate was. The search used knowledge that a drone does not have.

**The repair.** One map, built from all five reference folders: the route itself and 20 and 40 m north and south of it, 2,295 images. They are crops of the same aerial photos and agree with their coordinates to within 0.9 pixels (0.5 m), measured by phase correlation of overlapping images. The map is 8,024 by 1,969 pixels; 36 percent of it holds imagery, a strip about 380 m wide along the route. The navigator now searches a circle around its own estimate, with a radius of 60 m or 3 sigma, whichever is larger, and only at places where the whole frame lies on imagery. One search takes about 0.13 s on one laptop core.

| Fixes | Score check | Reference images on the true path | One map, circle around the estimate |
|---|---|---|---|
| Every 100 m | no | 25.5 m median, 50.3 m worst | 25.2 m, 49.9 m |
| Every 300 m | yes | 30.9 m, 72.8 m | 31.1 m, 72.9 m |
| Every 400 m | no | 285.1 m, 897.9 m (7 nearest images, 5 wrong fixes used) | 36.0 m, 279.9 m (1 wrong fix used) |
| Every 1,000 m | yes | 56.3 m, 278.1 m (sized search) | 56.1 m, 278.1 m |

What this means:

- **The results of sections 3.4 and 3.6 hold.** They did not depend on knowing the path: between fixes the estimate stayed close enough for the true place to lie inside the search.
- **The cliff at 400 m belonged to the small search of 7 images.** With a circle sized by the uncertainty and no score check, fixes every 400 m give a median of 36 m, with one wrong fix used.
- **What is still easier than reality:** the map is a strip 380 m wide around the flown route, so a wide search meets fewer look-alike places than it would on a full map. The next test, on UAV-VisLoc with full satellite maps, removes that.
- The runs on the reference images stay in `baseline/configs/alto_navigator.yaml` for comparison. The map runs are the ones to report.

### 3.9 A second dataset, held out: UAV-VisLoc (Saturday)

**Data.** UAV-VisLoc (Xu et al. 2024): real drone photos looking straight down, one every 95 m, 400 to 550 m above ground, with GPS, height, heading (`Phi1`, where the nose points) and course (`Phi2`) per photo, and a Google Earth satellite map at 0.3 m taken 2.5 to 5 years after the photos. No IMU, no video, no forward camera. Flight 03 (Taizhou, 74 km) was used to develop; flights 01 (Changjiang, 66 km) and 04 (Taizhou, 83 km) were chosen as held out before they were looked at.

**What is real and what is simulated.** The fixes are real: real photos against the real map, searched around the estimate (`search: area`, map resampled to 1 m per pixel). The photos are too far apart for optical flow, so the dead reckoning between them is simulated: the true step, turned by the error of a heading sensor model, with a slowly wandering scale error and noise (`baseline/src/sensors/`). GNSS is lost after 1 km. Three seeds per run, each drawing its own heading and dead-reckoning errors.

**Facts measured on flight 03.** 80 percent of the photos are found within 30 m of the truth when searched around it (median 16 m). The matched position lies about 13 m ahead of the recorded one along the flight direction on every leg; learned in the drone's own frame (forward, right) instead of north and east, the median fix error falls from 18.6 to 13.1 m. The camera points along `Phi1`, which differs from the course by the crab angle against the wind, 4 to 13 degrees.

**Why the first version failed on unseen data.** After a few rightly refused fixes the stated uncertainty grows, the search widens to 300 to 450 m, and a look-alike place with a borderline score is found there and believed, although the estimate was right to within 13 to 33 m. Two checks were added: a fix out of a search wider than 150 m that would move the estimate by more than 30 m is held until the next fix, over different ground, agrees with it (both are then used); and the search is capped at 600 m. Ilhan's quarters rule (four quarters of the frame must land where the whole frame did) was also implemented; it refuses too many right fixes on ALTO and is off.

**Results** (median over 3 seeds; "all clear while wrong": the share of the flight where the stated bound is within 50 m but the error is above it):

| Flight | Run | Median | 90% below | Worst | Wrong fixes used, per seed | Error within 3 sigma | All clear while wrong |
|---|---|---|---|---|---|---|---|
| 03 (development) | Dead reckoning only | 822 m | 1,391 m | 1,571 m | | 100% | 0% |
| | First version | 41 m | 427 m | 1,301 m | 11, 12, 12 | 73% | 2.5% |
| | New checks, compass | 28 m | 180 m | 368 m | 0, 0, 2 | 97% | 0.0% |
| 01 (held out) | Dead reckoning only | 227 m | 596 m | 973 m | | 100% | 0% |
| | First version | 474 m | 1,604 m | 2,051 m | 17, 18, 24 | 48% | 3.1% |
| | New checks, compass | 306 m | 851 m | 1,493 m | 6, 7, 4 | 97% | 0.9% |
| | New checks, sun sensor | 177 m | 556 m | 1,514 m | 5, 6, 3 | 98% | 1.0% |
| 04 (held out) | Dead reckoning only | 675 m | 1,235 m | 1,805 m | | 100% | 0% |
| | First version | 285 m | 1,592 m | 2,244 m | 37, 31, 53 | 39% | 6.5% |
| | New checks, compass | 60 m | 1,723 m | 2,620 m | 18, 27, 32 | 77% | 3.0% |
| | New checks, sun sensor | 64 m | 1,206 m | 1,860 m | 17, 29, 35 | 76% | 3.2% |

What it says: the new checks cut wrong fixes by a third to three quarters on unseen flights and make the stated uncertainty far more honest, but they are not safe enough yet. Flight 01 crosses an area that was built up between the photos and the map: bare land in 2018, high-rise estates in 2023; no matcher can recognise that ground. The sun sensor helps where the compass is the weak part (flight 01). Independently, Ilhan's held-out test on ALTO Round 2 Train found the same weakness (94 m per section instead of 31 m), with the zoom as the main cause there (branch `research/offline-nav-evidence`).

### 3.10 The limits: a worse camera picture (Saturday)

On the ALTO validation flight every frame after the jam was made worse in one of three ways, five levels each (`baseline/src/data/degrade.py`): less light (sensor noise of a camera that turns its gain up), blur (Gaussian, in metres on the ground) and haze (a bright veil leaving a share of the contrast, plus sensor noise). Fixes every 300 m, map search (`baseline/scripts/run_limits.py`).

- **It holds** down to 1/64 of the light (31 m, no wrong fix), blur up to 2 m and haze down to half the contrast. Worse pictures make the check refuse fixes; across all fifteen levels one wrong fix passed.
- **A dangerous failure.** When the picture is bad enough that the camera cannot see the motion (blur from 4 m, haze below 25 percent), the dead reckoning reported no motion: the estimate stood still, its stated uncertainty stopped growing because it was tied to the distance the camera measured, and no fixes were tried. The navigator reported tracking, sure to a few metres, while 1 to 2 km off; the bound held in 0 percent of the frames.
- **The repair** (`camera_motion_floor: 0.3`): while GNSS works the navigator learns the cruising speed. A camera step shorter than 30 percent of the cruising step is not believed; the navigator flies on at cruising speed along the last good direction, and its uncertainty grows by 30 instead of 10 percent of the distance. Clean frames are unchanged.

| Picture after the jam | Without the repair | With it |
|---|---|---|
| Haze, 10 percent of the contrast left | 1,640 m, bound held 0% | 54 m, status LOST, bound held 100% |
| Haze, 2 percent | 2,045 m, 0% | 93 m, 100% |
| Blur 16 m | 2,048 m, 0% | 42 m, 91% |
| 1/1024 of the light | 556 m, 30% | 530 m, 59% |
| Blur 4 m | 1,091 m, 1% | 897 m, 1%, 4 wrong fixes used |

The last row is the blind spot that remains: the camera's motion is wrong but looks plausible, so the check rarely triggers. Catching it needs a second source of motion to cross-check, as Alessandro's IMU filter does. The fallback direction is the last good one, which suits a straight flight; through turns it needs a heading sensor.

## 4. How this compares with existing products

Both product pages describe the same building blocks.

| | Vantor Raptor Guide | UAV Navigation VNS01 | Our test on ALTO |
|---|---|---|---|
| Form | Software for the drone's own camera | A hardware unit for their autopilot | Scripts on a laptop |
| Dead reckoning | Left to a partner's inertial system | Visual odometry | Camera motion |
| Absolute fixes | Camera against the vendor's 3D terrain data | Template matching against imagery of earlier flights, satellite map matching, terrain matching | Template matching against reference images |
| Stated accuracy | Under 10 m, 1 to 5 fixes per second, also at night | None stated | Median 26 to 31 m with a fix every 100 to 300 m, daylight only |

Neither page shows what happens between fixes or when a fix is wrong. More in [landscape.md](landscape.md).

## 5. Where this leaves the project

**For staying with Challenge 2**

- Every required deliverable is within reach: a dead-reckoning baseline, a correction that reduces the error, plots of path and error, limits.
- The core already runs on real data as an experiment, with a result close to the 26 to 31 m of the paper we follow.
- The need is real. Taiwan is adopting a commercial product for it.
- Any other challenge would start from zero on Friday at 23:00.

**Against**

- The design is known. Camera plus dead reckoning plus map matching is what the products above sell. The brief does not score novelty, but we should not claim it.
- No single dataset shows everything. Mid-Air has no map, ALTO has no IMU, and the simulator is our own world.
- There is no product code yet, only experiment scripts.

**What is ours in this**

- The system calibrates itself while GNSS still works. It needs no camera calibration and no altimeter.
- A measured failure case with a rule that predicts it, and a check that makes the system never worse than dead reckoning.
- Limits measured on open data, stated in numbers.

## 6. What this changes in the plan (proposal, not yet agreed)

| Result | Data | State | Owner |
|---|---|---|---|
| 1. IMU-only baseline and its drift | Mid-Air | Experiment done (2.4). Product code on branch `mid-air-baseline-fix`; Alessandro's branch `mid-air-vio` carries the same gyroscope rule and adds visual-inertial odometry (2.6). The team has to pick which reaches `main` | |
| 2. Camera dead reckoning plus position fixes | ALTO | Shared code on branch `alto-navigator`, reproducing 3.4 (3.6). Held-out test blocked by the download (3.7). Turns still open | |
| 3. Integrity check and drift budget | ALTO | In the shared code: score check, distance check, stated uncertainty and status (3.6). Agreement of nearby frames tested and dropped. Needs degraded images: blur, darkness, haze | |
| All sensors in one flight, and the demo view | Simulator | Needs the GNSS cut, recording and export | |

Changes against the current plan:

- Camera speed is built on ALTO, where it works. On Mid-Air we show the measured limit from section 2.5.
- The fog run on Mid-Air loses its purpose, because camera speed there is already poor in sunshine. Degraded images on ALTO take its place.
- Route memory across seasons on Mid-Air is dropped.
- DenseUAV is an optional upgrade. If someone tries it, compare it on the same frames as `experiments/f_alto_matching.py`.
- A review of the DenseUAV idea is on branch `docs/denseuav-critical-review`. It asks how often an accepted fix is wrong, which section 3.4 measures for brightness matching. It proposes keypoints to verify a match. Section 3.2 shows that keypoints fail on ALTO and that brightness matching can take that role.
- In the simulator, the IMU should keep reporting around the drone's own axes, as a real IMU does. Only code that reads Mid-Air needs the rule from section 2.2.
- The islands world in the simulator fits the open-water question. Over water the camera sees nothing fixed, so there are no fixes and camera motion is unreliable as well. Our number for land without fixes, 465 m of error after 2 km, is the best case to expect for a crossing.

## 7. Limits of these findings

- ALTO: one section of 4.6 km of one flight, in daylight, in summer, over rural land, on a nearly straight course.
- UAV-VisLoc: the fixes are real, the dead reckoning between photos is simulated (section 3.9). Two held-out flights, three seeds each.
- The settings for ALTO were chosen while looking at this same section: the zoom and rotation ranges, the score threshold of 0.33, the assumed drift of 10 percent and the fix accuracy of 15 m. Nothing has been tested on data we did not tune on.
- The map covers only a strip about 380 m wide around the flown route (section 3.8). Places that look alike further away cannot confuse the match, so a wide search is easier than on a map of a whole area.
- The rotation is learned once before the jam and kept. That works on a straight course, and the validation section is one: its course stays between 77 and 84 degrees. Turns need a heading from the drone's own attitude, or a search over rotation at every fix.
- Mid-Air: the IMU baseline uses the 30 sunny flights. The scale test uses two flights.
- Computing time was measured only as a whole on a laptop. Nothing has run on drone hardware.
- Night, fog, rain and water are untested.

## 8. How to reproduce

Run from the repository root after `uv sync`. The data has to be in `data/raw/` as described in [data/README.md](../data/README.md).

| Script | What it measures | Section | Run time |
|---|---|---|---|
| `experiments/e_midair_imu_noise.py` | IMU errors in Mid-Air and which axes the gyroscope uses | 2.2, 2.3 | 10 s |
| `experiments/j_midair_imu_only.py` | IMU-only drift after the jam, with figure | 2.4 | 40 s |
| `experiments/g_midair_scale_check.py` | Camera speed with a barometer on Mid-Air | 2.5 | 2 min |
| `experiments/f_alto_matching.py` | Keypoint and brightness matching on ALTO | 3.2 | 40 s |
| `experiments/i_alto_zoom_check.py` | Score at right and wrong places, zoom against height | 3.2, 3.5 | 50 s |
| `experiments/k_alto_camera_speed.py` | Camera speed on ALTO and how fast the scale goes stale | 3.3 | 20 s |
| `experiments/h_alto_end_to_end.py` | The full chain on ALTO, with figure | 3.4 | 90 s |
| `experiments/l_alto_cost_and_drift.py` | Computing time of one fix at three image sizes, and the sources of camera-only drift | 3.4 | 10 s |
| `experiments/m_alto_orientation.py` | What the orientation values in ALTO mean: heading, and how far the camera looks away from straight down | 3.1 | 5 s |
| `baseline/scripts/run_alto_navigator.py` (branch `alto-navigator`) | The navigator as shared code: the runs of 3.4 with stated uncertainty, every fix and why it was used or not, figure | 3.6 | 1 min |
| `baseline/scripts/run_visloc_navigator.py` (`--flights 01 04` for the held-out run) | UAV-VisLoc runs with heading sensors, simulated dead reckoning and the integrity regions | 3.9 | 4 to 14 min |
| `baseline/scripts/run_limits.py` (`--floor 0.3` with the repair) | The camera picture made worse after the jam; chart and example frames | 3.10 | 3 min |
| `baseline/scripts/make_replay.py alto` or `visloc --flight 04` | The demo videos | | 1 to 5 min |
| `python -m pytest` (branch `alto-navigator`) | 134 tests, including the regressions on ALTO, the limit of the distance check and the camera losing track | 3.6 | 1.5 min |

`e` takes the path of another sensor file as its argument, for example the foggy one. `i` fetches a small piece of the Copernicus elevation model on its first run.
