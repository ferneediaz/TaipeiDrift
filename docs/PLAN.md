# Plan

The one page that says what we build, who does what, and by when. Everything else in `docs/` is background.

Status: revised on Friday night at 21:45. Mid-Air and ALTO data are downloaded and checked. Not yet confirmed by the team. Edit this page when something is decided.

## What we are building

Challenge 2, navigation without GNSS.

Software that keeps a drone's position estimate usable after GNSS is jammed, from the sensors the drone already carries. It also says how long the estimate can be trusted and flags when it cannot.

We are not building a drone. The assumed platform is an existing drone with:

- a camera pointing down
- an IMU (rotation and acceleration)
- an altitude sensor (a barometer, or a laser altimeter where the data has one)

We do not fix a flight height. The method takes the height as an input. We test it on the heights the datasets contain and state how the numbers change with height.

One-liner for the team form:

```
Keeps a drone on course after GNSS is jammed, using only its camera, IMU and barometer, and says when not to trust it.
```

## Datasets

How to get each dataset onto a laptop is in [data/README.md](../data/README.md).

### Mid-Air: IMU baseline, camera speed, fog

[Mid-Air](https://midair.ulg.ac.be/) is a synthetic dataset of low drone flights from the University of Liège.

- 54 flights with a downward camera at 25 frames per second, an IMU at 100 Hz with noise and drift, a simulated GNSS at 1 Hz, and exact ground truth at 100 Hz.
- Each flight is rendered in several conditions: sunny, cloudy, foggy and sunset in one landscape, and spring, fall and winter in another.
- Each flight lasts about 88 seconds and covers 1 to 1.4 km at 10 to 16 m/s. Measured on the sunny flights.
- No barometer. We simulate one from the true altitude, with realistic noise and slow drift.
- No aerial map of the landscapes, so no position fixes from a map.
- Licence: CC BY-NC-SA 4.0, non-commercial, with attribution.
- Downloaded on Friday night: the sensor records of every flight in every condition (0.8 GB), and the downward camera for 21 flights (about 9 GB): six in sun, the same six in fog, three each in spring, fall and winter. All downward-camera data together would be 100.6 GB.

### ALTO: position fixes on real images

[ALTO](https://github.com/MetaSLAM/ALTO) is a real dataset from a helicopter flight of 150 km at over 300 m ([paper](https://arxiv.org/abs/2207.12317)). The full dataset is not public. What is public is the sample made for the ICRA 2022 place-recognition competition, and we checked its validation section on Friday night.

- A 4.59 km section flown in 84 seconds, about 56 m/s, with 1,684 camera frames of 500 by 500 pixels, one every 2.8 m.
- For every frame: the true position, the altitude and the orientation of the camera.
- 459 reference images from an aerial survey five years earlier, one every 10 m along the flown route, plus copies shifted 20 and 40 m to the north and south.
- A file that names the correct reference image for every camera frame.
- No raw IMU: no accelerations and no rotation rates.
- No height above ground. The altitude is given above the Earth model, 376 to 455 m.
- The camera frames are rotated to the helicopter's heading, show less ground than the reference images, and are strongly green and overexposed. A frame has to be rotated and rescaled before it can be compared.
- The reference images form a ribbon along the route and do not cover an area. A match says how far along the route the aircraft is. Places off the route that look alike cannot confuse it, so the task is easier than matching against a full map.

Decision: use the validation section (1.73 GB) for position fixes. The training section (9.93 GB) is only needed if we train a model.

### Our own simulator: all sensors in one flight

Branch `simulations`, folder `sim/`. Gazebo in Docker, shown in a browser tab.

- A drone with a downward camera, an IMU with the Mid-Air noise model, a barometer with drift, GNSS that can be switched off, and exact ground truth.
- The ground can be the real 2020 aerial image of Wufeng, Taichung. The 2018 image of the same site can then serve as the on-board map.
- It is the only source we have where IMU, camera, barometer, GNSS and a map exist in one flight.
- Limits: flat ground, an area of 1 km by 1 km, IMU noise values that are our own assumption, and about half real-time speed on a MacBook.
- Check of the noise assumption against the 30 sunny Mid-Air flights (`experiments/e_midair_imu_noise.py`, a rough estimate): the accelerometer values fit. The gyroscope white noise in Mid-Air is about 0.02 rad/s per sample in the median and up to 0.07, against 0.0005 to 0.005 assumed in the simulator.
- Not there yet: the GNSS cut, recorded flights, and export in the shared format. Gazebo uses East, North, Up, so recordings have to be converted.
- Results from our own simulator are the weakest evidence for the jury, because we control both the world and the method. Its strength is showing the whole system working together.

Its role is open: see the open questions.

### Blackbird: not used

A drone flying fast loops inside one room, with camera images rendered afterwards. No meaningful altitude, no map and no distance covered.

## The scenario

GNSS works at the start of the flight and is then jammed. From that moment the software has to hold the position on its own. The error is measured against the reference path.

## The three parts

### 1. Navigator

- Baseline: dead reckoning from the IMU alone after GNSS is lost.
- Correction 1, camera speed: image motion from the downward camera, times the height, corrected for rotation with the IMU. This gives speed over ground. Built on Mid-Air.
- Correction 2, position fixes, built on ALTO:
  - The camera frame is compared with the reference images near the filter's current estimate. The best match gives the position.
  - First version: classical matching of brightness patterns, after rotating and rescaling the frame.
  - Optional upgrade: the pretrained DenseUAV network described in [data.md](data.md). It needs PyTorch, and its weights and its behaviour on a MacBook are untested.
  - ALTO has no raw IMU. Between fixes the movement comes from the camera, or from simulated movement measurements with stated noise, as in the Kinnari paper.
  - Fallback: route memory on Mid-Air. The same flight in another season or weather is the stored route.
- A filter that combines IMU, camera speed and fixes, and carries its own uncertainty.

Done when: plots show the true path, the IMU baseline and the filter, with position error over time for each combination.

### 2. Integrity check

- Each camera measurement and each fix is tested before it is used.
- The system flags when the camera cannot be trusted, for example in fog or over ground without texture, and when the estimate has drifted too long without a fix.

Done when: the same Mid-Air flight is run in sunny and in foggy conditions, and the flag rises where the camera speed goes wrong.

### 3. Drift budget

- For each condition, a curve of expected position error against time since GNSS was lost.
- It answers the operator's question: how long can this drone fly without GNSS before the position is off by more than a set limit?

Done when: one chart shows the curves for at least three conditions, and one sentence states the time to reach 50 m of error for each.

## The method behind position fixes and the filter

We take two ideas from the paper "Season-invariant GNSS-denied visual localization for UAVs" (Kinnari, Verdoja, Kyrki, 2022).

- Matching: for a guessed position, cut the square of map the camera should see, compare it with the camera view, and get a similarity score. A further step turns the score into a probability that the match is right. That step is the core of our integrity check.
- Monte Carlo localization: keep a thousand guesses of the position. Move each by the measured movement, weigh each by its match probability, keep the good ones. The spread of the guesses is the uncertainty.

The paper matches against a map that covers an area. The ALTO sample has reference images along the route only, so our guesses spread along the route and up to 40 m to each side. The paper's reported accuracy is 26 to 31 m on real flights, and it assumes a movement error of about 2 percent of distance flown, which is the bar for our camera speed.

- Explanation with worked numbers, and which sections to read: [method.md](method.md)
- Paper: [arXiv 2110.01967](https://arxiv.org/abs/2110.01967), and the earlier one it builds on: [arXiv 2103.14381](https://arxiv.org/abs/2103.14381)
- Both PDFs: [research/](../research/)

## Checks to run on the data before building on it

1. Scale. On one Mid-Air flight, compare speed from image motion times barometer height with the true speed. The barometer gives height above the start point, and Mid-Air flies low over hilly ground, so this may be off by a large factor. If it is, the camera speed needs another source of scale.
2. Heading. Mid-Air has no compass and the camera does not see the sky, so heading comes from the gyroscope and drifts. Measure how fast.
3. Route memory. Confirm that a flight follows the identical path in every season and weather. If it does, add a deliberate offset so the test is not trivially easy.
4. ALTO matching. For 100 camera frames, rotate and rescale the frame and compare it with the reference images within 150 m. Count how often the best match is the correct one. If classical matching fails on these images, the DenseUAV upgrade becomes necessary.

## How we measure

- Scenario: GNSS is cut at a fixed time after the start of each flight.
- Metrics: position error as a percentage of distance flown, and time until the error passes 50 m.
- Reference: the error is measured against the ground truth path. The simulated GNSS in Mid-Air is noisy and is only an input before the cut.
- Flights: settings are tuned on some flights and reported on others. Report the median over several flights, not one good run.
- Flight length: a Mid-Air flight lasts about 88 seconds, so every drift curve on Mid-Air ends there.
- Noise sweep: more IMU noise, more height error, blurred images. Show where the method breaks.
- Computing: milliseconds per camera frame on a plain CPU, and what board that implies.

## How the parts connect

```
flight data (IMU, camera frames, height, GNSS until the jamming moment, reference path)
        |
   data reader  ->  simulated barometer where needed, jamming time
        |
   navigator  <-  camera speed, position fix   (each with a confidence)
        |              ^
        |        integrity check accepts or rejects each one
        v
   estimate + uncertainty per time step  ->  error against the reference  ->  plots, drift budget, demo view
```

Shared conventions, fixed now so six people can work in parallel:

- Coordinates: North, East, Down in metres, origin at the start of each flight, as in Mid-Air. ALTO data is converted to the same.
- Units: metres, seconds, radians.
- A measurement handed to the filter is: value, uncertainty, source name, confidence between 0 and 1.
- Every result that goes on a slide is produced by a script in the repository.

## Roles

Write names here once agreed. Each person owns one part and can explain it alone.

| # | Role | Owns | Name |
|---|---|---|---|
| 1 | Data | Mid-Air and ALTO readers, simulated barometer, jamming scenario, recordings from the simulator | |
| 2 | Filter | IMU dead-reckoning baseline, the filter that combines everything | |
| 3 | Camera speed | Image motion, height and rotation turned into speed over ground | |
| 4 | Position fixes | Matching on ALTO, the DenseUAV upgrade if time allows | |
| 5 | Integrity and evaluation | Checks, fog and season runs, metrics, noise sweep, timing, drift budget | |
| 6 | Demo and pitch | Demo view, slides, user, platform, the Taiwan case, deployment | |

## Timeline

Demo Day is Sunday 13:00. Code freeze is Sunday 10:00.

| When | What | Gate |
|---|---|---|
| Friday night | Confirm this plan, assign roles, request the Mid-Air links, check ALTO, read the sensor records | A plot shows the IMU-only path drifting away from the true path. ALTO is decided, yes or no |
| Saturday 09:00 to 13:00 | The three data checks. Camera speed on one sunny flight, filter combining it with the IMU | The correction beats the baseline in a plot. The brief's minimum is met |
| Saturday 13:00 to 14:00 | Show mentors, write down what they say | |
| Saturday 14:00 to 19:00 | Position fixes, fog and season runs, integrity check, drift budget, noise sweep, timing | Same flight compared across at least two conditions |
| Saturday 19:00 to 22:00 | Demo view, fallback video, slide draft | Video file saved |
| Sunday 08:30 to 10:00 | Bug fixes only | |
| Sunday 10:00 | Code freeze | Demo branch tagged |
| Sunday 10:00 to 13:00 | Rehearse three times, submit | Submission confirmed |

If we fall behind, cut in this order: position fixes, the season runs, the demo view. The IMU baseline, camera speed, one fog run and the drift budget are the smallest complete entry.

After tonight, no further datasets are considered.

## Demo

1. The downward camera video plays next to a map of the flight. GNSS is switched off.
2. The IMU-only estimate leaves the true path within seconds.
3. With camera speed, the estimate stays close. One chart shows both errors over time, with one headline number.
4. A position fix arrives and the error drops back.
5. The same flight in fog. The camera speed goes wrong, and the system flags it.
6. The drift budget: how long each condition allows before the error passes 50 m.
7. One slide: the sensors assumed, how the result changes with flight height, and what is not covered.

## What the pitch has to contain

The brief scores the user and product side. Role 6 collects it, everyone contributes.

- User: operators of small drones that lose GNSS under jamming.
- The Taiwan case: interference around the outlying islands is reported regularly, and Taiwan is adopting a map-based product that needs its vendor's data. Our method holds the position between map fixes and says for how long.
- Platform: which sensors and how much computing the method needs.
- Deployment: how it would run on an existing autopilot, and what a real test would require.
- Evidence on real data: position fixes on the ALTO helicopter images. A phone also has a camera, an IMU, a barometer and GNSS, so a two-minute walk with the camera pointing down gives a real recording for the same method.

## What we claim and what we do not

Claim: after GNSS is lost, the method holds the position far longer than the IMU alone, using sensors the drone already has. It tells the operator how long, and it flags when the camera cannot be trusted.

Do not claim:

- Results on real flights, unless ALTO or a phone recording provides them.
- A position without a known start. The method continues from the last GNSS position.
- Performance at a flight height we have not tested.
- That drift is removed. Camera speed slows it. Only a position fix resets it.

## Challenges the drone could face

Conditions that trouble a drone navigating with a downward camera, an IMU and a barometer. To think through before building: which of these the software must handle, which it must detect, and which are out of scope.

### Ground below

- Open water. Nothing fixed to see, and waves move on their own.
- Coastline and tidal flats. The shore moves with the tide, so it differs from any stored picture.
- Featureless ground. Snow, sand, mudflats and large fields of one crop give the camera nothing to hold on to.
- Repeating ground. Fields, forest, plantations and housing blocks look the same in many places.
- Hills and mountains. The ground rises and falls under the drone, so height above take-off is not the distance to the ground.
- Tall objects at low flight height. Buildings and trees look different from every angle and hide the ground.
- Moving things. Traffic, ships, crowds, tree tops in wind and water surfaces move independently of the drone.
- A changed landscape. Construction, floods, harvest, fire or battle damage make the ground differ from what was recorded earlier.

### Light and weather

- Night. A normal camera sees nothing.
- Cloud or fog below the drone. The ground is hidden.
- Haze, smoke and dust. Contrast fades with distance.
- Rain, spray and condensation on the lens. Blur and droplets.
- Low sun. Long shadows that move during the day.
- Glare. Sun reflecting off water, wet roads or roofs.
- Sudden brightness changes. Crossing from dark land to bright sea, or in and out of cloud shadow.
- Seasons. Snow, leaf fall and crop cycles change how the same place looks.
- Changing air pressure. A weather front shifts the barometer reading during a flight.
- Wind and gusts. They push the drone sideways and shake it.
- Cold and heat. Temperature changes shift the IMU's errors, and ice or fog can form on the lens.

### The flight

- Fast rotation. Turns and corrections blur the image and swamp the motion signal.
- Low and fast. The ground crosses the image too quickly to follow.
- High flight. Small tilt errors become large position errors on the ground.
- Climbing and descending. The scale of the image changes continuously.
- Vibration from motors and propellers, into both IMU and camera.
- Long duration. Every slowly growing error has more time to grow.
- Airflow over the barometer. Speed and propeller wash change the pressure it reads.

### Sensors and hardware

- IMU drift. Its small errors add up quickly when nothing corrects them.
- No compass. Heading has no absolute reference.
- Camera mounting and calibration. A slightly tilted or miscalibrated camera gives a constant error.
- Timing between sensors. Camera and IMU readings a few milliseconds apart do not describe the same instant.
- Camera limitations. Motion blur, rolling shutter, limited resolution, slow exposure adjustment.
- Limited computing and power on board. Image processing may not keep up.
- A sensor failing in flight. A blocked lens, a frozen barometer, a saturated IMU.

### The adversary

- Jamming from take-off. There is never a trusted GNSS position to start from.
- Intermittent jamming. GNSS comes and goes, and each return may or may not be trustworthy.
- Spoofing. False GNSS signals that look valid and lead the drone astray.
- Smoke screens and camouflage. Deliberate hiding of the ground.
- Dazzling. Lasers or strong lights aimed at the camera.
- No link to the operator. Nobody can correct the drone from outside.

### Knowledge the drone starts with

- Unknown start position or heading.
- No map of the area, or an outdated one.
- A route never flown before.
- A map taken in a different season or at a different time of day.

### Most serious for Taiwan

1. Open water, because of the Strait and the outlying islands.
2. Cloud, haze and rain, because of the climate.
3. Night, because that is when incursions are hardest to see.
4. Mountains, because about two thirds of the island is steep terrain.
5. Jamming from take-off and spoofing, because the interference is already there before the drone launches.

## Known weak points for the limits slide

- The barometer gives height above the start point. Over hills the true distance to the ground differs, and the speed estimate is off by the same proportion.
- The simulated barometer is derived from the true altitude. Its noise and drift model must be stated.
- Heading drifts when it comes from the gyroscope alone.
- Higher flight makes the camera see more ground and makes attitude errors count more. At 3,000 m a tilt error of 1 degree is about 52 m on the ground.
- Fog, darkness and ground without texture stop the camera from measuring speed.
- Camera-plus-IMU navigation is well studied. What is ours is the integrity check and the drift budget across conditions.

## Open questions

- Does the team confirm this plan, and who takes which role?
- Position fixes: do we agree on classical matching first and DenseUAV as an upgrade? [data.md](data.md) proposes DenseUAV from the start.
- What is the simulator for: the demo view, a test that all parts work together in one flight, or both? It does not replace Mid-Air and ALTO as evidence.
- Where does the height above ground come from on ALTO, if we want camera speed there?
- Which second input did the mentor name for the cold start, besides the position of the sun? He has not seen this version of the plan.
- How long is the demo slot, what are the judging weights, and what is the submission format?

## Mentors on site

From the participant page. Remove this section before the repository is made public.

| Mentor | Field | Most useful for |
|---|---|---|
| Milosch Meriac, CTO at [Bitqan Systems Design](https://bitqan.ae/about.html) | GNSS security, RF localisation, signal processing, acoustics, embedded hardware | The navigation design, sensor hardware and cost |
| MC (@minmax) | Sensor data and systems integration, maritime | Sensor fusion, the maritime side |
| Wenteng Chang | Multi-UAV task allocation, replanning, human-in-the-loop, demo and pitch | Operator workflow, pitch structure |
| Oleksandr Kulyniak | Current warfare challenges | Reality check for any idea |
| Caine Cortellino | Test scenarios based on battlefield conditions, government contracting | Failure scenarios, deployment slide |
| Yen Chang | RC planes | What a small aircraft can carry |
| Paruyr Abrahamyan | Defence industry, sales pitch | Pitch rehearsal |

## Background

- [brief.md](brief.md): what the challenge asks for
- [data.md](data.md): how Mid-Air and the DenseUAV model would be used, by Alessandro
- [method.md](method.md): how image matching and the particle filter work, with worked numbers
- [landscape.md](landscape.md): existing products and their limits
- [experiments.md](experiments.md): earlier measurements on Taiwan imagery and elevation, made before the dataset was chosen
- [challenge-2-research.md](challenge-2-research.md): papers, data sources, reading list
- [playbook.md](playbook.md): working rules, slide skeleton, submission checklist
