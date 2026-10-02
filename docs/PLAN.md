# Plan

The one page that says what we build, who does what, and by when. Everything else in `docs/` is background.

Status: revised on Friday night at 22:55, after the measurements in [findings.md](findings.md). The team decided on Friday night to stay with Challenge 2. The three results below and their owners are proposed and not yet confirmed. Edit this page when something is decided.

Every number on this page comes from [findings.md](findings.md).

## What we are building

Challenge 2, navigation without GNSS.

Software that keeps a drone's position estimate usable after GNSS is jammed, from the sensors the drone already carries. It also says how long the estimate can be trusted and flags when it cannot.

The target, set with our mentor on Friday night: navigation for low-cost drones. A price of about 500 dollars is the orientation for that class and not a hard cap. Such a drone has a camera, an IMU and a barometer. Parts that cost as much as the drone itself do not fit it: a graphics processor, a thermal camera, a radar altimeter or licensed map data.

The one thing we do well: position fixes from the camera the drone already has, against freely available aerial images, with a check that keeps a wrong fix from doing damage. Everything else on this page is either a baseline for that or a stated next step.

We are not building a drone. The assumed platform is an existing drone with:

- a camera pointing down
- an IMU (rotation and acceleration)
- an altitude sensor (a barometer, or a laser altimeter where the data has one)

On the real ALTO flight our result so far uses the camera alone, because that data has no IMU. The IMU carries the baseline on Mid-Air, and it is what holds the heading in turns.

We do not fix a flight height. The system learns the scale of the camera image while GNSS still works and renews it at every position fix, so it needs no sensor for the distance to the ground. It is tested on a flight roughly 100 to 200 m above the ground. Low flight that follows hilly terrain is a measured limit.

One-liner for the team form:

```
Keeps a drone on course after GNSS is jammed, using only its camera, IMU and barometer, and says when not to trust it.
```

## Datasets

How to get each dataset onto a laptop is in [data/README.md](../data/README.md).

### Mid-Air: the IMU baseline

[Mid-Air](https://midair.ulg.ac.be/) is a synthetic dataset of low drone flights from the University of Liège.

- 54 flights with a downward camera at 25 frames per second, an IMU at 100 Hz with noise and drift, a simulated GNSS at 1 Hz, and exact ground truth at 100 Hz.
- Each flight is rendered in several conditions: sunny, cloudy, foggy and sunset in one landscape, and spring, fall and winter in another.
- Each flight lasts about 88 seconds. The 30 sunny flights cover 270 m to 1.5 km, 1.0 km in the median, at 3 to 17 m/s.
- No barometer and no distance to the ground. The drone flies about 15 m above hilly terrain, and camera speed with a simulated barometer does not work there: the speed error was 37 and 161 percent on two flights.
- The gyroscope is given around the map's axes, which a textbook filter does not expect. The rule for reading it is in section 2.2 of the findings.
- No aerial map of the landscapes, so no position fixes from a map.
- Licence: CC BY-NC-SA 4.0, non-commercial, with attribution.
- Downloaded on Friday night: the sensor records of every flight in every condition (0.8 GB), and the downward camera for 21 flights (about 9 GB): six in sun, the same six in fog, three each in spring, fall and winter. All downward-camera data together would be 100.6 GB.

### ALTO: camera dead reckoning and position fixes on real images

[ALTO](https://github.com/MetaSLAM/ALTO) is a real dataset from a helicopter flight of 150 km at over 300 m ([paper](https://arxiv.org/abs/2207.12317)). The full dataset is not public. What is public is the sample made for the ICRA 2022 place-recognition competition, and we checked its validation section on Friday night.

- A 4.59 km section flown in 84 seconds, about 56 m/s, with 1,684 camera frames of 500 by 500 pixels, one every 2.8 m.
- For every frame: the true position, the altitude and the orientation of the camera.
- 459 reference images from an aerial survey five years earlier, one every 10 m along the flown route, plus copies shifted 20 and 40 m to the north and south.
- A file that names the correct reference image for every camera frame.
- No raw IMU: no accelerations and no rotation rates.
- No height above ground. The altitude is given above the Earth model, 376 to 455 m. According to the open elevation model the helicopter is roughly 100 to 200 m above the ground in this section.
- The camera frames are rotated to the helicopter's heading, show less ground than the reference images, and are strongly green and overexposed. A frame has to be rotated and rescaled before it can be compared.
- The reference images form a ribbon along the route and do not cover an area. A match says how far along the route the aircraft is. Places off the route that look alike cannot confuse it, so the task is easier than matching against a full map.

Decision: the validation section (1.73 GB) is where camera dead reckoning, position fixes, the integrity check and the drift budget are built. The training section (9.93 GB, roughly 30 km of the same flight) is the data to report on, because we did not tune on it. Whether to download it is open.

### Our own simulator: all sensors in one flight

Branch `simulations`, folder `sim/`. Gazebo in Docker, shown in a browser tab.

- A drone with a downward camera, an IMU with the Mid-Air noise model, a barometer with drift, GNSS that can be switched off, and exact ground truth.
- The ground can be the real 2020 aerial image of Wufeng, Taichung. The 2018 image of the same site can then serve as the on-board map.
- It is the only source we have where IMU, camera, barometer, GNSS and a map exist in one flight.
- Limits: IMU noise values that are our own assumption, and about half real-time speed on a MacBook. The first world is flat ground of 1 km by 1 km. A second world with two islands was added on Friday at 22:19.
- Check of the noise assumption against the 30 sunny Mid-Air flights (`experiments/e_midair_imu_noise.py`, a rough estimate): the accelerometer values fit. The gyroscope white noise in Mid-Air is about 0.02 rad/s per sample in the median and up to 0.07, against 0.0005 to 0.005 assumed in the simulator.
- Not there yet: the GNSS cut, recorded flights, and export in the shared format. Gazebo uses East, North, Up, so recordings have to be converted.
- Results from our own simulator are the weakest evidence for the jury, because we control both the world and the method. Its strength is showing the whole system working together.

Its role is open: see the open questions.

### Blackbird: not used

A drone flying fast loops inside one room, with camera images rendered afterwards. No meaningful altitude, no map and no distance covered.

## The scenario

GNSS works at the start of the flight and is then jammed. From that moment the software has to hold the position on its own. The error is measured against the reference path.

## The three results

Each result has a dataset, a state and a "done when". The measurements behind the states are in [findings.md](findings.md).

### 1. IMU-only baseline, on Mid-Air

- Dead reckoning from the IMU alone after GNSS is lost. This is the baseline the challenge asks for.
- State: measured. The position is 485 m off after 78 seconds and passes 50 m after 36 seconds, in the median of 30 flights. Product code is on branch `mid-air-baseline-fix`, which is the branch `mid-air-baseline` plus the gyroscope rule from section 2.2 of the findings. It gives the same numbers as the experiment script.
- Camera speed is not built on Mid-Air. The measured limit from section 2.5 of the findings goes on the limits slide.

Done when: the baseline code is merged into `main` and produces the plot of true path, IMU-only path and error over time for the 30 sunny flights.

### 2. Camera navigator, on ALTO

- Before the jam the system learns from GNSS how image motion translates into ground motion, and at which zoom, rotation and offset the camera frame fits the reference images.
- After the jam the position is carried forward by image motion. This is dead reckoning by camera, and it is the second baseline: 472 m median error over 4.3 km.
- Position fixes: the camera frame is compared with the reference images near the current estimate, by matching brightness patterns. A filter blends each fix with the estimate and carries the uncertainty.
- State: works as an experiment, with a median error of 26 to 31 m for a fix every 100 to 300 m.
- Still to do: a test on the training section, which we did not tune on. Turns, with the heading from a gyroscope or a search over rotation. A proper filter in place of the simple blend in the experiment.
- Optional upgrade: the pretrained DenseUAV network described in [data.md](data.md), compared on the same frames as `experiments/f_alto_matching.py`. It needs PyTorch, and its weights and its behaviour on a MacBook are untested. A review of that idea is on branch `docs/denseuav-critical-review`.

Done when: on data we did not tune on, a plot shows the true path, the camera-only path and the path with fixes, with the error over distance.

### 3. Integrity check and drift budget, on ALTO

- A fix is used only if its match score passes a threshold and it agrees with the filter's estimate. With this check the system is never worse than dead reckoning.
- The search grows with the filter's uncertainty, so a long gap without fixes can be recovered.
- The system reports a status: trusted, degraded when no fix has been accepted for longer than the search can absorb, and lost.
- Drift budget: the error against the distance since the last fix, and the longest gap the search can absorb. The rule is the size of the search divided by the drift per metre flown. For example, 40 m divided by 0.10 gives 400 m.
- Degraded images: blur, darkness and haze are added to the camera frames, to show where fixes get rejected and what the status then says.
- State: the score check and the sized search work as an experiment. With both, fixes 1,000 m apart give a median error of 56 m, where the plain version ends further off than with no fixes.

Done when: one chart shows the error against distance for at least three conditions, and a run with a fix after too long a gap shows the check rejecting it.

## The method behind position fixes and the filter

We take two ideas from the paper "Season-invariant GNSS-denied visual localization for UAVs" (Kinnari, Verdoja, Kyrki, 2022).

- Matching: for a guessed position, cut the square of map the camera should see, compare it with the camera view, and get a similarity score. A further step turns the score into a probability that the match is right. That step is the core of our integrity check.
- Monte Carlo localization: keep a thousand guesses of the position. Move each by the measured movement, weigh each by its match probability, keep the good ones. The spread of the guesses is the uncertainty. We start from a known position, so our experiment keeps one estimate with an uncertainty. The thousand guesses become necessary for a start without a position, which is out of scope.

The paper matches against a map that covers an area. The ALTO sample has reference images along the route only, which makes our task easier. The paper reports 26 to 31 m on real flights; our experiment reaches a median of 26 to 31 m with a fix every 100 to 300 m. The paper assumes a movement error of about 2 percent of distance flown; our camera dead reckoning is at about 10 percent.

- Explanation with worked numbers, and which sections to read: [method.md](method.md)
- Paper: [arXiv 2110.01967](https://arxiv.org/abs/2110.01967), and the earlier one it builds on: [arXiv 2103.14381](https://arxiv.org/abs/2103.14381)
- Both PDFs: [research/](../research/)

## Checks on the data

| Check | Result |
|---|---|
| Scale: does a barometer give the scale for camera speed on Mid-Air? | Done. No: 37 and 161 percent speed error on two flights |
| ALTO matching: does classical matching find the right place? | Done. Keypoints fail. Brightness matching puts 21 of 24 frames within 20 m |
| Held-out test: do the ALTO results hold on the training section? | Open |
| Heading: how fast does a heading from the gyroscope drift? | Open. Needed for turns |
| Route memory across seasons on Mid-Air | Dropped. ALTO replaced its purpose |

## How we measure

- Scenario: GNSS is cut at a fixed time after the start of each flight.
- Metrics: position error in metres and as a percentage of distance flown, and the time or distance until the error passes 50 m.
- Reference: the error is measured against the ground truth path. The simulated GNSS in Mid-Air is noisy and is only an input before the cut.
- Flights: settings are tuned on some data and reported on other data. On Mid-Air report the median over the 30 sunny flights. On ALTO tune on the validation section and report on the training section.
- Flight length: a Mid-Air flight lasts about 88 seconds, so every drift curve on Mid-Air ends there.
- Noise sweep: more IMU noise on Mid-Air, and blurred, darkened and hazy camera frames on ALTO. Show where the method breaks.
- Computing: milliseconds per camera frame and per fix on a plain CPU, and what board that implies. Measured on one processor core of a laptop: image motion takes 9 ms per frame, and one fix takes 285 ms at 500 pixels, 69 ms at 250 pixels and 20 ms at 125 pixels, with the same fix error on four test frames. Not yet measured on a small board.

## How the parts connect

```
flight data (IMU, camera frames, height, GNSS until the jamming moment, reference path)
        |
   data reader  ->  simulated barometer where needed, jamming time
        |
   navigator  <-  camera motion, position fix  (each with a confidence)
        |              ^
        |        integrity check accepts or rejects each one
        v
   estimate + uncertainty per time step  ->  error against the reference  ->  plots, drift budget, demo view
```

Shared conventions, fixed now so six people can work in parallel:

- Coordinates: North, East, Down in metres, origin at the start of each flight, as in Mid-Air. ALTO data is converted to the same.
- Units: metres, seconds, radians.
- A measurement handed to the filter is: value, uncertainty, source name, confidence between 0 and 1.
- Gyroscope: turn rates around the drone's own axes, as a real IMU reports them. Mid-Air stores them around the map's axes, so its loader marks that and the estimator applies the turn step on the matching side.
- Every result that goes on a slide is produced by a script in the repository.

## Roles

Write names here once agreed. Each person owns one part and can explain it alone.

| # | Role | Owns | Name |
|---|---|---|---|
| 1 | IMU baseline on Mid-Air | Loader, IMU dead reckoning, metrics and plots, merged into `main` | |
| 2 | Camera navigator on ALTO | ALTO reader, camera dead reckoning, the filter, turns | |
| 3 | Position fixes | Brightness matching, the search, the DenseUAV comparison if time allows | |
| 4 | Integrity and drift budget | Score and consistency checks, status, degraded images, held-out test, timing | |
| 5 | Simulator | GNSS cut, recording and export, all sensors in one flight | |
| 6 | Demo and pitch | Demo view, slides, user, platform, the Taiwan case, deployment | |

Work that exists already: the baseline code (Alessandro), the simulator (Dan), a review of the DenseUAV idea (Ilhan), and the experiment scripts for results 2 and 3 in `experiments/`.

## Timeline

Demo Day is Sunday 13:00. Code freeze is Sunday 10:00.

| When | What | Gate |
|---|---|---|
| Friday night | Done: data downloaded and checked, IMU-only drift measured, the ALTO chain run as an experiment, decision to stay with the challenge | Met: the plot of IMU-only drift exists, and ALTO is decided |
| Saturday 09:00 to 13:00 | Agree roles. Merge the baseline. Move camera dead reckoning and fixes from the experiment into shared code. Download the ALTO training section and run the held-out test | The correction beats the baseline in a plot made by the shared code, on data we did not tune on. The brief's minimum is met |
| Saturday 13:00 to 14:00 | Show mentors, write down what they say | |
| Saturday 14:00 to 19:00 | Integrity check and status, degraded images, drift budget, turns, noise sweep, timing. Simulator: GNSS cut and recording | A fix after too long a gap is shown and rejected. The drift budget chart exists |
| Saturday 19:00 to 22:00 | Demo view, fallback video, slide draft | Video file saved |
| Sunday 08:30 to 10:00 | Bug fixes only | |
| Sunday 10:00 | Code freeze | Demo branch tagged |
| Sunday 10:00 to 13:00 | Rehearse three times, submit | Submission confirmed |

If we fall behind, cut in this order: the simulator flight, the DenseUAV comparison, turns, the demo view. The IMU baseline on Mid-Air, camera dead reckoning with fixes on ALTO, the rejected wrong fix and the drift budget are the smallest complete entry.

After tonight, no further datasets are considered.

## Demo

1. Mid-Air: GNSS is switched off and the IMU-only estimate leaves the true path. One number: 50 m off after 36 seconds.
2. ALTO, real helicopter images: the camera video plays next to the route. GNSS is switched off after 300 m.
3. Camera only: the estimate drifts away, to a median error of 472 m.
4. With position fixes: the error drops back at every fix and stays at about 30 m.
5. A fix after too long a gap. Without the check it is accepted and the estimate jumps to a wrong place. With the check it is rejected and the status shows degraded.
6. The drift budget: how far the drone can fly between fixes before the search can no longer recover.
7. One slide: the sensors assumed, what the system learns from GNSS before the jam, and what is not covered.

### A second demo on real, cheap sensors: the phone walk

Proposed by our mentor. A phone has the same class of sensors as a cheap drone. Nobody has tried this yet.

- Walk a loop of 150 to 300 m on textured ground in daylight, with the phone held flat at chest height and the camera pointing straight down. Measure the holding height with a tape. That height is the distance to the ground, and it stays constant, which is what camera speed needs.
- Record video at a high bitrate with the stabilisation switched off and with exposure and focus locked, at 60 frames per second if possible. Stabilisation shifts the picture and falsifies the image motion. Log GNSS, IMU and barometer at the same time, and tap the phone once at the start so that both recordings can be lined up.
- Walk the loop twice. The first pass, with GNSS, is the reference. The second pass is the test: GNSS is used for the first 30 m and then ignored.
- What it shows: camera dead reckoning on real hardware, with the scale learned from GNSS. With the first pass as stored reference images, it can also show position fixes.
- How it is scored: the walk ends where it started, so the gap between the estimated start and end is an exact error. Phone GNSS is only good to about 3 to 5 m, which is too coarse to score a walk of 200 m.
- What to expect: a height held to within 5 cm of 1.3 m gives a speed error of about 4 percent from the height alone. Tilt and the bounce of walking come on top.

## What the pitch has to contain

The brief scores the user and product side. Role 6 collects it, everyone contributes.

- User: operators of small drones that lose GNSS under jamming.
- The Taiwan case: interference around the outlying islands is reported regularly, and Taiwan is adopting a map-based product that needs its vendor's data. Our method holds the position between map fixes and says for how long.
- Platform: which sensors and how much computing the method needs.
- Deployment: how it would run on an existing autopilot, and what a real test would require.
- Evidence on real data: camera dead reckoning and position fixes on the ALTO helicopter images. A phone also has a camera, an IMU, a barometer and GNSS, so a two-minute walk with the camera pointing down gives a real recording for the same method.
- Where we stand against existing products: Raptor and VNS01 use the same building blocks. Say this openly. Our angle is that it works with openly available reference images and without calibration, and that we state measured limits.

### The story for the slides

One thing done well, then what comes next.

1. The problem: jamming, and why the existing products do not fit a low-cost drone of around 500 dollars. Raptor needs a graphics processor and licensed 3D data. VNS01 is a dedicated unit that uses a radar altimeter where it cannot see.
2. What such a drone has, and how fast its IMU alone drifts: 50 m after 36 seconds.
3. Our one thing: fixes from the drone's own camera against free aerial images. On a real flight, 472 m of drift becomes about 30 m.
4. Knowing when not to trust it: the wrong fix, the check that rejects it, and the rule for how far the drone can fly between fixes.
5. What it costs: no extra sensor, no calibration, and one fix in a fraction of a second on one processor core.
6. The limits, measured.
7. The next steps, each with what it would require.

### Next steps and what each would require

Ideas from our mentor for the gaps we do not close this weekend.

| Gap | Idea | What it gives | What it would require |
|---|---|---|---|
| Heading in turns, and a start without GNSS | A sun sensor: a line sensor behind a slit, read by a microcontroller. An infrared filter makes the sun stand out through haze | An absolute heading, which is what our chain lacks in turns. A position only roughly: 1 degree of error in the vertical is 111 km on the ground | The sensor for a few dollars, the time of day, and the tilt from the IMU |
| Night | The moon and the stars | A heading from the moon in the same way. Light for the camera only with a more sensitive or an infrared sensor | Moon tables, another image sensor |
| Open water | Signals of opportunity: FM radio stations, which name themselves in their RDS signal, and TV stations. The positions of their masts are known. BAE Systems calls its version NAVSOP | A position where there is no ground to look at | A software radio receiver, a table of mast positions, and signal processing we have not built |

On our straight test section the direction is the smaller part of the camera-only drift. At the end, the error is 575 m along the route, from the scale, and 198 m across it, from the direction. A heading reference matters most in turns.

On radio over water: by the usual rule for radio range, a drone at 100 m hears a mast on a 1,000 m mountain at up to about 170 km. The Taiwan Strait is 130 to 180 km wide. This is an estimate from the rule and not a measurement.

## What we claim and what we do not

Claim: after GNSS is lost, a camera and stored reference images hold the position to a few tens of metres on a real flight, where the camera alone or the IMU alone drift by hundreds. The system needs no calibration beyond a few hundred metres of flight with GNSS, it rejects fixes it should not trust, and it tells the operator how far it can fly between fixes.

Do not claim:

- Results beyond one flight section, in daylight, over rural land.
- A position without a known start. The method continues from the last GNSS position.
- Performance at a flight height we have not tested.
- That drift is removed. Only a position fix resets it.
- A new design. Existing products use the same building blocks.
- One flight with all sensors. The IMU result is on Mid-Air and the camera result is on ALTO.

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

- The barometer gives height above the start point. Over hills the true distance to the ground differs, and the speed estimate is off by the same proportion. Measured on Mid-Air: 37 and 161 percent.
- A fix after too long a gap lands in a wrong place. Without the score check the run ends further off than with no fixes.
- The reference images cover only the flown route, and the settings were chosen on the same section we report on, until the held-out test is done.
- Heading drifts when it comes from the gyroscope alone. On ALTO the rotation is learned once and kept, which works only on a straight course.
- Higher flight makes the camera see more ground and makes attitude errors count more. At 3,000 m a tilt error of 1 degree is about 52 m on the ground.
- Fog, darkness, water and ground without texture stop the camera from measuring motion and from getting fixes.
- The design is known and sold as products. What is ours is the calibration from GNSS before the jam, the measured failure case with its rule, and limits stated in numbers.

## Open questions

- Does the team confirm the three results, and who takes which role?
- Position fixes: do we agree on brightness matching first and DenseUAV as an upgrade? [data.md](data.md) proposes DenseUAV from the start.
- What is the simulator for: the demo view, a test that all parts work together in one flight, or both? It does not replace Mid-Air and ALTO as evidence.
- Do we download the ALTO training section (9.93 GB) for the held-out test?
- Answered: existing products do not work at night with an ordinary camera. Raptor uses an infrared camera, and VNS01 falls back on a radar altimeter. Details and sources are in [landscape.md](landscape.md).
- Answered: besides the sun, the mentor named radio signals of opportunity for finding the position. See "Mentor feedback" below.
- What did the mentor mean by "lightex technology", and which paper on a line sensor for the sun did he refer to?
- Who records the phone walk, and with which phone and app?
- How long is the demo slot, what are the judging weights, and what is the submission format?

## Mentor feedback, Friday night

Notes from the team's conversation with our mentor, and what we do with each.

| Note | What we do with it |
|---|---|
| The use case: make navigation fit for drones under 500 dollars | It is the target at the top of this page. We take the price as an orientation for the class of drone and not as a hard cap |
| Pick one thing, do it really well, say what could be done in the future and what it would require. One cohesive story in the slides | The one thing is camera fixes with the integrity check. The story is in "The story for the slides" |
| Find the position with the sun: a line sensor and a microcontroller, as in a Chinese paper. The sun is the cheapest way. Use an infrared lens | A next step, in the table above. The sun gives a good heading and a rough position |
| At night the moon and the stars are still there | A next step, in the table above |
| Over open water: NAVSOP. Radio stations, identified by their RDS signal, or TV stations for calculating the position | A next step, in the table above |
| Demo: simulate optical flow by walking outside the campus with a phone, recording without compression. Can we get the phone's altitude, and how precise would it be? | The phone walk in the demo section. The altitude that matters is the holding height, measured with a tape |
| "lightex technology" | Not understood. Ask him |

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
- [findings.md](findings.md): what we measured on Friday night on Mid-Air and ALTO, and what it changes
- [data.md](data.md): how Mid-Air and the DenseUAV model would be used, by Alessandro
- [method.md](method.md): how image matching and the particle filter work, with worked numbers
- [landscape.md](landscape.md): existing products and their limits
- [experiments.md](experiments.md): earlier measurements on Taiwan imagery and elevation, made before the dataset was chosen
- [challenge-2-research.md](challenge-2-research.md): papers, data sources, reading list
- [playbook.md](playbook.md): working rules, slide skeleton, submission checklist
