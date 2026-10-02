# Plan

The one page that says what we build, who does what, and by when. Everything else in `docs/` is background.

Status: revised on Friday night after the team chose the Mid-Air dataset. Not yet confirmed by the team. Edit this page when something is decided.

## What we are building

Challenge 2, navigation without GNSS.

Software that keeps a drone's position estimate usable after GNSS is jammed, from the sensors the drone already carries. It also says how long the estimate can be trusted and flags when it cannot.

We are not building a drone. The assumed platform is an existing drone with:

- a camera pointing down
- an IMU (rotation and acceleration)
- a barometer for altitude

We do not fix a flight height. The method takes the height from the barometer as an input. We test it at low height, because that is what the dataset contains, and we state how the numbers change with height.

One-liner for the team form:

```
Keeps a drone on course after GNSS is jammed, using only its camera, IMU and barometer, and says when not to trust it.
```

## The dataset

[Mid-Air](https://midair.ulg.ac.be/), a synthetic dataset of low-altitude drone flights from the University of Liège.

- 54 flights with a downward camera at 25 frames per second, an IMU at 100 Hz with noise and drift, a simulated GNSS at 1 Hz, and exact ground truth at 100 Hz.
- Each flight is rendered in several conditions: sunny, cloudy, foggy and sunset in one landscape, and spring, fall and winter in another.
- No barometer. We simulate one from the true altitude plus noise.
- No aerial map of the landscapes.
- Licence: CC BY-NC-SA 4.0, non-commercial, with attribution.

First download, about 300 MB: the sensor records for one weather setting (43 MB) and the downward camera for one flight (trajectory 0003 in sunny is 260 MB). The links come from a form with a captcha on the [download page](https://midair.ulg.ac.be/download.html), so a team member has to request them.

## The scenario

GNSS works at the start of the flight and is then jammed. From that moment the software has to hold the position on its own. The error is measured against the ground truth.

## The three parts

### 1. Navigator

- Baseline: dead reckoning from the IMU alone after GNSS is lost.
- Correction 1, camera speed: image motion from the downward camera, times the height from the barometer, corrected for rotation with the IMU. This gives speed over ground.
- Correction 2, route memory: the same flight exists in another season or weather. One version is the stored route, the other is the flight. Recognising ground seen before gives a position fix.
- A filter that combines IMU, camera speed and fixes, and carries its own uncertainty.

Done when: for one flight, plots show the true path, the IMU baseline and the filter, with position error over time for each combination.

### 2. Integrity check

- Each camera measurement and each fix is tested before it is used.
- The system flags when the camera cannot be trusted, for example in fog or over ground without texture, and when the estimate has drifted too long without a fix.

Done when: the same flight is run in sunny and in foggy conditions, and the flag rises where the camera speed goes wrong.

### 3. Drift budget

- For each condition, a curve of expected position error against time since GNSS was lost.
- It answers the operator's question: how long can this drone fly without GNSS before the position is off by more than a set limit?

Done when: one chart shows the curves for at least three conditions, and one sentence states the time to reach 50 m of error for each.

## How the parts connect

```
Mid-Air flight (IMU, camera frames, GNSS until the jamming moment, ground truth)
        |
   data reader  ->  simulated barometer, jamming time
        |
   navigator  <-  camera speed, route-memory fix   (each with a confidence)
        |              ^
        |        integrity check accepts or rejects each one
        v
   estimate + uncertainty per time step  ->  error against ground truth  ->  plots, drift budget, demo view
```

Shared conventions, fixed now so six people can work in parallel:

- Coordinates: the dataset's own frame. North, East, Down in metres, origin at the start of each flight.
- Units: metres, seconds, radians, as in the dataset.
- A measurement handed to the filter is: value, uncertainty, source name, confidence between 0 and 1.
- Every result that goes on a slide is produced by a script in the repository.

## Roles

Write names here once agreed. Each person owns one part and can explain it alone.

| # | Role | Owns | Name |
|---|---|---|---|
| 1 | Data | Mid-Air reader, simulated barometer, jamming scenario, download | |
| 2 | Filter | IMU dead-reckoning baseline, the filter that combines everything | |
| 3 | Camera speed | Image motion, height and rotation turned into speed over ground | |
| 4 | Route memory | Recognising ground from another season or weather, position fix | |
| 5 | Integrity and evaluation | Checks, fog and season runs, all plots and numbers, drift budget | |
| 6 | Demo and pitch | Demo view, slides, target platform and deployment concept | |

## Timeline

Demo Day is Sunday 13:00. Code freeze is Sunday 10:00.

| When | What | Gate |
|---|---|---|
| Friday night | Confirm this plan, assign roles, request the download links, read the sensor records | A plot shows the IMU-only path drifting away from the true path |
| Saturday 09:00 to 13:00 | Camera speed working on one sunny flight, filter combining it with the IMU | The correction beats the baseline in a plot. The brief's minimum is met |
| Saturday 13:00 to 14:00 | Show mentors, write down what they say | |
| Saturday 14:00 to 19:00 | Fog and season runs, integrity check, drift budget, route memory | Same flight compared across at least two conditions |
| Saturday 19:00 to 22:00 | Demo view, fallback video, slide draft | Video file saved |
| Sunday 08:30 to 10:00 | Bug fixes only | |
| Sunday 10:00 | Code freeze | Demo branch tagged |
| Sunday 10:00 to 13:00 | Rehearse three times, submit | Submission confirmed |

If we fall behind, cut in this order: route memory, the season runs, the demo view. The IMU baseline, camera speed, one fog run and the drift budget are the smallest complete entry.

## Demo

1. The downward camera video plays next to a map of the flight. GNSS is switched off.
2. The IMU-only estimate leaves the true path within seconds.
3. With camera speed, the estimate stays close. One chart shows both errors over time, with one headline number.
4. The same flight in fog. The camera speed goes wrong, and the system flags it.
5. The drift budget: how long each condition allows before the error passes 50 m.
6. One slide: the sensors assumed, how the result changes with flight height, and what is not covered.

## What we claim and what we do not

Claim: after GNSS is lost, the method holds the position far longer than the IMU alone, using sensors the drone already has. It tells the operator how long, and it flags when the camera cannot be trusted.

Do not claim:

- Results on real flights. The data is synthetic.
- A position without a known start. The method continues from the last GNSS position.
- Performance at a specific flight height other than the low flight tested.
- That drift is removed. Camera speed slows it. Only a position fix resets it.

## Known weak points to address in the limits slide

- The barometer gives height above the start point. Over hills the true distance to the ground differs, and the speed estimate is off by the same proportion.
- Higher flight makes the camera see more ground and makes attitude errors count more. At 3,000 m a tilt error of 1 degree is about 52 m on the ground.
- Fog, darkness and ground without texture stop the camera from measuring speed.

## Open questions

- Does the team confirm this plan?
- Who requests the download links?
- Which second input did the mentor name for the cold start, besides the position of the sun?
- How long is the demo slot, and what is the submission format?

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
- [landscape.md](landscape.md): existing products and their limits
- [experiments.md](experiments.md): earlier measurements on Taiwan imagery and elevation, made before the dataset was chosen
- [challenge-2-research.md](challenge-2-research.md): papers, data sources, reading list
- [playbook.md](playbook.md): working rules, slide skeleton, submission checklist
