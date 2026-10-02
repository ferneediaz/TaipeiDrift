# Plan

The one page that says what we build, who does what, and by when. Everything else in `docs/` is background.

Status: proposed on Friday evening. Not yet confirmed by the team. Edit this page when something is decided.

## What we are building

Challenge 2, navigation without GNSS.

A navigation method for small drones that runs on free map data and cheap sensors. It tells the operator before launch where it will hold, and during flight when it should not be trusted.

One-liner for the team form:

```
Keeps a drone on course when GNSS is jammed, on open maps and cheap sensors, and says when not to trust it.
```

## Why this

- The brief asks for a dead-reckoning baseline, at least one correction, plots of path and error, and the limits. This plan covers all of it.
- Camera matching against a map already exists as a product, and Taiwan is adopting one that needs its vendor's data ([landscape.md](landscape.md)). We do not compete on accuracy.
- Our own tests on real Taiwan data show that each sensor works on different ground, and that a filter can be confident and wrong ([experiments.md](experiments.md)). Those two facts are what the entry is built around.

## The three parts

### 1. Navigator

- Dead-reckoning baseline from airspeed and heading.
- Particle filter over position and wind.
- Terrain fixes: downward range sensor against the free elevation model.
- Camera fixes: camera view against the free aerial image.
- Sun compass for heading at the start.

Done when: on one simulated route from sea to mountains, plots show the true path, the baseline and the filter, with position error over time for each sensor combination.

### 2. Integrity check

- Each fix is tested against the filter's current belief before it is used.
- The system raises a flag when sensors disagree or when no fix has arrived for too long.

Done when: a deliberately wrong camera fix and a filter started in the wrong place are both caught and shown on the plot.

### 3. Navigability map

- A map of the test region coloured by expected position error: terrain fixes in hills, camera fixes on flat land, growing drift over water.
- For a route drawn on the map, the predicted error along the way.

Done when: the map exists for the strip at 24.05 N, and the prediction for the demo route roughly matches what the navigator achieved on it.

## How the parts connect

```
scenario (route, wind, noise)
        |
   simulator  ->  true path  +  sensor readings per time step
        |
   navigator  <-  terrain fix, camera fix, sun heading   (each with a confidence)
        |              ^
        |        integrity check accepts or rejects each fix
        v
   estimate + uncertainty per time step  ->  plots, demo view
        
   navigability map uses the same sensor error models, without flying
```

Shared conventions, fixed now so six people can work in parallel:

- Coordinates: metres in EPSG:3826 (TWD97), x east, y north. Both aerial images already use it. The elevation model is converted once.
- Units: metres, seconds, metres per second. Heading in degrees, 0 is north, clockwise.
- One time step is a row: time, airspeed, heading, altitude, range to ground, and optionally a camera patch.
- A fix is: x, y, uncertainty in metres, source name, confidence between 0 and 1.
- Every result that goes on a slide is produced by a script in the repository.

## Roles

Write names here once agreed. Each person owns one part and can explain it alone.

| # | Role | Owns | Name |
|---|---|---|---|
| 1 | Simulator | Routes, wind, sensor readings with noise, sun heading | |
| 2 | Filter | Dead-reckoning baseline, particle filter with wind | |
| 3 | Terrain sensor | Elevation data, terrain fix and its confidence | |
| 4 | Camera sensor | Image matching, score turned into confidence | |
| 5 | Integrity and evaluation | Fix checking, failure cases, all plots and numbers | |
| 6 | Map and demo | Navigability map, demo view, slides | |

The pitch, the sensor package and the cost estimate are shared. Role 6 collects them.

Starting material: the four scripts in `experiments/` contain working first versions of roles 2, 3 and 4.

## Timeline

Demo Day is Sunday 13:00. Code freeze is Sunday 10:00.

| When | What | Gate |
|---|---|---|
| Friday until 23:00 | Confirm this plan, assign roles, set up the code structure, baseline running | A plot shows dead reckoning drifting away from the true path |
| Saturday 09:00 to 13:00 | Filter with wind, terrain fix and camera fix each working alone on a short leg | One correction beats the baseline in a plot. The brief's minimum is met |
| Saturday 13:00 to 14:00 | Show mentors, write down what they say | |
| Saturday 14:00 to 19:00 | All sensors on the full route, integrity check, first navigability map | Full route runs end to end |
| Saturday 19:00 to 22:00 | Demo view, fallback video, slide draft | Video file saved |
| Sunday 08:30 to 10:00 | Bug fixes only | |
| Sunday 10:00 | Code freeze | Demo branch tagged |
| Sunday 10:00 to 13:00 | Rehearse three times, submit | Submission confirmed |

If we fall behind, cut in this order: sun compass, navigability map reduced to the demo route only, camera fixes. The navigator with terrain fixes and the integrity check is the smallest complete entry.

## Demo

1. The map shows the route from the Strait across the plain into the mountains. Dead reckoning drifts off.
2. The navigability map predicts where the drone will hold position and where it will drift.
3. The flight runs. Over water the uncertainty grows, on the plain camera fixes pull it back, in the hills the terrain takes over.
4. A wrong fix is injected. The system rejects it and says so.
5. One chart: position error over time, baseline against ours, with one headline number.
6. One slide: sensor package and cost, free map sources, and what is not covered.

## What we claim and what we do not

Claim: it works on free maps and cheap sensors, it knows where it will hold, and it flags when it should not be trusted.

Do not claim: better accuracy than existing products, a solved crossing of the whole Strait, or performance at night and in fog.

## Open questions

- Does the team confirm this plan?
- Does anyone have a 360-degree camera here? If so, real footage becomes an extra on Saturday afternoon.
- Which second input did the mentor name for the cold start, besides the position of the sun?
- Did the organisers hand out the suggested dataset?
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
- [experiments.md](experiments.md): our first measurements on real data
- [challenge-2-research.md](challenge-2-research.md): papers, data sources, reading list
- [playbook.md](playbook.md): working rules, slide skeleton, submission checklist
