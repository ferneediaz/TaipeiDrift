# Challenge 4 in depth: drone detection, tracking and decision support

Research on what already exists for camera-based drone detection, where it falls short, and what a team of six could add in a weekend. Read [challenge-decision.md](challenge-decision.md) first for the comparison with the other challenges.

## Short version

- Camera-based drone detection is a real product category. Commercial systems detect, track and identify drones with ordinary cameras at more than a kilometre.
- Their known weak points are small targets, confusion with birds, night and weather, and false alarms when a camera is used alone.
- Taiwan has a current, public need: the Army's counter-drone contract for Kinmen, Matsu and Penghu was cancelled in July 2026 after the system failed acceptance tests.
- A weekend entry cannot beat the commercial detectors. It can show something they sell at a high price done cheaply: rejecting birds by how a target moves, and turning tracks into behaviour alerts an operator can accept or reject.
- With this, Challenge 4 becomes a third serious candidate beside 2 and 7. It has the strongest record at earlier events and the most competition.

## What the brief asks for

Detect drones in video, keep a persistent track, raise alerts from behaviour (speed, direction, dwell time, entry into a zone), recommend simulated responses, and let an operator accept, reject or modify them. The system must not control a weapon. A low-cost deployment concept and a plan for scaling across cameras and sites are scored.

## Existing products

| Product | What it does | Source |
|---|---|---|
| DroneShield DroneOptID 2.0 | AI software that detects and identifies drones and their payloads on camera, works at night with thermal sensors, estimates distance and altitude | [Unmanned Airspace](https://www.unmannedairspace.info/counter-uas-systems-and-policies/droneshield-releases-second-generation-droneoptid-2-0-drone-detection-software/) |
| OpenWorks SkyAI | Adds AI detection, tracking and motion control to a standard Axis camera; reported to work on small drones at over 1 km | [Unmanned Airspace](https://www.unmannedairspace.info/latest-news-and-information/openworks-adds-low-cost-camera-technology-to-its-skyai-drone-detection-as-part-of-the-guardion-c-uas-system/) |
| Walaris AirScout | Processes the whole image and classifies every object, to cut false alarms and automate detect, track, identify | [Walaris](https://walaris.co/) |
| Dedrone DedroneTracker.AI | Command software that fuses several sensor types and directs pan-tilt-zoom cameras | [Dedrone white paper](https://www.dedrone.com/white-papers/counter-uas) |
| ZVOOK NW0 (acoustic, for comparison) | Ukrainian acoustic sensor for FPV drones, declared range 150 to 450 m | [Militarnyi](https://militarnyi.com/en/news/ukraine-develops-acoustic-detector-for-fpv-drones/) |

Two patterns stand out. Every serious system combines sensors, because a camera alone is not trusted. And the camera's main job is often to confirm and identify a target that radar or radio sensing found first.

## Known limitations

- **Small targets.** A distant drone covers only a few pixels, and detection networks lose such targets in their deeper layers ([Improving Small Drone Detection](https://arxiv.org/pdf/2504.19347)).
- **Birds.** At that size a drone and a bird look alike in a single frame. There is a yearly research competition on exactly this, the Drone-vs-Bird challenge.
- **Night and weather.** Visible cameras fail in darkness and fog. Thermal cameras help but have lower resolution, shorter range and higher cost.
- **False alarms.** Optical sensing alone is described as having high false-alarm rates, here by a radar vendor, so read it with that in mind ([Robin Radar overview](https://www.robinradar.com/resources/10-counter-drone-technologies-to-detect-and-stop-drones-today)).
- **Coverage against range.** A wide lens sees a large area but few pixels per target; a zoom lens sees far but only a narrow patch and needs something to point it.
- **Compute.** Recent research on the Anti-UAV benchmark reports detection running at about 55 frames per second on a GPU but 13 on a CPU. Cheap edge hardware is a real constraint. For an overview of methods and benchmarks see this [survey](https://arxiv.org/pdf/2504.11967).

## Why cameras matter more now

Fibre-optic FPV drones are steered through a cable and emit no radio signal, so radio-based detection does not see them and jamming does not stop them. Detection has shifted towards optical, thermal and acoustic sensing ([Euromaidan Press](https://euromaidanpress.com/2025/02/03/russias-fiber-optic-drones-dodge-jamming-but-ukraine-hunts-them-with-infrared-and-sound/), [Journal of Electromagnetic Dominance](https://www.jedonline.com/2025/05/15/nato-seeks-solutions-for-fiber-optic-fpv-drones/)).

## The situation in Taiwan

After Chinese drones flew over Kinmen in 2022, Taiwan's Ministry of National Defense started a programme for counter-drone systems on the outlying islands. The Army contract went to Tron Future in January 2025 for 26 fixed sites. According to press reports, the requirement was to detect small, low-flying drones at six kilometres and jam several at four. The system failed acceptance testing and two retests, and the contract was cancelled in July 2026, with a new tender planned for 2027 ([The Defense Post](https://thedefensepost.com/2026/07/20/taiwan-drop-drone-deal/), [Domino Theory investigation](https://dominotheory.com/investigation-how-taiwan-botched-a-plan-to-counter-chinese-drones/)).

For a pitch this is a concrete customer with an open need. It also sets the bar: a camera prototype does not meet a six-kilometre requirement. The fair claim is a cheap confirmation and alerting layer that works beside radar.

## Where a weekend entry can add something

1. **Bird rejection by movement.** Classify a track by how it moves over a few seconds (speed changes, hovering, straightness, wing-beat flicker) instead of by a few blurry pixels. Recent research reports that trajectory features improve drone and bird separation ([Springer, 2026](https://link.springer.com/article/10.1007/s00521-026-12080-5)). It needs no high-resolution camera, so it supports the low-cost claim.
2. **Behaviour alerts.** Turn tracks into events: loitering, approach towards a protected zone, zone entry, sudden speed change. This is plain geometry on the track and can be explained to an operator.
3. **Operator decision loop.** Each alert carries a risk score, the reason, and a recommended simulated response. The operator accepts, rejects or edits, and the system records it.
4. **Two cameras together.** A visible and a thermal view of the same scene, or a wide and a zoom camera, each covering the other's weak point. A mentor on site said combining camera systems to identify drones is the direction that interests him most.

Ideas 1 to 3 form one coherent entry. Idea 4 is the stretch goal.

## Data

- **Drone video set, downloaded** (`data/raw/drone_video`, 190 MB, licence CC0): 285 visible and 365 infrared clips of 10 seconds, each showing a drone, bird, airplane or helicopter. The class is in the file name. Visible clips are 640 by 512 pixels, infrared 320 by 256, both at 30 frames per second. Source: [Drone-detection-dataset](https://github.com/DroneDetectionThesis/Drone-detection-dataset).
  - Limit found when testing: the bounding-box labels are stored as MATLAB objects that Python cannot read directly. The class per clip is usable at once; boxes would need a conversion in MATLAB or Octave, or a detector that finds the moving object against the sky.
  - The visible and infrared clips are separate recordings, not synchronised pairs.
- **[Anti-UAV](https://github.com/ucas-vg/Anti-UAV)** (licence MIT): visible and infrared drone videos with tracking labels. Large download; check size first.
- **[EDTH starter repository](https://github.com/pgryko/visual-drone-detector-hackathon)**: data tooling from earlier hackathons of this series. Ask the organisers whether a dataset is provided this time.

## Tools

- [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) for detection. Licence AGPL, which matters for any later commercial use.
- [ByteTrack](https://github.com/FoundationVision/ByteTrack) for persistent track IDs; a version is built into Ultralytics.
- OpenCV background subtraction as a detector that needs no training, which works well for objects moving against sky.

## Earlier entries

- Rome, first place: a counter-drone interceptor using one RGB camera, which cut detection compute by 89 percent by predicting where the target will be instead of searching the whole sky ([recap](https://eurodefense.tech/back-in-rome-recap-of-our-second-european-defense-tech-hackathon-in-italy/)).
- [drone_tracker](https://github.com/SamuelBleher/drone_tracker): camera detection steering a simulated drone (Munich, February 2026).
- [FPV drone detection and tracking](https://github.com/SowOuma/European-Defense-Tech-Hackathon-FPV-Drone-detection-and-tracking).
- [Radar multi-target tracking challenge from Bosch](https://github.com/JavaStudentAlex/drone-detection-mara): the same tracking ideas on radar data.
- [Kinematic simulation of several simultaneous drone threats](https://github.com/sorin373/European-Defense-Tech-Hackathon).

## Six roles

1. Detection: moving-object detector first, a trained model if time allows
2. Tracking: persistent IDs and stored paths
3. Track classification: drone, bird, airplane or helicopter from movement
4. Behaviour rules and risk score
5. Operator interface: video with tracks, zone drawing, alert cards, accept and reject
6. Deployment concept, cost, multi-camera scaling and pitch

## Demo

1. A video plays with boxes and track trails. A bird crosses and is labelled as a bird, with no alert.
2. A drone enters, loiters, then moves towards a zone drawn on screen. An alert appears with the reason and a risk score.
3. A recommended response is shown. The operator rejects one and accepts another.
4. One chart: false alarms with and without movement-based bird rejection.
5. One slide: camera and compute cost per site, and how sites connect.

## Risks

- This is the most common challenge type at these hackathons, so expect experienced teams.
- If detection is unreliable, everything after it looks broken. Start with the simple moving-object detector so the rest of the pipeline has input from the first evening.
- Training a detection model takes hours on a laptop. Treat it as an improvement, not as the base.
- The clips are short and show one object against sky. Judges may ask about cluttered backgrounds and several targets.

## How it compares with Challenges 2 and 7

| | Challenge 2 | Challenge 7 | Challenge 4 |
|---|---|---|---|
| Data today | To be simulated | Ready | Ready at clip level, boxes need work |
| Local relevance | High | Medium | Highest, given the cancelled contract |
| Record at earlier events | Placed | First and second place | First place, most entries |
| Competition expected | Low | Medium | High |
| Mentor match | Strong | Strong | Partial |
| Main risk | Many parts | Routine without the extras | Detector quality |
