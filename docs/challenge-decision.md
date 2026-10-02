# Which challenge should we take?

A briefing for the team to decide on one challenge. It covers all nine, with what we would build, what data and code already exist, what placed at earlier hackathons of this series, and the risks.

Reading time: about 15 minutes. If you have 2 minutes, read "Short version" and the comparison table.

## Short version

Four challenges are realistic first choices for a team of six. Two stand out:

- **Challenge 2, navigation without GPS.** The most ambitious option. A mentor who works on GNSS security gave us a concrete design, and we can generate test data ourselves with exact ground truth.
- **Challenge 7, acoustic drone detection.** The most reliable option. The data is already downloaded, the work splits cleanly across six people, and this challenge type took first and second place at an earlier hackathon.

Proposal: start with Challenge 2 and set a gate for Friday 22:00. If the first end-to-end test does not work by then, switch to Challenge 7 on Saturday morning. The switch costs one evening because the data and environment for 7 are ready.

Challenges 3 and 6 are good alternatives if the team prefers a product-style build (3) or a data-analysis build (6).

## How the challenges were assessed

- **Interest:** is there a real idea to discover, or is it routine work?
- **Data:** can we start today, or do we first need to find or create data?
- **Checkable:** is there a known right answer to test against?
- **Split:** can six people work in parallel without blocking each other?
- **Mentor:** is there a mentor on site whose field matches?
- **Record:** what placed at earlier hackathons of this series?
- **Demo:** can the result be shown in two minutes as one visible change?

Every brief also scores the user, the deployment concept and scalability. That part is needed for all nine, so it does not separate them.

All coding is done with AI assistance, so missing experience in a method (a Kalman filter, a detection model) is less of a barrier than usual. What AI does not replace: data, training time, checking that results are correct, hardware, and each person being able to explain their own part to the judges.

## Comparison

| # | Challenge | Interest | Data | Checkable | Split | Mentor | Record | Demo |
|---|---|---|---|---|---|---|---|---|
| 1 | Offline collaboration | Low | None needed | Yes | Good | None specific | Entries, no wins found | Good |
| 2 | Navigation without GPS | High | Aerial images ready, flight simulated | Yes, exactly | Good | Strong | Placed twice | Strong |
| 3 | Task allocation | Medium | None needed | Partly | Tight | Strong | Placed, many entries | Strongest |
| 4 | Drone detection and tracking | Medium | Download and train | Yes | Medium | Weak | Strongest, most contested | Strong |
| 5 | Visible and thermal fusion | Medium | Download | Hard | Poor | Partial | None found | Medium |
| 6 | Maritime anomaly alerting | Medium | Simulate | Yes | Good | Good | Thin | Strong |
| 7 | Acoustic drone detection | Medium to high | Ready on disk | Yes | Best | Strong | First and second place | Good |
| 8 | Edge model optimisation | Low | Uses another model | Yes | Small task | Good | Part of a second place | Weak |
| 9 | Drone teardown | Depends | Organiser drones | Partly | Poor | Strong | None found | Medium |

The full briefs are in [challenges.md](challenges.md).

## Challenge 2: navigation without GPS

A fuller study of this challenge, with fielded systems, their limits, the situation in Taiwan and the data we have, is in [challenge-2-research.md](challenge-2-research.md).

### What it asks

Estimate a vehicle's position when satellite positioning is jammed. Build a simple baseline, add at least one correction, and show the error before and after.

### What we would build

- **Baseline, dead reckoning:** add up speed and heading over time. It works for a short while, then the error grows without limit.
- **Speed from the camera (optical flow):** a camera pointing down sees the ground slide across the image. How fast it slides, multiplied by the height above ground, gives the speed over ground.
- **Heading from the sun:** with the time known, the sun's direction gives a heading that cannot be jammed. This also solves the cold start, which is knowing which way we face before any movement.
- **Position fixes from a map:** match the camera view against an aerial image stored on board to get an absolute position from time to time.
- **A particle filter to combine them:** it keeps a thousand guesses of position and heading, moves each with the measured speed, and keeps the guesses whose camera view fits the map. The spread of the guesses is the uncertainty. This is the method used in the published work on this problem.

The optical-flow and sun-compass ideas came from a mentor who works on GNSS security.

### Data

- **Simulated flight:** take a large aerial image, fly a virtual drone over it on a known path, and cut out what a downward camera would see. This gives camera frames, altitude and an exact true path, so every result can be checked. Two free aerial images of the same site in Taichung, from 2018 and 2020, are available for this: one as the world the drone sees, the other as the on-board map.
- **Organiser dataset:** the brief mentions a suggested dataset with IMU, speed, heading and reference position. We have not seen it yet.
- **Real footage for validation:** [UAV-VisLoc](https://github.com/IntelliSensing/UAV-VisLoc) is a public dataset for drone visual localisation. Check size and licence before using it (the repository states no licence).

### Tools

- [filterpy](https://github.com/rlabbe/filterpy): Kalman filter library, already installed
- [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python): a free book that explains the filter with code
- OpenCV for optical flow and image matching
- [PX4-Flow](https://github.com/PX4/PX4-Flow): firmware of a real optical-flow sensor, useful as proof that the method is used on real drones

### Six roles

1. Flight simulator: frames, altitude and true path from an aerial image
2. Optical flow: speed from the frames
3. Heading and cold start: sun compass
4. Filter: combine the sources, report uncertainty
5. Map matching: absolute position fixes
6. Visualisation, evaluation and pitch

### Record at earlier events

- Rome, third place: a visual positioning kit that matches camera images against satellite maps ([recap](https://eurodefense.tech/back-in-rome-recap-of-our-second-european-defense-tech-hackathon-in-italy/)).
- An earlier winning team had a group of drones share observations to navigate without GPS, as reported by [DroneXL](https://dronexl.co/2024/12/03/europes-largest-defense-tech-hackathon-drone-detection/).

Both went beyond a simple filter. A dead-reckoning filter with landmark fixes would meet the brief but would be the expected baseline.

### Risks

- It has the most moving parts of the four candidates.
- Judges may doubt a result that exists only in simulation. Answer: use a different image of the same area as the on-board map, or validate on real footage on Saturday.
- A filter can look right and be wrong. Answer: test every part against the exact true path first.

### Gate

Friday 22:00: the simulated flight produces frames, and the speed from optical flow roughly matches the true speed. If not, switch to Challenge 7.

## Challenge 7: acoustic drone detection

### What it asks

Listen to environmental audio and estimate whether a multirotor drone is present. Show confidence, the evidence behind the decision, and likely causes of false alarms. Propose a low-cost sensor concept.

### What we would build

- **Detector:** turn each second of audio into a spectrogram (a picture of which frequencies are loud over time) and classify it with a small neural network. Drone rotors produce evenly spaced lines in that picture.
- **Quiet contacts:** mix drone sound into background noise at lower and lower volume and find where detection breaks. This turns one accuracy number into an estimate of detection range.
- **Evidence view:** a live spectrogram with the rotor lines highlighted, a confidence meter, and the likely cause when the alert is a false alarm.
- **Direction finding (stretch):** two microphones a known distance apart hear the drone at slightly different times, which gives the direction. This needs a stereo microphone with a shared clock.
- **Speed and size:** one measurement of model size and processing time on a plain CPU, to support the low-cost claim.

### Data, already downloaded

See [data/README.md](../data/README.md) for details. Fetch with `bash scripts/fetch_data.sh`.

- [DroneAudioDataset](https://github.com/saraalemadi/DroneAudioDataset): 1,332 drone clips and 10,372 background clips. Recorded indoors, so results on it alone are too optimistic.
- [ESC-50](https://github.com/karolpiczak/ESC-50): 2,000 clips of environmental sounds, including engines, helicopters, wind and rain. Non-commercial licence.
- [Drone-detection-dataset](https://github.com/DroneDetectionThesis/Drone-detection-dataset): 90 outdoor clips of drones, helicopters and background, licence CC0. Kept as the final test because it comes from a different microphone.

### Tools

- librosa and PyTorch, already installed
- [PANNs](https://github.com/qiuqiangkong/audioset_tagging_cnn): pretrained audio networks that can serve as a starting point
- [ONNX Runtime](https://github.com/microsoft/onnxruntime) for the speed and size measurement

### Six roles

1. Model and training
2. Data: noise mixing, quiet-contact experiments, evaluation
3. Live audio pipeline from the microphone
4. Dashboard: spectrogram, timeline, evidence view
5. Direction finding, or sensor hardware
6. Sensor concept, cost, deployment and pitch

These parts connect through simple interfaces (audio in, probability out), so nobody waits for anybody.

### Record at earlier events

- Copenhagen 2025: first and second place both went to teams on the acoustic challenge set by Helsing. First place built an 8-microphone array, second place an edge AI model for acoustic sensors ([recap](https://eurodefense.tech/copenhagen-defense-tech-hackathon-2025/)).
- London, May 2025: Helsing's acoustic challenge is public: [ldth-2025-acoustics](https://github.com/Phissie/ldth-2025-acoustics). Its second phase asked for explainability, a lightweight model and synthetic data for quiet contacts, which is what we propose above.
- A related entry from another hackathon: [drones-shazam](https://github.com/Ajanzz/drones-shazam-ODT-hackathon), "Shazam for drones".

### Risks

- The plain version (drone or no drone) is routine and will not stand out. The quiet-contact, evidence and direction parts are what make it an entry.
- Three scoring criteria concern hardware cost and manufacturability. Without a physical sensor we answer them on paper. Ask the organisers whether microphones or boards can be borrowed.
- A loud hall is a hard place for a live microphone demo. Record a fallback video.

## Challenge 3: task allocation for one operator

### What it asks

A simulator that assigns missions to a mixed fleet of vehicles and proposes a new plan when a vehicle fails, loses its link, or a new priority task arrives. The human approves or overrides.

### What we would build

- A 2D map with vehicles that differ in capability and energy
- An allocator that scores every vehicle and task pair and solves the assignment ([OR-Tools](https://github.com/google/or-tools) or `scipy.optimize.linear_sum_assignment`)
- Replanning shown as a difference to the current plan, with the reasons
- An approve, edit or reject flow for the operator
- A view that stays readable with thirty vehicles

### Record at earlier events

- Rome, second place: multi-drone coordination software.
- Public entries: [Argus](https://github.com/kotr98/Argus) (swarm orchestration simulator), [ARGUS, Sheffield](https://github.com/knmlprz/ARGUS), and a [C++ simulation backend](https://github.com/sorin373/European-Defense-Tech-Hackathon) for several simultaneous drone threats.

### Mentor

One mentor on site works on heterogeneous drone task allocation, dynamic replanning and human-in-the-loop workflow. His description matches this brief closely.

### Risks

- The idea is common, and the algorithm is standard. The entry is decided by the operator experience.
- Simulator, allocator and interface form one connected application. Six people must coordinate closely.

## Challenge 6: maritime anomaly alerting

A fuller study of this challenge, with existing products, their limits, the cable incidents around Taiwan and data options, is in [challenge-6-research.md](challenge-6-research.md).

### What it asks

A dashboard that analyses vessel tracks (AIS), raises alerts with reasons and uncertainty, lets an operator give feedback, and compares a rule-based method with a statistical or machine-learning method.

### What we would build

- Simulated vessel tracks around Taiwan with labelled events (loitering over a cable zone, reporting gaps, route deviation)
- Rules for each event type, and one learned method such as an isolation forest
- A precision and recall comparison of both, with false-alarm control
- A map dashboard with timeline, risk score and operator feedback

### Data

- Simulation gives labelled events, which the brief allows.
- Real AIS for other regions: [MarineCadastre](https://hub.marinecadastre.gov/) (US waters), [Global Fishing Watch](https://globalfishingwatch.org/). Free AIS for the Taiwan Strait is hard to get.

### Record at earlier events

- One public entry with the same shape: [EDTH-ctrl_sea](https://github.com/bb1/EDTH-ctrl_sea) (AIS and radar input, a risk engine, a map).
- Copenhagen, third place, was maritime but hardware: a remotely controlled vessel.

### Why consider it

High local relevance (vessels loitering over undersea cables), a mentor with maritime expertise, clean separation of work, and a result that can be measured.

## The other five, briefly

### Challenge 1: offline collaboration

- **What:** a local-first tool that syncs messages and records between nearby devices and merges conflicting edits after reconnection.
- **Tools:** [Automerge](https://github.com/automerge/automerge) or [Yjs](https://github.com/yjs/yjs), libraries that merge concurrent edits automatically.
- **Earlier entries:** [OfflineLingo](https://github.com/cedric190703/EDTH-Hackathon-Berlin2026-OfflineLingo), and [kolbe](https://github.com/HAMHACK26/kolbe), a peer-to-peer drone mesh network.
- **Verdict:** very buildable and easy to demo, but little to discover and hard to make stand out.

### Challenge 4: drone detection and tracking

A fuller study of this challenge, with existing products, their limits and the situation in Taiwan, is in [challenge-4-research.md](challenge-4-research.md). It makes Challenge 4 a third serious candidate.

- **What:** detect drones in video, keep a track, raise behaviour alerts, recommend simulated responses for an operator to accept or reject.
- **Tools:** [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) for detection (AGPL licence), [ByteTrack](https://github.com/FoundationVision/ByteTrack) for tracking.
- **Data:** [Anti-UAV](https://github.com/ucas-vg/Anti-UAV), the video part of the [Drone-detection-dataset](https://github.com/DroneDetectionThesis/Drone-detection-dataset), and the [EDTH starter repository](https://github.com/pgryko/visual-drone-detector-hackathon).
- **Earlier entries:** Rome first place was a camera-based counter-drone system. Public repositories: [drone_tracker](https://github.com/SamuelBleher/drone_tracker), [FPV detection and tracking](https://github.com/SowOuma/European-Defense-Tech-Hackathon-FPV-Drone-detection-and-tracking), a [radar tracking challenge from Bosch](https://github.com/JavaStudentAlex/drone-detection-mara).
- **Verdict:** wins often, and is where experienced teams go. Training a detector takes hours of laptop time. Worth it only if someone on the team has trained detection models before.

### Challenge 5: visible and thermal fusion

- **What:** align a normal camera image with a thermal image and blend them into one useful picture, fast.
- **Data:** [LLVIP](https://github.com/bupt-ai-cz/LLVIP) (paired, already aligned, no licence stated), [Anti-UAV](https://github.com/ucas-vg/Anti-UAV) (visible and infrared drone videos).
- **Earlier entries:** none found.
- **Mentor remark:** one mentor said the most interesting direction for him is combining camera systems to identify drones. That would be Challenge 5 used for the purpose of Challenge 4.
- **Verdict:** "the fused image is better" is hard to prove. The convincing proof is a detector that finds more drones on the fused image, which means doing Challenge 4 as well. Teams with a thermal camera have a live demo we cannot match.

### Challenge 8: edge model optimisation

- **What:** shrink a model so it runs offline on cheap hardware, and measure before and after.
- **Verdict:** as its own entry it is a benchmark table with a weak demo. As a single measurement inside Challenge 7 it costs an hour and answers two scoring criteria.

### Challenge 9: drone teardown

- **What:** inspect an organiser-supplied drone under supervision, document its parts, and propose a passive assessment workflow.
- **Verdict:** depends on hardware knowledge and on access to a limited number of devices. Consider it only if someone on the team builds or repairs drones.

## Mentors on site

From the participant page. Remove this section before the repository is made public.

| Mentor | Field | Most useful for |
|---|---|---|
| Milosch Meriac, CTO at [Bitqan Systems Design](https://bitqan.ae/about.html) | GNSS security, RF localisation, signal processing, acoustics, embedded hardware | Challenges 2 and 7, sensor hardware and cost |
| MC (@minmax) | Sensor data and systems integration, maritime | Challenges 2 and 6 |
| Wenteng Chang | Multi-UAV task allocation, replanning, human-in-the-loop, demo and pitch | Challenge 3, pitch structure |
| Oleksandr Kulyniak | Current warfare challenges | Reality check for any idea |
| Caine Cortellino | Test scenarios based on battlefield conditions, government contracting | Failure scenarios, deployment slide |
| Yen Chang | RC planes | What a small aircraft can carry |
| Paruyr Abrahamyan | Defence industry, sales pitch | Pitch rehearsal |

## What earlier events show

- Entries that placed solved one sharply defined problem and showed a measured result.
- Hardware helps but is not required: Copenhagen's second place was a software model.
- Detection and tracking is the most common challenge type and the most contested.
- No event published its judging criteria, so this shows what placed, not why.

Sources: [Copenhagen recap](https://eurodefense.tech/copenhagen-defense-tech-hackathon-2025/), [Rome recap](https://eurodefense.tech/back-in-rome-recap-of-our-second-european-defense-tech-hackathon-in-italy/), [DroneXL report](https://dronexl.co/2024/12/03/europes-largest-defense-tech-hackathon-drone-detection/), [official site](https://tdth.org/), and the repositories linked above. Other entries worth a look for style: [Kolu](https://github.com/kolu-gears/ff1) (friend-or-foe identification for drones, London 2025), [mission-aware video compression](https://github.com/jaanisfehling/defense-tech-hackathon), and a [solution to Helsing's electronic warfare challenge](https://github.com/adaykoth/electronic_warefare_hackathon) (Munich 2025).

## How to decide

1. Everyone reads the short version and the comparison table.
2. Each person says what they have built before and which part they would want to own.
3. Check each candidate against the roles listed for it. A challenge is viable if every role has a willing owner.
4. Vote between Challenge 2 with the gate, and Challenge 7. Consider 3 or 6 only if someone argues for them.
5. Write the decision and the role owners into the README, then start with the idea filter in [playbook.md](playbook.md).

## Open questions for the organisers

- Is the suggested navigation dataset for Challenge 2 available, and what does it contain?
- Can microphones, a microphone array or development boards be borrowed?
- Are the public datasets acceptable, given that two have a non-commercial or missing licence?
- What are the judging weights, the demo length and the submission deadline?
- May a team enter two challenges, and does that help or hurt?
