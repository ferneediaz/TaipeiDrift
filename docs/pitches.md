# Candidate challenges

Three candidates for the team to choose from at kickoff. Each section has the idea, the demo, the technical approach, the roles, a weekend plan and the main risks. Full briefs for all nine are in [challenges.md](challenges.md).

The hackathon runs Friday 12:00 to Sunday 17:00. The plans below assume a demo freeze on Sunday at 12:00.

## Option A: Challenge 2, navigation without satellite positioning

### Idea

A vehicle that loses GPS can still estimate its position by integrating its own speed and heading, but the error grows with every second. We build a filter that pulls the estimate back whenever another clue arrives, such as a known landmark, and we measure how much error that removes.

### Target user

Operators of small drones or boats that must keep flying or sailing a route while GPS is jammed or spoofed.

### Demo (2 minutes)

1. A map shows the true path and the dead-reckoning estimate drifting away from it.
2. Switch on the correction. The estimate snaps back at each landmark fix and the uncertainty ellipse shrinks.
3. An error-over-time plot shows both runs. One headline number: position error before and after.
4. Turn up the sensor noise or drop a sensor live and show where the method stops working.

### Approach

- Baseline: dead reckoning from speed and heading.
- Correction: an extended Kalman filter. It keeps a position estimate and a measure of its own uncertainty, and weights each new measurement by how reliable it is compared with that uncertainty.
- Correction source: range or bearing to landmarks at known positions is the simplest. A visual or terrain match is a stretch goal.
- Evaluation: root-mean-square position error, final drift, and a sweep over noise levels and landmark spacing.
- Tools: Python, numpy, filterpy, matplotlib. All installed.

### Roles

- Estimation lead: the filter and its tuning. Needs a robotics, controls or physics background.
- Data and evaluation: dataset loading, coordinate conversion, metrics, noise sweeps.
- Visualisation: animated map and plots.
- Product and pitch: target platform, hardware and compute cost, deployment story.

### Weekend plan

- Friday: load the dataset, plot the reference path, get dead reckoning running and its error plotted.
- Saturday morning: filter with one correction source, verified on simulated data where the true answer is known.
- Saturday afternoon: run on the real dataset, noise and failure experiments.
- Sunday morning: animation, slides, rehearsal.

### Risks

- A filter can produce a plausible-looking track while being wrong. Test it first on simulated data with known truth.
- Coordinate frames and units cause most bugs. Fix one convention on Friday and write it down.
- Without a teammate who has built a filter before, this option is slow. Decide on it only after we know who is on the team.

## Option B: Challenge 7, acoustic drone detection, with Challenge 8 as an add-on

### Idea

A drone that emits no radio signal still makes a characteristic sound. We build a detector that listens through a cheap microphone, reports whether a multirotor is present and how sure it is, and runs offline on low-cost hardware.

### Target user

Site security teams and units that need a passive, cheap warning layer around a perimeter, in addition to radar or radio sensing.

### Demo (2 minutes)

1. A live scrolling spectrogram from the laptop microphone, with a confidence meter.
2. Play drone audio from a phone. The meter rises, the event is marked on a timeline, and the harmonic lines from the rotors are highlighted as evidence.
3. Play a motorbike and wind. The detector stays quiet, or names the likely false-alarm source.
4. Show the model size and latency after optimisation, running with the network off.
5. Keep a prerecorded version ready, because the hall will be loud.

### Approach

- Features: mel spectrograms of 1 to 2 second windows, resampled to 16 kHz.
- Model: a small convolutional network, or a pretrained audio model as feature extractor with a simple classifier on top.
- Data: drone audio set, ESC-50 and an outdoor test set, all already in `data/raw/` (see [data/README.md](../data/README.md)). Mix drone clips into background noise at controlled levels.
- Evaluation: precision and recall, a curve of accuracy against signal-to-noise ratio, and a breakdown of false alarms by sound class. Use the outdoor set, which comes from a different microphone and setting, only for the final test.
- Challenge 8 add-on: export to ONNX, quantize to 8-bit, report size, latency, memory and accuracy before and after in a CPU- and memory-limited container.
- Stretch: direction of arrival. Two microphones a known distance apart hear the drone at slightly different times, and that delay gives the bearing. This needs a stereo input where both channels share one clock, so two separate phones will not work.
- Tools: Python, librosa, PyTorch, onnxruntime, sounddevice. All installed.

### Roles

- Model lead: features, training, evaluation.
- Signal processing or embedded: live audio pipeline, direction finding, a real sensor board if someone brings one.
- Optimisation: ONNX export, quantization, benchmarks.
- Dashboard: live spectrogram, timeline, evidence view.
- Product and pitch: sensor bill of materials, installation and maintenance concept, market.

### Weekend plan

- Friday: data pipeline, first model, honest test numbers.
- Saturday morning: noise-robustness experiments, live microphone loop.
- Saturday afternoon: dashboard, optimisation and benchmarks.
- Sunday morning: prerecorded fallback, slides, rehearsal.

### Risks

- The public drone clips are indoor, close-range recordings. Numbers on that set overstate real performance, so say that plainly and test on other audio.
- Three scoring criteria concern hardware cost and manufacturability. A team with a physical sensor node will be more convincing than a cost table.
- Ask whether one team may submit to both 7 and 8. If not, the optimisation work stays as a section of the Challenge 7 entry.

## Option C: Challenge 3, one operator, many vehicles

### Idea

One person cannot steer ten vehicles at once. We build a simulator that assigns tasks to a mixed fleet, notices when something changes, and proposes a new plan with reasons. The operator approves, edits or rejects it.

### Target user

A single operator in a ground station who supervises a mixed fleet on search, patrol, transport or relay missions.

### Demo (2 minutes)

1. A map with six to eight vehicles moving towards their assigned tasks.
2. Click "vehicle fails". A proposed plan appears within a second, showing what changes and why.
3. The operator edits one assignment and approves.
4. Inject a high-priority task and cut one vehicle's communication link. Replan again.
5. Raise the fleet to thirty vehicles and show that the operator view stays readable.

### Approach

- Simulation: a 2D map, straight-line movement, a tick-based clock, energy that drains with distance.
- Allocation: a cost for each vehicle and task pair built from capability match, distance, remaining energy and link status, solved with the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`) or a greedy auction.
- Replanning: triggered by failure, link loss or a new task. The proposal is a diff against the current plan, with the cost terms as the explanation.
- Human control: nothing changes until the operator approves. Overrides are kept as constraints in later replans.
- The allocator stays deterministic so the demo is repeatable and needs no network.
- Tools: FastAPI for the simulator and allocator, a web front end with a map or canvas.

### Roles

- Allocation and simulation: scenario format, cost model, solver, replan triggers.
- Front end: map, plan diff, approval flow, timeline.
- Scenario and evaluation: test scenarios, metrics for mission completion, replan time and energy use.
- Product and pitch: operator workflow, integration with an existing ground station, scaling.

### Weekend plan

- Friday: scenario JSON, simulator loop, first allocation shown on a map.
- Saturday morning: the three events and the replan diff.
- Saturday afternoon: approval and override flow, metrics.
- Sunday morning: larger-fleet view, slides, rehearsal.

### Risks

- Scope. Simulator, solver, three event types and an animated interface is a lot. Keep the simulation crude and spend the time on the replan and approve loop.
- The algorithm is standard, so the entry stands or falls on the operator experience and the explanations.

## What earlier EDTH hackathons show

From public recaps and repositories of earlier events. Judging criteria were not published for any of them.

- Acoustic detection has placed well with software alone. At Copenhagen 2025, first and second place both went to teams on Helsing's acoustic challenge: an 8-microphone sensor array, and an edge AI model for acoustic sensors.
- Helsing's acoustic challenge (London, May 2025) had two phases: classify background, drone and helicopter, then make the model fit for the field through explainability, a lightweight or fast model, and synthetic data for quiet contacts. That second phase is close to Challenge 7 and 8 combined. The challenge repository is public: https://github.com/Phissie/ldth-2025-acoustics
- Navigation entries that placed went beyond a basic filter. Rome's third place matched live camera images against satellite maps; another winning team had drones share observations to localise as a group. A dead-reckoning filter with landmark fixes meets this brief, but it is the expected baseline, so the correction source has to be the interesting part.
- Task allocation and swarm simulators are common entries (several public repositories, second place in Rome), so Option C needs a clear angle on the operator side.

Sources: https://eurodefense.tech/copenhagen-defense-tech-hackathon-2025/ and https://eurodefense.tech/back-in-rome-recap-of-our-second-european-defense-tech-hackathon-in-italy/

## How to choose

- A teammate has built a Kalman filter or studied controls: Option A.
- A teammate has signal-processing or embedded experience, or brings a microphone board: Option B.
- The team is mostly software and front-end people: Option C.
- A strong computer-vision engineer joins: consider Challenge 4, whose operator layer overlaps with Option C.
