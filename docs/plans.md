# Demo scripts and weekend plans

For each candidate challenge: the target user, the two-minute demo, and a plan for the weekend. What we would build, the data, tools, roles and risks are in [challenge-decision.md](challenge-decision.md). The general timeline and working rules are in [playbook.md](playbook.md).

Demo Day starts Sunday at 13:00. All plans assume a code freeze on Sunday at 10:00.

## Challenge 2: navigation without GPS

### Target user

Operators of small drones that must keep flying a route while GPS is jammed or spoofed.

### Demo

1. A map shows the true path and the dead-reckoning estimate drifting away from it.
2. Switch on camera speed (optical flow with altitude) and the sun compass. The drift slows visibly.
3. Switch on map matching. The estimate snaps back to the true path at each fix and its uncertainty shrinks.
4. An error-over-time plot shows all three runs. One headline number: position error before and after.
5. Turn up the noise or switch a sensor off live, and show where the method stops working.

### Weekend plan

- Friday evening: simulated flight over an aerial image produces frames, altitude and a true path. Speed from optical flow is compared with the true speed. This is the 22:00 gate.
- Saturday morning: sun-compass heading and the filter that combines the sources, each tested against the true path.
- Saturday afternoon: map matching for position fixes, noise and failure experiments, a second image or real footage for validation.
- Saturday evening: animated map, fallback video, slide draft.
- Sunday until 10:00: bug fixes, slides, rehearsal.

## Challenge 7: acoustic drone detection

### Target user

Site security teams and units that need a cheap, passive warning layer around a perimeter, in addition to radar or radio sensing.

### Demo

1. A live scrolling spectrogram from the microphone, with a confidence meter.
2. Play drone audio from a phone. The meter rises, the event is marked on a timeline, and the rotor lines are highlighted as evidence.
3. Play a motorbike and wind. The detector stays quiet, or names the likely cause of the false alarm.
4. Show the quiet-contact curve: detection rate against how loud the drone is compared with the background.
5. Show model size and processing time on a plain CPU, with the network off.
6. Keep a recorded version ready, because the hall will be loud.

### Weekend plan

- Friday evening: data pipeline, first model, test numbers on the outdoor set that was kept out of training.
- Saturday morning: noise mixing and quiet-contact experiments, live microphone loop.
- Saturday afternoon: dashboard with evidence view, direction finding if a stereo microphone is available, speed and size measurement.
- Saturday evening: fallback video, sensor cost and deployment concept, slide draft.
- Sunday until 10:00: bug fixes, slides, rehearsal.

## Challenge 3: task allocation for one operator

### Target user

A single operator in a ground station who supervises a mixed fleet on search, patrol, transport or relay missions.

### Demo

1. A map with six to eight vehicles moving towards their assigned tasks.
2. Click "vehicle fails". A proposed plan appears within a second, showing what changes and why.
3. The operator edits one assignment and approves.
4. Add a high-priority task and cut one vehicle's communication link. The system replans again.
5. Raise the fleet to thirty vehicles and show that the operator view stays readable.

### Weekend plan

- Friday evening: scenario file, simulator loop, first allocation shown on a map.
- Saturday morning: the three events and the plan difference with reasons.
- Saturday afternoon: approve, edit and reject flow, metrics for mission completion, replan time and energy use.
- Saturday evening: larger-fleet view, fallback video, slide draft.
- Sunday until 10:00: bug fixes, slides, rehearsal.

## Challenge 6: maritime anomaly alerting

### Target user

Coast guard watch officers and cable operators who monitor dense vessel traffic and cannot inspect every track by hand.

### Demo

1. A map of the waters around Taiwan with vessel tracks replaying.
2. A vessel slows and loiters over a cable protection zone. An alert appears with a risk score, a timeline and the reason.
3. A second vessel stops reporting. The gap is flagged with a statement of uncertainty.
4. The operator marks one alert as a false alarm and moves a threshold. The alert list updates.
5. One chart compares the rule-based method with the learned method on precision and recall.

### Weekend plan

- Friday evening: track simulator with labelled events, tracks shown on a map.
- Saturday morning: rules for each event type, and the learned method.
- Saturday afternoon: comparison of both methods, operator feedback, handling of missing data.
- Saturday evening: fallback video, service and data concept, slide draft.
- Sunday until 10:00: bug fixes, slides, rehearsal.
