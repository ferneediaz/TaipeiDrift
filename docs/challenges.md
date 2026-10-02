# The nine challenges

Condensed from the participant page of the Taiwan Defense Tech Hackathon 2026. The full text is on the members-only challenges page; check it before relying on a detail here.

Every challenge also asks for a non-technical part: who the user or customer is, what deployment needs, and how the solution scales. That part is scored in all nine.

## 01 Offline collaboration for disconnected environments

- Build: a local-first tool where nearby devices exchange messages, tasks, locations or incident records over a local network and sync safely after reconnection.
- Must show: offline record creation, delivery and sync status, simulated disconnect and reconnect, conflicting edits to one record, sync that keeps the original history, a permissions, activity-log or node-status view.
- Resources: one Wi-Fi network or a simulated network is enough.
- Scored on: offline usability, sync accuracy, conflict handling, deployment feasibility, scalability, customer demand.

## 02 Navigation and drift correction in GNSS-denied environments

- Build: a navigation method that uses IMU, speed, heading, ranging, landmark or visual information to limit drift when satellite positioning is unavailable.
- Must show: a dead-reckoning baseline, at least one correction source or sensor-fusion method, plots of estimated against reference trajectory and of error over time, the limits when sensors fail or noise rises.
- Resources: a suggested dataset with IMU, speed, heading and reference position; a smartphone IMU recording is allowed.
- Scored on: reduction in positioning error, technical validity, noise tolerance, computing and integration requirements, deployment feasibility.

## 03 Multi-vehicle task allocation for a single human operator

- Build: a simulator that assigns missions to UAVs, UGVs or USVs by capability, location, remaining energy and communication status, and proposes a revised plan when conditions change. The human approves or overrides.
- Must show: vehicles with different capabilities, several task types with completion conditions, simulated vehicle failure, communication loss and a new high-priority task, a revised allocation the operator can approve or modify, progress and decision reasons on a map, timeline or animation.
- Resources: no physical vehicle; a common JSON scenario if available, otherwise a documented one.
- Scored on: mission completion, replanning speed, resource efficiency, human-control design, explainability, scalability to larger fleets.

## 04 Low-cost drone detection, tracking and response decision support

- Build: detection and tracking of drones in video, behaviour-based risk alerts, and simulated allocation of defensive resources. It must not control a weapon or physical interceptor.
- Must show: detection in prerecorded or live video, persistent track IDs and paths, alerts from speed, direction, dwell time or zone entry, recommended simulated actions, operator accept, reject or modify.
- Resources: licensed video or synthetic tracks; prerecorded video is acceptable.
- Scored on: detection and tracking performance, false-alarm control, explainability of alerts, processing speed, low-cost deployment, scaling across sites.

## 05 Real-time visible and thermal image fusion

- Build: alignment and fusion of paired visible-light and thermal images into a low-latency output.
- Must show: loading of paired imagery, spatial alignment with a displayed calibration result, at least two fusion methods or adjustable parameters, a before and after comparison, measured latency, update rate and computing needs.
- Resources: public or synthetic image pairs; own cameras optional.
- Scored on: alignment quality, preserved information, speed, output stability, downstream value, manufacturability of the sensor package.

## 06 Maritime track anomaly and grey-zone behaviour alerting

- Build: a dashboard that analyses AIS vessel tracks and raises explained alerts with stated uncertainty.
- Must show: historical and current tracks, detection of stopping, deviation, clustering, zone entry or reporting gaps, a risk score, timeline and explanation per alert, operator feedback (false alarms, notes, thresholds), a rule-based against a statistical or ML comparison.
- Resources: shared or simulated AIS tracks with labelled events.
- Scored on: detection performance, false-alarm control, explainability, visualisation, handling of incomplete data.

## 07 Low-cost acoustic drone detection

- Build: a system that listens to environmental audio and estimates whether a multirotor drone is present, with confidence, spectral evidence and likely causes of false alarms.
- Must show: analysis of prerecorded or live audio, separation of drones from vehicles, wind and other sounds, marked events with confidence in continuous audio, a spectrum or similar evidence view, a sensor cost and deployment concept. Optional: direction of arrival from several microphones.
- Resources: public, legally recorded or synthetic audio; two ordinary microphones are enough for direction finding.
- Scored on: detection accuracy, noise resistance, real-time performance, deployment cost, sensor scalability, customer demand.

## 08 Edge AI model optimisation and offline deployment

- Build: take an image, audio or sensor model and make it run offline on a CPU, phone or low-cost device through quantization, pruning, conversion or inference optimisation.
- Must show: baseline accuracy, latency, memory and size, at least one optimisation or format conversion, a before and after comparison, a repeatable demo on the target device or a constrained computer, the limits under offline and low-power conditions.
- Resources: a model from Challenge 4 or 7, or an approved public model; a CPU- and memory-limited laptop environment is enough.
- Scored on: model reduction, retained accuracy, inference speed, reproducible deployment, cost and energy benefit.

## 09 Commercial drone teardown and defensive risk assessment

- Build: under supervision, inspect a supplied commercial drone, document its subsystems, and propose a passive observation or safety assessment workflow.
- Must show: labelled photos or a component map, a repeatable inspection log with limitations, a passive demo (identification aid, acoustic log or review dashboard), where the workflow should not be trusted, a supervised demonstration.
- Resources: a limited number of drones from the organisers; handling rules to be announced. No flight, no active disruption.
- Scored on: accuracy of observations, usefulness of the workflow, evidence quality, treatment of uncertainty, safe handling.
