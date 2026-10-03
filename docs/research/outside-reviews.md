# External reviews of the strategy (3 October 2026)

The French reviews below have been translated into English.

Two models received the same context: overnight results, judging criteria, and available data. They wrote their own plan before reading ours, then critiqued it. Raw, unedited texts.

- Grok 4.7 (xAI), maximum reasoning
- GPT-6 Astra (OpenAI), maximum reasoning

## Grok 4.7

## 1. What I would build

Not continuous navigation. The product shown: a position reset, accepted only if it does not lie; otherwise, a drift budget. Three layers, never blended into a fake flight.

**(a) Data.**

1. MEASURED — error reduction. ALTO Round 2 Train, 8 sections, 37.4 km, frozen parameters (PUBLISHED: helicopter, Ohio, https://arxiv.org/abs/2207.12317). Real camera and map. Truth reserved for plots. Baseline on the same trajectory: camera dead reckoning, 219 m median per 4.6 km section. ZNCC every 300 m: 94 m (20–339). This is not “zero false”: zoom calibrated on three pre-cutoff corrections is often wrong; a better zoom raises good corrections from 21–43% to 86–100% on 3 sections. The 485 m / 78 s result (Mid-Air, synthetic dataset) is not the baseline: different aircraft.

2. MEASURED, separately. Baro Zurich, INSANE, PX4 RTK: 0.6 m at 60 s, 1.8 m at 5 min, 4.5 m at 20 min; 0.30 m + 0.112 m/√s. Scale is wrong by 3–7% without calibration under GNSS. These residuals justify noise. Do not merge them with the ALTO photos in the same number.

3. SIMULATED, second number. Real ALTO images. Baro = true altitude + replayed Zurich residuals (≥20 draws, frozen seed). DEM read at the estimated position, not the true position. The estimator does not see the true altitude. No IMU obtained by differentiating truth: the signal shape comes from truth, and a filter recovers it. Oracle zoom and camera speed (608 → 74 m): outside the video, labeled ORACLE.

INSANE (0–24 m, no orthophoto) proves the barometer, not navigation. No phone, no MARS-LVIG within the timeframe. Every graph: MEASURED, SIMULATED, or ORACLE, with the dataset, altitude, and acceptance rate next to the median.

**(b) Architecture, for this jury.**

GNSS only before the cutoff: position, barometric scale, zoom, then frozen. Camera dead reckoning. Template zoom = barometer − DEM. ZNCC, yaw and scale search. Acceptance: quad ≥ 3. Drift gate only on runs already completed, labeled SIMULATED. Output FIX / COAST / LOST.

I keep the baseline, ZNCC, quad ≥ 3, the barometer model, coverage calculated at 21% / 72% / 9% (Copernicus + WorldCover, not a flight), and the list of discarded ideas. I will not build XFeat (0/300), sun sensing (−6% final error), stereo, OSM, shadows, magnetism, or use Gazebo as proof (attitude leakage already observed).

Scoring: error reduction, validity, noise, compute, deployment. A pretty Kalman filter does not settle the question. A zero false rate on nadir imagery, an explained zoom failure, CPU results without a GPU, and a map that says no over 9% of Taiwan (37% around Kaohsiung) are useful to a skeptical judge. User: a ~500 USD multirotor mission, GNSS at takeoff, jammed afterward, that must refuse to report a position. ALTO (~450 m, 55 m/s) is the hard case, not the aircraft. INFERENCE about the jury profile; the cited errors are MEASURED.

**(c) Three demos.**

1. The 8 sections: trajectory and error over time, DR against corrections, including the 339 m section. Title: 219 → 94 m median per section, frozen parameters, with the share of false corrections stated.
2. One accepted image (4 subtemplates at the same point) against one rejected image. 44/300, 10 m median, 18 m max, 0 false. Synthetic crops from real orthophotos: 0 false among 2,395 negative groups. Without a gate, SIMULATED: 1,043 of 2,557 accepted corrections exceed 25 m.
3. Coverage: map 21/72/9, error since the last correction (240 s, SIMULATED: 18.9 m versus 220.7 m), real barometer curve.

**(d) Five ways to look like we are bluffing.**

1. Open with 26–31 m. That is the tuning section. Open with 94 m and the range.
2. Write “ALTO completed” with an IMU derived from truth. It does not affect the headline number.
3. One number that mixes real photos and simulated barometer data. Two numbers, two labels.
4. Hide 44/300. The thesis: we prefer to stay silent rather than invent a position.
5. “Zero false” with no domain. OrthoLoC, oblique ~100 m: 3 negatives accepted out of 3,540, and 3 to 5 acceptances out of 60. State the helicopter/drone domain gap before the question. Video: one full section, including a rejection.

## 2. Critique of the plan

Right: barometer − DEM → zoom → ZNCC → quad ≥ 3 → drift budget matches the measured cause. Truth used only for scoring, DEM at the estimated position, replayed Zurich residuals instead of a Gaussian, ablations and noise sweeps: this is the judging rubric. Map storage and PX4 companion computer: one slide, not code.

Too big, and dangerous. “ALTO completed” is the trap. Differentiating smoothed poses and attaching Allan variance from MUN-FRL or INSANE does not make a helicopter flying at 55 m/s. The low-frequency component is still truth. If that attitude rectifies the image, it is an oracle: OrthoLoC shows that 1° changes the result. I cut the synthetic IMU. The barometer speed term too: measured up to 10 m/s, confounded with altitude.

Sun sensing is out of the build. −6%, sun below 70°, clear skies for ~40% of daylight in Taichung in October. One line in the discarded ideas.

Global search and a growing window: 40.6% and 0/143 false, SIMULATED, 23 km². If the script runs, show a curve. Otherwise, do not write it. No new Kalman filter in 36 h.

Phone in Taiwan, rerun OrthoLoC, MARS-LVIG: a second thesis. The phone is not nadir; NLSC rights are unconfirmed; MARS-LVIG needs 8–30 GB per sequence, with no barometer. OrthoLoC is already out of spec. Cite it.

“Small map”: there is none in the brief. Time the Mac, name the chip.

Gaps. The plan combines two protocols. 94 m: forced correction every 300 m, including false corrections, outside the tuning set. Quad ≥ 3: 0 false, few acceptances; the barometer + quad test on those sections has already been inspected (70 m versus 128 m, 10/118, not held out). The jury will remember only one. Separate them. Missing: barometric scale calibration under GNSS, the domain gap, map age at Wufeng 2018/2020 (the best Taiwan evidence already on disk; real images, sensors labeled SIMULATED), and the rule against showing 26 m.

Cut down to: frozen ALTO replay, barometric zoom replay without a new IMU, three figures, one slide.

## 3. 36 hours, six people

Hours 0–2, everyone: freeze the pitch, legend, and storyboard. Forbidden in the title: 26 m, “ALTO completed,” sun sensing, XFeat, Gazebo, phone.

In parallel, about 8 h:
1. Replay the 8 sections, figures, no retuning.
2. Barometric zoom, ≥20 residual draws, DEM at the estimated position. If it is not done by H+10, the video goes out without this number.
3. Integrity table and one image pair. No new trials after H+12.
4. Map 21/72/9, MB/km² for the tiles on disk. Wufeng only if the number already exists.
5. Script matched to the files available.
6. ZNCC timing, PX4 slide, veto any number without a label.

H+14: end-to-end, look for leakage. Day 2, about 6 h: narration, export, one page (cutoff time, parameter commit, what is simulated). Stop trials 4 h before the presentation. Person 6 does not code a feature.

Pitch: “After GNSS is cut, we publish a corrected position only if the match against the onboard orthophoto agrees with itself — about 10 m and zero false among 300 real nadir images, with 44 accepted — otherwise we report drift: a 219 m median per 4.6 km section over 37 km outside the tuning set.”

Next action, 10 min: write this sentence, cross out “26 m,” launch the replay of the 8 sections without changing a parameter.

[You have received this identical output 4 times. Re-reading 'agent://AdvisorGrok/answer' will not change it — use a narrower selector (path:A-B), or proceed with the edit.]

## GPT-6 Astra

## 1. Build a causal replay, not a promise of an autonomous drone

**[INFERENCE] Proposal made before reading your plan, condensed here:** one pipeline, two separate proofs: visual navigation on a real flight; simulated multisensor integration.

### a. Assemble the data honestly

**[PUBLISHED]** Sources: [ALTO](https://github.com/MetaSLAM/ALTO), [INSANE](https://arxiv.org/html/2210.09114), [OrthoLoC](https://deepscenario.github.io/OrthoLoC/). **[MEASURED, team]** No available dataset covers your full pipeline at the target altitude.

**[INFERENCE]** ALTO proves camera→estimate→map; INSANE/PX4 characterize the sensors; OrthoLoC exposes the domain shift. Do not merge their streams as if they came from one flight. For full integration, reuse Wufeng: camera synthesized from the 2020 orthophoto, 2018 map, simulated trajectory and sensors. Label it “SIMULATED,” flat ground and no parallax; it is not a Taiwanese flight.

Barometer: white noise **0.30 m**, random walk **0.112 m/√s**, per-flight ramp **0.0024 m/s**, parameters from [MEASURED] logs. Preserve temporal correlation, compare distributions at 60/300/600 s; use 20 seeds and a doubled-noise scenario [INFERENCE]. For the IMU, characterize bias/noise on static INSANE sequences; generate specific force and rotations with correct gravity and coordinate frames. Any uncharacterized parameter remains an assumption.

**[INFERENCE]** Separate the generator, estimator, and evaluator. The estimator never receives truth. Calibration only before the GNSS cutoff; afterward, map search, DEM lookup, and correction cadence depend on the estimate, not the reference. Check clocks, extrinsics, and vertical datums. Leakage test: once the sensors are generated, removing truth does not change any estimate.

### b. Architecture and relevance to this jury

**[INFERENCE]** Keep the existing propagation: camera displacement, gyro for attitude, barometer−DEM to constrain scale; ZNCC with yaw/scale search, “quad ≥ 3,” and temporal consistency to accept corrections. No new SLAM. Show position, time since the last correction, and state “corrected / dead reckoning / unavailable”; do not invent probabilistic confidence.

**[MEASURED]** Frozen ALTO: median of section medians **219→94 m**, corrected **20–339 m** by section. ZNCC accepts **44/300** real images; XFeat **0/300**. Keep ZNCC; cut XFeat, solar, stereo, and Gazebo from the critical path. The calibration gain using the full truth is not deployable.

**[INFERENCE]** Score the same estimate with and without correction; degradations for robustness; p95 latency, RAM, and storage for integration. User: a civil inspection drone operator after GNSS loss. Deployment: calibrated camera, synchronized clocks, licensed local tiles. A Mac does not prove embedded performance or a complete $500 system.

### c. The three decisive views [INFERENCE]

1. **ALTO:** trajectories and error over time, cutoff and corrections visible; results of all eight sections, not just the best one.
2. **Wufeng:** GNSS cutoff then 60 s camera masking; recovery, median/p95 over 20 seeds, doubled noise. Permanent “SIMULATED” banner.
3. **Integrity:** true match accepted and false candidate rejected; availability and false accepts on ALTO/OrthoLoC.

### d. Prevent “bullshit”

**[MEASURED]** The 26–31 m values were tuned on Val; the later 70 m results were exploratory; OrthoLoC has **3 negatives accepted/3,540 pairs**. Neither “generalizes at 30 m” nor “zero false guaranteed.”

**[INFERENCE]** Barometer−DEM is not independent terrain localization. Forest, sea, and night remain limitations; map coverage does not mean success in flight.

[You have received this identical output 3 times. Re-reading 'agent://AdvisorAstra/partie_1' will not change it — use a narrower selector (path:A-B), or proceed with the edit.]

## 2. Keep the plan's core; remove “ALTO completed” as real-world evidence

**Plan now reviewed. [INFERENCE] Assessment:** the architecture is coherent; the experimental program exceeds what is needed to convince.

1. **Right:** pre-cutoff calibration, rectification, barometric scale, controlled corrections, ablations, and drift budget. Temporal consistency usefully complements “quad,” but it can also lock in a bad estimate: show a loss of correction, not only a successful recovery.
2. **Too ambitious:** synthesizing the ALTO IMU and then estimating its bias stability with Allan variance. Differentiated smoothed poses do not reproduce vibrations or fast dynamics; adding noise does not restore that information. A useful Allan estimate requires sufficiently long static sequences. Keep ALTO with the synthetic barometer already available; demonstrate IMU integration on Wufeng. Change “truth only for scoring” to “truth reserved for the generator and evaluator.”
3. **Missing:** ellipsoid/geoid/barometric altitude conversion, initial GNSS error, DEM uncertainty, and correlation between height, position, and scale. Barometer−DEM at a wrong position can reinforce the error. Zurich residuals do not validate ALTO's dynamic pressure at 55 m/s. Also report p95, worst section, false corrections, and time without a correction, using identical comparators.
4. **Cut:** solar sensing, global search, combinatorial sweeps, new MARS-LVIG. OrthoLoC has already been examined and is no longer a fresh test; a phone on the ground does not establish aerial navigation. For embedded compute, measure on the board only if it is available; otherwise say “Mac measured, embedded not validated.” Save time for the video and map rights.

## 3. Assign six owners now; lock the video before H36

**Proposed organization [INFERENCE] — H0 = now.**

| Owners | Individual deliverable |
|---|---|
| P1 / P2 | P1: replay, provenance, truth separation; P2: rectification, ZNCC, and rejecting false candidates. |
| P3 / P4 | P3: pre-cutoff calibration, propagation, and barometer−DEM; P4: comparisons, 20 seeds, and degraded scenarios. |
| P5 / P6 | P5: latency, RAM, storage, licenses, and adversarial audit; P6: user scenario, captures, editing, and export. |

1. **H0–H2:** everyone freezes interfaces, parameters, three views, and success criteria; P6 starts the storyboard.
2. **H2–H12:** P1–P3 connect the pipeline; P4 prepares evaluations; P5 checks inputs; P6 assembles a first video. At H12: functional freeze.
3. **H12–H20:** launch the computations, take eight hours of rest; no new features.
4. **H20–H28:** P4/P5 evaluate and perform an adversarial audit; P1–P3 fix defects only; P6 edits the real results.
5. **H28–H36:** final capture, hostile questions, export; reserve the last two hours for a full read-through and submission.

**Pitch:** “TaipeiDrift limits drift after GNSS loss by matching against offline maps, and makes it clear when its observations are no longer sufficient.”
