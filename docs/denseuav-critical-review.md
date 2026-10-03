# Offline DenseUAV Localization: Critical Review and Technical Proposal

## Verdict

**The proposed architecture is a credible offline image-retrieval component, not yet a reliable absolute-position sensor and not a complete GNSS-free navigation system.**

Precomputing georeferenced map embeddings is sound. Comparing an onboard camera embedding against them is supported by the official DenseUAV repository. The unsupported step is treating the winning tile as a trustworthy drone position without specifying rejection, spatial uncertainty, operating conditions, or navigation integration.

The central engineering question is not simply “Does the right tile rank first?” It is **“When the system accepts a fix, how often is that fix dangerously wrong, and how does it behave when it cannot localize?”**

This review comments on the supplied proposal, checks relevant public implementation details, and proposes evaluation requirements. It does not change the project's existing navigation plan. It is a static review: no checkpoint was downloaded, no model was executed, and no onboard performance or localization accuracy was measured.

## 1. What is being proposed

The supplied design has two stages:

1. Before deployment, encode map images into a georeferenced database.
2. During flight, encode a downward-looking image and return the coordinates of the most similar database image.

In notation:

\[
\mathcal D = \{(f_i,p_i)\}_{i=1}^{N},\qquad
q_t = E_{UAV}(I_t),\qquad
\hat p_t = p_{\arg\max_i \operatorname{sim}(q_t,f_i)}.
\]

Here, the coordinates are supplied by the map. The network does not invent latitude and longitude; it associates an observation with a reference image whose location is already known.

### What the proposal gets right

- Map encoding can be performed before flight, avoiding repeated map-encoder inference onboard.
- Runtime retrieval can use only local assets and does not intrinsically require internet access.
- A georeferenced reference database makes an absolute horizontal location estimate possible without a live GNSS measurement.
- Global retrieval can search without an initial position, provided the correct area is covered and visually distinguishable.
- Stored embeddings can be much smaller than storing all reference imagery, although this depends on descriptor dimensions and map sampling.

These are genuine advantages. None establishes accuracy, integrity, or real-time feasibility on its own.

## 2. What DenseUAV actually provides

DenseUAV is the dataset and associated research project for *Vision-Based UAV Self-Positioning in Low-Altitude Urban Environments*. The official repository provides baseline implementations, training/evaluation code, and links to baseline checkpoints. Calling it “the DenseUAV model” is convenient shorthand, but leaves the actual backbone, head, configuration, checkpoint, and preprocessing unspecified. [S1]

The proposal's two encoder boxes are conceptual roles. They do not establish that the selected implementation contains two independently trained networks; parameter sharing and branch behavior must follow the chosen configuration.

### Evidence in the public scripts

| Source | Observed behavior | Implication |
|---|---|---|
| `forwardAllSatelliteHub.py` | Encodes reference images, normalizes descriptors, and saves features and labels in a MATLAB file. | Offline gallery generation is implemented. |
| `inference_global.py` | Loads stored gallery features, computes a matrix-product score, sorts candidates, and selects the best label. | The proposed retrieval mechanism has a concrete precedent. |
| `inference_global.py` | Resolves the selected label through a position dictionary. | Coordinates are looked up from map metadata, not directly regressed. |
| `inference_global.py` | Reads GPS metadata from query photographs and records query/reference coordinates. | This demo is not an as-is GNSS-free runtime, even though its retrieval calculation does not use those query coordinates. |
| Both application scripts | Use CUDA calls, external assets/configurations, and machine-specific default paths. | Research scripts require integration work; they are not an onboard deployment package. |

Sources: [S2], [S3]. These observations concern the inspected files on the repository's mutable `main` branch, not an executed or commit-pinned reproduction.

The scripts also differ in how they unpack model outputs. That warrants checking compatibility with the intended checkpoint and model API before treating them as one ready-to-run pipeline. Static inspection alone does not establish a runnable combination.

## 3. Critical flaw: argmax cannot say “I do not know”

The proposed decision rule always returns a tile whenever the gallery is nonempty. It returns a winner even when:

- The drone is outside the mapped region.
- The image contains water, fog, glare, blur, or a covered lens.
- Every reference differs substantially from the observation.
- Several distant places are visually indistinguishable.
- The correct footprint or scale is absent from the database.

**Being the least bad match is not evidence of being a correct match.** A wrong result with plausible coordinates can be more harmful than no result, particularly if a navigation estimator assigns it excessive weight.

### Necessary change

The component needs an explicit rejected/unavailable output. Candidate generation and fix acceptance must be separate decisions.

Possible acceptance evidence includes descriptor scores, geographically distinct alternative hypotheses, geometric consistency, and consistency across a sequence. These are candidate checks to evaluate, not a guarantee that any particular combination is sufficient.

A single similarity threshold is not automatically transferable across terrain types, map sizes, or camera conditions. Searching more distractors increases the opportunity for a strong accidental match. Thresholds must be calibrated on representative valid and invalid queries, then assessed on separate held-out data.

## 4. A tile coordinate is not the exact camera position

The proposal assigns the winning tile's coordinate to the drone. This is a discretized location estimate. It does not determine where the camera footprint lies inside the tile, nor whether the most recognizable object is centered under the drone.

The database must explicitly define whether `p_i` means tile center, image footprint center, a landmark coordinate, or another anchor. That definition must be consistent across preprocessing and evaluation.

### Best-case grid error

For a square grid with center spacing \(s\), if the estimator always chooses the nearest correct center, positions lie inside the covered grid cells, and map geometry is exact, then:

\[
e_{\max}=\frac{s}{\sqrt{2}}.
\]

For positions uniformly distributed within a cell, the radial RMS discretization error is:

\[
e_{RMS}=\frac{s}{\sqrt{6}}.
\]

For an illustrative 20 m spacing, those values are approximately 14.1 m and 8.2 m. **They are ideal grid-quantization figures, not DenseUAV accuracy claims or bounds on actual retrieval error.** A wrong match can be arbitrarily farther away within the map.

Tile width and tile stride are different parameters. Overlapping large tiles can have closely spaced centers; saying “we use 100 m tiles” is not enough to infer localization resolution.

### What a refinement stage could add

Local feature matching and a suitable geometric model could estimate an offset within a candidate region. This would be an additional algorithm, not a capability implied by global embedding retrieval. A planar homography is also not universally valid over buildings and uneven terrain.

Without refinement, the honest output is “estimated reference location with measured uncertainty,” not “precise drone position.” The proposal also does not independently estimate altitude or full 6-DoF pose.

## 5. Image geometry is an input requirement, not an implementation detail

A downward-mounted camera does not guarantee a nadir image in flight. Body roll, pitch, camera mounting, gimbal motion, terrain elevation, and perspective affect what it sees.

### Scale

Under a flat-ground, nadir-view approximation, footprint width is:

\[
W \approx 2h_{AGL}\tan(\theta_h/2),
\]

where \(h_{AGL}\) is height above local ground and \(\theta_h\) is horizontal field of view. Consequently, altitude changes alter the physical area represented by a fixed-size image tensor.

Barometric altitude is not automatically height above ground. Terrain elevation and a compatible vertical reference are needed to derive that quantity from an altitude estimate; alternatively, another measurement may provide ground distance. The proposal does not specify this input or how unknown scale is handled.

### Heading and tilt

North-up map imagery and a rotating UAV camera are different input distributions. Training augmentation may help, but is not proof of invariance at the required accuracy. Tilt introduces perspective differences; buildings can create parallax and occlusion that simple rotations cannot remove.

Choose and test a concrete strategy: controlled imaging geometry, rectification using calibrated geometry, a bank of scale/orientation hypotheses, or demonstrated model robustness. Each has different computation and sensor requirements.

### Preprocessing contract

Checkpoint choice, resizing, cropping, RGB normalization, feature extraction, and descriptor normalization must be compatible between database construction and runtime queries. Equal embedding dimension alone does not make two descriptor sets compatible.

## 6. Domain generalization remains unproven

The official dataset description covers university campuses and includes drone images at 80, 90, and 100 m. Its published split separates training and query universities. That is useful evidence of a designed evaluation setup, but not a guarantee for a new operational region. [S1]

A target deployment may differ in:

- Terrain: campus buildings versus farmland, forest, coastline, or open water.
- Appearance: season, lighting, shadows, weather, or construction changes.
- Imaging: camera optics, resolution, exposure, blur, and compression.
- Geometry: altitude, oblique views, building height, and terrain relief.
- Reference data: provider, age, ground resolution, orthorectification, and georeferencing quality.

A successful campus retrieval cannot establish robustness across those shifts. Pretrained checkpoints are starting points for measurement, not certificates of transfer.

### Evaluation leakage to avoid

Randomly separating overlapping crops or neighboring video frames can make an experiment much easier than a new flight in a new area. Split by geographic region and acquisition session when those are the intended generalization claims. Keep threshold calibration separate from final evaluation.

Do not select the gallery search area or per-frame map crop using the query's evaluation GPS unless the experiment explicitly supplies that same prior to the operational system. Ground-truth coordinates are valid for scoring; silently using them to simplify inference changes the task.

## 7. Open water is a missing-observation case

Featureless water generally does not provide a stable, unique correspondence to a stored aerial map. Waves, reflections, and moving vessels are not reliable static map anchors. A model can still emit a descriptor and a confident-looking winner; that does not create geographic information.

A coastline or persistent structure may provide useful evidence when visible. That is a different observation condition from open-water imagery.

**The proposed component alone cannot establish bounded navigation error during an arbitrary open-water crossing.** During rejected visual observations, another estimator must propagate motion and represent increasing uncertainty. The allowed outage duration depends on measured drift and the mission's error tolerance; neither is supplied here.

A land-only demonstration is legitimate if declared. Quietly presenting it as validation for sea crossings is not.

## 8. Confidence and uncertainty need their own design

Cosine similarity is not a probability of correctness. Applying softmax to candidate scores does not fix this: it normalizes over the available candidates even when all are wrong.

### Top-two margins have a spatial trap

Two highly ranked overlapping tiles may describe the same correct neighborhood. Their small score margin need not indicate harmful ambiguity. Conversely, a strong margin over poor alternatives does not establish that the query belongs anywhere in the gallery.

Assess alternatives at the level of geographically distinct hypotheses, not just adjacent ranked tiles. Define the grouping distance from the map sampling and required localization tolerance, rather than choosing it solely to improve reported confidence.

### Do not average unrelated locations

A weighted average of two distant candidates can lie at a third location with no supporting image match. Preserve multiple hypotheses or reject ambiguity instead of manufacturing an intermediate fix.

### Useful component output

A deployment-oriented result should communicate:

| Field | Purpose |
|---|---|
| Image capture timestamp | Associates the observation with the correct navigation state. |
| Candidate coordinates and coordinate frame | Defines what position is being claimed. |
| Accepted/rejected/unavailable status | Prevents unconditional use of an arbitrary winner. |
| Calibrated uncertainty or explicitly uncalibrated quality evidence | Avoids pretending a ranking score is a measurement covariance. |
| Map/model version and rejection reason | Makes failures reproducible and distinguishes missing coverage from other failures. |

This is a proposed interface, not an interface provided by the reviewed description.

## 9. Navigation integration can amplify errors

A camera retrieval module and a navigation estimator have different responsibilities. Retrieval proposes geographically anchored observations; the estimator tracks motion and uncertainty over time.

### Timing

The returned position describes the image at capture time, not at the end of inference. At 15 m/s, an illustrative 200 ms delay corresponds to 3 m of travel. This is arithmetic, not a measured system delay. Ignoring timestamps can introduce motion-dependent error even with a correct visual match.

### Correlation

Neighboring video frames often share scene content and failure modes. Ten consistent matches are not necessarily ten independent confirmations. Repeatedly feeding a persistent false match into a filter can make the filter confidently wrong.

### Prior lock-in

Restricting retrieval to an estimated neighborhood saves computation and removes distractors. It can also exclude the true position after drift or an earlier false fix. A tracking mode needs a defined failure/reacquisition policy if local search is used.

### Cold start is a different experiment

Three conditions must not be conflated:

1. Global localization with no initial position.
2. Tracking from a known launch position, without using GNSS in flight.
3. Recovery after losing a previously available GNSS solution.

All can involve GNSS-free runtime retrieval. They differ substantially in search ambiguity and required prior information. The proposal should declare which is being demonstrated.

## 10. Offline does not imply affordable onboard computation

Map precomputation removes one cost, not all costs. Runtime still includes camera ingestion, preprocessing, query encoding, retrieval, acceptance checks, and any geometric verification.

### Descriptor storage

For \(N\) descriptors of dimension \(d\), stored with \(b\) bytes per component:

\[
M_{descriptors}=Ndb.
\]

Illustratively, 100,000 descriptors of dimension 768 in FP32 require 307,200,000 bytes, or about 307 MB decimal / 293 MiB. This excludes model weights, metadata, retrieval-index overhead, image buffers, activations, and reference imagery. These dimensions are an example, not a verified DenseUAV configuration.

For a roughly planar area \(A\), a regular grid with stride \(s\) has approximately \(A/s^2\) centers, ignoring boundary effects. Halving the stride roughly quadruples the number of centers. Additional reference scales or orientations can multiply storage again.

### Search cost

Exact dot-product retrieval is \(O(Nd)\) per query. The inspected demo sorts all scores even though it returns only the best label; that is a property of the demo, not a necessary cost of top-one retrieval. [S3]

Approximate indexing, quantization, and reduced precision may reduce cost, but can change rankings and rejection behavior. Measure their accuracy/integrity impact rather than assuming they preserve the system's behavior.

### Measurements required

Measure end-to-end latency distributions, peak memory, sustained throughput, and power/thermal behavior on the intended device. Separate initialization from steady-state operation. The README's GPU-memory prerequisite is not a measured minimum runtime requirement for every deployment configuration. [S1]

No specific onboard frame rate is justified by the supplied proposal or this review.

## 11. The map is a first-class dependency

“Store tiles onboard” omits important assumptions:

- **Coverage:** the actual camera footprint and plausible search region must be represented, including map boundaries.
- **Georeferencing:** correct recognition of a misregistered image still produces a biased coordinate.
- **Currency:** new buildings, vegetation changes, and roadworks may invalidate visual correspondences.
- **Traceability:** descriptors must stay paired with their coordinates, preprocessing, and checkpoint version.
- **Rights and availability:** imagery must be obtainable and licensed for the intended offline use.

Storing only embeddings is enough for descriptor retrieval. It is not enough for later pixel-level verification unless the corresponding reference imagery or suitable local features are also available onboard. Storage claims must match the actual verification design.

## 12. Failure severity and required evidence

The priorities below are engineering judgments for this proposal, not measured failure frequencies.

| Priority | Failure | Consequence | Evidence needed before relying on fixes |
|---|---|---|---|
| Critical | Always accepting top-one retrieval | A visually unsupported coordinate enters navigation. | Held-out rejection and false-accept measurements, including out-of-map queries. |
| Critical | Claiming open-water observability | Navigation depends on an absent absolute reference. | Explicit unavailable behavior and measured outage drift from the remaining system. |
| High | Treating a tile center as exact position | Systematic and discretization errors are hidden. | Meter-based error distribution tied to tile stride and anchor definition. |
| High | Assuming geographic/domain transfer | Benchmark success fails to transfer to deployment. | Target-area, target-camera, held-out flight results. |
| High | Ignoring timestamps and correlated errors | Correct fixes are misapplied; persistent wrong fixes dominate. | Sequential replay with capture times, delayed fixes, and repeated false hypotheses. |

Resource feasibility, map quality, and cold-start recovery are additional deployment gates even though they are not separate rows in this five-item priority table.

## 13. A defensible architecture

```text
BEFORE DEPLOYMENT
Georeferenced imagery + defined sampling/scale policy
                       |
              Compatible map encoder
                       |
       Versioned descriptors + coordinate metadata
       (+ imagery/local features if verification needs them)

ONBOARD
Timestamped camera image
          |
Compatible preprocessing and query encoder
          |
Candidate retrieval from local database
          |
Ambiguity / validity checks
(+ geometric or temporal verification if implemented)
          |
     Accept or reject
       /         \
Accepted fix     Unavailable observation
with uncertainty       |
       \               /
       Navigation estimator
       motion propagation + uncertainty tracking
```

This separates what the proposal already describes from the missing acceptance and navigation contracts. It does not prescribe that every possible verification technique must be implemented. The simplest adequate design is the one that meets a declared error/false-accept requirement on representative data.

## 14. Experiments that would actually test the claim

### A. Establish the retrieval baseline

Use the chosen checkpoint and its compatible preprocessing on a bounded, georeferenced target-area gallery. Start with exact search to avoid conflating model behavior with approximate-index behavior. Record ranked candidates and geographic errors, not only the winning image.

Baseline retrieval establishes whether the visual signal is useful. It does not establish fix integrity.

### B. Test rejection with deliberate negative queries

Include outside-map images, water, texture-poor terrain, degraded frames, and visually similar but geographically wrong locations. Calibrate the acceptance rule on a separate subset.

Define a maximum acceptable horizontal error \(E\) from the demonstration requirements before selecting thresholds. A fix with error above \(E\) is an incorrect accepted fix, even if the retrieved scene looks plausible.

### C. Report complementary metrics

| Metric | Why it matters |
|---|---|
| Accepted-fix error distribution in meters | Quantifies usefulness when the system emits a fix; report median, upper percentiles, and observed maximum with sample count. |
| Incorrect accepted fixes / all accepted fixes | Estimates how often an emitted fix is wrong under the evaluation distribution. |
| Accepted negative queries / all negative queries | Exposes failure to reject observations that should not yield a map fix. |
| Correct accepted fixes / eligible in-map queries, plus rejection rates | Prevents an always-reject system from appearing successful. |
| Time to first valid fix, reacquisition time, and end-to-end latency | Measures operational availability and timing. |

Report raw counts and distinguish frames from independent flights/regions. Zero observed false accepts is not proof of zero false-accept probability, especially with a small or strongly correlated sample.

### D. Replay a sequence

Test changing headings/scales, ambiguous repeated scenes, missing observations, and reacquisition. Compare retrieval alone with the proposed navigation integration if that integration exists. Track whether an incorrect fix causes lasting divergence rather than evaluating each frame in isolation.

### E. Run on the intended hardware

Only after establishing useful retrieval and rejection behavior, measure the complete runtime on the intended device. Acceptance criteria should cover both localization integrity and sustained resource use. Laptop success cannot substitute for this test.

## 15. Recommended wording for the proposal

Replace the opening claim with:

> DenseUAV-based cross-view image retrieval can serve as an offline source of candidate absolute horizontal position fixes. A compatible pretrained encoder and a georeferenced map-feature database are stored onboard. During flight, camera descriptors are matched against this database without requiring a live GNSS measurement or internet connection. A retrieved location is accepted only when the system's validated acceptance criteria are met; otherwise, the visual component reports no fix. Accuracy, availability, and onboard latency must be measured for the target camera, terrain, map, and hardware. The component alone does not provide continuous navigation or guarantee localization over open water.

Replace the diagram's final unconditional `Absolute position` with:

```text
Candidate map location
          |
   Acceptance decision
      /          \
Position fix     No visual fix
+ uncertainty
```

## 16. Decision for this project

**Proceed with a bounded feasibility experiment, not an unconditional navigation claim.** The approach is a reasonable way to test whether target-area imagery can produce useful absolute visual observations offline.

Before building around it, establish three things:

1. The selected checkpoint retrieves the correct neighborhood on independent target-area images.
2. An acceptance rule rejects enough unsupported/ambiguous observations without making useful fixes too rare.
3. The actual onboard runtime meets the selected latency and memory constraints.

If retrieval succeeds but rejection fails, the result is an image-search demo, not a trustworthy navigation sensor. If the scene has no stable map correspondence, a different model does not automatically resolve the lack of observable position information.

## 17. Technical proposal: retrieval followed by verified map registration

**Proposed improvement:** use the learned descriptor to find plausible neighborhoods, then estimate and verify the actual camera-footprint location against the corresponding map imagery. Keep motion propagation separate. Reject observations when either visual evidence or supported geometry is inadequate.

This section is a concrete implementation proposal, not an implemented or experimentally validated system. The proposed numerical settings are starting configurations or acceptance targets, not measured capabilities.

### 17.1 Scope and operating contract

The existing [project plan](PLAN.md) assumes GNSS loss after initialization. This proposal does not silently replace that scenario. The visual module itself must not read live GNSS, and two initialization modes should be evaluated separately:

| Mode | Allowed initialization | Required result |
|---|---|---|
| Known-start tracking | A launch location entered from the map, or the last authorized pre-outage state in the current project scenario. | Propagate motion and accept independently verified visual corrections. |
| Unknown-start acquisition | No geographic prior; locally stored operational-area map only. | Search globally, retain ambiguity, and report unavailable until acquisition criteria pass. |

For a demonstration claiming no GNSS at any point, use the manually defined launch location or unknown-start mode. Ground-truth GNSS can be retained in a separate evaluator, never exposed to inference.

The initial geometric operating envelope is daylight, textured land with locally approximately planar ground and a calibrated downward camera. This is a declared verification envelope, not a claim that water, night, forests, or dense high-rise scenes are solved. Those remain required negative/stress cases.

### 17.2 Inputs and assets

Implement against explicit inputs rather than assumed metadata:

| Input | Required content | Failure behavior |
|---|---|---|
| Camera sample | RGB image, monotonic capture timestamp, intrinsics, distortion calibration, camera-to-body transform. | Reject invalid samples; never use processing-completion time as capture time. |
| Motion sample | Timestamped relative motion, uncertainty, and attitude where available from the navigation backend. | Retrieval may still run; temporal/heading checks requiring missing information cannot claim to pass. |
| Height information | Height above ground with uncertainty, or a declared supported footprint-scale search interval. | Search supported scales or report unsupported geometry; do not silently equate barometric altitude with AGL. |
| Map package | Orthophoto, metric coordinate reference, pixel-to-world transform, coverage mask, tile metadata, descriptors. | Refuse incompatible packages; reject out-of-coverage projected positions. |
| Model package | Pinned source revision, checkpoint checksum, architecture/configuration, preprocessing definition, descriptor layout. | Fail initialization if gallery and query model contracts differ. |

Use a projected metric coordinate system suitable for the bounded operating area for distances and registration. Convert to WGS84 only at the component boundary when needed. Store the transform explicitly; do not calculate meter distances directly from latitude/longitude degrees.

### 17.3 Offline map build

1. Select legal, georeferenced imagery with known ground sampling distance. Record its acquisition date, coverage and any known alignment uncertainty.
2. Sample overlapping tiles at a proposed 20 m center stride. Select physical tile footprints from the camera field of view and supported AGL range; use three log-spaced footprints as an initial bank, expanding the bank if validation shows gaps.
3. Encode tiles with the selected checkpoint's exact preprocessing and descriptor normalization. Save normalized FP32 descriptors initially, plus center, footprint, scale, source-image reference and coordinate transform.
4. Retain reference pixels for geometric verification. Build a manifest tying those pixels, coordinates, descriptors and model/configuration checksums together.
5. Validate coordinate round trips and visualize footprints over the original orthophoto before evaluating retrieval.

Do not resample every footprint to the same physical ground resolution accidentally: the learned encoder's input resolution and the physical area represented are separate choices. Retain both in metadata.

Start with an exact matrix-product search, not an approximate index. It supplies a reproducible baseline before optimization. Candidate retrieval should operate over the whole declared map in acquisition mode; runtime GPS must never narrow that map.

### 17.4 Runtime candidate generation

Process the newest available frame with a queue of capacity one: replace an unprocessed old frame rather than accumulating stale observations. Preserve the selected frame's original timestamp.

Use these bounded stages:

1. Check image validity and quality. Calibrate blur/exposure rejection on real inputs rather than importing arbitrary universal thresholds.
2. Apply the checkpoint-compatible preprocessing and compute a normalized query descriptor.
3. Retrieve the top 50 tiles initially. Group overlapping/nearby matches in metric space, retaining up to five geographically distinct regions for verification. The grouping radius must reflect tile footprint and tolerated location error.
4. Fetch map patches covering those regions and the supported footprint sizes. Verify candidates as described below.
5. Return an accepted observation or an explicit rejection reason. If the best candidates disagree geographically and cannot be disambiguated, preserve ambiguity rather than averaging positions.

These top-50/top-five settings bound initial work; they are not guarantees that the correct region survives pruning. Measure candidate-region recall separately from final fix accuracy. If that recall is poor, adjust the retrieval/model/map design before trying to repair it with stronger acceptance thresholds.

Heading compensation must follow the checkpoint's demonstrated behavior. With no trusted absolute heading, do not rotate the camera frame as though IMU yaw were north-referenced. Test rotation robustness first; evaluate a finite rotation bank if necessary and include its inference cost.

### 17.5 Geometric verification and sub-tile position

Use an initial classical verifier: **OpenCV SIFT correspondences plus robust homography estimation** on undistorted camera images and candidate orthophoto patches. This is chosen as an inspectable baseline, not because cross-view correspondence is guaranteed. Descriptor retrieval and local feature matching solve different problems: the first can succeed while the second fails.

For each candidate:

1. Extract local features and match camera/map descriptors with a ratio filter and mutual consistency. Keep coordinate transforms from all crops/resizes.
2. Estimate a camera-pixel-to-map-pixel homography using robust sampling. Reject insufficient or nearly collinear support.
3. Check inlier count, inlier ratio, symmetric transfer residuals, and spatial spread across the image. Require support around the image center rather than a small corner of one roof.
4. Project the image-center ray's pixel and footprint into map coordinates. Reject ill-conditioned projections, implausible footprint size/shape, and coverage violations.
5. Compare surviving geographically distinct candidates using calibrated retrieval and geometric evidence. Reject competing valid solutions instead of selecting by inlier count alone.

An initial tuning configuration could require at least 20 inliers, at least 25% inliers among filtered matches, support across at least four cells of a 3-by-3 image grid, and median symmetric transfer error below 3 pixels at a recorded verification resolution. These values are hypotheses to calibrate; they do not imply a false-fix guarantee. Pixel thresholds must be interpreted at the configured image/map resolutions.

**Camera footprint center is still not automatically camera ground position.** A homography maps image pixels to ground points. For a truly nadir optical axis, the center ray intersects beneath the camera. For a tilted view, that ground point is horizontally displaced from the camera.

Under a local flat-ground model, with unit optical-axis direction `d` expressed in a local z-up world frame, `d_z < 0`, and AGL height `h`, the center-ray ground intersection `g` and camera horizontal position `c` satisfy:

```text
lambda = -h / d_z
g_xy = c_xy + lambda * d_xy
c_xy = g_xy - lambda * d_xy
```

Use calibrated attitude/extrinsics and propagate their uncertainty into this correction. If world-referenced attitude or AGL is unavailable, do not call the corrected camera position observed. Either reject the camera-position fix or restrict acceptance to a justified near-nadir envelope whose maximum displacement `h * tan(tilt)` fits the error budget. At 100 m AGL, even 5 degrees implies approximately 8.75 m displacement.

In unknown-heading acquisition, a homography alone does not automatically supply an independently trustworthy full attitude/height solution. Pose recovery would need its own calibrated geometry, ambiguity handling and validation. The conservative initial design rejects cases that require an unavailable pose correction.

If the SIFT verifier has poor availability under cross-view appearance changes, evaluate a learned local matcher on the same frozen split. Do not fall back to emitting unchecked retrieval winners. If planar registration fails systematically over relief, the required design change is terrain-aware registration or a narrower declared envelope, not looser integrity thresholds.

### 17.6 Acceptance and uncertainty

Separate three questions: does the image contain usable evidence, is one geographic solution supported, and is its estimated error acceptable?

Calibrate acceptance using held-out positive/negative examples and geographically distinct competing regions. Avoid multiplying retrieval confidence, inlier ratio, and temporal agreement as if they were independent probabilities.

For accepted fixes, account for reference-map uncertainty, registration error, attitude/height correction, camera-to-body lever arm, and timing. Estimate residual error against independent ground truth across representative conditions. Validate coverage of the reported error regions; do not derive measurement covariance directly from cosine similarity or an inlier count.

Until calibrated uncertainty exists, emit a diagnostic candidate result marked uncalibrated and keep it out of automatic navigation corrections. This is a development result, not a deployable fallback.

### 17.7 Temporal logic and navigation boundary

Use four explicit states:

| State | Behavior | Transition |
|---|---|---|
| `ACQUIRING` | Global search; no trusted global visual fix yet. | Enter `TRACKING` after three geometrically accepted observations whose inter-fix displacement agrees with measured relative motion. |
| `TRACKING` | Supply accepted timestamped fixes; motion backend propagates between them. | Enter `DEGRADED` when no accepted fix arrives for a proposed 2 s, or the navigation uncertainty exceeds its declared limit. |
| `DEGRADED` | No forced visual correction; report growing uncertainty and try reacquisition. | Recover through the acquisition checks; enter `LOST` at the declared navigation error limit. |
| `LOST` | Mark navigation position unusable for the declared requirement; continue bounded global search. | Recover only after acquisition checks and an explicit estimator reinitialization policy. |

Three agreeing observations are a proposed debounce condition, not three independent proofs. Require relative motion with valid uncertainty for that check; adjacent identical frames are not sufficient. If relative motion is unavailable, the system can report individually verified visual observations but must not claim the specified tracking/acquisition contract is satisfied.

The 2 s timeout is provisional. The uncertainty limit must come from the navigation requirement and measured propagation error; time alone does not establish trustworthiness.

For this proposal, keep exact global retrieval on the bounded map in all states initially. This avoids prior-induced search lock-in. If later optimization introduces local search, retain a global reacquisition path and test recovery after deliberately wrong priors.

The existing navigation backend should consume accepted observations at capture time, using buffered state updates and repropagation where supported. If delayed-state updates are unavailable, reject observations older than a declared allowable age; do not inject them as current-position fixes. This component does not prescribe an additional parallel navigation filter.

Avoid reusing the same prior twice as independent evidence. If a navigation prior influences candidate selection or acceptance, evaluate that dependence before treating the returned fix as an independent measurement. Geometric and map evidence should remain the main absolute-position evidence.

### 17.8 Proposed software boundaries

The names below describe implementation responsibilities; these modules do not exist as a result of this document.

| Component | Input → output | Contract |
|---|---|---|
| Map builder | Imagery + camera envelope + model → immutable map package | Offline, deterministic for pinned inputs, manifest-checked. |
| Retriever | Timestamped image + package → ranked geographic candidate regions | No query ground-truth access; exact search baseline. |
| Verifier | Image + region pixels + geometry → registration evidence and estimated location | Explicit unsupported-geometry and ambiguity outputs. |
| Fix manager | Verification + relative motion + calibration → accepted fix or rejection | State machine and calibrated uncertainty; no unconditional top-one fallback. |
| Replay evaluator | Sensor stream + hidden ground truth + outputs → metrics | Ground truth isolated from inference; compares acceptance, errors and runtime. |

Represent rejection reasons explicitly, for example `INVALID_IMAGE`, `UNSUPPORTED_GEOMETRY`, `NO_VERIFIED_MATCH`, `AMBIGUOUS_REGIONS`, `OUTSIDE_COVERAGE`, and `STALE_OBSERVATION`. Do not label every failure “low confidence”; the causes require different fixes.

### 17.9 Experimental targets and implementation gates

Use these as proposed demonstration targets, to be agreed before claiming success. They are not safety certification criteria:

| Gate | Proposed acceptance evidence | If it fails |
|---|---|---|
| Retrieval | Correct region appears in the five verification regions for at least 95% of eligible held-out land queries. | Revisit checkpoint/domain, footprint scales, rotation handling and candidate budget. |
| Verified fixes | At least 50% of eligible land queries produce accepted fixes; at least 95% of accepted fixes are within 10 m; report all fixes beyond 25 m separately. | Inspect registration, attitude/height, map bias and acceptance calibration. |
| Rejection | Zero accepted fixes on a declared set of at least 1,000 negative queries spanning distinct sessions/regions; report sample dependence and uncertainty. | Improve rejection or declare the conditions unsupported; do not hide failed negatives. |
| Acquisition/recovery | A valid acquisition within 5 s in at least 90% of declared observable acquisition trials; demonstrate recovery after a wrong prior and an observation outage. | Diagnose observability, candidate recall and state-machine constraints. |
| Runtime | Sustained end-to-end p95 latency at or below 500 ms at a 2 Hz request rate, without growing queues and within the actual device memory/power budget. | Profile and optimize measured bottlenecks; rerun accuracy/rejection after changes. |

The 10 m target requires geometric refinement and suitable map/attitude accuracy; the 20 m gallery stride alone cannot establish it. The negative-query target is deliberately stronger than a few successful screenshots, but zero observed failures in 1,000 independent trials still gives an approximate one-sided 95% upper failure-rate bound of 0.3%, not zero risk. Correlated frames provide weaker evidence.

The combined system must also beat the same motion-only baseline on held-out sequences without increasing large-error events. Report outage drift and recovery discontinuities even when per-frame retrieval metrics look good.

Implementation should follow dependency order:

1. Pin and run a compatible upstream checkpoint on a reproducible reference example; confirm gallery/query descriptor contracts.
2. Build the target-area package and measure exact retrieval, with evaluator-only ground truth.
3. Add geometric registration, camera-position correction and frozen-split acceptance calibration.
4. Connect the explicit state machine to recorded relative motion and then to the existing navigation backend; test delay and outage behavior.
5. Execute the whole pipeline on the intended device and freeze the evaluated configuration.

These are gates, not permission to present an unfinished stage as a complete navigation system.

### 17.10 Available evidence and remaining prerequisites

The inspected project plan says Mid-Air lacks a georeferenced aerial map and treats ALTO map/sensor availability as unresolved. Therefore, Mid-Air motion replay alone cannot validate this proposed map-localization pipeline. Route-memory retrieval is a different experiment and must not be substituted silently.

An actual implementation run still needs an accessible compatible checkpoint, target-area orthophoto with usable georeferencing, independent drone imagery, camera calibration, and target hardware for performance claims. Motion/height/attitude data are additionally needed for the corresponding geometry and sequential-navigation claims. None of their availability is established by publishing this proposal.

**Expected improvement if validated:** retrieval supplies a bounded set of map hypotheses; geometry estimates a sub-tile location; acceptance prevents unsupported fixes; temporal integration supplies continuity while explicitly exposing loss of observability. The design makes failure visible rather than claiming the model has eliminated it.

## Sources and evidence limits

- **[S1] Official repository and README:** [Dmmm1997/DenseUAV](https://github.com/Dmmm1997/DenseUAV). Dataset description, supported configurations, checkpoint links, and prerequisites.
- **[S2] Offline feature generation:** [`forwardAllSatelliteHub.py`](https://github.com/Dmmm1997/DenseUAV/blob/main/tool/applications/forwardAllSatelliteHub.py). Inspected for feature normalization and gallery serialization.
- **[S3] Global retrieval demo:** [`inference_global.py`](https://github.com/Dmmm1997/DenseUAV/blob/main/tool/applications/inference_global.py). Inspected for similarity calculation, coordinate lookup, query GPS reads, and execution assumptions.
- **[S4] Associated paper:** [Vision-Based UAV Self-Positioning in Low-Altitude Urban Environments](https://arxiv.org/abs/2201.09201), linked by the official repository; bibliographic reference, not a source of reproduced performance numbers in this review.
- **Project context:** [Existing matching/filter explanation](method.md). Related explanation only; this critique does not change its algorithm or the project's selected scenario.

Sources inspected on 2026-10-02. Repository links track mutable upstream content. Quantization, storage, and timing numbers above are explicitly illustrative calculations. Proposed checks and acceptance requirements are engineering recommendations, not claims that DenseUAV already implements or satisfies them.
