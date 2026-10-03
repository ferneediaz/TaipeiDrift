## Mid-Air

### Objective

Mid-Air is used to develop and evaluate the **relative GNSS-denied navigation pipeline**.

It provides:

- IMU at 100 Hz
- Ground-truth position, velocity, and attitude at 100 Hz
- Downward-facing RGB camera at 25 Hz
- GPS at 1 Hz
- Depth and semantic information
- Multiple environments and visual conditions

### Usage

GPS is excluded from the estimator and retained only for evaluation.

The main experiment compares:

\[
\text{IMU-only dead reckoning}
\]

with:

\[
\text{IMU + camera-based velocity correction}.
\]

Optical flow is estimated from consecutive downward-facing images. Using altitude information, the flow is converted into horizontal velocity and fused with the IMU using an EKF/ESKF.

Mid-Air is therefore mainly used to test whether visual velocity measurements can reduce inertial drift.

### Limitations

- Synthetic environment
- No real barometer measurement
- No RF/AIS measurements
- Not specifically designed for maritime flight
- Does not test absolute map-based localization

---

## DenseUAV Model

### Objective

The pretrained DenseUAV model is used for **absolute visual localization** against a previously downloaded satellite map.

It complements the continuous relative navigation developed with Mid-Air:

\[
\text{IMU + optical flow}
\rightarrow
\text{relative trajectory}
\]

while DenseUAV periodically provides:

\[
\text{camera image + offline map}
\rightarrow
\text{absolute position correction}.
\]

### Pretrained Model

We initially use the existing pretrained DenseUAV model rather than training a new localization network from scratch.

The model converts both UAV images and satellite-map images into visual embeddings:

\[
I \rightarrow f_\theta(I)=\mathbf z.
\]

Satellite-map embeddings are computed **offline before the mission** and stored onboard.

During flight, only the current UAV image needs to pass through the network.

### Offline Map Representation

The downloaded operating area is divided into georeferenced map tiles.

For each tile \(m_i\), store:

\[
m_i =
\{
x_i,
y_i,
\mathbf z_i,
\mathcal N_i
\},
\]

where:

- \(x_i,y_i\): geographic position,
- \(\mathbf z_i\): pretrained DenseUAV embedding,
- \(\mathcal N_i\): neighboring map nodes.

This produces a lightweight topological map:

```text
m12 ---- m13 ---- m14
 |        |        |
m22 ---- m23 ---- m24
 |        |        |
m32 ---- m33 ---- m34
```

The DenseUAV embedding provides the **visual similarity**, while the graph provides the **spatial constraint**.

### Localization During Flight

For a new downward-facing camera image:

\[
I_t \rightarrow \mathbf z_t.
\]

The embedding is compared against candidate map nodes:

\[
s_i =
\operatorname{sim}
(\mathbf z_t,\mathbf z_i).
\]

The current ESKF position estimate is used to restrict the search to nearby map nodes instead of searching the entire map.

Therefore:

\[
\text{ESKF estimate}
\rightarrow
\text{local graph region}
\rightarrow
\text{DenseUAV retrieval}.
\]

The graph also constrains successive matches according to the estimated UAV motion.

### Confidence

The model must not always force an absolute-position update.

A localization is accepted only when the visual match is sufficiently reliable.

Possible confidence measures include:

- similarity of the best candidate,
- difference between the best and second-best candidates,
- consistency with the previous graph node,
- consistency with the ESKF-predicted displacement,
- optional geometric verification.

If confidence is low:

\[
q_{\text{map}} < \tau,
\]

the absolute localization update is rejected and the UAV continues using relative navigation.

This is particularly important over:

- open water,
- snow,
- repetitive terrain,
- low-texture regions,
- strong appearance changes.

### Quantization

For onboard deployment, the pretrained model can later be converted from FP32 to:

\[
FP16
\]

or:

\[
INT8.
\]

The objective is to reduce:

- model size,
- inference time,
- memory usage,
- power consumption.

Quantization should be evaluated by comparing localization performance before and after compression, for example:

\[
\text{Recall@1}_{FP32}
\]

against:

\[
\text{Recall@1}_{INT8}.
\]

### Fine-Tuning

Fine-tuning is optional for the initial prototype.

It may later be useful for environments not well represented by DenseUAV, such as:

- snow,
- coastline,
- maritime environments,
- fog,
- different camera characteristics.

The first implementation should therefore use:

\[
\boxed{
\text{pretrained DenseUAV}
+
\text{precomputed map embeddings}
+
\text{topological graph}
+
\text{confidence-based rejection}
}
\]

before attempting additional training.