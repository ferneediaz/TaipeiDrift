# Offline DenseUAV Localization

DenseUAV can be used as an **offline absolute-localization component** on the drone. The model and the required geographic map representation are stored onboard, so localization does not require GNSS or an internet connection during flight.

## Core Idea

Before deployment, satellite/aerial imagery of the operational area is divided into geographic tiles.

Each map tile is processed by the DenseUAV map encoder:

\[
I_i^{map} \rightarrow f_i
\]

The resulting feature embedding is stored together with the geographic position of the tile:

\[
\mathcal{D} =
\{(f_1,p_1),(f_2,p_2),\ldots,(f_N,p_N)\}
\]

where:

- \(f_i\) is the learned feature representation of map tile \(i\)
- \(p_i\) is its geographic position

This database is generated **before the flight** and stored onboard.

During flight, the downward-facing camera produces an image:

\[
I_t^{UAV}
\]

which is processed by the UAV encoder:

\[
I_t^{UAV} \rightarrow q_t
\]

The query embedding \(q_t\) is compared against the stored map embeddings:

\[
i^* =
\arg\max_i
\operatorname{sim}(q_t,f_i)
\]

The position associated with the best matching map tile provides an estimate of the drone's **absolute geographic position**.

---

## Offline Architecture

```text
               ONBOARD DRONE
          No GNSS / No Internet

Downward RGB camera
        |
        v
+-------------------+
| DenseUAV UAV      |
| image encoder     |
+---------+---------+
          |
          | query embedding
          v
+---------------------------+
| Offline map database      |
|                           |
| tile 001 -> embedding     |
| tile 002 -> embedding     |
| tile 003 -> embedding     |
| ...                       |
+------------+--------------+
             |
             | similarity search
             v
      Candidate map tile
             |
             v
      Absolute position