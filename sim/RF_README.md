# Finding the drone's position from ships' radio (AIS), without GPS

In the `strait` world, three warships transmit AIS radio. The drone has no GPS. It listens to the ships and works out where it is in two ways:

1. **From bearings:** the direction each signal comes from. This is the navigator, `nodes/rf_nav.py`, good to tens of metres.
2. **From signal strength (RSSI):** how strong each signal is. This is the live map, `nodes/rssi_map.py`, good to a few hundred metres. It is for watching only.

This document explains the radio, what the drone measures, the math of both methods, how well they work and how to run them.

## Run it

```bash
sim/run.sh                                                        # on the host: start, open the browser
docker compose exec sim bash -ic "python3 sim/scripts/check_rf_nav.py 300"   # score the bearing navigator
docker compose exec sim bash -ic "python3 sim/scripts/check_rf.py"           # check the radio model, no sim needed
```

`sim/run.sh` starts the container, launches the `strait` world without GPS (`gps:=false`) with a demo flight, and opens http://localhost:6080. The browser desktop shows:

| Where | What |
|---|---|
| Gazebo window, main view | the drone from behind (3rd person) |
| Gazebo window, right-hand panels | overview of the ships (red ball = drone), the drone's down camera |
| Top right | the RSSI map: where the drone could be from signal strength alone |
| Bottom right | the sensor monitor: GPS "no fix", the bearing estimate and its error, each ship's RSSI and bearing |

## 1. The radio: AIS

AIS (Automatic Identification System) is the radio every large ship carries. Every few seconds it broadcasts a short packet with the ship's ID (MMSI) and its own GPS position. The numbers below are from ITU-R M.1371-5 and IEC 61993-2, and are set in `config/rf.yaml`.

| | |
|---|---|
| Frequency | 161.975 and 162.025 MHz, used in turn (wavelength 1.85 m) |
| Modulation | GMSK, BT 0.4, 9600 bit/s, 25 kHz channel, 256-bit packets. GMSK has no spreading factor (that is LoRa) |
| Transmit power | Class A (large ships, all three warships here) 12.5 W = 41 dBm; Class B (small craft) 2 W = 33 dBm |
| How often | Class A every 10 s up to 14 knots, 6 s up to 23 knots |
| Drone receiver | noise floor −124 dBm, sensitivity −110 dBm |
| Range | out to the radio horizon, about 48 km from a drone at 40 m |

Gazebo's own RF system (`RFComms`) is not used: its modulation is fixed to QPSK, it only says whether a packet arrived, and it has no bearings. `nodes/rf_model.py` models the link instead:

- **Path loss:** the direct ray plus the ray reflected off the sea (two-ray model).
- **Fading:** 3 dB of random fading on every packet.
- **Packet loss:** the GMSK bit error rate, turned into the chance of losing the packet.

## 2. What the drone measures

For each packet it decodes, `nodes/rf_sensor.py` publishes on `/rf/detections`:

| Field | What it is | How it is simulated |
|---|---|---|
| `mmsi` | which ship | exact |
| `lat`, `lon` | where the ship says it is | the ship's true antenna position + 3 m of GPS error |
| `rssi_dbm` | received signal strength | true received power + 1 dB noise, rounded to 1 dB |
| `azimuth_body_rad` | bearing to the ship, measured from the drone's nose | true bearing + noise σ 3° + a fixed 1° mounting error |

A **bearing** is the direction something lies in, as an angle. A bearing of 60° means the ship is 60° to the left of straight ahead. It says which way, not how far. Measuring it needs a direction-finding antenna: several antennas spaced apart, which compare when the wave reaches each one.

```
            ship
              ●
             /
            /  60°
           /
  drone ▲ ─ ─ ─ ─ ▶ nose (straight ahead)
```

## 3. Position from bearings: `nodes/rf_nav.py`

**Three ships at least.** A bearing puts the drone somewhere on a line from the ship. The drone measures bearings against its own nose, and without GPS it does not know exactly which way it faces. So there are three unknowns: x, y and heading. Three ships give three bearings, enough for one solution: where the three lines cross.

```
     carrier ●
              \
               \
                ✕  ← drone
               / \
              /   \
  destroyer ●       ● frigate
```

**The math.** A ship at (xᵢ, yᵢ) seen from a drone at (x, y) with heading ψ should appear at bearing

```
bᵢ = atan2(yᵢ − y, xᵢ − x) − ψ
```

1. **First fix.** Once three ships have been heard within 12 s, find the (x, y, ψ) that best fits the three measured bearings: a grid search, then least squares (Gauss-Newton).
2. **Then a Kalman filter.** It tracks position, velocity, heading and gyro bias:
   - the gyro carries the heading between packets, and the position moves at the estimated velocity;
   - each new bearing pulls the estimate towards the line it defines;
   - a bearing more than 4σ off is rejected.
3. **Inputs:** only `/rf/detections` and the noisy IMU. It never reads ground truth.

**Accuracy.** A 3° bearing error at 1 km moves the line about 50 m sideways (1000 m × tan 3°). The scored run (`check_rf_nav.py`, pad A to pad B) got:
- a median error of 48 m and a 95th percentile of 113 m;
- heading within 2° RMS;
- the truth inside the filter's own 2σ ellipse 98 % of the time.

## 4. Position from signal strength (RSSI): `nodes/rssi_map.py`

This needs no special antenna and no heading, only a plain AIS receiver. It is much less accurate. Worked example: the carrier, heard at RSSI −28 dBm.

**Step 1. What the ship sends (EIRP).** Transmit power, plus its antenna gain, minus its cable loss, for a standard Class A installation:

```
EIRP = 41 dBm + 2.15 dB − 2 dB = 41.15 dBm
```

**Step 2. How much was lost on the way.** Our antenna adds 0 dB:

```
loss = EIRP − RSSI = 41.15 − (−28) = 69.15 dB
```

**Step 3. Loss to distance.** In free space the loss over a distance d is `20·log10(4π·d / λ)` dB, with λ = c / f = 1.85 m. Solving for d:

```
d = (λ / 4π) · 10^(loss / 20) = 0.147 · 10^(69.15 / 20) = 0.147 · 2,870 ≈ 422 m
```

The drone is on a circle of radius 422 m around the carrier.

**Step 4. How sure that distance is.** The signal wobbles for reasons other than distance:

| Source | dB |
|---|---|
| fading | 3 |
| RSSI measurement error | 1 |
| model error: the free-space formula ignores the sea reflection | 2 |
| combined, σ = √(3² + 1² + 2²) | **3.7** |

```
d × 10^(±3.7/20)  =  d × 0.65 … d × 1.54  →  274 m … 650 m
```

On the map, the shaded ring around each ship is this band; it holds the drone about 68 % of the time. The band is measured in dB because fading multiplies the signal, so its error is symmetric in dB, not in metres.

**Step 5. Combine the ships.** For every point (x, y) of a 10 m grid, compare its distance to each ship with the distance that ship's RSSI implies:

```
rᵢ   = √((x − xᵢ)² + (y − yᵢ)²)       distance from the point to ship i
eᵢ   = 20·log10(rᵢ / dᵢ)               how far off that is, in dB
cost = Σᵢ (eᵢ / σ)²
```

- **The ×:** the point with the lowest cost, the most likely position.
- **Dark red:** `cost − min < 2.30`, the 68 % region.
- **Light red:** `cost − min < 6.18`, the 95 % region.

2.30 and 6.18 are the chi-square limits for two unknowns (x and y).

**Step 6. Why three ships, enforced.** One ring could put the drone anywhere on it. Two rings usually cross at two points. The third ring picks one of them. So the map draws nothing until three different ships have been heard within the last 30 s (`MIN_SHIPS`). Until then it shows "Heard N of 3 ships".

**Accuracy.** The ring's width grows with the distance: −35 % to +54 % is a ring about 600 m wide for a ship 1 km away. On top of that, the sea reflection makes the real signal 4 to 6 dB stronger or weaker than free space at some distances. In the example, the carrier was really about 750 m away, but its signal came in about 4 dB strong, so step 3 said 422 m. The receiver cannot know this. Typical error: a few hundred metres.

## 5. How good is it?

One snapshot from the current triangle of ships, simulated 400 times:

| What the drone measures | Median error | 95 % under |
|---|---|---|
| Bearings, 3° (what the sim assumes) | 53 m | 127 m |
| Bearings, 8° (more realistic for a small drone) | 131 m | 339 m |
| RSSI only (a plain AIS receiver) | 400 m | 3 km |

**Verdict:** a good backup layer, not a primary position source.

**What's good:**
- It doesn't drift: an absolute fix every few seconds, however long the flight.
- It works over open sea, where camera and terrain matching have nothing to see.
- There's plenty of shipping in the Taiwan Strait.

**What limits it:**
- Bearings need a direction-finding antenna, hard to fit on a small quad at a 1.85 m wavelength.
- RSSI alone is only good to hundreds of metres.
- It trusts the positions the ships report. Their GPS can be jammed too, warships often switch AIS off, and AIS spoofing happens.
- AIS transmissions are only loosely tied to the clock, so timing them for distance (time of arrival) would be tens of km off.

**Best use:** camera and terrain matching as the main source over land, AIS bearings to keep the error to about 100 m over sea, and both merged with the IMU in one filter.

## Files

| File | What |
|---|---|
| `config/rf.yaml` | the ships (size, route, AIS class), the radio, the receiver, the direction finder |
| `nodes/rf_model.py` | radio physics: path loss, noise, GMSK bit errors, packet loss, lat/lon conversion |
| `nodes/ship_traffic.py` | sails the ships, publishes `/ships/<name>/odom` |
| `nodes/rf_sensor.py` | the drone's AIS receiver and direction finder: `/rf/detections`, `/rf/truth` |
| `nodes/rf_nav.py` | position from bearings: `/rf_nav/odom` |
| `nodes/rssi_map.py` | the live RSSI map |
| `scripts/check_rf.py` | checks the radio model and prints the link budget against range |
| `scripts/check_rf_nav.py` | scores `rf_nav` against ground truth and saves a plot to `data/sim/rf_nav/` |
| `run.sh` | starts everything and opens the browser |
