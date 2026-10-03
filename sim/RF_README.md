# Finding the drone's position from ships' radio (AIS), without GPS

In the `strait` world, three warships transmit AIS radio. The drone's GPS is cut 20 s into the flight. It listens to the ships and works out where it is from the angle of arrival (AoA) of their signals, shown two ways:

1. **Filtered:** the bearing (AoA) to each ship, the direction its signal comes from. This is the navigator, `nodes/rf_nav.py`, good to tens of metres.
2. **A live snapshot from the same bearings:** the AoA map, `nodes/aoa_map.py`, which redoes the three-ship fix every half second with no filter, so you can watch the lines of position cross. It is for watching only. Signal strength (RSSI) is not used for the position: it is good only to a few hundred metres.

This document explains the radio, what the drone measures, the math, how well they work and how to run them.

## Run it

```bash
sim/run.sh                                                        # on the host: start, open the browser
docker compose exec sim bash -ic "python3 sim/scripts/check_rf_nav.py 300"   # score the navigators
docker compose exec sim bash -ic "python3 sim/scripts/check_rf.py"           # check the radio model, no sim needed
```

`sim/run.sh` starts the container, launches the `strait` world with a demo flight (GPS first, cut after 20 s; `sim/run.sh strait gps:=false` for no GPS at all), and opens http://localhost:6080. The bearing fix is also fused into the ESKF estimator (`/nav_rf/odom`); see the sim README, "The ESKF with the ships' fix". The browser desktop shows:

| Where | What |
|---|---|
| Gazebo window, main view | the drone from behind (3rd person) |
| Gazebo window, right-hand panels | overview of the ships (red ball = drone), the drone's down camera |
| Top right | the AoA map: where the drone could be from the ships' angles of arrival alone |
| Bottom right | the sensor monitor: the NAVIGATION table (RF only, ESKF, ESKF + RF, each against the truth), the sensors, each ship's angle of arrival (AoA) and its error |

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

A **bearing** is the direction something lies in, as an angle. A bearing of 60° means the ship is 60° to the left of straight ahead. It says which way, not how far. It is the signal's angle of arrival (AoA).

**The antenna that measures it.** The drone carries a pseudo-Doppler direction finder, built by the launch file from `direction_finder` in `config/rf.yaml` (strait world only; the Mid-Air model file is unchanged). It copies a design that has flown on a four-rotor UAS at 150 MHz, close to AIS: Gerhard and Tokekar, [Experimental Evaluation of a Pseudo-Doppler Direction-Finding System for Localizing Radio Tags](https://arxiv.org/pdf/2003.00386) (Virginia Tech, 2020), figure 1. It has no moving parts.
- **The array:** four 17 cm helical VHF stubs standing on the rotor arms, 9.5 cm from the centre, inboard of the propeller discs. They form a square with 13 cm sides, 0.07 wavelength. A square array is unambiguous while its side is under 0.35 wavelength; `rf_sensor.py` refuses a larger one.
- **How it measures the angle:** an RF switch connects the four stubs one after another to a single receiver, 1000 times a second. The received phase then moves as if one antenna were circling, so a ship off to one side produces a Doppler tone. The phase of that tone against the switching gives the bearing. An AIS packet lasts 26.67 ms, which is 26.7 electronic rotations; `rf_sensor.py` refuses fewer than 10.
- **The radio:** the switch board and a software-defined radio sit on the body, as the paper's Opera Cake and HackRF One do. The packets are decoded through the same stubs, so there is no separate whip. A helical stub picks up about 5 dB less than a full whip (`antenna_gain_db: -5`). At these ranges the ships are still 70 dB or more above the noise, so the 3° floor (calibration, multipath off the airframe) sets the accuracy.

**Where the design comes from.**

- **The direct source:** Gerhard and Tokekar (Virginia Tech, 2020), [Experimental Evaluation of a Pseudo-Doppler Direction-Finding System for Localizing Radio Tags](https://arxiv.org/pdf/2003.00386), figure 1 on page 1. Four whips at the corners of a square feed one radio through an RF switch (their Opera Cake board and HackRF One), and the caption says the antennas "can be mounted on the arms of the hybrid UAS", a four-rotor VTOL drone. From it we take:
  - four antennas in a square;
  - mounted on the rotor arms;
  - one RF switch and one radio (our green board and grey box);
  - the size rules: under 0.35 wavelength for an unambiguous bearing, about 0.22 works best (their section III).

  They flew it at 150 MHz, close to AIS's 162 MHz, and found a radio tag to within 5 m.
- **Where they got it:** two older ideas.
  - [Doppler direction finding](https://en.wikipedia.org/wiki/Doppler_radio_direction_finding): an antenna circling at a known rate sees the signal's frequency rise and fall, and the timing gives the direction. Pseudo-Doppler fakes the circling by switching fixed antennas; it is the standard tool of amateur-radio "fox hunting", which is where the paper's 0.22-wavelength rule comes from.
  - [The Adcock array](https://en.wikipedia.org/wiki/Adcock_antenna) (Frank Adcock's patent of 1919): four vertical elements in a square, the standard direction-finding layout for a century, from 40 m masts to 13 cm tactical sets. Commercial direction finders still use circular arrays like this, in a radome, for example the [R&S ADD507](https://www.rohde-schwarz.com/us/products/aerospace-defense-security/compact-single-channel/rs-add507-compact-vhf-uhf-df-antenna_334290.html).
- **Our own choices:** these adapt the design to this drone and to AIS; they are not from the paper.
  - 17 cm helical stubs instead of the paper's full whips, to stay clear of the propellers.
  - Placed 9.5 cm out on the arms, to fit the Mid-Air drone.
  - Switching at 1 kHz with a minimum of 10 rotations per packet, for AIS's 26.67 ms packets. The paper tracked continuous tag beeps, not AIS.
  - The mounts, the coax and the box's look are cosmetic.
- **Why not a spinning antenna:** the first version of this drone carried a spinning loop. Rotating loops are historically real (the aircraft direction finders of the 1930s, such as the Bendix loop on Amelia Earhart's Lockheed Electra), but aviation replaced them with fixed antennas switched electronically, and no drone with a fast-spinning loop has been published. Drones that rotate a directional antenna, such as the wildlife-tracking multirotors that turn the whole drone with a Yagi ([ConservationBots](https://arxiv.org/pdf/2308.08104)), take seconds per bearing, too slow for 26.67 ms AIS packets.

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

## 4. The live AoA map: `nodes/aoa_map.py`

The top-right window shows a snapshot fix from angles of arrival alone: no filter, no memory, no RSSI. It is the resection of section 3 redone every half second, so you can watch the lines of position cross.

**Step 1. Take one bearing per ship.** Use the last packet of each ship heard within 12 s (rf_nav's `START_WINDOW_S`). Nothing is drawn until three ships are in that window, because the heading is unknown: three bearings for three unknowns.

**Step 2. Refer every bearing to one moment.** The drone turns between packets, and a bearing is measured from its nose. The gyro's integrated yaw rate says how far it turned between packet i and the newest packet:

```
bᵢ' = bᵢ − (yaw_gyro(t_now) − yaw_gyro(tᵢ))
```

The text box lists each bearing, its σ, its age and this gyro correction.

**Step 3. How sure each bearing is.** The direction finder's own σ, its fixed mounting bias, the ship's reported-position error, and the drone's movement during the window (rf_nav's `START_SIGMA_M`, 40 m) across the line:

```
σᵢ(x)² = σ_DFᵢ² + (1°)² + (3 m² + 40 m²) / |sᵢ − x|²
```

**Step 4. Score every point of a 10 m grid.** For each point, take the heading that fits best (the weighted circular mean of the residuals), then:

```
cost(x) = Σᵢ ( wrap(atan2(sᵢ − x) − ψ − bᵢ') / σᵢ(x) )²
```

- **Dark red:** `cost − min < 2.30`, the 68 % region. **Light red:** `cost − min < 6.18`, 95 %. Chi-square limits for two unknowns (x and y), with the heading fitted out.
- **The ×:** the best grid point, refined by rf_nav's own `resection()`.
- **Coloured lines:** each ship's line of position, from the ship along `ψ + bᵢ' + 180°`, with dashed lines at ±2σ.

**Step 5. Compare.** The title gives the snapshot's error against the truth at the newest bearing's time, and its heading error. The black circle is rf_nav's filtered estimate, the gold star the true position now. Typical snapshot error: tens of metres, a few times worse than rf_nav, which averages the bearings over time with the gyro.

**Why not RSSI.** Signal strength gives a distance by inverting free-space loss, but fading (3 dB), RSSI error (1 dB) and the sea reflection (2 dB) make it uncertain by about ±3.7 dB, which is −35 % to +54 % of the distance: a ring about 600 m wide for a ship 1 km away. That puts an RSSI-only fix hundreds of metres off (section 5), so RSSI is used only to decide whether a packet is decoded.

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
| `nodes/aoa_map.py` | the live AoA map |
| `scripts/check_rf.py` | checks the radio model and prints the link budget against range |
| `scripts/check_rf_nav.py` | scores `rf_nav` and both ESKFs against ground truth and saves a plot to `data/sim/rf_nav/` |
| `run.sh` | starts everything and opens the browser |
