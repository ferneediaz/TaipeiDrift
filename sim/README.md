# Simulator

ROS 2 Jazzy and Gazebo Harmonic in Docker, with a drone that carries the same sensors as the Mid-Air dataset platform. This folder holds the environment and the drone. Flight scenarios (the GNSS cut, the trajectories, recorded runs) come later.

## Quick start: watch the drone fly

### What you need

- **Docker Desktop**, installed and running (the whale icon in the menu bar). Under Settings, then Resources, give it at least 6 GB of memory. The default 8 GB is fine.
- **Disk:** about 5 GB free for the image.
- **Port 6080** free on your machine.
- **Any recent machine:** Apple Silicon Mac, Intel Mac, Linux, or Windows with Docker Desktop. The image runs natively on both arm64 and x86.

You do not need ROS, Gazebo or Python on your machine. Everything runs in the container.

### Steps

Run these from the repository root:

```bash
git checkout simulations && git pull       # the branch with the simulator
cd sim
docker compose up -d --build               # first time about 10 minutes, later a few seconds
docker compose exec -d sim bash -ic "ros2 launch sim/launch/sim.launch.py cam_res:=512 demo:=true > /tmp/sim.log 2>&1"
```

Then open **http://localhost:6080** in your browser. On a Mac you can type `open http://localhost:6080`.

Gazebo does not open as a window on your own desktop. It runs in a Linux desktop inside the container, and that desktop is shown in the browser tab.

After one to two minutes the browser desktop shows three windows:

- **Gazebo Sim (left):** a third-person chase camera that follows the drone. It is a spectator view, not a sensor. Drag with the mouse to look around.
- **Image View (top right):** what the drone's down camera sees. This is the image the navigation uses.
- **Sensor monitor (bottom right):** live values from the IMU, the barometer (air pressure sensor), GPS and the camera, each next to the ground truth with its error.

The drone takes off to 40 m and flies 40 m circles at 6 m/s. To see the sensor monitor in your own Mac terminal as well:

```bash
docker compose exec sim bash -ic "python3 sim/nodes/sensor_monitor.py"
```

### The islands world: fly across open sea

Same steps, with `world:=islands` added to the launch line:

```bash
docker compose exec -d sim bash -ic "ros2 launch sim/launch/sim.launch.py cam_res:=512 demo:=true world:=islands > /tmp/sim.log 2>&1"
```

The drone starts on the helipad of island A, climbs to 40 m, flies about 850 m east across the open sea to the helipad of island B, turns, and flies back, again and again. If a simulation is already running, run `docker compose restart` first. For the sensor monitor in your own terminal, add `--world islands` to its command.

### Stop and restart

```bash
docker compose restart     # stops the simulation and gives a fresh container, in a few seconds
docker compose down        # stops the container when you are done
```

To start the demo again, run the `docker compose exec -d ...` line once more (after `docker compose up -d` if you ran `down`). Only one simulation can run at a time; a second start says so and exits.

### If something does not work

| Symptom | Fix |
|---|---|
| `Cannot connect to the Docker daemon` | Start Docker Desktop and wait until it says it is running |
| `port is already allocated` on 6080 | Another program uses the port. Stop it, or change `"6080:6080"` in `compose.yaml` to `"6081:6080"` and open http://localhost:6081 |
| Browser shows an empty black desktop | The simulation takes one to two minutes to open its windows. Wait, then reload the page |
| Black desktop after several minutes | Read the log: `docker compose exec sim tail -50 /tmp/sim.log` |
| Gazebo window opens, but no drone in view | The chase camera starts a few seconds after the window opens. If it does not, run `docker compose exec sim bash -ic "python3 sim/nodes/demo_flight.py --camera-only"` |
| Very slow simulation | Close other heavy apps, keep `cam_res:=512`, and give Docker more CPUs under Settings, then Resources |
| You pulled new changes to `docker/` | Rebuild: `docker compose up -d --build` |

## Run

```bash
cd sim
docker compose up -d --build
```

Open http://localhost:6080. This is a Linux desktop in the browser. Start the simulator from your terminal:

```bash
docker compose exec sim bash -ic "ros2 launch sim/launch/sim.launch.py"
```

The Gazebo window appears in the browser tab. Launch options:

| Option | Default | Meaning |
|---|---|---|
| `cam_res` | `1024` | Down-camera width and height in pixels. 1024 matches Mid-Air; use 512 if the simulation runs slowly |
| `gps` | `true` | `false` removes the GNSS receiver |
| `gnss_cutoff_s` | `20.0` | Seconds since the first raw GPS fix when the gate closes; negative means never |
| `record_mode` | `light` | `light` records sensor/pose topics but no images; `full` also records down and forward images |
| `run_label` | empty | Optional run folder under `outputs/sim_runs/velocity_phase/` |
| `stereo` | `false` | `true` adds a second down camera at `/camera/down_right/image_raw` |
| `stereo_baseline_m` | `0.30` | Stereo camera offset along body -Y (right), used when `stereo:=true` |
| `gui` | `true` | `false` runs Gazebo without its window |
| `world` | `terrain` | A world file in `worlds/`: `terrain`, `islands`, or `city` |
| `demo` | `false` | `true` flies circles and opens the camera view and sensor monitor |

Example: `ros2 launch sim/launch/sim.launch.py cam_res:=512 gui:=false world:=terrain gnss_cutoff_s:=60 stereo:=true`.

### City GNSS-denial run

The `city` world is generated on first launch and contains roads, irregular building footprints and heights, a plaza and simple vegetation. `demo:=true` flies a loop across the city at 80 m. GNSS begins available and the gate closes 20 simulation seconds after its first fix by default. The cutoff is logged by the gate and published on `/nav/gnss_available`; raw GPS remains visible on `/sim/gps_raw`, while `/gps/fix` stops publishing after denial.

Every launch creates `outputs/sim_runs/<world>_YYYYMMDD_HHMMSS/` with the rosbag, `metadata.json`, `config.json`, `sim.log`, `trajectory.csv`, `estimator_debug.csv`, `gnss_debug.csv` and `velocity_debug.csv`. Supplying `run_label:=V1_gnss_velocity_fit_open` places it under `outputs/sim_runs/velocity_phase/V1_gnss_velocity_fit_open/`. Light mode records ground truth (evaluation only), raw and gated GPS, status, IMU, barometer, camera calibration, TF/TF-static, and estimator output. Full mode also records both image streams.

The live adapter does not subscribe to ground truth. It converts `/gps/fix` WGS84 coordinates into local ENU using the world's declared spherical-coordinate origin, initializes position from GNSS, initializes tilt and IMU biases from a settled stationary IMU window (yaw is unobservable and starts at 0), then fuses GNSS position updates and a robust generalized least-squares velocity fit from GNSS positions. The fit uses an 8 s window, at least six fixes spanning six seconds, the per-fix position covariance, and one normalized-residual outlier rejection pass. Velocity covariance is the slope block of the GLS parameter covariance. Fits use non-overlapping fix windows before being passed to the existing ESKF velocity update. After cutoff the estimator propagates IMU, barometer, and enabled visual updates only. Ground truth stays in the logger for evaluation. `velocity_debug.csv` records estimate and fit errors in ENU; the logger rotates Gazebo odometry's body-frame GT velocity into ENU before scoring it. `/nav/estimator_status`, `estimator_debug.csv`, and `gnss_debug.csv` contain update decisions and diagnostics. `vision_rotation:=false` and `vision_direction:=false` independently disable those updates without removing the code.

PowerShell commands from `sim/`:

```powershell
docker compose up -d --build
docker compose exec sim bash -ic "ros2 launch sim/launch/sim.launch.py world:=city demo:=true gui:=false cam_res:=512 gnss_cutoff_s:=-1 record_mode:=light vision_rotation:=true vision_direction:=true run_label:=V1_gnss_velocity_fit_open"
# Stop V1 with Ctrl+C, then run V2:
docker compose exec sim bash -ic "ros2 launch sim/launch/sim.launch.py world:=city demo:=true gui:=false cam_res:=512 gnss_cut_s:=20 record_mode:=light vision_rotation:=true vision_direction:=true run_label:=V2_gnss_velocity_fit_denial"
```

Watch Gazebo at http://localhost:6080. In a second terminal, inspect the launch and gate logs with `docker compose exec sim bash -ic "tail -f /tmp/sim.log"`. End the launch with Ctrl+C in its attached terminal, or restart the container; stop the container with `docker compose down`.

### Record a common replay

With the simulator running, start the recorder before the scenario. From the host, in `sim/`:

```bash
docker compose exec -d sim bash -ic "cd /ws/TaipeiDrift && python3 sim/nodes/recorder.py --name t_sim_terrain_cut60 --world terrain --duration-s 255 --image-rate-hz 5 --cam-res 512 --gnss-cut-s 60 --stereo --stereo-baseline-m 0.30 > /tmp/recorder.log 2>&1"
docker compose exec -d sim bash -ic "cd /ws/TaipeiDrift && python3 sim/scripts/t_scenario.py > /tmp/scenario.log 2>&1"
```

The recorder writes `taipeidrift-replay/1` under `/ws/TaipeiDrift/recordings/<name>` (the mounted host working copy). Images are sampled at `image_rate_hz`; truth is evaluator-only.

Docker on a Mac has no access to the graphics chip, so the processor draws the camera images. The simulation then runs slower than real time. Gazebo shows the real-time factor at the bottom right. All timestamps use simulation time, so recorded data is correct at any speed. On an M3 MacBook:

| Setup | Real-time factor |
|---|---|
| `cam_res:=512`, no Gazebo window | 0.94 |
| `cam_res:=1024`, no Gazebo window | 0.75 |
| `cam_res:=512`, Gazebo window open (the demo) | about 0.55 |

Sensors keep their rates in simulation time in every case.

Gazebo also installs natively on macOS (`brew install gz-harmonic`), but its GUI is marked unstable there, and ROS 2 has to be built from source. That is why we use Docker.

To check that every sensor publishes at the right rate with the Mid-Air intrinsics, run `python3 sim/scripts/check_sensors.py` in the container. `python3 sim/scripts/smoke_flight.py` flies a short test pattern.

Stop with Ctrl+C, or `docker compose restart` from another terminal, and `docker compose down` when done.

## Fly

The drone takes velocity commands. They need no autopilot.

```bash
docker compose exec sim bash -i
ros2 topic pub --once /enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 2.0}}"   # climb 2 m/s; Ctrl+C to stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                  # hover
```

`linear.x/y/z` are in m/s and `angular.z` is the yaw rate in rad/s, all in the drone's heading frame.

To see the down camera in the browser desktop: `docker compose exec -d sim bash -ic "rqt_image_view /camera/down/image_raw"`.

## The drone

Its sensors match Mid-Air (Fonder and Van Droogenbroeck, CVPRW 2019, section 3.1 and Fig. 3), minus the front stereo cameras we do not use, plus a barometer.

| Sensor | Mid-Air | Here | ROS topic |
|---|---|---|---|
| Down camera | 1024×1024, 90° FOV, 25 Hz, ideal pinhole, global shutter | Same (`cam_res` can lower it); optional right camera uses the same intrinsics | `/camera/down/image_raw`, `/camera/down/camera_info`, optional `/camera/down_right/image_raw` |
| Forward camera | Forward camera for relative rotation and translation direction | 1024×1024, 90° FOV, 25 Hz, ideal pinhole | `/camera/forward/image_raw`, `/camera/forward/camera_info` |
| IMU | 100 Hz, eq. 1 noise: white noise + random-walk bias, drawn per flight | Same model; published orientation is zeroed and marked unavailable | `/imu/data`; the draw is on `/imu/params` |
| Barometer | none | Gazebo Air Pressure sensor, 50 Hz, 10 Pa noise + slow drift | `/air_pressure` (Pa) |
| GNSS | 1 Hz | 1 Hz, σ 1.5 m horizontal, 3 m vertical; optional simulation-time cutoff | `/gps/fix` (gated from `/sim/gps_raw`) |
| Ground truth | 100 Hz | 100 Hz | `/ground_truth/odom` |

- **Airframe.** It is AirSim's default quadcopter, as in Mid-Air: 1 kg, arm 0.2275 m, about 4.2 N thrust per rotor.
- **Sensor placement.** All sensors sit at one point, `sensor_link`, 0.5 m ahead of the airframe centre on the body X-axis, as in Mid-Air. That point is the model origin, so ground truth describes the IMU itself and there are no lever arms.
- **Camera.** It looks straight down, and the top of the image points forward. Intrinsics are fx = fy = cx = cy = width/2, as in Mid-Air. Stereo is generated as a temporary model variant; the base SDF remains unchanged.
- **IMU noise.** The bounds in `config/sensor_noise.yaml` are our assumption for a consumer MEMS IMU; Mid-Air does not publish its own. Use `seed` for repeatable runs and `scale` for the noise sweep. The true Gazebo attitude is zeroed before `/imu/data` is published.
- **Untouched data.** The noise-free IMU and pressure stay available on `/sim/imu_raw` and `/sim/air_pressure_raw`; GNSS raw data is bridged to `/sim/gps_raw`.

## The environments

There are two worlds. Choose one with `world:=`.

### `terrain`: fields and woods

`worlds/terrain.sdf` is flat textured ground at the Wufeng, Taichung site. The ground model is built by `scripts/make_ground.py`, and the launch file runs it the first time:

- **Default:** a procedural 1 × 1 km landscape of fields, tracks and fine grain, at 0.24 m per pixel.
- **The real orthophoto:** fetch it with `python scripts/fetch_aerial.py`, then run `uv run python sim/scripts/make_ground.py --aerial` on the host. It needs rasterio.

On top of the ground stand 400 real 3D trees, generated by `scripts/make_trees.py` on first launch:

- **Kinds:** broadleaf trees with rounded crowns, and conifers with cones, 6 to 18 m tall.
- **Placement:** clumps of woods with single trees between them, over 400 × 400 m around the start point. A 12 m patch around the start is kept clear.
- **What they add:** the chase camera shows depth, and the down camera sees real height and parallax, as over Mid-Air's forests.
- **Collision:** trees are solid.

Use `--count`, `--area` and `--seed` for a different forest.

Hills from the Copernicus DEM are a later step.

### `islands`: two islands and open sea

`worlds/islands.sdf` is for flights out over water, where GNSS is gone and the camera sees almost nothing it can track. Everything in it is generated by `scripts/make_islands.py` on first launch:

- **Island A**, about 300 m across, holds the start helipad. The world origin is that helipad.
- **Island B**, about 220 m across, lies about 850 m to the east (helipad at x = 850, y = 140), across roughly 700 m of open sea.
- **The islands** are low, as in Penghu: 3D terrain up to about 14 m above the sea, with beaches, dry grass, scrub, rock and dirt tracks. They carry 3D trees, a few stone houses, a helipad each, and a red and white lighthouse on island B as a landmark you can see from the sea. All of it is solid.
- **The sea** is a flat, opaque surface 4 m below the helipads (z = -4). It is coloured by depth: surf and turquoise shallows at the shore, deep blue offshore with only a faint swell. A large plain ocean runs to the horizon.
- **Location:** open water in the Taiwan Strait between Penghu and Taiwan (23.65 N, 119.85 E), so GPS reports real coordinates there. GPS altitude is above the sea.
- **Heights:** z = 0 is the helipad top, 4 m above the sea. The demo's 40 m is above the helipad, so 44 m above the water.

The water does not move, so it gives a camera slightly more to hold on to than real waves would. Waves, wind and the GNSS cut belong to the scenario step. `make_islands.py --seed` gives different islands; `layout.json` in the generated model lists the helipads, houses, lighthouse and trees for scripts.

**Topographic map.** `maps/` holds a map of this world and its elevation grid:

- `islands_topo.png` and `islands_topo.pdf`: contours every 1 m on land and every 2 m under water, the coastline, helipads, houses, trees, the lighthouse, the demo route, a 100 m grid in world metres, and latitude and longitude on the edges.
- `islands_dem.tif`: elevation in metres above the sea (negative is depth), float32, 1 m per pixel, top is north. `islands_dem.json` says where it lies in world metres and in latitude and longitude. This is the map a GNSS-denied navigator can match against.

Heights on the map are above the sea; world z is that minus 4 m. After changing the islands, redraw it in the container with `python3 sim/scripts/make_topo_map.py`.

## Record data in the Mid-Air format

`nodes/record_midair.py` saves a flight in the files and layout of the [Mid-Air dataset](https://midair.ulg.ac.be/data_organization.html), so code written for Mid-Air reads our world's flights unchanged:

```
data/sim/<world>/sunny/
  color_down/trajectory_0000/000000.JPEG ...   down camera, 1024 x 1024, 25 Hz
  sensor_records.hdf5                         per trajectory: camera_data, groundtruth, imu, gps (as Mid-Air)
```

- **Camera:** `color_down` remains the Mid-Air downward stream. `color_left` is the forward stream used by the visual rotation and velocity-direction experiments; both are JPEG at 25 Hz. Mid-Air uses PNG only for depth, disparity, segmentation and normals, which we do not record.
- **Rates and rows:** frame j goes with row 4j of the 100 Hz ground truth and IMU, and row j // 25 of the 1 Hz GPS. Quaternions are w, x, y, z; axes are north, east, down.
- **Turn rates follow the Mid-Air files, not their documentation:** the gyroscope and the true angular velocity are around the world axes, the accelerometer is in the drone's axes (forward, right, down). See `docs/findings.md`, section 2.2. Code written for Mid-Air therefore works on these recordings unchanged.
- **Look up one picture:** `python3 sim/scripts/frame_info.py data/sim/islands/sunny/color_down/trajectory_0000/000123.JPEG` prints every sensor value recorded with it.
- **Differences from Mid-Air:** every trajectory uses the world origin (helipad A), not its own start point, so all flights share one map frame. GPS DOP values are fixed, since no satellites are simulated. Barometer readings are added under `barometer/pressure`.
- **Every frame is kept:** the recorder pauses the simulation and steps it 40 ms at a time, waiting for each frame. A CPU renders the camera slower than real time, so without this most frames would be skipped. It takes the pictures and steps the simulation through Gazebo's own transport, not ROS: over ROS a 3 MB picture could wait about 3 s whenever a piece of it was lost. Recording runs at about half real time.
- **Check a recording:** `python3 sim/scripts/check_recording.py data/sim/islands/sunny` compares every sensor with the ground truth and the pictures with the map, and prints PASS or FAIL for each.

To record a set of flights in one go, run on the host (each flight restarts the simulator, takes off from helipad A, and records one pass of its route):

```bash
sim/scripts/record_islands_set.sh "pads 40 0"                  # one round trip, pad A to B and back at 40 m, about 10 minutes
sim/scripts/record_islands_set.sh                              # survey at 40 and 80 m, round trips at 40, 60, 80 and 100 m, about 2 hours
```

Or by hand: start the world without the window, start a flight, then record:

```bash
docker compose exec -d sim bash -ic "ros2 launch sim/launch/sim.launch.py world:=islands gui:=false > /tmp/sim.log 2>&1"
docker compose exec -d sim bash -ic "python3 sim/nodes/demo_flight.py --world islands --route survey > /tmp/demo.log 2>&1"
docker compose exec sim bash -ic "python3 sim/nodes/record_midair.py --world islands --duration 600"
```

`--route pads` flies between the helipads, and `--route survey` flies lines over both islands. Each run of the recorder adds the next `trajectory_XXXX`. Wait until the drone has climbed (about 20 s) before recording. Sizes: about 45 kB per frame over open sea and about 150 kB over land, so roughly 0.5 GB for a flight from pad to pad and back and 3 GB for a full survey. `data/sim/` is not committed.

## Frames

Gazebo and ROS use ENU (x east, y north, z up) and a body frame that is forward, left, up. Mid-Air and `docs/PLAN.md` use NED. The live ROS topics are in ENU; `record_midair.py` converts to NED when it writes a recording.

## Files

```
compose.yaml, docker/       the container: ROS 2 Jazzy, Gazebo Harmonic, browser desktop
launch/sim.launch.py        starts Gazebo, spawns the drone, bridges topics, adds noise
models/midair_quad/         the drone
models/ground/              generated ground (not committed)
models/trees/               generated 3D trees (not committed)
models/islands/             generated islands, sea and helipads (not committed)
worlds/terrain.sdf          fields and woods
worlds/islands.sdf          two islands and open sea
config/bridge.yaml          Gazebo ↔ ROS topics
config/sensor_noise.yaml    IMU noise bounds, barometer drift
nodes/sensor_noise.py       Mid-Air IMU noise model, barometer drift and attitude zeroing
nodes/gnss_gate.py          simulation-time GNSS cutoff
nodes/recorder.py           common replay-format recorder
nodes/sensor_monitor.py     live sensor values in the terminal
nodes/demo_flight.py        demo flights (circles, island to island, or a survey of the islands) and the chase camera
nodes/record_midair.py      records a flight in the Mid-Air dataset format
scripts/record_islands_set.sh  records a set of flights over the islands world
scripts/frame_info.py       prints the sensor values recorded with one picture
scripts/check_recording.py  checks a recording: shapes, files, sensors against the truth, pictures against the map
scripts/make_ground.py      ground texture
scripts/make_trees.py       3D trees
scripts/make_islands.py     islands world scenery
scripts/make_topo_map.py    topographic map and elevation grid of the islands world
maps/                       the islands map (PNG, PDF) and elevation grid (TIFF + JSON)
scripts/check_sensors.py    checks rates, frames and camera intrinsics
scripts/smoke_flight.py     short test flight, saves one down-camera frame
scripts/t_scenario.py       60 m GNSS-cut flight scenario
```
