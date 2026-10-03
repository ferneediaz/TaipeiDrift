"""Start Gazebo with the Mid-Air-like drone, the ROS bridge and the sensor noise node.

    ros2 launch sim/launch/sim.launch.py [cam_res:=1024] [gps:=true] [gui:=true] [world:=terrain] [demo:=false]
        [stereo:=false] [stereo_baseline_m:=0.30] [wind:=none] [ships:=auto] [gnss_cutoff_s:=20.0]
        [record_mode:=light] [vision_rotation:=true] [vision_direction:=true] [run_label:=]

cam_res  down camera width and height in pixels; 1024 matches Mid-Air, 512 renders faster on a CPU
gps      false removes the GNSS receiver from the drone
gui      false runs Gazebo without its window (server only)
world    a file name in sim/worlds/ without .sdf: terrain (fields and woods), islands (two islands and open sea),
         strait (the islands with warships that transmit AIS) or city (roads and buildings)
stereo              true adds a right down-camera by generating a temporary model variant
stereo_baseline_m   right camera offset along body -Y, in metres
wind                SPEED_MPS,FROM_DEG, for example 6,20: 6 m/s from the north-north-east, with gusts
ships    true sails the AIS-transmitting ships of config/rf.yaml (nodes/ship_traffic.py), runs the drone's
         AIS receiver and direction finder (nodes/rf_sensor.py) and the triangulation navigator (nodes/rf_nav.py),
         adds a fixed overview camera, shown as a panel beside the chase view in the Gazebo window, and a red
         beacon over the drone so the overview shows where it is;
         default (auto): in the strait world only
demo     true flies the drone (circles; island to island in islands and strait, nodes/demo_flight.py), and opens
         the down-camera view (with the ships: the RF navigation display, nodes/aoa_map.py; the down camera is then a panel
         in the Gazebo window)
         and the navigation dashboard (nodes/nav_dashboard.py) on the desktop
monitor  with demo: window (default) opens the navigation dashboard; terminal opens the same information in a
         terminal (nodes/sensor_monitor.py)
gnss_cutoff_s     seconds after the first raw GPS fix when the GNSS gate closes (nodes/gnss_gate.py); negative: never
gnss_cut_s        legacy spelling for gnss_cutoff_s
record_mode       light records sensor and pose topics to outputs/sim_runs/<run>/rosbag2; full also the images;
                  off records no bag (the CSV logs are still written)
vision_rotation   the estimator (nodes/eskf_ros_adapter.py) fuses the forward camera's relative rotation
vision_direction  ... and its direction of travel
run_label         a name (letters, digits, _ and -): the run is written to outputs/sim_runs/velocity_phase/<name>

Every launch runs the ESKF estimator (GNSS + IMU + barometer + forward camera) on /nav/odom. With the ships, a
second instance also fuses the position from the ships' bearings (rf_fix:=true) on /nav_rf/odom, so the two can be
compared on the same flight after the GNSS cutoff; sim/scripts/check_rf_nav.py scores both against ground truth.
"""
import copy
import datetime as dt
import json
import math
import os
import re
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM = Path(__file__).resolve().parents[1]
GENERATED_MODEL = Path("/tmp/taipeidrift_midair_quad.sdf")
RF_CONFIG = SIM / "config/rf.yaml"
# Scenery each world needs, generated on first run
WORLD_ASSETS = {
    "terrain": ["make_ground.py", "make_trees.py"],  # replace the ground with the orthophoto via make_ground.py --aerial
    "islands": ["make_islands.py"],
    "strait": ["make_islands.py"],  # the islands scenery, with ships
    "city": ["make_city.py"],
}


WIND_FORCE_FACTOR = 0.15  # Gazebo pushes each link with this share of mass times the air's speed past it (WindEffects);
# 0.15 lets a hovering drone in a 6 m/s wind lean about 5 degrees, as a small quadcopter does


def windy_world(world: Path, speed_mps: float, from_deg: float) -> Path:
    """A copy of the world with wind: ``speed_mps`` from the bearing ``from_deg`` (20: from north-north-east),
    with gusts of about 20 percent in strength and 10 degrees in direction."""
    tree = ET.parse(world)
    w = tree.find("world")
    towards = math.radians(from_deg + 180.0)
    wind = ET.SubElement(w, "wind")
    ET.SubElement(wind, "linear_velocity").text = f"{speed_mps * math.sin(towards):.3f} {speed_mps * math.cos(towards):.3f} 0"
    plugin = ET.fromstring(f"""
      <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
        <force_approximation_scaling_factor>{WIND_FORCE_FACTOR}</force_approximation_scaling_factor>
        <horizontal>
          <magnitude>
            <time_for_rise>10</time_for_rise>
            <sin><amplitude_percent>0.2</amplitude_percent><period>15</period></sin>
            <noise type="gaussian"><mean>0</mean><stddev>0.05</stddev></noise>
          </magnitude>
          <direction>
            <time_for_rise>30</time_for_rise>
            <sin><amplitude>10</amplitude><period>30</period></sin>
            <noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise>
          </direction>
        </horizontal>
        <vertical><noise type="gaussian"><mean>0</mean><stddev>0.05</stddev></noise></vertical>
      </plugin>""")
    w.insert(0, plugin)
    out = Path(f"/tmp/taipeidrift_{world.stem}_wind.sdf")
    tree.write(out, xml_declaration=True, encoding="utf-8")
    return out


# The strait world's overview camera, shown as a panel in the Gazebo window (a gz topic, not bridged to ROS)
OVERVIEW_TOPIC = "/views/overview"
# Fixed camera high in the south, looking north over the triangle of ships and the drone's route
OVERVIEW_POSE = (425.0, -1700.0, 1100.0, 0.0, 0.53, 1.5708)


def df_antenna_sdf(rf_cfg: dict) -> list:
    """The direction finder, from config/rf.yaml, built like the pseudo-Doppler UAS payload of Gerhard and Tokekar
    (arXiv 2003.00386, figure 1): four fixed VHF stubs standing on the rotor arms at the corners of a square, each on
    an SMA mount with its coax running inboard, into an RF switch and a software-defined radio on the body (their
    HackRF One and Opera Cake). No moving parts. Returns visuals of base_link."""
    df = rf_cfg["direction_finder"]
    n, radius, length = df["elements"], df["array_radius_m"], df["element_length_m"]
    arm_top, top = 0.015, 0.02                    # top of the arms and of the body box, in base_link
    def colour(rgb):
        return f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>"
    black, grey, silver, board = (colour(c) for c in ("0.04 0.04 0.04", "0.3 0.3 0.32", "0.78 0.78 0.8",
                                                      "0.08 0.25 0.12"))
    def cyl(name, pose, r, h, mat):
        return (f'<visual name="{name}"><pose>{pose}</pose><geometry><cylinder><radius>{r}</radius>'
                f"<length>{h}</length></cylinder></geometry>{mat}</visual>")
    def box(name, pose, size, mat):
        return f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>{mat}</visual>'
    parts = [  # RF switch board under the radio, as the Opera Cake sits under the HackRF
        box("df_switch_board", f"0 0 {top + 0.003} 0 0 0", "0.085 0.06 0.006", board),
        box("df_radio", f"0 0 {top + 0.017} 0 0 0", "0.075 0.05 0.022", grey),
    ]
    stub = length - 0.03                          # the helical section; a thinner tip above it
    for k in range(n):
        yaw = math.pi / 4 + 2 * math.pi * k / n   # along the arms (the arms are at +-45 and +-135 deg)
        x, y = radius * math.cos(yaw), radius * math.sin(yaw)
        z = arm_top
        parts += [
            box(f"df_mount_{k}", f"{x} {y} {z + 0.003} 0 0 {yaw}", "0.024 0.024 0.006", grey),
            cyl(f"df_sma_{k}", f"{x} {y} {z + 0.012} 0 0 0", 0.005, 0.012, silver),
            cyl(f"df_stub_{k}", f"{x} {y} {z + 0.018 + stub / 2} 0 0 0", 0.0065, stub, black),
            cyl(f"df_tip_{k}", f"{x} {y} {z + 0.018 + stub + 0.0125} 0 0 0", 0.004, 0.025, black),
            # the coax from the mount inboard along the arm to the switch board
            cyl(f"df_coax_{k}", f"{x / 2} {y / 2} {z + 0.004} 0 1.5708 {yaw}", 0.002, radius, black),
        ]
    return [ET.fromstring(v) for v in parts]


def drone_sdf(cam_res: int, gps: bool, stereo: bool = False, stereo_baseline_m: float = 0.30, views: bool = False,
              wind: bool = False, rf_cfg: dict = None, range_min_m: float = 0.20, range_max_m: float = 100.0,
              range_noise_std_m: float = 0.02) -> Path:
    tree = ET.parse(SIM / "models/midair_quad/model.sdf")
    if wind:
        ET.SubElement(tree.find("model"), "enable_wind").text = "true"
    link = tree.find(".//link[@name='sensor_link']")
    image = link.find("sensor[@name='camera_down']/camera/image")
    forward_image = link.find("sensor[@name='camera_forward']/camera/image")
    image.find("width").text = image.find("height").text = str(cam_res)
    forward_image.find("width").text = forward_image.find("height").text = str(cam_res)
    range_sensor = link.find("sensor[@name='range_down']/lidar/range")
    range_sensor.find("min").text = str(range_min_m)
    range_sensor.find("max").text = str(range_max_m)
    link.find("sensor[@name='range_down']/lidar/noise/stddev").text = str(range_noise_std_m)
    if not gps:
        link.remove(link.find("sensor[@name='navsat']"))
    if stereo:
        if stereo_baseline_m <= 0:
            sys.exit("stereo_baseline_m must be > 0 when stereo:=true")
        right = copy.deepcopy(link.find("sensor[@name='camera_down']"))
        right.set("name", "camera_down_right")
        right.find("pose").text = f"0 {-stereo_baseline_m:.9g} 0 0 1.5708 0"
        right.find("topic").text = "camera/down_right/image_raw"
        right.find("gz_frame_id").text = "camera_down_right"
        link.append(right)
    if views:
        # A red ball high above the drone, visual only, so the drone can be found in the overview, which is
        # kilometres wide. The down camera looks the other way and never sees it.
        link.append(ET.fromstring(
            '<visual name="beacon"><pose>0 0 30 0 0 0</pose><geometry><sphere><radius>15</radius></sphere>'
            '</geometry><material><ambient>1 0.1 0.1 1</ambient><diffuse>1 0.1 0.1 1</diffuse>'
            '<emissive>0.6 0 0 1</emissive></material></visual>'))
    if rf_cfg:
        base = tree.find(".//link[@name='base_link']")
        for element in df_antenna_sdf(rf_cfg):
            base.append(element)
    tree.write(GENERATED_MODEL, xml_declaration=True, encoding="utf-8")
    return GENERATED_MODEL


def geodetic_origin(world_path: Path):
    coords = ET.parse(world_path).find(".//spherical_coordinates")
    if coords is None:
        return (0.0, 0.0, 0.0)
    return tuple(float(coords.findtext(tag, default="0")) for tag in
                 ("latitude_deg", "longitude_deg", "elevation"))


def overview_camera_sdf() -> Path:
    """A fixed camera over the strait world, for the overview panel."""
    path = Path("/tmp/taipeidrift_overview_camera.sdf")
    path.write_text(f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="overview_camera">
    <static>true</static>
    <link name="link">
      <sensor name="overview" type="camera">
        <topic>{OVERVIEW_TOPIC}</topic>
        <update_rate>5</update_rate>
        <always_on>true</always_on>
        <camera>
          <horizontal_fov>1.2</horizontal_fov>
          <image><width>800</width><height>450</height></image>
          <clip><near>1</near><far>20000</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
""")
    return path


def views_gui_config() -> Path:
    """The Gazebo window for the strait world: the 3rd-person chase view of the drone, with two panels beside it:
    the overview of the ships and the drone's down camera (the desktop's top-right slot holds
    the RF navigation display instead, nodes/aoa_map.py). The far clip is raised so ships kilometres away
    are drawn (the default cuts them off)."""
    text = (SIM / "config/gui.config").read_text()
    text = text.replace("<camera_pose>-6 0 6 0 0.5 0</camera_pose>",
                        "<camera_pose>-6 0 6 0 0.5 0</camera_pose>\n"
                        "  <camera_clip><near>0.25</near><far>30000</far></camera_clip>")
    panel = lambda title, topic: f"""
<plugin filename="ImageDisplay" name="{title}">
  <gz-gui>
    <title>{title}</title>
    <property type="bool" key="showTitleBar">true</property>
    <property type="string" key="state">docked</property>
  </gz-gui>
  <topic>{topic}</topic>
  <topic_picker>false</topic_picker>
</plugin>
"""
    text += panel("Overview: ships and drone (red ball)", OVERVIEW_TOPIC) + panel("Drone view: down camera",
                                                                                 "/camera/down/image_raw")
    path = Path("/tmp/taipeidrift_gui_views.config")
    path.write_text(text)
    return path


def ship_sdf(ship: dict) -> Path:
    """A warship drawn from boxes and cylinders, origin at the waterline, bow along +x: hull with a pointed bow,
    then a flight deck and an island (carrier) or a superstructure and a gun (destroyer, frigate), and the AIS mast
    at antenna_xy_m up to antenna_height_m. Visual only and without gravity: nodes/ship_traffic.py places it."""
    length, beam, top = ship["length_m"], ship["beam_m"], ship["antenna_height_m"]
    mx, my = ship.get("antenna_xy_m", [0.0, 0.0])
    grey, dark, deck = "0.48 0.51 0.54", "0.3 0.32 0.34", "0.22 0.23 0.25"

    def part(name, kind, size, x, y, z, rgb, yaw=0.0, pitch=0.0):
        geometry = (f"<box><size>{size}</size></box>" if kind == "box"
                    else f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>")
        return (f'<visual name="{name}"><pose>{x} {y} {z} 0 {pitch} {yaw}</pose><geometry>{geometry}</geometry>'
                f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>')

    if ship.get("kind") == "carrier":
        free = 18.0  # flight deck height above the sea
        hull_len = 0.9 * length
        parts = [
            part("hull", "box", f"{hull_len} {0.5 * beam} {free + 10}", -0.05 * length, 0, free / 2 - 5, grey),
            part("bow", "box", f"{0.354 * beam} {0.354 * beam} {free + 10}", 0.4 * length, 0, free / 2 - 5, grey,
                 0.785),
            part("flight_deck", "box", f"{length} {beam} 2", 0, 0, free + 1, deck),
            part("deck_line", "box", f"{0.9 * length} 1.5 0.1", 0, 0, free + 2.05, "0.9 0.9 0.9"),
            part("island", "box", f"{0.08 * length} {0.12 * beam} {top - free - 12}", mx, my,
                 free + 2 + (top - free - 12) / 2, grey),
        ]
        mast_base = top - 10
    else:
        free = 0.06 * length  # main deck height above the sea
        house_h = 0.09 * length
        parts = [
            part("hull", "box", f"{0.85 * length} {beam} {free + 5}", -0.075 * length, 0, free / 2 - 2.5, grey),
            part("bow", "box", f"{0.707 * beam} {0.707 * beam} {free + 5}", 0.35 * length, 0, free / 2 - 2.5, grey,
                 0.785),
            part("deck", "box", f"{0.85 * length} {0.95 * beam} 0.3", -0.075 * length, 0, free + 0.15, dark),
            part("superstructure", "box", f"{0.22 * length} {0.8 * beam} {house_h}", mx - 0.05 * length, 0,
                 free + house_h / 2, grey),
            part("hangar", "box", f"{0.12 * length} {0.7 * beam} {0.6 * house_h}", -0.3 * length, 0,
                 free + 0.3 * house_h, grey),
            part("gun", "cylinder", (0.12 * beam, 3.0), 0.25 * length, 0, free + 1.5, dark),
            part("barrel", "cylinder", (0.4, 0.06 * length), 0.28 * length, 0, free + 2.2, dark, pitch=1.5708),
        ]
        mast_base = free + house_h
    parts.append(part("mast", "cylinder", (0.6, top - mast_base), mx, my, (top + mast_base) / 2, dark))
    parts.append(part("ais_antenna", "cylinder", (0.1, 3.0), mx, my, top + 1.5, "0.95 0.95 0.95"))

    path = Path(f"/tmp/taipeidrift_ship_{ship['name']}.sdf")
    path.write_text(f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{ship['name']}">
    <link name="hull">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial><mass>1000</mass></inertial>
      {''.join(parts)}
    </link>
  </model>
</sdf>
""")
    return path


def already_running() -> bool:
    """True if a Gazebo server is already running in this container."""
    return subprocess.run(["pgrep", "-f", "^gz sim server"], capture_output=True).returncode == 0


def wait_for_display(timeout_s: float = 60.0) -> None:
    """Right after the container starts, its virtual display needs a few seconds to come up."""
    display = os.environ.get("DISPLAY", ":1")
    path = f"/tmp/.X11-unix/X{display.lstrip(':').split('.')[0]}"
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            with socket.socket(socket.AF_UNIX) as s:
                s.connect(path)  # the X server accepts connections: ready
            time.sleep(2)  # give the window manager a moment too
            return
        except OSError:
            time.sleep(1)
    sys.exit(f"No display {display} after {timeout_s:.0f} s. Restart the container: docker compose restart")


def setup(context):
    if already_running():
        sys.exit("A simulation is already running in this container. Stop it first with:\n"
                 "    docker compose restart")
    cam_res = int(LaunchConfiguration("cam_res").perform(context))
    gps = LaunchConfiguration("gps").perform(context).lower() == "true"
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world = SIM / "worlds" / f"{LaunchConfiguration('world').perform(context)}.sdf"
    demo = LaunchConfiguration("demo").perform(context).lower() == "true"
    monitor = LaunchConfiguration("monitor").perform(context).lower()
    if monitor not in ("window", "terminal"):
        sys.exit("monitor must be window or terminal")
    stereo = LaunchConfiguration("stereo").perform(context).lower() == "true"
    stereo_baseline_m = float(LaunchConfiguration("stereo_baseline_m").perform(context))
    ships = LaunchConfiguration("ships").perform(context).lower()
    ships = world.stem == "strait" if ships == "auto" else ships == "true"
    legacy_cut = LaunchConfiguration("gnss_cut_s").perform(context)
    gnss_cut_s = float(legacy_cut if legacy_cut != "unset" else LaunchConfiguration("gnss_cutoff_s").perform(context))
    record_mode = LaunchConfiguration("record_mode").perform(context).lower()
    vision_rotation = LaunchConfiguration("vision_rotation").perform(context).lower()
    vision_direction = LaunchConfiguration("vision_direction").perform(context).lower()
    run_label = LaunchConfiguration("run_label").perform(context).strip()
    if run_label and not re.fullmatch(r"[A-Za-z0-9_-]+", run_label):
        sys.exit("run_label may contain only letters, numbers, underscores, and hyphens")
    metric_flow = LaunchConfiguration("metric_flow").perform(context).lower() == "true"
    range_min_m = float(LaunchConfiguration("range_min_m").perform(context))
    range_max_m = float(LaunchConfiguration("range_max_m").perform(context))
    range_noise_std_m = float(LaunchConfiguration("range_noise_std_m").perform(context))
    flow_update_every_n = max(1, int(LaunchConfiguration("flow_update_every_n").perform(context)))
    if not (0 < range_min_m < range_max_m and range_noise_std_m >= 0):
        sys.exit("range_min_m/range_max_m/noise must satisfy 0 < min < max and noise >= 0")
    if not world.exists():
        sys.exit(f"No world {world}. Choose one of: {', '.join(sorted(w.stem for w in world.parent.glob('*.sdf')))}")
    if record_mode not in ("off", "light", "full"):
        sys.exit("record_mode must be off, light or full")
    gps_origin = geodetic_origin(world)
    if gui or demo:
        wait_for_display()

    for script in WORLD_ASSETS.get(world.stem, []):
        subprocess.run([sys.executable, str(SIM / "scripts" / script), "--if-missing"], check=True)
    wind = LaunchConfiguration("wind").perform(context).strip().lower()
    world_file = world
    if wind not in ("", "none"):
        try:
            speed, from_deg = (float(v) for v in wind.split(","))
        except ValueError:
            sys.exit(f"wind must be SPEED_MPS,FROM_DEG (for example 6,20), not {wind!r}")
        world_file = windy_world(world, speed, from_deg)
    model = drone_sdf(cam_res, gps, stereo, stereo_baseline_m, views=ships, wind=world_file != world,
                      rf_cfg=yaml.safe_load(RF_CONFIG.read_text()) if ships else None,
                      range_min_m=range_min_m, range_max_m=range_max_m, range_noise_std_m=range_noise_std_m)
    gui_config = views_gui_config() if ships else SIM / "config/gui.config"
    sim_time = {"use_sim_time": True}

    run_id = dt.datetime.now().strftime(f"{world.stem}_%Y%m%d_%H%M%S")
    output_phase = "metric_velocity" if metric_flow or run_label.startswith("OF") else "velocity_phase"
    run_dir = (SIM.parent / "outputs" / "sim_runs" / output_phase / run_label
               if run_label else SIM.parent / "outputs" / "sim_runs" / run_id)
    run_dir.mkdir(parents=True, exist_ok=False)
    try:
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=SIM.parent,
                                capture_output=True, text=True, check=True).stdout.strip()
        working_tree_dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=SIM.parent,
                                                 capture_output=True, text=True, check=True).stdout.strip())
    except Exception:
        commit = "unknown"
        working_tree_dirty = None
    # the vision thresholds the estimator runs with: its tracker re-anchors at the pose estimator's minimum
    eskf_cfg = yaml.safe_load((SIM.parent / "vio/configs/midair_eskf.yaml").read_text())
    pose_cfg = yaml.safe_load((SIM.parent / eskf_cfg["vio_config"]).read_text())["pose"]
    gnss_cfg = eskf_cfg["sim_gnss"]
    velocity_fit_config = {"window_s": gnss_cfg["velocity_window_s"], "min_samples": gnss_cfg["velocity_min_samples"],
                           "min_span_s": gnss_cfg["velocity_min_span_s"],
                           "method": "robust generalized least squares; non-overlapping windows"}
    metadata = {
        "run_id": run_label or run_id, "world": world.stem, "gnss_cutoff_s_since_first_fix": gnss_cut_s,
        "camera_resolution": cam_res, "demo_trajectory": "city_loop" if world.stem == "city" else "default_world_route",
        "estimator_config": "vio/configs/midair_eskf.yaml; simulated GNSS + IMU + barometer; vision flags recorded below",
        "git_commit": commit, "working_tree_dirty": working_tree_dirty,
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "run_created_local": dt.datetime.now().astimezone().isoformat(), "record_mode": record_mode,
        "topics": {"gt": "/ground_truth/odom", "raw_gps": "/sim/gps_raw", "gated_gps": "/gps/fix",
                   "gnss_status": "/nav/gnss_available", "imu": "/imu/data", "barometer": "/air_pressure",
                   "camera_forward": "/camera/forward/image_raw", "camera_down": "/camera/down/image_raw",
                   "estimator": "/nav/odom", "estimator_status": "/nav/estimator_status",
                   "tf": "/tf", "tf_static": "/tf_static"},
        "frames": {"gazebo_world": "ENU", "body": "FLU", "camera_forward": "optical frame (+Z forward)",
                   "camera_down": "optical frame (+Z down)", "gps": "WGS84 latitude/longitude/altitude",
                   "estimator": "local ENU position/velocity; FLU body orientation from IMU gravity and yaw=0; GNSS position updates"},
        "vision_rotation": vision_rotation == "true", "vision_direction": vision_direction == "true",
        "sim_tracker_min_tracks": pose_cfg["min_correspondences"],
        "pose_min_correspondences": pose_cfg["min_correspondences"], "pose_min_inliers": pose_cfg["min_inliers"],
        "gnss_velocity_fit": velocity_fit_config,
        "metric_flow": {"enabled": metric_flow, "algorithm": "Shi-Tomasi/LK + homography RANSAC + ESKF-attitude derotation",
                        "range_min_m": range_min_m, "range_max_m": range_max_m,
                        "range_noise_std_m": range_noise_std_m, "range_topic": "/range/down",
                        "update_every_n_images": flow_update_every_n,
                        "update_rate_qualifier": "consecutive image-pair temporal correlation decimation"},
        "gnss_local_enu_origin": {"latitude_deg": gps_origin[0], "longitude_deg": gps_origin[1],
                                   "elevation_m": gps_origin[2]},
    }
    if ships:
        metadata["topics"].update({"rf_detections": "/rf/detections", "rf_nav": "/rf_nav/odom",
                                   "estimator_rf": "/nav_rf/odom", "estimator_rf_status": "/nav_rf/estimator_status"})
        metadata["frames"]["estimator_rf"] = ("as estimator, plus horizontal position and heading updates from "
                                              "/rf_nav/odom (settings: config/rf.yaml, eskf_rf_fix)")
        metadata["eskf_rf_fix"] = yaml.safe_load(RF_CONFIG.read_text())["eskf_rf_fix"]
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    (run_dir / "config.json").write_text(json.dumps({"world": world.stem, "demo": demo, "cam_res": cam_res,
        "gnss_cutoff_s_since_first_fix": gnss_cut_s, "record_mode": record_mode, "ships": ships,
        "vision_rotation": vision_rotation == "true", "vision_direction": vision_direction == "true",
        "metric_flow": metric_flow, "range_min_m": range_min_m, "range_max_m": range_max_m,
        "range_noise_std_m": range_noise_std_m,
        "flow_update_every_n": flow_update_every_n,
        "gnss_velocity_fit": velocity_fit_config}, indent=2), encoding="utf-8")
    (run_dir / "sim.log").write_text(f"Run {run_id}; launch logs are emitted by ros2 launch.\n", encoding="utf-8")
    bag_topics = ["/ground_truth/odom", "/sim/gps_raw", "/gps/fix", "/nav/gnss_available",
                  "/sim/imu_raw", "/imu/data", "/sim/air_pressure_raw", "/air_pressure", "/tf", "/tf_static",
                  "/camera/down/camera_info", "/camera/forward/camera_info", "/nav/odom", "/nav/estimator_status"]
    bag_topics += ["/range/down"]
    if ships:
        bag_topics += ["/rf/detections", "/rf_nav/odom", "/nav_rf/odom", "/nav_rf/estimator_status"]
    if record_mode == "full":
        bag_topics += ["/camera/down/image_raw", "/camera/forward/image_raw"]
    bag_args = ["ros2", "bag", "record", "--storage", "mcap", "-o", str(run_dir / "rosbag2"),
                "--topics", *bag_topics]
    estimator = [sys.executable, str(SIM / "nodes/eskf_ros_adapter.py"),
                 "--ros-args", "-p", f"vision_rotation:={vision_rotation}",
                 "-p", f"vision_direction:={vision_direction}",
                 "-p", f"metric_flow:={str(metric_flow).lower()}",
                 "-p", f"flow_range_std_m:={range_noise_std_m}",
                 "-p", f"flow_update_every_n:={flow_update_every_n}",
                 "-p", f"gps_origin_latitude:={gps_origin[0]}",
                 "-p", f"gps_origin_longitude:={gps_origin[1]}",
                 "-p", f"gps_origin_elevation:={gps_origin[2]}"]

    actions = [
        ExecuteProcess(output="screen", cmd=["gz", "sim", "-r", str(world_file), *(
            ["--gui-config", str(gui_config)] if gui else ["-s"])]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", world.stem, "-file", str(model), "-name", "midair_quad", "-z", "0.05"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(SIM / "config/bridge.yaml")}, sim_time]),
        Node(package="ros_gz_image", executable="image_bridge", output="screen",
             arguments=["/camera/down/image_raw"], parameters=[sim_time]),
        Node(package="ros_gz_image", executable="image_bridge", output="screen",
             arguments=["/camera/forward/image_raw"], parameters=[sim_time]),
        ExecuteProcess(output="screen", cmd=[
            sys.executable, str(SIM / "nodes/sensor_noise.py"), "--ros-args",
            "--params-file", str(SIM / "config/sensor_noise.yaml"), "-p", "use_sim_time:=true"]),
        ExecuteProcess(output="screen", cmd=[
            sys.executable, str(SIM / "nodes/gnss_gate.py"), "--ros-args",
            "-p", f"gnss_cut_s:={gnss_cut_s}", "-p", "use_sim_time:=true"]),
        *([ExecuteProcess(output="screen", cmd=bag_args)] if record_mode != "off" else []),
        ExecuteProcess(output="screen", cmd=[sys.executable, str(SIM / "nodes/run_logger.py"), "--out", str(run_dir)]),
        ExecuteProcess(output="screen", cmd=estimator),
    ]
    if stereo:
        actions.append(Node(package="ros_gz_image", executable="image_bridge", output="screen",
                            arguments=["/camera/down_right/image_raw"], parameters=[sim_time]))
    if ships:
        rf = yaml.safe_load(RF_CONFIG.read_text())
        actions += [
            Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", world.stem, "-file", str(ship_sdf(ship)), "-name", ship["name"],
                            "-x", str(ship["waypoints"][0][0]), "-y", str(ship["waypoints"][0][1]),
                            "-z", str(rf["sea_level_z"])])
            for ship in rf["ships"]
        ]
        x, y, z, roll, pitch, yaw = OVERVIEW_POSE
        actions.append(Node(package="ros_gz_sim", executable="create", output="screen", arguments=[
            "-world", world.stem, "-file", str(overview_camera_sdf()), "-name", "overview_camera",
            "-x", str(x), "-y", str(y), "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)]))
        actions += [
            # set_pose's name holds the world, so it is bridged here rather than in config/bridge.yaml
            Node(package="ros_gz_bridge", executable="parameter_bridge", name="set_pose_bridge", output="screen",
                 arguments=[f"/world/{world.stem}/set_pose@ros_gz_interfaces/srv/SetEntityPose"]),
            ExecuteProcess(output="screen", cmd=[sys.executable, str(SIM / "nodes/ship_traffic.py"),
                                                 "--world", world.stem, "--config", str(RF_CONFIG)]),
            ExecuteProcess(output="screen", cmd=[sys.executable, str(SIM / "nodes/rf_sensor.py"),
                                                 "--world", world.stem, "--config", str(RF_CONFIG)]),
            # the drone's position from the ships' bearings, without GNSS
            ExecuteProcess(output="screen", cmd=[sys.executable, str(SIM / "nodes/rf_nav.py"), "--world", world.stem]),
            # the same estimator, also fusing the ships' position fix: keeps navigating after the GNSS cutoff
            ExecuteProcess(output="screen", cmd=[*estimator, "-p", "rf_fix:=true", "-p", "publish_tf:=false",
                                                 "-r", "__node:=eskf_rf_adapter", "-r", "/nav/odom:=/nav_rf/odom",
                                                 "-r", "/nav/estimator_status:=/nav_rf/estimator_status"]),
        ]
    if demo:
        actions += [
            # Give the Gazebo window time to open before asking it to follow the drone
            TimerAction(period=15.0, actions=[
                ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/demo_flight.py"), "--world", world.stem,
                                    *(["--route", "city_loop", "--height", "80"] if world.stem == "city" else [])],
                               output="screen")]),
            # Started late: opened before the camera topic exists, the viewer can stay blank. With the ships, the
            # down camera is a panel in the Gazebo window and this slot shows the RF navigation display.
            TimerAction(period=20.0, actions=[
                ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/aoa_map.py"), "--world", world.stem],
                               output="screen", respawn=True, respawn_delay=2.0) if ships else
                Node(package="rqt_image_view", executable="rqt_image_view", arguments=["/camera/down/image_raw"])]),
            # the lower-right corner of the 1920 x 1080 desktop, under the RF navigation display: the navigation
            # dashboard window (monitor:=window) or the same information in a terminal (-0-0: right, bottom)
            ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/nav_dashboard.py"), "--world", world.stem],
                           output="screen", respawn=True, respawn_delay=2.0) if monitor == "window" else
            ExecuteProcess(cmd=["xterm", "-T", "Sensor monitor", "-geometry", "118x42-0-0", "-fa", "Monospace", "-fs", "8",
                                "-bg", "black", "-fg", "white", "-e", sys.executable,
                                str(SIM / "nodes/sensor_monitor.py"), "--world", world.stem]),
        ]
    return actions


def generate_launch_description():
    resource_path = os.pathsep.join(filter(None, [str(SIM / "models"), os.environ.get("GZ_SIM_RESOURCE_PATH")]))
    return LaunchDescription([
        DeclareLaunchArgument("cam_res", default_value="1024"),
        DeclareLaunchArgument("gps", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value="terrain"),
        DeclareLaunchArgument("demo", default_value="false"),
        DeclareLaunchArgument("monitor", default_value="window", description="with demo: the dashboard window, or terminal"),
        DeclareLaunchArgument("stereo", default_value="false"),
        DeclareLaunchArgument("stereo_baseline_m", default_value="0.30"),
        DeclareLaunchArgument("wind", default_value="none"),
        DeclareLaunchArgument("ships", default_value="auto"),
        DeclareLaunchArgument("gnss_cut_s", default_value="unset", description="Legacy spelling; cutoff is relative to first GPS fix"),
        DeclareLaunchArgument("gnss_cutoff_s", default_value="20.0"),
        DeclareLaunchArgument("record_mode", default_value="light"),
        DeclareLaunchArgument("vision_rotation", default_value="true"),
        DeclareLaunchArgument("vision_direction", default_value="true"),
        DeclareLaunchArgument("run_label", default_value=""),
        DeclareLaunchArgument("metric_flow", default_value="false"),
        DeclareLaunchArgument("range_min_m", default_value="0.20"),
        DeclareLaunchArgument("range_max_m", default_value="100.0"),
        DeclareLaunchArgument("range_noise_std_m", default_value="0.02"),
        DeclareLaunchArgument("flow_update_every_n", default_value="5"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=setup),
    ])
