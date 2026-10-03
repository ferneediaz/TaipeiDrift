"""Start Gazebo with the Mid-Air-like drone, the ROS bridge and the sensor noise node.

    ros2 launch sim/launch/sim.launch.py [cam_res:=1024] [gps:=true] [gui:=true] [world:=terrain]
        [demo:=false] [gnss_cutoff_s:=20.0] [record_mode:=light] [stereo:=false]

gnss_cutoff_s       seconds after the first raw GPS fix; negative keeps GNSS enabled
gnss_cut_s          legacy spelling for gnss_cutoff_s
stereo              true adds a right down-camera by generating a temporary model variant
stereo_baseline_m   right camera offset along body -Y, in metres
"""
import copy
import datetime as dt
import json
import os
import re
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM = Path(__file__).resolve().parents[1]
GENERATED_MODEL = Path("/tmp/taipeidrift_midair_quad.sdf")
# Scenery each world needs, generated on first run
WORLD_ASSETS = {
    "terrain": ["make_ground.py", "make_trees.py"],  # replace the ground with the orthophoto via make_ground.py --aerial
    "islands": ["make_islands.py"],
    "city": ["make_city.py"],
}


def drone_sdf(cam_res: int, gps: bool, stereo: bool, stereo_baseline_m: float) -> Path:
    tree = ET.parse(SIM / "models/midair_quad/model.sdf")
    link = tree.find(".//link[@name='sensor_link']")
    image = link.find("sensor[@name='camera_down']/camera/image")
    forward_image = link.find("sensor[@name='camera_forward']/camera/image")
    image.find("width").text = image.find("height").text = str(cam_res)
    forward_image.find("width").text = forward_image.find("height").text = str(cam_res)
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
    tree.write(GENERATED_MODEL, xml_declaration=True, encoding="utf-8")
    return GENERATED_MODEL


def already_running() -> bool:
    """True if a Gazebo server is already running in this container."""
    return subprocess.run(["pgrep", "-f", "^gz sim server"], capture_output=True).returncode == 0


def geodetic_origin(world_path: Path):
    coords = ET.parse(world_path).find(".//spherical_coordinates")
    if coords is None:
        return (0.0, 0.0, 0.0)
    return tuple(float(coords.findtext(tag, default="0")) for tag in
                 ("latitude_deg", "longitude_deg", "elevation"))


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
    gps_origin = geodetic_origin(world)
    demo = LaunchConfiguration("demo").perform(context).lower() == "true"
    legacy_cut = LaunchConfiguration("gnss_cut_s").perform(context)
    gnss_cut_s = float(legacy_cut if legacy_cut != "unset" else LaunchConfiguration("gnss_cutoff_s").perform(context))
    record_mode = LaunchConfiguration("record_mode").perform(context).lower()
    vision_rotation = LaunchConfiguration("vision_rotation").perform(context).lower()
    vision_direction = LaunchConfiguration("vision_direction").perform(context).lower()
    run_label = LaunchConfiguration("run_label").perform(context).strip()
    if run_label and not re.fullmatch(r"[A-Za-z0-9_-]+", run_label):
        sys.exit("run_label may contain only letters, numbers, underscores, and hyphens")
    stereo = LaunchConfiguration("stereo").perform(context).lower() == "true"
    stereo_baseline_m = float(LaunchConfiguration("stereo_baseline_m").perform(context))
    if not world.exists():
        sys.exit(f"No world {world}. Choose one of: {', '.join(sorted(w.stem for w in world.parent.glob('*.sdf')))}")
    if gui or demo:
        wait_for_display()

    for script in WORLD_ASSETS.get(world.stem, []):
        subprocess.run([sys.executable, str(SIM / "scripts" / script), "--if-missing"], check=True)
    model = drone_sdf(cam_res, gps, stereo, stereo_baseline_m)
    sim_time = {"use_sim_time": True}

    run_id = dt.datetime.now().strftime(f"{world.stem}_%Y%m%d_%H%M%S")
    run_dir = (SIM.parent / "outputs" / "sim_runs" / "velocity_phase" / run_label
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
    velocity_fit_config = {"window_s": 8.0, "min_samples": 6, "min_span_s": 6.0,
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
        "sim_tracker_min_tracks": 30, "pose_min_correspondences": 30, "pose_min_inliers": 30,
        "gnss_velocity_fit": velocity_fit_config,
        "gnss_local_enu_origin": {"latitude_deg": gps_origin[0], "longitude_deg": gps_origin[1],
                                   "elevation_m": gps_origin[2]},
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    (run_dir / "config.json").write_text(json.dumps({"world": world.stem, "demo": demo, "cam_res": cam_res,
        "gnss_cutoff_s_since_first_fix": gnss_cut_s, "record_mode": record_mode,
        "vision_rotation": vision_rotation == "true", "vision_direction": vision_direction == "true",
        "gnss_velocity_fit": velocity_fit_config}, indent=2), encoding="utf-8")
    (run_dir / "sim.log").write_text(f"Run {run_id}; launch logs are emitted by ros2 launch.\n", encoding="utf-8")
    if record_mode not in ("light", "full"):
        sys.exit("record_mode must be light or full")
    bag_topics = ["/ground_truth/odom", "/sim/gps_raw", "/gps/fix", "/nav/gnss_available",
                  "/sim/imu_raw", "/imu/data", "/sim/air_pressure_raw", "/air_pressure", "/tf", "/tf_static",
                  "/camera/down/camera_info", "/camera/forward/camera_info", "/nav/odom", "/nav/estimator_status"]
    if record_mode == "full":
        bag_topics += ["/camera/down/image_raw", "/camera/forward/image_raw"]
    bag_args = ["ros2", "bag", "record", "--storage", "mcap", "-o", str(run_dir / "rosbag2"),
                "--topics", *bag_topics]

    actions = [
        ExecuteProcess(output="screen", cmd=["gz", "sim", "-r", str(world), *(
            ["--gui-config", str(SIM / "config/gui.config")] if gui else ["-s"])]),
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
    ]
    actions.append(ExecuteProcess(output="screen", cmd=[
        sys.executable, str(SIM / "nodes/gnss_gate.py"), "--ros-args",
        "-p", f"gnss_cut_s:={gnss_cut_s}", "-p", "use_sim_time:=true",
    ]))
    actions.extend([
        ExecuteProcess(output="screen", cmd=bag_args),
        ExecuteProcess(output="screen", cmd=[sys.executable, str(SIM / "nodes/run_logger.py"),
                                               "--out", str(run_dir)]),
        ExecuteProcess(output="screen", cmd=[sys.executable, str(SIM / "nodes/eskf_ros_adapter.py"),
                                               "--ros-args", "-p", f"vision_rotation:={vision_rotation}",
                                               "-p", f"vision_direction:={vision_direction}",
                                               "-p", f"gps_origin_latitude:={gps_origin[0]}",
                                               "-p", f"gps_origin_longitude:={gps_origin[1]}",
                                               "-p", f"gps_origin_elevation:={gps_origin[2]}"]),
    ])
    if stereo:
        actions.append(Node(package="ros_gz_image", executable="image_bridge", output="screen",
                            arguments=["/camera/down_right/image_raw"], parameters=[sim_time]))
    if demo:
        actions += [
            # Give the Gazebo window time to open before asking it to follow the drone
            TimerAction(period=15.0, actions=[
                ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/demo_flight.py"), "--world", world.stem,
                                    "--route", "city_loop" if world.stem == "city" else "pads",
                                    "--height", "80" if world.stem == "city" else "40"],
                               output="screen")]),
            # Started late: opened before the camera topic exists, the viewer can stay blank
            TimerAction(period=20.0, actions=[
                Node(package="rqt_image_view", executable="rqt_image_view", arguments=["/camera/down/image_raw"])]),
            ExecuteProcess(cmd=["xterm", "-T", "Sensor monitor", "-fa", "Monospace", "-fs", "9",
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
        DeclareLaunchArgument("gnss_cut_s", default_value="unset", description="Legacy spelling; cutoff is relative to first GPS fix"),
        DeclareLaunchArgument("gnss_cutoff_s", default_value="20.0"),
        DeclareLaunchArgument("record_mode", default_value="light"),
        DeclareLaunchArgument("run_label", default_value=""),
        DeclareLaunchArgument("vision_rotation", default_value="true"),
        DeclareLaunchArgument("vision_direction", default_value="true"),
        DeclareLaunchArgument("stereo", default_value="false"),
        DeclareLaunchArgument("stereo_baseline_m", default_value="0.30"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=setup),
    ])
