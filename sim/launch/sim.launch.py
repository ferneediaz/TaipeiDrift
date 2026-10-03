"""Start Gazebo with the Mid-Air-like drone, the ROS bridge and the sensor noise node.

    ros2 launch sim/launch/sim.launch.py [cam_res:=1024] [gps:=true] [gui:=true] [world:=terrain]
        [demo:=false] [gnss_cut_s:=-1] [stereo:=false] [stereo_baseline_m:=0.30] [wind:=none]

gnss_cut_s          absolute simulation time in seconds; negative keeps GNSS enabled
stereo              true adds a right down-camera by generating a temporary model variant
stereo_baseline_m   right camera offset along body -Y, in metres
wind                SPEED_MPS,FROM_DEG, for example 6,20: 6 m/s from the north-north-east, with gusts
"""
import copy
import math
import os
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


def drone_sdf(cam_res: int, gps: bool, stereo: bool, stereo_baseline_m: float, wind: bool = False) -> Path:
    tree = ET.parse(SIM / "models/midair_quad/model.sdf")
    if wind:
        ET.SubElement(tree.find("model"), "enable_wind").text = "true"
    link = tree.find(".//link[@name='sensor_link']")
    image = link.find("sensor[@name='camera_down']/camera/image")
    image.find("width").text = image.find("height").text = str(cam_res)
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
    gnss_cut_s = float(LaunchConfiguration("gnss_cut_s").perform(context))
    stereo = LaunchConfiguration("stereo").perform(context).lower() == "true"
    stereo_baseline_m = float(LaunchConfiguration("stereo_baseline_m").perform(context))
    if not world.exists():
        sys.exit(f"No world {world}. Choose one of: {', '.join(sorted(w.stem for w in world.parent.glob('*.sdf')))}")
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
    model = drone_sdf(cam_res, gps, stereo, stereo_baseline_m, wind=world_file != world)
    sim_time = {"use_sim_time": True}

    actions = [
        ExecuteProcess(output="screen", cmd=["gz", "sim", "-r", str(world_file), *(
            ["--gui-config", str(SIM / "config/gui.config")] if gui else ["-s"])]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", world.stem, "-file", str(model), "-name", "midair_quad", "-z", "0.05"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(SIM / "config/bridge.yaml")}, sim_time]),
        Node(package="ros_gz_image", executable="image_bridge", output="screen",
             arguments=["/camera/down/image_raw"], parameters=[sim_time]),
        ExecuteProcess(output="screen", cmd=[
            sys.executable, str(SIM / "nodes/sensor_noise.py"), "--ros-args",
            "--params-file", str(SIM / "config/sensor_noise.yaml"), "-p", "use_sim_time:=true"]),
    ]
    actions.append(ExecuteProcess(output="screen", cmd=[
        sys.executable, str(SIM / "nodes/gnss_gate.py"), "--ros-args",
        "-p", f"gnss_cut_s:={gnss_cut_s}", "-p", "use_sim_time:=true",
    ]))
    if stereo:
        actions.append(Node(package="ros_gz_image", executable="image_bridge", output="screen",
                            arguments=["/camera/down_right/image_raw"], parameters=[sim_time]))
    if demo:
        actions += [
            # Give the Gazebo window time to open before asking it to follow the drone
            TimerAction(period=15.0, actions=[
                ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/demo_flight.py"), "--world", world.stem],
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
        DeclareLaunchArgument("gnss_cut_s", default_value="-1"),
        DeclareLaunchArgument("stereo", default_value="false"),
        DeclareLaunchArgument("stereo_baseline_m", default_value="0.30"),
        DeclareLaunchArgument("wind", default_value="none"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=setup),
    ])
