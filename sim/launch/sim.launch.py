"""Start Gazebo with the Mid-Air-like drone, the ROS bridge and the sensor noise node.

    ros2 launch sim/launch/sim.launch.py [cam_res:=1024] [gps:=true] [gui:=true] [world:=terrain] [demo:=false]

cam_res  down camera width and height in pixels; 1024 matches Mid-Air, 512 renders faster on a CPU
gps      false removes the GNSS receiver from the drone
gui      false runs Gazebo without its window (server only)
world    a file name in sim/worlds/ without .sdf: terrain (fields and woods) or islands (two islands and open sea)
demo     true flies the drone (circles; island to island in the islands world, nodes/demo_flight.py), and opens
         the down-camera view
         and a terminal with the live sensor monitor (nodes/sensor_monitor.py) on the desktop
"""
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


def drone_sdf(cam_res: int, gps: bool) -> Path:
    tree = ET.parse(SIM / "models/midair_quad/model.sdf")
    link = tree.find(".//link[@name='sensor_link']")
    image = link.find("sensor[@name='camera_down']/camera/image")
    image.find("width").text = image.find("height").text = str(cam_res)
    if not gps:
        link.remove(link.find("sensor[@name='navsat']"))
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
    if not world.exists():
        sys.exit(f"No world {world}. Choose one of: {', '.join(sorted(w.stem for w in world.parent.glob('*.sdf')))}")
    if gui or demo:
        wait_for_display()

    for script in WORLD_ASSETS.get(world.stem, []):
        subprocess.run([sys.executable, str(SIM / "scripts" / script), "--if-missing"], check=True)
    model = drone_sdf(cam_res, gps)
    sim_time = {"use_sim_time": True}

    actions = [
        ExecuteProcess(output="screen", cmd=["gz", "sim", "-r", str(world), *(
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
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=setup),
    ])
