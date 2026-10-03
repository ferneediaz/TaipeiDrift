"""Start Gazebo with the Mid-Air-like drone, the ROS bridge and the sensor noise node.

    ros2 launch sim/launch/sim.launch.py [cam_res:=1024] [gps:=true] [gui:=true] [world:=terrain] [demo:=false]
                                         [ships:=auto]

cam_res  down camera width and height in pixels; 1024 matches Mid-Air, 512 renders faster on a CPU
gps      false removes the GNSS receiver from the drone
gui      false runs Gazebo without its window (server only)
world    a file name in sim/worlds/ without .sdf: terrain (fields and woods), islands (two islands and open sea)
         or strait (the islands with warships that transmit AIS)
ships    true sails the AIS-transmitting ships of config/rf.yaml (nodes/ship_traffic.py), runs the drone's
         AIS receiver and direction finder (nodes/rf_sensor.py) and the triangulation navigator (nodes/rf_nav.py),
         adds a fixed overview camera, shown as a panel beside the chase view in the Gazebo window, and a red
         beacon over the drone so the overview shows where it is;
         default (auto): in the strait world only
demo     true flies the drone (circles; island to island in islands and strait, nodes/demo_flight.py), and opens
         the down-camera view (with the ships: the RSSI map, nodes/rssi_map.py; the down camera is then a panel
         in the Gazebo window)
         and a terminal with the live sensor monitor (nodes/sensor_monitor.py) on the desktop
"""
import os
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
}


# The strait world's overview camera, shown as a panel in the Gazebo window (a gz topic, not bridged to ROS)
OVERVIEW_TOPIC = "/views/overview"
# Fixed camera high in the south, looking north over the triangle of ships and the drone's route
OVERVIEW_POSE = (425.0, -1700.0, 1100.0, 0.0, 0.53, 1.5708)


def drone_sdf(cam_res: int, gps: bool, views: bool = False) -> Path:
    tree = ET.parse(SIM / "models/midair_quad/model.sdf")
    link = tree.find(".//link[@name='sensor_link']")
    image = link.find("sensor[@name='camera_down']/camera/image")
    image.find("width").text = image.find("height").text = str(cam_res)
    if not gps:
        link.remove(link.find("sensor[@name='navsat']"))
    if views:
        # A red ball high above the drone, visual only, so the drone can be found in the overview, which is
        # kilometres wide. The down camera looks the other way and never sees it.
        link.append(ET.fromstring(
            '<visual name="beacon"><pose>0 0 30 0 0 0</pose><geometry><sphere><radius>15</radius></sphere>'
            '</geometry><material><ambient>1 0.1 0.1 1</ambient><diffuse>1 0.1 0.1 1</diffuse>'
            '<emissive>0.6 0 0 1</emissive></material></visual>'))
    tree.write(GENERATED_MODEL, xml_declaration=True, encoding="utf-8")
    return GENERATED_MODEL


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
    the RSSI map instead, nodes/rssi_map.py). The far clip is raised so ships kilometres away
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
    ships = LaunchConfiguration("ships").perform(context).lower()
    ships = world.stem == "strait" if ships == "auto" else ships == "true"
    if not world.exists():
        sys.exit(f"No world {world}. Choose one of: {', '.join(sorted(w.stem for w in world.parent.glob('*.sdf')))}")
    if gui or demo:
        wait_for_display()

    for script in WORLD_ASSETS.get(world.stem, []):
        subprocess.run([sys.executable, str(SIM / "scripts" / script), "--if-missing"], check=True)
    model = drone_sdf(cam_res, gps, views=ships)
    gui_config = views_gui_config() if ships else SIM / "config/gui.config"
    sim_time = {"use_sim_time": True}

    actions = [
        ExecuteProcess(output="screen", cmd=["gz", "sim", "-r", str(world), *(
            ["--gui-config", str(gui_config)] if gui else ["-s"])]),
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
        ]
    if demo:
        actions += [
            # Give the Gazebo window time to open before asking it to follow the drone
            TimerAction(period=15.0, actions=[
                ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/demo_flight.py"), "--world", world.stem],
                               output="screen")]),
            # Started late: opened before the camera topic exists, the viewer can stay blank. With the ships, the
            # down camera is a panel in the Gazebo window and this slot shows the RSSI map.
            TimerAction(period=20.0, actions=[
                ExecuteProcess(cmd=[sys.executable, str(SIM / "nodes/rssi_map.py"), "--world", world.stem],
                               output="screen") if ships else
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
        DeclareLaunchArgument("ships", default_value="auto"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        OpaqueFunction(function=setup),
    ])
