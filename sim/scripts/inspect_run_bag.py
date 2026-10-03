"""Summarize GNSS and estimator status events from a ROS 2 MCAP run."""
import argparse
import json

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    args = ap.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts, status = {}, []
    status_type = get_message("std_msgs/msg/String")
    gnss_type = get_message("sensor_msgs/msg/NavSatFix")
    camera_type = get_message("sensor_msgs/msg/CameraInfo")
    first_fix = last_fix = None
    camera_info = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == "/nav/estimator_status":
            text = deserialize_message(raw, status_type).data
            try:
                status.append(json.loads(text))
            except json.JSONDecodeError:
                status.append({"event": text})
        elif topic == "/gps/fix":
            msg = deserialize_message(raw, gnss_type)
            fix = {"stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                   "lat": msg.latitude, "lon": msg.longitude, "alt": msg.altitude,
                   "covariance": list(msg.position_covariance), "covariance_type": msg.position_covariance_type}
            first_fix = first_fix or fix
            last_fix = fix
        elif topic in ("/camera/forward/camera_info", "/camera/down/camera_info"):
            msg = deserialize_message(raw, camera_type)
            camera_info[topic] = {"width": msg.width, "height": msg.height,
                                  "fx": msg.k[0], "fy": msg.k[4], "cx": msg.k[2], "cy": msg.k[5],
                                  "frame_id": msg.header.frame_id}
    gnss = [x for x in status if x.get("event") in ("gnss_initialized", "gnss_update")]
    rotations = [x for x in status if x.get("event") == "visual_span"]
    print(json.dumps({"topic_counts": counts, "camera_info": camera_info,
                      "first_gated_fix": first_fix, "last_gated_fix": last_fix,
                      "gnss_initialization": [x for x in gnss if x.get("event") == "gnss_initialized"],
                      "gnss_updates": len([x for x in gnss if x.get("event") == "gnss_update"]),
                      "gnss_update_samples": [x for x in gnss if x.get("event") == "gnss_update"][:5],
                      "gnss_update_tail": [x for x in gnss if x.get("event") == "gnss_update"][-5:],
                      "visual_spans": len(rotations)}, indent=2))


if __name__ == "__main__":
    main()
