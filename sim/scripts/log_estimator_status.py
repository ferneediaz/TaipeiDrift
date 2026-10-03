"""Write the status messages of several filter instances (JSON strings) to one JSONL file, each with its instance name.

Run inside the container while the simulator runs:
    python3 sim/scripts/log_estimator_status.py OUT.jsonl NAME=TOPIC [NAME=TOPIC ...]
    python3 sim/scripts/log_estimator_status.py run/status.jsonl ours=/nav_rf/estimator_status camera=/nav/estimator_status

The records say which readings a filter took and which it refused (events gnss_update, metric_flow, rf_update, ...).
"""
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    out, names = sys.argv[1], sys.argv[2:]
    rclpy.init()
    node = Node("status_logger")
    f = open(out, "w")

    def writer(name):
        def on(msg):
            try:
                record = json.loads(msg.data)
            except ValueError:
                return
            record["instance"] = name
            f.write(json.dumps(record) + "\n")
            f.flush()
        return on

    for item in names:
        name, topic = item.split("=", 1)
        node.create_subscription(String, topic, writer(name), 50)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
