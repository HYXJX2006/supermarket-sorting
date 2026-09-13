#!/usr/bin/env python3
"""odom/scan 数据流探针：容器内独立进程订阅并计数。"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main() -> int:
    topics = sys.argv[1:] or ["/slamware_ros_sdk_server_node/odom", "/slamware_ros_sdk_server_node/scan"]
    rclpy.init()
    node = Node("data_probe")
    counts = {t: [0] for t in topics}
    for t in topics:
        if "odom" in t:
            node.create_subscription(Odometry, t, (lambda c: (lambda m: c.__setitem__(0, c[0] + 1)))(counts[t]), qos_profile_sensor_data)
        else:
            node.create_subscription(LaserScan, t, (lambda c: (lambda m: c.__setitem__(0, c[0] + 1)))(counts[t]), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    for t in topics:
        print(f"PROBE {t}: {counts[t][0]} msgs in 8s")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
