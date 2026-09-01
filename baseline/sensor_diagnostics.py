#!/usr/bin/env python3
"""ROS2 传感器诊断：只读取传感器，不发送任何机器人控制命令。"""

import argparse
import math
import time
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan
from tf2_msgs.msg import TFMessage


TOPICS = {
    "head_rgb": "/head_camera/color/image_raw",
    "head_depth": "/head_camera/aligned_depth_to_color/image_raw",
    "scan": "/slamware_ros_sdk_server_node/scan",
    "odom": "/slamware_ros_sdk_server_node/odom",
    "joint_states": "/joint_states",
    "tf": "/tf",
}


class SensorDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("sensor_diagnostics")
        self.samples: dict[str, Any] = {}
        self.first_seen: dict[str, float] = {}

        self.create_subscription(
            Image, TOPICS["head_rgb"], lambda msg: self._record_image("head_rgb", msg), qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            TOPICS["head_depth"],
            lambda msg: self._record_image("head_depth", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, TOPICS["scan"], self._record_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, TOPICS["odom"], self._record_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, TOPICS["joint_states"], self._record_joint_states, qos_profile_sensor_data
        )
        self.create_subscription(
            TFMessage, TOPICS["tf"], self._record_tf, qos_profile_sensor_data
        )

    def _mark(self, key: str, value: Any) -> None:
        self.samples[key] = value
        self.first_seen.setdefault(key, time.time())

    def _record_image(self, key: str, msg: Image) -> None:
        self._mark(
            key,
            {
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
                "step": msg.step,
                "data_bytes": len(msg.data),
                "frame_id": msg.header.frame_id,
            },
        )

    def _record_scan(self, msg: LaserScan) -> None:
        values = [value for value in msg.ranges if math.isfinite(value)]
        self._mark(
            "scan",
            {
                "count": len(msg.ranges),
                "valid": len(values),
                "min_range": min(values) if values else None,
                "max_range": max(values) if values else None,
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "frame_id": msg.header.frame_id,
            },
        )

    def _record_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._mark(
            "odom",
            {
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                },
            },
        )

    def _record_joint_states(self, msg: JointState) -> None:
        self._mark(
            "joint_states",
            {
                "count": len(msg.name),
                "names": list(msg.name),
                "positions": list(msg.position),
            },
        )

    def _record_tf(self, msg: TFMessage) -> None:
        self._mark(
            "tf",
            {
                "count": len(msg.transforms),
                "frames": [
                    {
                        "parent": transform.header.frame_id,
                        "child": transform.child_frame_id,
                    }
                    for transform in msg.transforms
                ],
            },
        )

    def complete(self) -> bool:
        return set(self.samples) == set(TOPICS)

    def print_report(self, duration: float) -> None:
        print("=== sensor diagnostics ===")
        print(f"duration_seconds={duration:.1f}")
        for key in TOPICS:
            if key in self.samples:
                print(f"[{key}] {self.samples[key]}")
            else:
                print(f"[{key}] NO_MESSAGE")
        print("=== end ===", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="只读检查智慧零售 ROS2 传感器话题")
    parser.add_argument("--duration", type=float, default=10.0, help="最多等待多少秒")
    args = parser.parse_args()

    rclpy.init()
    node = SensorDiagnostics()
    started = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = time.time() - started
            if node.complete() or elapsed >= args.duration:
                node.print_report(elapsed)
                break
    except KeyboardInterrupt:
        node.print_report(time.time() - started)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
