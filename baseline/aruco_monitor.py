#!/usr/bin/env python3
"""只读监控 ArUco 货位检测，并映射到 ShelfMap；不发送任何控制命令。"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray, String

from shelf_map import ShelfMap


class ArucoMonitor(Node):
    def __init__(self, camera: str = "head") -> None:
        super().__init__("aruco_monitor")
        self.camera = camera
        self.shelf_map = ShelfMap()
        self.last_ids: list[int] = []
        self.last_records: list[dict] = []
        self.last_update = 0.0
        prefix = f"/aruco/{camera}"
        self.create_subscription(
            Int32MultiArray,
            f"{prefix}/ids",
            self._on_ids,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            f"{prefix}/detections",
            self._on_detections,
            10,
        )
        self.get_logger().info(f"monitoring {prefix}/ids and {prefix}/detections")

    def _on_ids(self, message: Int32MultiArray) -> None:
        self.last_ids = sorted({int(value) for value in message.data})
        self.last_update = time.monotonic()
        labels = [self.shelf_map.slots[aruco_id].label for aruco_id in self.last_ids]
        print(
            json.dumps(
                {"camera": self.camera, "ids": self.last_ids, "slots": labels},
                ensure_ascii=False,
            ),
            flush=True,
        )

    def _on_detections(self, message: String) -> None:
        try:
            records = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"ArUco detections 不是合法 JSON: {exc}")
            return
        if isinstance(records, list):
            self.last_records = records

    def report(self) -> None:
        print("=== aruco monitor report ===")
        print(f"camera={self.camera}")
        print(f"last_ids={self.last_ids}")
        print(
            "last_slots="
            + json.dumps(
                [self.shelf_map.slots[i].to_dict() for i in self.last_ids],
                ensure_ascii=False,
            )
        )
        print("=== end ===", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="只读监控 ArUco 货位检测")
    parser.add_argument("--camera", default="head", choices=("head", "left", "right"))
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = ArucoMonitor(args.camera)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
