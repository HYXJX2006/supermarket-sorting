#!/usr/bin/env python3
"""只读货架扫描记录器：持久化 ArUco 货位观察和带标注图像。"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from shelf_map import ShelfMap


class ShelfScanRecorder(Node):
    def __init__(self, output_dir: Path, camera: str = "head") -> None:
        super().__init__("shelf_scan_recorder")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.camera = camera
        self.bridge = CvBridge()
        self.shelf_map = ShelfMap()
        self.current_ids: list[int] = []
        self.observations: dict[int, dict] = {}
        self.image_count = 0
        self.started = time.monotonic()

        prefix = f"/aruco/{camera}"
        self.create_subscription(
            Int32MultiArray,
            f"{prefix}/ids",
            self._on_ids,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            f"{prefix}/result_image",
            self._on_result_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"recording {prefix}/ids and {prefix}/result_image into {output_dir}"
        )

    def _on_ids(self, message: Int32MultiArray) -> None:
        self.current_ids = sorted(
            value for value in set(int(x) for x in message.data) if 0 <= value < 45
        )
        now = time.time()
        for aruco_id in self.current_ids:
            slot = self.shelf_map.slots[aruco_id]
            record = self.observations.setdefault(
                aruco_id,
                {
                    "aruco_id": aruco_id,
                    "slot": slot.label,
                    "first_seen": now,
                    "frames_seen": 0,
                    "image_files": [],
                },
            )
            record["last_seen"] = now
            record["frames_seen"] += 1
        self._flush_json()

    def _on_result_image(self, message: Image) -> None:
        if not self.current_ids:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"无法转换 ArUco 标注图像: {exc}")
            return

        self.image_count += 1
        # 限制采样频率，避免同一视角产生大量重复图片。
        if self.image_count % 10 != 1:
            return
        filename = f"scan_{self.image_count:05d}_ids_"
        filename += "_".join(str(x) for x in self.current_ids) + ".png"
        path = self.output_dir / filename
        if not cv2.imwrite(str(path), image):
            self.get_logger().error(f"保存标注图失败: {path}")
            return
        for aruco_id in self.current_ids:
            files = self.observations[aruco_id]["image_files"]
            if str(path.name) not in files:
                files.append(str(path.name))
        self._flush_json()
        self.get_logger().info(
            f"记录货位观察: ids={self.current_ids}, image={path.name}"
        )

    def _flush_json(self) -> None:
        payload = {
            "camera": self.camera,
            "started_at": self.started,
            "updated_at": time.time(),
            "observations": [
                self.observations[key] for key in sorted(self.observations)
            ],
        }
        (self.output_dir / "shelf_scan.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="记录 ArUco 货位扫描结果")
    parser.add_argument("--camera", default="head", choices=("head", "left", "right"))
    parser.add_argument("--output-dir", default="/workspace/baseline/debug_data/shelf_scan")
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = ShelfScanRecorder(Path(args.output_dir), args.camera)
    try:
        while rclpy.ok() and time.monotonic() - node.started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node._flush_json()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
