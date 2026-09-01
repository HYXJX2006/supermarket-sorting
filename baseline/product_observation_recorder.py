#!/usr/bin/env python3
"""只读商品观察记录器：把商品检测与当前唯一可见货位做保守关联。"""

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
from vision_msgs.msg import Detection3DArray

from shelf_map import ShelfMap


class ProductObservationRecorder(Node):
    def __init__(self, output_dir: Path, camera: str = "head") -> None:
        super().__init__("product_observation_recorder")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.camera = camera
        self.bridge = CvBridge()
        self.shelf_map = ShelfMap()
        self.current_ids: list[int] = []
        self.image_count = 0
        self.started = time.time()
        self.total_detections = 0
        self.unresolved_detections = 0
        self.records: dict[str, dict] = {}

        prefix = f"/aruco/{camera}"
        self.create_subscription(
            Int32MultiArray, f"{prefix}/ids", self._on_ids, qos_profile_sensor_data
        )
        self.create_subscription(
            Detection3DArray, "/kele/detections", self._on_detections, 10
        )
        self.create_subscription(
            Image, "/kele/result_image", self._on_result_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"recording /kele/detections + {prefix}/ids into {output_dir}"
        )

    def _on_ids(self, message: Int32MultiArray) -> None:
        self.current_ids = sorted(
            value for value in set(int(x) for x in message.data) if 0 <= value < 45
        )

    def _on_detections(self, message: Detection3DArray) -> None:
        visible = list(self.current_ids)
        for detection in message.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            kind = str(result.hypothesis.class_id or "unknown")
            confidence = float(result.hypothesis.score)
            position = result.pose.pose.position
            world = [float(position.x), float(position.y), float(position.z)]
            self.total_detections += 1

            aruco_id = visible[0] if len(visible) == 1 else None
            if aruco_id is None:
                self.unresolved_detections += 1

            key = f"{aruco_id}:{kind}" if aruco_id is not None else f"unresolved:{kind}"
            record = self.records.setdefault(
                key,
                {
                    "kind": kind,
                    "aruco_id": aruco_id,
                    "slot": self.shelf_map.slots[aruco_id].label
                    if aruco_id is not None
                    else None,
                    "frames_seen": 0,
                    "max_confidence": 0.0,
                    "world_positions": [],
                    "last_seen": None,
                    "association_rule": "single_visible_aruco_only",
                },
            )
            record["frames_seen"] += 1
            record["max_confidence"] = max(record["max_confidence"], confidence)
            record["last_seen"] = time.time()
            if len(record["world_positions"]) < 20:
                record["world_positions"].append(world)
        self._flush_json()

    def _on_result_image(self, message: Image) -> None:
        self.image_count += 1
        if self.image_count % 20 != 1:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"无法转换商品标注图像: {exc}")
            return
        path = self.output_dir / f"product_{self.image_count:05d}.png"
        if cv2.imwrite(str(path), image):
            self.get_logger().info(f"保存商品标注图: {path.name}")

    def _flush_json(self) -> None:
        payload = {
            "camera": self.camera,
            "started_at": self.started,
            "updated_at": time.time(),
            "total_detections": self.total_detections,
            "unresolved_detections": self.unresolved_detections,
            "records": list(self.records.values()),
        }
        (self.output_dir / "product_observations.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="记录商品检测与货位观察")
    parser.add_argument("--camera", default="head", choices=("head", "left", "right"))
    parser.add_argument(
        "--output-dir", default="/workspace/baseline/debug_data/product_observations"
    )
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = ProductObservationRecorder(Path(args.output_dir), args.camera)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
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
