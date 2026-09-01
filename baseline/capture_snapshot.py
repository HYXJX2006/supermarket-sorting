#!/usr/bin/env python3
"""只读采样头部 RGB 和深度图，不发送任何机器人控制命令。"""

import argparse
import json
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


RGB_TOPIC = "/head_camera/color/image_raw"
DEPTH_TOPIC = "/head_camera/aligned_depth_to_color/image_raw"


class SnapshotCapture(Node):
    def __init__(self, output_dir: Path) -> None:
        super().__init__("snapshot_capture")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.rgb_saved = False
        self.depth_saved = False
        self.metadata: dict[str, object] = {}

        self.create_subscription(
            Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data
        )

    def _on_rgb(self, msg: Image) -> None:
        if self.rgb_saved:
            return
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().error(f"暂不支持 RGB 编码: {msg.encoding}")
            return

        channels = 3
        row_width = msg.step // channels
        array = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * row_width * channels
        if array.size < expected:
            self.get_logger().error(
                f"RGB 数据长度不足: got={array.size}, expected>={expected}"
            )
            return

        image = array[:expected].reshape(msg.height, row_width, channels)
        image = image[:, : msg.width, :]
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        path = self.output_dir / "head_rgb.png"
        if not cv2.imwrite(str(path), image):
            self.get_logger().error(f"RGB 保存失败: {path}")
            return

        self.metadata["rgb"] = {
            "topic": RGB_TOPIC,
            "encoding": msg.encoding,
            "width": msg.width,
            "height": msg.height,
            "step": msg.step,
            "frame_id": msg.header.frame_id,
            "file": str(path),
        }
        self.rgb_saved = True
        self.get_logger().info(f"已保存 RGB: {path}")

    def _on_depth(self, msg: Image) -> None:
        if self.depth_saved:
            return
        if msg.encoding not in ("mono16", "16UC1"):
            self.get_logger().error(f"暂不支持深度编码: {msg.encoding}")
            return

        row_width = msg.step // 2
        array = np.frombuffer(msg.data, dtype=np.uint16)
        expected = msg.height * row_width
        if array.size < expected:
            self.get_logger().error(
                f"深度数据长度不足: got={array.size}, expected>={expected}"
            )
            return

        depth = array[:expected].reshape(msg.height, row_width)[:, : msg.width]
        raw_path = self.output_dir / "head_depth_mm.png"
        if not cv2.imwrite(str(raw_path), depth):
            self.get_logger().error(f"深度原始图保存失败: {raw_path}")
            return

        preview = np.clip(depth.astype(np.float32) / 4000.0 * 255.0, 0, 255).astype(
            np.uint8
        )
        preview_path = self.output_dir / "head_depth_preview.png"
        cv2.imwrite(str(preview_path), preview)

        valid = depth[depth > 0]
        self.metadata["depth"] = {
            "topic": DEPTH_TOPIC,
            "encoding": msg.encoding,
            "width": msg.width,
            "height": msg.height,
            "step": msg.step,
            "frame_id": msg.header.frame_id,
            "unit_assumption": "millimeter",
            "valid_pixels": int(valid.size),
            "min_raw": int(valid.min()) if valid.size else None,
            "max_raw": int(valid.max()) if valid.size else None,
            "file": str(raw_path),
            "preview_file": str(preview_path),
        }
        self.depth_saved = True
        self.get_logger().info(f"已保存深度图: {raw_path}")

    def complete(self) -> bool:
        return self.rgb_saved and self.depth_saved

    def save_metadata(self) -> None:
        path = self.output_dir / "metadata.json"
        path.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.get_logger().info(f"已保存元数据: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="保存一帧头部 RGB 和深度图")
    parser.add_argument("--output-dir", default="/workspace/baseline/debug_data")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = SnapshotCapture(Path(args.output_dir))
    started = time.time()
    try:
        while rclpy.ok() and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.time() - started >= args.timeout:
                break
        node.save_metadata()
        if not node.complete():
            node.get_logger().error("超时：未能同时收到 RGB 和深度图")
            return 2
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
