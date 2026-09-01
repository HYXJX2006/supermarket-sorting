#!/usr/bin/env python3
"""保存一次商品识别运行的原图、结果图、深度图和元数据。

这是只读 ROS 订阅节点，不发布 /cmd_vel、不控制机械臂。
"""
from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray


class RecognitionCapture(Node):
    def __init__(self, output_dir: str, interval: float) -> None:
        super().__init__("recognition_capture_session")
        self.bridge = CvBridge()
        root = Path(output_dir)
        session = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S") + f"-p{os.getpid()}"
        self.session_dir = root / session
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.interval = max(0.1, float(interval))
        self.started = time.monotonic()
        self.last_save = 0.0
        self.frame_index = 0
        self.latest_depth = None
        self.latest_result = None
        self.latest_detections = []
        self.latest_odom = None
        self.task = None
        self.create_subscription(Image, "/head_camera/color/image_raw", self._on_rgb, 10)
        self.create_subscription(Image, "/head_camera/aligned_depth_to_color/image_raw", self._on_depth, 10)
        self.create_subscription(Image, "/multiclass/result_image", self._on_result, 10)
        self.create_subscription(Detection3DArray, "/multiclass/detections", self._on_detections, 10)
        self.create_subscription(Odometry, "/slamware_ros_sdk_server_node/odom", self._on_odom, 10)
        self.create_subscription(String, "/supermarket_sorting/task", self._on_task, 10)
        self._write_json("session.json", {
            "session": session,
            "started_at": datetime.now().astimezone().isoformat(),
            "interval_s": self.interval,
            "read_only": True,
            "topics": {
                "rgb": "/head_camera/color/image_raw",
                "depth": "/head_camera/aligned_depth_to_color/image_raw",
                "result": "/multiclass/result_image",
                "detections": "/multiclass/detections",
                "odom": "/slamware_ros_sdk_server_node/odom",
                "task": "/supermarket_sorting/task",
            },
        })
        self.get_logger().info(f"识别图像留档启动：{self.session_dir}，每 {self.interval:.1f}s 保存一帧")

    def _write_json(self, name: str, payload: dict) -> None:
        (self.session_dir / name).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"深度图转换失败：{exc}")

    def _on_result(self, msg: Image) -> None:
        try:
            self.latest_result = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"结果图转换失败：{exc}")

    def _on_detections(self, msg: Detection3DArray) -> None:
        records = []
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            p = result.pose.pose.position
            records.append({
                "class": str(result.hypothesis.class_id),
                "confidence": round(float(result.hypothesis.score), 5),
                "world": [round(float(p.x), 5), round(float(p.y), 5), round(float(p.z), 5)],
            })
        self.latest_detections = records

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = {
            "position": [round(float(p.x), 5), round(float(p.y), 5), round(float(p.z), 5)],
            "orientation": [round(float(q.x), 5), round(float(q.y), 5), round(float(q.z), 5), round(float(q.w), 5)],
        }

    def _on_task(self, msg: String) -> None:
        try:
            self.task = json.loads(msg.data)
        except Exception:
            self.task = {"raw": msg.data}

    def _depth_preview(self, depth) -> np.ndarray:
        if depth is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        arr = np.asarray(depth)
        valid = arr[np.isfinite(arr) & (arr > 0) & (arr < 8000)]
        if valid.size == 0:
            return np.zeros((*arr.shape[:2], 3), dtype=np.uint8)
        gray = (np.clip(arr.astype(np.float32), 0, 8000) * (255.0 / 8000.0)).astype(np.uint8)
        return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)

    def _on_rgb(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_save < self.interval:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"原图转换失败：{exc}")
            return
        self.last_save = now
        self.frame_index += 1
        stem = f"frame_{self.frame_index:06d}"
        result = self.latest_result if self.latest_result is not None else rgb
        cv2.imwrite(str(self.session_dir / f"{stem}_rgb.jpg"), rgb)
        cv2.imwrite(str(self.session_dir / f"{stem}_result.jpg"), result)
        cv2.imwrite(str(self.session_dir / f"{stem}_depth.png"), self._depth_preview(self.latest_depth))
        stamp = msg.header.stamp
        self._write_json(f"{stem}.json", {
            "frame_index": self.frame_index,
            "elapsed_s": round(now - self.started, 4),
            "ros_stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "task": self.task,
            "odom": self.latest_odom,
            "detections": self.latest_detections,
        })
        if self.frame_index == 1 or self.frame_index % 10 == 0:
            self.get_logger().info(
                f"已保存 {stem}：detections={len(self.latest_detections)} dir={self.session_dir}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="保存商品识别运行图像")
    parser.add_argument("--output", default="/workspace/baseline/debug_data/recognition_runs")
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = RecognitionCapture(args.output, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())