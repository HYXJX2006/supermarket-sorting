#!/usr/bin/env python3
"""五货架正对、停稳、分层观察采样器。

只控制底盘和头部相机，不控制机械臂、夹爪、升降柱。
每个货架先到外侧横线，再前移到观察位；观察位的 yaw 根据当前底盘
位置到对应货架中心实时计算。位置、朝向和速度稳定后，依次采集上/中/下层。
"""
from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection3DArray


ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
RGB_TOPIC = "/head_camera/color/image_raw"
RESULT_TOPIC = "/multiclass/result_image"
DEPTH_TOPIC = "/head_camera/aligned_depth_to_color/image_raw"
JOINT_TOPIC = "/joint_states"
CMD_TOPIC = "/cmd_vel"
HEAD_CMD_TOPIC = "/head_forward_position_controller/commands"

# 官方布局中五组货架中心的 x 坐标；商品仍由实时视觉识别，不由这些坐标推断类别。
SHELVES = (("E", 1.805), ("D", 0.920), ("C", 0.035), ("B", -0.850), ("A", -1.735))
SHELF_CENTER_Y = 3.243
TRANSIT_Y = 2.475
OBSERVE_Y = 2.60
# 负值为相机下俯；中层沿用官方约 -0.6 的观察姿态。
PITCHES = (("upper", -0.35), ("middle", -0.60), ("lower", -0.85))


def wrap_to_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ShelfObserver(Node):
    def __init__(self, output_root: Path, max_speed: float, timeout: float) -> None:
        super().__init__("shelf_observer_capture")
        stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
        self.output_dir = output_root / stamp
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.max_speed = max(0.05, float(max_speed))
        self.timeout = max(60.0, float(timeout))
        self.started = time.monotonic()
        self.bridge = CvBridge()

        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.vx = 0.0
        self.wz = 0.0
        self.front_min: float | None = None
        self.left_min: float | None = None
        self.right_min: float | None = None
        self.avoid_heading: float | None = None
        self.pitch: float | None = None
        self.latest_rgb: np.ndarray | None = None
        self.latest_result: np.ndarray | None = None
        self.latest_depth: np.ndarray | None = None
        self.latest_rgb_at = 0.0
        self.latest_result_at = 0.0
        self.latest_depth_at = 0.0
        self.latest_detections: list[dict] = []

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.head_pub = self.create_publisher(Float64MultiArray, HEAD_CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, RESULT_TOPIC, self._on_result, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, "/multiclass/detections", self._on_detections, qos_profile_sensor_data)

        self.phase = "wait_odom"
        self.shelf_index = 0
        self.stage_index = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.phase_started = time.monotonic()
        self.stable_since: float | None = None
        self.capture_stage_started = 0.0
        self.last_log = 0.0
        self.records: list[dict] = []
        self.finished = False
        self.success = False
        self.get_logger().warning(
            f"货架观察采样器启动：输出={self.output_dir}，速度上限={self.max_speed:.2f} m/s；"
            "只控制底盘和头部相机"
        )
        self._write_json("session.json", {
            "started_at": datetime.now().astimezone().isoformat(),
            "mode": "shelf_facing_layered_capture",
            "read_only_for_manipulation": True,
            "shelves": [{"name": n, "x": x, "center_y": SHELF_CENTER_Y} for n, x in SHELVES],
            "transit_y": TRANSIT_Y,
            "observe_y": OBSERVE_Y,
            "pitches": [{"stage": n, "command": p} for n, p in PITCHES],
            "topics": {"rgb": RGB_TOPIC, "result": RESULT_TOPIC, "depth": DEPTH_TOPIC, "detections": "/multiclass/detections"},
        })

    def _write_json(self, name: str, payload: dict) -> None:
        (self.output_dir / name).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.x, self.y = float(p.position.x), float(p.position.y)
        self.yaw = yaw_from_quaternion(p.orientation)
        self.vx = float(msg.twist.twist.linear.x)
        self.wz = float(msg.twist.twist.angular.z)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        if not ranges:
            return
        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)

        def sector(center: float, half_width: float) -> float | None:
            values = []
            for index, value in enumerate(ranges):
                if not math.isfinite(value) or value <= 0.0:
                    continue
                angle = wrap_to_pi(angle_min + index * angle_inc)
                if abs(wrap_to_pi(angle - center)) <= half_width:
                    values.append(float(value))
            return min(values) if values else None

        self.front_min = sector(0.0, math.radians(22.0))
        self.left_min = sector(math.pi / 2.0, math.radians(55.0))
        self.right_min = sector(-math.pi / 2.0, math.radians(55.0))

    def _on_joints(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if name == "head_pitch_joint" and i < len(msg.position):
                self.pitch = float(msg.position[i])
                break

    def _on_rgb(self, msg: Image) -> None:
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_rgb_at = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"RGB 转换失败：{exc}")

    def _on_result(self, msg: Image) -> None:
        try:
            self.latest_result = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_result_at = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"结果图转换失败：{exc}")

    def _on_depth(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_at = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"深度图转换失败：{exc}")

    def _on_detections(self, msg: Detection3DArray) -> None:
        records = []
        for det in msg.detections:
            if not det.results:
                continue
            result = det.results[0]
            p = result.pose.pose.position
            records.append({
                "class": str(result.hypothesis.class_id),
                "confidence": round(float(result.hypothesis.score), 5),
                "world": [round(float(p.x), 5), round(float(p.y), 5), round(float(p.z), 5)],
            })
        self.latest_detections = records

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _head(self, pitch: float) -> None:
        self.head_pub.publish(Float64MultiArray(data=[0.0, float(pitch)]))

    def _set_target(self, x: float, y: float, yaw: float, phase: str) -> None:
        self.target_x, self.target_y, self.target_yaw = float(x), float(y), float(yaw)
        self.phase = phase
        self.phase_started = time.monotonic()
        self.stable_since = None
        self.get_logger().info(
            f"目标={phase} pos=({x:.3f},{y:.3f}) yaw={yaw:.3f}"
        )

    def _face_yaw(self, base_x: float, base_y: float, shelf_x: float) -> float:
        # 货架中心是唯一朝向参考，不使用固定 π/2 作为最终判据。
        return math.atan2(SHELF_CENTER_Y - base_y, shelf_x - base_x)

    def _drive(self, *, ignore_front: bool = False) -> bool:
        """先沿目标方向移动，到位后再按 target_yaw 原地对齐并停稳。"""
        now = time.monotonic()
        if self.x is None or self.y is None or self.yaw is None:
            self.stop()
            return False
        if now - self.started > self.timeout:
            self.get_logger().error("采样总超时，已停车")
            self.stop()
            self.finished = True
            return False

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)
        travel_heading = math.atan2(dy, dx)
        travel_error = wrap_to_pi(travel_heading - self.yaw)
        command = Twist()

        # 行驶阶段先对准当前航点方向；最终观察朝向只在位置到达后使用。
        if dist > 0.055 and abs(travel_error) > 0.30:
            command.angular.z = float(np.clip(1.8 * travel_error, -1.2, 1.2))
            self.cmd_pub.publish(command)
            if now - self.last_log > 1.0:
                self.get_logger().info(
                    f"对准行驶方向：dist={dist:.2f} heading_err={travel_error:.2f}"
                )
                self.last_log = now
            return False

        # 只在非货架最后接近段启用雷达绕障；货架本体不作为障碍绕开。
        if not ignore_front and self.front_min is not None and self.front_min < 0.35 and dist > 0.20:
            if self.avoid_heading is None:
                left = self.left_min if self.left_min is not None else 0.0
                right = self.right_min if self.right_min is not None else 0.0
                sign = 1.0 if left >= right else -1.0
                self.avoid_heading = wrap_to_pi(self.yaw + sign * math.radians(75.0))
                side = "左" if sign > 0 else "右"
                self.get_logger().warning(
                    f"前方障碍={self.front_min:.2f}m，向{side}侧绕行；"
                    f"left={left:.2f} right={right:.2f}"
                )
            avoid_error = wrap_to_pi(self.avoid_heading - self.yaw)
            if abs(avoid_error) > 0.14:
                command.angular.z = float(np.clip(1.6 * avoid_error, -0.9, 0.9))
            else:
                command.linear.x = min(0.25, 0.45 * dist)
            self.cmd_pub.publish(command)
            return False

        if self.avoid_heading is not None and (self.front_min is None or self.front_min > 0.55):
            self.get_logger().info("前方障碍已清除，恢复当前货架路线")
            self.avoid_heading = None

        if dist > 0.055:
            alignment = max(0.0, math.cos(travel_error))
            command.linear.x = min(
                self.max_speed if self.phase == "transit" else 0.30,
                1.25 * dist * alignment,
            )
            command.angular.z = float(np.clip(1.6 * travel_error, -1.0, 1.0))
            self.cmd_pub.publish(command)
            return False

        # 位置已到：此时才严格对齐货架正面方向，并等待速度归零。
        yaw_error = wrap_to_pi(self.target_yaw - self.yaw)
        if abs(yaw_error) > 0.06:
            command.angular.z = float(np.clip(1.5 * yaw_error, -0.8, 0.8))
            self.cmd_pub.publish(command)
            self.stable_since = None
            return False
        self.stop()
        speed_ok = abs(self.vx) < 0.035 and abs(self.wz) < 0.05
        if speed_ok:
            if self.stable_since is None:
                self.stable_since = now
            return now - self.stable_since >= 1.0
        self.stable_since = None
        return False

    def _save_view(self, shelf: str, stage: str, pitch_command: float) -> None:
        if self.latest_rgb is None:
            return
        now = time.monotonic()
        stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-%f")[:-3]
        stem = f"{shelf}_{stage}_{stamp}"
        rgb_path = self.output_dir / f"{stem}_rgb.jpg"
        result_path = self.output_dir / f"{stem}_result.jpg"
        depth_path = self.output_dir / f"{stem}_depth.jpg"
        cv2.imwrite(str(rgb_path), self.latest_rgb, [cv2.IMWRITE_JPEG_QUALITY, 70])
        result = self.latest_result if self.latest_result is not None else self.latest_rgb
        cv2.imwrite(str(result_path), result, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if self.latest_depth is not None:
            depth = np.asarray(self.latest_depth)
            valid = depth[np.isfinite(depth) & (depth > 0)]
            if valid.size:
                lo, hi = float(np.percentile(valid, 2)), float(np.percentile(valid, 98))
                preview = np.clip((depth.astype(np.float32) - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)
                preview = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
                cv2.imwrite(str(depth_path), preview, [cv2.IMWRITE_JPEG_QUALITY, 60])
        record = {
            "shelf": shelf,
            "stage": stage,
            "pitch_command": pitch_command,
            "pitch_feedback": self.pitch,
            "base": {"x": self.x, "y": self.y, "yaw": self.yaw, "vx": self.vx, "wz": self.wz},
            "shelf_center": [next(x for n, x in SHELVES if n == shelf), SHELF_CENTER_Y],
            "detections": self.latest_detections,
            "files": {"rgb": rgb_path.name, "result": result_path.name, "depth": depth_path.name if depth_path.exists() else None},
            "captured_at": datetime.now().astimezone().isoformat(),
        }
        self.records.append(record)
        self._write_json(f"{shelf}_{stage}.json", record)
        self.get_logger().info(
            f"已保存 {shelf}/{stage}：pitch_feedback={self.pitch} detections={len(self.latest_detections)}"
        )

    def tick(self) -> None:
        now = time.monotonic()
        if self.finished:
            return
        if now - self.started > self.timeout:
            self.stop()
            self.finished = True
            return
        if self.x is None or self.y is None or self.yaw is None:
            self.stop()
            self._head(-0.60)
            return
        if self.shelf_index >= len(SHELVES):
            self.stop()
            self._head(-0.60)
            self._write_json("manifest.json", {"success": True, "count": len(self.records), "records": self.records})
            self.success = len(self.records) == len(SHELVES) * len(PITCHES)
            self.finished = True
            self.get_logger().info(f"全部货架采样完成：{len(self.records)}/15")
            return

        shelf, shelf_x = SHELVES[self.shelf_index]
        if self.phase == "wait_odom":
            # 第一段只沿北向移动，不提前要求正对货架。
            self._set_target(self.x, TRANSIT_Y, math.pi / 2.0, "transit")
            return
        if self.phase == "transit":
            if self._drive(ignore_front=False):
                # 第二段横向移动到当前货架 x，行驶方向为东西向。
                travel_yaw = 0.0 if shelf_x >= float(self.x) else math.pi
                self._set_target(shelf_x, TRANSIT_Y, travel_yaw, "transit_x")
            return
        if self.phase == "transit_x":
            if self._drive(ignore_front=False):
                # 第三段前进至货架外侧观察线，行驶方向为北向；
                # 到位后 _drive 才会按 pi/2 正对货架。
                self._set_target(shelf_x, OBSERVE_Y, math.pi / 2.0, "observe_approach")
            return
        if self.phase == "observe_approach":
            if self._drive(ignore_front=True):
                self.stage_index = 0
                self.capture_stage_started = 0.0
                self._head(PITCHES[0][1])
                self.phase = "capture"
                self.phase_started = now
                self.stable_since = None
                self.get_logger().info(
                    f"{shelf} 已到位并正对货架：base=({self.x:.3f},{self.y:.3f}) "
                    f"yaw={self.yaw:.3f}，开始上中下三层采样"
                )
            return
        if self.phase == "capture":
            stage, pitch_command = PITCHES[self.stage_index]
            self._head(pitch_command)
            pitch_ok = self.pitch is None or abs(self.pitch - pitch_command) < 0.06
            fresh = (now - self.latest_rgb_at < 0.8) and (now - self.latest_result_at < 0.8)
            if pitch_ok and fresh:
                if self.capture_stage_started == 0.0:
                    self.capture_stage_started = now
                if now - self.capture_stage_started >= 1.0:
                    self._save_view(shelf, stage, pitch_command)
                    self.stage_index += 1
                    self.capture_stage_started = 0.0
                    if self.stage_index >= len(PITCHES):
                        self.shelf_index += 1
                        if self.shelf_index < len(SHELVES):
                            next_x = SHELVES[self.shelf_index][1]
                            travel_yaw = 0.0 if next_x >= float(self.x) else math.pi
                            self._set_target(next_x, TRANSIT_Y, travel_yaw, "transit_x")
                        else:
                            self.phase = "done"
                    else:
                        self._head(PITCHES[self.stage_index][1])
            return
        if self.phase == "done":
            self.stop()
            self._write_json("manifest.json", {"success": True, "count": len(self.records), "records": self.records})
            self.success = len(self.records) == len(SHELVES) * len(PITCHES)
            self.finished = True
            self.get_logger().info(f"全部货架采样完成：{len(self.records)}/15")

    def finalize(self) -> None:
        self.stop()
        self._head(-0.60)
        self._write_json("manifest.json", {"success": self.success, "count": len(self.records), "records": self.records})


def main() -> int:
    parser = argparse.ArgumentParser(description="五货架正对、停稳、分层观察采样器")
    parser.add_argument("--output-root", default="/workspace/baseline/debug_data/observation_views")
    parser.add_argument("--max-speed", type=float, default=0.90)
    parser.add_argument("--timeout", type=float, default=900.0)
    args = parser.parse_args()
    rclpy.init()
    node = ShelfObserver(Path(args.output_root), args.max_speed, args.timeout)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
    except KeyboardInterrupt:
        node.get_logger().warning("收到中断，已停车")
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    raise SystemExit(main())