#!/usr/bin/env python3
"""DG-202606 底盘线速度标定。

只控制 /cmd_vel，不控制机械臂、夹爪或升降柱。
默认以 0.08 m/s 前进 40 秒，读取 odom 计算实际位移和平均速度；
前方激光距离不足时立即停车。
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"


class SpeedCalibration(Node):
    def __init__(self, speed: float, duration: float, front_stop: float) -> None:
        super().__init__("speed_calibration")
        self.speed = min(max(float(speed), 0.0), 0.08)
        self.duration = min(max(float(duration), 1.0), 60.0)
        self.front_stop = max(float(front_stop), 0.25)
        self.started_at = time.monotonic()
        self.motion_started_at: float | None = None
        self.done = False
        self.reason = ""
        self.x: float | None = None
        self.y: float | None = None
        self.start_x: float | None = None
        self.start_y: float | None = None
        self.last_odom_at = 0.0
        self.last_scan_at = 0.0
        self.front: float | None = None
        self.samples = 0
        self.max_odom_speed = 0.0
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"底盘速度标定开始：command={self.speed:.3f} m/s，最多 {self.duration:.1f}s，"
            f"front_stop={self.front_stop:.2f}m"
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
        self.last_odom_at = time.monotonic()
        self.samples += 1
        linear = message.twist.twist.linear
        self.max_odom_speed = max(
            self.max_odom_speed,
            math.hypot(float(linear.x), float(linear.y)),
        )

    def _on_scan(self, message: LaserScan) -> None:
        values: list[float] = []
        angle = float(message.angle_min)
        step = float(message.angle_increment)
        for index, value in enumerate(message.ranges):
            current_angle = (angle + index * step + math.pi) % (2.0 * math.pi) - math.pi
            if abs(current_angle) <= math.radians(24.0):
                value = float(value)
                if math.isfinite(value) and value > 0.0:
                    values.append(value)
        self.front = min(values) if values else None
        self.last_scan_at = time.monotonic()

    def _publish(self, speed: float) -> None:
        command = Twist()
        command.linear.x = speed
        self.cmd_pub.publish(command)

    def _finish(self, reason: str) -> None:
        self._publish(0.0)
        self.reason = reason
        self.done = True
        elapsed = (time.monotonic() - self.motion_started_at) if self.motion_started_at else 0.0
        distance = self._distance()
        average = distance / elapsed if elapsed > 0.0 else 0.0
        self.get_logger().info(
            "标定结束：reason=%s start=(%.4f,%.4f) end=(%.4f,%.4f) "
            "distance=%.4fm elapsed=%.2fs average=%.4fm/s max_odom=%.4fm/s samples=%d"
            % (
                reason,
                self.start_x or 0.0,
                self.start_y or 0.0,
                self.x or 0.0,
                self.y or 0.0,
                distance,
                elapsed,
                average,
                self.max_odom_speed,
                self.samples,
            )
        )

    def _distance(self) -> float:
        if self.start_x is None or self.start_y is None or self.x is None or self.y is None:
            return 0.0
        return math.hypot(self.x - self.start_x, self.y - self.start_y)

    def _tick(self) -> None:
        if self.done:
            self._publish(0.0)
            return
        now = time.monotonic()
        if now - self.started_at > self.duration + 15.0:
            self._finish("初始化或通信超时")
            return
        if self.x is None or now - self.last_odom_at > 1.0:
            self._publish(0.0)
            self._log("等待 odom，保持停车")
            return
        if self.front is None or now - self.last_scan_at > 1.0:
            self._publish(0.0)
            self._log("等待 LaserScan，保持停车")
            return
        if self.front < self.front_stop:
            self._finish(f"前方距离过近 front={self.front:.3f}m")
            return
        if self.motion_started_at is None:
            self.motion_started_at = now
        if now - self.motion_started_at >= self.duration:
            self._finish("达到标定时长")
            return
        self._publish(self.speed)
        self._log(
            f"标定中 distance={self._distance():.3f}m front={self.front:.2f}m "
            f"odom_speed_max={self.max_odom_speed:.3f}m/s"
        )

    def _log(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(text)
            self.last_log = now


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 底盘速度标定")
    parser.add_argument("--speed", type=float, default=0.08)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--front-stop", type=float, default=0.70)
    args = parser.parse_args()

    rclpy.init()
    node = SpeedCalibration(args.speed, args.duration, args.front_stop)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node._finish("收到中断")
    finally:
        node._publish(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
