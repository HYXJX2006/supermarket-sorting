#!/usr/bin/env python3
"""DG-202606 配送区导航控制器。

默认 plan-only；真实执行必须 --execute --confirm deliver_nav。
使用 odom + LaserScan 朝固定配送区观察点低速导航；不控制机械臂、夹爪、升降柱。
"""
from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
CMD_TOPIC = "/cmd_vel"
STATUS_TOPIC = "/competition/delivery_status"
DELIVERY_GOAL = (-1.88, -2.80, -math.pi / 2.0)


def wrap(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DeliveryNavigator(Node):
    def __init__(self, execute: bool, confirm: str, timeout: float, goal: tuple[float, float, float]) -> None:
        super().__init__("delivery_navigator")
        self.execute_enabled = execute and confirm == "deliver_nav"
        self.rejected = execute and confirm != "deliver_nav"
        self.timeout = max(1.0, float(timeout))
        self.goal_x, self.goal_y, self.goal_yaw = goal
        self.started = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.x = self.y = self.yaw = None
        self.front = None
        self.left = None
        self.right = None
        self.last_scan = 0.0
        self.avoid_turn = 0.0
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"配送导航 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'} "
            f"goal=({self.goal_x:.2f},{self.goal_y:.2f})；不控制机械臂"
        )

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.x, self.y, self.yaw = float(p.position.x), float(p.position.y), yaw_from_quaternion(msg.pose.pose.orientation)

    def _scan(self, msg: LaserScan) -> None:
        ranges = list(msg.ranges)
        angle_min = float(msg.angle_min)
        increment = float(msg.angle_increment)

        def sector(center: float, half_width: float) -> float | None:
            values = []
            for index, distance in enumerate(ranges):
                angle = wrap(angle_min + index * increment)
                if abs(wrap(angle - center)) <= half_width:
                    if math.isfinite(distance) and distance > 0:
                        values.append(float(distance))
            return min(values) if values else None

        self.front = sector(0.0, math.radians(24.0))
        self.left = sector(math.pi / 2.0, math.radians(50.0))
        self.right = sector(-math.pi / 2.0, math.radians(50.0))
        self.last_scan = time.monotonic()

    def _status(self, state: str, reason: str) -> None:
        m = String(); m.data = json.dumps({"schema_version": 1, "state": state, "reason": reason, "goal": [self.goal_x, self.goal_y, self.goal_yaw]}, ensure_ascii=False, separators=(",", ":")); self.status_pub.publish(m)

    def _stop(self, reason: str, success: bool = False) -> None:
        self.cmd_pub.publish(Twist()); self.success = success; self._status("reached" if success else "failed", reason); self.get_logger().info(f"配送导航停止：{reason}"); self.done = True

    def _tick(self) -> None:
        if self.done:
            self.cmd_pub.publish(Twist()); return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm deliver_nav"); self.done = True; return
        if time.monotonic() - self.started > self.timeout:
            self._stop("超时，自动停车"); return
        if self.x is None or self.y is None or self.yaw is None or time.monotonic() - self.last_scan > 1.0 or self.front is None:
            self.cmd_pub.publish(Twist()); self._log("等待 odom/LaserScan，保持停车"); return
        if not self.execute_enabled:
            self.cmd_pub.publish(Twist())
            self._status("planned", "传感器有效，配送导航计划已生成")
            self._log("PLAN-ONLY 配送导航计划，保持停车")
            self.success = True
            self.done = True
            return
        dist = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if dist < 0.08:
            err = wrap(self.goal_yaw - self.yaw)
            if abs(err) < 0.10:
                self._stop("已到配送区观察点并完成朝向对齐", True); return
            cmd = Twist(); cmd.angular.z = max(-0.35, min(0.35, 1.5 * err))
        else:
            # 局部避障：前方堵塞时原地向更宽的一侧转向，不能顶着障碍直冲。
            if self.front < 0.45:
                left = self.left if self.left is not None else 0.0
                right = self.right if self.right is not None else 0.0
                self.avoid_turn = 1.0 if left >= right else -1.0
                cmd = Twist()
                cmd.angular.z = self.avoid_turn * 0.35
                self._publish_and_log(cmd, f"配送路径前方堵塞 front={self.front:.3f}m，原地向{'左' if self.avoid_turn > 0 else '右'}绕行")
                return

            heading = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            err = wrap(heading - self.yaw)
            # 避障转向后，直到重新朝向目标才恢复前进。
            if self.avoid_turn and abs(err) < 0.18:
                self.avoid_turn = 0.0
            cmd = Twist()
            if self.avoid_turn:
                cmd.angular.z = self.avoid_turn * 0.30
            else:
                cmd.linear.x = min(0.08, max(0.0, 0.35 * dist * max(0.0, math.cos(err))))
                cmd.angular.z = max(-0.45, min(0.45, 1.4 * err))
        if not self.execute_enabled:
            cmd = Twist()
        self._publish_and_log(cmd, f"配送导航 dist={dist:.3f} front={self.front:.3f} left={self.left} right={self.right}")

    def _publish_and_log(self, cmd: Twist, text: str) -> None:
        if not self.execute_enabled:
            cmd = Twist()
        self.cmd_pub.publish(cmd)
        self._log(text)

    def _log(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_log > 2.0: self.get_logger().info(text); self.last_log = now


def main() -> int:
    p = argparse.ArgumentParser(description="DG-202606 配送区导航")
    p.add_argument("--execute", action="store_true"); p.add_argument("--confirm", default=""); p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--goal-x", type=float, default=DELIVERY_GOAL[0]); p.add_argument("--goal-y", type=float, default=DELIVERY_GOAL[1]); p.add_argument("--goal-yaw", type=float, default=DELIVERY_GOAL[2])
    a = p.parse_args()
    if a.execute and a.confirm != "deliver_nav": print("拒绝执行：必须使用 --confirm deliver_nav"); return 2
    rclpy.init(); n = DeliveryNavigator(a.execute, a.confirm, a.timeout, (a.goal_x, a.goal_y, a.goal_yaw))
    try:
        while rclpy.ok() and not n.done: rclpy.spin_once(n, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        result = n.success
        if rclpy.ok(): n.cmd_pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0 if result else 1

if __name__ == "__main__": raise SystemExit(main())
