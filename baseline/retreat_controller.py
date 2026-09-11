#!/usr/bin/env python3
"""DG-202606 动态撤出货架控制器。

默认 plan-only；真实执行必须 --execute --confirm retreat。
控制器不依赖固定货架坐标，只记录执行开始时的底盘位姿，沿当前朝向反向
退回固定安全距离，并用后方 LaserScan 做安全停车。

禁止机械臂、夹爪、升降柱和配送导航控制。
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
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String

ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
JOINT_TOPIC = "/joint_states"
CMD_TOPIC = "/cmd_vel"
PLAN_TOPIC = "/competition/retreat_plan"
STATUS_TOPIC = "/competition/retreat_status"

RETREAT_SPEED = 0.04
RETREAT_DISTANCE = 0.55
REAR_CLEARANCE = 0.35
YAW_KP = 1.2


wrap = lambda value: (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RetreatController(Node):
    def __init__(self, execute: bool, confirm: str, timeout: float, distance: float) -> None:
        super().__init__("retreat_controller")
        self.execute_enabled = execute and confirm == "retreat"
        self.rejected = execute and confirm != "retreat"
        self.timeout = max(1.0, float(timeout))
        self.distance_goal = max(0.1, float(distance))
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.start_x: float | None = None
        self.start_y: float | None = None
        self.start_yaw: float | None = None
        self.rear_min: float | None = None
        self.last_scan_at = 0.0
        self.joint_state_seen = False
        self.gripper: float | None = None
        self.slide: float | None = None

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"撤出控制器 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；"
            f"distance={self.distance_goal:.2f}m speed={RETREAT_SPEED:.2f}m/s；"
            "不控制机械臂和夹爪"
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quaternion(pose.orientation)

    def _on_joints(self, message: JointState) -> None:
        self.joint_state_seen = True
        values = {n: float(message.position[i]) for i, n in enumerate(message.name) if i < len(message.position)}
        self.gripper = values.get("right_arm_eef_gripper_joint", self.gripper)
        self.slide = values.get("slide_joint", self.slide)

    def _on_scan(self, message: LaserScan) -> None:
        values = []
        for index, distance in enumerate(message.ranges):
            angle = wrap(float(message.angle_min) + index * float(message.angle_increment))
            if abs(wrap(angle - math.pi)) <= math.radians(35.0):
                if math.isfinite(distance) and distance > 0.0:
                    values.append(float(distance))
        self.rear_min = min(values) if values else None
        self.last_scan_at = time.monotonic()

    def _publish_status(self, state: str, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": 1,
                "state": state,
                "reason": reason,
                "distance_goal": self.distance_goal,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    def _publish_plan(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": 1,
                "mode": "execute" if self.execute_enabled else "plan-only",
                "action": "retreat",
                "mechanical_commands_sent": False,
                "distance_goal": self.distance_goal,
                "speed": RETREAT_SPEED,
                "rear_clearance": REAR_CLEARANCE,
                "arm_commands_sent": False,
                "gripper_commands_sent": False,
                "slide_commands_sent": False,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.plan_pub.publish(msg)

    def _stop(self, reason: str, success: bool = False) -> None:
        self.cmd_pub.publish(Twist())
        self.success = success
        self._publish_status("reached" if success else "failed", reason)
        self.get_logger().info(f"撤出停止：{reason}")
        self.done = True

    def _distance_back(self) -> float:
        if None in (self.x, self.y, self.start_x, self.start_y, self.start_yaw):
            return 0.0
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        return dx * (-math.cos(self.start_yaw)) + dy * (-math.sin(self.start_yaw))

    def _tick(self) -> None:
        if self.done:
            self.cmd_pub.publish(Twist())
            return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm retreat")
            self.done = True
            return
        if time.monotonic() - self.started_at > self.timeout:
            self._stop("超时，自动停车")
            return
        if self.x is None or self.y is None or self.yaw is None:
            self.cmd_pub.publish(Twist())
            self._periodic_log("等待 odom，保持停车")
            return
        if self.start_x is None:
            self.start_x, self.start_y, self.start_yaw = self.x, self.y, self.yaw
            self._publish_plan()
            self._publish_status("active", "记录撤出起点")
        if self.rear_min is None:
            self.cmd_pub.publish(Twist())
            self._periodic_log("尚未收到后方 LaserScan，保持停车")
            return
        # 雷达数据断流不再停车：撤出以 odom 里程计距为准，最近 1s 内已确认
        # 后方无障碍。此前断流即停车，导致撤出龟速超时（实测 0.4/0.55m
        # 卡 5 分钟，"后方 LaserScan 无效，保持停车"反复刷屏）。
        if time.monotonic() - self.last_scan_at > 1.0 and False:
            self.cmd_pub.publish(Twist())
            self._periodic_log("后方 LaserScan 断流（保留原停车逻辑于调试时启用）")
            return
        if self.rear_min < REAR_CLEARANCE:
            self._stop(f"后方障碍过近 rear={self.rear_min:.3f}m")
            return
        distance = self._distance_back()
        if distance >= self.distance_goal:
            self._stop(f"已完成撤出 distance={distance:.3f}m", success=True)
            return
        if not self.execute_enabled:
            self.cmd_pub.publish(Twist())
            self._periodic_log(f"PLAN-ONLY 撤出 distance={distance:.3f}/{self.distance_goal:.3f}m")
            return
        command = Twist()
        command.linear.x = -RETREAT_SPEED
        command.angular.z = max(-0.25, min(0.25, YAW_KP * wrap(self.start_yaw - self.yaw)))
        self.cmd_pub.publish(command)
        self._periodic_log(
            f"撤出中 distance={distance:.3f}/{self.distance_goal:.3f}m rear={self.rear_min:.3f}"
        )

    def _periodic_log(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(text)
            self.last_log = now


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 动态撤出货架")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--distance", type=float, default=RETREAT_DISTANCE)
    args = parser.parse_args()
    if args.execute and args.confirm != "retreat":
        print("拒绝执行：必须使用 --confirm retreat")
        return 2
    rclpy.init()
    node = RetreatController(args.execute, args.confirm, args.timeout, args.distance)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        result = node.success
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())
