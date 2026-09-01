#!/usr/bin/env python3
"""只导航到货架观察点并停车：不控制机械臂、不抓取、不配送。"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


ROUTE = ((1.92, 2.475), (0.852, 2.475))
POS_TOL = 0.08
YAW_TOL = 0.06
MAX_SPEED = 0.20
MAX_ANGULAR = 0.80
TIMEOUT = 240.0


def wrap_to_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ShelfApproach(Node):
    def __init__(self) -> None:
        super().__init__("approach_shelf_watch")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Odometry,
            "/slamware_ros_sdk_server_node/odom",
            self._on_odom,
            10,
        )
        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.waypoint_index = 0
        self.mode = "turn"
        self.started = time.monotonic()
        self.last_log = 0.0
        self.get_logger().warning(
            "只导航到货架观察点：不控制机械臂、不抓取、不配送"
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quaternion(pose.orientation)

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def step(self) -> bool:
        now = time.monotonic()
        if now - self.started > TIMEOUT:
            self.get_logger().error("导航超时，已停车")
            self.stop()
            return True
        if self.x is None or self.y is None or self.yaw is None:
            if now - self.last_log > 1.0:
                self.get_logger().info("等待 odom...")
                self.last_log = now
            self.stop()
            return False

        target_x, target_y = ROUTE[self.waypoint_index]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = wrap_to_pi(target_yaw - self.yaw)
        command = Twist()

        if self.mode == "turn":
            command.angular.z = max(
                -MAX_ANGULAR, min(MAX_ANGULAR, 1.8 * yaw_error)
            )
            if abs(yaw_error) < YAW_TOL:
                self.mode = "drive"
                self.stop()
        elif distance < POS_TOL:
            self.stop()
            if self.waypoint_index + 1 < len(ROUTE):
                self.waypoint_index += 1
                self.mode = "turn"
                self.get_logger().info(
                    f"已到达航点 {self.waypoint_index}，准备转向下一航点"
                )
                return False
            self.get_logger().info("已到达货架前观察点，停车完成")
            return True
        else:
            alignment = max(0.0, math.cos(yaw_error))
            command.linear.x = min(MAX_SPEED, 0.65 * distance * alignment)
            command.angular.z = max(
                -MAX_ANGULAR, min(MAX_ANGULAR, 1.8 * yaw_error)
            )

        self.cmd_pub.publish(command)
        if now - self.last_log > 1.0:
            self.get_logger().info(
                f"phase={self.mode} waypoint={self.waypoint_index + 1}/{len(ROUTE)} "
                f"base=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.2f} "
                f"distance={distance:.2f}"
            )
            self.last_log = now
        return False


def main() -> int:
    rclpy.init()
    node = ShelfApproach()
    finished = False
    try:
        while rclpy.ok() and not finished:
            rclpy.spin_once(node, timeout_sec=0.05)
            finished = node.step()
    except KeyboardInterrupt:
        node.get_logger().warning("收到中断，已停车")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if finished else 1


if __name__ == "__main__":
    raise SystemExit(main())

