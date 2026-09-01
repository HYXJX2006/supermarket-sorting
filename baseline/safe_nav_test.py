#!/usr/bin/env python3
"""限时低速底盘测试：仅发布 /cmd_vel，结束或异常时立即刹停。"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


CMD_VEL_TOPIC = "/cmd_vel"
MAX_SPEED = 0.05
MAX_DURATION = 2.0


class SafeNavTest(Node):
    def __init__(self, speed: float, duration: float) -> None:
        super().__init__("safe_nav_test")
        self.speed = min(max(speed, 0.0), MAX_SPEED)
        self.duration = min(max(duration, 0.0), MAX_DURATION)
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.started_at = time.monotonic()
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().warn(
            f"安全导航测试开始：linear.x={self.speed:.3f} m/s，最多 {self.duration:.1f} 秒"
        )

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started_at
        if elapsed >= self.duration:
            self.stop()
            self.get_logger().info("安全导航测试结束，已发布零速度")
            rclpy.shutdown()
            return

        command = Twist()
        command.linear.x = self.speed
        command.angular.z = 0.0
        self.publisher.publish(command)

    def stop(self) -> None:
        self.publisher.publish(Twist())


def main() -> int:
    parser = argparse.ArgumentParser(description="限时低速 /cmd_vel 仿真测试")
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = SafeNavTest(args.speed, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("收到中断，立即发布零速度")
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
