#!/usr/bin/env python3
"""slide 复位控制器：把升降柱降回安全巡航高度。

用途：高层（L3）抓取后 slide 处于高位（可达 0.87），若保持高位去配送，
经过料框/桌子区域时机身+展开的机械臂会撞桌沿。在 retreat 完成后、
deliver 导航开始前执行本 worker，降回 SLIDE_GRASP（0.11）安全高度。

只控制升降柱，不控制底盘/机械臂/夹爪。
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

SPINE_TOPIC = "/spine_forward_position_controller/commands"
JOINT_TOPIC = "/joint_states"
PLAN_TOPIC = "/competition/slide_reset_plan"
STATUS_TOPIC = "/competition/slide_reset_status"
SLIDE_TARGET = 0.11          # SLIDE_GRASP：安全巡航高度（与 lift 起点一致）
TOLERANCE = 0.02


class SlideResetController(Node):
    def __init__(self, execute: bool, confirm: str, timeout: float, target: float) -> None:
        super().__init__("slide_reset_controller")
        self.execute_enabled = execute and confirm == "slide_reset"
        self.rejected = execute and confirm != "slide_reset"
        self.timeout = max(1.0, float(timeout))
        self.target = float(target)
        self.started = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.joints: dict[str, float] = {}
        self.seen = False
        self.spine = self.create_publisher(Float64MultiArray, SPINE_TOPIC, 10)
        self.plan = self.create_publisher(String, PLAN_TOPIC, 10)
        self.status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            f"SLIDE_RESET mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；"
            f"目标 slide={self.target:.3f}；只控制升降柱，不控制底盘/机械臂/夹爪"
        )

    def _on_joints(self, message: JointState) -> None:
        self.seen = True
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _emit(self, state: str, reason: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "state": state,
                "reason": reason,
                "slide_target": round(self.target, 5),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status.publish(message)

    def _tick(self) -> None:
        if self.done:
            return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm slide_reset")
            self.done = True
            return
        if time.monotonic() - self.started > self.timeout:
            self._emit("failed", "超时")
            self.get_logger().error("SLIDE_RESET 超时，停止发送控制")
            self.done = True
            return
        if not self.seen:
            self._emit("waiting", "等待 joint_states")
            self._log("等待 joint_states，不发送复位命令")
            return
        current = self.joints.get("slide_joint")
        if current is None:
            self._emit("waiting", "等待 slide_joint 反馈")
            return
        self.spine.publish(Float64MultiArray(data=[self.target]))
        plan_message = String()
        plan_message.data = json.dumps(
            {"schema_version": 1, "action": "slide_reset", "slide_target": round(self.target, 5)},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.plan.publish(plan_message)
        error = abs(current - self.target)
        if error <= TOLERANCE:
            self._emit("reached", f"slide 复位完成 current={current:.3f}")
            self.get_logger().info(
                f"SLIDE_RESET 完成：slide={current:.3f} → 目标 {self.target:.3f}；"
                "机身已降至安全巡航高度"
            )
            self.success = True
            self.done = True
            return
        if time.monotonic() - self.last_log > 2.0:
            self.get_logger().info(
                f"SLIDE_RESET 执行中：slide={current:.3f} target={self.target:.3f} err={error:.3f}"
            )
            self.last_log = time.monotonic()

    def _log(self, text: str) -> None:
        if time.monotonic() - self.last_log > 2.0:
            self.get_logger().info(text)
            self.last_log = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 升降柱复位到安全巡航高度")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--target", type=float, default=SLIDE_TARGET)
    args = parser.parse_args()
    rclpy.init()
    node = SlideResetController(args.execute, args.confirm, args.timeout, args.target)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
