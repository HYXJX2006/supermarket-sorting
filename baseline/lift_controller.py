#!/usr/bin/env python3
"""DG-202606 单次升降柱抬升控制器。

官方 Baseline 的抬升逻辑：slide_target = slide_grasp - lift_amount = 0.06。
默认 plan-only；真实执行必须：--execute --confirm lift。

只控制升降柱，并保持右臂关节与闭合夹爪；禁止底盘、左臂、松爪、撤出和配送。
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

SPINE_TOPIC = "/spine_forward_position_controller/commands"
RIGHT_ARM_TOPIC = "/right_arm_forward_position_controller/commands"
JOINT_TOPIC = "/joint_states"
PLAN_TOPIC = "/competition/lift_plan"
STATUS_TOPIC = "/competition/lift_status"

SLIDE_GRASP = 0.11
LIFT_AMOUNT = 0.05
SLIDE_MIN = -0.04
SLIDE_LIFT = SLIDE_GRASP - LIFT_AMOUNT
SLIDE_EXPECTED_MIN = 0.08
SLIDE_EXPECTED_MAX = 0.16
GRIP_CLOSE = 0.08
GRIP_TOLERANCE = 0.10
INIT_ARM_R = [0.0, -0.166, 0.032, 0.0, -1.571, -2.223]


class LiftController(Node):
    def __init__(self, execute: bool, confirm: str, timeout: float) -> None:
        super().__init__("lift_controller")
        self.execute_enabled = execute and confirm == "lift"
        self.rejected = execute and confirm != "lift"
        self.timeout = max(1.0, float(timeout))
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.joint_state_seen = False
        self.joints: dict[str, float] = {}
        self.target_slide: float | None = None
        self.plan: dict | None = None
        self.motion_started = False

        self.spine_pub = self.create_publisher(Float64MultiArray, SPINE_TOPIC, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, RIGHT_ARM_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            f"LIFT 控制器 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；"
            "只抬升升降柱，保持右臂和闭爪，不控制底盘"
        )

    def _on_joints(self, message: JointState) -> None:
        self.joint_state_seen = True
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _slide_now(self) -> float:
        return float(self.joints.get("slide_joint", SLIDE_GRASP))

    def _right_arm(self) -> list[float]:
        return [self.joints.get(f"right_arm_joint{i}", INIT_ARM_R[i - 1]) for i in range(1, 7)]

    def _plan(self) -> dict:
        # Lock the target once. Recomputing this every timer tick would move the
        # target downward as feedback changes and could cause over-lifting.
        if self.plan is not None:
            return self.plan
        current = self._slide_now()
        # The official sequence deploys at 0.11 and lifts to exactly 0.06.
        # Never derive a lower target from an unexpected current value.
        target = SLIDE_LIFT
        self.target_slide = target
        self.plan = {
            "schema_version": 1,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "action": "lift",
            "mechanical_commands_sent": False,
            "slide_current": round(current, 5),
            "slide_target": round(target, 5),
            "lift_amount": LIFT_AMOUNT,
            "right_arm_joints": [round(float(v), 5) for v in self._right_arm()],
            "right_gripper": GRIP_CLOSE,
            "base_commands_sent": False,
            "left_arm_commands_sent": False,
            "gripper_open_command_sent": False,
        }
        return self.plan

    def _publish_plan(self, plan: dict) -> None:
        msg = String()
        msg.data = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
        self.plan_pub.publish(msg)

    def _publish_status(self, state: str, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {"schema_version": 1, "state": state, "reason": reason, "slide_target": self.target_slide},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    def _gripper_closed(self) -> bool:
        value = self.joints.get("right_arm_eef_gripper_joint")
        return value is not None and value <= GRIP_CLOSE + GRIP_TOLERANCE

    def _publish_commands(self, plan: dict) -> None:
        self.spine_pub.publish(Float64MultiArray(data=[float(plan["slide_target"])]))
        self.right_pub.publish(
            Float64MultiArray(data=list(plan["right_arm_joints"]) + [float(plan["right_gripper"])])
        )

    def _tick(self) -> None:
        if self.done:
            return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm lift")
            self.done = True
            return
        if time.monotonic() - self.started_at > self.timeout:
            self.get_logger().error("抬升控制器超时，停止发送")
            self.done = True
            return

        plan = self._plan()
        if not self.execute_enabled:
            self._publish_plan(plan)
            self.get_logger().info("PLAN-ONLY 抬升计划：" + json.dumps(plan, ensure_ascii=False, separators=(",", ":")))
            self.success = True
            self.done = True
            return

        if not self.joint_state_seen:
            self._publish_status("waiting", "等待 joint_states，拒绝发送抬升命令")
            if time.monotonic() - self.last_log > 2.0:
                self.get_logger().warning("未收到 /joint_states，当前不发送抬升命令")
                self.last_log = time.monotonic()
            return
        # Validate the starting state only once. During a valid lift the slide
        # is expected to cross below 0.08 on its way to the locked 0.06 target.
        if not self.motion_started:
            current_slide = self._slide_now()
            if current_slide < SLIDE_EXPECTED_MIN or current_slide > SLIDE_EXPECTED_MAX:
                self._publish_status("blocked", f"slide 当前初始值不在部署范围: {current_slide:.4f}")
                self.get_logger().error(
                    f"当前初始 slide={current_slide:.4f} 不在安全部署范围 "
                    f"[{SLIDE_EXPECTED_MIN:.2f},{SLIDE_EXPECTED_MAX:.2f}]，拒绝 lift"
                )
                self.done = True
                return
            if not self._gripper_closed():
                self._publish_status("blocked", "夹爪反馈未显示闭合，拒绝抬升")
                self.get_logger().error("夹爪尚未闭合，拒绝执行 lift")
                self.done = True
                return
            self.motion_started = True
        if not self._gripper_closed():
            self._publish_status("blocked", "夹爪反馈未显示闭合，拒绝抬升")
            self.get_logger().error("夹爪尚未闭合，拒绝执行 lift")
            self.done = True
            return

        plan["mechanical_commands_sent"] = True
        self._publish_commands(plan)
        slide = self.joints.get("slide_joint")
        if slide is not None and abs(slide - plan["slide_target"]) <= 0.02:
            self._publish_status("reached", "升降柱达到抬升目标")
            self.get_logger().info("抬升动作完成；夹爪保持闭合，未执行撤出")
            self.success = True
            self.done = True
        elif time.monotonic() - self.last_log > 2.0:
            self.get_logger().info(f"抬升执行中：slide={slide} target={plan['slide_target']}")
            self.last_log = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 单次升降柱抬升")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if args.execute and args.confirm != "lift":
        print("拒绝执行：必须使用 --confirm lift")
        return 2

    rclpy.init()
    node = LiftController(args.execute, args.confirm, args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        result = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())
