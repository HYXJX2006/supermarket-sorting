#!/usr/bin/env python3
"""DG-202606 安全机械臂单动作控制器。

默认是 plan-only；只有显式传入 --execute 才发布控制命令。
当前仅支持：
- safe_pose: 官方 Baseline 初始安全姿态 + 夹爪张开；
- open_gripper: 保持当前右臂关节，只张开右夹爪。

不控制底盘，不执行完整抓取，不闭合夹爪，不抬升，不撤出。
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
HEAD_TOPIC = "/head_forward_position_controller/commands"
LEFT_ARM_TOPIC = "/left_arm_forward_position_controller/commands"
RIGHT_ARM_TOPIC = "/right_arm_forward_position_controller/commands"
JOINT_TOPIC = "/joint_states"
PLAN_TOPIC = "/competition/grasp_plan"

INIT_ARM_L = [0.0, -0.166, 0.032, 0.0, 1.571, 2.223]
INIT_ARM_R = [0.0, -0.166, 0.032, 0.0, -1.571, -2.223]
GRIP_OPEN = 1.0
POST_PLACE_CLEAR_SLIDE = 0.30
POST_PLACE_STAGE_TOL = 0.06


class SafeArmController(Node):
    def __init__(self, action: str, execute: bool, timeout: float) -> None:
        super().__init__("safe_arm_controller")
        self.action_name = action
        self.execute_enabled = execute
        self.timeout = max(1.0, float(timeout))
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.joints: dict[str, float] = {}
        self.joint_state_seen = False
        self._stable_cycles = 0
        self._safe_start: dict | None = None
        self._safe_ramp_seconds = 4.0
        self._post_place_stage = "raise_slide" if action == "post_place_safe_pose" else None
        self._post_place_clear_slide: float | None = None

        self.spine_pub = self.create_publisher(Float64MultiArray, SPINE_TOPIC, 10)
        self.head_pub = self.create_publisher(Float64MultiArray, HEAD_TOPIC, 10)
        self.left_pub = self.create_publisher(Float64MultiArray, LEFT_ARM_TOPIC, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, RIGHT_ARM_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_timer(0.1, self._tick)

        self.get_logger().warning(
            f"机械臂单动作控制器：action={action} mode={'EXECUTE' if execute else 'PLAN-ONLY'}；"
            "不控制底盘、不执行完整抓取"
        )

    def _on_joints(self, message: JointState) -> None:
        self.joint_state_seen = True
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _current_right(self) -> list[float]:
        return [self.joints.get(f"right_arm_joint{i}", INIT_ARM_R[i - 1]) for i in range(1, 7)]

    def _plan(self) -> dict:
        if self.action_name in ("safe_pose", "post_place_safe_pose"):
            return {
                "schema_version": 1,
                "mode": "execute" if self.execute_enabled else "plan-only",
                "action": self.action_name,
                "mechanical_commands_sent": False,
                "slide": 0.11,
                "head": [0.0, 0.0],
                "left_arm": INIT_ARM_L,
                "right_arm": INIT_ARM_R,
                "left_gripper": GRIP_OPEN,
                "right_gripper": GRIP_OPEN,
            }
        return {
            "schema_version": 1,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "action": "open_gripper",
            "mechanical_commands_sent": False,
            "slide": self.joints.get("slide_joint", 0.11),
            "head": [
                self.joints.get("head_yaw_joint", 0.0),
                self.joints.get("head_pitch_joint", 0.0),
            ],
            "left_arm": [self.joints.get(f"left_arm_joint{i}", INIT_ARM_L[i - 1]) for i in range(1, 7)],
            "right_arm": self._current_right(),
            "left_gripper": self.joints.get("left_arm_eef_gripper_joint", GRIP_OPEN),
            "right_gripper": GRIP_OPEN,
        }

    def _publish_plan(self, plan: dict) -> None:
        message = String()
        message.data = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
        self.plan_pub.publish(message)

    def _publish_commands(self, plan: dict) -> None:
        self.spine_pub.publish(Float64MultiArray(data=[float(plan["slide"])]))
        self.head_pub.publish(Float64MultiArray(data=[float(v) for v in plan["head"]]))
        self.left_pub.publish(
            Float64MultiArray(data=[float(v) for v in plan["left_arm"]] + [float(plan["left_gripper"])])
        )
        self.right_pub.publish(
            Float64MultiArray(data=[float(v) for v in plan["right_arm"]] + [float(plan["right_gripper"])])
        )

    def _safe_pose_command(self, final_plan: dict) -> dict:
        """将抓取后的大姿态变化分段发送，避免一次跳变让关节卡住。"""
        if self._safe_start is None:
            self._safe_start = {
                "slide": float(self.joints.get("slide_joint", final_plan["slide"])),
                "head": [
                    float(self.joints.get("head_yaw_joint", final_plan["head"][0])),
                    float(self.joints.get("head_pitch_joint", final_plan["head"][1])),
                ],
                "left_arm": [float(self.joints.get(f"left_arm_joint{i}", final_plan["left_arm"][i - 1])) for i in range(1, 7)],
                "left_gripper": float(self.joints.get("left_arm_eef_gripper_joint", final_plan["left_gripper"])),
                "right_arm": [float(self.joints.get(f"right_arm_joint{i}", final_plan["right_arm"][i - 1])) for i in range(1, 7)],
                "right_gripper": float(self.joints.get("right_arm_eef_gripper_joint", final_plan["right_gripper"])),
            }
        alpha = min(1.0, max(0.0, (time.monotonic() - self.started_at) / self._safe_ramp_seconds))
        def lerp(start, end):
            return float(start) + alpha * (float(end) - float(start))
        return {
            "slide": lerp(self._safe_start["slide"], final_plan["slide"]),
            "head": [lerp(self._safe_start["head"][i], final_plan["head"][i]) for i in range(2)],
            "left_arm": [lerp(self._safe_start["left_arm"][i], final_plan["left_arm"][i]) for i in range(6)],
            "left_gripper": lerp(self._safe_start["left_gripper"], final_plan["left_gripper"]),
            "right_arm": [lerp(self._safe_start["right_arm"][i], final_plan["right_arm"][i]) for i in range(6)],
            "right_gripper": lerp(self._safe_start["right_gripper"], final_plan["right_gripper"]),
        }

    def _reached(self, plan: dict) -> bool:
        if not self.joints:
            return False
        if self.action_name == "open_gripper":
            return self.joints.get("right_arm_eef_gripper_joint", -999.0) >= GRIP_OPEN - 0.08
        checks = [("slide_joint", plan["slide"], 0.06)]
        checks += [(f"right_arm_joint{i}", plan["right_arm"][i - 1], 0.12) for i in range(1, 7)]
        checks += [("right_arm_eef_gripper_joint", plan["right_gripper"], 0.12)]
        ready = all(abs(self.joints.get(name, 999.0) - target) <= tol for name, target, tol in checks)
        self._stable_cycles = self._stable_cycles + 1 if ready else 0
        return self._stable_cycles >= 3

    def _post_place_command(self, final_plan: dict) -> dict:
        """分阶段离开配送台，再折叠到官方 safe_pose。"""
        if self._safe_start is None:
            self._safe_start = {
                "slide": float(self.joints.get("slide_joint", final_plan["slide"])),
                "head": [
                    float(self.joints.get("head_yaw_joint", final_plan["head"][0])),
                    float(self.joints.get("head_pitch_joint", final_plan["head"][1])),
                ],
                "left_arm": [float(self.joints.get(f"left_arm_joint{i}", final_plan["left_arm"][i - 1])) for i in range(1, 7)],
                "left_gripper": float(self.joints.get("left_arm_eef_gripper_joint", final_plan["left_gripper"])),
                "right_arm": [float(self.joints.get(f"right_arm_joint{i}", final_plan["right_arm"][i - 1])) for i in range(1, 7)],
                "right_gripper": float(self.joints.get("right_arm_eef_gripper_joint", final_plan["right_gripper"])),
            }
            self._post_place_clear_slide = max(POST_PLACE_CLEAR_SLIDE, self._safe_start["slide"])

        stage = self._post_place_stage or "raise_slide"
        clear_slide = float(self._post_place_clear_slide or POST_PLACE_CLEAR_SLIDE)
        if stage == "raise_slide":
            return {
                "slide": clear_slide,
                "head": list(self._safe_start["head"]),
                "left_arm": list(self._safe_start["left_arm"]),
                "left_gripper": float(self._safe_start["left_gripper"]),
                "right_arm": list(self._safe_start["right_arm"]),
                "right_gripper": GRIP_OPEN,
            }
        if stage == "fold_arm":
            return {
                "slide": clear_slide,
                "head": list(self._safe_start["head"]),
                "left_arm": list(self._safe_start["left_arm"]),
                "left_gripper": float(self._safe_start["left_gripper"]),
                "right_arm": list(final_plan["right_arm"]),
                "right_gripper": GRIP_OPEN,
            }
        return {
            "slide": float(final_plan["slide"]),
            "head": list(final_plan["head"]),
            "left_arm": list(final_plan["left_arm"]),
            "left_gripper": float(final_plan["left_gripper"]),
            "right_arm": list(final_plan["right_arm"]),
            "right_gripper": GRIP_OPEN,
        }

    def _post_place_reached(self, final_plan: dict) -> bool:
        if not self.joints:
            return False
        stage = self._post_place_stage or "raise_slide"
        clear_slide = float(self._post_place_clear_slide or POST_PLACE_CLEAR_SLIDE)
        if stage == "raise_slide":
            ready = abs(self.joints.get("slide_joint", 999.0) - clear_slide) <= POST_PLACE_STAGE_TOL
        elif stage == "fold_arm":
            ready = all(
                abs(self.joints.get(f"right_arm_joint{i}", 999.0) - final_plan["right_arm"][i - 1]) <= 0.12
                for i in range(1, 7)
            )
        else:
            return self._reached(final_plan)
        self._stable_cycles = self._stable_cycles + 1 if ready else 0
        if self._stable_cycles < 3:
            return False
        self._stable_cycles = 0
        if stage == "raise_slide":
            self._post_place_stage = "fold_arm"
            self.get_logger().info("post_place_safe_pose：升降柱已抬高，开始折叠右臂")
            return False
        self._post_place_stage = "lower_slide"
        self.get_logger().info("post_place_safe_pose：右臂已折叠，开始降回 safe_pose 高度")
        return False

    def _tick(self) -> None:
        if self.done:
            return
        plan = self._plan()
        if not self.execute_enabled:
            self._publish_plan(plan)
            self.get_logger().info("PLAN-ONLY 计划：" + json.dumps(plan, ensure_ascii=False, separators=(",", ":")))
            self.success = True
            self.done = True
            return

        # Execute mode is explicit and limited to the two safe actions.
        if not self.joint_state_seen:
            if time.monotonic() - self.started_at > self.timeout:
                self.get_logger().error("执行模式超时：未收到 /joint_states，拒绝发送控制命令")
                self.done = True
            elif time.monotonic() - self.last_log > 2.0:
                self.get_logger().warning("执行模式等待 /joint_states，当前拒绝发送控制命令")
                self.last_log = time.monotonic()
            return
        plan["mechanical_commands_sent"] = True
        if self.action_name == "safe_pose":
            command_plan = self._safe_pose_command(plan)
        elif self.action_name == "post_place_safe_pose":
            command_plan = self._post_place_command(plan)
        else:
            command_plan = plan
        self._publish_commands(command_plan)
        if self.action_name == "post_place_safe_pose":
            reached = self._post_place_reached(plan)
            if reached:
                self.get_logger().info(f"安全单动作完成：{self.action_name}")
                self.success = True
                self.done = True
        elif self._reached(plan):
            self.get_logger().info(f"安全单动作完成：{self.action_name}")
            self.success = True
            self.done = True
        elif time.monotonic() - self.started_at > self.timeout:
            self.get_logger().error(f"安全单动作超时：{self.action_name}，停止继续发送")
            self.done = True
        elif time.monotonic() - self.last_log > 2.0:
            max_error = 0.0
            detail = ""
            if self.action_name == "safe_pose":
                arm_errors = [
                    abs(self.joints.get(f"right_arm_joint{i}", 999.0) - plan["right_arm"][i - 1])
                    for i in range(1, 7)
                ]
                max_error = max(
                    [
                        abs(self.joints.get("slide_joint", 999.0) - plan["slide"]),
                        *arm_errors,
                        abs(self.joints.get("right_arm_eef_gripper_joint", 999.0) - plan["right_gripper"]),
                    ]
                )
                detail = " arm=" + ",".join(f"j{i + 1}:{value:.3f}" for i, value in enumerate(arm_errors))
            self.get_logger().info(
                f"执行中：{self.action_name}，等待 joint_states 到位，"
                f"max_error={max_error:.3f} stable={self._stable_cycles}/3{detail}"
            )
            self.last_log = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 安全机械臂单动作控制器")
    parser.add_argument("action", choices=("safe_pose", "post_place_safe_pose", "open_gripper"))
    parser.add_argument("--execute", action="store_true", help="明确允许发布机械臂/夹爪/升降柱命令")
    parser.add_argument("--confirm", default="", help="执行模式必须与动作名完全一致，例如 --confirm safe_pose")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if args.execute and args.confirm != args.action:
        print(f"拒绝执行：--confirm 必须精确等于 {args.action}")
        return 2

    rclpy.init()
    node = SafeArmController(args.action, args.execute, args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
