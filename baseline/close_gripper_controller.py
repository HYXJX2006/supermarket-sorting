#!/usr/bin/env python3
"""DG-202606 单次闭合右夹爪控制器。

默认 plan-only；真实执行必须：
    --execute --confirm close_gripper

执行时只发布右臂当前关节 + 右夹爪闭合值，保持手臂位置不变。
禁止底盘、左臂、升降柱、抬升、撤出和配送。
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

from grasp_geometry import geometry_for_kind

RIGHT_ARM_TOPIC = "/right_arm_forward_position_controller/commands"
META_TOPIC = "/competition/grasp_goal_meta"
JOINT_TOPIC = "/joint_states"
PLAN_TOPIC = "/competition/close_gripper_plan"
STATUS_TOPIC = "/competition/close_gripper_status"
GRIP_CLOSE = 0.08
GRIP_TOLERANCE = 0.10
INIT_ARM_R = [0.0, -0.166, 0.032, 0.0, -1.571, -2.223]


class CloseGripperController(Node):
    def __init__(self, execute: bool, confirm: str, timeout: float, hold_seconds: float) -> None:
        super().__init__("close_gripper_controller")
        self.execute_enabled = execute and confirm == "close_gripper"
        self.rejected = execute and confirm != "close_gripper"
        self.timeout = max(1.0, float(timeout))
        self.hold_seconds = max(0.2, float(hold_seconds))
        self.started_at = time.monotonic()
        self.execute_started_at: float | None = None
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.joint_state_seen = False
        self.joints: dict[str, float] = {}
        self.kind = ""
        self.grip_close = GRIP_CLOSE
        self.geometry = None

        self.right_pub = self.create_publisher(Float64MultiArray, RIGHT_ARM_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"闭爪控制器 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；"
            "只控制右夹爪，不控制底盘/升降柱/左臂"
        )

    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.kind = str(payload.get("kind", "")).strip().lower()
            self.geometry = geometry_for_kind(self.kind)
            self.grip_close = float(self.geometry.grip_close)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _on_joints(self, message: JointState) -> None:
        self.joint_state_seen = True
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _right_arm(self) -> list[float]:
        return [self.joints.get(f"right_arm_joint{i}", INIT_ARM_R[i - 1]) for i in range(1, 7)]

    def _plan(self) -> dict:
        return {
            "schema_version": 1,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "action": "close_gripper",
            "mechanical_commands_sent": False,
            "right_arm_joints": [round(float(v), 5) for v in self._right_arm()],
            "right_gripper": self.grip_close,
            "hold_seconds": self.hold_seconds,
            "kind": self.kind,
            "left_arm_commands_sent": False,
            "spine_commands_sent": False,
            "base_commands_sent": False,
        }

    def _publish_plan(self, plan: dict) -> None:
        msg = String()
        msg.data = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
        self.plan_pub.publish(msg)

    def _publish_status(self, state: str, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {"schema_version": 1, "state": state, "reason": reason},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    def _publish_close_command(self, plan: dict) -> None:
        self.right_pub.publish(
            Float64MultiArray(data=list(plan["right_arm_joints"]) + [float(plan["right_gripper"])])
        )

    def _gripper_feedback(self) -> float | None:
        """Return the tendon-position feedback for diagnostics only.

        The official baseline advances after holding the close command for a
        fixed interval; this feedback is not on the same scale as GRIP_CLOSE
        and must not be used as a hard completion gate.
        """
        value = self.joints.get("right_arm_eef_gripper_joint")
        return float(value) if value is not None else None

    def _tick(self) -> None:
        if self.done:
            return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm close_gripper")
            self.done = True
            return
        if time.monotonic() - self.started_at > self.timeout:
            self.get_logger().error("闭爪控制器超时，停止发送")
            self.done = True
            return

        plan = self._plan()
        if not self.execute_enabled:
            self._publish_plan(plan)
            self.get_logger().info("PLAN-ONLY 闭爪计划：" + json.dumps(plan, ensure_ascii=False, separators=(",", ":")))
            self.success = True
            self.done = True
            return

        if not self.joint_state_seen:
            self._publish_status("waiting", "等待 joint_states，拒绝发送闭爪命令")
            if time.monotonic() - self.last_log > 2.0:
                self.get_logger().warning("未收到 /joint_states，当前不发送闭爪命令")
                self.last_log = time.monotonic()
            return

        if self.execute_started_at is None:
            self.execute_started_at = time.monotonic()
            self._publish_status("active", "开始保持闭爪命令")
            self.get_logger().info("开始闭合右夹爪，保持右臂关节不变")

        plan["mechanical_commands_sent"] = True
        self._publish_close_command(plan)
        elapsed = time.monotonic() - self.execute_started_at
        feedback = self._gripper_feedback()
        # Match the official client: hold the close target for the configured
        # interval, then advance. Feedback is reported for diagnostics only.
        if elapsed >= self.hold_seconds:
            # 夹空检测：feedback 低于类别下限 = 指头越过商品闭到指令位，
            # 手里没有东西。比赛流程靠这一步避免"空手配送"——未加 min 前
            # 空手也会走完 deliver/place（实测 random5 任务 1 heweidao
            # feedback=0.22 夹空却全链 success）。
            grip_min = float(getattr(self.geometry, "grip_feedback_min", 0.0) or 0.0)
            if self.execute_enabled and grip_min > 0.0 and feedback is not None and feedback < grip_min:
                self._publish_status(
                    "failed",
                    f"夹空检测：feedback={feedback:.4f} < 下限 {grip_min:.3f}，商品未入指间",
                )
                self.get_logger().error(
                    f"夹空检测失败：feedback={feedback:.4f} < grip_feedback_min={grip_min:.3f}；"
                    "商品不在指间，拒绝继续（executor 将标记本目标失败）"
                )
                self.success = False
                self.done = True
                return
            self._publish_status(
                "reached",
                f"已保持闭爪命令 {elapsed:.2f}s；反馈仅作诊断值={feedback}",
            )
            self.get_logger().info(
                f"闭爪时间门禁完成：held={elapsed:.2f}s feedback={feedback}; "
                "未执行抬升或撤出"
            )
            self.success = True
            self.done = True
        elif time.monotonic() - self.last_log > 2.0:
            self.get_logger().info(
                f"闭爪执行中：feedback={feedback} elapsed={elapsed:.1f}s"
            )
            self.last_log = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 单次闭合右夹爪")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--hold-seconds", type=float, default=0.8)
    args = parser.parse_args()
    if args.execute and args.confirm != "close_gripper":
        print("拒绝执行：必须使用 --confirm close_gripper")
        return 2

    rclpy.init()
    node = CloseGripperController(args.execute, args.confirm, args.timeout, args.hold_seconds)
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
