#!/usr/bin/env python3
"""DG-202606 单次升降柱抬升控制器。

官方 Baseline 的抬升逻辑是“在当前 deploy 高度基础上下降 lift_amount”。
中层默认 deploy=0.11 时目标仍为 0.06；随机多层任务使用实际 deploy 高度计算。
默认 plan-only；真实执行必须：--execute --confirm lift。

只控制升降柱，并保持右臂关节与闭合夹爪；禁止底盘、左臂、松爪、撤出和配送。
"""

from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from grasp_geometry import geometry_for_kind

SPINE_TOPIC = "/spine_forward_position_controller/commands"
RIGHT_ARM_TOPIC = "/right_arm_forward_position_controller/commands"
META_TOPIC = "/competition/grasp_goal_meta"
JOINT_TOPIC = "/joint_states"
PLAN_TOPIC = "/competition/lift_plan"
STATUS_TOPIC = "/competition/lift_status"

SLIDE_GRASP = 0.11
LIFT_AMOUNT = 0.05
SLIDE_MIN = -0.04
SLIDE_LIFT = SLIDE_GRASP - LIFT_AMOUNT
# 随机多层任务的 deploy slide 可能高于官方中层 0.11；只要下降
# 5cm 后仍在升降柱行程内即可。
SLIDE_EXPECTED_MIN = SLIDE_MIN + LIFT_AMOUNT
SLIDE_EXPECTED_MAX = 0.87
# JointState feedback can overshoot the calibrated endpoint by a small amount
# because of floating-point/solver quantization. Keep the nominal travel range
# unchanged and allow only a 1 mm feedback tolerance at the safety gate.
SLIDE_FEEDBACK_TOLERANCE = float(
    os.getenv("SUPERMARKET_LIFT_SLIDE_FEEDBACK_TOLERANCE", "0.001")
)
GRIP_CLOSE = 0.08
# The joint-state value is the tendon-position feedback, not the 0..1
# controller target.  In this simulation open is about 1.00 and the closed
# command settles around 0.66, so use a calibrated separation threshold.
GRIP_CLOSED_FEEDBACK_MAX = float(os.getenv("SUPERMARKET_GRIP_CLOSED_FEEDBACK_MAX", "0.85"))

# 等待 grasp_goal_meta 的最长秒数。
# 发布器与 lift_controller 是各自独立启动的容器，存在启动竞态：若在元数据
# 到达之前就用构造函数默认门禁做判断，像 heweidao 这类「轻夹」商品
# （闭爪后 feedback≈0.99，必须用类别门禁 0.95 才判为已闭合）会被误判为
# 未闭合而直接拒绝 lift。实测 sanmingzhi 之所以侥幸通过，是因为它夹得紧
# （feedback 0.29），默认门禁 0.85 也放行——掩盖了这个竞态。
META_WAIT_SECONDS = float(os.getenv("SUPERMARKET_LIFT_META_WAIT_SECONDS", "8.0"))
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
        self.kind = ""
        self.grip_close = GRIP_CLOSE
        self.grip_feedback_max = GRIP_CLOSED_FEEDBACK_MAX
        # 是否已收到 grasp_goal_meta。类别门禁必须等它到达才可靠。
        self.meta_received = False
        self.meta_warned = False

        self.spine_pub = self.create_publisher(Float64MultiArray, SPINE_TOPIC, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, RIGHT_ARM_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            f"LIFT 控制器 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；"
            "只抬升升降柱，保持右臂和闭爪，不控制底盘"
        )

    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.kind = str(payload.get("kind", "")).strip().lower()
            geometry = geometry_for_kind(self.kind)
            self.grip_close = float(geometry.grip_close)
            self.grip_feedback_max = float(geometry.grip_feedback_max)
            self.meta_received = True
            self.get_logger().info(
                f"收到抓取元数据：kind={self.kind!r} "
                f"grip_close={self.grip_close:.4f} "
                f"grip_feedback_max={self.grip_feedback_max:.4f}"
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            return

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
        feedback_current = self._slide_now()
        # 官方中层序列是 0.11 -> 0.06；随机多层场景必须沿用
        # 本次 deploy 的实际高度，否则高层/低层目标会在 lift 阶段
        # 被错误拒绝或产生不匹配的抬升距离。对容差内的反馈值按名义
        # 行程边界计算，避免 0.8704 这类量化误差改变动作目标。
        current = min(max(feedback_current, SLIDE_EXPECTED_MIN), SLIDE_EXPECTED_MAX)
        target = max(SLIDE_MIN, current - LIFT_AMOUNT)
        self.target_slide = target
        self.plan = {
            "schema_version": 1,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "action": "lift",
            "mechanical_commands_sent": False,
            "slide_current": round(current, 5),
            "slide_feedback": round(feedback_current, 5),
            "slide_target": round(target, 5),
            "lift_amount": LIFT_AMOUNT,
            "right_arm_joints": [round(float(v), 5) for v in self._right_arm()],
            "right_gripper": self.grip_close,
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
        """Use the calibrated tendon feedback range as a lift safety gate."""
        value = self.joints.get("right_arm_eef_gripper_joint")
        return value is not None and value <= self.grip_feedback_max

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

        # EXECUTE 模式必须先收到 deploy 后的真实 slide 反馈，再锁定
        # lift 计划；不能在反馈缺失时用默认 0.11 伪造起始高度。
        if self.execute_enabled and not self.joint_state_seen:
            self._publish_status("waiting", "等待 joint_states，拒绝生成抬升计划")
            if time.monotonic() - self.last_log > 2.0:
                self.get_logger().warning("未收到 /joint_states，当前不生成或发送抬升计划")
                self.last_log = time.monotonic()
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
        # 首次状态校验前先给 grasp_goal_meta 一个软等待窗口。
        # 不阻塞、不重入 spin，只是推迟下结论：拿不到类别门禁就不能用默认值
        # 判定"夹爪是否已闭合"，否则 heweidao 这类轻夹商品会被误拒。
        if (
            self.joint_state_seen
            and not self.meta_received
            and time.monotonic() - self.started_at < META_WAIT_SECONDS
        ):
            self._publish_status("waiting", "等待 grasp_goal_meta 以取得类别门禁")
            if time.monotonic() - self.last_log > 1.0:
                self.get_logger().info(
                    f"等待 grasp_goal_meta（类别门禁）：已 "
                    f"{time.monotonic() - self.started_at:.1f}s / {META_WAIT_SECONDS:.0f}s"
                )
                self.last_log = time.monotonic()
            return
        if not self.meta_received and not self.motion_started and not self.meta_warned:
            self.meta_warned = True
            self.get_logger().warning(
                f"等待 {META_WAIT_SECONDS:.0f}s 仍未收到 grasp_goal_meta，"
                f"沿用默认门禁 {self.grip_feedback_max:.4f}"
            )

        # Validate the starting state only once. During a valid lift the slide
        # decreases by the locked lift amount while remaining inside its travel.
        if not self.motion_started:
            current_slide = self._slide_now()
            lower_limit = SLIDE_EXPECTED_MIN - SLIDE_FEEDBACK_TOLERANCE
            upper_limit = SLIDE_EXPECTED_MAX + SLIDE_FEEDBACK_TOLERANCE
            if current_slide < lower_limit or current_slide > upper_limit:
                self._publish_status("blocked", f"slide 当前初始值不在部署范围: {current_slide:.4f}")
                self.get_logger().error(
                    f"当前初始 slide={current_slide:.4f} 不在安全部署范围 "
                    f"[{SLIDE_EXPECTED_MIN:.2f},{SLIDE_EXPECTED_MAX:.2f}]，"
                    f"容差={SLIDE_FEEDBACK_TOLERANCE:.4f}，拒绝 lift"
                )
                self.done = True
                return
            if current_slide < SLIDE_EXPECTED_MIN or current_slide > SLIDE_EXPECTED_MAX:
                self.get_logger().warning(
                    f"slide 反馈={current_slide:.4f} 在名义范围外但处于 "
                    f"±{SLIDE_FEEDBACK_TOLERANCE:.4f} 容差内，按名义边界计算 lift 目标"
                )
            if not self._gripper_closed():
                feedback = self.joints.get("right_arm_eef_gripper_joint")
                self._publish_status("blocked", "夹爪反馈未达到闭合安全范围，拒绝抬升")
                self.get_logger().error(
                    f"夹爪尚未达到闭合安全范围：feedback={feedback} "
                    f"threshold={self.grip_feedback_max:.3f}，拒绝执行 lift"
                )
                self.done = True
                return
            self.motion_started = True
        if not self._gripper_closed():
            feedback = self.joints.get("right_arm_eef_gripper_joint")
            self._publish_status("blocked", "夹爪反馈未达到闭合安全范围，拒绝抬升")
            self.get_logger().error(
                f"夹爪尚未达到闭合安全范围：feedback={feedback} "
                f"threshold={self.grip_feedback_max:.3f}，拒绝执行 lift"
            )
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
