#!/usr/bin/env python3
"""DG-202606 单商品主流程编排器。

默认是 plan-only：只接收任务/目标信息并发布每个阶段的计划，不发送底盘、
机械臂、夹爪或升降柱命令。真实编排必须同时使用：
  --execute --confirm single_item --manage-workers

主流程：
  safe_pose -> approach -> deploy -> creep -> close_gripper -> lift -> retreat
  -> deliver_nav_global -> place -> verify

每个低层动作仍由独立控制器执行；本节点只负责顺序、超时、失败和目标流转。
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TASK_TOPIC = "/supermarket_sorting/task"
TARGET_STATUS_TOPIC = "/competition/target_status"
NAV_GOAL_TOPIC = "/competition/navigation_goal"
NAV_META_TOPIC = "/competition/navigation_goal_meta"
GRASP_META_TOPIC = "/competition/grasp_goal_meta"
GRASP_PLAN_TOPIC = "/competition/grasp_plan"
PLACE_STATUS_TOPIC = "/competition/place_status"
STATUS_TOPIC = "/competition/single_item_status"
PLAN_TOPIC = "/competition/single_item_plan"

ACTION_STAGES = (
    "safe_pose",
    "approach_search",
    "target_lock",
    "approach_target",
    "deploy",
    "creep",
    "close_gripper",
    "lift",
    "retreat",
    "deliver_nav_global",
    "place",
    "verify",
)

WORKER_SCRIPTS = {
    "safe_pose": ("safe_arm_controller.py", "safe_pose"),
    "approach_search": ("low_speed_navigator.py", ""),
    "approach_target": ("low_speed_navigator.py", ""),
    "deploy": ("grasp_deploy_controller.py", "deploy"),
    "creep": ("grasp_creep_controller.py", "creep"),
    "close_gripper": ("close_gripper_controller.py", "close_gripper"),
    "lift": ("lift_controller.py", "lift"),
    "retreat": ("retreat_controller.py", "retreat"),
    "deliver_nav_global": ("global_delivery_navigator.py", "deliver_nav_global"),
    "place": ("place_controller.py", "place"),
}


@dataclass
class Target:
    target_id: str
    kind: str
    status: str = "pending"
    candidate: dict[str, Any] | None = None
    navigation_meta: dict[str, Any] | None = None
    grasp_meta: dict[str, Any] | None = None
    grasp_plan: dict[str, Any] | None = None
    plan_only_allowed: bool = False
    recheck_gate_active: bool = False
    attempts: int = 0
    error: str | None = None


@dataclass
class Worker:
    stage: str
    process: subprocess.Popen[Any]
    started_at: float = field(default_factory=time.monotonic)


class SingleItemExecutor(Node):
    def __init__(
        self,
        *,
        execute: bool,
        confirm: str,
        manage_workers: bool,
        max_items: int,
        timeout: float,
        max_attempts: int,
        plan_audit: bool,
        demo_target: dict[str, Any] | None,
    ) -> None:
        super().__init__("single_item_executor")
        self.execute_enabled = execute and confirm == "single_item"
        self.rejected = execute and confirm != "single_item"
        self.manage_workers = bool(manage_workers)
        self.max_items = max(1, int(max_items))
        self.timeout = max(5.0, float(timeout))
        self.max_attempts = max(1, int(max_attempts))
        self.plan_audit = bool(plan_audit)
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.state = "waiting_task"
        self.stage_index = 0
        self.targets: list[Target] = []
        self.current_index = 0
        self.current: Target | None = None
        self.task_message = ""
        self.worker: Worker | None = None
        self.demo_target = demo_target
        self.last_plan_key = ""
        self.nav_goal: PoseStamped | None = None
        self.nav_meta: dict[str, Any] | None = None
        self.grasp_meta: dict[str, Any] | None = None
        self.completed_count = 0
        self.failure_count = 0
        self.verify_received = False
        self.verify_reason = ""

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, NAV_GOAL_TOPIC, 10)
        self.nav_meta_pub = self.create_publisher(String, NAV_META_TOPIC, 10)
        self.grasp_meta_pub = self.create_publisher(String, GRASP_META_TOPIC, 10)
        self.create_subscription(String, GRASP_PLAN_TOPIC, self._on_grasp_plan, 10)

        task_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, TASK_TOPIC, self._on_task, task_qos)
        self.create_subscription(String, TARGET_STATUS_TOPIC, self._on_target_status, 10)
        self.create_subscription(String, NAV_META_TOPIC, self._on_nav_meta, 10)
        self.create_subscription(String, GRASP_META_TOPIC, self._on_grasp_meta, 10)
        self.create_subscription(String, PLACE_STATUS_TOPIC, self._on_place_status, 10)
        self.create_timer(0.1, self._tick)

        if self.demo_target:
            self._bootstrap_demo_target(self.demo_target)

        self.get_logger().warning(
            f"SINGLE-ITEM mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'} "
            f"manage_workers={self.manage_workers} plan_audit={self.plan_audit}"
        )
        if self.execute_enabled and not self.manage_workers:
            self.get_logger().warning(
                "真实执行未启用 worker 管理；只会编排并发布计划，不会自动启动低层控制器"
            )

    @staticmethod
    def _json(data: str) -> dict[str, Any] | None:
        try:
            value = json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return None
        return value if isinstance(value, dict) else None

    def _emit(self, state: str, reason: str, *, stage: str | None = None) -> None:
        self.state = state
        payload = {
            "schema_version": 1,
            "state": state,
            "stage": stage or self._stage_name(),
            "reason": reason,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "manage_workers": self.manage_workers,
            "current_index": self.current_index,
            "completed_count": self.completed_count,
            "failure_count": self.failure_count,
            "target": self.current.target_id if self.current else None,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if rclpy.ok():
            try:
                self.status_pub.publish(message)
            except RCLError:
                return
        self.get_logger().info(message.data)

    def _stage_name(self) -> str:
        if self.stage_index >= len(ACTION_STAGES):
            return "done"
        return ACTION_STAGES[self.stage_index]

    def _current_target(self) -> Target | None:
        if self.current_index >= len(self.targets):
            return None
        return self.targets[self.current_index]

    def _bootstrap_demo_target(self, payload: dict[str, Any]) -> None:
        target_id = str(payload.get("target_id", "item_demo"))
        kind = str(payload.get("kind", "unknown"))
        self.targets = [Target(target_id=target_id, kind=kind)]
        self.current_index = 0
        self.current = self.targets[0]
        self.task_message = "demo-target"
        self.started_at = time.monotonic()
        self._apply_candidate(payload)
        self._emit("task_received", "使用演示目标启动 plan-audit")

    def _finish_current_target(self, reason: str) -> None:
        """完成当前目标；若队列还有目标则无缝切换到下一个目标。"""
        if self.current is not None:
            self.current.status = "done"
        self.completed_count += 1
        if self.current_index + 1 < len(self.targets):
            previous_id = self.current.target_id if self.current else None
            self.current_index += 1
            self.current = self.targets[self.current_index]
            self.stage_index = 0
            self.nav_goal = None
            self.nav_meta = None
            self.grasp_meta = None
            self.grasp_plan = None
            self.verify_received = False
            self.verify_reason = ""
            self.last_plan_key = ""
            self._emit(
                "target_complete",
                f"目标 {previous_id} 完成，切换到第 {self.current_index + 1} 个目标",
                stage="verify",
            )
            self._emit(
                "stage_start",
                f"进入目标 {self.current.target_id} 的 safe_pose",
                stage="safe_pose",
            )
            return
        self.success = True
        self.done = True
        self._emit("done", reason, stage="verify")

    def _retry_or_skip_current(self, reason: str, stage: str) -> None:
        """按统一策略重试当前目标，耗尽后跳过并继续队列。"""
        target = self.current
        if target is None:
            self.done = True
            return
        target.attempts += 1
        target.error = reason
        if target.attempts < self.max_attempts:
            self.worker = None
            self.stage_index = 0
            self.nav_goal = None
            self.nav_meta = None
            self.grasp_meta = None
            self.grasp_plan = None
            self.verify_received = False
            self.verify_reason = ""
            self.last_plan_key = ""
            self._emit("retrying", f"阶段 {stage} 失败：{reason}；第 {target.attempts + 1} 次尝试从 safe_pose 重来", stage=stage)
            self._emit("stage_start", "重试当前目标", stage="safe_pose")
            return
        target.status = "failed"
        self.failure_count += 1
        self._emit("target_failed", f"目标 {target.target_id} 达到最大尝试次数，跳过当前目标：{reason}", stage=stage)
        if self.current_index + 1 < len(self.targets):
            self.current_index += 1
            self.current = self.targets[self.current_index]
            self.stage_index = 0
            self.nav_goal = None
            self.nav_meta = None
            self.grasp_meta = None
            self.grasp_plan = None
            self.verify_received = False
            self.verify_reason = ""
            self.last_plan_key = ""
            self._emit("next_target", "跳过失败目标，继续下一个目标", stage="safe_pose")
            return
        self.done = True
        self.success = False
        self._emit("done", "整局队列已结束，但存在失败目标", stage=stage)

    def _on_place_status(self, message: String) -> None:
        payload = self._json(message.data)
        if not payload or self.done:
            return
        state = str(payload.get("state", ""))
        if state in {"reached", "placed", "verified"}:
            # 只记录结果，由 _execute_tick 在 verify 阶段统一完成目标切换。
            # 这样不会在 ROS 回调中直接修改队列，避免与定时器分支竞态或重复计数。
            self.verify_received = True
            self.verify_reason = str(payload.get("reason", state))
        elif state == "failed" and self._stage_name() == "verify":
            reason = str(payload.get("reason", "放置结果失败"))
            self._emit("failed", reason, stage="verify")
            self._retry_or_skip_current(reason, "verify")

    def _on_task(self, message: String) -> None:
        if message.data == self.task_message:
            return
        payload = self._json(message.data)
        if not payload or not isinstance(payload.get("targets"), list):
            self.get_logger().error("任务消息缺少 targets 数组")
            return
        parsed: list[Target] = []
        for raw in payload["targets"]:
            if not isinstance(raw, dict):
                continue
            target_id = raw.get("id")
            kind = raw.get("kind")
            if isinstance(target_id, str) and target_id and isinstance(kind, str) and kind:
                parsed.append(Target(target_id=target_id, kind=kind))
        if not parsed:
            self.get_logger().error("任务中没有合法目标")
            return
        self.task_message = message.data
        self.targets = parsed[: self.max_items]
        self.current_index = 0
        self.current = self.targets[0]
        self.stage_index = 0
        self.started_at = time.monotonic()
        self._emit("task_received", f"收到任务，共{len(parsed)}个目标；本节点处理前{len(self.targets)}个")
        if not self.demo_target:
            # 真正的目标锁定来自 competition_executor 发布的 target_status；
            # 这里不把 task 中的 kind 当成已经完成视觉定位。
            self._emit("waiting_target_lock", "等待 competition_executor 发布候选和货位")

        if self.demo_target:
            self._apply_candidate(self.demo_target)

    def _on_target_status(self, message: String) -> None:
        payload = self._json(message.data)
        if not payload:
            return
        current = payload.get("current_target")
        if not isinstance(current, dict):
            return
        target_id = current.get("id")
        if not isinstance(target_id, str):
            return
        target = next((item for item in self.targets if item.target_id == target_id), None)
        if target is None:
            return
        target.recheck_gate_active = True
        target.status = str(current.get("status", target.status))
        if current.get("plan_only_allowed") is True:
            target.plan_only_allowed = True
        # 感知节点可能周期性重复发布旧的 attempts=0；不能让外部快照
        # 覆盖执行器已经累计的重试次数，否则失败目标永远无法被跳过。
        reported_attempts = int(current.get("attempts", target.attempts) or 0)
        target.attempts = max(target.attempts, reported_attempts)
        candidate = current.get("candidate")
        if isinstance(candidate, dict):
            self._apply_candidate(candidate, target=target)

    def _apply_candidate(self, payload: dict[str, Any], target: Target | None = None) -> None:
        target = target or self.current
        if target is None:
            return
        candidate = payload.get("candidate") if isinstance(payload.get("candidate"), dict) else payload
        target.candidate = candidate
        if isinstance(candidate.get("navigation_goal"), list) and len(candidate["navigation_goal"]) >= 3:
            target.navigation_meta = {"navigation_goal": candidate["navigation_goal"]}
            if self.current is target:
                self._make_nav_goal(candidate["navigation_goal"])
        if isinstance(payload.get("navigation_goal"), list):
            target.navigation_meta = {"navigation_goal": payload["navigation_goal"]}
        if isinstance(payload.get("object_world"), list):
            target.grasp_meta = payload
            if "plan_only_allowed" in payload or target.recheck_gate_active:
                target.plan_only_allowed = (
                    payload.get("plan_only_allowed") is True
                    and payload.get("local_recheck_required") is not True
                )
            else:
                # 兼容未接入库存编排器的官方单目标流程。
                target.plan_only_allowed = True
        self._switch_to_target_navigation_if_ready(target)
        if (
            self.current is target
            and self.stage_index == 0
            and self.state in {"waiting_task", "waiting_target_lock", "task_received"}
        ):
            # 候选只能把初始等待状态标记为已锁定；如果候选在
            # approach_search/target_lock 期间到达，绝不能把动作流水线
            # 重置回 safe_pose，避免重复导航并导致状态竞态。
            self._emit("target_locked", f"目标 {target.target_id} 已获得候选信息")

    def _stop_worker(self, reason: str) -> None:
        """停止当前低层 worker，并确保底盘 worker 有机会发布零速度。"""
        if self.worker is None:
            return
        process = self.worker.process
        stage = self.worker.stage
        self.get_logger().info(f"停止低层 worker：stage={stage} reason={reason}")
        try:
            process.terminate()
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
        finally:
            self.worker = None

    def _switch_to_target_navigation_if_ready(self, target: Target) -> bool:
        """候选锁定后打断通用扫描，直接切换到目标专属导航。"""
        if not (self.execute_enabled and self.manage_workers and self.current is target):
            return False
        approach_search_index = ACTION_STAGES.index("approach_search")
        approach_target_index = ACTION_STAGES.index("approach_target")
        if self.stage_index != approach_search_index:
            return False
        if target.candidate is None or self.nav_goal is None:
            return False
        if self.worker is not None and self.worker.stage != "approach_search":
            return False
        if self.worker is not None:
            self._stop_worker("已锁定目标，取消剩余通用扫描路线")
        self.stage_index = approach_target_index
        self.last_plan_key = ""
        self._emit(
            "target_navigation_start",
            f"目标 {target.target_id} 已锁定，跳过剩余扫描航点，切换目标专属导航",
            stage="approach_target",
        )
        return True

    def _on_nav_meta(self, message: String) -> None:
        payload = self._json(message.data)
        if not payload:
            return
        target_id = payload.get("target_id")
        target = self._find_target(target_id)
        if target is None:
            return
        target.navigation_meta = payload
        if isinstance(payload.get("navigation_goal"), list):
            self._make_nav_goal(payload["navigation_goal"])

    def _on_grasp_meta(self, message: String) -> None:
        payload = self._json(message.data)
        if not payload:
            return
        target_id = payload.get("target_id")
        target = self._find_target(target_id)
        if target is None:
            return
        target.grasp_meta = payload
        target.candidate = payload
        target.plan_only_allowed = (
            payload.get("plan_only_allowed") is True
            and payload.get("local_recheck_required") is not True
        ) if ("plan_only_allowed" in payload or target.recheck_gate_active) else True
        if self.current is target:
            self.grasp_meta = payload

    def _on_grasp_plan(self, message: String) -> None:
        payload = self._json(message.data)
        if not payload:
            return
        target_id = payload.get("target_id")
        target = self._find_target(target_id)
        if target is None:
            return
        target.grasp_plan = payload
        ik = payload.get("ik")
        reachable = isinstance(ik, dict) and ik.get("reachable") is True
        if self.current is target:
            self._emit(
                "ik_verified" if reachable else "ik_rejected",
                "plan-only IK 已通过" if reachable else f"plan-only IK 未通过：{ik.get('error') if isinstance(ik, dict) else '缺少IK结果'}",
                stage="deploy",
            )

    def _find_target(self, target_id: Any) -> Target | None:
        if not isinstance(target_id, str):
            return self.current
        return next((item for item in self.targets if item.target_id == target_id), None)

    def _make_nav_goal(self, values: list[Any]) -> None:
        if len(values) < 3:
            return
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.pose.position.x = float(values[0])
        goal.pose.position.y = float(values[1])
        goal.pose.position.z = 0.0
        import math

        yaw = float(values[2])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav_goal = goal

    def _publish_plan(self, stage: str, reason: str) -> None:
        target = self.current
        payload = {
            "schema_version": 1,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "stage": stage,
            "reason": reason,
            "target_id": target.target_id if target else None,
            "kind": target.kind if target else None,
            "candidate": target.candidate if target else None,
            "navigation_meta": target.navigation_meta if target else None,
            "grasp_meta": target.grasp_meta if target else None,
            "grasp_plan": target.grasp_plan if target else None,
            "mechanical_commands_sent": False,
            "base_commands_sent": False,
        }
        key = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if key == self.last_plan_key:
            return
        self.last_plan_key = key
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.plan_pub.publish(message)

    def _launch_worker(self, stage: str) -> None:
        if not self.execute_enabled or not self.manage_workers:
            return
        if self.worker is not None:
            return
        script, confirm = WORKER_SCRIPTS[stage]
        baseline = Path(os.getenv("BASELINE_DIR", "/workspace/baseline"))
        command = [sys.executable, str(baseline / script)]
        if stage == "safe_pose":
            # safe_arm_controller.py 的动作名是位置参数，不能只传 --confirm。
            command += ["safe_pose", "--execute", "--confirm", "safe_pose", "--timeout", "60"]
        elif stage == "approach_search":
            # 先沿官方无障碍取货走廊到黄线中点，再横向进入货架观察位；
            # 禁止从起点到最终观察位走欧氏直线穿越货架/墙体。
            search_speed = os.getenv("SUPERMARKET_APPROACH_SEARCH_MAX_SPEED", "0.3")
            search_route = os.getenv(
                "SUPERMARKET_APPROACH_ROUTE",
                "1.92,2.475,1.5707963267948966;0.852,2.475,1.5707963267948966",
            )
            command += [
                "--route", search_route,
                "--max-speed", search_speed, "--timeout", "600",
            ]
        elif stage == "approach_target":
            target_speed = os.getenv("SUPERMARKET_APPROACH_TARGET_MAX_SPEED", "0.3")
            command += [
                "--goal-topic", NAV_GOAL_TOPIC,
                "--pickup-approach",
                "--max-speed", target_speed,
                "--timeout", "300",
            ]
        elif stage == "deliver_nav_global":
            command += ["--execute", "--confirm", "deliver_nav_global", "--timeout", "900"]
        else:
            command += ["--execute", "--confirm", confirm, "--timeout", "180"]
            if stage == "creep":
                command += ["--pickup-approach"]
        self.get_logger().info(f"启动低层 worker：{' '.join(command)}")
        self.worker = Worker(stage=stage, process=subprocess.Popen(command))

    def _poll_worker(self) -> bool | None:
        if self.worker is None:
            return None
        code = self.worker.process.poll()
        if code is None:
            if time.monotonic() - self.worker.started_at > self.timeout:
                self._stop_worker("单商品编排总超时")
                return False
            return None
        stage = self.worker.stage
        self.worker = None
        return code == 0

    def _advance_after_success(self) -> None:
        current_stage = self._stage_name()
        self._emit("stage_reached", f"阶段 {current_stage} 完成", stage=current_stage)
        self.stage_index += 1
        self.last_plan_key = ""
        if self.stage_index >= len(ACTION_STAGES):
            return
        next_stage = self._stage_name()
        if next_stage == "approach_search" and self.current is not None:
            # 如果候选在 safe_pose 期间已经到达，进入扫描阶段前也应立即
            # 切换到目标专属导航，不能重新跑完整扫描路线。
            if self._switch_to_target_navigation_if_ready(self.current):
                return
            next_stage = self._stage_name()
        if next_stage == "verify":
            self._publish_plan(next_stage, "等待放置控制器/裁判确认结果")
            if self.plan_audit or not self.execute_enabled:
                self._finish_current_target("单目标计划已覆盖到结果确认阶段")
            else:
                self._emit("verify_waiting", "等待 place_status reached/verified", stage=next_stage)
            return
        self._emit("stage_start", f"进入阶段 {next_stage}", stage=next_stage)

    def _plan_tick(self) -> None:
        if self.current is None:
            self._emit("waiting_target_lock", "等待任务目标和视觉锁定")
            return
        if self.current.candidate is None:
            self._emit("waiting_target_lock", "等待目标候选、货位和世界坐标")
            return
        stage = self._stage_name()
        # plan-only 也必须实际收到 grasp_planner 的 IK 输出，不能把 deploy
        # 及后续动作仅当作标签顺序演练。IK 不通过时停在硬门槛前。
        if (
            stage in {"deploy", "creep", "close_gripper", "lift", "retreat"}
            and self.current.recheck_gate_active
            and not self.current.plan_only_allowed
        ):
            self._emit("waiting_local_recheck", "局部复核尚未通过，禁止进入 IK/机械臂阶段", stage=stage)
            return
        if stage in {"deploy", "creep", "close_gripper", "lift", "retreat"}:
            plan = self.current.grasp_plan
            if plan is None:
                self._emit("waiting_ik", "等待 grasp_planner 发布 plan-only IK 结果", stage=stage)
                return
            ik = plan.get("ik")
            if not isinstance(ik, dict) or ik.get("reachable") is not True:
                self._emit(
                    "ik_rejected",
                    f"IK 未通过，禁止进入 {stage}：{ik.get('error') if isinstance(ik, dict) else '缺少IK结果'}",
                    stage=stage,
                )
                return
        self._publish_plan(stage, "plan-only 不发送控制命令")
        self._advance_after_success()

    def _execute_tick(self) -> None:
        if self.current is None:
            self._emit("waiting_target_lock", "等待任务目标")
            return
        stage = self._stage_name()
        if stage == "target_lock":
            if self.current.candidate is None:
                self._emit("waiting_target_lock", "已到货架观察点，等待视觉候选和唯一 ArUco 货位")
                return
            self._publish_plan(stage, "目标已锁定，先导航到目标专属预抓取点")
            self._advance_after_success()
            return
        if (
            stage in {"deploy", "creep", "close_gripper", "lift", "retreat"}
            and self.current.recheck_gate_active
            and not self.current.plan_only_allowed
        ):
            self._emit("waiting_local_recheck", "局部复核尚未通过，禁止启动真实机械臂 worker", stage=stage)
            return
        if self.current.grasp_meta is None and stage in {"deploy", "creep", "close_gripper", "lift"}:
            self._emit("waiting_target_lock", "等待抓取元数据")
            return
        if stage == "verify":
            self._publish_plan(stage, "等待放置结果")
            if self.verify_received:
                reason = self.verify_reason or "放置结果已确认"
                self._emit("verified", reason, stage=stage)
                self._finish_current_target(reason)
            return
        # 先启动订阅型 worker，再重复发布目标/元数据，避免启动竞态丢失一次性消息。
        if self.worker is None:
            self._launch_worker(stage)
            return
        if stage == "approach_target" and self.nav_goal is not None:
            self.nav_goal_pub.publish(self.nav_goal)
        if stage in {"deploy", "creep", "close_gripper", "lift"} and self.current.grasp_meta:
            self._publish_string(self.grasp_meta_pub, self.current.grasp_meta)
        self._publish_plan(stage, "执行模式由低层 worker 负责控制")
        result = self._poll_worker()
        if result is None:
            return
        if result:
            self._advance_after_success()
        else:
            self._retry_or_skip_current(f"worker {stage} failed", stage)

    @staticmethod
    def _publish_string(publisher, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(message)

    def _tick(self) -> None:
        if self.done:
            return
        if self.rejected:
            self._emit("failed", "必须使用 --execute --confirm single_item")
            self.done = True
            return
        if time.monotonic() - self.started_at > self.timeout:
            self._emit("failed", "单商品编排总超时")
            if self.worker is not None:
                self._stop_worker("编排器退出")
            self.done = True
            return
        if self.plan_audit or not self.execute_enabled:
            self._plan_tick()
        else:
            self._execute_tick()

    def run(self) -> int:
        try:
            while rclpy.ok() and not self.done:
                rclpy.spin_once(self, timeout_sec=0.1)
        except (KeyboardInterrupt, ExternalShutdownException):
            self.state = "failed"
            if rclpy.ok():
                self._emit("failed", "收到中断，停止编排")
        finally:
            if self.worker is not None:
                self.worker.process.terminate()
                self.worker = None
            try:
                self.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
        return 0 if self.success else 1


def parse_demo_target(value: str) -> dict[str, Any] | None:
    if not value:
        return None
    try:
        payload = json.loads(value)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"--demo-target JSON 非法: {exc}") from exc
    if not isinstance(payload, dict):
        raise SystemExit("--demo-target 必须是 JSON 对象")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 单商品主流程编排器")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--manage-workers", action="store_true")
    parser.add_argument("--max-items", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--plan-audit", action="store_true", help="收到目标后自动走完整计划但不发送控制命令")
    parser.add_argument("--demo-target", default="", help="用于 plan-audit 的目标 JSON")
    args = parser.parse_args()
    if args.execute and args.confirm != "single_item":
        print("拒绝执行：必须使用 --execute --confirm single_item")
        return 2
    rclpy.init()
    node = SingleItemExecutor(
        execute=args.execute,
        confirm=args.confirm,
        manage_workers=args.manage_workers,
        max_items=args.max_items,
        timeout=args.timeout,
        max_attempts=args.max_attempts,
        plan_audit=args.plan_audit,
        demo_target=parse_demo_target(args.demo_target),
    )
    return node.run()


if __name__ == "__main__":
    raise SystemExit(main())
