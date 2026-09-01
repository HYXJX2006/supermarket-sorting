#!/usr/bin/env python3
"""随机45货位的 plan-only 货架扫描规划器。

读取 /supermarket_sorting/task 的五个目标类别，生成真实货架组观察点的扫描顺序；
只发布计划与状态，不发布 /cmd_vel，不控制机械臂、夹爪或升降柱。
目标与货位的绑定必须等待视觉结果，规划器不会按目标序号猜测货架。
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TASK_TOPIC = "/supermarket_sorting/task"
PLAN_TOPIC = "/competition/shelf_scan_plan"
STATUS_TOPIC = "/competition/shelf_scan_status"

# 5组货架的中心观察点。x来自官方 retail_competition_layout.json，
# y=2.475为官方固定流程使用的货架观察横线，yaw=+pi/2朝向货架。
SHELF_VIEWPOINTS = {
    "A": (-1.735, 2.475, math.pi / 2),
    "B": (-0.850, 2.475, math.pi / 2),
    "C": (0.035, 2.475, math.pi / 2),
    "D": (0.920, 2.475, math.pi / 2),
    "E": (1.805, 2.475, math.pi / 2),
}
# 起点在场景东南侧，开发阶段按最近组到最远组扫描；正式导航器仍需结合雷达重规划。
DEFAULT_SCAN_ORDER = ("E", "D", "C", "B", "A")


@dataclass
class Target:
    target_id: str
    kind: str
    status: str = "pending"
    observed_shelf: str | None = None
    observed_slot: str | None = None


class ShelfScanPlanner(Node):
    def __init__(self, expected_count: int, dwell_seconds: float, scan_order: tuple[str, ...]) -> None:
        super().__init__("shelf_scan_planner")
        self.expected_count = max(1, int(expected_count))
        self.dwell_seconds = max(0.0, float(dwell_seconds))
        self.scan_order = scan_order
        self.targets: list[Target] = []
        self.run_prefix = ""
        self.last_task = ""
        self.last_emit = 0.0
        self.finished = False
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, TASK_TOPIC, self._on_task, qos)
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, qos)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

    def _on_task(self, msg: String) -> None:
        if msg.data == self.last_task:
            return
        try:
            payload = json.loads(msg.data)
            raw_targets = payload.get("targets")
            if not isinstance(raw_targets, list):
                raise ValueError("targets must be a list")
            targets = []
            seen_ids: set[str] = set()
            for item in raw_targets:
                if not isinstance(item, dict) or not item.get("id") or not item.get("kind"):
                    raise ValueError("each target needs id and kind")
                target_id = str(item["id"])
                if target_id in seen_ids:
                    raise ValueError(f"duplicate target id: {target_id}")
                seen_ids.add(target_id)
                targets.append(Target(target_id, str(item["kind"])))
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self.get_logger().error(f"拒绝非法任务消息: {exc}")
            return
        if len(targets) != self.expected_count:
            self.get_logger().error(
                f"任务数量={len(targets)}，不符合期望={self.expected_count}；不生成扫描计划"
            )
            return
        self.last_task = msg.data
        self.run_prefix = str(payload.get("run_prefix", ""))
        self.targets = targets
        self._emit("task_received")

    def _target_queue(self) -> list[dict[str, object]]:
        return [
            {
                "id": target.target_id,
                "kind": target.kind,
                "status": target.status,
                "assigned_shelf_group": None,
                "slot": target.observed_slot,
                "binding": "pending_vision" if target.observed_slot is None else "vision_confirmed",
            }
            for target in self.targets
        ]

    def _emit(self, reason: str) -> None:
        queue = self._target_queue()
        observations = [
            {
                "sequence": index + 1,
                "shelf": shelf,
                "viewpoint": list(SHELF_VIEWPOINTS[shelf]),
                "dwell_seconds": self.dwell_seconds,
                "binding_rule": "only_unique_aruco_plus_stable_rgbd_candidate",
            }
            for index, shelf in enumerate(self.scan_order)
        ]
        plan = {
            "schema_version": 1,
            "mode": "plan-only",
            "run_prefix": self.run_prefix,
            "reason": reason,
            "publish_cmd_vel": False,
            "publish_arm_commands": False,
            "target_count": len(self.targets),
            "observation_order": observations,
            "targets": queue,
        }
        self.plan_pub.publish(String(data=json.dumps(plan, ensure_ascii=False, separators=(",", ":"))))
        status = {
            "schema_version": 1,
            "reason": reason,
            "mode": "plan-only",
            "publish_cmd_vel": False,
            "targets": queue,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, separators=(",", ":"))))
        self.get_logger().info(
            f"plan-only 扫描计划：目标数={len(self.targets)}，顺序={'→'.join(self.scan_order)}；"
            "目标货位全部等待视觉确认；不发布 /cmd_vel"
        )
        self.last_emit = time.monotonic()

    def tick(self) -> None:
        if not self.targets:
            if time.monotonic() - self.last_emit >= 3.0:
                self.get_logger().info("等待随机5目标任务消息；不移动")
                self.last_emit = time.monotonic()
            return
        if time.monotonic() - self.last_emit >= 5.0:
            self._emit("periodic_plan_refresh")


def parse_scan_order(raw: str) -> tuple[str, ...]:
    order = tuple(item.strip().upper() for item in raw.replace(";", ",").split(",") if item.strip())
    if set(order) != set(SHELF_VIEWPOINTS) or len(order) != len(SHELF_VIEWPOINTS):
        raise ValueError("scan order must contain A,B,C,D,E exactly once")
    return order


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 随机货架 plan-only 扫描规划器")
    parser.add_argument("--expected-target-count", type=int, default=5)
    parser.add_argument("--dwell-seconds", type=float, default=1.0)
    parser.add_argument("--scan-order", default=",".join(DEFAULT_SCAN_ORDER))
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()
    scan_order = parse_scan_order(args.scan_order)
    rclpy.init()
    node = ShelfScanPlanner(args.expected_target_count, args.dwell_seconds, scan_order)
    deadline = time.monotonic() + max(1.0, args.duration) if args.duration > 0 else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
