#!/usr/bin/env python3
"""Publish one known-slot target for single-item plan-only/IK integration tests.

This helper only publishes task and grasp metadata. It never publishes cmd_vel,
arm, gripper, lift, place, or other actuator commands.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TASK_TOPIC = "/supermarket_sorting/task"
GRASP_META_TOPIC = "/competition/grasp_goal_meta"
NAV_META_TOPIC = "/competition/navigation_goal_meta"


class KnownSlotPublisher(Node):
    def __init__(self, *, target_id: str, kind: str, slot: str, world: list[float], nav: list[float],
                 window_seconds: float = 40.0) -> None:
        super().__init__("known_slot_plan_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.task_pub = self.create_publisher(String, TASK_TOPIC, qos)
        self.grasp_pub = self.create_publisher(String, GRASP_META_TOPIC, qos)
        self.nav_pub = self.create_publisher(String, NAV_META_TOPIC, qos)
        self.target_id = target_id
        self.kind = kind
        self.slot = slot
        self.world = [float(v) for v in world]
        self.nav = [float(v) for v in nav]
        self.task = String()
        self.task.data = json.dumps(
            {
                "schema_version": 1,
                "run_prefix": "known_slot_plan_only",
                "count": 1,
                "targets": [{"id": target_id, "kind": kind}],
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.meta = String()
        self.meta.data = json.dumps(
            {
                "schema_version": 1,
                "target_id": target_id,
                "kind": kind,
                "slot": slot,
                "object_world": self.world,
                "navigation_goal": self.nav,
                "plan_only_allowed": True,
                "local_recheck_required": False,
                "source": "known_slot_plan_only",
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.nav_meta = String()
        self.nav_meta.data = json.dumps(
            {
                "schema_version": 1,
                "target_id": target_id,
                "kind": kind,
                "slot": slot,
                "navigation_goal": self.nav,
                "source": "known_slot_plan_only",
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.started = time.monotonic()
        # 元数据窗口必须覆盖整条动作链（deploy→creep→close_gripper→lift）。
        # 高层(L3)货位的 creep 单独就可能耗时 60s 以上，默认 40s 会让
        # lift_controller 在订阅之前就失去元数据来源，从而退回构造函数里的
        # 默认门禁值（0.85），表现为"几何修复不生效"。
        self.window_seconds = max(5.0, float(window_seconds))
        self.create_timer(0.2, self._publish)

    def _publish(self) -> None:
        self.task_pub.publish(self.task)
        self.grasp_pub.publish(self.meta)
        self.nav_pub.publish(self.nav_meta)
        if time.monotonic() - self.started > self.window_seconds:
            self.get_logger().info(
                f"known-slot metadata publishing window complete ({self.window_seconds:.0f}s)"
            )
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="known-slot plan-only publisher")
    parser.add_argument("--target-id", default="known_product_032")
    parser.add_argument("--kind", default="kele")
    parser.add_argument("--slot", default="D/L2/C2")
    parser.add_argument("--world", nargs=3, type=float, default=[0.92, 3.243, 0.9235])
    parser.add_argument("--navigation-goal", nargs=3, type=float, default=[1.04, 2.60, 1.5707963267948966])
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=float(os.getenv("SUPERMARKET_META_WINDOW_SECONDS", "600")),
        help="元数据持续发布时长；must outlive the whole action chain",
    )
    args = parser.parse_args()
    rclpy.init()
    node = KnownSlotPublisher(
        target_id=args.target_id,
        kind=args.kind,
        slot=args.slot,
        world=args.world,
        nav=args.navigation_goal,
        window_seconds=args.window_seconds,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
