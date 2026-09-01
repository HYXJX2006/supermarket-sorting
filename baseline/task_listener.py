#!/usr/bin/env python3
"""智慧零售任务管理器：接收任务并维护每个目标的生命周期状态。"""

import argparse
import json
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TASK_TOPIC = "/supermarket_sorting/task"
VALID_STATUSES = {
    "pending",
    "searching",
    "grasping",
    "delivering",
    "done",
    "failed",
}


class TaskManager(Node):
    def __init__(self, once: bool = False) -> None:
        super().__init__("task_manager")
        self.once = once
        self.received = False
        self.run_prefix: str | None = None
        self.schema_version: int | None = None
        self.targets: list[dict[str, Any]] = []

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            String,
            TASK_TOPIC,
            self._on_task,
            qos,
        )
        self.get_logger().info(f"Listening on {TASK_TOPIC}")

    def _on_task(self, message: String) -> None:
        try:
            task = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"任务消息不是合法 JSON: {exc}")
            return

        if not isinstance(task, dict):
            self.get_logger().error("任务 JSON 顶层必须是对象")
            return

        raw_targets = task.get("targets")
        if not isinstance(raw_targets, list):
            self.get_logger().error("任务字段 targets 必须是数组")
            return

        normalized: list[dict[str, Any]] = []
        seen_ids: set[str] = set()
        for index, raw_target in enumerate(raw_targets):
            if not isinstance(raw_target, dict):
                self.get_logger().error(f"第 {index} 个目标不是对象，拒绝整条任务")
                return
            target_id = raw_target.get("id")
            kind = raw_target.get("kind")
            if not isinstance(target_id, str) or not target_id:
                self.get_logger().error(f"第 {index} 个目标缺少合法 id")
                return
            if not isinstance(kind, str) or not kind:
                self.get_logger().error(f"目标 {target_id} 缺少合法 kind")
                return
            if target_id in seen_ids:
                self.get_logger().error(f"目标 id 重复: {target_id}")
                return
            seen_ids.add(target_id)
            normalized.append(
                {
                    "id": target_id,
                    "kind": kind,
                    "status": "pending",
                    "attempts": 0,
                    "error": None,
                }
            )

        declared_count = task.get("count", len(normalized))
        if declared_count != len(normalized):
            self.get_logger().warning(
                f"count={declared_count} 与 targets 数量={len(normalized)} 不一致，使用实际 targets 数量"
            )

        self.schema_version = task.get("schema_version")
        self.run_prefix = task.get("run_prefix")
        self.targets = normalized
        self.received = True
        self._print_snapshot("任务已接收")

    def next_target(self) -> dict[str, Any] | None:
        for target in self.targets:
            if target["status"] == "pending":
                return target
        return None

    def set_status(self, target_id: str, status: str, error: str | None = None) -> bool:
        if status not in VALID_STATUSES:
            raise ValueError(f"未知目标状态: {status}")
        for target in self.targets:
            if target["id"] == target_id:
                target["status"] = status
                target["error"] = error
                if status == "failed":
                    target["attempts"] += 1
                self._print_snapshot(f"目标状态更新: {target_id} -> {status}")
                return True
        return False

    def snapshot(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "run_prefix": self.run_prefix,
            "count": len(self.targets),
            "targets": self.targets,
        }

    def _print_snapshot(self, title: str) -> None:
        print(f"=== {title} ===", flush=True)
        print(json.dumps(self.snapshot(), ensure_ascii=False, indent=2), flush=True)
        next_target = self.next_target()
        if next_target:
            print(
                f"NEXT_TARGET id={next_target['id']} kind={next_target['kind']}",
                flush=True,
            )
        else:
            print("NEXT_TARGET none", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="维护智慧零售任务目标状态")
    parser.add_argument(
        "--once",
        action="store_true",
        help="收到第一条任务后打印状态并退出",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="非 once 模式最多等待多少秒",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TaskManager(once=args.once)
    started = time.monotonic()
    try:
        while rclpy.ok() and not (args.once and node.received):
            rclpy.spin_once(node, timeout_sec=0.5)
            if not args.once and time.monotonic() - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.received else 2


if __name__ == "__main__":
    sys.exit(main())
