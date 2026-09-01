#!/usr/bin/env python3
"""仅用于验证比赛业务状态机：发送 5 个已锁定目标，不发送任何控制命令。"""
from __future__ import annotations
import argparse, json, time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TASK_TOPIC = "/supermarket_sorting/task"
STATUS_TOPIC = "/competition/target_status"

class LogicStimulus(Node):
    def __init__(self, count: int) -> None:
        super().__init__("logic_flow_simulator")
        self.count = count
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_pub = self.create_publisher(String, TASK_TOPIC, qos)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.targets = []
        kinds = ["kele", "pingguo", "maidong", "shupian", "zhijin"]
        for i in range(count):
            self.targets.append({"id": f"logic_item_{i+1:02d}", "kind": kinds[i % len(kinds)]})

    def publish_all(self) -> None:
        task = String()
        task.data = json.dumps({"schema_version": 1, "run_prefix": "logic_run", "count": self.count, "targets": self.targets}, separators=(",", ":"))
        self.task_pub.publish(task)
        for i, target in enumerate(self.targets):
            candidate = {
                "kind": target["kind"], "confidence": 0.99,
                "world": [0.85 + i * 0.02, 3.20, 0.93],
                "object_world": [0.85 + i * 0.02, 3.20, 0.93],
                "aruco_ids": [28 + i], "slot": f"D/L2/C{i+1}", "samples": 10,
                "navigation_goal": [0.852, 2.475, 3.08],
            }
            msg = String()
            msg.data = json.dumps({
                "schema_version": 1, "reason": "logic_sim_candidate",
                "current_target": {"id": target["id"], "kind": target["kind"], "status": "searching", "attempts": 0, "error": None, "candidate": candidate},
                "targets": [],
            }, separators=(",", ":"))
            self.status_pub.publish(msg)
            time.sleep(0.15)
        self.get_logger().info(f"LOGIC_STIMULUS_SENT={self.count}")

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args()
    rclpy.init()
    node = LogicStimulus(max(1, args.count))
    try:
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.publish_all()
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
