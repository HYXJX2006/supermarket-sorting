#!/usr/bin/env python3
"""配送段底盘冻结的二分诊断。

用途：抓取成功后底盘在走廊以 ~1% 速度爬行然后冻住。需要区分两种可能：
  A. 导航节点在发零速（上游逻辑问题：目标判定/避障/到达误判）
  B. 导航发了非零速度，但底盘不执行（下游问题：/cmd_vel 被覆盖、
     server 侧速度接口、或仿真里底盘被"冻结"）

方法：
  1. 持续监听 /cmd_vel，记录导航实际发布的值（含发布者时间戳）
  2. 每 2 秒对比 odom 位移，判定"是否真的不动"
  3. 冻结时：绕过导航，直接向 /cmd_vel 发一个测试速度（0.2 m/s，1.5 秒）
     → 若 odom 动了 = 底盘能执行，问题在导航逻辑（A）
     → 若 odom 仍不动 = 底盘不执行 / server 侧问题（B）

用法（挂在 client 容器里跑）：
  source /opt/ros/humble/setup.bash
  python3 -u /workspace/baseline/diag_delivery_freeze.py
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

CMD_TOPIC = os.getenv("DIAG_CMD_TOPIC", "/cmd_vel")
ODOM_TOPIC = os.getenv("DIAG_ODOM_TOPIC", "/slamware_ros_sdk_server_node/odom")
TEST_DURATION = float(os.getenv("DIAG_TEST_SEC", "1.5"))
TEST_SPEED = float(os.getenv("DIAG_TEST_SPEED", "0.20"))
STALL_SEC = float(os.getenv("DIAG_STALL_SEC", "8.0"))
STALL_DIST = float(os.getenv("DIAG_STALL_DIST", "0.02"))


class DiagNode(Node):
    def __init__(self) -> None:
        super().__init__("diag_delivery_freeze")
        # 导航发布 /cmd_vel 用的是 RELIABLE；订阅端若用 BEST_EFFORT 会
        # "requesting incompatible QoS. No messages will be sent"（实测踩过）。
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_sub = self.create_subscription(Twist, CMD_TOPIC, self._on_cmd, qos)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, qos)
        self.last_cmd = None
        self.last_cmd_at = None
        self.cmd_count = 0
        self.x = self.y = None
        # 冻结检测用的基准点
        self.anchor = None
        self.anchor_at = None
        self.tested = False

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_count += 1
        self.last_cmd = (float(msg.linear.x), float(msg.angular.z))
        self.last_cmd_at = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

    def _check(self) -> None:
        now = time.monotonic()
        if self.x is None:
            return
        # 维护"位移基准"：超过 STALL_DIST 就重置基准（说明在动）
        if self.anchor is None:
            self.anchor = (self.x, self.y, now)
            self.anchor_at = now
            return
        moved = math.hypot(self.x - self.anchor[0], self.y - self.anchor[1])
        if moved > STALL_DIST:
            self.anchor = (self.x, self.y, now)
            self.anchor_at = now
            self.tested = False
            return

        frozen_for = now - self.anchor_at
        cmd_age = (now - self.last_cmd_at) if self.last_cmd_at else None
        if frozen_for < STALL_SEC:
            return

        # 冻结已超过阈值 —— 输出诊断快照
        cmd = self.last_cmd
        cmd_str = f"linear={cmd[0]:.4f} angular={cmd[1]:.4f}" if cmd else "(从未收到)"
        print(
            f"\n===== 冻结快照 ====="
            f"\n  已冻结 {frozen_for:.1f}s（位移 <{STALL_DIST}m）"
            f"\n  位置 ({self.x:.3f},{self.y:.3f})"
            f"\n  /cmd_vel 累计 {self.cmd_count} 条，最新 {cmd_str}"
            f"\n  距上一条 cmd_vel {('%.2fs' % cmd_age) if cmd_age is not None else 'n/a'}",
            flush=True,
        )

        if self.tested:
            return
        self.tested = True

        # 二分：直接发测试速度看底盘动不动
        print(
            f"\n>>> 二分测试：直接向 {CMD_TOPIC} 发 {TEST_SPEED}m/s，持续 {TEST_DURATION}s",
            flush=True,
        )
        before = (self.x, self.y)
        twist = Twist()
        twist.linear.x = TEST_SPEED
        t_end = time.monotonic() + TEST_DURATION
        while time.monotonic() < t_end:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.2)
        after = (self.x, self.y) if self.x is not None else before
        delta = math.hypot(after[0] - before[0], after[1] - before[1])
        print(f"    直发指令后位移 = {delta * 100:.1f}cm", flush=True)
        if delta > 0.05:
            print(
                "  ✅ 结论：底盘能执行 /cmd_vel → 问题在【导航节点发了零速/停止】(A)",
                flush=True,
            )
        else:
            print(
                "  ❌ 结论：底盘不执行 /cmd_vel → 问题在【下游/server 侧】(B)",
                flush=True,
            )
        # 停住，避免测试速度把车带走
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = DiagNode()
    print(
        json.dumps(
            {
                "cmd_topic": CMD_TOPIC,
                "odom_topic": ODOM_TOPIC,
                "stall_sec": STALL_SEC,
                "stall_dist": STALL_DIST,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    # 只跑有限时长，避免长驻影响正式流程
    budget = float(os.getenv("DIAG_BUDGET_SEC", "900"))
    started = time.monotonic()
    while time.monotonic() - started < budget:
        rclpy.spin_once(node, timeout_sec=0.5)
        node._check()
    print("诊断结束（预算用尽）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
