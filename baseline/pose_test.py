#!/usr/bin/env python3
"""机械臂姿态试验台：发一组右臂关节目标并保持，用于人工目视挑选"收臂运输姿态"。

背景：官方 safe_pose 的 INIT_ARM_R 里 j5=-1.571(=-90°) 是"小臂水平外伸"，
车体外廓因此变宽，配送时蹭走廊随机障碍被卡死。需要一个真正贴身的姿态。

用法（client 容器内）：
  python3 pose_test.py "0,-0.166,0.032,0,0,-2.223" 8      # 关节值 + 保持秒数
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

RIGHT_TOPIC = "/right_arm_forward_position_controller/commands"
GRIP_TOPIC = "/right_gripper_forward_position_controller/commands" if False else None


class PoseTest(Node):
    def __init__(self, target: list[float], hold: float) -> None:
        super().__init__("pose_test")
        self.target = target
        self.hold = hold
        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.pub = self.create_publisher(Float64MultiArray, RIGHT_TOPIC, 10)
        self.started = time.monotonic()
        self.last_log = 0.0

    def _on_js(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joints[name] = float(msg.position[i])

    def tick(self) -> bool:
        if not self.joints:
            return False
        # 臂命令 = 6 关节 + 1 夹爪 = 7 个值；只发 6 个会被 server 拒
        # （[ERROR] MMK2_mujoco_node: right arm command length error）。
        grip = float(self.joints.get("right_arm_eef_gripper_joint", 0.0))
        self.pub.publish(Float64MultiArray(data=[float(v) for v in self.target] + [grip]))
        now = time.monotonic()
        if now - self.last_log > 1.5:
            self.last_log = now
            cur = [self.joints.get(f"right_arm_joint{i}", float("nan")) for i in range(1, 7)]
            err = max(abs(cur[i] - self.target[i]) for i in range(6))
            self.get_logger().info(
                "目标=" + ",".join(f"{v:.3f}" for v in self.target)
                + " 当前=" + ",".join(f"{v:.3f}" for v in cur)
                + f" max_err={err:.3f}"
            )
        return now - self.started > self.hold


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    target = [float(v) for v in sys.argv[1].split(",")]
    if len(target) != 6:
        print("需要 6 个关节值")
        return 2
    hold = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    rclpy.init()
    node = PoseTest(target, hold)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.tick():
            break
    print("姿态试验结束（保持 %.1fs）" % hold, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
