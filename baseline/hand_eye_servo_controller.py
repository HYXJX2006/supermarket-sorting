#!/usr/bin/env python3
"""横向对准控制器（世界系 FK 闭环，2026-09-13 定案）。

替代旧的手眼 2D 团块推算（像素测量与被控对象耦合，方向/灵敏度靠手标，
实测 +3.2/-9.6 振荡）。新设计三层：
  1. 商品世界坐标：头相机链路（ArUco 货位 + YOLO/RGB-D + 近距复核）在
     遮挡发生前测量并冻结——本控制器不做任何视觉测量；
  2. 对准闭环：error_x = 目标x − FK(/joint_states 实测关节, odom)。
     仿真连杆刚性，FK(实测角) 即手臂真实位形（关节跟随误差可见）。
     Δj3 = error_x / 数值FK灵敏度，阻尼+限幅，发→等→FK复测，≤10mm 收敛；
  3. "罐在不在指间"由腱反馈二值判定（夹空检测），图像仅存诊断。
世界系约定：机器人面向货架 yaw≈+90°，世界 +X = 机器人右手边。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

TASK_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
if not (TASK_DIR / "mmk2_kdl.py").is_file():
    bundled = Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"
    if (bundled / "mmk2_kdl.py").is_file():
        TASK_DIR = bundled
sys.path.insert(0, str(TASK_DIR))
from mmk2_kdl import MMK2Kdl  # noqa: E402

from grasp_geometry import geometry_for_kind  # noqa: E402

META_TOPIC = "/competition/grasp_goal_meta"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
ARM_CMD_TOPIC = "/right_arm_forward_position_controller/commands"
RESULT_DIR = Path("/workspace/baseline/debug_data")
RESULT_PATH = RESULT_DIR / "servo_result.json"


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class WorldFrameAligner(Node):
    def __init__(self, *, timeout: float, max_iterations: int,
                 tol_m: float, dj3_max: float, damping: float) -> None:
        super().__init__("hand_eye_servo")
        self.timeout = float(timeout)
        self.max_iterations = int(max_iterations)
        self.tol_m = float(tol_m)
        self.dj3_max = float(dj3_max)
        self.damping = float(damping)
        self.started_at = time.monotonic()

        self.kdl = MMK2Kdl()
        self.joints: dict[str, float] = {}
        self.x = self.y = self.yaw = None
        self.kind = ""
        self.object_world: tuple[float, float, float] | None = None

        self.arm_cmd_pub = self.create_publisher(Float64MultiArray, ARM_CMD_TOPIC, 10)
        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)

    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        world = payload.get("object_world")
        if isinstance(world, list) and len(world) == 3:
            self.kind = str(payload.get("kind", "")).strip().lower()
            self.object_world = (float(world[0]), float(world[1]), float(world[2]))

    def _on_joints(self, message: JointState) -> None:
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _on_odom(self, message: Odometry) -> None:
        p = message.pose.pose
        self.x, self.y = float(p.position.x), float(p.position.y)
        self.yaw = yaw_from_quaternion(p.orientation)

    def _arm_q(self) -> list[float] | None:
        names = [f"right_arm_joint{i}" for i in range(1, 7)]
        if "slide_joint" not in self.joints or any(n not in self.joints for n in names):
            return None
        return [self.joints["slide_joint"]] + [self.joints[n] for n in names]

    def _ee_world(self, q: list[float]) -> tuple[float, float, float] | None:
        if self.x is None or self.yaw is None:
            return None
        try:
            _, t = self.kdl.forward_kinematics(q, index="right")
        except Exception:
            return None
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (
            self.x + c * float(t[0, 3]) - s * float(t[1, 3]),
            self.y + s * float(t[0, 3]) + c * float(t[1, 3]),
            float(t[2, 3]),
        )

    def _j3_world_x_sensitivity(self, q: list[float]) -> float:
        h = 0.02
        try:
            _, t0 = self.kdl.forward_kinematics(q, index="right")
            q2 = list(q)
            q2[3] += h
            _, t2 = self.kdl.forward_kinematics(q2, index="right")
        except Exception:
            return 0.0
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        wx0 = c * t0[0, 3] - s * t0[1, 3]
        wx2 = c * t2[0, 3] - s * t2[1, 3]
        return (wx2 - wx0) / h

    def run(self) -> int:
        result: dict[str, object] = {"measured": False, "converged": False}
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and (
            self.object_world is None or self._arm_q() is None or self.x is None
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.object_world is None or self._arm_q() is None or self.x is None:
            result["reason"] = "meta/joint_states/odom 未就绪"
            self._write_result(result)
            return 0

        geometry = geometry_for_kind(self.kind) if self.kind else None
        deploy_dx = geometry.deploy_offset[0] if geometry is not None else -0.011
        target_x = self.object_world[0] + float(deploy_dx)

        iterations = []
        total_dx = 0.0
        converged = False
        for iteration in range(self.max_iterations):
            q = self._arm_q()
            ee = self._ee_world(q)
            if ee is None:
                result["reason"] = "FK 失败（关节/odom 缺失）"
                break
            err_x = target_x - ee[0]
            iterations.append({
                "ee_x": round(ee[0], 4), "ee_y": round(ee[1], 4), "ee_z": round(ee[2], 4),
                "err_x_m": round(err_x, 4),
                "err_y_m": round(self.object_world[1] - ee[1], 4),
                "err_z_m": round(self.object_world[2] - ee[2], 4),
            })
            if abs(err_x) <= self.tol_m:
                converged = True
                break
            sens = self._j3_world_x_sensitivity(q)
            if abs(sens) < 0.05:
                result["reason"] = f"j3 横向灵敏度过低（{sens:.3f}）"
                break
            dj3 = max(-self.dj3_max, min(self.dj3_max, err_x / sens * self.damping))
            if abs(dj3) < 1e-3:
                converged = True
                break
            arm_msg = Float64MultiArray()
            arm_msg.data = [q[1], q[2], q[3] + dj3, q[4], q[5], q[6], 1.0]
            self.arm_cmd_pub.publish(arm_msg)
            total_dx += dj3 * sens
            self.get_logger().info(
                f"世界系横向校准 {iteration+1}: dj3={dj3:+.4f} "
                f"(err_x={err_x*100:+.1f}cm, sens={sens:.3f} m/rad)"
            )
            t_move = time.monotonic()
            while time.monotonic() - t_move < 1.2 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

        result.update({
            "measured": True,
            "converged": converged,
            "iterations": iterations,
            "correction": [round(total_dx, 4), 0.0, 0.0],
            "kind": self.kind,
            "target_x": round(target_x, 4),
            "ee_world": [round(v, 4) for v in (self._ee_world(self._arm_q()) or (0, 0, 0))],
        })
        self._write_result(result)
        return 0

    def _write_result(self, result: dict[str, object]) -> None:
        result["schema_version"] = 2
        result["wall_time"] = time.time()
        try:
            RESULT_DIR.mkdir(parents=True, exist_ok=True)
            RESULT_PATH.write_text(
                json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            self.get_logger().info(
                f"伺服结果已写入 {RESULT_PATH}: {json.dumps(result, ensure_ascii=False)[:260]}"
            )
        except OSError as exc:
            self.get_logger().error(f"写入伺服结果失败：{exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="世界系 FK 横向对准（替代手眼 2D 推算）")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--max-iterations", type=int,
                        default=int(os.getenv("SUPERMARKET_SERVO_MAX_ITER", "3")))
    parser.add_argument("--tol-m", type=float,
                        default=float(os.getenv("SUPERMARKET_SERVO_TOL_M", "0.010")))
    parser.add_argument("--dj3-max", type=float,
                        default=float(os.getenv("SUPERMARKET_SERVO_DJ3_MAX", "0.08")))
    parser.add_argument("--damping", type=float,
                        default=float(os.getenv("SUPERMARKET_SERVO_DAMPING", "0.9")))
    args = parser.parse_args()
    if args.execute and args.confirm != "servo":
        print("拒绝执行：必须使用 --confirm servo")
        return 2
    rclpy.init()
    node = WorldFrameAligner(
        timeout=args.timeout,
        max_iterations=args.max_iterations,
        tol_m=args.tol_m,
        dj3_max=args.dj3_max,
        damping=args.damping,
    )
    try:
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.run()
    except Exception:
        code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
