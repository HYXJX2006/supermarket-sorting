#!/usr/bin/env python3
"""只读抓取/安全姿态评估器。

默认只订阅 /joint_states，不发布任何底盘、机械臂或夹爪命令。
可用于 safe_pose、deploy、lift 等阶段的关节误差和 FK 误差审计。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from statistics import mean

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "slide_joint", "head_yaw_joint", "head_pitch_joint",
    "left_arm_joint1", "left_arm_joint2", "left_arm_joint3",
    "left_arm_joint4", "left_arm_joint5", "left_arm_joint6",
    "left_arm_eef_gripper_joint", "right_arm_joint1",
    "right_arm_joint2", "right_arm_joint3", "right_arm_joint4",
    "right_arm_joint5", "right_arm_joint6", "right_arm_eef_gripper_joint",
]
SAFE_POSE = {
    "slide_joint": 0.11,
    "head_yaw_joint": 0.0,
    "head_pitch_joint": 0.0,
    "left_arm_joint1": 0.0, "left_arm_joint2": -0.166,
    "left_arm_joint3": 0.032, "left_arm_joint4": 0.0,
    "left_arm_joint5": 1.571, "left_arm_joint6": 2.223,
    "left_arm_eef_gripper_joint": 1.0,
    "right_arm_joint1": 0.0, "right_arm_joint2": -0.166,
    "right_arm_joint3": 0.032, "right_arm_joint4": 0.0,
    "right_arm_joint5": -1.571, "right_arm_joint6": -2.223,
    "right_arm_eef_gripper_joint": 1.0,
}


def _load_kdl():
    candidates = [
        "/workspace/supermarket_sorting_task/examples/supermarket_sorting",
        str(Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"),
    ]
    for path in candidates:
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        from mmk2_kdl import MMK2Kdl
        return MMK2Kdl()
    except Exception:
        return None


class GraspEvaluator(Node):
    def __init__(self, stage: str, duration: float, output: Path, expected: dict[str, float],
                 joint_tolerance: float = 0.08, stable_tolerance: float = 0.08):
        super().__init__("grasp_evaluator")
        self.stage = stage
        self.duration = max(0.5, float(duration))
        self.output = output
        self.expected = expected
        self.joint_tolerance = max(0.001, float(joint_tolerance))
        self.stable_tolerance = max(0.001, float(stable_tolerance))
        self.started = time.monotonic()
        self.last_message = 0.0
        self.samples: list[dict] = []
        self.kdl = _load_kdl()
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data)
        self.get_logger().info(f"只读抓取评估启动：stage={stage} duration={self.duration:.1f}s output={output}")
        if self.kdl is None:
            self.get_logger().warning("MMK2Kdl 不可用：将只记录关节反馈，FK 字段为 null")

    def _on_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        positions = {name: float(msg.position[i]) for i, name in enumerate(msg.name) if i < len(msg.position)}
        if not positions:
            return
        missing = sorted(set(self.expected) - set(positions))
        errors = {name: abs(positions[name] - target) for name, target in self.expected.items() if name in positions}
        left = [errors[n] for n in errors if n.startswith("left_arm_joint")]
        right = [errors[n] for n in errors if n.startswith("right_arm_joint")]
        left_arm_names = [f"left_arm_joint{i}" for i in range(1, 7)] + ["left_arm_eef_gripper_joint"]
        right_arm_names = [f"right_arm_joint{i}" for i in range(1, 7)] + ["right_arm_eef_gripper_joint"]
        left_missing = [name for name in left_arm_names if name not in positions]
        right_missing = [name for name in right_arm_names if name not in positions]
        left_arm_error = max((errors.get(name, float("inf")) for name in left_arm_names), default=float("inf"))
        right_arm_error = max((errors.get(name, float("inf")) for name in right_arm_names), default=float("inf"))
        sample = {
            "elapsed_s": round(now - self.started, 4),
            "joint_count": len(positions),
            "missing_joints": missing,
            "left_arm_missing_joints": left_missing,
            "right_arm_missing_joints": right_missing,
            "joint_errors_rad": {k: round(v, 6) for k, v in errors.items()},
            "max_error_rad": round(max(errors.values()), 6) if errors else None,
            "left_arm_max_error_rad": round(max(left), 6) if left else None,
            "right_arm_max_error_rad": round(max(right), 6) if right else None,
            "left_arm_pass": not left_missing and left_arm_error <= self.joint_tolerance,
            "right_arm_pass": not right_missing and right_arm_error <= self.joint_tolerance,
            "all_joints_pass": not missing and bool(errors) and max(errors.values()) <= self.joint_tolerance,
            "positions": positions,
            "fk": self._fk(positions),
        }
        self.samples.append(sample)
        if now - self.last_message >= 1.0:
            self.get_logger().info(
                f"反馈审计：samples={len(self.samples)} left_max={sample['left_arm_max_error_rad']} "
                f"right_max={sample['right_arm_max_error_rad']} max={sample['max_error_rad']}"
            )
            self.last_message = now

    def _fk(self, positions: dict[str, float]) -> dict | None:
        if self.kdl is None:
            return None
        try:
            slide = positions.get("slide_joint", self.expected.get("slide_joint", 0.11))
            left_q = np.array([slide] + [positions[f"left_arm_joint{i}"] for i in range(1, 7)], dtype=float)
            right_q = np.array([slide] + [positions[f"right_arm_joint{i}"] for i in range(1, 7)], dtype=float)
            left, _ = self.kdl.forward_kinematics(left_q, index="left")
            _, right = self.kdl.forward_kinematics(right_q, index="right")
            return {
                "left_position": [round(float(x), 6) for x in left[:3, 3]],
                "right_position": [round(float(x), 6) for x in right[:3, 3]],
            }
        except Exception as exc:
            return {"error": f"{type(exc).__name__}: {exc}"}

    def finished(self) -> bool:
        return time.monotonic() - self.started >= self.duration

    def summary(self) -> dict:
        def vals(key):
            return [float(s[key]) for s in self.samples if s.get(key) is not None]
        maxes = vals("max_error_rad")
        left = vals("left_arm_max_error_rad")
        right = vals("right_arm_max_error_rad")
        latest = self.samples[-1] if self.samples else None
        stable_samples = [s for s in self.samples if s.get("all_joints_pass")]
        left_pass = bool(latest and latest.get("left_arm_pass"))
        right_pass = bool(latest and latest.get("right_arm_pass"))
        return {
            "schema_version": 1,
            "read_only": True,
            "stage": self.stage,
            "expected_pose": self.expected,
            "joint_tolerance_rad": self.joint_tolerance,
            "stable_tolerance_rad": self.stable_tolerance,
            "sample_count": len(self.samples),
            "stable_sample_count": len(stable_samples),
            "left_arm_pass": left_pass,
            "right_arm_pass": right_pass,
            "safe_pose_pass": left_pass and right_pass and len(stable_samples) >= 3,
            "duration_s": round(time.monotonic() - self.started, 4),
            "max_error_rad": {"latest": latest.get("max_error_rad") if latest else None, "mean": round(mean(maxes), 6) if maxes else None, "peak": round(max(maxes), 6) if maxes else None},
            "left_arm_max_error_rad": {"latest": latest.get("left_arm_max_error_rad") if latest else None, "mean": round(mean(left), 6) if left else None, "peak": round(max(left), 6) if left else None},
            "right_arm_max_error_rad": {"latest": latest.get("right_arm_max_error_rad") if latest else None, "mean": round(mean(right), 6) if right else None, "peak": round(max(right), 6) if right else None},
            "latest": latest,
            "samples": self.samples,
        }

    def write_summary(self) -> dict:
        payload = self.summary()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return payload


def main() -> int:
    parser = argparse.ArgumentParser(description="只读关节/FK抓取评估")
    parser.add_argument("--stage", default="safe_pose")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--output", default="/workspace/baseline/debug_data/grasp_evaluation/safe_pose_audit.json")
    parser.add_argument("--joint-tolerance", type=float, default=0.08)
    parser.add_argument("--stable-tolerance", type=float, default=0.08)
    args = parser.parse_args()
    rclpy.init()
    node = GraspEvaluator(
        args.stage, args.duration, Path(args.output), SAFE_POSE,
        joint_tolerance=args.joint_tolerance, stable_tolerance=args.stable_tolerance,
    )
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        payload = node.write_summary()
        node.get_logger().info(
            f"只读评估结束：samples={payload['sample_count']} "
            f"left_peak={payload['left_arm_max_error_rad']['peak']} "
            f"right_peak={payload['right_arm_max_error_rad']['peak']} "
            f"left_pass={payload['left_arm_pass']} right_pass={payload['right_arm_pass']} "
            f"safe_pose_pass={payload['safe_pose_pass']} output={node.output}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
