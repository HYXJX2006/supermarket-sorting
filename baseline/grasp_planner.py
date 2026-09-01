#!/usr/bin/env python3
"""DG-202606 plan-only 抓取规划器。

复用官方固定 Baseline 的 DEPLOY_OFFSET、GRASP_ROT 和 MMK2Kdl，
把视觉/导航阶段输出的商品世界坐标转换为抓取动作计划。

安全边界：本节点只订阅和发布 JSON 计划，不发布任何机械臂、夹爪、
升降柱或底盘控制命令。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from grasp_geometry import geometry_for_kind

TASK_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
# 真实动作 worker 在 Client 镜像中运行；该镜像默认没有 Server 的
# /workspace/supermarket_sorting_task。优先使用镜像内官方路径，缺失时
# 回退到随 baseline 一起挂载的官方源码副本，避免 deploy/creep 因导入失败退出。
if not (TASK_DIR / "mmk2_kdl.py").is_file():
    bundled_task_dir = Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"
    if (bundled_task_dir / "mmk2_kdl.py").is_file():
        TASK_DIR = bundled_task_dir
sys.path.insert(0, str(TASK_DIR))
from mmk2_kdl import MMK2Kdl  # noqa: E402


GRASP_ROT = np.eye(3)
SLIDE_GRASP = 0.11
SLIDE_MIN = -0.04
SLIDE_MAX = 0.87
SLIDE_STEP = 0.02
SLIDE_LIFT = -0.04
INIT_ARM_R = np.array([0.0, -0.166, 0.032, 0.0, -1.571, -2.223], dtype=float)
LIFT_AMOUNT = 0.05

META_TOPIC = "/competition/grasp_goal_meta"
PLAN_TOPIC = "/competition/grasp_plan"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
JOINT_TOPIC = "/joint_states"


class GraspPlanner(Node):
    def __init__(self, publish_plan: bool = True) -> None:
        super().__init__("grasp_planner_plan_only")
        self.publish_plan_enabled = publish_plan
        self.base_xy = np.array([1.92, -3.17], dtype=float)
        self.base_yaw = math.pi / 2.0
        self.slide = SLIDE_GRASP
        self.right_arm = INIT_ARM_R.copy()
        self.kdl = MMK2Kdl()
        self.last_target_id = ""
        self.plan_pub = self.create_publisher(String, PLAN_TOPIC, 10)
        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, 10)
        self.get_logger().warning(
            "PLAN-ONLY 抓取规划器：只计算 IK 和动作计划，不发送机械臂/夹爪/升降柱命令"
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.base_xy[:] = [float(pose.position.x), float(pose.position.y)]
        q = pose.orientation
        self.base_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_joints(self, message: JointState) -> None:
        values = {name: float(message.position[i]) for i, name in enumerate(message.name) if i < len(message.position)}
        self.slide = values.get("slide_joint", self.slide)
        self.right_arm = np.array(
            [values.get(f"right_arm_joint{i + 1}", self.right_arm[i]) for i in range(6)],
            dtype=float,
        )

    def _world_to_footprint(self, point: np.ndarray) -> np.ndarray:
        d = point - np.array([self.base_xy[0], self.base_xy[1], 0.0])
        c, s = math.cos(-self.base_yaw), math.sin(-self.base_yaw)
        return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])

    def _ik_for_world(self, world_point: np.ndarray) -> dict[str, Any]:
        """按正式 deploy 控制器相同的升降柱搜索策略做只读 IK。"""
        footprint = self._world_to_footprint(world_point)
        transform = np.eye(4)
        transform[:3, :3] = GRASP_ROT
        transform[:3, 3] = footprint
        current_slide = float(self.slide)
        slide_candidates = [
            round(SLIDE_MIN + index * SLIDE_STEP, 3)
            for index in range(int(round((SLIDE_MAX - SLIDE_MIN) / SLIDE_STEP)) + 1)
        ]
        slide_candidates.sort(key=lambda value: abs(value - current_slide))
        last_exception = None
        for slide_candidate in slide_candidates:
            ref = np.array([slide_candidate, *self.right_arm], dtype=float)
            try:
                solutions = self.kdl.inverse_kinematics(
                    T_left=None,
                    T_right=transform,
                    ref_pos=ref,
                    target_height=float(slide_candidate),
                )
            except Exception as exc:
                last_exception = exc
                continue
            if solutions is None or len(solutions) == 0:
                continue
            joints = np.asarray(solutions[0], dtype=float)
            try:
                _, fk_right = self.kdl.forward_kinematics(joints, index="right")
                position_error = float(np.linalg.norm(fk_right[:3, 3] - footprint))
            except Exception:
                position_error = None
            return {
                "reachable": True,
                "footprint": [round(float(v), 5) for v in footprint],
                "slide": round(float(joints[0]), 5),
                "right_arm_joints": [round(float(v), 5) for v in joints[1:7]],
                "ik_position_error_m": (
                    round(position_error, 6) if position_error is not None else None
                ),
            }
        suffix = f": {last_exception}" if last_exception else ""
        return {
            "reachable": False,
            "error": f"IK returned no solution{suffix}",
            "footprint": [round(float(v), 5) for v in footprint],
        }

    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning("忽略非法抓取元信息 JSON")
            return
        target_id = str(payload.get("target_id", ""))
        if not target_id or target_id == self.last_target_id:
            return
        world_raw = payload.get("object_world")
        if not isinstance(world_raw, list) or len(world_raw) != 3:
            self.get_logger().warning(f"目标 {target_id} 缺少合法 object_world")
            return
        kind = str(payload.get("kind", ""))
        geometry = geometry_for_kind(kind)
        object_world = np.array([float(v) for v in world_raw], dtype=float)
        forward = np.array([math.cos(self.base_yaw), math.sin(self.base_yaw), 0.0])
        object_center = object_world + geometry.surface_to_center_fwd * forward
        deploy_world = object_center + np.asarray(geometry.deploy_offset, dtype=float)
        lift_world = deploy_world.copy()
        lift_world[2] += LIFT_AMOUNT
        creep_stop = float(object_center[1] + geometry.creep_stop_dy)
        ik = self._ik_for_world(deploy_world)
        plan = {
            "schema_version": 1,
            "mode": "plan-only",
            "mechanical_commands_sent": False,
            "target_id": target_id,
            "kind": kind,
            "grasp_geometry": {
                "shape": "cylindrical_container" if kind.strip().lower() == "shupian" else "default",
                "surface_to_center_fwd": round(float(geometry.surface_to_center_fwd), 5),
                "deploy_offset": [round(float(v), 5) for v in geometry.deploy_offset],
                "creep_stop_dy": round(float(geometry.creep_stop_dy), 5),
            },
            "slot": payload.get("slot"),
            "input_object_world": [round(float(v), 5) for v in object_world],
            "object_center_world": [round(float(v), 5) for v in object_center],
            "deploy_world": [round(float(v), 5) for v in deploy_world],
            "lift_reference_world": [round(float(v), 5) for v in lift_world],
            "creep_stop_y": round(creep_stop, 5),
            "slide_grasp": SLIDE_GRASP,
            "slide_lift": SLIDE_LIFT,
            "grip": {"open": 1.0, "close": 0.08},
            "stages": ["deploy", "creep", "close_gripper", "lift", "retreat"],
            "ik": ik,
            "base_at_plan": {
                "x": round(float(self.base_xy[0]), 5),
                "y": round(float(self.base_xy[1]), 5),
                "yaw": round(float(self.base_yaw), 5),
            },
            "created_at": time.time(),
        }
        self.last_target_id = target_id
        if self.publish_plan_enabled:
            output = String()
            output.data = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))
            self.plan_pub.publish(output)
        self.get_logger().info(
            f"抓取计划已生成（不执行）：target={target_id} kind={plan['kind']} "
            f"deploy={plan['deploy_world']} IK={ik['reachable']}"
        )
        if not ik["reachable"]:
            self.get_logger().error(f"目标 {target_id} 暂不可达：{ik.get('error')}")


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 plan-only 抓取规划器")
    parser.add_argument("--no-publish", action="store_true", help="只打印计划，不发布 /competition/grasp_plan")
    args = parser.parse_args()
    rclpy.init()
    node = GraspPlanner(publish_plan=not args.no_publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
