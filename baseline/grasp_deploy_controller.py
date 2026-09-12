#!/usr/bin/env python3
"""DG-202606 单次机械臂部署测试控制器。

仅执行：导航到位后，右臂移动到商品预抓取部署点，夹爪保持张开。
默认 plan-only；真实执行必须同时传入 --execute --confirm deploy。

禁止：底盘、闭爪、抬升、撤出、配送。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from grasp_geometry import GraspGeometry, geometry_for_kind

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

SPINE_TOPIC = "/spine_forward_position_controller/commands"
HEAD_TOPIC = "/head_forward_position_controller/commands"
LEFT_ARM_TOPIC = "/left_arm_forward_position_controller/commands"
RIGHT_ARM_TOPIC = "/right_arm_forward_position_controller/commands"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
JOINT_TOPIC = "/joint_states"
META_TOPIC = "/competition/grasp_goal_meta"
GRASP_GOAL_TOPIC = "/competition/grasp_goal"

_GRASP_ROT_MODE = os.getenv("SUPERMARKET_GRASP_ROT_MODE", "side")
if _GRASP_ROT_MODE == "top":
    # 真正的俯抓（Ry(-90°)）：指头长轴（局部 x）指向世界 -z（竖直向下），
    # 开合轴（局部 y）保持水平（世界 y）——两指沿商品前后壁从上方咬顶棱。
    # 注意 Rx(180°) 只是倒置，指头仍水平伸出，会贴着顶棱擦过（已实测失败）。
    # 咬棱距离与商品 yaw 无关 ⇒ 不需要拨动改朝向。
    GRASP_ROT = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
else:
    GRASP_ROT = np.eye(3)
SLIDE_GRASP = 0.11
SLIDE_MIN = -0.04
SLIDE_MAX = 0.87
SLIDE_STEP = 0.02
HEAD_PITCH = -0.6
GRIP_OPEN = 1.0

INIT_ARM_L = [0.0, -0.166, 0.032, 0.0, 1.571, 2.223]
INIT_ARM_R = [0.0, -0.166, 0.032, 0.0, -1.571, -2.223]


class DeployController(Node):
    def __init__(self, execute: bool, confirm: str, timeout: float) -> None:
        super().__init__("grasp_deploy_controller")
        self.execute_enabled = execute and confirm == "deploy"
        self.timeout = max(1.0, float(timeout))
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.rejected = execute and confirm != "deploy"

        self.base_xy: np.ndarray | None = None
        self.base_yaw: float | None = None
        self.joints: dict[str, float] = {}
        self.joint_state_seen = False
        self.target_meta: dict | None = None
        self.target_id = ""
        self.geometry: GraspGeometry = geometry_for_kind(None)
        self.kdl = MMK2Kdl()
        self.plan: dict | None = None
        self.plan_base_xy: np.ndarray | None = None
        self.plan_base_yaw: float | None = None
        self.last_fk_position_error: float | None = None
        self.last_fk_rotation_error: float | None = None
        self._reached_stable_cycles = 0
        self._deploy_ramp_started_at: float | None = None
        # top 模式 ramp 含悬停中间路点，时长翻倍（先上后下各 3s）
        self._deploy_ramp_seconds = 6.0 if _GRASP_ROT_MODE == "top" else 1.5   # 比赛提速
        self._deploy_ramp_start: dict | None = None

        self.spine_pub = self.create_publisher(Float64MultiArray, SPINE_TOPIC, 10)
        self.head_pub = self.create_publisher(Float64MultiArray, HEAD_TOPIC, 10)
        self.left_pub = self.create_publisher(Float64MultiArray, LEFT_ARM_TOPIC, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, RIGHT_ARM_TOPIC, 10)
        self.plan_pub = self.create_publisher(String, "/competition/deploy_plan", 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, qos_profile_sensor_data)
        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_subscription(PoseStamped, GRASP_GOAL_TOPIC, self._on_goal, 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().warning(
            f"部署控制器 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'}；"
            "只移动右臂到预抓取点，保持夹爪张开，不控制底盘"
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.base_xy = np.array([float(pose.position.x), float(pose.position.y)], dtype=float)
        q = pose.orientation
        self.base_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_joints(self, message: JointState) -> None:
        self.joint_state_seen = True
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning("部署控制器忽略非法 grasp_goal_meta")
            return
        target_id = str(payload.get("target_id", ""))
        world = payload.get("object_world")
        if not target_id or not isinstance(world, list) or len(world) != 3:
            self.get_logger().warning("部署控制器收到不完整抓取目标")
            return
        if target_id != self.target_id:
            self.target_id = target_id
            self.target_meta = payload
            self.geometry = geometry_for_kind(str(payload.get("kind", "")))
            self.plan = None
            self.last_fk_position_error = None
            self.last_fk_rotation_error = None
            self.get_logger().info(f"收到部署目标：target={target_id} kind={payload.get('kind')}")

    def _on_goal(self, message: PoseStamped) -> None:
        # 保留接口订阅，确保部署控制器只在导航目标消息存在时工作；
        # 实际部署坐标使用 grasp_goal_meta 中的商品世界坐标。
        pass

    def _world_to_footprint(self, point: np.ndarray) -> np.ndarray:
        assert self.base_xy is not None and self.base_yaw is not None
        d = point - np.array([self.base_xy[0], self.base_xy[1], 0.0])
        c, s = math.cos(-self.base_yaw), math.sin(-self.base_yaw)
        return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])

    def _current_left(self) -> list[float]:
        return [self.joints.get(f"left_arm_joint{i}", INIT_ARM_L[i - 1]) for i in range(1, 7)]

    def _current_right(self) -> list[float]:
        return [self.joints.get(f"right_arm_joint{i}", INIT_ARM_R[i - 1]) for i in range(1, 7)]

    def _build_plan(self) -> dict | None:
        if self.target_meta is None or self.base_xy is None or self.base_yaw is None or not self.joint_state_seen:
            return None
        raw = np.array([float(v) for v in self.target_meta["object_world"]], dtype=float)
        forward = np.array([math.cos(self.base_yaw), math.sin(self.base_yaw), 0.0])
        object_center = raw + self.geometry.surface_to_center_fwd * forward
        if _GRASP_ROT_MODE == "top":
            # 俯抓：正对商品中心上方，只有 z 偏置（咬棱高度）。
            # 侧抓的 y 向退避(-0.220)会让俯抓点落在商品后方 22cm 的空中
            # （实测 NO_CONTACT，指头悬空根本没下到商品上方）。
            top_dz = float(os.getenv("SUPERMARKET_TOP_GRASP_DZ", "0.05"))
            deploy_offset_arr = np.array([0.0, 0.0, top_dz], dtype=float)
        else:
            deploy_offset_arr = np.asarray(self.geometry.deploy_offset, dtype=float)
        deploy_world = object_center + deploy_offset_arr
        footprint = self._world_to_footprint(deploy_world)
        transform = np.eye(4)
        transform[:3, :3] = GRASP_ROT
        transform[:3, 3] = footprint
        # 锁定计划生成时的底盘位姿；执行过程中底盘可能发生微小回正，
        # 不能因此把同一个世界目标重新变换成漂移的 FK 目标。
        self.plan_base_xy = self.base_xy.copy()
        self.plan_base_yaw = float(self.base_yaw)
        # 不再把所有货架高度都强行压到固定 SLIDE_GRASP。
        # 随机商品可能位于 L1/L2/L3；扫描升降柱高度并选择第一个
        # 可达解，优先靠近当前高度，仍然只在 plan-only 中计算不发布命令。
        current_slide = float(self.joints.get("slide_joint", SLIDE_GRASP))
        # deploy 后还要执行 lift（下降 0.05m），因此禁止选择贴近
        # 下限的 slide；从官方中层 deploy=0.11 起搜索，保证动作
        # 骨架和随机多层目标都能安全进入 lift。
        slide_search_min = max(SLIDE_GRASP, SLIDE_MIN)
        slide_candidates = [
            round(slide_search_min + i * SLIDE_STEP, 3)
            for i in range(int(round((SLIDE_MAX - slide_search_min) / SLIDE_STEP)) + 1)
        ]
        slide_candidates.sort(key=lambda value: abs(value - current_slide))
        chosen_slide = None
        joints = None
        best_global_score = None
        last_exception = None
        for slide_candidate in slide_candidates:
            ref = np.array([slide_candidate] + self._current_right(), dtype=float)
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
            if solutions is not None and len(solutions) > 0:
                # IK 可能返回多个解；solutions[0] 不一定是当前姿态下
                # 末端误差最小的解。逐个做 FK 评分，优先选择真正
                # 对准 deploy 目标且关节跳变较小的方案。
                best_solution = None
                best_score = None
                for solution in solutions:
                    candidate = np.asarray(solution, dtype=float).reshape(-1)
                    if candidate.size < 7 or not np.all(np.isfinite(candidate[:7])):
                        continue
                    candidate = candidate[:7]
                    try:
                        _, fk_candidate = self.kdl.forward_kinematics(candidate, index="right")
                        position_error = float(np.linalg.norm(fk_candidate[:3, 3] - footprint))
                        rotation_error = self._rotation_error(
                            GRASP_ROT, np.asarray(fk_candidate[:3, :3], dtype=float)
                        )
                    except Exception:
                        continue
                    joint_delta = float(np.linalg.norm(candidate - ref))
                    score = position_error + 0.02 * rotation_error + 0.001 * joint_delta
                    if best_score is None or score < best_score:
                        best_score = score
                        best_solution = candidate
                if best_solution is not None and (
                    best_global_score is None or best_score < best_global_score
                ):
                    chosen_slide = slide_candidate
                    joints = best_solution
                    best_global_score = best_score
        if joints is None or chosen_slide is None:
            suffix = f" exception={last_exception}" if last_exception else ""
            self.get_logger().error(f"部署 IK 不可达：world={deploy_world.tolist()}{suffix}")
            return None
        slide = chosen_slide
        try:
            _, fk_right = self.kdl.forward_kinematics(joints, index="right")
            ik_position_error = float(np.linalg.norm(fk_right[:3, 3] - footprint))
        except Exception:
            ik_position_error = None

        # top 模式两段下探：先悬停到商品正上方，再竖直下压到咬棱高度。
        # 悬停段作为 ramp 插值的中间路点（见 _publish_commands），保证
        # 下压路径近似竖直，不会横扫到货架层板或相邻商品。
        hover_stage = None
        if _GRASP_ROT_MODE == "top":
            hover_dz = float(os.getenv("SUPERMARKET_TOP_HOVER_DZ", "0.16"))
            hover_world = object_center + np.array([0.0, 0.0, hover_dz], dtype=float)
            hover_fp = self._world_to_footprint(hover_world)
            hover_tf = np.eye(4)
            hover_tf[:3, :3] = GRASP_ROT
            hover_tf[:3, 3] = hover_fp
            hover_joints = None
            hover_slide = None
            for slide_candidate in slide_candidates:
                ref = np.array([slide_candidate] + self._current_right(), dtype=float)
                try:
                    solutions = self.kdl.inverse_kinematics(
                        T_left=None,
                        T_right=hover_tf,
                        ref_pos=ref,
                        target_height=float(slide_candidate),
                    )
                except Exception:
                    continue
                if solutions is None or len(solutions) == 0:
                    continue
                best_hover = None
                best_hover_score = None
                for solution in solutions:
                    candidate = np.asarray(solution, dtype=float).reshape(-1)
                    if candidate.size < 7 or not np.all(np.isfinite(candidate[:7])):
                        continue
                    candidate = candidate[:7]
                    try:
                        _, fk_c = self.kdl.forward_kinematics(candidate, index="right")
                        pe = float(np.linalg.norm(fk_c[:3, 3] - hover_fp))
                        re = self._rotation_error(GRASP_ROT, np.asarray(fk_c[:3, :3], dtype=float))
                    except Exception:
                        continue
                    jd = float(np.linalg.norm(candidate - ref))
                    score = pe + 0.02 * re + 0.001 * jd
                    if best_hover_score is None or score < best_hover_score:
                        best_hover_score = score
                        best_hover = candidate
                if best_hover is not None:
                    hover_joints = best_hover[1:7]
                    hover_slide = float(best_hover[0])
                    break
            if hover_joints is None:
                self.get_logger().error("top 模式悬停点 IK 不可达")
                return None
            hover_stage = {
                "slide": round(float(hover_slide), 5),
                "right_arm": [round(float(v), 5) for v in hover_joints],
            }
            self.get_logger().info(
                f"top 模式：悬停={hover_world.tolist()} grasp={deploy_world.tolist()}"
            )

        return {
            "schema_version": 1,
            "mode": "execute" if self.execute_enabled else "plan-only",
            "action": "deploy",
            "mechanical_commands_sent": False,
            "target_id": self.target_id,
            "kind": self.target_meta.get("kind"),
            "slot": self.target_meta.get("slot"),
            "grasp_geometry": self.geometry.to_dict(),
            "object_world": [round(float(v), 5) for v in raw],
            "object_center_world": [round(float(v), 5) for v in object_center],
            "deploy_world": [round(float(v), 5) for v in deploy_world],
            # 补全 lerp_plan 所需的全部键（head/left 在悬停段保持主段值：
            # head_yaw 与 left_arm 全程不动，head_pitch 有一次微小跳变，无碍）
            "hover_stage": (
                {
                    **hover_stage,
                    "head": [self.joints.get("head_yaw_joint", 0.0), HEAD_PITCH],
                    "left_arm": self._current_left(),
                    "left_gripper": self.joints.get("left_arm_eef_gripper_joint", GRIP_OPEN),
                    "right_gripper": GRIP_OPEN,
                }
                if hover_stage else None
            ),
            "slide": round(float(slide), 5),
            "ik_position_error_m": (
                round(float(ik_position_error), 6)
                if ik_position_error is not None else None
            ),
            "runtime_fk_required": True,
            "runtime_fk_position_tolerance_m": round(float(self.geometry.deploy_position_tolerance), 5),
            "runtime_fk_rotation_tolerance_rad": round(float(self.geometry.deploy_rotation_tolerance), 5),
            "head": [self.joints.get("head_yaw_joint", 0.0), HEAD_PITCH],
            "left_arm": self._current_left(),
            "right_arm": [round(float(v), 5) for v in joints[1:7]],
            "left_gripper": self.joints.get("left_arm_eef_gripper_joint", GRIP_OPEN),
            "right_gripper": GRIP_OPEN,
        }

    def _publish_plan(self) -> None:
        if self.plan is None:
            return
        message = String()
        message.data = json.dumps(self.plan, ensure_ascii=False, separators=(",", ":"))
        self.plan_pub.publish(message)

    def _publish_commands(self) -> None:
        assert self.plan is not None
        self.plan["mechanical_commands_sent"] = True
        command_plan = self.plan
        if self._deploy_ramp_start is not None and self._deploy_ramp_started_at is not None:
            alpha = min(1.0, max(0.0, (time.monotonic() - self._deploy_ramp_started_at) / self._deploy_ramp_seconds))
            start = self._deploy_ramp_start

            def lerp_plan(base: dict, target: dict, t: float) -> dict:
                def mix(a: float, b: float) -> float:
                    return float(a) + t * (float(b) - float(a))
                return {
                    "slide": mix(base["slide"], target["slide"]),
                    "head": [mix(base["head"][i], target["head"][i]) for i in range(2)],
                    "left_arm": [mix(base["left_arm"][i], target["left_arm"][i]) for i in range(6)],
                    "left_gripper": mix(base["left_gripper"], target["left_gripper"]),
                    "right_arm": [mix(base["right_arm"][i], target["right_arm"][i]) for i in range(6)],
                    "right_gripper": mix(base["right_gripper"], target["right_gripper"]),
                }

            hover_stage = self.plan.get("hover_stage")
            if hover_stage:
                # top 模式两段路点：alpha<0.5 从当前位姿到悬停点（商品正上方），
                # alpha>=0.5 从悬停点竖直下压到咬棱位。先上后下保证下压路径
                # 近似竖直，不横扫货架层板或相邻商品。
                if alpha < 0.5:
                    command_plan = lerp_plan(start, hover_stage, min(1.0, alpha / 0.5))
                else:
                    command_plan = lerp_plan(hover_stage, self.plan, min(1.0, (alpha - 0.5) / 0.5))
            else:
                command_plan = lerp_plan(start, self.plan, alpha)
        self.spine_pub.publish(Float64MultiArray(data=[float(command_plan["slide"])]))
        self.head_pub.publish(Float64MultiArray(data=[float(v) for v in command_plan["head"]]))
        self.left_pub.publish(Float64MultiArray(data=[float(v) for v in command_plan["left_arm"]] + [float(command_plan["left_gripper"])]))
        self.right_pub.publish(Float64MultiArray(data=[float(v) for v in command_plan["right_arm"]] + [float(command_plan["right_gripper"])]))

    @staticmethod
    def _rotation_error(target: np.ndarray, actual: np.ndarray) -> float:
        relative = target.T @ actual
        cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
        return float(math.acos(cosine))

    def _runtime_fk_errors(self) -> tuple[float, float] | None:
        """用当前反馈重新 FK，避免只凭关节角宣称 deploy 已对准。"""
        if self.plan_base_xy is None or self.plan_base_yaw is None or not self.joint_state_seen:
            return None
        names = [f"right_arm_joint{i}" for i in range(1, 7)]
        if "slide_joint" not in self.joints or any(name not in self.joints for name in names):
            return None
        q = np.array([self.joints["slide_joint"]] + [self.joints[name] for name in names], dtype=float)
        try:
            _, fk = self.kdl.forward_kinematics(q, index="right")
            target_world = np.asarray(self.plan["deploy_world"], dtype=float)
            d = target_world - np.array([self.plan_base_xy[0], self.plan_base_xy[1], 0.0])
            c, s = math.cos(-self.plan_base_yaw), math.sin(-self.plan_base_yaw)
            target_footprint = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
            position_error = float(np.linalg.norm(fk[:3, 3] - target_footprint))
            rotation_error = self._rotation_error(GRASP_ROT, np.asarray(fk[:3, :3], dtype=float))
            return position_error, rotation_error
        except Exception as exc:
            self.get_logger().warning(f"无法计算 deploy 运行时 FK 误差：{exc}")
            return None

    def _reached(self) -> bool:
        assert self.plan is not None
        if not self.joints:
            return False
        # deploy 的安全完成标准以末端 FK 为主；头部是感知关节，
        # 不应因其自身稳态误差阻塞右臂。升降柱和夹爪仍保留硬门槛。
        slide_ok = abs(self.joints.get("slide_joint", 999.0) - self.plan["slide"]) <= 0.06
        gripper_ok = abs(self.joints.get("right_arm_eef_gripper_joint", 999.0) - self.plan["right_gripper"]) <= 0.12
        fk_errors = self._runtime_fk_errors()
        if fk_errors is None:
            return False
        self.last_fk_position_error, self.last_fk_rotation_error = fk_errors
        self.plan["runtime_fk_position_error_m"] = round(float(self.last_fk_position_error), 6)
        self.plan["runtime_fk_rotation_error_rad"] = round(float(self.last_fk_rotation_error), 6)
        ready = (
            slide_ok
            and gripper_ok
            and self.last_fk_position_error <= self.geometry.deploy_position_tolerance
            and self.last_fk_rotation_error <= self.geometry.deploy_rotation_tolerance
        )
        self._reached_stable_cycles = self._reached_stable_cycles + 1 if ready else 0
        return self._reached_stable_cycles >= 3

    def _tick(self) -> None:
        if self.done:
            return
        if self.rejected:
            self.get_logger().error("拒绝部署：必须使用 --execute --confirm deploy")
            self.done = True
            return
        if time.monotonic() - self.started_at > self.timeout:
            self.get_logger().error("部署测试超时，停止发送控制")
            self.done = True
            return
        if self.plan is None:
            self.plan = self._build_plan()
            if self.plan is None:
                if time.monotonic() - self.last_log > 2.0:
                    self.get_logger().info("等待 grasp_goal_meta + odom + joint_states")
                    self.last_log = time.monotonic()
                return
            self._publish_plan()
            if self.execute_enabled:
                self._deploy_ramp_started_at = time.monotonic()
                self._deploy_ramp_start = {
                    "slide": float(self.joints.get("slide_joint", self.plan["slide"])),
                    "head": [
                        float(self.joints.get("head_yaw_joint", self.plan["head"][0])),
                        float(self.joints.get("head_pitch_joint", self.plan["head"][1])),
                    ],
                    "left_arm": [float(self.joints.get(f"left_arm_joint{i}", self.plan["left_arm"][i - 1])) for i in range(1, 7)],
                    "left_gripper": float(self.joints.get("left_arm_eef_gripper_joint", self.plan["left_gripper"])),
                    "right_arm": [float(self.joints.get(f"right_arm_joint{i}", self.plan["right_arm"][i - 1])) for i in range(1, 7)],
                    "right_gripper": float(self.joints.get("right_arm_eef_gripper_joint", self.plan["right_gripper"])),
                }
                self.get_logger().info("deploy 启用渐进关节插值：3.0s")
            self.get_logger().info(
                f"部署计划生成：target={self.target_id} deploy={self.plan['deploy_world']} "
                f"execute={self.execute_enabled}"
            )
            if not self.execute_enabled:
                self.success = True
                self.done = True
                return
        if not self.execute_enabled:
            return
        self._publish_commands()
        if self._reached():
            self.get_logger().info(
                f"部署动作完成：FK 末端误差 position={self.last_fk_position_error:.3f}m "
                f"rotation={self.last_fk_rotation_error:.3f}rad；夹爪保持张开"
            )
            self.success = True
            self.done = True
        elif time.monotonic() - self.last_log > 2.0:
            target_arm = [float(v) for v in self.plan["right_arm"]]
            arm_errors = [
                abs(self.joints.get(f"right_arm_joint{i}", 999.0) - target_arm[i - 1])
                for i in range(1, 7)
            ]
            slide_error = abs(self.joints.get("slide_joint", 999.0) - float(self.plan["slide"]))
            head_error = abs(self.joints.get("head_pitch_joint", 999.0) - float(self.plan["head"][1]))
            fk_errors = self._runtime_fk_errors()
            fk_text = (
                f"fk_pos={fk_errors[0]:.3f} fk_rot={fk_errors[1]:.3f}"
                if fk_errors is not None else "fk=unavailable"
            )
            self.get_logger().info(
                "部署执行中："
                f"slide={self.joints.get('slide_joint')} target={self.plan['slide']} err={slide_error:.3f}; "
                f"head_pitch={self.joints.get('head_pitch_joint')} target={self.plan['head'][1]} err={head_error:.3f}; "
                f"max_arm_err={max(arm_errors):.3f}; {fk_text}"
            )
            self.last_log = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 机械臂部署测试")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if args.execute and args.confirm != "deploy":
        print("拒绝执行：必须使用 --confirm deploy")
        return 2
    rclpy.init()
    node = DeployController(args.execute, args.confirm, args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
