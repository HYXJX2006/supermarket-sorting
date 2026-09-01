#!/usr/bin/env python3
"""DG-202606 安全 dry-run 任务执行器。

当前版本只做任务编排和感知联调：
- 订阅 /supermarket_sorting/task；
- 订阅商品 Detection3DArray 和 ArUco 货位 ID；
- 对目标商品做多帧、置信度和唯一货位标记确认；
- 输出下一步动作建议；
- 持续发布零速度，绝不控制机械臂、夹爪或升降柱。

该文件不读取 Server 内部布局真值，也不使用固定商品位置。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from vision_msgs.msg import Detection3DArray

from shelf_map import ShelfMap


TASK_TOPIC = "/supermarket_sorting/task"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
DETECTION_TOPIC = "/multiclass/detections"
ARUCO_TOPIC = "/aruco/head/ids"
CMD_VEL_TOPIC = "/cmd_vel"
NAV_GOAL_TOPIC = "/competition/navigation_goal"
NAV_META_TOPIC = "/competition/navigation_goal_meta"
NAV_STATUS_TOPIC = "/competition/navigation_status"
TARGET_STATUS_TOPIC = "/competition/target_status"
GRASP_GOAL_TOPIC = "/competition/grasp_goal"
GRASP_META_TOPIC = "/competition/grasp_goal_meta"

# 官方 retail_competition 的固定货架槽位几何。随机场景只会交换商品
# 与槽位，不会改变这些槽位；该表只用于把已观测 world 坐标映射到
# 最近货位，不能从任务 ID 推断商品位置。ArUco 仍是首选来源。
SHELF_SLOT_X = {
    "A": (-1.955, -1.735, -1.515),
    "B": (-1.070, -0.850, -0.630),
    "C": (-0.185, 0.035, 0.255),
    "D": (0.700, 0.920, 1.140),
    "E": (1.585, 1.805, 2.025),
}
SHELF_SLOT_Y = 3.243
SHELF_SURFACE_Z = {1: 0.499, 2: 0.851, 3: 1.189}
SHELF_OBJECT_HALF_HEIGHT_GUESS = 0.06
# 已验证/可复用的货架外侧观察位。D 是官方固定 Baseline 的右臂
# 特殊姿态；E 使用货架扫描线中心位，适合 E 组商品的 plan-only IK。
GRASP_APPROACH_YAW = math.pi / 2.0 - math.radians(11.0)
SHELF_APPROACH_POSES = {
    "D": (0.852, 2.475, GRASP_APPROACH_YAW),
    "E": (1.805, 2.475, GRASP_APPROACH_YAW),
}

VALID_STATUSES = {"pending", "searching", "grasping", "delivering", "done", "failed"}


@dataclass
class TargetState:
    target_id: str
    kind: str
    status: str = "pending"
    attempts: int = 0
    error: str | None = None
    candidate: dict[str, Any] | None = None
    last_log_key: str = ""
    pending_lock_key: str = ""
    pending_lock_streak: int = 0
    pending_lock_last_seen: float = 0.0
    candidate_locked: bool = False


@dataclass
class StableCandidate:
    track_id: str
    kind: str
    confidence: float
    world: tuple[float, float, float]
    aruco_ids: tuple[int, ...]
    slot: str | None
    samples: int
    first_seen: float
    last_seen: float


@dataclass
class InstanceTrack:
    """同一商品实例的世界坐标轨迹。"""

    track_id: str
    kind: str
    samples: deque = field(default_factory=lambda: deque(maxlen=30))
    last_seen: float = 0.0


@dataclass
class SensorState:
    x: float | None = None
    y: float | None = None
    yaw: float | None = None
    scan_stamp: float = 0.0
    scan_min_range: float | None = None
    aruco_ids: tuple[int, ...] = ()
    detection_stamp: float = 0.0


class DryRunExecutor(Node):
    def __init__(
        self,
        *,
        duration: float,
        min_samples: int,
        min_confidence: float,
        camera: str,
        detection_topic: str,
        shelf_normal_x: float,
        shelf_normal_y: float,
        approach_distance: float,
        publish_zero: bool,
        slot_lock_frames: int,
        aruco_hold_seconds: float,
        tracker_window_frames: int = 7,
    ) -> None:
        super().__init__("competition_executor_dry_run")
        self.duration = max(0.0, float(duration))
        self.min_samples = max(1, int(min_samples))
        self.min_confidence = float(min_confidence)
        self.camera = camera
        self.detection_topic = detection_topic
        normal_norm = math.hypot(float(shelf_normal_x), float(shelf_normal_y))
        if normal_norm < 1e-6:
            raise ValueError("货架法向量不能为零")
        self.shelf_normal_x = float(shelf_normal_x) / normal_norm
        self.shelf_normal_y = float(shelf_normal_y) / normal_norm
        self.approach_distance = max(0.1, float(approach_distance))
        self.publish_zero_enabled = bool(publish_zero)
        self.slot_lock_frames = max(1, int(slot_lock_frames))
        self.aruco_hold_seconds = max(0.0, float(aruco_hold_seconds))
        # 行驶中允许先跟踪候选，但只在进入取货区后收集，避免把远处
        # 背景误检当作目标；锁定采用最近 N 帧中的 M 帧稳定策略。
        self.tracker_window_frames = max(3, int(tracker_window_frames))
        self.pick_zone_y_min = float(os.getenv("SUPERMARKET_PICK_ZONE_Y_MIN", "1.70"))
        self.pick_zone_y_max = float(os.getenv("SUPERMARKET_PICK_ZONE_Y_MAX", "3.40"))
        self.track_x_min = float(os.getenv("SUPERMARKET_TRACK_WORLD_X_MIN", "-2.40"))
        self.track_x_max = float(os.getenv("SUPERMARKET_TRACK_WORLD_X_MAX", "2.40"))
        self.track_y_min = float(os.getenv("SUPERMARKET_TRACK_WORLD_Y_MIN", "3.00"))
        self.track_y_max = float(os.getenv("SUPERMARKET_TRACK_WORLD_Y_MAX", "3.50"))
        self.track_z_min = float(os.getenv("SUPERMARKET_TRACK_WORLD_Z_MIN", "0.35"))
        self.track_z_max = float(os.getenv("SUPERMARKET_TRACK_WORLD_Z_MAX", "1.45"))
        self.track_max_jitter = max(
            0.05, float(os.getenv("SUPERMARKET_TRACK_MAX_JITTER", "0.15"))
        )
        target_level_raw = os.getenv("SUPERMARKET_TARGET_LEVEL", "0").strip()
        try:
            self.target_level = int(target_level_raw or "0")
        except ValueError as exc:
            raise ValueError("SUPERMARKET_TARGET_LEVEL must be 0, 1, 2, or 3") from exc
        if self.target_level not in {0, 1, 2, 3}:
            raise ValueError("SUPERMARKET_TARGET_LEVEL must be 0, 1, 2, or 3")
        # 同类别检测按世界坐标聚类为独立实例；同一帧内的重复框只保留
        # 置信度最高的一次，跨帧则要求在匹配半径内继续跟踪。
        self.instance_match_radius = max(
            0.05, float(os.getenv("SUPERMARKET_INSTANCE_MATCH_RADIUS", "0.22"))
        )
        self.instance_stale_seconds = max(
            1.0, float(os.getenv("SUPERMARKET_INSTANCE_STALE_SECONDS", "3.0"))
        )
        self.last_tracking_log = 0.0
        expected_raw = os.getenv("SUPERMARKET_EXPECTED_TARGET_COUNT", "").strip()
        self.expected_target_count = 0
        if expected_raw:
            try:
                self.expected_target_count = max(0, int(expected_raw))
            except ValueError as exc:
                raise ValueError("SUPERMARKET_EXPECTED_TARGET_COUNT must be an integer") from exc
        # 仅用于固定 Baseline 开发验证；随机赛不设置此变量。
        self.fixed_target_slot = os.getenv("SUPERMARKET_FIXED_TARGET_SLOT", "").strip() or None
        self.fixed_target_world = None
        fixed_world_raw = os.getenv("SUPERMARKET_FIXED_TARGET_WORLD", "").strip()
        if fixed_world_raw:
            try:
                values = [float(item.strip()) for item in fixed_world_raw.split(",")]
                if len(values) != 3:
                    raise ValueError
                self.fixed_target_world = tuple(values)
            except ValueError as exc:
                raise ValueError("SUPERMARKET_FIXED_TARGET_WORLD must be x,y,z") from exc
        self.fixed_target_world_tolerance = max(
            0.01, float(os.getenv("SUPERMARKET_FIXED_TARGET_WORLD_TOLERANCE", "0.12"))
        )
        # 视觉深度异常会把 uint16 饱和值反投影到几十米外；在协调器入口
        # 再做一次物理距离和有限值保护，避免污染稳定候选历史。
        self.min_detection_range = max(
            0.05, float(os.getenv("SUPERMARKET_MIN_DETECTION_RANGE", "0.15"))
        )
        self.max_detection_range = max(
            self.min_detection_range + 0.1,
            float(os.getenv("SUPERMARKET_MAX_DETECTION_RANGE", "8.0")),
        )
        self.max_world_abs = max(
            5.0, float(os.getenv("SUPERMARKET_MAX_WORLD_ABS", "20.0"))
        )
        self.max_candidate_jitter = max(
            0.05, float(os.getenv("SUPERMARKET_MAX_CANDIDATE_JITTER", "0.35"))
        )
        # 与官方动作骨架一致的当前观察位可达包络；只过滤当前视野中
        # 无法由右臂部署的误检，不猜测货架或目标真值。
        self.reach_forward_min = max(
            0.05, float(os.getenv("SUPERMARKET_REACH_FORWARD_MIN", "0.30"))
        )
        self.reach_forward_max = max(
            self.reach_forward_min + 0.1,
            float(os.getenv("SUPERMARKET_REACH_FORWARD_MAX", "1.50")),
        )
        self.reach_lateral_max = max(
            0.05, float(os.getenv("SUPERMARKET_REACH_LATERAL_MAX", "0.25"))
        )
        self.reach_z_min = float(os.getenv("SUPERMARKET_REACH_Z_MIN", "0.35"))
        self.reach_z_max = float(os.getenv("SUPERMARKET_REACH_Z_MAX", "1.80"))
        self.invalid_detection_count = 0
        self.last_invalid_detection_log = 0.0
        self.started_at = time.monotonic()
        self.last_report_at = 0.0
        self.last_zero_at = 0.0
        self.last_task_message = ""
        self.received_task = False
        self.finished_reason: str | None = None
        self.finished = False

        self.targets: list[TargetState] = []
        self.target_index = 0
        self.sensor = SensorState()
        self.shelf_map = ShelfMap()
        self.instance_tracks: dict[str, list[InstanceTrack]] = {}
        self.next_instance_id = 0

        task_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, TASK_TOPIC, self._on_task, task_qos)
        self.create_subscription(
            Detection3DArray,
            self.detection_topic,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int32MultiArray,
            f"/aruco/{camera}/ids",
            self._on_aruco_ids,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, NAV_STATUS_TOPIC, self._on_navigation_status, 10)
        self.zero_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, NAV_GOAL_TOPIC, 10)
        self.navigation_meta_pub = self.create_publisher(String, NAV_META_TOPIC, 10)
        self.target_status_pub = self.create_publisher(String, TARGET_STATUS_TOPIC, 10)
        self.last_navigation_goal_key = ""
        self.navigation_state = "unknown"
        self.grasp_goal_pub = self.create_publisher(PoseStamped, GRASP_GOAL_TOPIC, 10)
        self.grasp_meta_pub = self.create_publisher(String, GRASP_META_TOPIC, 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().warning(
            "DRY-RUN 模式：只编排任务和读取感知，不控制机械臂、夹爪、升降柱；"
            f"底盘零速度发布={self.publish_zero_enabled}"
        )
        self.get_logger().info(
            f"task={TASK_TOPIC} detections={detection_topic} aruco=/aruco/{camera}/ids "
            f"duration={self.duration:.1f}s min_samples={self.min_samples} "
            f"min_confidence={self.min_confidence:.2f}"
        )

    @staticmethod
    def _yaw_from_quaternion(q: Any) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _on_task(self, message: String) -> None:
        if message.data == self.last_task_message:
            return
        try:
            task = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"任务 JSON 无法解析: {exc}")
            return
        if not isinstance(task, dict) or not isinstance(task.get("targets"), list):
            self.get_logger().error("任务必须是包含 targets 数组的 JSON 对象")
            return

        parsed: list[TargetState] = []
        seen_ids: set[str] = set()
        for index, raw in enumerate(task["targets"]):
            if not isinstance(raw, dict):
                self.get_logger().error(f"任务目标 {index} 不是对象")
                return
            target_id = raw.get("id")
            kind = raw.get("kind")
            if not isinstance(target_id, str) or not target_id:
                self.get_logger().error(f"任务目标 {index} 缺少合法 id")
                return
            if not isinstance(kind, str) or not kind:
                self.get_logger().error(f"任务目标 {target_id} 缺少合法 kind")
                return
            if target_id in seen_ids:
                self.get_logger().error(f"任务目标 id 重复: {target_id}")
                return
            seen_ids.add(target_id)
            parsed.append(TargetState(target_id=target_id, kind=kind))

        if not parsed:
            self.get_logger().error("任务 targets 为空，拒绝执行")
            return
        if self.expected_target_count and len(parsed) != self.expected_target_count:
            self.get_logger().error(
                f"任务目标数量不符合当前联调约束：收到={len(parsed)}，"
                f"期望={self.expected_target_count}；拒绝进入 plan-only 队列"
            )
            return

        self.last_task_message = message.data
        self.targets = parsed
        self.target_index = 0
        self.received_task = True
        # 新任务必须先经过 approach_search 到达货架观察位，再允许锁定视觉候选。
        # 防止机器人行驶途中把远处/错误视角的检测结果提前绑定。
        self.navigation_state = "approach_pending"
        self.instance_tracks.clear()
        self.next_instance_id = 0
        self.get_logger().info(
            f"收到任务 run_prefix={task.get('run_prefix')} count={len(parsed)}；"
            f"expected={self.expected_target_count or 'any'} target_level={self.target_level or 'any'}；"
            "dry-run 不会执行运动"
        )
        self._log_snapshot("task_received")
        self._publish_target_status("task_received")

    def _on_aruco_ids(self, message: Int32MultiArray) -> None:
        ids = sorted({int(value) for value in message.data if 0 <= int(value) < 45})
        self.sensor.aruco_ids = tuple(ids)

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.sensor.x = float(pose.position.x)
        self.sensor.y = float(pose.position.y)
        self.sensor.yaw = self._yaw_from_quaternion(pose.orientation)

    @staticmethod
    def _navigation_status_matches_target(payload: dict[str, Any], target: TargetState) -> bool:
        """区分取货前观察路线到达与目标专属导航到达。"""
        if not target.candidate or not isinstance(target.candidate.get("navigation_goal"), list):
            return False
        reported = payload.get("goal")
        expected = target.candidate["navigation_goal"]
        if not isinstance(reported, list) or len(reported) < 3 or len(expected) < 3:
            return False
        try:
            xy_error = math.hypot(float(reported[0]) - float(expected[0]), float(reported[1]) - float(expected[1]))
            yaw_error = abs((float(reported[2]) - float(expected[2]) + math.pi) % (2.0 * math.pi) - math.pi)
        except (TypeError, ValueError):
            return False
        return xy_error <= 0.15 and yaw_error <= 0.20

    def _on_navigation_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            state = str(payload.get("state", "unknown"))
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(f"忽略非法导航状态: {message.data[:120]}")
            return
        target = self._current_target()
        if target is None:
            self.navigation_state = state
            return
        # 首段“到观察线”与候选锁定后的“目标专属导航”都发布同一个
        # navigation_status 话题；只有 goal 与候选中记录的 navigation_goal
        # 匹配时，才允许进入 grasping/生成抓取元信息。
        target_goal_match = self._navigation_status_matches_target(payload, target)
        if state == "reached" and target.status == "searching" and target.candidate:
            if target_goal_match:
                self.navigation_state = "reached"
                self._set_status(target, "grasping")
                self._publish_grasp_plan(target)
            else:
                self.navigation_state = "observation_ready"
                self.get_logger().info(
                    f"忽略取货前观察路线 reached：reported_goal={payload.get('goal')} "
                    f"target_goal={target.candidate.get('navigation_goal')}；等待目标专属导航到位"
                )
        elif state == "failed" and target.status == "searching":
            if target_goal_match:
                self.navigation_state = "failed"
                self._set_status(target, "failed", "目标专属导航器报告失败")
            else:
                self.navigation_state = "observation_ready"
                self.get_logger().warning(
                    f"忽略取货前观察路线 failed：reported_goal={payload.get('goal')}；"
                    "不把观察路线失败算作目标导航失败"
                )
        else:
            self.navigation_state = state

    def _on_scan(self, message: LaserScan) -> None:
        valid = [float(r) for r in message.ranges if math.isfinite(r) and r > 0.0]
        self.sensor.scan_min_range = min(valid) if valid else None
        self.sensor.scan_stamp = time.monotonic()

    @staticmethod
    def _world_slot_geometry(world: tuple[float, float, float]) -> tuple[int, str] | None:
        """将视觉 world 点吸附到最近固定货架槽位。

        这是 ArUco 不可用时的保守 fallback：只接受与槽位几何足够
        接近的观测，不把商品类别或任务匿名 ID 当作货位信息。
        """
        wx, wy, wz = (float(value) for value in world)
        best = None
        for shelf_index, (shelf, xs) in enumerate(SHELF_SLOT_X.items()):
            for column, x in enumerate(xs, start=1):
                xy_error = math.hypot(wx - x, wy - SHELF_SLOT_Y)
                if xy_error > 0.12:
                    continue
                for level in (1, 2, 3):
                    expected_z = SHELF_SURFACE_Z[level] + SHELF_OBJECT_HALF_HEIGHT_GUESS
                    z_error = abs(wz - expected_z)
                    if z_error > 0.16:
                        continue
                    score = xy_error + 0.35 * z_error
                    if best is None or score < best[0]:
                        aruco_id = shelf_index * 9 + (level - 1) * 3 + (column - 1)
                        best = (score, aruco_id, f"{shelf}/L{level}/C{column}")
        return None if best is None else (best[1], best[2])

    @staticmethod
    def _world_level(world: tuple[float, float, float]) -> int | None:
        geometry = DryRunExecutor._world_slot_geometry(world)
        if geometry is None:
            return None
        label = geometry[1]
        try:
            return int(label.split("/L", 1)[1].split("/", 1)[0])
        except (IndexError, ValueError):
            return None

    def _world_point_is_valid(self, world: tuple[float, float, float]) -> bool:
        if not all(math.isfinite(float(value)) for value in world):
            return False
        if any(abs(float(value)) > self.max_world_abs for value in world):
            return False
        if not (self.reach_z_min <= float(world[2]) <= self.reach_z_max):
            return False
        if self.sensor.x is None or self.sensor.y is None or self.sensor.yaw is None:
            return True
        dx = float(world[0]) - self.sensor.x
        dy = float(world[1]) - self.sensor.y
        distance = math.hypot(dx, dy)
        if not (self.min_detection_range <= distance <= self.max_detection_range):
            return False
        # Footprint x is forward and y is lateral for the current base yaw.
        forward = math.cos(self.sensor.yaw) * dx + math.sin(self.sensor.yaw) * dy
        lateral = -math.sin(self.sensor.yaw) * dx + math.cos(self.sensor.yaw) * dy
        return (
            self.reach_forward_min <= forward <= self.reach_forward_max
            and abs(lateral) <= self.reach_lateral_max
        )

    def _in_pick_zone(self) -> bool:
        return (
            self.sensor.y is not None
            and self.pick_zone_y_min <= float(self.sensor.y) <= self.pick_zone_y_max
        )

    def _world_point_is_trackable(self, world: tuple[float, float, float]) -> bool:
        """行驶中候选的宽松场景过滤，不使用当前机械臂可达包络。"""
        if not all(math.isfinite(float(value)) for value in world):
            return False
        wx, wy, wz = (float(value) for value in world)
        return (
            self.track_x_min <= wx <= self.track_x_max
            and self.track_y_min <= wy <= self.track_y_max
            and self.track_z_min <= wz <= self.track_z_max
        )

    def _current_target(self) -> TargetState | None:
        if not self.targets or self.target_index >= len(self.targets):
            return None
        return self.targets[self.target_index]

    def _upsert_instance_track(
        self,
        kind: str,
        confidence: float,
        world: tuple[float, float, float],
        now: float,
    ) -> None:
        tracks = self.instance_tracks.setdefault(kind, [])
        tracks[:] = [
            track
            for track in tracks
            if now - track.last_seen <= self.instance_stale_seconds
        ]
        matched = None
        best_distance = self.instance_match_radius
        for track in tracks:
            if not track.samples:
                continue
            previous = track.samples[-1][2]
            distance = math.sqrt(sum((world[i] - previous[i]) ** 2 for i in range(3)))
            if distance <= best_distance:
                matched = track
                best_distance = distance
        if matched is None:
            self.next_instance_id += 1
            matched = InstanceTrack(
                track_id=f"{kind}-instance-{self.next_instance_id:03d}",
                kind=kind,
            )
            tracks.append(matched)
        # 同一 ROS 检测消息里的重复框可能落到同一实例；只保留置信度最高框，
        # 避免一帧被错误计数多次。
        if matched.samples and abs(matched.samples[-1][0] - now) < 1e-6:
            if confidence > matched.samples[-1][1]:
                matched.samples[-1] = (now, confidence, world)
        else:
            matched.samples.append((now, confidence, world))
        matched.last_seen = now

    def _on_detections(self, message: Detection3DArray) -> None:
        now = time.monotonic()
        self.sensor.detection_stamp = now
        moving_track = self._in_pick_zone() and self.navigation_state not in {
            "reached", "observation_ready"
        }
        for detection in message.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            kind = str(result.hypothesis.class_id or "").strip()
            confidence = float(result.hypothesis.score)
            if not kind or confidence < self.min_confidence:
                continue
            p = result.pose.pose.position
            world = (float(p.x), float(p.y), float(p.z))
            normal_valid = self._world_point_is_valid(world)
            if not normal_valid and not (moving_track and self._world_point_is_trackable(world)):
                self.invalid_detection_count += 1
                if now - self.last_invalid_detection_log >= 3.0:
                    self.get_logger().warning(
                        f"丢弃异常视觉坐标：kind={kind} world={world} "
                        f"range_limit=[{self.min_detection_range:.2f},{self.max_detection_range:.2f}] "
                        f"reach_lateral={self.reach_lateral_max:.2f} "
                        f"invalid_total={self.invalid_detection_count}"
                    )
                    self.last_invalid_detection_log = now
                continue
            if moving_track and not normal_valid and now - self.last_tracking_log >= 2.0:
                self.get_logger().info(
                    f"取货区移动中跟踪候选：kind={kind} world={world}；"
                    f"暂不要求机械臂可达，等待 {self.min_samples}/{self.tracker_window_frames} 帧稳定"
                )
                self.last_tracking_log = now
            self._upsert_instance_track(kind, confidence, world, now)

    def _stable_candidates(self, kind: str) -> list[StableCandidate]:
        tracks = self.instance_tracks.get(kind, [])
        if not tracks:
            return []
        now = time.monotonic()
        moving_track = self._in_pick_zone() and self.navigation_state not in {
            "reached", "observation_ready"
        }
        candidates: list[StableCandidate] = []
        for track in tracks:
            recent = [
                item for item in track.samples
                if now - item[0] <= self.instance_stale_seconds
            ][-self.tracker_window_frames:]
            if len(recent) < self.min_samples:
                continue
            xs = sorted(item[2][0] for item in recent)
            ys = sorted(item[2][1] for item in recent)
            zs = sorted(item[2][2] for item in recent)
            mid = len(recent) // 2
            center = (xs[mid], ys[mid], zs[mid])
            jitter_limit = self.track_max_jitter if moving_track else self.max_candidate_jitter
            inliers = [
                item for item in recent
                if math.sqrt(sum((item[2][axis] - center[axis]) ** 2 for axis in range(3)))
                <= jitter_limit
            ]
            if len(inliers) < self.min_samples:
                continue
            xs = sorted(item[2][0] for item in inliers)
            ys = sorted(item[2][1] for item in inliers)
            zs = sorted(item[2][2] for item in inliers)
            mid = len(inliers) // 2
            world = (xs[mid], ys[mid], zs[mid])
            if not self._world_point_is_valid(world):
                if not (moving_track and self._world_point_is_trackable(world)):
                    continue
            inferred_level = self._world_level(world)
            if self.target_level and inferred_level != self.target_level:
                continue
            confidence = sum(item[1] for item in inliers) / len(inliers)
            aruco_ids = self.sensor.aruco_ids
            slot = self.shelf_map.slots[aruco_ids[0]].label if len(aruco_ids) == 1 else None
            candidates.append(
                StableCandidate(
                    track_id=track.track_id,
                    kind=kind,
                    confidence=confidence,
                    world=world,
                    aruco_ids=aruco_ids,
                    slot=slot,
                    samples=len(inliers),
                    first_seen=inliers[0][0],
                    last_seen=inliers[-1][0],
                )
            )
        return candidates

    def _stable_candidate(self, kind: str) -> StableCandidate | None:
        """兼容旧调用方：返回当前类别中最近的稳定可达实例。"""
        return self._select_stable_candidate(kind)

    def _select_stable_candidate(self, kind: str) -> StableCandidate | None:
        candidates = self._stable_candidates(kind)
        if not candidates:
            return None
        if self.sensor.x is None or self.sensor.y is None:
            candidates.sort(key=lambda item: (-item.confidence, -item.samples, item.track_id))
        else:
            candidates.sort(
                key=lambda item: (
                    math.hypot(item.world[0] - self.sensor.x, item.world[1] - self.sensor.y),
                    -item.confidence,
                    -item.samples,
                    item.track_id,
                )
            )
        selected = candidates[0]
        if len(candidates) > 1 and time.monotonic() - self.last_report_at >= 2.0:
            distances = [
                (item.track_id, round(math.hypot(item.world[0] - self.sensor.x, item.world[1] - self.sensor.y), 3))
                for item in candidates
            ] if self.sensor.x is not None and self.sensor.y is not None else []
            self.get_logger().info(
                f"多个 {kind} 实例通过稳定性/可达性过滤，选择最近候选 "
                f"track={selected.track_id} candidates={distances}"
            )
            self.last_report_at = time.monotonic()
        return selected

    def _set_status(self, target: TargetState, status: str, error: str | None = None) -> None:
        if status not in VALID_STATUSES:
            raise ValueError(f"未知状态: {status}")
        if target.status == status and target.error == error:
            return
        target.status = status
        target.error = error
        if status == "failed":
            target.attempts += 1
        self._log_snapshot(f"status_{status}")
        self._publish_target_status(f"status_{status}")

    def _publish_target_status(self, reason: str) -> None:
        current = self._current_target()
        payload = {
            "schema_version": 1,
            "mode": "dry-run",
            "reason": reason,
            "received_task": self.received_task,
            "current_index": self.target_index,
            "count": len(self.targets),
            "current_target": {
                "id": current.target_id,
                "kind": current.kind,
                "status": current.status,
                "attempts": current.attempts,
                "error": current.error,
                "candidate": current.candidate,
                "candidate_locked": current.candidate_locked,
            } if current else None,
            "targets": [
                {
                    "id": target.target_id,
                    "kind": target.kind,
                    "status": target.status,
                    "attempts": target.attempts,
                    "error": target.error,
                    "candidate": target.candidate,
                    "candidate_locked": target.candidate_locked,
                }
                for target in self.targets
            ],
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.target_status_pub.publish(message)

    def _log_snapshot(self, reason: str) -> None:
        current = self._current_target()
        payload = {
            "reason": reason,
            "mode": "dry-run",
            "received_task": self.received_task,
            "current_index": self.target_index,
            "count": len(self.targets),
            "current_target": {
                "id": current.target_id,
                "kind": current.kind,
                "status": current.status,
                "attempts": current.attempts,
                "candidate": current.candidate,
            }
            if current
            else None,
            "targets": [
                {
                    "id": target.target_id,
                    "kind": target.kind,
                    "status": target.status,
                    "attempts": target.attempts,
                    "error": target.error,
                }
                for target in self.targets
            ],
            "sensor": {
                "odom": [self.sensor.x, self.sensor.y, self.sensor.yaw],
                "aruco_ids": list(self.sensor.aruco_ids),
                "scan_min_range": self.sensor.scan_min_range,
            },
        }
        self.get_logger().info(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))

    def _publish_navigation_goal(self, target: TargetState, candidate: StableCandidate) -> None:
        """把锁定商品转换成底盘观察点；只发布目标，不发布速度。"""
        wx, wy, _ = candidate.world
        shelf_group = candidate.slot.split("/", 1)[0] if candidate.slot and "/" in candidate.slot else None
        mapped_pose = SHELF_APPROACH_POSES.get(shelf_group or "")
        if mapped_pose is not None and os.getenv("SUPERMARKET_USE_SHELF_APPROACH_MAP", "1") == "1":
            gx, gy, goal_yaw = mapped_pose
        elif os.getenv("SUPERMARKET_FIXED_APPROACH", "0") == "1":
            # 固定 Baseline 的货架观察位已由 3DGS/IK 联调验证；不要把底盘
            # 直接推到商品 x 坐标，否则右臂部署姿态可能不可达。
            gx, gy, goal_yaw = 0.852, 2.475, GRASP_APPROACH_YAW
        else:
            gx = wx - self.shelf_normal_x * self.approach_distance
            gy = wy - self.shelf_normal_y * self.approach_distance
            # 目标货架未进入已验证映射时使用可覆盖的默认朝向。
            goal_yaw = float(os.getenv("SUPERMARKET_APPROACH_YAW", str(GRASP_APPROACH_YAW)))
        key = json.dumps(
            {
                "target_id": target.target_id,
                "kind": target.kind,
                "world": [round(wx, 4), round(wy, 4)],
                "goal": [round(gx, 4), round(gy, 4), round(goal_yaw, 4)],
            },
            sort_keys=True,
        )
        if key == self.last_navigation_goal_key:
            return

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        self.navigation_goal_pub.publish(pose)

        meta = String()
        meta.data = json.dumps(
            {
                "schema_version": 1,
                "mode": "coordinator",
                "target_id": target.target_id,
                "kind": target.kind,
                "slot": candidate.slot,
                "object_world": [round(v, 4) for v in candidate.world],
                "navigation_goal": [round(gx, 4), round(gy, 4), round(goal_yaw, 4)],
                "approach_distance": self.approach_distance,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.navigation_meta_pub.publish(meta)
        # target_status 也携带导航目标，避免编排器在一次性 nav_meta 消息发布前后启动
        # approach_target worker 时丢失目标；这仍然只是 PoseStamped 计划，不是速度命令。
        target.candidate["navigation_goal"] = [round(gx, 4), round(gy, 4), round(goal_yaw, 4)]
        self._publish_target_status("navigation_goal_ready")
        self.last_navigation_goal_key = key
        self.get_logger().info(
            f"已发布导航目标（不移动底盘）：target={target.target_id} "
            f"goal=({gx:.3f},{gy:.3f},yaw={goal_yaw:.3f})"
        )

    def _publish_grasp_plan(self, target: TargetState) -> None:
        """发布抓取计划，不发布任何机械臂关节或夹爪命令。"""
        if not target.candidate:
            return
        world = target.candidate["world"]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(world[0])
        pose.pose.position.y = float(world[1])
        pose.pose.position.z = float(world[2])
        pose.pose.orientation.w = 1.0
        self.grasp_goal_pub.publish(pose)

        meta = String()
        meta.data = json.dumps(
            {
                "schema_version": 1,
                "mode": "coordinator",
                "target_id": target.target_id,
                "kind": target.kind,
                "slot": target.candidate.get("slot"),
                "object_world": target.candidate["world"],
                "confidence": target.candidate.get("confidence"),
                "orientation_strategy": "pending_calibration",
                "mechanical_commands_sent": False,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.grasp_meta_pub.publish(meta)
        self.get_logger().info(
            f"已发布抓取计划（不控制机械臂）：target={target.target_id} "
            f"kind={target.kind} world={target.candidate['world']}"
        )

    def _publish_zero(self) -> None:
        if not self.publish_zero_enabled:
            return
        self.zero_pub.publish(Twist())
        self.last_zero_at = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        self._publish_zero()

        if self.duration and now - self.started_at >= self.duration:
            self.finished_reason = "duration_elapsed"
            self.finished = True
            self.get_logger().info("dry-run 时间到，已持续发布零速度")
            self._log_snapshot("duration_elapsed")
            return

        if not self.received_task:
            if now - self.last_report_at >= 2.0:
                self.get_logger().info("等待 /supermarket_sorting/task；当前保持停车")
                self.last_report_at = now
            return

        target = self._current_target()
        if target is None:
            if now - self.last_report_at >= 2.0:
                self.get_logger().info("所有任务目标已完成编排；当前保持停车")
                self.last_report_at = now
            return

        if target.status == "grasping":
            if now - self.last_report_at >= 2.0:
                self.get_logger().info(
                    f"目标 {target.target_id} 已进入 grasping；等待机械臂控制器（当前不发机械臂命令）"
                )
                self.last_report_at = now
            return
        if target.status in {"done", "failed"}:
            return

        if target.status == "pending":
            self._set_status(target, "searching")
            self.get_logger().info(
                f"dry-run 当前目标 {target.target_id} kind={target.kind}：开始等待视觉候选"
            )

        at_observation = self.navigation_state in {"reached", "observation_ready"}
        in_pick_zone = self._in_pick_zone()
        candidate = self._select_stable_candidate(target.kind) if (at_observation or in_pick_zone) else None
        if candidate is None:
            if now - self.last_report_at >= 2.0:
                if not at_observation and in_pick_zone:
                    reason = (
                        f"取货区移动中跟踪 kind={target.kind}；"
                        f"等待稳定候选 {self.min_samples}/{self.tracker_window_frames} 帧"
                    )
                elif not at_observation:
                    reason = (
                        f"等待进入取货区后再识别 kind={target.kind}；"
                        f"当前 y={self.sensor.y} navigation_state={self.navigation_state}"
                    )
                else:
                    reason = (
                        f"dry-run 等待 kind={target.kind} 的稳定检测；"
                        f"当前可见 ArUco={list(self.sensor.aruco_ids)}；保持停车"
                    )
                self.get_logger().info(reason)
                self.last_report_at = now
            return
        if not at_observation and in_pick_zone:
            self.get_logger().info(
                f"移动中已获得稳定 {candidate.kind} 候选，开始规划目标货架；"
                f"当前底盘继续沿安全取货路线，world={candidate.world}"
            )

        # 已经锁定的有效候选是单目标闭环的事实来源；后续短暂丢失 ArUco
        # 或检测抖动不能覆盖它，避免抓取计划突然变成无货位/错误货位。
        if target.candidate_locked:
            if now - self.last_report_at >= 3.0:
                self.get_logger().info(
                    f"dry-run 候选已锁定，保持货位={target.candidate.get('slot') if target.candidate else None}；"
                    f"target={target.target_id}"
                )
                self.last_report_at = now
            return

        # 优先使用唯一 ArUco 绑定货位；若 ArUco 暂时不可用，则只在
        # world 坐标贴近固定槽位几何时使用保守 fallback，明确记录来源。
        lock_key = ""
        association_mode = "aruco"
        candidate_slot = candidate.slot
        candidate_ids = tuple(candidate.aruco_ids)
        geometry_slot = self._world_slot_geometry(candidate.world) if candidate_slot is None else None
        if geometry_slot is not None and not candidate_ids:
            geometry_id, geometry_label = geometry_slot
            candidate_slot = geometry_label
            candidate.slot = geometry_label
            association_mode = "world_slot_geometry_fallback"
            self.get_logger().info(
                f"视觉坐标近邻货位 fallback（非ArUco）：slot={geometry_label} "
                f"aruco_id={geometry_id} world={candidate.world}"
            )

        # 固定 Baseline 开发兜底：只用视觉 world 与明确配置的固定目标
        # 坐标做近邻关联，不伪造真实ArUco检测；随机赛不设置这些变量。
        if (
            self.fixed_target_slot
            and self.fixed_target_world is not None
            and candidate_slot is None
        ):
            distance = float(np.linalg.norm(
                np.asarray(candidate.world, dtype=float)
                - np.asarray(self.fixed_target_world, dtype=float)
            ))
            if distance <= self.fixed_target_world_tolerance:
                candidate_slot = self.fixed_target_slot
                association_mode = "fixed_baseline_world_gate"
                candidate.slot = candidate_slot
                self.get_logger().info(
                    f"固定Baseline视觉近邻兜底：world距离={distance:.3f}m，"
                    f"绑定目标槽位={candidate_slot}"
                )

        if self.fixed_target_slot and candidate_slot != self.fixed_target_slot:
            if now - self.last_report_at >= 2.0:
                self.get_logger().info(
                    f"忽略非目标货位：当前={candidate_slot}，固定目标={self.fixed_target_slot}；"
                    "继续等待目标实例"
                )
                self.last_report_at = now
            return

        if candidate_slot is not None and len(candidate_ids) == 1:
            lock_key = f"{candidate.track_id}|{candidate_slot}|{candidate_ids[0]}"
        elif candidate_slot is not None and not candidate_ids:
            lock_key = f"{candidate.track_id}|geometry|{candidate_slot}|{candidate.kind}"
        elif not candidate_ids and candidate_slot is None:
            # 没有货位标记时，只用 kind + 稳定世界坐标形成 plan-only 锁定键。
            # 坐标按厘米量化，允许毫米级检测抖动但避免锁定不同实例。
            wx, wy, wz = candidate.world
            lock_key = (
                f"{candidate.track_id}|world|{candidate.kind}|"
                f"{round(wx, 2)}|{round(wy, 2)}|{round(wz, 2)}"
            )
            association_mode = "world_coordinate"

        if not lock_key:
            if target.pending_lock_key and now - target.pending_lock_last_seen > self.aruco_hold_seconds:
                target.pending_lock_key = ""
                target.pending_lock_streak = 0
            if now - self.last_report_at >= 2.0:
                self.get_logger().warning(
                    f"目标 {target.target_id} 检测到，但当前货位标记未唯一稳定；"
                    f"ids={list(candidate.aruco_ids)} slot={candidate.slot}，不覆盖有效候选"
                )
                self.last_report_at = now
            return

        if target.pending_lock_key == lock_key:
            target.pending_lock_streak += 1
        else:
            target.pending_lock_key = lock_key
            target.pending_lock_streak = 1
        target.pending_lock_last_seen = now

        if target.pending_lock_streak < self.slot_lock_frames:
            if now - self.last_report_at >= 2.0:
                self.get_logger().info(
                    f"目标 {target.target_id} 货位确认中：{lock_key} "
                    f"{target.pending_lock_streak}/{self.slot_lock_frames}"
                )
                self.last_report_at = now
            return

        target.candidate = {
            "track_id": candidate.track_id,
            "kind": candidate.kind,
            "confidence": round(candidate.confidence, 4),
            "world": [round(v, 4) for v in candidate.world],
            "aruco_ids": list(candidate.aruco_ids),
            "slot": candidate.slot,
            "samples": candidate.samples,
            "association_mode": association_mode,
        }
        target.candidate_locked = True
        target.last_log_key = json.dumps(target.candidate, sort_keys=True, ensure_ascii=False)
        self._publish_navigation_goal(target, candidate)
        self._publish_target_status("candidate_locked")
        self.get_logger().info(
            f"目标 {target.target_id} 已锁定候选：track={candidate.track_id} kind={candidate.kind} "
            f"slot={candidate.slot} aruco={list(candidate.aruco_ids)} "
            f"world={candidate.world} samples={candidate.samples}；"
            "下一步应导航到货架（当前 dry-run 不执行）"
        )

    def stop_and_shutdown(self) -> None:
        self._publish_zero()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 安全 dry-run 任务执行器")
    parser.add_argument("--duration", type=float, default=30.0, help="运行秒数，0 表示持续运行")
    parser.add_argument("--min-samples", type=int, default=5, help="稳定检测所需的最近帧数")
    parser.add_argument("--min-confidence", type=float, default=0.65, help="最低检测置信度")
    parser.add_argument("--camera", default="head", choices=("head", "left", "right"))
    parser.add_argument("--detection-topic", default=DETECTION_TOPIC)
    parser.add_argument("--shelf-normal-x", type=float, default=0.0)
    parser.add_argument("--shelf-normal-y", type=float, default=1.0)
    parser.add_argument("--approach-distance", type=float, default=0.75)
    parser.add_argument("--slot-lock-frames", type=int, default=5, help="同一唯一 ArUco 连续确认周期数")
    parser.add_argument("--aruco-hold-seconds", type=float, default=0.8, help="短暂丢失 ArUco 时保留待确认状态的秒数")
    parser.add_argument("--tracker-window-frames", type=int, default=7, help="移动中候选跟踪窗口帧数")
    parser.add_argument("--no-zero", action="store_true", help="导航器接管底盘时，不发布零速度")
    parser.add_argument("--expected-target-count", type=int, default=None,
                        help="本轮任务目标数校验；也可用 SUPERMARKET_EXPECTED_TARGET_COUNT")
    parser.add_argument("--target-level", type=int, choices=(0, 1, 2, 3), default=None,
                        help="目标商品层位过滤：0=不限制，1/2/3=只接受对应货架层；也可用 SUPERMARKET_TARGET_LEVEL")
    args = parser.parse_args()

    if args.expected_target_count is not None:
        os.environ["SUPERMARKET_EXPECTED_TARGET_COUNT"] = str(max(0, args.expected_target_count))
    if args.target_level is not None:
        os.environ["SUPERMARKET_TARGET_LEVEL"] = str(args.target_level)

    rclpy.init()
    node = DryRunExecutor(
        duration=args.duration,
        min_samples=args.min_samples,
        min_confidence=args.min_confidence,
        camera=args.camera,
        detection_topic=args.detection_topic,
        shelf_normal_x=args.shelf_normal_x,
        shelf_normal_y=args.shelf_normal_y,
        approach_distance=args.approach_distance,
        publish_zero=not args.no_zero,
        slot_lock_frames=args.slot_lock_frames,
        aruco_hold_seconds=args.aruco_hold_seconds,
        tracker_window_frames=args.tracker_window_frames,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning("收到中断，保持零速度退出")
    finally:
        node.stop_and_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

