#!/usr/bin/env python3
"""随机45商品比赛编排器：增量库存地图 + 任务驱动抓取。

安全默认：只发布导航/抓取计划，不发布 /cmd_vel，不直接控制机械臂。
真实动作仍需由 single_item_executor 显式使用 --execute --confirm single_item 管理。
ArUco 仅作可选校验；主定位链路为 YOLO + RGB-D 世界坐标 + 固定货架几何。
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import subprocess
import sys
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, String
from vision_msgs.msg import Detection3DArray

from inventory_map import SHELF_ORDER_FOR_ROUTE, SLOT_X, InventoryEntry, InventoryMap

TASK_TOPIC = "/supermarket_sorting/task"
DETECTION_TOPIC = "/multiclass/detections"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
NAV_STATUS_TOPIC = "/competition/navigation_status"
NAV_GOAL_TOPIC = "/competition/navigation_goal"
NAV_META_TOPIC = "/competition/navigation_goal_meta"
TARGET_STATUS_TOPIC = "/competition/target_status"
GRASP_GOAL_TOPIC = "/competition/grasp_goal"
GRASP_META_TOPIC = "/competition/grasp_goal_meta"
PLACE_STATUS_TOPIC = "/competition/place_status"
INVENTORY_TOPIC = "/competition/inventory_map"
FLOW_STATUS_TOPIC = "/competition/inventory_flow_status"

NAV_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ---- 自动巡游扫描 + 自主执行常量（固定记录 45 个货位）----
HEAD_CMD_TOPIC = "/head_forward_position_controller/commands"
SHELF_STAGING_Y = 2.475   # 官方黄线：返程先回到这里，再沿货架横向换列
OBSERVE_Y = float(os.getenv("SUPERMARKET_OBSERVE_Y", "2.40"))
# 逐排扫描观察线后移，给头部相机留出更完整的三层视野。
PREGRASP_Y = OBSERVE_Y       # 兼容旧名称：视觉复核仍在观察线进行

def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return float(default)


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return int(default)


# 机械臂预抓取安全线：视觉复核完成后，底盘先沿货架法线后退，
# 让 deploy 的负向 DY 偏置不会把末端推入 KDL 的近距离不可达区。
ARM_PREGRASP_Y = _env_float("SUPERMARKET_ARM_PREGRASP_Y", 2.35)
ARM_ODOM_STABLE_SAMPLES = max(3, _env_int("SUPERMARKET_ARM_ODOM_STABLE_SAMPLES", 5))
ARM_ODOM_STABLE_POS_TOL = _env_float("SUPERMARKET_ARM_ODOM_STABLE_POS_TOL", 0.015)
ARM_ODOM_STABLE_YAW_TOL = _env_float("SUPERMARKET_ARM_ODOM_STABLE_YAW_TOL", 0.05)
PLAN_GATE_TIMEOUT = max(10.0, _env_float("SUPERMARKET_PLAN_GATE_TIMEOUT", 60.0))
TARGET_APPROACH_OFFSET_X = 0.12  # 目标列右侧安全余量，降低边列 IK 横向偏差
# 用户指定的实际扫描顺序：E(最右) → D → C → B；A 仅在目标缺失时补扫。
# 不按字母排序，也不在 BCDE 扫描后临时追加 A；一轮固定建立 45 货位地图。
MAPPING_SHELVES = ("E", "D", "C", "B")
TASK_SHELVES = frozenset(("A", "B", "C", "D", "E"))
# 相机俯仰：负值下俯；中层沿用官方约 -0.6（参照 observe_shelves_capture）
PITCH_SCAN = (
    (float(os.getenv("SUPERMARKET_PITCH_UPPER", "-0.20")), "upper"),
    (float(os.getenv("SUPERMARKET_PITCH_MIDDLE", "-0.45")), "middle"),
    (float(os.getenv("SUPERMARKET_PITCH_LOWER", "-0.70")), "lower"),
)
PITCH_DWELL_S = float(os.getenv("SUPERMARKET_PITCH_DWELL_S", "1.5"))   # 比赛时限 420s
# 配送站位参数化：桌子(delivery_table)中心 (-1.940,-3.410)，桌面南缘 y=-3.19。
# 原站位 y=-2.80 时车头离桌沿仅 ~0.09m（实测"离桌子太近"）；外移到 y=-2.55。
DELIVERY_GOAL = (
    _env_float("SUPERMARKET_DELIVERY_GOAL_X", -1.94),
    _env_float("SUPERMARKET_DELIVERY_GOAL_Y", -2.55),
    math.radians(_env_float("SUPERMARKET_DELIVERY_GOAL_YAW_DEG", -90.0)),
)   # 配送区北缘内侧放置站位（referee delivery_base）
DELIVERY_BASE_X = (-2.420, -1.460)
DELIVERY_BASE_Y = (-3.880, -2.620)
# 检测坐标吸附到最近同类货位的半径：覆盖实测 8cm 误差，同时小于相邻货位
# 间距（同层 0.22m / 同列 0.33m）的一半，避免吸到隔壁。
SLOT_SNAP_RADIUS = _env_float("SUPERMARKET_SLOT_SNAP_RADIUS", 0.13)
# 槽位吸附开关，默认关闭。slot_truth 读的是静态默认布局；随机布局下商品
# 位置已重排，吸附/就近回退会把正确检测扳到错误位置（2026-09-12 对账：
# 吸附后地图与真实布局仅 1/34 相符，纯检测聚类 89% 召回/中位 3.5cm 偏差）。
# 仅固定基线标定（SUPERMARKET_SLOT_SNAP=1）时启用。
SLOT_SNAP_ENABLED = os.getenv("SUPERMARKET_SLOT_SNAP", "0").strip().lower() in {"1", "true", "yes", "on"}
# creep 失败后的重对准重试次数（回退 10cm 后重新 deploy→creep）
CREEP_RETRY_MAX = int(_env_float("SUPERMARKET_CREEP_RETRY_MAX", 2))
# 单个目标失败后先收臂到安全位再继续下一目标；连续失败到上限才终结整轮。
# 此前"一个目标失败即停止本轮"让 5 个任务的比赛只跑了 1 个就结束。
MAX_CONSECUTIVE_FAILURES = int(_env_float("SUPERMARKET_MAX_CONSECUTIVE_FAILURES", 3))
# arm worker 硬超时兜底：worker 自身超时逻辑可能失效（实测 safe_arm 在等
# joint_states 时挂了 50 分钟不退出），整个流程就卡死在收尾阶段。
# 超过该值一律强杀并推进，保证比赛流程能跑完而不是挂住。
# 各 arm worker 正常耗时都 <50s（safe_pose 5s / deploy 15-25s / creep 8-40s /
# retreat 3s / slide_reset 1s / place 10s），90s 已很宽裕；此前 240s 让
# safe_pose 卡死时要白等 4 分钟（比赛时限 420s 里这是致命的）。
ARM_WORKER_HARD_TIMEOUT = _env_float("SUPERMARKET_ARM_WORKER_HARD_TIMEOUT", 90.0)
# 配送段导航看门狗：RTF<1 时 5-8m 走 40-60s，加避障绕行余量取 150s。
DELIVER_NAV_TIMEOUT = _env_float("SUPERMARKET_DELIVER_NAV_TIMEOUT", 150.0)
# 抓前近距复核：机器人到预抓取线、伸爪之前，用近距离检测更新目标坐标。
# 远距离扫描的 RGB-D 投影偶发大偏差（2026-09-12 实测单罐 z 偏 29cm、
# 常态 y/z 偏 2-5cm，足以让指头平面错过罐身）；机器人此刻已贴近货架，
# 同 kind 的最新检测精度远高于扫描期。仅取旧目标 REFINE_MAX_RADIUS 内
# 的近距样本，避免修到旁边的同类副本。2026-09-12 提议并实现。
REFINE_WINDOW_S = _env_float("SUPERMARKET_REFINE_WINDOW_S", 2.5)
REFINE_MIN_SAMPLES = max(2, int(_env_float("SUPERMARKET_REFINE_MIN_SAMPLES", 4)))
REFINE_MAX_RADIUS = _env_float("SUPERMARKET_REFINE_MAX_RADIUS", 0.40)
# 检测 y 偏差补偿：默认 0。实测检测中位数长期看无偏（各轮 -4.4~+2.9cm
# 双向波动，是随机噪声而非系统差），固定补偿会双向过头。近距复核的中位数
# （REFINE_*）才是有效校正。保留环境变量供后续实测再开。
DETECTION_Y_BIAS = _env_float("SUPERMARKET_DETECTION_Y_BIAS", 0.0)
# 跳巡游扫描：比赛时限 420s，而 E→D→C→B→A 全扫描实测要 5-8 分钟。
# 任务清单（目标 kind）与 45 槽位布局都是已知的，可直接用布局真值预填地图
# 并立即进入执行阶段，把扫描时间全部省掉。
SKIP_SCAN = os.getenv("SUPERMARKET_SKIP_SCAN", "0").strip() == "1"
# 导航连续失败上限：此前只重启不放弃，车被障碍挡住时会无限"超时→重启"
# 循环卡死整轮（实测卡 27 分钟）。超过上限即判当前目标失败并交给
# 失败恢复链路（收臂→跳过→下一目标）。
NAV_RETRY_MAX = int(_env_float("SUPERMARKET_NAV_RETRY_MAX", 3))
# 倾角监控：机器人翻倒时 odom 的 roll/pitch 会大幅偏离（实测翻车后
# 流程还傻跑了十几分钟无人发现）。超过阈值立即停止本轮。
TILT_ABORT_DEG = _env_float("SUPERMARKET_TILT_ABORT_DEG", 45.0)
# 载货配送限速：夹持商品 + 机械臂前伸 => 重心前移，空车不翻的速度在这里会翻
# （实测 deliver 阶段倾角 47.1°）。必须显著低于 cruise_max_speed。
DELIVERY_SPEED = _env_float("SUPERMARKET_DELIVERY_SPEED", 0.80)
# 角速度上限：0.45 rad/s 时一次 180° 掉头要 7 秒，是流程里最贵的纯浪费动作。
# 翻车根因是"线速度+加减速惯性"（09-11 实测），不是角速度；原地转向时线速度
# 为零、且掉头发生在 slide_reset 之后（升降柱已降），重心低，可安全放宽。
# 载货段仍取保守值（夹持商品是翻车敏感场景）。
NAV_MAX_ANGULAR = _env_float("SUPERMARKET_NAV_MAX_ANGULAR", 0.90)
DELIVERY_MAX_ANGULAR = _env_float("SUPERMARKET_DELIVERY_MAX_ANGULAR", 0.60)
NAV_GOAL_TOPIC_EXEC = NAV_GOAL_TOPIC
DEFAULT_MAPPING_SPEED = 0.35   # 货架前巡游扫描安全低速
DEFAULT_CRUISE_SPEED = 1.2     # 长直 clear 段（去配送/返程）巡航速度
DEFAULT_OBSTACLE = 0.5
PREGRASP_GOAL_TOLERANCE = 0.05
# 0.10 rad（5.7°）在臂前伸 0.8m 处 = ~8cm 横向偏差（实测 creep x 超差、
# 苹果偏到夹爪一侧的根因之一）。收紧到 0.06（≈3.4° → ~4.8cm），
# 仍留有余量避免门禁频繁重试。
PREGRASP_ODOM_YAW_TOLERANCE = 0.06
PREGRASP_ODOM_GATE_MAX_RETRIES = 3
# 低置信度检测先进入跟踪，只有更长时间的空间稳定后才写入库存地图。
TRACK_STABILITY_WINDOW = max(6, _env_int("SUPERMARKET_TRACK_STABILITY_WINDOW", 8))

# 动作 worker 表：脚本 + 固定参数（参照 single_item_executor.WORKER_SCRIPTS 的 spawn 约定）。
# 这些 worker 只控制机械臂/夹爪/升降柱，不控底盘；底盘由本 executor 独占。
# servo：creep 到位后、闭爪前的手眼伺服——测"手指-商品"横向/高度偏差，
# 超阈值则回退重部署（复用 creep-retry 的 backoff→deploy→creep 通道）。
ARM_ACTIONS = ("safe_pose", "deploy", "creep", "servo", "close_gripper", "lift", "retreat", "slide_reset")
POST_PLACE_SAFE_POSE = "post_place_safe_pose"
# 手眼伺服：偏差超过 SERVO_APPLY_THRESHOLD 则带修正量回退重部署；
# 最多 SERVO_RETRY_MAX 次（每次部署 ~20s，比赛时限内可承受）。
SERVO_RESULT_PATH = "debug_data/servo_result.json"
# 8mm 低于伺服的团块质心噪声底（±1-2cm），会永远触发重部署；1.5cm
# 与"6.5cm 罐身可容忍的横向偏差"匹配，小残差直接闭爪。
SERVO_APPLY_THRESHOLD = _env_float("SUPERMARKET_SERVO_APPLY_THRESHOLD", 0.015)
SERVO_RETRY_MAX = int(_env_float("SUPERMARKET_SERVO_RETRY_MAX", 2))
WORKER_CFG = {
    "servo": ("hand_eye_servo_controller.py", ["--execute", "--confirm", "servo", "--timeout", "40"]),
    # 伺服重部署序列：先退 30cm 让手完全脱离货架前沿，再抬手退出格子，
    # 再 deploy。顺序反了会刮层板/顶上层板（在格内先抬手会撞 L3 板）。
    "backoff_far": ("retreat_controller.py", ["--execute", "--confirm", "retreat", "--distance", "0.30", "--timeout", "60"]),
    # 伺服重部署序列第二步：把手从货架格子里竖直抽出来。从格内姿态直接
    # 关节插值重部署会刮蹭层板（max_arm_err 卡 0.23 rad 不收敛 → 硬超时，
    # 实测三次），先退远+抬手再 deploy 就顺畅了。
    "slide_up": ("slide_reset_controller.py", ["--execute", "--confirm", "slide_reset", "--target", "0.87", "--timeout", "30"]),
    "safe_pose": ("safe_arm_controller.py", ["safe_pose", "--execute", "--confirm", "safe_pose", "--timeout", "90"]),
    "deploy": ("grasp_deploy_controller.py", ["--execute", "--confirm", "deploy", "--timeout", "180"]),
    "creep": ("grasp_creep_controller.py", ["--execute", "--confirm", "creep", "--pickup-approach", "--object-stop-only", "--timeout", "180"]),
    "close_gripper": ("close_gripper_controller.py", ["--execute", "--confirm", "close_gripper", "--timeout", "120"]),
    "lift": ("lift_controller.py", ["--execute", "--confirm", "lift", "--timeout", "120"]),
    "retreat": ("retreat_controller.py", ["--execute", "--confirm", "retreat", "--timeout", "120"]),
    # creep 失败后的小幅回退（10cm），用于重对准重试——不是正常撤出的 0.55m
    "backoff": ("retreat_controller.py", ["--execute", "--confirm", "retreat", "--distance", "0.10", "--timeout", "60"]),
    # 高层抓取后机身（slide）复位到安全巡航高度：否则保持 0.87 高位去配送，
    # 机身+机械臂会撞料框桌沿（主人实测观察）。
    "slide_reset": ("slide_reset_controller.py", ["--execute", "--confirm", "slide_reset", "--timeout", "30"]),
    "place": ("place_controller.py", ["--execute", "--confirm", "place", "--timeout", "30"]),
    "post_place_safe_pose": ("safe_arm_controller.py", ["post_place_safe_pose", "--execute", "--confirm", "post_place_safe_pose", "--timeout", "90"]),
}

def _load_slot_truth(layout_path: Path) -> list[dict[str, object]]:
    """45 货位真值表：商品必定位于其中之一，用于校正 YOLO+RGB-D 的检测坐标。

    比赛流程用检测定位（标定用布局真值），实测末端对位误差可达 8cm，而 creep
    的 x 容差只有 7cm —— 吸附到最近同类货位可把精度提到 ±1cm。
    """
    slots: list[dict[str, object]] = []
    try:
        with open(layout_path, encoding="utf-8") as handle:
            for entry in json.load(handle):
                wp = entry.get("world_position")
                if not wp or len(wp) < 3:
                    continue
                slots.append(
                    {
                        "slot": f"{entry.get('shelf', '')}/{entry.get('level', '')}/{entry.get('column', '')}",
                        "kind": str(entry.get("object_kind", "")).strip().lower(),
                        "world": tuple(float(v) for v in wp[:3]),
                    }
                )
    except (OSError, ValueError):
        return []
    return slots


@dataclass
class Track:
    kind: str
    samples: deque = field(default_factory=lambda: deque(maxlen=12))
    last_seen: float = 0.0

@dataclass
class Target:
    target_id: str
    kind: str
    status: str = "pending"
    candidate: InventoryEntry | None = None
    error: str | None = None
    reserved: bool = False

class InventoryCompetitionExecutor(Node):
    def __init__(self, *, map_timeout: float, min_map_seconds: float, min_samples: int, min_confidence: float, map_path: str, no_publish: bool,
                 execute: bool = False, confirm: str = "", mapping_speed: float = DEFAULT_MAPPING_SPEED,
                 cruise_speed: float = DEFAULT_CRUISE_SPEED, obstacle_distance: float = DEFAULT_OBSTACLE,
                 arm_timeout: float = 180.0, scan_only: bool = False, stop_after_ik: bool = False, stop_after_close: bool = False) -> None:
        super().__init__("inventory_competition_executor")
        self.map_timeout = max(10.0, float(map_timeout))
        self.min_map_seconds = max(0.0, float(min_map_seconds))
        self.min_samples = max(3, int(min_samples))
        self.min_confidence = float(min_confidence)
        self.track_min_confidence = max(
            0.05,
            min(
                self.min_confidence,
                _env_float("SUPERMARKET_TRACK_MIN_CONFIDENCE", 0.15),
            ),
        )
        self.track_stability_window = max(self.min_samples, TRACK_STABILITY_WINDOW)
        self.map_path = map_path
        self.no_publish = bool(no_publish)
        self.started_at = time.monotonic()
        self.map = InventoryMap()
        self.tracks: dict[str, list[Track]] = defaultdict(list)
        self.creep_retry = 0
        self.servo_retry = 0
        self.servo_redeploy = False   # 伺服重部署序列：slide_up → backoff → deploy
        self._servo_depth_extra = 0.0  # 伸入深度修正（只进 creep 停止线）
        self.depth_probe_count = 0     # 闭爪落空后的深度探测计数
        self._servo_standoff = 0.0
        self.consecutive_failures = 0
        self.recovering_from_failure = False
        self.nav_retry = 0
        self.tilt_deg = 0.0
        self.tilt_reported = False
        self.targets: list[Target] = []
        self.current_index = 0
        self.task_received = False
        self.mapping_active = True
        self.map_completed_at: float | None = None
        self.last_task_message = ""
        self.last_nav_key = ""
        self.last_nav_publish = 0.0
        self.nav_state = "unknown"
        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.goal: list[float] | None = None
        self.done = False
        self.last_log = 0.0

        # ---- execute 自主运行状态（仅 --execute --confirm random5 时生效）----
        self.execute_enabled = bool(execute) and confirm == "random5"
        self.rejected_execute = bool(execute) and confirm != "random5"
        self.mapping_max_speed = max(0.1, float(mapping_speed))
        self.cruise_max_speed = max(0.2, float(cruise_speed))
        self.obstacle_distance = max(0.25, float(obstacle_distance))
        # 仅由本次运行环境控制的无动态障碍单跑，不删除官方障碍物资产。
        self.no_obstacles = os.getenv("SUPERMARKET_RANDOMIZE_OBSTACLES", "1").strip().lower() in {
            "0", "false", "no", "off",
        }
        self.arm_timeout = max(10.0, float(arm_timeout))
        self.scan_only = bool(scan_only)
        self.stop_after_ik = bool(stop_after_ik)
        self.stop_after_close = bool(stop_after_close)
        # scan-only 与正式 execute 共享 E→D→C→B 顺序；A 只在目标缺失时补扫。
        self.scan_shelves = MAPPING_SHELVES
        # Explicit task lists are competition/test requests whose order is part
        # of the contract. Random task mode keeps shortest-route reordering.
        self.fixed_task_order = os.getenv("SUPERMARKET_TASK_ORDER_LOCKED", "0").strip().lower() in {
            "1", "true", "yes", "on",
        }
        self.phase = "idle"            # idle|sweep|target_nav|pregrasp_nav|pregrasp_stable|plan_gate|arm|deliver|place|done
        self.mapping_mode = "idle"     # idle|drive|scan|wait
        self.sweep_shelf_index = 0
        self.pitch_index = 0
        self.pitch_started_at: float | None = None
        self.scan_started_at: float | None = None
        self.deliver_started_at: float | None = None
        self.arm_index = 0
        self.nav_worker: subprocess.Popen[Any] | None = None
        self.arm_worker: subprocess.Popen[Any] | None = None
        self.plan_gate_worker: subprocess.Popen[Any] | None = None
        self.pregrasp_stable_started_at: float | None = None
        self.pregrasp_stable_samples = 0
        self.pregrasp_last_pose: tuple[float, float, float] | None = None
        self.pregrasp_odom_gate_failures = 0
        self.post_place_safe_pose_active = False
        self.exec_success = False
        self.baseline_dir = Path(__file__).resolve().parent
        # 45 货位真值表：把检测坐标吸附到最近同类货位，消除 8cm 级检测抖动。
        # 必须在 baseline_dir 之后初始化（顺序敏感）。
        self.slot_truth = _load_slot_truth(
            self.baseline_dir / "official_baseline" / "examples" / "supermarket_sorting" / "retail_competition_layout.json"
        )
        if self.slot_truth:
            self.get_logger().info(
                f"[exec] 货位真值表已加载：{len(self.slot_truth)} 槽位，吸附半径 {SLOT_SNAP_RADIUS}m"
            )
        # 分段底盘驾驶：每段 (x,y,yaw,pickup,速度)；pickup 段忽略货架本体雷达
        self.drive_goals: list[tuple[float, float, float, bool, float, bool]] = []
        self.observe_row = False   # 已在货架前观察行（相邻货架可 pickup 横向移动）
        self.return_route_active = False  # place 后返回货架期间保持预规划控制
        self._drive_pickup = False
        self._drive_pickup_transit = False

        task_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, TASK_TOPIC, self._on_task, task_qos)
        self.create_subscription(Detection3DArray, DETECTION_TOPIC, self._on_detections, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, NAV_STATUS_TOPIC, self._on_nav_status, 10)
        self.create_subscription(String, PLACE_STATUS_TOPIC, self._on_place_status, 10)
        self.inventory_pub = self.create_publisher(String, INVENTORY_TOPIC, NAV_QOS)
        self.flow_pub = self.create_publisher(String, FLOW_STATUS_TOPIC, NAV_QOS)
        self.target_pub = self.create_publisher(String, TARGET_STATUS_TOPIC, NAV_QOS)
        self.nav_goal_pub = self.create_publisher(PoseStamped, NAV_GOAL_TOPIC, NAV_QOS)
        self.nav_meta_pub = self.create_publisher(String, NAV_META_TOPIC, NAV_QOS)
        self.grasp_goal_pub = self.create_publisher(PoseStamped, GRASP_GOAL_TOPIC, 10)
        self.grasp_meta_pub = self.create_publisher(String, GRASP_META_TOPIC, NAV_QOS)
        self.head_pub = self.create_publisher(Float64MultiArray, HEAD_CMD_TOPIC, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            f"库存编排器启动：map_timeout={self.map_timeout:.0f}s，min_map_seconds={self.min_map_seconds:.1f}s，"
            f"track_confidence={self.track_min_confidence:.2f}，stability_window={self.track_stability_window}；"
            "ArUco非硬依赖，默认只做YOLO+RGB-D+货架几何；"
            + (
                "SCAN-ONLY：真实底盘扫描，扫描完成后停车，不启动机械臂"
                if self.scan_only
                else ("EXECUTE：自动巡游扫描 + 底盘/机械臂 worker 由本节点 spawn" if self.execute_enabled else "PLAN-ONLY：不直接控制机器人")
            )
        )
        if self.execute_enabled:
            self.get_logger().warning(
                f"自主运行参数：巡游限速={self.mapping_max_speed:.2f}m/s，巡航限速={self.cruise_max_speed:.2f}m/s，"
                f"障碍距离={self.obstacle_distance:.2f}m；首轮扫描 E→D→C→B，目标缺失时补扫 A；真实动作先 safe_pose 再 deploy→creep→close_gripper；"
                "执行阶段按当前路线预计时间动态选下一目标；"
                + ("本轮无障碍直达模式" if self.no_obstacles else "本轮随机障碍模式")
            )
        elif self.rejected_execute:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm random5")
            self.done = True

    def _on_task(self, msg: String) -> None:
        if msg.data == self.last_task_message:
            return
        try:
            payload = json.loads(msg.data)
            raw = payload.get("targets")
            if not isinstance(raw, list) or not raw:
                raise ValueError("targets must be a non-empty list")
            targets = []
            seen = set()
            for item in raw:
                target_id, kind = str(item.get("id", "")), str(item.get("kind", "")).strip()
                if not target_id or not kind or target_id in seen:
                    raise ValueError("each target needs unique id and kind")
                seen.add(target_id)
                targets.append(Target(target_id, kind))
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().error(f"拒绝非法任务：{exc}")
            return
        self.last_task_message = msg.data
        self.map.run_prefix = str(payload.get("run_prefix", ""))
        self.targets = targets
        self.current_index = 0
        self.task_received = True
        self.mapping_active = True
        self.map_completed_at = None
        self._publish_flow("task_received")
        self._save_map()
        self.get_logger().info(f"收到任务：{len(targets)}个目标；先建立45槽位初始地图")
        if SKIP_SCAN:
            self._preload_map_from_layout()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.x, self.y = float(p.position.x), float(p.position.y)
        q = p.orientation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        self.yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        # 倾倒角：机体 z 轴与世界 z 轴的夹角（R[2][2] = 1 - 2(qx^2+qy^2)）
        cos_tilt = max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))
        self.tilt_deg = math.degrees(math.acos(cos_tilt))
        if self.tilt_deg > TILT_ABORT_DEG and not self.tilt_reported and self.execute_enabled:
            self.tilt_reported = True
            self.get_logger().error(
                f"[exec] 机器人倾角 {self.tilt_deg:.1f}° > 阈值 {TILT_ABORT_DEG:.0f}°，"
                "疑似翻倒，立即停止本轮（不再空跑）"
            )
            self._finish_exec(False)

    def _on_detections(self, msg: Detection3DArray) -> None:
        now = time.monotonic()
        for det in msg.detections:
            if not det.results:
                continue
            result = det.results[0]
            kind = str(result.hypothesis.class_id or "").strip()
            confidence = float(result.hypothesis.score)
            if not kind or confidence < self.track_min_confidence:
                continue
            p = result.pose.pose.position
            world = (float(p.x), float(p.y), float(p.z))
            if not all(math.isfinite(v) for v in world):
                continue
            # 吸附到最近同类货位真值：消除检测抖动（实测最大 8cm）。
            # ⚠️ 仅限固定基线（SUPERMARKET_SLOT_SNAP=1）：slot_truth 读的是
            # 静态默认布局，随机布局下商品位置已重排——吸附和"就近同类回退"
            # 会把本来大致正确的检测硬扳到错误位置（2026-09-12 实测 shupian
            # 真实位置 D/L2 (0.70,3.243,0.956)，静态表写 B/L1/C3，机器人
            # 对着 1.3m 外的空气闭爪）。随机流程回归纯检测+多帧稳定。
            if SLOT_SNAP_ENABLED:
                snapped = self._snap_to_slot(kind, world)
                if snapped is not None:
                    drift = math.sqrt(sum((world[i] - snapped[i]) ** 2 for i in range(3)))
                    if not getattr(self, "_snap_logged", None):
                        self._snap_logged = 0
                    if self._snap_logged < 12:
                        self._snap_logged += 1
                        self.get_logger().info(
                            f"[exec] 槽位吸附：{kind} 检测=({world[0]:.3f},{world[1]:.3f},{world[2]:.3f}) "
                            f"→ 真值=({snapped[0]:.3f},{snapped[1]:.3f},{snapped[2]:.3f}) 修正 {drift * 100:.1f}cm"
                        )
                    world = snapped
                elif self.slot_truth:
                    # 吸附未命中（检测误差 > 半径）时不退回检测原值——那正是造成
                    # 夹取不准的东西。改为就近取同类货位真值：任务只要求品类，
                    # 任意同类副本都算有效，而货位坐标是已知真值，精度远高于视觉。
                    nearest = self._nearest_same_kind_slot(kind)
                    if nearest is not None:
                        if not getattr(self, "_snap_fallback", 0):
                            self._snap_fallback = 0
                        self._snap_fallback += 1
                        if self._snap_fallback <= 8:
                            self.get_logger().warning(
                                f"[exec] 吸附未命中，改用就近同类货位真值：{kind} "
                                f"检测=({world[0]:.3f},{world[1]:.3f},{world[2]:.3f}) → "
                                f"真值=({nearest[0]:.3f},{nearest[1]:.3f},{nearest[2]:.3f})"
                            )
                        world = nearest
            tracks = self.tracks[kind]
            match = None
            best = 0.16
            for track in tracks:
                if not track.samples:
                    continue
                old = track.samples[-1][1]
                d = math.sqrt(sum((world[i] - old[i]) ** 2 for i in range(3)))
                if d < best:
                    best, match = d, track
            if match is None:
                match = Track(kind)
                tracks.append(match)
            match.samples.append((now, world, confidence))
            match.last_seen = now

    def _snap_to_slot(self, kind: str, world: tuple[float, float, float]) -> tuple[float, float, float] | None:
        """把检测坐标吸附到最近的同类货位真值；无同类货位或距离过远则不吸附。"""
        if not self.slot_truth:
            return None
        kind_key = str(kind).strip().lower()
        best: tuple[float, float, float] | None = None
        best_distance = SLOT_SNAP_RADIUS
        for slot in self.slot_truth:
            if slot["kind"] != kind_key:
                continue
            slot_world = slot["world"]
            distance = math.sqrt(sum((world[i] - slot_world[i]) ** 2 for i in range(3)))
            if distance < best_distance:
                best_distance, best = distance, slot_world
        return best

    def _nearest_same_kind_slot(self, kind: str) -> tuple[float, float, float] | None:
        """返回离机器人当前位置最近的同类货位真值坐标。

        任务只要求品类（任意同类副本有效），而货位坐标是布局真值，
        精度远高于 YOLO+RGB-D 的估计值（实测偏差 8~11cm）。
        """
        if not self.slot_truth:
            return None
        kind_key = str(kind).strip().lower()
        candidates = [s for s in self.slot_truth if s["kind"] == kind_key]
        if not candidates:
            return None
        rx = float(self.x) if self.x is not None else 0.0
        ry = float(self.y) if self.y is not None else 0.0
        best = min(
            candidates,
            key=lambda s: (s["world"][0] - rx) ** 2 + (s["world"][1] - ry) ** 2,
        )
        return best["world"]

    def _stable_world(self, track: Track, *, after: float | None = None) -> tuple[tuple[float, float, float], float, int] | None:
        now = time.monotonic()
        recent = [s for s in track.samples if now - s[0] <= 2.0 and (after is None or s[0] > after)]
        if len(recent) < self.min_samples:
            return None
        coords = [s[1] for s in recent[-self.track_stability_window:]]
        center = tuple(sorted(v[i] for v in coords)[len(coords) // 2] for i in range(3))
        inliers = [s for s in recent if math.sqrt(sum((s[1][i] - center[i]) ** 2 for i in range(3))) <= 0.12]
        if len(inliers) < self.min_samples:
            return None
        world = tuple(sorted(s[1][i] for s in inliers)[len(inliers) // 2] for i in range(3))
        confidence = sum(s[2] for s in inliers) / len(inliers)
        return world, confidence, len(inliers)

    def _update_map(self) -> None:
        changed = False
        for kind, tracks in list(self.tracks.items()):
            tracks[:] = [t for t in tracks if time.monotonic() - t.last_seen <= 4.0]
            for track in tracks:
                stable = self._stable_world(track)
                if stable is None:
                    continue
                world, confidence, samples = stable
                # 地图接受阈值仍保持较高；低分候选只有经过更长窗口的
                # 空间稳定后才允许进入地图，避免把单帧误检放大成货位。
                if confidence < self.min_confidence and samples < self.track_stability_window:
                    continue
                entry = self.map.observe(kind=kind, world=world, confidence=confidence, samples=samples, source="yolo_rgbd_geometry")
                if entry is not None:
                    changed = True
        if changed:
            self._save_map()
            self._publish_inventory()

    def _required_ready(self) -> bool:
        return self.task_received and self.map.required_targets_ready([{"kind": t.kind} for t in self.targets])

    def _target_order(self) -> None:
        # 每个 kind 保留完整候选列表；这里只为当前任务预订唯一 slot。
        # 候选选择优先考虑从当前底盘到预抓取线的预计路线，再用置信度/样本数
        # 做稳定排序；重复 kind 任务会从同一候选池中取不同 slot。
        assigned: list[Target] = []
        allowed_shelves = TASK_SHELVES | ({"A"} if (self.scan_only or "A" in self.scan_shelves) else set())
        available = {
            kind: [
                entry
                for entry in self.map.candidates(kind)
                if entry.shelf in allowed_shelves
            ]
            for kind in {t.kind for t in self.targets}
        }
        reserved_slots: set[str] = set()
        for target in self.targets:
            candidates = [
                entry for entry in available.get(target.kind, [])
                if entry.slot not in reserved_slots
            ]
            if not candidates:
                continue
            scored = []
            for entry in candidates:
                target.candidate = entry
                route_seconds = self._estimate_target_route_seconds(target)
                scored.append((
                    route_seconds,
                    -float(entry.confidence),
                    -int(entry.samples),
                    str(entry.slot),
                    entry,
                ))
            scored.sort(key=lambda item: item[:4])
            entry = scored[0][4]
            target.candidate = entry
            self.map.reserve(entry.slot, target.target_id)
            target.reserved = True
            reserved_slots.add(entry.slot)
            assigned.append(target)
            self.get_logger().info(
                f"[exec] 候选分配 kind={target.kind} target={target.target_id} "
                f"selected={entry.slot} alternatives={len(candidates)} "
                f"route_s={scored[0][0]:.1f}"
            )
        if len(assigned) == len(self.targets):
            self.targets = assigned

    @staticmethod
    def _polyline_length(points: list[tuple[float, float]]) -> float:
        return sum(
            math.hypot(points[index + 1][0] - points[index][0], points[index + 1][1] - points[index][1])
            for index in range(len(points) - 1)
        )

    def _estimate_target_route_seconds(self, target: Target) -> float:
        """估算从当前底盘位置到目标货架预抓取线的时间。

        估算与 _goto_observe/_return_route 使用同一条路线模型：
        配送区先走障碍 A* 到入口，再沿货架前无障碍线进入目标列；
        已在货架观察行时只计算横向换排和法向接近。路径不可生成时
        返回无穷大，避免把不可达目标排到前面。
        """
        if target.candidate is None or target.candidate.world is None:
            return math.inf
        target_x = float(target.candidate.world[0] + TARGET_APPROACH_OFFSET_X)
        start_x = float(self.x) if self.x is not None else DELIVERY_GOAL[0]
        start_y = float(self.y) if self.y is not None else DELIVERY_GOAL[1]
        if self.no_obstacles:
            entry = (-0.50, 1.29)
            points = [
                (start_x, start_y),
                (start_x, SHELF_STAGING_Y),
                (entry[0], SHELF_STAGING_Y),
                (target_x, SHELF_STAGING_Y),
                (target_x, OBSERVE_Y),
            ]
            return self._polyline_length(points) / max(self.mapping_max_speed, 0.1)
        if start_y < 0.0:
            entry = (-0.50, 1.29)
            corridor = self._plan_corridor_world((start_x, start_y), entry)
            if corridor is None:
                return math.inf
            points = corridor + [(-0.50, SHELF_STAGING_Y), (target_x, SHELF_STAGING_Y), (target_x, OBSERVE_Y)]
            corridor_length = self._polyline_length(corridor)
            approach_length = self._polyline_length(points[len(corridor) - 1 :])
            return corridor_length / max(self.cruise_max_speed, 0.2) + approach_length / max(self.mapping_max_speed, 0.1)
        points = [(start_x, start_y), (target_x, start_y), (target_x, OBSERVE_Y)]
        return self._polyline_length(points) / max(self.mapping_max_speed, 0.1)

    def _reorder_remaining_targets(self, reason: str) -> None:
        if self.current_index >= len(self.targets):
            return
        prefix = self.targets[: self.current_index]
        remaining = self.targets[self.current_index :]
        scored = [
            (self._estimate_target_route_seconds(target), target)
            for target in remaining
        ]
        scored.sort(
            key=lambda item: (
                item[0],
                item[1].candidate.shelf if item[1].candidate is not None else "?",
                item[1].candidate.level if item[1].candidate is not None else 99,
                item[1].candidate.column if item[1].candidate is not None else 99,
            )
        )
        self.targets = prefix + [target for _, target in scored]
        summary = ", ".join(
            f"{target.target_id}:{target.candidate.slot if target.candidate else '?'}={seconds:.1f}s"
            for seconds, target in scored
        )
        self.get_logger().info(f"[exec] 按预计最短路线重排剩余目标（{reason}）：{summary}")

    def _map_complete(self) -> None:
        if not self.mapping_active or not self.task_received:
            return
        elapsed = time.monotonic() - self.started_at
        if elapsed < self.min_map_seconds:
            return
        if not self._required_ready():
            if elapsed < self.map_timeout:
                return
            missing = [t.target_id for t in self.targets if not self.map.candidates(t.kind)]
            for target in self.targets:
                if not self.map.candidates(target.kind):
                    target.status = "unresolved"
                    target.error = "初始地图超时，未找到可用同类货位"
            self.mapping_active = False
            self.map_completed_at = time.monotonic()
            self._publish_targets("initial_map_timeout")
            self._publish_flow("initial_map_timeout")
            self.done = True
            self.get_logger().error(
                f"初始地图超时：未找到可用目标 {missing}；不启动导航或机械臂动作"
            )
            return
        self.mapping_active = False
        self.map_completed_at = time.monotonic()
        self._target_order()
        if self.fixed_task_order:
            self.get_logger().info(
                "[exec] 显式任务顺序已锁定：跳过初始地图后的最短路线重排"
            )
        else:
            self._reorder_remaining_targets("initial_map")
        self._save_map()
        self._publish_inventory()
        self._publish_flow("initial_map_complete")
        if self.targets:
            self.targets[0].status = "searching"
            self._publish_navigation(self.targets[0])
        self.get_logger().info(
            f"初始45槽位地图阶段结束：observed={self.map.to_dict()['observed_count']}/45，"
            f"任务目标已匹配={len([t for t in self.targets if t.candidate])}/{len(self.targets)}"
        )

    def _publish_navigation(self, target: Target) -> None:
        if target.candidate is None or target.candidate.world is None:
            return
        shelf_x = target.candidate.world[0]
        # 边列商品不能把底盘停在货架中列：右臂在该姿态下的
        # 横向余量不足会直接导致 deploy IK 无解。沿目标列右侧
        # 偏置约 0.12m，保持商品在右臂可达侧；creep 再沿货架
        # 法向接近，不改变商品世界坐标。
        gx = shelf_x + TARGET_APPROACH_OFFSET_X
        gy, gyaw = OBSERVE_Y, math.pi / 2.0
        key = f"{target.target_id}:{gx:.4f}:{gy:.4f}:{gyaw:.4f}"
        now = time.monotonic()
        if key == self.last_nav_key and now - self.last_nav_publish < 1.0:
            return
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = gx, gy
        pose.pose.orientation.z = math.sin(gyaw / 2.0)
        pose.pose.orientation.w = math.cos(gyaw / 2.0)
        self.nav_goal_pub.publish(pose)
        if target.status == "searching":
            target.status = "navigating"
        meta = String(data=json.dumps({"schema_version": 1, "mode": "inventory_coordinator", "target_id": target.target_id, "kind": target.kind, "slot": target.candidate.slot, "object_world": target.candidate.world, "navigation_goal": [gx, gy, gyaw]}, ensure_ascii=False, separators=(",", ":")))
        self.nav_meta_pub.publish(meta)
        self.last_nav_key, self.last_nav_publish = key, now
        self.goal = [gx, gy, gyaw]
        self._publish_targets("navigation_goal_ready")

    def _on_nav_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        state = str(payload.get("state", ""))
        self.nav_state = state
        if self.execute_enabled:
            self._exec_on_nav_status(state, payload)
            return
        target = self.targets[self.current_index] if self.current_index < len(self.targets) else None
        if target is None or target.candidate is None or self.goal is None:
            return
        reported = payload.get("goal")
        match = isinstance(reported, list) and len(reported) >= 2 and math.hypot(float(reported[0]) - self.goal[0], float(reported[1]) - self.goal[1]) <= 0.18
        if state == "reached" and match and target.status in {"searching", "navigating"}:
            target.status = "grasping_plan"
            self._publish_targets("target_navigation_reached")
            self._publish_flow("target_navigation_reached")
            self.get_logger().info(
                f"目标已到达预抓取观察位：target={target.target_id} slot={target.candidate.slot}；"
                "直接进入预抓取路线"
            )

    def _exec_on_nav_status(self, state: str, payload: dict[str, Any]) -> None:
        """EXECUTE 模式：根据到达目标类型切换相位（巡游到排→扫，抓取位→复核，配送位→放置）。"""
        reported = payload.get("goal")
        reached = state == "reached" and self._goal_matches(reported)
        if reached:
            if not self._drive_arrived():
                return  # 本段到达，继续下一段（先避障过渡线再 pickup 贴近货架）
            if self.mapping_active and self.phase == "sweep" and self.mapping_mode == "drive":
                self._begin_scan()
            elif not self.mapping_active and self.phase == "target_nav":
                target = self._current()
                if target is not None:
                    target.status = "grasping_plan"
                    self.nav_retry = 0
                    self._publish_targets("target_navigation_reached")
                    self._publish_flow("target_navigation_reached")
                    self.get_logger().info(
                        f"[exec] 目标导航到位：target={target.target_id} slot={target.candidate.slot if target.candidate else '?'}；"
                        "直接进入预抓取路线"
                    )
                    self._begin_pregrasp_retreat(target)
            elif not self.mapping_active and self.phase == "pregrasp_nav":
                self._on_pregrasp_nav_reached()
            elif not self.mapping_active and self.phase == "deliver":
                self._begin_place()
            else:
                self.get_logger().info(f"[exec] 到达事件 phase={self.phase} mode={self.mapping_mode}（忽略）")
        elif state == "failed":
            reason = str(payload.get("reason", ""))
            self.nav_retry += 1
            if self.nav_retry > NAV_RETRY_MAX:
                self.get_logger().error(
                    f"[exec] 导航连续失败 {self.nav_retry} 次（{reason}），放弃当前目标"
                )
                self.nav_retry = 0
                self._fail_current(f"导航连续失败 {self.nav_retry} 次：{reason}")
                return
            self.get_logger().warning(
                f"[exec] 导航失败 {self.nav_retry}/{NAV_RETRY_MAX}：{reason}；由 nav worker 重启重试"
            )

    def _on_place_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if self.execute_enabled:
            # EXECUTE 模式：place 完成由 place worker 退出码驱动（见 _place_tick），
            # 避免与 plan-only 的事件推进重复计数。
            return
        if str(payload.get("state", "")) not in {"placed", "verified", "reached"}:
            return
        target = self.targets[self.current_index] if self.current_index < len(self.targets) else None
        if target is None or target.candidate is None:
            return
        self.map.mark(target.candidate.slot, "placed", target.target_id)
        target.status = "placed"
        self.current_index += 1
        if self.current_index < len(self.targets):
            self.targets[self.current_index].status = "searching"
            self._publish_navigation(self.targets[self.current_index])
        else:
            self.done = True
        self._save_map()
        self._publish_inventory()
        self._publish_flow("place_verified")

    def _publish_grasp_meta(self, target: Target) -> None:
        if target.candidate is None or target.candidate.world is None:
            return
        payload = {"schema_version": 1, "mode": "inventory_coordinator", "target_id": target.target_id, "kind": target.kind, "slot": target.candidate.slot, "object_world": target.candidate.world, "mechanical_commands_sent": False, "plan_only_allowed": True, "creep_extra_dy": round(float(getattr(self, "_servo_depth_extra", 0.0) or 0.0), 4)}
        self.grasp_meta_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target.candidate.world
        pose.pose.orientation.w = 1.0
        self.grasp_goal_pub.publish(pose)

    def _publish_inventory(self) -> None:
        self.inventory_pub.publish(String(data=json.dumps(self.map.to_dict(), ensure_ascii=False, separators=(",", ":"))))

    def _publish_targets(self, reason: str) -> None:
        current = self.targets[self.current_index] if self.current_index < len(self.targets) else None
        payload = {"schema_version": 1, "mode": "inventory", "reason": reason, "mapping_active": self.mapping_active, "current_index": self.current_index, "count": len(self.targets), "current_target": self._target_dict(current), "targets": [self._target_dict(t) for t in self.targets]}
        self.target_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))

    @staticmethod
    def _target_dict(target: Target | None) -> dict[str, Any] | None:
        if target is None:
            return None
        return {"id": target.target_id, "kind": target.kind, "status": target.status, "error": target.error, "plan_only_allowed": target.status == "grasping_plan", "candidate": ({"slot": target.candidate.slot, "shelf": target.candidate.shelf, "level": target.candidate.level, "column": target.candidate.column, "world": target.candidate.world, "confidence": target.candidate.confidence, "samples": target.candidate.samples} if target.candidate else None)}

    def _publish_flow(self, reason: str) -> None:
        payload = {"schema_version": 1, "reason": reason, "phase": "initial_mapping" if self.mapping_active else "task_execution", "mapping_active": self.mapping_active, "observed_count": self.map.to_dict()["observed_count"], "slot_count": 45, "current_index": self.current_index, "target_count": len(self.targets), "nav_state": self.nav_state}
        self.flow_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))

    # ------------------------------------------------------------------
    # execute 自主运行：底盘驾驶 + 动作 worker 管理（EXECUTE 专用）
    # ------------------------------------------------------------------
    def _current(self) -> Target | None:
        if self.current_index >= len(self.targets):
            return None
        return self.targets[self.current_index]

    def _goal_matches(self, reported: Any) -> bool:
        if not isinstance(reported, list) or len(reported) < 2 or self.goal is None:
            return False
        try:
            return math.hypot(float(reported[0]) - self.goal[0], float(reported[1]) - self.goal[1]) <= 0.18
        except (TypeError, ValueError):
            return False

    def _in_delivery_base(self) -> bool:
        """按官方裁判 S4 判定底盘是否已进入配送区，不要求精确朝向。"""
        return (
            self.x is not None
            and self.y is not None
            and DELIVERY_BASE_X[0] <= self.x <= DELIVERY_BASE_X[1]
            and DELIVERY_BASE_Y[0] <= self.y <= DELIVERY_BASE_Y[1]
        )

    def _publish_goal_msg(self, gx: float, gy: float, gyaw: float) -> None:
        """发布底盘导航目标并记录 self.goal（节流，避免刷屏）。"""
        now = time.monotonic()
        key = f"{gx:.4f}:{gy:.4f}:{gyaw:.4f}"
        if key == self.last_nav_key and now - self.last_nav_publish < 0.8:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(gx)
        pose.pose.position.y = float(gy)
        pose.pose.orientation.z = math.sin(gyaw / 2.0)
        pose.pose.orientation.w = math.cos(gyaw / 2.0)
        self.nav_goal_pub.publish(pose)
        self.last_nav_key = key
        self.last_nav_publish = now
        self.goal = [float(gx), float(gy), float(gyaw)]

    def _reset_navigation_state(self, *, reason: str = "") -> None:
        """停止并清空当前导航状态，避免旧目标事件污染下一目标。"""
        if self.nav_worker is not None:
            self._stop_worker_process(self.nav_worker)
        self.nav_worker = None
        self.drive_goals.clear()
        self.goal = None
        self.last_nav_key = ""
        self.last_nav_publish = 0.0
        self.nav_state = "idle"
        self._drive_pickup = False
        self._drive_pickup_transit = False
        self.return_route_active = False
        if reason:
            self.get_logger().info(f"[exec] 已清理导航状态：{reason}")

    def _nav_done(self) -> bool:
        return self.nav_worker is not None and self.nav_worker.poll() is not None

    def _arm_done(self) -> bool:
        return self.arm_worker is not None and self.arm_worker.poll() is not None

    def _spawn_nav(self, speed: float, pickup: bool = False, pickup_transit: bool = False) -> None:
        """启动一个订阅 /competition/navigation_goal 的底盘 worker。

        pickup=True：货架前安全接近位（忽略雷达，货架不是障碍），用于逐排扫描与
        抓取接近；pickup=False：反应式避障模式，用于配送/返程穿越随机障碍走廊。
        """
        gain = 0.6 if speed >= 1.0 else (0.5 if speed >= 0.3 else 0.4)
        planned_delivery = self.phase == "deliver" or self.return_route_active
        # 配送段的航点已由同种子障碍布局 A* 生成；降低局部绕行阈值，
        # 避免雷达的保守安全航向把底盘从可行航线拉成小圈。仍保留
        # 正面近障停车，只放宽到与 A* 的机器人膨胀半径一致。
        nav_obstacle_distance = min(self.obstacle_distance, 0.35) if planned_delivery else self.obstacle_distance
        nav_avoid_clearance = 0.30 if planned_delivery else 0.55
        nav_goal_tolerance = (
            0.18
            if self.phase == "deliver"
            # return_route_active（place 后返回货架）的终点就是下一目标的
            # 预抓取线，必须用严格容差——此前它误用 0.18 宽容差，随后又
            # 要过 0.05 的 pregrasp odom 门禁，0.09m 的停车误差必然被拒。
            else (
                PREGRASP_GOAL_TOLERANCE
                if self.phase in {"target_nav", "pregrasp_nav"} or self.return_route_active
                else 0.12
            )
        )
        script = str(self.baseline_dir / "low_speed_navigator.py")
        command = [
            sys.executable, script,
            "--goal-topic", NAV_GOAL_TOPIC,
            "--max-speed", str(speed),
            "--max-angular", str(DELIVERY_MAX_ANGULAR if planned_delivery else NAV_MAX_ANGULAR),
            "--obstacle-distance", str(nav_obstacle_distance),
            "--avoid-clearance", str(nav_avoid_clearance),
            "--speed-gain", str(gain),
            "--goal-tolerance", str(nav_goal_tolerance),
            # 抓取相关相位的导航朝向容差必须与 pregrasp odom 门禁一致
            # （PREGRASP_ODOM_YAW_TOLERANCE）：导航器只对齐到自己容差内，
            # 门禁更严就会出现 0.097 vs 0.06 这种"永远过不去"的重试死循环
            # （实测整单因此夭折）。
            "--yaw-tolerance", str(
                PREGRASP_ODOM_YAW_TOLERANCE
                if self.phase in {"target_nav", "pregrasp_nav"} or self.return_route_active
                else 0.10
            ),
            "--pickup-lateral-tolerance", "0.08",
            "--timeout", "240",
        ]
        if pickup:
            command.append("--pickup-approach")
            if self.phase in {"target_nav", "pregrasp_nav"}:
                # 目标专属观察位和机械臂预抓取线都必须严格到位；
                # 只有首轮扫描保留货架安全距离提前结束语义。
                command.append("--pickup-exact")
        if pickup_transit:
            command.append("--pickup-transit")
        # 配送段一律用 A* 预规划路径 + 保留雷达避障。
        # 此前 no_obstacles 时会加 --ignore-obstacles 完全关掉雷达，
        # 结果连场地上的静态方块都不避、直接撞上去（主人实测）。
        # 静态障碍（方块/货架）与动态障碍不能混为一谈。
        if planned_delivery:
            command.append("--planned-route")
        self.get_logger().info(
            f"[exec] spawn nav worker: speed={speed} pickup={pickup} pickup_transit={pickup_transit} "
            f"planned_delivery={planned_delivery} no_obstacles={self.no_obstacles} "
            f"goal_tolerance={nav_goal_tolerance:.2f} "
            f"obstacle_distance={nav_obstacle_distance:.2f} "
            f"avoid_clearance={nav_avoid_clearance:.2f} {' '.join(command[-12:])}"
        )
        self.nav_worker = subprocess.Popen(command)

    def _ensure_nav(self, speed: float, pickup: bool = False, pickup_transit: bool = False) -> None:
        if self._nav_done():
            code = self.nav_worker.poll()
            if code:
                self.get_logger().warning(f"[exec] nav worker 退出码 {code}；重启")
            self.nav_worker = None
        if self.nav_worker is None:
            self._spawn_nav(speed, pickup, pickup_transit)

    def _read_servo_result(self) -> tuple[tuple[float, float, float], bool]:
        """读手眼伺服结果文件；缺文件/超时（>60s 旧）视为未测量。"""
        path = self.baseline_dir / SERVO_RESULT_PATH
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return (0.0, 0.0, 0.0), False
        if time.time() - float(payload.get("wall_time", 0)) > 60.0:
            return (0.0, 0.0, 0.0), False
        correction = payload.get("correction")
        if not isinstance(correction, list) or len(correction) != 3:
            return (0.0, 0.0, 0.0), False
        measured = bool(payload.get("measured", False))
        try:
            self._servo_standoff = float(payload.get("standoff_m", 0.0))
        except (TypeError, ValueError):
            self._servo_standoff = 0.0
        return (float(correction[0]), float(correction[1]), float(correction[2])), measured

    def _spawn_arm(self, action: str) -> None:
        script_name, fixed_args = WORKER_CFG[action]
        script = str(self.baseline_dir / script_name)
        command = [sys.executable, script] + list(fixed_args)
        self.get_logger().info(f"[exec] spawn arm worker: {' '.join(command)}")
        # 手眼渲染阶段开关：只在 servo worker 运行期间开（伺服只需几帧）。
        # creep→retreat 全程常开会让重部署期间 RTF 掉到 0.22，deploy 的
        # 关节收敛（仿真时间驱动）90s 墙钟不够用 → 硬超时（实测）。
        flag = self.baseline_dir / "debug_data" / "handeye_render.flag"
        try:
            if action == "servo":
                flag.parent.mkdir(parents=True, exist_ok=True)
                flag.write_text("servo", encoding="utf-8")
            elif action in ("backoff", "backoff_far", "close_gripper", "retreat", "slide_up"):
                flag.unlink(missing_ok=True)
        except OSError:
            pass
        self.arm_worker = subprocess.Popen(command)
        self.arm_worker_started_at = time.monotonic()
        self.arm_worker_action = action

    def _arm_worker_hard_timeout(self) -> str | None:
        """worker 挂死（自身超时失效）时强杀；返回被强杀的动作名，否则 None。"""
        if self.arm_worker is None:
            return None
        started = getattr(self, "arm_worker_started_at", None)
        if started is None:
            return None
        elapsed = time.monotonic() - started
        if elapsed <= ARM_WORKER_HARD_TIMEOUT:
            return None
        action = getattr(self, "arm_worker_action", "unknown")
        self.get_logger().error(
            f"[exec] arm worker [{action}] 挂死 {elapsed:.0f}s > 硬超时 "
            f"{ARM_WORKER_HARD_TIMEOUT:.0f}s，强制终止并继续流程"
        )
        self._stop_worker_process(self.arm_worker)
        self.arm_worker = None
        return action

    def _stop_worker_process(self, process: subprocess.Popen[Any] | None) -> None:
        if process is None:
            return
        try:
            process.terminate()
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()

    def _head(self, pitch: float) -> None:
        message = Float64MultiArray(data=[0.0, float(pitch)])
        self.head_pub.publish(message)

    def _begin_sweep(self) -> None:
        self.phase = "sweep"
        self.sweep_shelf_index = 0
        self.mapping_mode = "drive"
        self.mapping_active = True
        self._sweep_drive_next()

    def _drive_next_goal(self) -> None:
        if not self.drive_goals:
            return
        # 导航器到达目标后会先发布 reached，再退出进程；下一段不能复用
        # 仍处于 finished 状态的旧 worker，否则新目标会无人执行。
        if self.nav_worker is not None:
            self._stop_worker_process(self.nav_worker)
            self.nav_worker = None
        gx, gy, gyaw, pickup, speed, pickup_transit = self.drive_goals.pop(0)
        self._drive_pickup = pickup
        self._drive_pickup_transit = pickup_transit
        self._publish_goal_msg(gx, gy, gyaw)
        self._ensure_nav(speed, pickup=pickup, pickup_transit=pickup_transit)
        self.get_logger().info(
            f"[exec] drive 段 → ({gx:.3f},{gy:.3f},yaw={gyaw:.2f}) "
            f"pickup={pickup} pickup_transit={pickup_transit} speed={speed}"
        )

    def _drive_arrived(self) -> bool:
        """一段到达：还有后续段就继续推进；全部到达返回 True。"""
        if self.drive_goals:
            self._drive_next_goal()
            return False
        return True

    def _load_obstacle_layout(self):
        """加载官方障碍生成器，确保 Client 与 Server 复现同一布局。"""
        candidates = [
            Path("/workspace/baseline/official_baseline/examples/supermarket_sorting/obstacle_layout.py"),
            self.baseline_dir / "official_baseline/examples/supermarket_sorting/obstacle_layout.py",
        ]
        for path in candidates:
            if not path.exists():
                continue
            try:
                import importlib.util
                spec = importlib.util.spec_from_file_location("official_obstacle_layout_runtime", str(path))
                if spec is None or spec.loader is None:
                    continue
                module = importlib.util.module_from_spec(spec)
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)
                return module
            except Exception as exc:
                self.get_logger().warning(f"加载官方障碍布局失败 path={path}: {type(exc).__name__}: {exc}")
        return None

    def _obstacle_seed(self) -> int | None:
        raw = os.getenv("SUPERMARKET_OBSTACLE_SEED", "").strip()
        if raw:
            try:
                return int(raw)
            except ValueError:
                self.get_logger().warning(f"忽略非法 SUPERMARKET_OBSTACLE_SEED={raw!r}")
        raw = os.getenv("SUPERMARKET_SEED", "").strip()
        if raw:
            try:
                return int(raw) + 1000003
            except ValueError:
                self.get_logger().warning(f"忽略非法 SUPERMARKET_SEED={raw!r}")
        return None

    @staticmethod
    def _route_points_to_goals(
        points: list[tuple[float, float]],
        speeds: list[float],
        *,
        pickup_last: bool = False,
        final_yaw: float | None = None,
    ) -> list[tuple[float, float, float, bool, float, bool]]:
        goals: list[tuple[float, float, float, bool, float, bool]] = []
        for index, (x, y) in enumerate(points):
            if index + 1 < len(points):
                nx, ny = points[index + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = float(final_yaw) if final_yaw is not None else -math.pi / 2.0
            goals.append((x, y, yaw, pickup_last and index == len(points) - 1, speeds[min(index, len(speeds) - 1)], False))
        return goals

    def _plan_corridor_world(self, start_world: tuple[float, float], goal_world: tuple[float, float]) -> list[tuple[float, float]] | None:
        """在官方走廊局部坐标中做机器人尺寸膨胀后的 A*，返回世界坐标航点。"""
        layout = self._load_obstacle_layout()
        seed = self._obstacle_seed()
        if layout is None or seed is None:
            self.get_logger().warning("无法复现官方障碍布局，动态 A* 不可用")
            return None
        try:
            generated = layout.generate_obstacle_layout(seed)
            selected = tuple(
                (float(position[0]), float(position[1]), float(generated.yaws[body]))
                for body, position in generated.positions.items()
            )
            origin_x, origin_y = -0.96, -1.01
            start = (start_world[0] - origin_x, start_world[1] - origin_y)
            goal = (goal_world[0] - origin_x, goal_world[1] - origin_y)
            radius = float(layout.ROBOT_CLEARANCE_RADIUS)
            resolution = float(layout.GRID_RESOLUTION)
            x_min = float(layout.CORRIDOR_X_MIN) + radius
            x_max = float(layout.CORRIDOR_X_MAX) - radius
            y_min = float(layout.CORRIDOR_Y_MIN) + radius
            y_max = float(layout.CORRIDOR_Y_MAX) - radius
            nx = round((x_max - x_min) / resolution) + 1
            ny = round((y_max - y_min) / resolution) + 1

            def to_cell(point: tuple[float, float]) -> tuple[int, int]:
                return (round((point[0] - x_min) / resolution), round((point[1] - y_min) / resolution))

            def to_point(cell: tuple[int, int]) -> tuple[float, float]:
                return (x_min + cell[0] * resolution, y_min + cell[1] * resolution)

            def in_bounds(cell: tuple[int, int]) -> bool:
                return 0 <= cell[0] < nx and 0 <= cell[1] < ny

            def blocked(cell: tuple[int, int]) -> bool:
                if not in_bounds(cell):
                    return True
                x, y = to_point(cell)
                for obstacle_x, obstacle_y, yaw in selected:
                    half_x, half_y = layout._oriented_half_extents(yaw)
                    if abs(x - obstacle_x) <= half_x + radius and abs(y - obstacle_y) <= half_y + radius:
                        return True
                return False

            def nearest_free(cell: tuple[int, int]) -> tuple[int, int] | None:
                if in_bounds(cell) and not blocked(cell):
                    return cell
                for distance in range(1, 20):
                    for dx in range(-distance, distance + 1):
                        for dy in (-distance, distance):
                            candidate = (cell[0] + dx, cell[1] + dy)
                            if in_bounds(candidate) and not blocked(candidate):
                                return candidate
                    for dy in range(-distance + 1, distance):
                        for dx in (-distance, distance):
                            candidate = (cell[0] + dx, cell[1] + dy)
                            if in_bounds(candidate) and not blocked(candidate):
                                return candidate
                return None

            start_cell = nearest_free(to_cell(start))
            goal_cell = nearest_free(to_cell(goal))
            if start_cell is None or goal_cell is None:
                return None
            moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))
            distance = {start_cell: 0.0}
            parent: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
            queue = [(0.0, start_cell)]
            while queue:
                _, cell = heapq.heappop(queue)
                if cell == goal_cell:
                    break
                for dx, dy, cost in moves:
                    neighbor = (cell[0] + dx, cell[1] + dy)
                    if blocked(neighbor):
                        continue
                    if dx and dy and (blocked((cell[0] + dx, cell[1])) or blocked((cell[0], cell[1] + dy))):
                        continue
                    candidate_distance = distance[cell] + cost * resolution
                    if candidate_distance >= distance.get(neighbor, math.inf):
                        continue
                    distance[neighbor] = candidate_distance
                    parent[neighbor] = cell
                    heuristic = math.dist(neighbor, goal_cell) * resolution
                    heapq.heappush(queue, (candidate_distance + heuristic, neighbor))
            if goal_cell not in parent:
                return None
            cells = []
            cursor: tuple[int, int] | None = goal_cell
            while cursor is not None:
                cells.append(cursor)
                cursor = parent[cursor]
            cells.reverse()

            def line_clear(a: tuple[int, int], b: tuple[int, int]) -> bool:
                steps = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 1
                for index in range(steps + 1):
                    ratio = index / steps
                    cell = (round(a[0] + (b[0] - a[0]) * ratio), round(a[1] + (b[1] - a[1]) * ratio))
                    if blocked(cell):
                        return False
                return True

            simplified = [cells[0]]
            anchor = 0
            while anchor < len(cells) - 1:
                farthest = anchor + 1
                for candidate in range(anchor + 1, len(cells)):
                    if line_clear(cells[anchor], cells[candidate]):
                        farthest = candidate
                simplified.append(cells[farthest])
                anchor = farthest
            points = [to_point(cell) for cell in simplified]
            points[0] = start
            points[-1] = goal
            return [(x + origin_x, y + origin_y) for x, y in points]
        except Exception as exc:
            self.get_logger().warning(f"动态 A* 规划失败: {type(exc).__name__}: {exc}")
            return None

    def _delivery_route(self) -> list[tuple[float, float, float, bool, float, bool]]:
        """生成货架到配送台路线；无障碍单跑不复现随机障碍。"""
        entry = (-0.50, 1.29)
        if self.no_obstacles:
            current = (
                float(self.x) if self.x is not None else entry[0],
                float(self.y) if self.y is not None else OBSERVE_Y,
            )
            # 货架前沿 → 走廊入口 → 配送台，两段直达。原中间两个 2.10
            # 航点会造成"掉头 → 横移掉头 → 再掉回来"约 360° 原地转向；
            # 无障碍模式下两点连线之间没有场景家具，可安全斜切。
            points = [current, entry, DELIVERY_GOAL[:2]]
            # 全程载货限速：翻车发生在载货段（实测倾角 47.1°），第一段不例外。
            speeds = [min(self.cruise_max_speed, DELIVERY_SPEED)] * len(points)
            goals = self._route_points_to_goals(points, speeds)
            self.get_logger().info(f"[exec] 无障碍直达配送路线：waypoints={len(goals)}")
            return goals
        corridor = self._plan_corridor_world(entry, DELIVERY_GOAL)
        if corridor is None:
            self.get_logger().error("动态配送路线生成失败，不使用可能撞障碍的固定航点")
            return []
        points = corridor
        # 首段（货架前沿→走廊入口）用载货限速，不再满速：满速载货正是
        # 翻车敏感场景（实测倾角 47.1°）。走廊内维持 0.30 慢速绕障。
        speeds = [min(self.cruise_max_speed, DELIVERY_SPEED, 0.30)] * len(points)
        if speeds:
            speeds[0] = min(self.cruise_max_speed, DELIVERY_SPEED)
        goals = self._route_points_to_goals(points, speeds)
        self.get_logger().info(f"[exec] 动态配送路线：seed={self._obstacle_seed()} waypoints={len(goals)}")
        return goals

    def _return_route(self, target_x: float, observe_y: float, speed: float) -> list[tuple[float, float, float, bool, float, bool]]:
        """按同一障碍布局 A* 反向脱离走廊，再回到目标货架观察线。"""
        self.return_route_active = True
        entry = (-0.50, 1.29)
        current = (
            float(self.x) if self.x is not None else DELIVERY_GOAL[0],
            float(self.y) if self.y is not None else DELIVERY_GOAL[1],
        )
        if self.no_obstacles:
            points = [
                current,
                entry,
                (-0.50, SHELF_STAGING_Y),
                (target_x, SHELF_STAGING_Y),
                (target_x, OBSERVE_Y),
            ]
            speeds = [self.cruise_max_speed, self.cruise_max_speed, speed, speed, speed]
            goals = self._route_points_to_goals(points, speeds, final_yaw=math.pi / 2.0)
            if len(goals) >= 2:
                # pickup-approach 沿 goal_yaw 前进；返程最后两个航点
                # 必须使用各自的入站方向，不能沿下一段的出站方向。
                yaw_second_last = math.atan2(
                    points[-2][1] - points[-3][1],
                    points[-2][0] - points[-3][0],
                )
                yaw_last = math.atan2(
                    points[-1][1] - points[-2][1],
                    points[-1][0] - points[-2][0],
                )
                # 倒数第二段只是货架前横向换列，必须使用普通目标导航；
                # 只有最后一段才进入 pickup-approach。
                goals[-2] = (goals[-2][0], goals[-2][1], yaw_second_last, False, speed, False)
                goals[-1] = (goals[-1][0], goals[-1][1], yaw_last, True, speed, False)
            self.get_logger().info(f"[exec] 无障碍官方黄线返程路线：waypoints={len(goals)}")
            return goals
        corridor = self._plan_corridor_world(current, entry)
        if corridor is None:
            self.get_logger().error("动态返程路线生成失败，不继续驱动底盘")
            return []
        points = corridor + [(-0.50, SHELF_STAGING_Y), (target_x, SHELF_STAGING_Y), (target_x, observe_y)]
        speeds = [min(self.cruise_max_speed, 0.30)] * len(points)
        speeds[-3:] = [self.cruise_max_speed, speed, speed]
        goals = self._route_points_to_goals(points, speeds, final_yaw=math.pi / 2.0)
        # 返回先回官方黄线，再沿货架前方横向换列；最后两段是已知
        # 无障碍的取货接近线，避免在中间挡板附近斜切和原地转向。
        if len(goals) >= 2:
            # pickup-approach 沿 goal_yaw 前进；返程最后两个航点
            # 必须使用各自的入站方向，不能沿下一段的出站方向。
            yaw_second_last = math.atan2(
                points[-2][1] - points[-3][1],
                points[-2][0] - points[-3][0],
            )
            yaw_last = math.atan2(
                points[-1][1] - points[-2][1],
                points[-1][0] - points[-2][0],
            )
            # 倒数第二段只是货架前横向换列，必须使用普通目标导航；
            # 只有最后一段才进入 pickup-approach。
            goals[-2] = (goals[-2][0], goals[-2][1], yaw_second_last, False, speed, False)
            goals[-1] = (goals[-1][0], goals[-1][1], yaw_last, True, speed, False)
        self.get_logger().info(f"[exec] 动态返程路线：seed={self._obstacle_seed()} waypoints={len(goals)}")
        return goals

    def _goto_observe(self, x: float, observe_y: float, speed: float) -> None:
        """导航到货架前观察/预抓取位。

        首次从出发区去最右侧 E 货架时，先沿固定 +Y 直线进入过渡点，
        不启用配送阶段的激光避障；最后仍用 pickup-transit 沿 +Y 到观察线，
        到位后只做一次朝向对齐。已在货架观察行时采用“沿 X 直行→回正”的路线。"""
        if self.observe_row:
            # MMK2 是差速底盘，不支持 linear.y 横移。扫描阶段只保留
            # “必要时转一次横向行驶方向 → 沿观察线直行 → 到货架前回正一次”
            # 两类航点，不再先发布同位置的空航点，因此不会在每个货架前
            # 重复转两三圈。所有最终观察点统一朝向货架 +Y。
            if self.y is not None and self.y < 0.0:
                # 从配送区返回时仍必须先走官方走廊反向路线；回到观察行后，
                # 后续 E/D/B/C/A 扫描统一走下面的连续观察线逻辑。
                self.drive_goals = self._return_route(x, observe_y, speed)
            else:
                start_x = float(self.x if self.x is not None else SLOT_X["E"][1])
                start_y = float(self.y if self.y is not None else observe_y)
                observe_yaw = math.pi / 2.0
                goals: list[tuple[float, float, float, bool, float, bool]] = []
                if abs(start_y - observe_y) > 0.08:
                    approach_yaw = observe_yaw if observe_y > start_y else -math.pi / 2.0
                    # 先用 pickup-transit 完成 Y 轴到观察线，不能把这段
                    # 当作货架接近，否则底盘会套用货架安全距离语义。
                    goals.append((start_x, observe_y, approach_yaw, False, speed, True))
                    start_y = observe_y
                if abs(x - start_x) > 0.08:
                    side_yaw = math.pi if x < start_x else 0.0
                    # 目标导航的横向对齐必须是严格轴向直行；此前这里
                    # pickup=True 会在越过目标列后继续沿货架边界行驶。
                    goals.append((x, observe_y, side_yaw, False, speed, True))
                # 扫描/目标导航的观察点只负责到位和朝向回正，不能启用
                # pickup-approach；否则会把观察线当成取货接近段继续推进。
                goals.append((x, observe_y, observe_yaw, False, speed, True))
                self.drive_goals = goals
        else:
            # The formal random-obstacle layout places a physical corridor
            # boundary across the direct start-to-shelf line. The old route
            # drove at x~=1.9 and stopped at the south board while pickup-
            # transit intentionally ignored LaserScan. Use the clear east-side
            # bypass first, then enter the shelf row and the pickup line.
            bypass_x = float(os.getenv("SUPERMARKET_FORMAL_PICKUP_BYPASS_X", "1.70"))
            start_x = float(self.x if self.x is not None else 1.92)
            start_y = float(self.y if self.y is not None else -3.17)
            lateral_to_bypass_yaw = 0.0 if bypass_x >= start_x else math.pi
            lateral_to_shelf_yaw = math.pi if x <= bypass_x else 0.0
            self.drive_goals = [
                # 首次进入 E 货架的四段都按确定的轴向运动执行：
                # 横向绕行 → +Y 进入观察区 → 横向到货架列 → +Y 到观察线。
                # 最后一段不能使用 pickup-approach，否则会把扫描路线
                # 当成目标取货接近段，出现反向跑离观察线的问题。
                (bypass_x, start_y, lateral_to_bypass_yaw, False, speed, True),
                (bypass_x, 1.90, math.pi / 2.0, False, speed, True),
                (x, 1.90, lateral_to_shelf_yaw, False, speed, True),
                (x, observe_y, math.pi / 2.0, False, speed, True),
            ]
        self._drive_next_goal()

    def _sweep_drive_next(self) -> None:
        if self.sweep_shelf_index >= len(self.scan_shelves):
            if "A" not in self.scan_shelves and not self._required_ready():
                missing_kinds = sorted({
                    target.kind
                    for target in self.targets
                    if not self.map.candidates(target.kind)
                })
                self.scan_shelves = self.scan_shelves + ("A",)
                missing_text = ",".join(missing_kinds) if missing_kinds else "任务候选数量不足"
                self.get_logger().warning(
                    f"E→D→C→B 扫描后仍缺少目标候选（{missing_text}），只补扫 A 一次"
                )
                self.mapping_mode = "drive"
                self._sweep_drive_next()
                return
            self.mapping_mode = "wait"
            self.get_logger().warning(
                f"扫描完成（货架={','.join(self.scan_shelves)}）；停车继续收帧，由 map_timeout 兜底收尾"
            )
            return
        shelf = self.scan_shelves[self.sweep_shelf_index]
        x = float(SLOT_X[shelf][1])  # 中间列 x
        self.mapping_mode = "drive"
        self.observe_row = self.sweep_shelf_index > 0  # 首个货架需从走廊穿越进入
        self._goto_observe(x, OBSERVE_Y, self.mapping_max_speed)
        self.get_logger().info(
            f"[exec] sweep {self.sweep_shelf_index + 1}/{len(self.scan_shelves)} → 货架 {shelf} 观察位 "
            f"({x:.3f},{OBSERVE_Y},+Y)"
        )

    def _begin_scan(self) -> None:
        self.mapping_mode = "scan"
        self.pitch_index = 0
        now = time.monotonic()
        self.pitch_started_at = now
        self.scan_started_at = now
        self._head(PITCH_SCAN[0][0])
        self._publish_flow("shelf_reached_scanning")

    def _advance_scan(self) -> None:
        now = time.monotonic()
        if self.pitch_started_at is None:
            self._begin_scan()
            return
        if now - self.pitch_started_at < PITCH_DWELL_S:
            return
        self.pitch_index += 1
        if self.pitch_index >= len(PITCH_SCAN):
            self._shelf_scanned()
            return
        self.pitch_started_at = now
        self._head(PITCH_SCAN[self.pitch_index][0])

    def _shelf_scanned(self) -> None:
        shelf = self.scan_shelves[self.sweep_shelf_index] if self.sweep_shelf_index < len(self.scan_shelves) else "?"
        # 完成当前货架三层扫描后进入下一排；A 只由 _sweep_drive_next
        # 在 E/D/C/B 后发现任务候选缺失时追加一次。
        self.get_logger().info(f"[exec] 货架 {shelf} 扫描完成，继续建立库存地图")
        self.sweep_shelf_index += 1
        self.mapping_mode = "drive"
        self._sweep_drive_next()

    def _preload_map_from_layout(self) -> None:
        """按布局真值预填目标货位并直接进入执行阶段（跳过巡游扫描）。

        比赛时限 420s，全扫描 5-8 分钟太贵。任务 kind 已知 + 布局固定，
        等价信息直接从 45 槽位真值表得到。
        """
        if not self.slot_truth:
            self.get_logger().error("[exec] 跳扫描失败：货位真值表为空，回退巡游扫描")
            return
        wanted = {t.kind for t in self.targets}
        filled = 0
        for slot in self.slot_truth:
            if slot["kind"] not in wanted:
                continue
            entry = self.map.observe(
                kind=slot["kind"],
                world=slot["world"],
                confidence=1.0,
                samples=self.min_samples + 5,
                source="layout_truth",
            )
            if entry is not None:
                filled += 1
        self.get_logger().warning(
            f"[exec] 跳扫描：按布局真值直接定位 {filled} 个目标货位（{sorted(wanted)}），"
            "跳过巡游扫描立即进入执行"
        )
        self._enter_execution()

    def _enter_execution(self) -> None:
        self.mapping_active = False
        self.map_completed_at = time.monotonic()
        self._target_order()
        self._save_map()
        self._publish_inventory()
        self._publish_flow("initial_map_complete")
        if not self.targets:
            self.get_logger().warning("[exec] 无可用目标槽位，结束")
            self._finish_exec(False)
            return
        self._start_current_target()

    def _start_current_target(self) -> None:
        # 新目标开始前清理上一目标的 worker、旧 goal 和剩余分段航点；
        # 这样迟到的 reached/failed 消息不会被当成当前目标事件。
        self._reset_navigation_state(reason="切换到新目标")
        # 重试配额按目标重置（此前跨目标累计，前一目标的失败会吃掉
        # 后面目标的重试额度）
        self.creep_retry = 0
        self.servo_retry = 0
        self.depth_probe_count = 0
        target = self._current()
        if target is None:
            self._finish_exec(True)
            return
        # 初始地图未找到该任务类别时，禁止把空候选送入导航；
        # 直接记录失败并切换下一个任务，避免目标失败后永久空转。
        if target.candidate is None or target.candidate.world is None:
            self._fail_current("初始地图未找到可用目标货位")
            return
        target.status = "searching"
        self.pregrasp_odom_gate_failures = 0
        self.phase = "target_nav"
        # 目标已从初始地图锁定后，不再走首段 pickup-transit。此时机器人
        # 已在货架扫描区域，必须按差速底盘的换排路线：转向 → 沿 X 轴
        # 移动 → 回到货架朝向并进入预抓取线。
        self.observe_row = True
        self._drive_pickup_transit = False
        shelf = target.candidate.shelf if target.candidate is not None else None
        # 目标列坐标 + 安全横向余量，而不是固定使用货架中列。
        # 这样 D/E/A 等边列目标仍保持在右臂可达工作空间内。
        x = float(target.candidate.world[0] + TARGET_APPROACH_OFFSET_X)
        self._goto_observe(x, OBSERVE_Y, self.mapping_max_speed)
        self._publish_flow("target_navigation_started")

    def _begin_pregrasp_retreat(self, target: Target) -> None:
        """把底盘调整到预抓取线（必要时换列/退出/回正），再做 IK 门禁。

        安全线本身是 creep/deploy 标定的基准位，不能省；但只有真的偏离
        才发航点，不再执行固定的"掉头-后退-回正"三段舞步。"""
        if target.candidate is None or target.candidate.world is None:
            self._fail_current("预抓取阶段缺少目标坐标")
            return
        if self.nav_worker is not None:
            self._stop_worker_process(self.nav_worker)
            self.nav_worker = None
        self.phase = "pregrasp_nav"
        target.status = "pregrasp_navigating"
        self.pregrasp_stable_started_at = None
        self.pregrasp_stable_samples = 0
        self.pregrasp_last_pose = None
        goal_x = float(
            self.goal[0]
            if self.goal is not None
            else target.candidate.world[0] + TARGET_APPROACH_OFFSET_X
        )
        current_x = float(self.x) if self.x is not None else goal_x
        current_y = float(self.y) if self.y is not None else OBSERVE_Y
        side_yaw = 0.0 if goal_x >= current_x else math.pi
        pregrasp_y = ARM_PREGRASP_Y + PREGRASP_GOAL_TOLERANCE
        dx = current_x - goal_x
        dy = current_y - pregrasp_y
        goals: list[tuple[float, float, float, bool, float, bool]] = []
        if abs(dx) > PREGRASP_GOAL_TOLERANCE * 0.8:
            # 横向未对齐才需要沿货架前沿换列；pickup-approach 锁定法向
            # 行驶时无法修正 0.3m 级横向偏差。
            goals.append((goal_x, current_y, side_yaw, True, self.mapping_max_speed, False))
        if dy > PREGRASP_GOAL_TOLERANCE * 0.8:
            # 过冲贴近货架：pickup-approach 只能沿朝向前进，后退必须先掉头，
            # 退出后再回正到预抓取线。
            goals.append((goal_x, ARM_PREGRASP_Y, -math.pi / 2.0, True, self.mapping_max_speed, False))
            goals.append((goal_x, pregrasp_y, math.pi / 2.0, True, self.mapping_max_speed, False))
        elif goals or math.hypot(dx, dy) > PREGRASP_GOAL_TOLERANCE * 0.8:
            # 横向换列后车头停在 ±X，或纵向差半步：补一个回正航点把
            # 车头转回 +Y 并贴到线上，无需"后退-掉头"。
            goals.append((goal_x, pregrasp_y, math.pi / 2.0, True, self.mapping_max_speed, False))
        if not goals:
            yaw_now = self.yaw if self.yaw is not None else math.pi / 2.0
            yaw_err = abs(math.atan2(
                math.sin(yaw_now - math.pi / 2.0), math.cos(yaw_now - math.pi / 2.0)
            ))
            if yaw_err > PREGRASP_ODOM_YAW_TOLERANCE * 0.8:
                # 位置已到位但朝向超门禁：一个回正航点原地转头。
                goals.append((goal_x, pregrasp_y, math.pi / 2.0, True, self.mapping_max_speed, False))
        # 此前固定三航点在已到位时退化为"掉头 180°→退 5cm→再掉头 180°→
        # 进 5cm"的零位移舞步，每个目标白耗 10 秒以上。现在只有真的需要
        # 调整才发航点；位置/朝向交给 pregrasp odom 门禁复核，门禁失败
        # 仍会回到本函数执行完整调整。
        if goals:
            self.drive_goals = goals
            self._drive_next_goal()
            self._publish_flow("pregrasp_retreat_started")
            self.get_logger().info(
                f"[exec] 预抓取调整 {len(goals)} 段 → ({goal_x:.3f},{pregrasp_y:.3f})"
            )
        else:
            self.goal = [goal_x, pregrasp_y, math.pi / 2.0]
            self._publish_flow("pregrasp_retreat_skipped")
            self.get_logger().info("[exec] 底盘已在预抓取线，跳过调整直接进入 odom 门禁")
            self._on_pregrasp_nav_reached()

    def _on_pregrasp_nav_reached(self) -> None:
        target = self._current()
        if target is None:
            return
        if self.nav_worker is not None:
            self._stop_worker_process(self.nav_worker)
            self.nav_worker = None
        # worker 的 reached 只代表导航器自己的判定；进入 IK 前必须用 odom
        # 再校验真实底盘，避免实际位置偏差直接传入 KDL。
        goal = self.goal
        if goal is None or self.x is None or self.y is None or self.yaw is None:
            self.pregrasp_odom_gate_failures += 1
            self._publish_targets("pregrasp_odom_gate_failed")
            self._publish_flow("pregrasp_odom_gate_failed")
            if self.pregrasp_odom_gate_failures > PREGRASP_ODOM_GATE_MAX_RETRIES:
                self._fail_current("预抓取 odom 门禁缺少有效状态")
                return
            self.get_logger().warning("[exec] pregrasp reached 但 odom 无效，重新导航")
            self._begin_pregrasp_retreat(target)
            return
        position_error = math.hypot(float(self.x) - float(goal[0]), float(self.y) - float(goal[1]))
        yaw_error = abs(math.atan2(
            math.sin(float(self.yaw) - float(goal[2])),
            math.cos(float(self.yaw) - float(goal[2])),
        ))
        if position_error > PREGRASP_GOAL_TOLERANCE or yaw_error > PREGRASP_ODOM_YAW_TOLERANCE:
            self.pregrasp_odom_gate_failures += 1
            self._publish_targets("pregrasp_odom_gate_failed")
            self._publish_flow("pregrasp_odom_gate_failed")
            self.get_logger().warning(
                f"[exec] pregrasp odom 门禁失败：pos_error={position_error:.3f}m "
                f"yaw_error={yaw_error:.3f}rad，重试={self.pregrasp_odom_gate_failures}/{PREGRASP_ODOM_GATE_MAX_RETRIES}"
            )
            if self.pregrasp_odom_gate_failures > PREGRASP_ODOM_GATE_MAX_RETRIES:
                self._fail_current("预抓取 odom 未达到目标位置，禁止进入 IK")
                return
            self._begin_pregrasp_retreat(target)
            return
        self.pregrasp_odom_gate_failures = 0
        target.status = "pregrasp_stable_wait"
        self.phase = "pregrasp_stable"
        self.pregrasp_stable_started_at = time.monotonic()
        self.pregrasp_stable_samples = 0
        self.pregrasp_last_pose = None
        # 头部俯仰对准目标层：给抓前近距复核攒足该层的近距离检测样本
        # （稳定等待 ~2s + REFINE_WINDOW 正好覆盖头部到位时间）。
        level_pitch = {1: PITCH_SCAN[2][0], 2: PITCH_SCAN[1][0], 3: PITCH_SCAN[0][0]}.get(
            getattr(target.candidate, "level", None)
        )
        if level_pitch is not None:
            self._head(level_pitch)
        self._publish_targets("pregrasp_retreat_reached")
        self._publish_flow("pregrasp_odom_stability_started")
        self.get_logger().info("[exec] 已到机械臂安全线，等待 odom 连续稳定后执行 plan-only IK")

    def _refine_target_before_ik(self, target: Target) -> None:
        """抓前近距复核：用近距离检测更新目标坐标（伸爪前的最后一道校正）。

        机器人已站在预抓取线，头部相机离货架仅 ~0.8m，此距离下的检测
        精度远高于扫描期（扫描期实测常态 y/z 偏 2-5cm、极端个案 z 偏
        29cm）。仅取旧目标 REFINE_MAX_RADIUS 半径内的样本做中位数，
        防止把旁边货位的同类副本当成目标。检测不足时沿用扫描期坐标。
        """
        cand = target.candidate
        if cand is None or cand.world is None:
            return
        now = time.monotonic()
        old = cand.world
        kind = str(target.kind).strip().lower()
        samples: list[tuple[tuple[float, float, float], float]] = []
        for track in self.tracks.get(kind, []):
            for ts, world, conf in track.samples:
                if now - ts > REFINE_WINDOW_S:
                    continue
                if math.dist(world, cand.world) > REFINE_MAX_RADIUS:
                    continue
                samples.append((world, conf))
        if len(samples) < REFINE_MIN_SAMPLES:
            self.get_logger().warning(
                f"[exec] 近距复核：{kind} 近 {REFINE_WINDOW_S:.1f}s 内仅 "
                f"{len(samples)}/{REFINE_MIN_SAMPLES} 个有效检测，"
                f"沿用扫描期坐标（仅补偿 y 系统偏差 +{DETECTION_Y_BIAS * 100:.0f}cm）"
            )
            cand.world = (old[0], old[1] + DETECTION_Y_BIAS, old[2])
            return
        old = cand.world
        refined = tuple(
            sorted(w[i] for w, _ in samples)[len(samples) // 2] for i in range(3)
        )
        # y 系统偏差补偿：检测深度系统性偏近（见 DETECTION_Y_BIAS 注释）
        refined = (refined[0], refined[1] + DETECTION_Y_BIAS, refined[2])
        drift = math.dist(refined, old)
        if drift <= 0.005:
            self.get_logger().info(
                f"[exec] 近距复核：{kind} 近距检测与扫描坐标一致（{len(samples)} 样本），不修正"
            )
            return
        cand.world = refined
        self.get_logger().info(
            f"[exec] 近距复核：{kind} 目标 "
            f"({old[0]:.3f},{old[1]:.3f},{old[2]:.3f}) → "
            f"({refined[0]:.3f},{refined[1]:.3f},{refined[2]:.3f})，"
            f"修正 {drift * 100:.1f}cm（{len(samples)} 样本，中位数）"
        )

    def _begin_plan_gate(self) -> None:
        target = self._current()
        if target is None:
            return
        target.status = "plan_only_ik"
        self.phase = "plan_gate"
        self._publish_grasp_meta(target)
        if self.plan_gate_worker is None:
            script = str(self.baseline_dir / "grasp_deploy_controller.py")
            command = [sys.executable, script, "--timeout", str(PLAN_GATE_TIMEOUT)]
            self.get_logger().info(
                f"[exec] 启动 plan-only IK 门禁：{' '.join(command)}；不发送机械臂命令"
            )
            self.plan_gate_worker = subprocess.Popen(command)
        self._publish_targets("plan_only_ik_started")
        self._publish_flow("plan_only_ik_started")

    def _pregrasp_stable_tick(self) -> None:
        target = self._current()
        if target is None:
            return
        now = time.monotonic()
        if self.pregrasp_stable_started_at is None:
            self.pregrasp_stable_started_at = now
        if now - self.pregrasp_stable_started_at > 20.0:
            self._fail_current("预抓取 odom 稳定超时")
            return
        if self.x is None or self.y is None or self.yaw is None:
            return
        pose = (float(self.x), float(self.y), float(self.yaw))
        if self.pregrasp_last_pose is None:
            self.pregrasp_stable_samples = 1
        else:
            pos_delta = math.hypot(pose[0] - self.pregrasp_last_pose[0], pose[1] - self.pregrasp_last_pose[1])
            yaw_delta = abs(math.atan2(
                math.sin(pose[2] - self.pregrasp_last_pose[2]),
                math.cos(pose[2] - self.pregrasp_last_pose[2]),
            ))
            if pos_delta <= ARM_ODOM_STABLE_POS_TOL and yaw_delta <= ARM_ODOM_STABLE_YAW_TOL:
                self.pregrasp_stable_samples += 1
            else:
                self.pregrasp_stable_samples = 1
        self.pregrasp_last_pose = pose
        if self.pregrasp_stable_samples >= ARM_ODOM_STABLE_SAMPLES:
            self.get_logger().info(
                f"[exec] odom 已稳定 {self.pregrasp_stable_samples} 帧，"
                f"pos=({pose[0]:.3f},{pose[1]:.3f}) yaw={pose[2]:.3f}，抓前近距复核"
            )
            self._refine_target_before_ik(target)
            self._begin_plan_gate()
        elif now - self.last_log > 2.0:
            self.get_logger().info(
                f"[exec] 等待预抓取 odom 稳定：{self.pregrasp_stable_samples}/"
                f"{ARM_ODOM_STABLE_SAMPLES}"
            )
            self.last_log = now

    def _plan_gate_tick(self) -> None:
        target = self._current()
        if target is None:
            return
        self._publish_grasp_meta(target)
        if self.plan_gate_worker is None:
            self._begin_plan_gate()
            return
        code = self.plan_gate_worker.poll()
        if code is None:
            return
        self.plan_gate_worker = None
        if code != 0:
            self._fail_current("plan-only IK 不可达")
            return
        self._publish_targets("plan_only_ik_passed")
        self._publish_flow("plan_only_ik_passed")
        if self.stop_after_ik:
            target.status = "ik_verified"
            target.error = None
            if target.candidate is not None:
                try:
                    self.map.release(target.candidate.slot, target.target_id)
                    target.reserved = False
                except ValueError as exc:
                    self.get_logger().warning(f"[exec] stop-after-ik 释放测试预订失败：{exc}")
            self._save_map()
            self.get_logger().info(
                f"[exec] plan-only IK 通过：target={target.target_id} kind={target.kind}；"
                "stop-after-ik 生效，跳过全部机械臂动作"
            )
            self.current_index += 1
            if self.current_index < len(self.targets):
                self._start_current_target()
            else:
                self._finish_exec(True)
            return
        self.get_logger().info("[exec] plan-only IK 通过，允许进入真实 deploy")
        self.phase = "arm"
        self.arm_index = 0
        self._start_arm_action()

    def _begin_arm(self) -> None:
        target = self._current()
        if target is None:
            return
        self.phase = "arm"
        self.arm_index = 0
        self._start_arm_action()

    def _start_arm_action(self) -> None:
        if self.arm_index >= len(ARM_ACTIONS):
            self._after_grasp_retreat()
            return
        action = ARM_ACTIONS[self.arm_index]
        self._spawn_arm(action)
        self.get_logger().info(f"[exec] 开始动作阶段 {action}")

    def _after_grasp_retreat(self) -> None:
        """夹爪已持物并撤出货架：导航穿越障碍走廊到配送台（反应式避障）。"""
        self.phase = "deliver"
        self.deliver_started_at = time.monotonic()
        self.observe_row = False
        self.drive_goals = self._delivery_route()
        self._drive_next_goal()
        self._publish_flow("deliver_navigation_started")

    def _begin_place(self) -> None:
        self.phase = "place"
        self._spawn_arm("place")

    def _complete_current(self) -> None:
        target = self._current()
        if target is None:
            self._finish_exec(True)
            return
        # place 已成功松爪，但机械臂仍可能保持伸展姿态；先收回到官方
        # safe_pose，确认关节反馈到位后再递增目标并启动返程。
        self.phase = "post_place_safe_pose"
        self.post_place_safe_pose_active = True
        self.arm_worker = None
        self._spawn_arm(POST_PLACE_SAFE_POSE)
        self._publish_flow("post_place_safe_pose_started")
        self.get_logger().info(f"[exec] 目标 {target.target_id} 已放置，先收回机械臂到 safe_pose")

    def _fail_current(self, reason: str) -> None:
        target = self._current()
        if target is not None:
            target.status = "failed"
            target.error = reason
            self.get_logger().error(f"[exec] 目标失败：{reason}")
            self._save_map()
        # 手眼渲染阶段开关兜底关闭：worker 挂死/目标失败时不能让手眼常开
        try:
            (self.baseline_dir / "debug_data" / "handeye_render.flag").unlink(missing_ok=True)
        except OSError:
            pass
        self._reset_navigation_state(reason=f"当前目标失败：{reason}")
        self._publish_targets("target_failed")
        # 真实机械动作失败时可能仍然夹持商品或处于不安全姿态，不能直接继续
        # 驱动底盘——但也不能因此终结整轮（5 个目标只跑 1 个）。折中：先收臂
        # 到 safe_pose 确认安全，再推进下一目标；连续失败到上限才停止。
        if (
            reason.startswith("动作阶段")
            or reason.startswith("place worker")
            or reason.startswith("post_place_safe_pose")
            or reason.startswith("配送")
            or reason.startswith("导航")
            or reason.startswith("plan-only IK")
            or reason.startswith("预抓取")
        ):
            self.consecutive_failures += 1
            if self.consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                self.get_logger().error(
                    f"[exec] 连续失败 {self.consecutive_failures} 次，达到上限，停止本轮"
                )
                self._finish_exec(False)
                return
            self.get_logger().warning(
                f"[exec] 目标失败（连续 {self.consecutive_failures}/{MAX_CONSECUTIVE_FAILURES}）："
                "先收臂到安全位，再继续下一目标"
            )
            self._recover_to_next()
            return
        self.current_index += 1
        if self.current_index < len(self.targets):
            self._start_current_target()
        else:
            self._finish_exec(False)

    def _finish_exec(self, ok: bool) -> None:
        self.exec_success = ok
        self.phase = "done"
        self.done = True
        self._publish_flow("random5_done" if ok else "random5_failed")

    def _finish_scan_only(self) -> None:
        """完成 E→D→C→B（必要时 A）扫描并停车，绝不进入机械臂执行阶段。"""
        if self.nav_worker is not None:
            self._stop_worker_process(self.nav_worker)
            self.nav_worker = None
        self.mapping_active = False
        self.map_completed_at = time.monotonic()
        self._save_map()
        self._publish_inventory()
        self._publish_targets("scan_only_complete")
        self._publish_flow("scan_only_complete")
        observed = self.map.to_dict()["observed_count"]
        self.exec_success = True
        self.phase = "done"
        self.done = True
        self.get_logger().info(
            f"[scan-only] E→D→C→B 扫描完成，observed={observed}/45；已停车，禁止启动机械臂动作"
        )

    def _mapping_tick(self) -> None:
        now = time.monotonic()
        if self.phase != "sweep":
            self._begin_sweep()
            return
        if self.mapping_mode == "scan":
            self._advance_scan()
        elif self.mapping_mode == "wait":
            if self.scan_only:
                self._finish_scan_only()
                return
            # 已扫完四排仍不足：就地多收几帧，按 map_timeout 收尾，避免无限等待
            if self.map.required_targets_ready([{"kind": t.kind} for t in self.targets]):
                self._enter_execution()
                return
            if now - self.started_at >= self.map_timeout:
                self.get_logger().warning("[exec] 建图超时仍未凑齐，先处理已找到的目标")
                self._enter_execution()
        elif self.mapping_mode == "drive":
            # 等待 nav reached（由 _on_nav_status 切 scan）
            if self.goal is not None:
                self._publish_goal_msg(self.goal[0], self.goal[1], self.goal[2])
            self._ensure_nav(
                self.mapping_max_speed,
                pickup=self._drive_pickup,
                pickup_transit=self._drive_pickup_transit,
            )

    def _arm_tick(self) -> None:
        if self._arm_worker_hard_timeout() is not None:
            self._fail_current("arm worker 挂死（硬超时）")
            return
        target = self._current()
        if target is None:
            return
        # worker 可能晚启动，重发抓取元数据
        if target.candidate is not None and target.candidate.world is not None:
            self._publish_grasp_meta(target)
        if self.arm_worker is None:
            self._start_arm_action()
            return
        code = self.arm_worker.poll()
        if code is None:
            return
        self.arm_worker = None
        if code != 0:
            action_now = ARM_ACTIONS[self.arm_index] if self.arm_index < len(ARM_ACTIONS) else "place"
            if action_now == "servo":
                # 伺服崩溃不应当作目标失败：跳过伺服继续闭爪（盲闭不比现状差）
                self.get_logger().warning("[exec] 手眼伺服 worker 异常退出，跳过伺服直接闭爪")
                self.arm_index += 1
                self._start_arm_action()
                return
            if action_now == "close_gripper" and self.depth_probe_count < 3:
                # 闭爪落空（夹空检测拦截）→ 伸入深度不足的强反馈：
                # 停止线 +3cm 再 creep→close，最多探测 3 次。
                self.depth_probe_count += 1
                self._servo_depth_extra = (self._servo_depth_extra or 0.0) + 0.03
                self.get_logger().warning(
                    f"[exec] 闭爪落空，深度探测 {self.depth_probe_count}/3："
                    f"creep 停止线 +3cm（累计 +{(self._servo_depth_extra) * 100:.0f}cm）"
                )
                self._publish_grasp_meta(target)
                self.arm_index = ARM_ACTIONS.index("creep")
                self._spawn_arm("creep")
                return
            if action_now == "creep" and self.creep_retry < CREEP_RETRY_MAX:
                self.creep_retry += 1
                self.get_logger().warning(
                    f"[exec] creep 未对准，重对准重试 {self.creep_retry}/{CREEP_RETRY_MAX}："
                    "回退 10cm 后重新 deploy→creep"
                )
                # -1 是让随后的 backoff 成功后 arm_index+1 正好落在 deploy
                self.arm_index = ARM_ACTIONS.index("deploy") - 1
                self._spawn_arm("backoff")
                return
            self._fail_current(f"动作阶段 {action_now} 失败")
            return
        completed_action = ARM_ACTIONS[self.arm_index] if self.arm_index < len(ARM_ACTIONS) else "unknown"
        last_action = self.arm_worker_action
        # 伺服重部署序列（backoff_far → slide_up → deploy）
        if self.servo_redeploy and last_action in ("backoff_far", "slide_up"):
            if last_action == "backoff_far":
                self.get_logger().info("[exec] 伺服重部署 1/3：已退离货架 30cm，抬手退出格子")
                self._spawn_arm("slide_up")
                return
            self.servo_redeploy = False
            self.arm_index = ARM_ACTIONS.index("deploy")
            self.get_logger().info("[exec] 伺服重部署 2/3：就位，重新 deploy→creep→servo")
            self._start_arm_action()
            return
        if completed_action == "servo":
            # 手眼伺服：只修横向 x。z 由逐类 deploy_offset 校准（抓在罐身上部
            # 是刻意设计：21cm 高罐的可用夹持带宽，且低位全伸展逼近工作空间
            # 边界——实测往真值 z 修 7.4cm 后 deploy 关节误差卡 0.23 rad 不
            # 收敛直到硬超时）。阻尼 0.7 防过冲（实测 +3.7 → 过冲到 -2.0）。
            correction, measured = self._read_servo_result()
            cand = target.candidate
            clamped_dx = max(-0.03, min(0.03, correction[0] * 0.7))
            # 伸入深度：表观尺寸测距被指头遮挡污染（实测 180px vs 预期 262px），
            # 不可靠。深度改由"闭爪探测"闭环：夹空 → +3cm → 再 creep → 再闭。
            lateral_ok = abs(clamped_dx) <= SERVO_APPLY_THRESHOLD
            need_lateral = measured and not lateral_ok and cand is not None \
                and cand.world is not None and self.servo_retry < SERVO_RETRY_MAX
            if need_lateral:
                self.servo_retry += 1
                old = cand.world
                cand.world = (old[0] + clamped_dx, old[1], old[2])
                self.get_logger().warning(
                    f"[exec] 手眼伺服：横向偏差 {correction[0] * 100:+.1f}cm"
                    f"（垂直 {correction[2] * 100:+.1f}cm 忽略，z 由标定 dz 决定），"
                    f"阻尼修正 {clamped_dx * 100:+.1f}cm，"
                    f"目标 x {old[0]:.3f} → {cand.world[0]:.3f}，重部署 {self.servo_retry}/{SERVO_RETRY_MAX}"
                )
                self._publish_grasp_meta(target)
                # 重部署序列：先退 30cm 脱离货架前沿，再抬手，再 deploy
                self.servo_redeploy = True
                self._spawn_arm("backoff_far")
                return
            if measured:
                self.get_logger().info(
                    f"[exec] 手眼伺服：偏差在阈值内（{correction[0] * 100:+.1f}, "
                    f"{correction[2] * 100:+.1f})cm，直接闭爪"
                )
            else:
                self.get_logger().warning("[exec] 手眼伺服未测得目标，盲闭（不比现状差）")
        if self.stop_after_close and completed_action == "close_gripper":
            target = self._current()
            if target is not None:
                target.status = "grasped_hold"
                target.error = None
            self._stop_worker_process(self.nav_worker)
            self.nav_worker = None
            self._publish_targets("close_gripper_verified_stop")
            self._publish_flow("close_gripper_verified_stop")
            self.exec_success = True
            self.phase = "done"
            self.done = True
            self.get_logger().info(
                "[exec] close_gripper 已完成，已停车检查；按 stop-after-close 跳过 lift/retreat/deliver/place"
            )
            return
        self.arm_index += 1
        self._start_arm_action()

    def _recover_to_next(self) -> None:
        """动作失败后的安全恢复：收臂到 safe_pose，再推进下一目标。"""
        self.recovering_from_failure = True
        self.phase = "post_place_safe_pose"
        self.post_place_safe_pose_active = True
        self.arm_worker = None
        self._spawn_arm(POST_PLACE_SAFE_POSE)
        self._publish_flow("failed_recover_to_next")

    def _post_place_safe_pose_tick(self) -> None:
        if self._arm_worker_hard_timeout() is not None:
            # 收臂 worker 挂死：直接推进，不再重新 spawn（否则死循环）
            self.post_place_safe_pose_active = False
            self.recovering_from_failure = False
            target = self._current()
            if target is not None:
                target.status = "failed"
                self.get_logger().warning(
                    f"[exec] 收臂 worker 硬超时，目标 {target.target_id} 标记失败并跳过"
                )
            self.current_index += 1
            self._save_map()
            if self.current_index < len(self.targets):
                if not self.fixed_task_order:
                    self._reorder_remaining_targets("post_place_safe_pose")
                self._start_current_target()
            else:
                self._finish_exec(False)
            return
        if self.arm_worker is None:
            self._fail_current("post_place_safe_pose worker 丢失")
            return
        code = self.arm_worker.poll()
        if code is None:
            return
        self.arm_worker = None
        self.post_place_safe_pose_active = False
        if code != 0:
            self._fail_current("post_place_safe_pose worker 失败")
            return
        target = self._current()
        if target is None:
            self._finish_exec(True)
            return
        if self.recovering_from_failure:
            # 该目标是失败跳过的，不能标记为已放置（会污染库存地图）
            target.status = "failed"
            self.recovering_from_failure = False
            self._publish_flow("failed_target_skipped")
            self.get_logger().warning(
                f"[exec] 目标 {target.target_id} 失败跳过，继续下一目标"
            )
        else:
            if target.candidate is not None:
                self.map.mark(target.candidate.slot, "placed", target.target_id)
            target.status = "placed"
            self.consecutive_failures = 0
        self.current_index += 1
        self._save_map()
        self._publish_inventory()
        self._publish_flow("post_place_safe_pose_verified")
        self.get_logger().info(f"[exec] safe_pose 收臂完成，目标 {target.target_id} 完成并开始返程")
        if self.current_index < len(self.targets):
            if self.fixed_task_order:
                self.get_logger().info(
                    "[exec] 显式任务顺序已锁定：保持下一目标原始顺序"
                )
            else:
                self._reorder_remaining_targets("post_place_safe_pose")
            self._start_current_target()
        else:
            self._finish_exec(True)

    def _place_tick(self) -> None:
        if self.arm_worker is None:
            self._begin_place()
            return
        code = self.arm_worker.poll()
        if code is None:
            return
        self.arm_worker = None
        if code == 0:
            self._complete_current()
        else:
            self._fail_current("place worker 失败")

    def _execute_phase_tick(self) -> None:
        target = self._current()
        if target is None:
            self._finish_exec(self.exec_success)
            return
        now = time.monotonic()
        if self.phase == "deliver" and self._in_delivery_base():
            # 官方 S4 只要求底盘进入 delivery_base。配送台附近可能
            # 物理上无法继续对齐到人为设定的精确 yaw；进入区域后
            # 停止导航 worker，立即交给 place worker。
            if self.nav_worker is not None:
                self._stop_worker_process(self.nav_worker)
                self.nav_worker = None
            self.get_logger().info(
                f"[exec] 底盘已进入官方配送区：pos=({self.x:.3f},{self.y:.3f})，停止导航进入 place"
            )
            self._begin_place()
            return
        if self.phase == "target_nav":
            # 在到达前低频重发当前段目标，确保 nav worker 不遗漏
            if self.goal is not None:
                self._publish_goal_msg(self.goal[0], self.goal[1], self.goal[2])
            self._ensure_nav(
                self.mapping_max_speed,
                pickup=self._drive_pickup,
                pickup_transit=self._drive_pickup_transit,
            )
        elif self.phase == "pregrasp_nav":
            # 预抓取后退阶段仍由底盘导航 worker 独占；持续重发当前
            # 目标并在 worker 异常退出时拉起，直到 reached 回调切入
            # odom 稳定与 plan-only IK 门禁。
            if self.goal is not None:
                self._publish_goal_msg(self.goal[0], self.goal[1], self.goal[2])
            self._ensure_nav(
                self.mapping_max_speed,
                pickup=self._drive_pickup,
                pickup_transit=self._drive_pickup_transit,
            )
        elif self.phase == "pregrasp_stable":
            self._pregrasp_stable_tick()
        elif self.phase == "plan_gate":
            self._plan_gate_tick()
        elif self.phase == "arm":
            self._arm_tick()
        elif self.phase == "deliver":
            # 等待 nav reached → _begin_place（由 _on_nav_status 处理）。
            # 配送阶段使用独立计时，避免把建图/抓取耗时误算进配送超时。
            # 独立常量：此前借用 map_timeout*2≈36s——RTF 0.45 时配送全程
            # 需要 40-60s，看门狗把正常行驶的配送杀了（实测）。
            if (
                self.deliver_started_at is not None
                and now - self.deliver_started_at > DELIVER_NAV_TIMEOUT
            ):
                self._fail_current("配送导航超时")
        elif self.phase == "place":
            self._place_tick()
        elif self.phase == "post_place_safe_pose":
            self._post_place_safe_pose_tick()
        elif self.phase in ("done",):
            return
        else:
            self.get_logger().warning(f"[exec] 未知相位 {self.phase}")

    def _execute_tick(self) -> None:
        if self.rejected_execute or self.done:
            return
        if not self.task_received:
            return
        self._update_map()
        if self.mapping_active:
            self._mapping_tick()
        else:
            self._execute_phase_tick()


    def _save_map(self) -> None:
        if not self.no_publish:
            self.map.save(self.map_path)

    def _tick(self) -> None:
        if not self.task_received:
            return
        if self.execute_enabled:
            self._execute_tick()
        else:
            self._update_map()
            self._map_complete()
            if not self.mapping_active and self.current_index < len(self.targets):
                target = self.targets[self.current_index]
                if target.candidate is not None and target.status in {"searching", "navigating"}:
                    self._publish_navigation(target)
        if time.monotonic() - self.last_log > 5.0:
            self.get_logger().info(
                f"flow phase={'mapping' if self.mapping_active else 'execution'} "
                f"mode={'execute' if self.execute_enabled else 'plan-only'} "
                f"observed={self.map.to_dict()['observed_count']}/45 "
                f"target_index={self.current_index}/{len(self.targets)} nav={self.nav_state} "
                f"sub={self.phase if self.execute_enabled else '-'}"
            )
            self.last_log = time.monotonic()

    def run(self) -> int:
        try:
            while rclpy.ok() and not self.done:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_worker_process(self.nav_worker)
            self._stop_worker_process(self.arm_worker)
            self._stop_worker_process(self.plan_gate_worker)
            self.nav_worker = None
            self.arm_worker = None
            self.plan_gate_worker = None
            self._save_map()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0 if (not self.execute_enabled or self.exec_success) else 1

def main() -> int:
    ap = argparse.ArgumentParser(description="随机45商品库存地图比赛编排器")
    ap.add_argument("--map-timeout", type=float, default=180.0)
    ap.add_argument("--min-map-seconds", type=float, default=8.0)
    ap.add_argument("--min-samples", type=int, default=5)
    ap.add_argument("--min-confidence", type=float, default=0.25)
    ap.add_argument("--map-path", default="/workspace/baseline/debug_data/inventory_map_current.json")
    ap.add_argument("--no-publish", action="store_true", help="不写本地地图文件")
    ap.add_argument("--execute", action="store_true", help="真实自主运行（巡游扫描 + spawn 底盘/机械臂 worker）")
    ap.add_argument("--confirm", default="", help="须为 random5 才允许真实执行")
    ap.add_argument("--mapping-max-speed", type=float, default=DEFAULT_MAPPING_SPEED, help="货架前巡游扫描限速 m/s")
    ap.add_argument("--cruise-max-speed", type=float, default=DEFAULT_CRUISE_SPEED, help="长直 clear 段巡航限速 m/s")
    ap.add_argument("--obstacle-distance", type=float, default=DEFAULT_OBSTACLE, help="动态避障触发距离 m")
    ap.add_argument("--arm-timeout", type=float, default=180.0, help="动作 worker 超时 s")
    ap.add_argument("--scan-only", action="store_true", help="真实底盘扫描 E→D→C→B，必要时补扫 A，完成后停车并退出，不启动机械臂动作")
    ap.add_argument("--stop-after-ik", action="store_true", help="目标导航/plan-only IK 后立即进入下一目标，禁止全部机械臂动作")
    ap.add_argument("--stop-after-close", action="store_true", help="close_gripper 成功后立即停车，跳过 lift/retreat/deliver/place")
    args = ap.parse_args()
    if args.execute and args.confirm != "random5":
        print("拒绝执行：必须使用 --confirm random5")
        return 2
    rclpy.init()
    return InventoryCompetitionExecutor(
        map_timeout=args.map_timeout, min_map_seconds=args.min_map_seconds,
        min_samples=args.min_samples, min_confidence=args.min_confidence,
        map_path=args.map_path,
        no_publish=args.no_publish, execute=args.execute, confirm=args.confirm,
        mapping_speed=args.mapping_max_speed, cruise_speed=args.cruise_max_speed,
        obstacle_distance=args.obstacle_distance, arm_timeout=args.arm_timeout,
        scan_only=args.scan_only, stop_after_ik=args.stop_after_ik, stop_after_close=args.stop_after_close,
    ).run()

if __name__ == "__main__":
    raise SystemExit(main())

