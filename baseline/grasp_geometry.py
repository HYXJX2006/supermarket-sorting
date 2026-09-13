#!/usr/bin/env python3
"""按官方 MJCF 尺寸维护商品类别抓取几何。

RGB-D 检测点通常落在商品朝向机器人一侧的可见表面；抓取规划需要先把
表面点沿机器人前进方向修正到商品中心，再生成 deploy/creep 目标。

本文件只记录几何事实和已验证的官方动作基线。几何尺寸来自
``official_baseline/.../mjcf/retail_competition.xml``；运动偏置仍可由
``SUPERMARKET_<KIND>_*`` 环境变量覆盖，并且必须通过真实 lift 验证后
才能视为该类别已标定。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class GraspGeometry:
    """相对于商品世界中心的抓取几何和动作参数。"""

    # RGB-D 表面点沿机器人前进方向到商品中心的距离（米）。
    surface_to_center_fwd: float
    # 预抓取末端相对商品中心的 x/y/z 偏置（米）。
    deploy_offset: tuple[float, float, float]
    # creep 阶段末端在前进方向上相对商品中心的停止偏置（米）。
    creep_stop_dy: float
    # deploy 末端笛卡尔位置和姿态容差。
    deploy_position_tolerance: float = 0.065
    deploy_rotation_tolerance: float = 0.35
    # creep 最终末端位置容差。
    creep_position_tolerance: tuple[float, float, float] = (0.065, 0.025, 0.045)
    # 官方 MJCF 几何元数据。
    shape: str = "unknown"
    dimensions_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mass_kg: float = 0.0
    source: str = "unknown"
    calibrated: bool = False
    # Controller target for the tendon/gripper command. A larger value means
    # a less aggressively closed gripper in this simulation.
    grip_close: float = 0.08
    # JointState feedback threshold accepted by lift as "closed enough".
    grip_feedback_max: float = 0.85
    # 夹空检测下限：闭爪后 feedback 低于该值 = 指头越过商品闭到指令位
    # （没夹到东西）。0 = 不启用。比赛流程靠它识别"空手配送"。
    grip_feedback_min: float = 0.0
    # creep 接近速度（m/s），逐类标定值。0.15 是 09-11 翻车排查后的安全上限；
    # 轻且形状不规则（sanmingzhi 顶推敏感）、预压敏感（maidong 滑）的类别
    # 保留标定时的 0.12。覆盖通道：SUPERMARKET_<KIND>_CREEP_SPEED。
    creep_speed: float = 0.15

    def to_dict(self) -> dict[str, object]:
        """输出可写入 ROS JSON 的稳定元数据。"""

        return {
            "shape": self.shape,
            "dimensions_m": [round(float(v), 5) for v in self.dimensions_m],
            "mass_kg": round(float(self.mass_kg), 5),
            "source": self.source,
            "calibrated": bool(self.calibrated),
            "grip_close": round(float(self.grip_close), 5),
            "grip_feedback_max": round(float(self.grip_feedback_max), 5),
            "grip_feedback_min": round(float(self.grip_feedback_min), 5),
            "surface_to_center_fwd": round(float(self.surface_to_center_fwd), 5),
            "deploy_offset": [round(float(v), 5) for v in self.deploy_offset],
            "creep_stop_dy": round(float(self.creep_stop_dy), 5),
            "deploy_position_tolerance": round(float(self.deploy_position_tolerance), 5),
            "deploy_rotation_tolerance": round(float(self.deploy_rotation_tolerance), 5),
            "creep_position_tolerance": [round(float(v), 5) for v in self.creep_position_tolerance],
        }


# 官方动作骨架的共同基线；类别尺寸和表面到中心距离在下方单独定义。
DEFAULT_GEOMETRY = GraspGeometry(
    surface_to_center_fwd=float(os.getenv("SUPERMARKET_SURFACE_TO_CENTER_FWD", "0.0265")),
    deploy_offset=(
        # 横向偏置（09-13 修正）：DX 加在世界坐标 x 上，机器人面向货架
        # （yaw=+90°）时 +x=机器人右侧、-x=左侧。此前 -0.056/-0.110 的
        # "左漂补偿"把坐标系搞反了（REP-103 的 x 朝前/y 朝左是本体系，
        # 不是这里的世界系），实测越负越偏左：-0.060 偏左 6cm、-0.110
        # 偏左 11cm（诊断帧+主人截图）。回退基线 -0.011（8/8 标定全成
        # 功的值），残余偏差交伺服 j3 收尾（±3-4cm 量程）。
        float(os.getenv("SUPERMARKET_DEPLOY_DX", "0.02")),   # 09-13 主人定：+0.02（向右 2cm 微调）
        float(os.getenv("SUPERMARKET_DEPLOY_DY", "-0.220")),
        float(os.getenv("SUPERMARKET_DEPLOY_DZ", "-0.010")),
    ),
    creep_stop_dy=float(os.getenv("SUPERMARKET_CREEP_STOP_DY", "0.035")),
    shape="unknown",
    dimensions_m=(0.0, 0.0, 0.0),
    mass_kg=0.0,
    source="fallback",
    calibrated=False,
    grip_close=float(os.getenv("SUPERMARKET_GRIP_CLOSE", "0.08")),
    grip_feedback_max=float(os.getenv("SUPERMARKET_GRIP_FEEDBACK_MAX", "0.85")),
    creep_speed=float(os.getenv("SUPERMARKET_CREEP_SPEED", "0.15")),
)

# --- 夹爪开口换算：来自官方 MJCF，不是经验值 -----------------------------
# mmk2_options.xml:      腱 rgt_gripper_gear = -12.5*左指 + 12.5*右指
# airbot_play_dependencies.xml: finger1/finger2 为对称 slide，各行程 0.04 m，
#                         并被 equality 约束成反向同步。
# 因此 joint_states 里的 right_arm_eef_gripper_joint（腱位移）与两指开口是
# 固定线性关系：
#     两指开口(m) = feedback / 12.5        最大开口 0.08 m（feedback = 1.0）
# 反过来说，feedback 就是"两指张开多少米 × 12.5"。
#
# 交叉验证：kele 直径 0.0530 m → 预期 0.663（lift_controller 注释独立记录
# 的"闭合后稳定在 0.66"正是它）；shupian 直径 0.0650 m → 预期 0.813，低于
# 0.85 门禁，实测可正常抬升。两者都与换算一致。
TENDON_PER_METER = 12.5
MAX_GRIPPER_OPENING_M = 0.08
# 通用门禁下限：夹持宽度小于 0.068 m 的类别不必单独放宽，沿用已验证的 0.85。
GRIP_FEEDBACK_FLOOR = 0.85
# 正确夹住时留 12% 余量：腱反馈的最终值有 ±3% 波动（MuJoCo 接触求解，
# 实测同一参数多次闭爪 0.8265/0.856/0.881 横跨门禁），余量不足会把
# 成功夹取误拒在 lift 门口。门禁只是"允许尝试 lift"的门槛，
# 成功与否最终以裁判遥测（gripped 帧数 + 物体 z 上升）为准。
_GRIP_GATE_MARGIN = 1.12


def grip_gate_for_opening(opening_m: float) -> float:
    """把"正确夹住时两指应张开的米数"换算成 lift 门禁阈值。

    opening_m 取商品被夹持方向的宽度（球/圆柱即直径，长方体取两指跨过的
    那一维尺寸）。该阈值**只决定是否允许进入一次真实 lift**，不构成抓取
    成功的判定——最终仍以物体是否随升降柱上升（遥测 freejoint 高度）为准。
    """

    derived = float(opening_m) * TENDON_PER_METER * _GRIP_GATE_MARGIN
    return round(max(GRIP_FEEDBACK_FLOOR, derived), 4)


# ``dimensions_m`` 统一表示包络尺寸 (x, y, z)，不是 MuJoCo 的半尺寸。
# surface_to_center_fwd 对应官方物体朝机器人一侧的 y 半深度。
#
# grip_feedback_max 逐类由 gripper 开口反推，不再手写常数：
#     kouxiangtang 0.0490 → 0.8500(下限)   kele  0.0530 → 0.8500(下限)
#     sanmingzhi   0.0650 → 0.8531         shupian 0.0650 → 0.8531
#     maidong      0.0650 → 0.8531         pingguo 0.0700 → 0.9188
#     chengzi      0.0740 → 0.9713         zhijin  见该条目注释（开口超行程）
_OFFICIAL_GEOMETRY: dict[str, GraspGeometry] = {
    "sanmingzhi": GraspGeometry(
        # 2026-09-10 修正：原 0.0500（按 y 深 0.1000 的一半估算）是错的。
        # 从 MJCF mesh 顶点看，三角形截面在 yz 平面（x 为拉伸方向），
        # 机器人沿 +y 接近遇到的是斜面，在商品中心高度（z=0）处：
        #   斜面从 (y=-0.0500, z=-0.0494) 到 (y=+0.0345, z=+0.0494)
        #   t=0.5 → y = -0.0500 + 0.5 × 0.0845 = -0.00775
        # 真实表面距仅 0.00775。原 0.0500 使 stop_y 超前真实表面 0.093 m，
        # 夹爪停在商品前方一路顶推（实测推走 12.2 cm，SINGLE_FINGER）。
        # 扫描确认 0.005 最优（得分 890.4，抬升 42.2 mm，位移仅 6.4 mm）。
        surface_to_center_fwd=0.0050,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        # 标定（得分 890.4 那轮）用 0.12 接近；轻且正面斜面，速度敏感。
        creep_speed=0.12,
        shape="mesh_prism",
        dimensions_m=(0.0650, 0.1000, 0.0988),
        mass_kg=0.1220,
        source="official_mjcf",
        # 两指跨过 x 维 0.0650（approach 沿 y，故 surface 修正为 0.1000/2）。
        grip_feedback_max=grip_gate_for_opening(0.0650),
        grip_feedback_min=0.7,
        calibrated=True,
    ),
    "heweidao": GraspGeometry(
        # At the cup mid-height, the tapered mesh radius is 0.040 m;
        # use this front-surface radius for the RGB-D center correction.
        surface_to_center_fwd=0.0400,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        shape="mesh_frustum",
        dimensions_m=(0.0950, 0.0950, 0.1050),
        mass_kg=0.0990,
        source="official_mjcf",
        # Telemetry: 0.205 still held both fingers; 0.095 lost the left finger.
        # Use a less aggressive hold target and a category-specific lift gate.
        # 轻夹策略（grip_close=0.22）下"夹住"与"没夹住"的反馈差异天然很小
        # （实测波动带 0.94~0.99），门禁卡 0.95 会把成功夹取误拒在毫厘之间
        # （demo 实测 0.9541 vs 0.950）。放宽到 0.99 允许尝试 lift ——
        # 最终判定以裁判遥测为准，门禁不构成成功判据。
        grip_close=0.22,
        grip_feedback_max=0.99,
        grip_feedback_min=0.5,
    ),
    "shupian": GraspGeometry(
        surface_to_center_fwd=0.0325,
        # 2026-09-12：dz 从 -0.010 抬到 +0.030。实测 L2 货位接近时手（指+
        # 掌+腕）前端刮到下层板前沿，底盘推进被层板抵住（ee_y 卡 3.150，
        # 每秒仅 0.5mm，主人目视确认"被薯片下面的货架板抵住"）。罐高 0.21，
        # 指头抓在中心上方 3cm 依然稳定，整只手随之上抬离开层板沿。
        deploy_offset=(0.02, -0.220, 0.030),   # dx +0.02（主人 09-13 定）；dz+0.030 防层板刮蹭
        # 2026-09-12 深夜：0.035 时指头平面落后罐身 ~2cm（比赛流程连续夹空）。
        # 2026-09-13 主人实测：0.010 仍"只夹到桶的一半"——插入深度不足，
        # 再加深 2cm 到 -0.010（停止线 = y+0.0225，指头平面越过罐心）。
        creep_stop_dy=-0.010,
        shape="cylinder",
        dimensions_m=(0.0650, 0.0650, 0.2100),
        mass_kg=0.1370,
        source="official_mjcf",
        grip_feedback_max=grip_gate_for_opening(0.0650),
        grip_feedback_min=0.6,
    ),
    "zhijin": GraspGeometry(
        surface_to_center_fwd=0.0425,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        shape="box",
        dimensions_m=(0.1720, 0.0850, 0.0880),
        mass_kg=0.1300,
        source="official_mjcf",
        # 注意：夹爪最大开口 0.08 m，而本商品最小边 0.0850 m。按当前
        # surface_to_center_fwd=0.0425（= 0.0850/2，说明 approach 沿 y）
        # 两指要跨过 x 维 0.1720，几何上不可能合拢。此处刻意不套用开口
        # 换算（推导值会超过物理上限 1.0 而使门禁失效），保持 0.85，
        # 待实测确认是否需要改从窄边/边角夹持后再单独标定。
        grip_feedback_max=0.85,
    ),
    "maidong": GraspGeometry(
        surface_to_center_fwd=0.0325,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        # 预压敏感（0.060 滑 / 0.040 压歪），最重 0.643kg，接近速度保守。
        creep_speed=0.12,
        shape="cylinder",
        dimensions_m=(0.0650, 0.0650, 0.2100),
        mass_kg=0.6430,
        source="official_mjcf",
        # 2026-09-10 修正：0.643 kg 是九类最重（shupian 的 4.7 倍）。
        # 默认 grip_close=0.08 时指头闭到「贴住表面」即停（开口 66.1≈直径 65，
        # 零预压），lift 只抬到 4.8 mm 商品就在指间下滑（摩擦力撑不住重力）。
        # 扫描 0.060/0.050/0.040：0.050 成功（抬升 41.6 mm，得分 768.5），
        # 0.060 太松滑落、0.040 过紧把商品压歪挤出（倾角 27°）——
        # 典型的预压最优曲线。重物必须带预压，这正是逐类参数的意义。
        # 2026-09-11 补充：闭爪接触的最终反馈有 ±3% 随机波动（MuJoCo 接触
        # 求解），两次实测 0.8265/0.8808 横跨原门禁 0.8531 —— 门禁卡在波动带内
        # 会时好时坏。放宽到 0.90：仍能拒绝"完全没夹"（feedback→1.0 全开），
        # 但吸收波动。标定扫描已证 0.050 预压下能稳定抬升 41.6mm。
        grip_close=0.050,
        grip_feedback_max=0.90,
        grip_feedback_min=0.7,
        calibrated=True,
    ),
    "kele": GraspGeometry(
        surface_to_center_fwd=0.0265,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        shape="cylinder",
        dimensions_m=(0.0530, 0.0530, 0.1450),
        mass_kg=0.3570,
        source="official_mjcf",
        # 0.0530 → 0.696，低于下限，仍用 0.85；实测已通过。
        grip_feedback_max=grip_gate_for_opening(0.0530),
        grip_feedback_min=0.6,
    ),
    "kouxiangtang": GraspGeometry(
        surface_to_center_fwd=0.0245,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        shape="cylinder",
        dimensions_m=(0.0490, 0.0490, 0.0800),
        mass_kg=0.0733,
        source="official_mjcf",
        # 0.0490 → 0.643，低于下限，仍用 0.85。
        grip_feedback_max=grip_gate_for_opening(0.0490),
        grip_feedback_min=0.6,
    ),
    "pingguo": GraspGeometry(
        surface_to_center_fwd=0.0350,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        shape="sphere",
        dimensions_m=(0.0700, 0.0700, 0.0700),
        mass_kg=0.2510,
        source="official_mjcf",
        # 2026-09-10 修复：球径 0.0700 → 两指正确夹住时反馈应为 0.875，
        # 高于通用 0.85 门禁，导致 apple 每次都被 lift 安全门禁拒绝。
        # 实测反馈 0.872 与该换算吻合（误差 3 mm），说明球体本身已夹正，
        # 是门禁阈值按 0.85 硬编码造成的误判，不是抓取几何错误。
        grip_feedback_max=grip_gate_for_opening(0.0700),
        grip_feedback_min=0.75,
    ),
    "chengzi": GraspGeometry(
        surface_to_center_fwd=0.0370,
        deploy_offset=DEFAULT_GEOMETRY.deploy_offset,
        creep_stop_dy=DEFAULT_GEOMETRY.creep_stop_dy,
        shape="sphere",
        dimensions_m=(0.0740, 0.0740, 0.0740),
        mass_kg=0.2530,
        source="official_mjcf",
        # 与 pingguo 同类的门禁误判：0.0740 → 0.925，高于 0.85。
        # 提前一并修正，避免标定完苹果后再在橙子上重复同一个问题。
        grip_feedback_max=grip_gate_for_opening(0.0740),
        grip_feedback_min=0.85,
    ),
}

KNOWN_KINDS = (
    "chengzi", "heweidao", "kele", "kouxiangtang", "maidong",
    "pingguo", "sanmingzhi", "shupian", "zhijin",
)


def _kind_geometry(kind: str, base: GraspGeometry) -> GraspGeometry:
    """应用类别环境变量覆盖，同时保留官方形状/尺寸元数据。"""

    prefix = f"SUPERMARKET_{kind.upper()}_"
    dims = tuple(
        float(os.getenv(prefix + f"DIM_{axis}", str(base.dimensions_m[index])))
        for index, axis in enumerate(("X", "Y", "Z"))
    )
    return GraspGeometry(
        surface_to_center_fwd=float(
            os.getenv(prefix + "SURFACE_TO_CENTER_FWD", str(base.surface_to_center_fwd))
        ),
        deploy_offset=(
            float(os.getenv(prefix + "DEPLOY_DX", str(base.deploy_offset[0]))),
            float(os.getenv(prefix + "DEPLOY_DY", str(base.deploy_offset[1]))),
            float(os.getenv(prefix + "DEPLOY_DZ", str(base.deploy_offset[2]))),
        ),
        creep_stop_dy=float(os.getenv(prefix + "CREEP_STOP_DY", str(base.creep_stop_dy))),
        deploy_position_tolerance=float(
            os.getenv(prefix + "DEPLOY_POS_TOL", str(base.deploy_position_tolerance))
        ),
        deploy_rotation_tolerance=float(
            os.getenv(prefix + "DEPLOY_ROT_TOL", str(base.deploy_rotation_tolerance))
        ),
        creep_position_tolerance=(
            float(os.getenv(prefix + "CREEP_X_TOL", str(base.creep_position_tolerance[0]))),
            float(os.getenv(prefix + "CREEP_Y_TOL", str(base.creep_position_tolerance[1]))),
            float(os.getenv(prefix + "CREEP_Z_TOL", str(base.creep_position_tolerance[2]))),
        ),
        shape=os.getenv(prefix + "SHAPE", base.shape),
        dimensions_m=dims,
        mass_kg=float(os.getenv(prefix + "MASS_KG", str(base.mass_kg))),
        source=base.source,
        calibrated=os.getenv(prefix + "CALIBRATED", str(base.calibrated)).lower() in {"1", "true", "yes"},
        grip_close=float(os.getenv(prefix + "GRIP_CLOSE", str(base.grip_close))),
        grip_feedback_max=float(os.getenv(prefix + "GRIP_FEEDBACK_MAX", str(base.grip_feedback_max))),
        # 夹空检测下限：2026-09-12 实测整场漏抓——_kind_geometry 重建时
        # 漏了这个字段，八类全部静默回落 0.0（检测不启用），夹空 feedback
        # 0.08 照样被 lift 放行、空手配送。逐类值见 _OFFICIAL_GEOMETRY。
        grip_feedback_min=float(os.getenv(prefix + "GRIP_FEEDBACK_MIN", str(base.grip_feedback_min))),
        creep_speed=float(os.getenv(prefix + "CREEP_SPEED", str(base.creep_speed))),
    )


GEOMETRY_BY_KIND: Mapping[str, GraspGeometry] = {
    kind: _kind_geometry(kind, _OFFICIAL_GEOMETRY[kind]) for kind in KNOWN_KINDS
}

# 保留旧导入名，避免已有调试脚本断裂。
SHUPIAN_GEOMETRY = GEOMETRY_BY_KIND["shupian"]


def geometry_for_kind(kind: str | None) -> GraspGeometry:
    """返回商品类别抓取几何；未知类别使用通用回退配置。"""

    return GEOMETRY_BY_KIND.get(str(kind or "").strip().lower(), DEFAULT_GEOMETRY)
