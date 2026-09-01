#!/usr/bin/env python3
"""商品类别相关的抓取几何参数。

视觉深度点通常落在商品朝向相机的可见表面，而机械臂需要对准可夹取的
几何中心。这里集中维护不同商品形态的修正，避免 planner、deploy、creep
各自使用不一致的常量。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class GraspGeometry:
    """相对于世界坐标商品中心的抓取几何参数。"""

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
    creep_position_tolerance: tuple[float, float, float] = (0.045, 0.025, 0.045)


# 默认值保持原有通用商品行为，避免影响尚未标定的类别。
DEFAULT_GEOMETRY = GraspGeometry(
    surface_to_center_fwd=float(os.getenv("SUPERMARKET_SURFACE_TO_CENTER_FWD", "0.0265")),
    deploy_offset=(
        float(os.getenv("SUPERMARKET_DEPLOY_DX", "-0.011")),
        float(os.getenv("SUPERMARKET_DEPLOY_DY", "-0.220")),
        float(os.getenv("SUPERMARKET_DEPLOY_DZ", "-0.010")),
    ),
    creep_stop_dy=float(os.getenv("SUPERMARKET_CREEP_STOP_DY", "0.035")),
)

# shupian 在仿真中是桶装/圆柱形容器，不是软袋。
# 0.0325 m 来自官方 MJCF product_003 的 cylinder size 半径，并保留环境变量便于后续实测标定。
SHUPIAN_GEOMETRY = GraspGeometry(
    surface_to_center_fwd=float(os.getenv("SUPERMARKET_SHUPIAN_SURFACE_TO_CENTER_FWD", "0.0325")),
    deploy_offset=(
        float(os.getenv("SUPERMARKET_SHUPIAN_DEPLOY_DX", "-0.011")),
        float(os.getenv("SUPERMARKET_SHUPIAN_DEPLOY_DY", "-0.220")),
        float(os.getenv("SUPERMARKET_SHUPIAN_DEPLOY_DZ", "-0.010")),
    ),
    creep_stop_dy=float(os.getenv("SUPERMARKET_SHUPIAN_CREEP_STOP_DY", "0.035")),
    deploy_position_tolerance=float(os.getenv("SUPERMARKET_SHUPIAN_DEPLOY_POS_TOL", "0.065")),
    deploy_rotation_tolerance=float(os.getenv("SUPERMARKET_SHUPIAN_DEPLOY_ROT_TOL", "0.35")),
    creep_position_tolerance=(
        float(os.getenv("SUPERMARKET_SHUPIAN_CREEP_X_TOL", "0.045")),
        float(os.getenv("SUPERMARKET_SHUPIAN_CREEP_Y_TOL", "0.025")),
        float(os.getenv("SUPERMARKET_SHUPIAN_CREEP_Z_TOL", "0.045")),
    ),
)

GEOMETRY_BY_KIND: Mapping[str, GraspGeometry] = {
    "shupian": SHUPIAN_GEOMETRY,
}


def geometry_for_kind(kind: str | None) -> GraspGeometry:
    """返回商品类别的抓取几何配置；未知类别使用通用配置。"""

    return GEOMETRY_BY_KIND.get(str(kind or "").strip().lower(), DEFAULT_GEOMETRY)

