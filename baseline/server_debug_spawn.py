#!/usr/bin/env python3
"""Server build_config patch for opt-in shelf-front debug spawning.

The official competition start pose remains unchanged unless
SUPERMARKET_DEBUG_SPAWN=shelf is explicitly set.
"""
from __future__ import annotations

import math
import os
from typing import Any


SHELF_X = {
    "A": (-1.955, -1.735, -1.515),
    "B": (-1.070, -0.850, -0.630),
    "C": (-0.185, 0.035, 0.255),
    "D": (0.700, 0.920, 1.140),
    "E": (1.585, 1.805, 2.025),
}
SHELF_FRONT_Y = 2.475
DEFAULT_YAW = math.pi / 2.0 - math.radians(11.0)
LEVELS = {"L1", "L2", "L3"}
COLUMNS = {"C1": 0, "C2": 1, "C3": 2}


def _truthy(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def apply_debug_spawn(cfg: Any) -> Any:
    mode = os.getenv("SUPERMARKET_DEBUG_SPAWN", "").strip().lower()
    if mode not in {"shelf", "shelf_front"}:
        return cfg

    shelf = os.getenv("SUPERMARKET_DEBUG_SHELF", "E").strip().upper()
    level = os.getenv("SUPERMARKET_DEBUG_LEVEL", "L1").strip().upper()
    column = os.getenv("SUPERMARKET_DEBUG_COLUMN", "C2").strip().upper()
    if shelf not in SHELF_X:
        raise ValueError(f"SUPERMARKET_DEBUG_SHELF must be A-E, got {shelf!r}")
    if level not in LEVELS:
        raise ValueError(f"SUPERMARKET_DEBUG_LEVEL must be L1/L2/L3, got {level!r}")
    if column not in COLUMNS:
        raise ValueError(f"SUPERMARKET_DEBUG_COLUMN must be C1/C2/C3, got {column!r}")

    # Keep a safe gap in front of the shelf; this is a debug observation pose,
    # not a pose inside the shelf and not a replacement for the competition start.
    x = float(SHELF_X[shelf][COLUMNS[column]])
    y = float(os.getenv("SUPERMARKET_DEBUG_SPAWN_Y", str(SHELF_FRONT_Y)))
    yaw = float(os.getenv("SUPERMARKET_DEBUG_SPAWN_YAW", str(DEFAULT_YAW)))
    cfg.init_state["base_position"] = [x, y, 0.0]
    # MuJoCo/Discoverse expects quaternion order [w, x, y, z].
    cfg.init_state["base_orientation"] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
    print(
        f"[server] DEBUG shelf-front spawn enabled: shelf={shelf}/{level}/{column} "
        f"pose=({x:.3f},{y:.3f},0.000,yaw={yaw:.3f}); official start unchanged by default",
        flush=True,
    )
    return cfg


def patch_build_config(module: Any) -> None:
    original = module.build_config

    def wrapped_build_config(*args: Any, **kwargs: Any) -> Any:
        return apply_debug_spawn(original(*args, **kwargs))

    module.build_config = wrapped_build_config
