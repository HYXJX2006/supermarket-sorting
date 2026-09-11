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



def _remove_named_body(xml_text: str, body_name: str) -> str:
    """Remove one nested MJCF body by balanced body-tag scanning.

    This is an opt-in runtime test override.  It avoids rewriting the official
    source MJCF and does not touch shelf/product/delivery-table bodies.
    """
    import re

    match = re.search(r'<body\b[^>]*\bname="' + re.escape(body_name) + r'"[^>]*>', xml_text)
    if match is None:
        raise RuntimeError(f"runtime MJCF body not found: {body_name}")
    token_re = re.compile(r'<body\b[^>]*>|</body\s*>')
    depth = 0
    end = None
    for token in token_re.finditer(xml_text, match.start()):
        if token.group(0).startswith('</body'):
            depth -= 1
            if depth == 0:
                end = token.end()
                break
        else:
            depth += 1
    if end is None:
        raise RuntimeError(f"unterminated MJCF body: {body_name}")
    return xml_text[:match.start()] + xml_text[end:]


def apply_camera_performance_overrides(cfg: Any) -> Any:
    """Optionally render only the head camera for the competition detector."""
    if _truthy(os.getenv("SUPERMARKET_GS_HEAD_ONLY", "1")):
        # The detector and RGB-D geometry consume the head camera. Avoid
        # serializing left/right arm-camera GS renders in the physics thread.
        cfg.obs_rgb_cam_id = [0]
        cfg.obs_depth_cam_id = [0]
        print("[server] 3DGS camera mode: head-only", flush=True)
    return cfg


def apply_render_performance_overrides(cfg: Any) -> Any:
    """Apply opt-in 3DGS render performance settings for the runtime only."""
    if _truthy(os.getenv("SUPERMARKET_GS_BATCH_RENDER", "1")):
        # Preserve all camera outputs but avoid serializing head/left/right
        # gsplat renders in the simulator render loop.
        if hasattr(cfg, "gs_render_sequential"):
            cfg.gs_render_sequential = False
        print("[server] 3DGS render mode: batched cameras", flush=True)
    return cfg


def apply_runtime_scene_overrides(cfg: Any) -> Any:
    """Apply explicitly requested runtime-only scene simplification.

    SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES=1 removes the official corridor
    board and five obstacle bodies from the generated runtime XML.  The source
    MJCF and random-layout generator remain unchanged, so this is only for the
    no-obstacle pickup/delivery smoke test.
    """
    if not _truthy(os.getenv("SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES")):
        return cfg

    from pathlib import Path

    runtime_xml = Path(str(cfg.mjcf_file_path))
    text = runtime_xml.read_text(encoding="utf-8")
    text = _remove_named_body(text, "dynamic_obstacle_corridor")
    runtime_xml.write_text(text, encoding="utf-8")

    model_dict = getattr(cfg, "gs_model_dict", None)
    if isinstance(model_dict, dict):
        cfg.gs_model_dict = {
            name: path
            for name, path in model_dict.items()
            if not name.startswith("dynamic_obstacle_box_")
        }
    print(
        "[server] runtime test override: removed dynamic_obstacle_corridor "
        "(board + 5 boxes); source MJCF unchanged",
        flush=True,
    )
    return cfg

def patch_build_config(module: Any) -> None:
    original = module.build_config

    def wrapped_build_config(*args: Any, **kwargs: Any) -> Any:
        cfg = original(*args, **kwargs)
        cfg = apply_render_performance_overrides(cfg)
        cfg = apply_camera_performance_overrides(cfg)
        cfg = apply_runtime_scene_overrides(cfg)
        return apply_debug_spawn(cfg)

    module.build_config = wrapped_build_config
