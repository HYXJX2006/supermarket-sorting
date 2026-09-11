#!/usr/bin/env python3
"""ROS2 bridge for the official MuJoCo Referee.

The official ``Referee`` class owns scoring semantics. This module only exposes
its state over the competition ROS topics and serializes reset requests so the
simulation thread performs the actual reset safely.
"""
from __future__ import annotations

import json
import math
import threading
from typing import Any, Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


TASK_TOPIC = "/referee/taskinfo"
GAME_TOPIC = "/referee/gameinfo"
SCORE_TOPIC = "/referee/score"
TARGET_STATE_TOPIC = "/referee/target_state"
RESET_SERVICE = "/supermarket_sorting/reset_run"


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def build_task_info(config: Any, time_limit_s: float) -> dict[str, Any]:
    """Build the stable task snapshot consumed by monitoring clients."""
    targets = list(getattr(config, "task_targets", []) or [])
    return {
        "schema_version": 1,
        "run_prefix": str(getattr(config, "task_run_prefix", "")),
        "count": len(targets),
        "time_limit_s": float(time_limit_s),
        "targets": targets,
    }


def _rounded_vec3(value: Any) -> list[float] | None:
    """Convert a MuJoCo vector to JSON-safe coordinates."""

    if value is None:
        return None
    try:
        values = list(value)
        if len(values) < 3:
            return None
        return [round(float(values[index]), 6) for index in range(3)]
    except (TypeError, ValueError):
        return None


def build_target_telemetry(referee: Any, mj_data: Any) -> dict[str, Any]:
    """Build read-only per-target physical evidence from MuJoCo state.

    This deliberately reuses the official referee's contact and motion helpers;
    it does not alter scoring or infer success from a controller exit code.
    """

    flow = getattr(referee, "flow", None)
    flow_target = getattr(flow, "target", None) if flow is not None else None
    flow_step = int(getattr(flow, "step", 0) if flow is not None else 0)
    try:
        contact_pairs = referee._contact_pairs(mj_data)
    except Exception:
        contact_pairs = set()

    targets: list[dict[str, Any]] = []
    for body_name in list(getattr(referee, "targets", []) or []):
        row: dict[str, Any] = {
            "body": str(body_name),
            "is_flow_target": body_name == flow_target,
            "flow_step": flow_step,
            "position_world": None,
            "initial_position_world": _rounded_vec3(getattr(referee, "init_pos", {}).get(body_name)),
            "xy_shift_m": None,
            "speed_m_s": None,
            "tilt_deg": None,
            "orientation_yaw_deg": None,
            "gripped": False,
            "finger_contacts": {},
        }
        try:
            position = mj_data.body(body_name).xpos
            row["position_world"] = _rounded_vec3(position)
            # 绕 z 轴的旋转角（yaw）。拨动改朝向类策略必须有这个量——
            # 否则只能靠"下一轮能不能夹住"间接猜商品转没转。
            # xquat = [w, x, y, z]；yaw = atan2(2(wz+xy), 1-2(y²+z²))
            xq = mj_data.body(body_name).xquat
            w, x, y, z = (float(v) for v in xq)
            yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
            row["orientation_yaw_deg"] = round(yaw, 3)
            initial = getattr(referee, "init_pos", {}).get(body_name)
            if initial is not None:
                dx = float(position[0]) - float(initial[0])
                dy = float(position[1]) - float(initial[1])
                row["xy_shift_m"] = round(math.hypot(dx, dy), 6)
            row["speed_m_s"] = round(float(referee._speed(mj_data, body_name)), 6)
            row["tilt_deg"] = round(float(referee._tilt_deg(mj_data, body_name)), 6)
            fingers = list(referee.cfg.get("grasp_fingers", []) or [])
            row["finger_contacts"] = {
                str(finger): bool(referee._touch(contact_pairs, [finger], body_name))
                for finger in fingers
            }
            row["gripped"] = bool(referee._gripped(contact_pairs, body_name))
        except Exception as exc:
            row["telemetry_error"] = f"{type(exc).__name__}: {exc}"
        targets.append(row)

    return {
        "schema_version": 1,
        "sim_time_s": round(float(getattr(mj_data, "time", 0.0)), 6),
        "flow_step": flow_step,
        "current_target": flow_target,
        "targets": targets,
    }


def build_game_info(referee: Any, sim_time: float, mj_data: Any | None = None) -> dict[str, Any]:
    """Build a JSON-compatible live score snapshot from official Referee state."""
    flow = getattr(referee, "flow", None)
    result = referee.result_dict()
    payload = {
        "schema_version": 1,
        "sim_time_s": float(sim_time),
        "game_info": str(referee.game_info),
        "total_score": int(referee.total_score),
        "completed": int(referee.completed_count),
        "target_count": int(len(getattr(referee, "targets", []) or [])),
        "flow_step": int(getattr(flow, "step", 0) if flow is not None else 0),
        "current_target": getattr(flow, "target", None) if flow is not None else None,
        "finished": bool(getattr(referee, "finished", False)),
        "records": result.get("flows", []),
    }
    if mj_data is not None:
        payload["target_telemetry"] = build_target_telemetry(referee, mj_data)
    return payload


class RefereeBridge(Node):
    """Publish official referee state and queue safe simulation resets."""

    def __init__(
        self,
        referee: Any,
        config: Any,
        reset_request: threading.Event,
        on_reset_ack: Callable[[], None] | None = None,
    ) -> None:
        super().__init__("supermarket_referee")
        self.referee = referee
        self.config = config
        self.reset_request = reset_request
        self.on_reset_ack = on_reset_ack
        qos = _latched_qos()
        self.task_pub = self.create_publisher(String, TASK_TOPIC, qos)
        self.game_pub = self.create_publisher(String, GAME_TOPIC, 10)
        self.score_pub = self.create_publisher(Int32, SCORE_TOPIC, 10)
        self.target_state_pub = self.create_publisher(String, TARGET_STATE_TOPIC, 10)
        self.create_service(Trigger, RESET_SERVICE, self._on_reset)
        self.task_info = build_task_info(config, referee.cfg["time_limit_s"])
        self.publish_task_info()

    def publish_task_info(self) -> None:
        self.task_pub.publish(
            String(data=json.dumps(self.task_info, ensure_ascii=False, separators=(",", ":")))
        )

    def publish_state(self, sim_time: float, mj_data: Any | None = None) -> None:
        payload = build_game_info(self.referee, sim_time, mj_data)
        self.game_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        self.score_pub.publish(Int32(data=int(self.referee.total_score)))
        if mj_data is not None:
            telemetry = payload.get("target_telemetry", {})
            self.target_state_pub.publish(
                String(data=json.dumps(telemetry, ensure_ascii=False, separators=(",", ":")))
            )

    def acknowledge_reset(self, mj_data: Any | None = None) -> None:
        """Publish the fresh task/state snapshot after the sim thread resets."""
        self.publish_task_info()
        self.publish_state(0.0, mj_data)
        if self.on_reset_ack is not None:
            self.on_reset_ack()

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self.reset_request.is_set():
            response.success = False
            response.message = "reset already pending"
            return response
        self.reset_request.set()
        response.success = True
        response.message = "reset accepted; applied at next simulation tick"
        return response
