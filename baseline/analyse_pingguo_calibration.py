#!/usr/bin/env python3
"""把 pingguo 标定遥测汇总成可判定的结论。

输入目录需包含 referee_target_monitor.py 产出的
``target_state.jsonl``（首选，含裁判 ground-truth）与 ``joint_states.jsonl``。

判定优先级（严格递减）：
  1. 裁判 ``gripped=True`` 持续帧数  → 双指确实接触物体
  2. 物体 ``position_world`` 的 z 相对首帧上升 ≥ 0.02 m → 真的被抬离货架
  3. 夹爪腱反馈换算的两指开口 ≈ 苹果直径 → 几何上跨住了球体
任何一项为否都要如实报告；不得以控制器退出码代替上述证据。
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean

TENDON_PER_METER = 12.5
GRIPPER_JOINT = "right_arm_eef_gripper_joint"


def _iter_jsonl(path: Path):
    if not path.exists():
        return
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def analyse(directory: Path, baseline_z: float | None = None) -> dict:
    """汇总遥测为判定。

    baseline_z: 货位原始 z（未抬升）。默认用首帧 z 作基线，但若只采集到
    "保持姿态"（动作链已结束、物体已在高位），首帧本身就是抬升后的值，
    此时必须显式传入货位原始 z，否则抬升量会被算成 0。
    """
    result: dict = {"directory": str(directory), "evidence": {}, "verdict": {}}

    # --- 1. 裁判 ground-truth ---
    targets: list[dict] = []
    for record in _iter_jsonl(directory / "target_state.jsonl"):
        for item in record.get("targets", []) or []:
            targets.append(item)

    gripper_series: list[float] = []
    for record in _iter_jsonl(directory / "joint_states.jsonl"):
        names = record.get("name") or []
        positions = record.get("position") or []
        if GRIPPER_JOINT in names:
            index = names.index(GRIPPER_JOINT)
            if index < len(positions):
                gripper_series.append(float(positions[index]))

    z_values = [float(t["position_world"][2]) for t in targets if t.get("position_world")]
    gripped_rows = [t for t in targets if t.get("gripped")]
    contact_left = [t for t in targets if (t.get("finger_contacts") or {}).get("rgt_finger_left_link")]
    contact_right = [t for t in targets if (t.get("finger_contacts") or {}).get("rgt_finger_right_link")]

    z_baseline = baseline_z if baseline_z is not None else (z_values[0] if z_values else None)
    result["evidence"] = {
        "target_state_frames": len(targets),
        "joint_state_frames": len(gripper_series),
        "gripped_frames": len(gripped_rows),
        "left_finger_contact_frames": len(contact_left),
        "right_finger_contact_frames": len(contact_right),
        "object_z_baseline": round(z_baseline, 5) if z_baseline is not None else None,
        "object_z_first": round(z_values[0], 5) if z_values else None,
        "object_z_last": round(z_values[-1], 5) if z_values else None,
        "object_z_min": round(min(z_values), 5) if z_values else None,
        "object_z_max": round(max(z_values), 5) if z_values else None,
        "object_z_rise_m": (
            round(max(z_values) - z_baseline, 5)
            if z_values and z_baseline is not None else None
        ),
        "max_xy_shift_m": round(max((t.get("xy_shift_m") or 0.0) for t in targets), 5) if targets else None,
        "max_tilt_deg": round(max((t.get("tilt_deg") or 0.0) for t in targets), 3) if targets else None,
        "gripper_feedback_min": round(min(gripper_series), 5) if gripper_series else None,
        "gripper_feedback_last": round(gripper_series[-1], 5) if gripper_series else None,
        "gripper_opening_min_mm": round(min(gripper_series) / TENDON_PER_METER * 1000, 2) if gripper_series else None,
        "gripper_opening_last_mm": round(gripper_series[-1] / TENDON_PER_METER * 1000, 2) if gripper_series else None,
    }

    if gripper_series:
        tail = gripper_series[len(gripper_series) // 2:]
        result["evidence"]["gripper_opening_secondhalf_median_mm"] = round(
            mean(tail) / TENDON_PER_METER * 1000, 2
        )

    e = result["evidence"]
    telemetry_ok = e["target_state_frames"] > 0
    grasped = e["gripped_frames"] > 0
    lifted = (e["object_z_rise_m"] or 0.0) >= 0.02
    # 用 .get()：该键仅在 gripper_series 非空时写入。若 joint_states 里没有
    # 夹爪关节（例如只在保持姿态补采集），硬索引会 KeyError 而非给出 None。
    opening = e.get("gripper_opening_secondhalf_median_mm")

    result["verdict"] = {
        "telemetry_available": telemetry_ok,
        "both_fingers_contacted": grasped,
        "object_rose_with_gripper": lifted,
        "measured_grip_width_mm": opening,
        "apple_diameter_mm": 70.0,
        "grip_width_matches_apple": (
            opening is not None and abs(opening - 70.0) <= 8.0
        ),
        "conclusion": (
            "SUCCESS: 双指接触且物体随升降柱抬升"
            if grasped and lifted
            else "CONTACT_ONLY: 双指接触但物体未抬升"
            if grasped
            else "NO_CONTACT: 两指未同时接触物体"
            if telemetry_ok
            else "NO_TELEMETRY: 未收到裁判遥测，无法判定"
        ),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="pingguo 标定遥测判定")
    parser.add_argument("--dir", required=True, help="遥测输出目录（粘滞记录器 --output-dir）")
    parser.add_argument(
        "--baseline-z", type=float, default=None,
        help="货位原始 z（未抬升）。只采集到保持姿态时用它作为抬升基线；"
             "缺省则用首帧 z。苹果 D/L3/C2 为 1.2240。",
    )
    parser.add_argument("--json", action="store_true", help="输出完整 JSON")
    args = parser.parse_args()

    payload = analyse(Path(args.dir), baseline_z=args.baseline_z)
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    e, v = payload["evidence"], payload["verdict"]
    print(f"遥测目录            : {payload['directory']}")
    print(f"target_state 帧数   : {e['target_state_frames']}")
    print(f"joint_states 帧数   : {e['joint_state_frames']}")
    print("-" * 52)
    print(f"双指同时接触帧数    : {e['gripped_frames']}  "
          f"(左指 {e['left_finger_contact_frames']} / 右指 {e['right_finger_contact_frames']})")
    print(f"抬升基线 z          : {e['object_z_baseline']}"
          f"{'  (显式指定)' if args.baseline_z is not None else '  (首帧)'}")
    print(f"物体 z 首帧/末帧    : {e['object_z_first']} -> {e['object_z_last']}")
    print(f"物体 z 最大上升     : {e['object_z_rise_m']} m")
    print(f"物体水平位移最大    : {e['max_xy_shift_m']} m")
    print(f"物体最大倾角        : {e['max_tilt_deg']} deg")
    print(f"夹爪腱反馈 min/last : {e['gripper_feedback_min']} / {e['gripper_feedback_last']}")
    print(f"两指开口 min/last   : {e['gripper_opening_min_mm']} / {e['gripper_opening_last_mm']} mm")
    print(f"两指开口后半段中位  : {e.get('gripper_opening_secondhalf_median_mm')} mm")
    print("-" * 52)
    print(f"结论                : {v['conclusion']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
