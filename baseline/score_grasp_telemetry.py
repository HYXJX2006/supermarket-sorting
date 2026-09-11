#!/usr/bin/env python3
"""给一次抓取打分，用于逐类商品的最优抓取参数寻优。

判定只用裁判 ground-truth（``/referee/target_state``），不看控制器退出码。

分数设计意图：
  * 失败 → 大负分，但仍保留 ``gripped_ratio`` 信息，便于区分"完全没碰到"
    与"碰到了但没夹稳"。
  * 成功 → 以 1000 为基准，按夹持质量扣分：
      - ``max_tilt_deg``：物体被夹起的倾角，越小说明夹得越正；
      - ``max_xy_shift_m``：物体相对货位的水平位移，越小说明没被推歪；
      - 单指接触帧：夹偏的信号，额外扣分。
目标是选出"能稳定夹住且夹得最正"的那组参数，而不是仅仅"能抬起来"。

用法：
    score_grasp_telemetry.py --dir <遥测目录> --baseline-z <货位原始z> [--json]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


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


def score(directory: Path, baseline_z: float | None = None) -> dict:
    targets: list[dict] = []
    for record in _iter_jsonl(directory / "target_state.jsonl"):
        for item in record.get("targets", []) or []:
            targets.append(item)

    total = len(targets)
    if total == 0:
        return {
            "total_frames": 0, "gripped_frames": 0, "gripped_ratio": 0.0,
            "z_rise_m": None, "max_tilt_deg": None, "max_xy_shift_m": None,
            "single_finger_frames": 0, "success": False, "score": -2000.0,
            "reason": "NO_TELEMETRY",
        }

    gripped = [t for t in targets if t.get("gripped")]
    single = [
        t for t in targets
        if bool((t.get("finger_contacts") or {}).get("rgt_finger_left_link"))
        != bool((t.get("finger_contacts") or {}).get("rgt_finger_right_link"))
    ]

    z_values = [float(t["position_world"][2]) for t in targets if t.get("position_world")]
    z_base = baseline_z if baseline_z is not None else (z_values[0] if z_values else None)
    z_rise = (max(z_values) - z_base) if (z_values and z_base is not None) else None

    tilts = [float(t.get("tilt_deg") or 0.0) for t in targets]
    shifts = [float(t.get("xy_shift_m") or 0.0) for t in targets]
    max_tilt = max(tilts) if tilts else None
    max_shift = max(shifts) if shifts else None

    gripped_ratio = len(gripped) / total
    success = len(gripped) > 0 and (z_rise or 0.0) >= 0.02

    if success:
        # 夹持质量：倾角与水平位移越小越好；单指帧按比例扣分。
        penalty = 0.0
        penalty += (max_tilt or 0.0) * 3.0
        penalty += (max_shift or 0.0) * 1000.0 * 2.0   # m → mm 系数量化
        penalty += (len(single) / total) * 150.0
        penalty += (1.0 - gripped_ratio) * 120.0
        value = round(1000.0 - penalty, 3)
        reason = "SUCCESS"
    else:
        # 未抬升：用接触比例给出区分度（碰到过 > 完全没碰）。
        value = round(-1000.0 + gripped_ratio * 100.0 + (len(single) / total) * 30.0, 3)
        reason = "CONTACT_ONLY" if len(gripped) > 0 else (
            "SINGLE_FINGER" if len(single) > 0 else "NO_CONTACT"
        )

    return {
        "total_frames": total,
        "gripped_frames": len(gripped),
        "gripped_ratio": round(gripped_ratio, 4),
        "z_rise_m": round(z_rise, 5) if z_rise is not None else None,
        "max_tilt_deg": round(max_tilt, 3) if max_tilt is not None else None,
        "max_xy_shift_m": round(max_shift, 5) if max_shift is not None else None,
        "single_finger_frames": len(single),
        "success": success,
        "score": value,
        "reason": reason,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="抓取遥测评分（参数寻优用）")
    parser.add_argument("--dir", required=True)
    parser.add_argument("--baseline-z", type=float, default=None,
                        help="货位原始 z；只采到保持姿态时必填")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    result = score(Path(args.dir), baseline_z=args.baseline_z)
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
        return 0

    print(f"帧数 {result['total_frames']}  双指接触 {result['gripped_frames']} "
          f"({result['gripped_ratio']*100:.1f}%)  单指 {result['single_finger_frames']}")
    print(f"抬升 {result['z_rise_m']} m  倾角 {result['max_tilt_deg']} deg  "
          f"水平位移 {result['max_xy_shift_m']} m")
    print(f"结论 {result['reason']}  得分 {result['score']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
