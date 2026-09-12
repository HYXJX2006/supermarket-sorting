#!/usr/bin/env python3
"""随机商品比赛的45货位库存地图。

ArUco 不是硬依赖：默认用 YOLO 的稳定 RGB-D 世界坐标吸附到固定货架几何；
ArUco 若可用，只作为外部校验来源。商品类别绝不从 product 编号推断。
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

SHELVES = ("A", "B", "C", "D", "E")
SHELF_ORDER_FOR_ROUTE = ("E", "D", "C", "B", "A")
SLOT_X = {
    "A": (-1.955, -1.735, -1.515),
    "B": (-1.070, -0.850, -0.630),
    "C": (-0.185, 0.035, 0.255),
    "D": (0.700, 0.920, 1.140),
    "E": (1.585, 1.805, 2.025),
}
SHELF_Y = 3.243
LEVEL_Z = {1: 0.56, 2: 0.91, 3: 1.24}

@dataclass
class InventoryEntry:
    aruco_id: int
    slot: str
    shelf: str
    level: int
    column: int
    kind: str | None = None
    world: list[float] | None = None
    confidence: float = 0.0
    samples: int = 0
    state: str = "unknown"
    target_id: str | None = None
    source: str | None = None
    updated_at: float = 0.0
    observations: int = 0
    conflicts: int = 0
    rechecks: int = 0

class InventoryMap:
    def __init__(self, *, x_tolerance: float = 0.14, y_tolerance: float = 0.18, z_tolerance: float = 0.18) -> None:
        self.x_tolerance = float(x_tolerance)
        self.y_tolerance = float(y_tolerance)
        self.z_tolerance = float(z_tolerance)
        self.entries: dict[str, InventoryEntry] = {}
        for shelf_index, shelf in enumerate(SHELVES):
            for level in (1, 2, 3):
                for column in (1, 2, 3):
                    aruco_id = shelf_index * 9 + (level - 1) * 3 + (column - 1)
                    slot = f"{shelf}/L{level}/C{column}"
                    self.entries[slot] = InventoryEntry(aruco_id, slot, shelf, level, column)
        self.run_prefix = ""
        self.started_at = time.time()

    @staticmethod
    def slot_from_world(world: tuple[float, float, float], *, x_tolerance: float = 0.14, y_tolerance: float = 0.18, z_tolerance: float = 0.18) -> str | None:
        wx, wy, wz = (float(v) for v in world)
        if not all(math.isfinite(v) for v in (wx, wy, wz)):
            return None
        best: tuple[float, str] | None = None
        for shelf in SHELVES:
            for column, sx in enumerate(SLOT_X[shelf], start=1):
                dx, dy = wx - sx, wy - SHELF_Y
                if abs(dx) > x_tolerance or abs(dy) > y_tolerance:
                    continue
                for level, sz in LEVEL_Z.items():
                    dz = wz - sz
                    if abs(dz) > z_tolerance:
                        continue
                    score = math.hypot(dx, dy) + 0.45 * abs(dz)
                    slot = f"{shelf}/L{level}/C{column}"
                    if best is None or score < best[0]:
                        best = (score, slot)
        return None if best is None else best[1]

    def observe(self, *, kind: str, world: tuple[float, float, float], confidence: float, samples: int, source: str = "rgbd_geometry") -> InventoryEntry | None:
        slot = self.slot_from_world(world, x_tolerance=self.x_tolerance, y_tolerance=self.y_tolerance, z_tolerance=self.z_tolerance)
        if slot is None:
            return None
        entry = self.entries[slot]
        confidence = max(0.0, min(1.0, float(confidence)))
        samples = max(0, int(samples))
        incoming_kind = str(kind).strip()
        entry.observations += 1
        if entry.kind is not None and entry.kind != incoming_kind:
            # 已经预订/取走的货位不能被后续误检覆盖；未锁定货位也要求
            # 新类别明显更可信，避免相邻商品框把一个槽位来回改写。
            entry.conflicts += 1
            if entry.state != "unknown" or confidence < entry.confidence + 0.12:
                return entry
        if entry.kind is None or incoming_kind != entry.kind or confidence >= entry.confidence:
            entry.kind = incoming_kind
            # 已预订/取放中的条目坐标冻结：world 是抓取目标，抓前近距复核
            # （executor._refine_target_before_ik）刚校正过的坐标若被后续
            # 检测流覆盖，机械臂就会抓向旧位置（2026-09-12 实测复核修正
            # 6cm 后被冲回 -1.9cm，闭爪落空）。
            if entry.state not in ("reserved", "picking", "picked"):
                entry.world = [round(float(v), 5) for v in world]
            entry.confidence = confidence
            entry.samples = max(entry.samples, samples)
            entry.source = source
            entry.updated_at = time.time()
            if entry.state == "unknown":
                entry.state = "available"
            if source.startswith("local_recheck"):
                entry.rechecks += 1
        return entry

    def candidates(self, kind: str, *, states: tuple[str, ...] = ("available",)) -> list[InventoryEntry]:
        wanted = str(kind).strip()
        return sorted(
            [e for e in self.entries.values() if e.kind == wanted and e.state in states and e.world is not None],
            key=lambda e: (SHELF_ORDER_FOR_ROUTE.index(e.shelf), -e.confidence, -e.samples, e.level, e.column),
        )

    def reserve(self, slot: str, target_id: str) -> InventoryEntry:
        entry = self.entries[slot]
        if entry.state not in {"available", "reserved"}:
            raise ValueError(f"slot {slot} is not available: {entry.state}")
        entry.state = "reserved"
        entry.target_id = str(target_id)
        entry.updated_at = time.time()
        return entry

    def mark(self, slot: str, state: str, target_id: str | None = None) -> InventoryEntry:
        if state not in {"available", "reserved", "picking", "picked", "placed", "invalid"}:
            raise ValueError(f"invalid inventory state: {state}")
        entry = self.entries[slot]
        entry.state = state
        if target_id is not None:
            entry.target_id = str(target_id)
        entry.updated_at = time.time()
        return entry

    def release(self, slot: str, target_id: str | None = None) -> InventoryEntry:
        """释放一次局部复核失败的预订，不改变商品类别和世界坐标。"""
        entry = self.entries[slot]
        owner = str(target_id) if target_id is not None else None
        if entry.state != "reserved":
            raise ValueError(f"slot {slot} is not reserved: {entry.state}")
        if owner is not None and entry.target_id not in {None, owner}:
            raise ValueError(f"slot {slot} is reserved by another target: {entry.target_id}")
        entry.state = "available"
        entry.target_id = None
        entry.updated_at = time.time()
        return entry

    def required_targets_ready(self, targets: list[dict[str, Any]]) -> bool:
        counts: dict[str, int] = {}
        for target in targets:
            kind = str(target.get("kind", "")).strip()
            counts[kind] = counts.get(kind, 0) + 1
        return all(len(self.candidates(kind)) >= count for kind, count in counts.items())

    def to_dict(self) -> dict[str, Any]:
        observed = [e for e in self.entries.values() if e.kind is not None]
        return {
            "schema_version": 1,
            "run_prefix": self.run_prefix,
            "started_at": self.started_at,
            "updated_at": time.time(),
            "slot_count": len(self.entries),
            "observed_count": len(observed),
            "states": {state: sum(e.state == state for e in self.entries.values()) for state in ("unknown", "available", "reserved", "picking", "picked", "placed", "invalid")},
            "entries": [asdict(self.entries[slot]) for slot in sorted(self.entries)],
        }

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

if __name__ == "__main__":
    m = InventoryMap()
    assert len(m.entries) == 45
    assert InventoryMap.slot_from_world((-1.735, 3.243, 1.24)) == "A/L3/C2"
    assert InventoryMap.slot_from_world((0.92, 3.243, 0.91)) == "D/L2/C2"
    e = m.observe(kind="shupian", world=(1.805, 3.243, 1.24), confidence=.9, samples=7)
    assert e and e.slot == "E/L3/C2"
    assert len(m.candidates("shupian")) == 1
    print("inventory_map self-test: OK")
