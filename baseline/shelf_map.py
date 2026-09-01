#!/usr/bin/env python3
"""智慧零售货架货位数据结构。

规则约束：ArUco ID 只固定表示货位；商品 kind 与货位的关系必须通过当局视觉扫描得到。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Optional


SHELF_NAMES = ("A", "B", "C", "D", "E")
LEVEL_COUNT = 3
COLUMN_COUNT = 3
SLOTS_PER_SHELF = LEVEL_COUNT * COLUMN_COUNT
TOTAL_SLOTS = len(SHELF_NAMES) * SLOTS_PER_SHELF


@dataclass
class ShelfSlot:
    aruco_id: int
    shelf: str
    level: int
    column: int
    kind: Optional[str] = None
    confidence: float = 0.0
    position_world: Optional[tuple[float, float, float]] = None
    last_seen_source: Optional[str] = None

    @property
    def label(self) -> str:
        return f"{self.shelf}/L{self.level}/C{self.column}"

    @property
    def observed(self) -> bool:
        return self.kind is not None

    def to_dict(self) -> dict:
        data = asdict(self)
        data["label"] = self.label
        data["observed"] = self.observed
        return data


def slot_from_aruco(aruco_id: int) -> ShelfSlot:
    """根据固定规则把 ArUco ID 转换为货位，不推断商品种类。"""
    if not isinstance(aruco_id, int) or not 0 <= aruco_id < TOTAL_SLOTS:
        raise ValueError(f"ArUco ID 必须在 0..{TOTAL_SLOTS - 1}: {aruco_id!r}")

    shelf_index, within_shelf = divmod(aruco_id, SLOTS_PER_SHELF)
    level, column_zero = divmod(within_shelf, COLUMN_COUNT)
    return ShelfSlot(
        aruco_id=aruco_id,
        shelf=SHELF_NAMES[shelf_index],
        level=level + 1,
        column=column_zero + 1,
    )


class ShelfMap:
    """保存本局已经观察到的 kind -> 货位关系。"""

    def __init__(self) -> None:
        self.slots: dict[int, ShelfSlot] = {
            aruco_id: slot_from_aruco(aruco_id)
            for aruco_id in range(TOTAL_SLOTS)
        }

    def observe(
        self,
        aruco_id: int,
        kind: str,
        *,
        confidence: float = 1.0,
        position_world: Optional[tuple[float, float, float]] = None,
        source: Optional[str] = None,
    ) -> ShelfSlot:
        """记录一次视觉观察；不会把商品 kind 写死到某个 ArUco。"""
        if not isinstance(kind, str) or not kind.strip():
            raise ValueError("商品 kind 不能为空")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f"confidence 必须在 0..1: {confidence}")
        if position_world is not None and len(position_world) != 3:
            raise ValueError("position_world 必须是长度为3的坐标")

        slot = self.slots[aruco_id]
        # 低置信度的新观察不覆盖更高置信度的旧观察。
        if slot.kind is None or confidence >= slot.confidence:
            slot.kind = kind.strip()
            slot.confidence = float(confidence)
            slot.position_world = position_world
            slot.last_seen_source = source
        return slot

    def candidates_for_kind(self, kind: str) -> list[ShelfSlot]:
        """返回当前已观察到的同类商品货位，按置信度降序。"""
        return sorted(
            (slot for slot in self.slots.values() if slot.kind == kind),
            key=lambda slot: slot.confidence,
            reverse=True,
        )

    def unobserved_slots(self) -> list[ShelfSlot]:
        return [slot for slot in self.slots.values() if not slot.observed]

    def observed_kinds(self) -> set[str]:
        return {slot.kind for slot in self.slots.values() if slot.kind is not None}

    def to_dict(self) -> dict:
        return {
            "slot_count": len(self.slots),
            "observed_kinds": sorted(self.observed_kinds()),
            "slots": [self.slots[i].to_dict() for i in sorted(self.slots)],
        }


def self_test() -> None:
    shelf_map = ShelfMap()
    assert len(shelf_map.slots) == 45
    assert shelf_map.slots[0].label == "A/L1/C1"
    assert shelf_map.slots[8].label == "A/L3/C3"
    assert shelf_map.slots[9].label == "B/L1/C1"
    assert shelf_map.slots[35].label == "D/L3/C3"
    assert shelf_map.slots[44].label == "E/L3/C3"

    shelf_map.observe(31, "kele", confidence=0.9, source="head_rgb")
    shelf_map.observe(7, "pingguo", confidence=0.8, source="head_rgb")
    assert shelf_map.candidates_for_kind("kele")[0].label == "D/L2/C2"
    assert len(shelf_map.unobserved_slots()) == 43
    assert shelf_map.slots[31].to_dict()["observed"] is True

    # 低置信度结果不能覆盖已经确认的高置信度结果。
    shelf_map.observe(31, "pingguo", confidence=0.2)
    assert shelf_map.slots[31].kind == "kele"


if __name__ == "__main__":
    self_test()
    print("shelf_map self-test: OK")
