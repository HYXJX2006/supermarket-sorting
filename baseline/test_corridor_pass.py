#!/usr/bin/env python3
"""障碍区穿越测试（独立，不含抓取）：规划 A* 路线并驱动底盘走完走廊。

用法（client 容器内）：
  python3 test_corridor_pass.py --seed 7
流程：
  1. 用与 server 相同的 obstacle_layout（含靠墙偏置）+ 相同障碍种子复现布局；
  2. 与 executor 相同参数的 A*（膨胀 1.0、中央偏置 0.35、端点钳制）规划路线；
  3. 降采样后逐航点 spawn low_speed_navigator 驱动底盘；
  4. 任一航点失败（超时/无路）→ FAIL；全部到达 → PASS。
输出：PASS/FAIL + 总耗时 + 各段耗时（供 summary 汇总）。
"""
from __future__ import annotations

import argparse
import heapq
import math
import subprocess
import sys
import time
from pathlib import Path

BASELINE = Path("/workspace/baseline")
OBST_MODULE = BASELINE / "official_baseline/examples/supermarket_sorting/obstacle_layout.py"
NAVIGATOR = BASELINE / "low_speed_navigator.py"

sys.path.insert(0, str(OBST_MODULE.parent))
import obstacle_layout as ol  # noqa: E402

ENTRY_WORLD = (-0.50, 1.29)          # 官方走廊入口（executor 同款）
DELIVERY_WORLD = (-1.94, -2.55)      # 配送台站位
ORIGIN_X, ORIGIN_Y = -0.96, -1.01    # 走廊局部坐标系原点（executor 同款）
NAV_INFLATE = 1.0
CENTER_COST = 0.35


def plan_route(obstacle_seed: int) -> list[tuple[float, float]]:
    layout = ol.generate_obstacle_layout(obstacle_seed)
    selected = tuple(
        (float(p[0]), float(p[1]), float(layout.yaws[b]))
        for b, p in layout.positions.items()
    )
    radius = float(ol.ROBOT_CLEARANCE_RADIUS) * NAV_INFLATE
    resolution = float(ol.GRID_RESOLUTION)
    x_min = float(ol.CORRIDOR_X_MIN) + radius
    x_max = float(ol.CORRIDOR_X_MAX) - radius
    y_min = float(ol.CORRIDOR_Y_MIN) + radius
    y_max = float(ol.CORRIDOR_Y_MAX) - radius
    nx = round((x_max - x_min) / resolution) + 1
    ny = round((y_max - y_min) / resolution) + 1

    def to_cell(pt):
        return (round((pt[0] - x_min) / resolution), round((pt[1] - y_min) / resolution))

    def to_point(cell):
        return (x_min + cell[0] * resolution, y_min + cell[1] * resolution)

    def in_bounds(cell):
        return 0 <= cell[0] < nx and 0 <= cell[1] < ny

    def blocked(cell):
        if not in_bounds(cell):
            return True
        x, y = to_point(cell)
        for ox, oy, yaw in selected:
            hx, hy = ol._oriented_half_extents(yaw)
            if abs(x - ox) <= hx + radius and abs(y - oy) <= hy + radius:
                return True
        return False

    def nearest_free(cell):
        cell = (max(0, min(nx - 1, cell[0])), max(0, min(ny - 1, cell[1])))
        if in_bounds(cell) and not blocked(cell):
            return cell
        for dist in range(1, 20):
            for dx in range(-dist, dist + 1):
                for dy in (-dist, dist):
                    cand = (cell[0] + dx, cell[1] + dy)
                    if in_bounds(cand) and not blocked(cand):
                        return cand
            for dy in range(-dist + 1, dist):
                for dx in (-dist, dist):
                    cand = (cell[0] + dx, cell[1] + dy)
                    if in_bounds(cand) and not blocked(cand):
                        return cand
        return None

    center_x = (float(ol.CORRIDOR_X_MAX) + float(ol.CORRIDOR_X_MIN)) / 2.0
    half_w = (float(ol.CORRIDOR_X_MAX) - float(ol.CORRIDOR_X_MIN)) / 2.0

    start = (ENTRY_WORLD[0] - ORIGIN_X, ENTRY_WORLD[1] - ORIGIN_Y)
    goal = (DELIVERY_WORLD[0] - ORIGIN_X, DELIVERY_WORLD[1] - ORIGIN_Y)
    start_cell = nearest_free(to_cell(start))
    goal_cell = nearest_free(to_cell(goal))
    if start_cell is None or goal_cell is None:
        return []

    distance = {start_cell: 0.0}
    parent: dict = {start_cell: None}
    queue = [(0.0, start_cell)]
    while queue:
        _, cell = heapq.heappop(queue)
        if cell == goal_cell:
            break
        for dx, dy, cost in (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        ):
            neighbor = (cell[0] + dx, cell[1] + dy)
            if blocked(neighbor):
                continue
            if dx and dy and (blocked((cell[0] + dx, cell[1])) or blocked((cell[0], cell[1] + dy))):
                continue
            px, py = to_point(neighbor)
            step = cost * resolution + CENTER_COST * resolution * (
                abs(px - center_x) / half_w
            )
            cand_dist = distance[cell] + step
            if cand_dist >= distance.get(neighbor, math.inf):
                continue
            distance[neighbor] = cand_dist
            parent[neighbor] = cell
            heapq.heappush(queue, (cand_dist + math.dist(neighbor, goal_cell) * resolution, neighbor))

    if goal_cell not in parent:
        return []
    # 回溯 + 世界坐标 + 降采样（每 ~0.35m 一个航点）
    cells = []
    node = goal_cell
    while node is not None:
        cells.append(node)
        node = parent[node]
    cells.reverse()
    world_pts = [to_point(c) for c in cells]
    out = [ENTRY_WORLD]
    last = ENTRY_WORLD
    for pt in world_pts:
        wx, wy = pt[0] + ORIGIN_X, pt[1] + ORIGIN_Y
        if math.hypot(wx - last[0], wy - last[1]) >= 0.35:
            out.append((wx, wy))
            last = (wx, wy)
    out.append(DELIVERY_WORLD)
    return out


def drive_waypoint(wx: float, wy: float, yaw: float, timeout: float) -> bool:
    cmd = [
        sys.executable, str(NAVIGATOR),
        "--goal-x", f"{wx:.4f}", "--goal-y", f"{wy:.4f}", "--goal-yaw", f"{yaw:.4f}",
        "--max-speed", "0.8", "--obstacle-distance", "0.45",
        "--avoid-clearance", "0.30", "--goal-tolerance", "0.18",
        "--timeout", str(int(timeout)),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout or "")[-1200:]
        print("NAV_FAIL tail:", tail, flush=True)
    return proc.returncode == 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True, help="布局种子（与 server 的 SEED 一致）")
    parser.add_argument("--obstacle-seed", type=int, default=None,
                        help="障碍种子（默认 SEED+1000003，与 executor 约定一致）")
    parser.add_argument("--wp-timeout", type=float, default=100.0)
    args = parser.parse_args()
    obstacle_seed = args.obstacle_seed if args.obstacle_seed is not None else args.seed + 1000003

    route = plan_route(obstacle_seed)
    if not route:
        print(f"RESULT seed={args.seed} PASS=False reason=route_planning_failed")
        return 1
    print(f"ROUTE seed={args.seed} waypoints={len(route)}", flush=True)

    t0 = time.time()
    # 从出生点先驶到走廊入口（route[0]），再沿 A* 航点穿越到配送台
    for index in range(1, len(route)):
        wx, wy = route[index]
        prev = route[index - 1]
        yaw = math.atan2(wy - prev[1], wx - prev[0])
        if index == len(route) - 1:
            yaw = -math.pi / 2.0  # 配送台朝向
        ok = drive_waypoint(wx, wy, yaw, args.wp_timeout)
        print(f"WP seed={args.seed} {index}/{len(route)-1} ({wx:.2f},{wy:.2f}) ok={ok} t={time.time()-t0:.0f}s", flush=True)
        if not ok:
            print(f"RESULT seed={args.seed} PASS=False reason=waypoint_{index}_failed elapsed={time.time()-t0:.0f}s")
            return 1
    print(f"RESULT seed={args.seed} PASS=True elapsed={time.time()-t0:.0f}s waypoints={len(route)-1}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
