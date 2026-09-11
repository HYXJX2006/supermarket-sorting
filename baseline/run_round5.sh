#!/usr/bin/env bash
# 第五轮：sanmingzhi 修正 SURFACE_TO_CENTER_FWD（真正的原因）。
#
# 依据：直接读 MJCF 的 mesh 顶点，推翻了原先"正面是平面所以顶推"的判断。
#
#   <mesh name="sanmingzhi_mesh" vertex="
#       -0.0325 -0.0500 -0.0494
#       -0.0325  0.0500 -0.0494
#       -0.0325  0.0345  0.0494
#        0.0325 -0.0500 -0.0494
#        0.0325  0.0500 -0.0494
#        0.0325  0.0345  0.0494"/>
#
# 三角形截面在 **yz 平面**（x 为拉伸方向，厚度 0.0650）：
#   底边左端 (y=-0.0500, z=-0.0494) → 顶点 (y=+0.0345, z=+0.0494)  ← 斜面 A
#   底边右端 (y=+0.0500, z=-0.0494) → 顶点 (y=+0.0345, z=+0.0494)  ← 斜面 B
#   （顶点偏 y 正侧 0.0345，不是居中）
#
# 机器人沿 +y 接近，遇到斜面 A。夹爪在商品中心高度（z=0）接触处：
#   z 由 -0.0494 → +0.0494，y 由 -0.0500 → +0.0345，线性插值 t=0.5
#   y = -0.0500 + 0.5 × 0.0845 = **-0.00775**
#
# 即真实"表面到中心"的距离只有 **0.00775 m**，而配置里写的是 0.0500
# （当初按 y 向深度 0.1000 的一半估算）—— **放大了 6.5 倍**。
#
# 后果（代入 stop_y = 商品y + creep_stop_dy + surface_to_center_fwd）：
#   stop_y = 3.243 + 0.035 + 0.0500 = 3.328
#   实际接触需要 ≈ 3.243 - 0.00775 ≈ 3.235
#   差距 0.093 m ⇒ 夹爪停在商品前方 9.3 cm 处，根本够不到 ⇒ 一路顶推
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-35}"

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

log "sanmingzhi 扫描 SURFACE_TO_CENTER_FWD（真实值约 0.00775）"
KIND=sanmingzhi \
TARGET_ID=product_028 \
SLOT=D/L1/C1 \
WORLD="0.70 3.243 0.5484" \
PARAM=SURFACE_TO_CENTER_FWD \
VALUES="0.005 0.015 0.030" \
  bash "$ROOT/baseline/run_param_sweep.sh" \
  > "$ROOT/baseline/debug_data/round5_sanmingzhi_surf.log" 2>&1

echo
echo "================================================================"
echo "第五轮结束"
echo "================================================================"
tail -30 "$ROOT/baseline/debug_data/round5_sanmingzhi_surf.log" 2>/dev/null
