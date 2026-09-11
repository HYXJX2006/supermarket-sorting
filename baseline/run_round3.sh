#!/usr/bin/env bash
# 第三轮：sanmingzhi 换参数方向 —— 扫描 CREEP_STOP_DY（前伸停止线）。
#
# 依据（从日志反推出的关系）：
#     stop_y = 商品y + creep_stop_dy + surface_to_center_fwd
#   sanmingzhi: 3.243 + 0.035 + 0.0500 = 3.328   ✓ 与日志一致
#   heweidao:   3.243 + 0.035 + 0.0400 = 3.318   ✓ 与日志一致
#
# sanmingzhi 的 surface_to_center_fwd = 0.0500（y 深 0.1000，九类里最深），
# 比通用 creep_stop_dy=0.035 还大 —— 夹爪停在商品表面之外，形成"夹空"
# （实测开口 39.9 mm 远小于商品 x 维 65 mm，双指之间没有商品）。
#
# 故减小 creep_stop_dy，让末端更靠近商品中心，指头得以真正跨到商品两侧。
#
# 第二轮 DEPLOY_DZ 扫描已证伪下探方向：
#   -0.060 → NO_CONTACT，-0.035 → SINGLE_FINGER，均比基线更差。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-35}"

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

log "sanmingzhi 扫描 CREEP_STOP_DY（前伸停止线，减小=更靠近商品）"
KIND=sanmingzhi \
TARGET_ID=product_028 \
SLOT=D/L1/C1 \
WORLD="0.70 3.243 0.5484" \
PARAM=CREEP_STOP_DY \
VALUES="0.025 0.010 -0.005" \
  bash "$ROOT/baseline/run_param_sweep.sh" \
  > "$ROOT/baseline/debug_data/round3_sanmingzhi_creep.log" 2>&1

echo
echo "================================================================"
echo "第三轮结束"
echo "================================================================"
tail -30 "$ROOT/baseline/debug_data/round3_sanmingzhi_creep.log" 2>/dev/null
