#!/usr/bin/env bash
# 最后一轮：zhijin 拨动 v2 + maidong 补测。
#
# zhijin v2 改进（相对 v1）：
#   1. 拨动偏移 x +0.06 → +0.10（力臂更长，转动力矩更大）
#   2. CREEP_SPEED 0.12 → 0.03（慢速推转，避免把纸巾推跑而非推转）
#   3. 连续拨两次（每次一轮 deploy+creep，夹爪保持张开）
#   4. 之后抓取轮用真实位置完整链
#
# maidong：几何与 shupian 完全相同（cylinder 0.0650×0.0650×0.2100），
# 但质量 0.643 kg（shupian 的 4.7 倍）。选 E/L2/C2（x=1.805），
# 离起点 (1.92,-3.17) 最近，导航路线最短。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
NAV_YAW="1.5707963267948966"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-30}"
export CREEP_SPEED="${CREEP_SPEED:-0.03}"

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

# ---------- zhijin 拨动 v2 ----------
log "zhijin 拨动 v2：第 1 次拨动（偏移 +0.10）"
STOP_AFTER=creep \
KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
WORLD="0.80 3.243 0.895" LABEL=zhijin_n2a \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/zhijin_n2a.log" 2>&1
grep -a "CREEP 停止" "$ROOT/baseline/debug_data/zhijin_n2a.log" | tail -1

log "zhijin 拨动 v2：第 2 次拨动（同偏移，继续转）"
STOP_AFTER=creep \
KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
WORLD="0.80 3.243 0.895" LABEL=zhijin_n2b \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/zhijin_n2b.log" 2>&1
grep -a "CREEP 停止" "$ROOT/baseline/debug_data/zhijin_n2b.log" | tail -1

log "zhijin 抓取轮（真实位置完整链）"
KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
WORLD="0.70 3.243 0.895" LABEL=zhijin_g2 \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/zhijin_g2.log" 2>&1

TG=$(grep -a '遥测目录: ' "$ROOT/baseline/debug_data/zhijin_g2.log" | tail -1 | sed 's/.*遥测目录: //')
if [[ -n "$TG" && -d "$TG" ]]; then
  echo "--- zhijin 裁判真值 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=${TG/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=0.895" 2>&1 | tail -4
else
  echo "zhijin 抓取轮无遥测"
fi

# ---------- maidong 补测 ----------
log "maidong 补测（E/L2/C2，几何同 shupian，质量 0.643 kg）"
KIND=maidong TARGET_ID=product_041 SLOT=E/L2/C2 \
WORLD="1.805 3.243 0.956" LABEL=maidong \
NAV_ROUTE="1.92,2.475,$NAV_YAW;1.737,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/maidong_test.log" 2>&1

TM=$(grep -a '遥测目录: ' "$ROOT/baseline/debug_data/maidong_test.log" | tail -1 | sed 's/.*遥测目录: //')
if [[ -n "$TM" && -d "$TM" ]]; then
  echo "--- maidong 裁判真值 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=${TM/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=0.956" 2>&1 | tail -4
else
  echo "maidong 无遥测"
fi

echo
echo "================================================================"
echo "最后一轮结束"
echo "================================================================"
