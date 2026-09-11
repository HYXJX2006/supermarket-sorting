#!/usr/bin/env bash
# 补跑 3 类：heweidao / sanmingzhi / maidong。
#
# 首轮演示这三类 lift 被拒，根因是演示脚本把 CREEP_SPEED 从 0.12 改成 0.08
# （为画面稳）——闭爪接触状态微偏使腱反馈上浮 1~3%，恰好越过按标定值卡的
# lift 门禁（heweidao 0.9868>0.950 / sanmingzhi 0.8806>0.853 / maidong
# 0.8599>0.853）。标定时三类的 feedback 都在门禁内。教训：标定与执行的
# 接近速度必须一致。本脚本恢复默认速度重跑。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
NAV_YAW="1.5707963267948966"
export NO_GPU=0
export HEADLESS=0
export USE_GS=1
export HOLD_SECONDS="${HOLD_SECONDS:-20}"
# CREEP_SPEED 不设 —— 使用 run_item_grasp.sh 默认 0.12（与标定一致）

ITEMS=(
  "heweidao|product_029|D/L1/C2|0.92 3.243 0.5515|0.852"
  "sanmingzhi|product_028|D/L1/C1|0.70 3.243 0.5484|0.632"
  "maidong|product_041|E/L2/C2|1.805 3.243 0.956|1.737"
)

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

PASS=0
for entry in "${ITEMS[@]}"; do
  IFS='|' read -r kind target slot world endx <<< "$entry"
  route="1.92,2.475,$NAV_YAW;${endx},2.475,$NAV_YAW"
  log ">>> [$kind] slot=$slot"

  logf="$ROOT/baseline/debug_data/demo_${kind}.log"
  KIND=$kind TARGET_ID=$target SLOT=$slot WORLD="$world" LABEL="demo_$kind" \
  NAV_ROUTE="$route" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$logf" 2>&1
  rc=$?

  if grep -aq "STAGE lift=success" "$logf" 2>/dev/null; then
    PASS=$((PASS+1)); log "<<< [$kind] ✅ 完成 [累计 $PASS]"
  else
    grep -a -E "ERROR|拒绝" "$logf" | tail -2
    log "<<< [$kind] ⚠️ 未完整 (rc=$rc)"
  fi
done

log "补跑结束：$PASS / 3 类通过（加上首轮 5 类 = 共 $((PASS+5)) / 8）"
