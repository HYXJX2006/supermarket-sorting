#!/usr/bin/env bash
# zhijin 拨动改朝向策略（主人指定方案）。
#
# 问题：zhijin 是 box (0.1720, 0.0850, 0.0880)，approach 沿 +y 时
# 两指需跨 x 维 0.1720，而夹爪最大开口 0.0800 —— 物理不可能。
# 实测：过行程硬顶（开口 -13.3 mm），把商品推走 11.8 cm。
#
# 策略（主人指定）：先拨动纸巾改变朝向。
#   拨动轮：夹爪张开，对准商品「右半部」（x 偏 +0.06）前进，
#           指头扫过商品右前角 → 产生绕 z 力矩 → 商品旋转让窄边朝前。
#   抓取轮：对准真实位置完整抓取。此时横向宽度从 0.1720 变为 0.0850，
#           超出夹爪开口仅 5 mm，配合指头 mesh 齿面与摩擦（4.0）有夹住机会。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
NAV_YAW="1.5707963267948966"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-30}"

# zhijin 参数：D/L2/C1, product_031, world=(0.70 3.243 0.895)
# 拨动偏移：x +0.06（让指头扫过商品右半部，产生绕 z 的旋转力矩）
NUDGE_WORLD="0.76 3.243 0.895"
REAL_WORLD="0.70 3.243 0.895"

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

log "步骤 1/2：拨动轮（STOP_AFTER=creep，夹爪张开扫商品右前角）"
STOP_AFTER=creep \
KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
WORLD="$NUDGE_WORLD" LABEL=zhijin_nudge \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/zhijin_nudge.log" 2>&1
grep -a -E "STAGE|CREEP 停止" "$ROOT/baseline/debug_data/zhijin_nudge.log" | tail -4

log "步骤 2/2：抓取轮（真实位置完整链）"
KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
WORLD="$REAL_WORLD" LABEL=zhijin_grab \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/zhijin_grab.log" 2>&1

# 判定：只看裁判真值
T=$(grep -a '遥测目录: ' "$ROOT/baseline/debug_data/zhijin_grab.log" | tail -1 | sed 's/.*遥测目录: //')
if [[ -n "$T" && -d "$T" ]]; then
  echo "--- zhijin 裁判真值 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/analyse_pingguo_calibration.py --dir=${T/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=0.895" 2>&1 | tail -12
  echo "--- 评分 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=${T/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=0.895" 2>&1 | tail -4
else
  echo "zhijin 抓取轮无遥测"
fi
