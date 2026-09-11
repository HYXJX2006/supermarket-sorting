#!/usr/bin/env bash
# zhijin 拨动 v3：单会话临界擦角 + 姿态闭环。
#
# v3 的结构性修正：**拨动与抓取必须在同一个 Server 会话内连续完成**。
# 此前每轮 run_item_grasp.sh 都重启 Server，SUPERMARKET_FIXED_BASELINE=1 会
# 重置场景——商品朝向回到初始值，拨动成果全部丢失。遥测实证：v2/v3 每轮
# 首帧 yaw 都是 0.000，三次拨动互不累积。
#
# 本版流程（单 Server）：
#   循环拨动（deploy+creep，夹爪张开，STOP_AFTER=creep）：
#     - 擦角点跟随商品：擦角 x = 商品当前 x + 右缘半宽(θ) + 擦缘间隙
#     - 每轮读该轮遥测的 orientation_yaw_deg 与 position_world
#     - |yaw| ≥ 85° 即停止拨动（85° 时横向投影 = 短边 85 mm，最小）
#   抓取轮（同 Server）：
#     - 位置 = 拨动后商品实际位置
#     - surface_to_center_fwd = 0.0860（转 90° 后 y 深 0.1720 的一半）
#     - grip_feedback_max = 1.2（两指咬 85 mm 必然过行程，门禁放行）
#     - TOLERATE_CREEP_TIMEOUT=1（商品横在指间，stop 线算不准超时是预期的）
#
# 矩形旋转投影：横向宽 = 172·|cosθ| + 85·|sinθ|，θ=0 → 172，θ=90 → 85。
# 中途（26° 附近）投影最大 ≈192，**转到位才有意义**。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
NAV_YAW="1.5707963267948966"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-15}"
export CREEP_SPEED="${CREEP_SPEED:-0.03}"

SERVER_NAME="zhijin_v3_server"
EDGE_GAP="${EDGE_GAP:-0.006}"        # 指头面停在商品右缘外的距离
MAX_NUDGES="${MAX_NUDGES:-6}"
YAW_TARGET="${YAW_TARGET:-85}"

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

# 商品当前状态（初始：原始货位 D/L2/C1，yaw=0）
PX=0.70; PY=3.243; PZ=0.895
YAW=0.0

i=0
while (( i < MAX_NUDGES )); do
  i=$((i + 1))
  if (( i == 1 )); then REUSE=0; else REUSE=1; fi

  # 擦角点：商品右缘 + gap（右缘半宽随当前 yaw 变化）
  NUDGE_WORLD=$(python3 -c "
import math
half = (0.1720*abs(math.cos(math.radians($YAW))) + 0.0850*abs(math.sin(math.radians($YAW))))/2
print(f'{$PX + half + $EDGE_GAP:.4f} {$PY:.4f} {$PZ:.4f}')
")
  log "拨动 $i/$MAX_NUDGES：yaw=$YAW°  商品=($PX,$PY,$PZ)  擦角点=$NUDGE_WORLD"

  REUSE_SERVER=$REUSE SERVER_NAME=$SERVER_NAME \
  KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
  WORLD="$NUDGE_WORLD" LABEL="zhijin_v3_n$i" \
  NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
    bash "$ROOT/baseline/run_item_grasp.sh" \
    > "$ROOT/baseline/debug_data/zhijin_v3_n$i.log" 2>&1

  TDIR=$(grep -a '遥测目录: ' "$ROOT/baseline/debug_data/zhijin_v3_n$i.log" | tail -1 | sed 's/.*遥测目录: //')
  if [[ -z "$TDIR" || ! -d "$TDIR" ]]; then
    log "第 $i 次拨动无遥测，重试下一轮"
    continue
  fi
  read -r YAW PX PY PZ <<< "$(python3 - "$TDIR/target_state.jsonl" <<PY
import json, sys
rows = []
for line in open(sys.argv[1], encoding="utf-8"):
    line = line.strip()
    if not line: continue
    try: d = json.loads(line)
    except: continue
    for t in d.get("targets", []):
        y = t.get("orientation_yaw_deg")
        pw = t.get("position_world")
        if y is None or pw is None: continue
        rows.append((y, pw[0], pw[1], pw[2]))
if rows:
    print(f"{rows[-1][0]:.3f} {rows[-1][1]:.4f} {rows[-1][2]:.4f} {rows[-1][3]:.4f}")
PY
)"
  if [[ -z "$YAW" ]]; then
    log "第 $i 次拨动遥测无 yaw，重试下一轮"
    continue
  fi
  log "拨动 $i 后：yaw=$YAW°  商品=($PX,$PY,$PZ)"
  if python3 -c "import sys;sys.exit(0 if abs($YAW) >= float('$YAW_TARGET') else 1)" 2>/dev/null; then
    log "旋转达标（|yaw|≥$YAW_TARGET°），进入抓取"
    break
  fi
done

# ---------- 抓取轮（同 Server，完整链）----------
log "抓取轮：位置=($PX,$PY,$PZ)  yaw=$YAW°  surf=0.0860  门禁=1.2"
TOLERATE_CREEP_TIMEOUT=1 \
SUPERMARKET_ZHIJIN_SURFACE_TO_CENTER_FWD=0.0860 \
SUPERMARKET_ZHIJIN_GRIP_FEEDBACK_MAX=1.2 \
REUSE_SERVER=1 SERVER_NAME=$SERVER_NAME \
KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
WORLD="$PX $PY $PZ" LABEL=zhijin_v3_grab \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/zhijin_v3_grab.log" 2>&1

TG=$(grep -a '遥测目录: ' "$ROOT/baseline/debug_data/zhijin_v3_grab.log" | tail -1 | sed 's/.*遥测目录: //')
if [[ -n "$TG" && -d "$TG" ]]; then
  echo "--- zhijin v3 抓取轮裁判真值 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=${TG/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=$PZ" 2>&1 | tail -4
  echo "--- 朝向变化 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/analyse_pingguo_calibration.py --dir=${TG/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=$PZ" 2>&1 | tail -6
else
  echo "zhijin v3 抓取轮无遥测"
fi

log "清理 Server"
docker rm -f "$SERVER_NAME" >/dev/null 2>&1 || true
