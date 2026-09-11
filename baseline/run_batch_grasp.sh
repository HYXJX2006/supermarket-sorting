#!/usr/bin/env bash
# 批量抓取标定驱动：逐个商品跑完整动作链并采集裁判真值。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（Bash 工具 run_in_background=true）。
# 参见 run_item_grasp.sh 顶部关于 vmIdleTimeout 的说明。
#
# 清单为货架 D 上尚未验证的商品。D 架 x 坐标随列变化（C1=0.70 / C2=0.92 / C3=1.14），
# 导航终点 x 按 NAV_X_OFFSET（夹爪前伸补偿）换算：end_x = 目标x - 0.068。
#
# maidong 不在货架 D（A[2] B[1] C[1] E[1]），需单独用其他货架的导航路线验证。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
export HEADLESS="${HEADLESS:-1}"        # 批量模式用 OSMesa 离屏，快且不占桌面
export HOLD_SECONDS="${HOLD_SECONDS:-40}"
NAV_X_OFFSET="${NAV_X_OFFSET:-0.068}"
NAV_YAW="1.5707963267948966"

# 判定阶段要用 client 镜像起一个一次性容器跑分析脚本
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"

# kind|target_id|slot|world_x|world_y|world_z
ITEMS=(
  "sanmingzhi|product_028|D/L1/C1|0.70|3.243|0.5484"
  "heweidao|product_029|D/L1/C2|0.92|3.243|0.5515"
  "zhijin|product_031|D/L2/C1|0.70|3.243|0.895"
  "kouxiangtang|product_034|D/L3/C1|0.70|3.243|1.229"
  "chengzi|product_036|D/L3/C3|1.14|3.243|1.226"
)

echo "================================================================"
echo "[$(date +%H:%M:%S)] 批量标定开始，共 ${#ITEMS[@]} 项"
echo "================================================================"

PASS=0
FAIL=0
declare -a SUMMARY

for entry in "${ITEMS[@]}"; do
  IFS='|' read -r kind target slot wx wy wz <<< "$entry"

  endx=$(python3 -c "print(round($wx - $NAV_X_OFFSET, 4))")
  route="1.92,2.475,$NAV_YAW;${endx},2.475,$NAV_YAW"
  logfile="$ROOT/baseline/debug_data/batch_${kind}.log"

  echo
  echo "----------------------------------------------------------------"
  echo "[$(date +%H:%M:%S)] >>> $kind  slot=$slot  target=$target"
  echo "    world=($wx $wy $wz)  nav_end_x=$endx"
  echo "----------------------------------------------------------------"

  KIND="$kind" TARGET_ID="$target" SLOT="$slot" \
  WORLD="$wx $wy $wz" LABEL="$kind" NAV_ROUTE="$route" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$logfile" 2>&1
  rc=$?

  # 客观判定：只看裁判真值，不看控制器退出码
  telem_dir=$(grep -a '遥测目录: ' "$logfile" | tail -1 | sed 's/.*遥测目录: //')
  stages=$(grep -ac 'STAGE .*=success' "$logfile" || true)

  verdict="UNKNOWN"
  rise="?"
  opening="?"
  if [[ -n "$telem_dir" && -d "$telem_dir" ]]; then
    out=$(docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
      "$IMAGE_CLIENT" bash -lc \
      "source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/analyse_pingguo_calibration.py \
       --dir=/workspace/baseline/${telem_dir#$ROOT/baseline/} --baseline-z=$wz" 2>&1 | tail -20)
    verdict=$(echo "$out" | grep -a '结论' | sed 's/.*结论 *: //')
    rise=$(echo "$out" | grep -a '物体 z 最大上升' | awk '{print $NF}')
    opening=$(echo "$out" | grep -a '两指开口后半段中位' | awk '{print $NF}')
    echo "$out" | grep -a -E '双指同时接触帧数|物体 z 最大上升|两指开口后半段中位|结论'
  else
    verdict="NO_TELEMETRY"
    echo "(未找到遥测目录)"
  fi

  if [[ "$verdict" == SUCCESS* ]]; then
    PASS=$((PASS+1)); mark="✅"
  else
    FAIL=$((FAIL+1)); mark="❌"
  fi
  SUMMARY+=("$mark $kind  stages=$stages  rise=${rise}  opening=${opening}mm  ${verdict}")
  echo "[$(date +%H:%M:%S)] $mark $kind -> $verdict (rc=$rc)"
done

echo
echo "================================================================"
echo "[$(date +%H:%M:%S)] 批量标定结束：通过 $PASS / 失败 $FAIL"
echo "================================================================"
for line in "${SUMMARY[@]}"; do echo "$line"; done
