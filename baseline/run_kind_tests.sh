#!/usr/bin/env bash
# 比赛版全流程逐类抓取测试：每类一单（该类为第一单），跑完整
# 扫描→识别→伺服抓取→配送→放置，结果汇总到 debug_data/kind_tests/summary.md
#
# 用法（宿主 Git Bash）：
#   wsl.exe -d Ubuntu-22.04 -u root -- bash /mnt/e/workspace/claude_workplace/揭榜挂帅/baseline/run_kind_tests.sh
set -uo pipefail

ROOT=/mnt/e/workspace/claude_workplace/揭榜挂帅
OUT="$ROOT/baseline/debug_data/kind_tests"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.md"
KINDS="${1:-pingguo kouxiangtang kele chengzi heweidao sanmingzhi maidong shupian}"

echo "# 逐类比赛流程测试（$(date '+%m-%d %H:%M')）" > "$SUMMARY"
echo "每类一单，比赛版全流程（扫描→识别→伺服抓取→配送→放置）。" >> "$SUMMARY"
echo "" >> "$SUMMARY"
echo "| 品类 | 结果 | placed | 备注 |" >> "$SUMMARY"
echo "|---|---|---|---|" >> "$SUMMARY"

for KIND in $KINDS; do
  echo "=== [$KIND] 开始 $(date '+%H:%M:%S') ===" | tee -a "$OUT/progress.log"
  STAMP=$(date +%m%d_%H%M%S)
  LOG="$OUT/${KIND}_${STAMP}.log"
  SEED=$((RANDOM % 100000))
  SUPERMARKET_TASKS="$KIND" \
  SUPERMARKET_TASK_COUNT=1 \
  SUPERMARKET_SEED="$SEED" \
  bash "$ROOT/baseline/run_random5_autopilot.sh" > "$LOG" 2>&1
  CODE=$?
  # 提取结果
  PLACED=$(grep -oE "placed': [0-9]+" "$ROOT/baseline/debug_data/inventory_map_autopilot.json" 2>/dev/null | tail -1 | grep -oE "[0-9]+" || echo 0)
  if grep -q "已放置\|placed" "$LOG" | grep -qv "0"; then :; fi
  if grep -qE "伺服完成|闭爪时间门禁完成" "$LOG"; then CLOSE_OK="闭爪✓"; else CLOSE_OK="闭爪✗"; fi
  if grep -q "抬升动作完成" "$LOG"; then LIFT_OK="抬升✓"; else LIFT_OK="抬升✗"; fi
  if grep -q "配送导航超时\|配送段卡死" "$LOG"; then DELIV="配送✗"; elif grep -q "post_place_safe_pose" "$LOG"; then DELIV="配送✓"; else DELIV="配送?"; fi
  echo "| $KIND | $CLOSE_OK $LIFT_OK $DELIV | $PLACED | 退出码$CODE 种子$SEED 日志$LOG |" >> "$SUMMARY"
  echo "=== [$KIND] 结束 $(date '+%H:%M:%S') placed=$PLACED ===" | tee -a "$OUT/progress.log"
  docker rm -f random5_autopilot_server random5_autopilot_server_telem random5_autopilot_client > /dev/null 2>&1
  sleep 5
done
echo "" >> "$SUMMARY"
echo "全部完成 $(date '+%m-%d %H:%M')" >> "$SUMMARY"
