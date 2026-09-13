#!/usr/bin/env bash
# 不同随机障碍布局的走廊穿越测试（不含抓取）。
# 用法：wsl.exe -d Ubuntu-22.04 -u root -- bash /mnt/e/.../baseline/run_obstacle_seed_tests.sh [seeds...]
set -uo pipefail

ROOT=/mnt/e/workspace/claude_workplace/揭榜挂帅
OUT="$ROOT/baseline/debug_data/obstacle_seed_tests"
mkdir -p "$OUT"
SUMMARY="$OUT/summary.md"
SEEDS="${1:-7 1000010 11 42 99}"
IMAGE=crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final

echo "# 障碍区穿越测试（$(date '+%m-%d %H:%M')）" > "$SUMMARY"
echo "| 种子 | 结果 | 耗时s | 备注 |" >> "$SUMMARY"
echo "|---|---|---|---|" >> "$SUMMARY"

for SEED in $SEEDS; do
  echo "=== seed=$SEED $(date '+%H:%M:%S') ===" | tee -a "$OUT/progress.log"
  docker rm -f obstacle_test_server > /dev/null 2>&1
  bash "$ROOT/baseline/test_server_start.sh" "$SEED" > /dev/null 2>&1
    > /dev/null 2>&1
  sleep 40   # 等 server 就绪（GS 加载）
  docker run --rm --name obstacle_test_nav --network host --ipc host \
    -e ROS_DOMAIN_ID=99 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$ROOT/baseline:/workspace/baseline:ro" \
    "$IMAGE:client" bash -lc "source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/test_corridor_pass.py --seed $SEED" \
    2>&1 | tee -a "$OUT/seed_${SEED}.log" | grep -E "RESULT|ROUTE" || true
  RESULT=$(grep -E "RESULT" "$OUT/seed_${SEED}.log" | tail -1)
  PASS=$(echo "$RESULT" | grep -o "PASS=True" || echo "PASS=False")
  ELAPSED=$(echo "$RESULT" | grep -oE "elapsed=[0-9]+" | cut -d= -f2 || echo "-")
  REASON=$(echo "$RESULT" | grep -oE "reason=[a-z_0-9]+" | cut -d= -f2 || echo "-")
  echo "| $SEED | $PASS | $ELAPSED | $REASON |" >> "$SUMMARY"
  docker rm -f obstacle_test_server obstacle_test_nav > /dev/null 2>&1
  sleep 5
done
echo "" >> "$SUMMARY"
echo "完成 $(date '+%m-%d %H:%M')" >> "$SUMMARY"
echo "=== 全部完成，见 $SUMMARY ==="
