#!/usr/bin/env bash
# 比赛版全流程"自持"启动器：在 WSL 内部 setsid 脱离，日志落盘。
# 目的：不再依赖任何 wsl.exe / 后台任务进程的存活 —— 之前用 pwsh 后台任务
# 托管时，任务被中断会连带把 client(143)/server(255)/viewer(255) 一起带走。
#
# 用法：bash launch_comp.sh [TASKS] [TASK_COUNT] [SEED]
#   例：bash launch_comp.sh sanmingzhi 1 7
set -u

TASKS="${1:-${SUPERMARKET_TASKS:-sanmingzhi}}"
COUNT="${2:-${SUPERMARKET_TASK_COUNT:-1}}"
SEED="${3:-${SUPERMARKET_SEED:-7}}"

ROOT=/mnt/e/workspace/claude_workplace/揭榜挂帅
RUNDIR=/opt/jbgz/run
LOG="$RUNDIR/comp_${TASKS}.log"
SCRIPT="$ROOT/baseline/run_random5_autopilot.sh"

mkdir -p "$RUNDIR" 2>&1 | tee -a /tmp/launch_comp.err
if [ ! -d "$RUNDIR" ]; then
  echo "FATAL: 无法创建 $RUNDIR"
  exit 1
fi

docker rm -f viewer_stream >/dev/null 2>&1 || true
docker run -d --name viewer_stream --network host --ipc host \
  -e ROS_DOMAIN_ID=99 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
  bash -lc 'source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/three_cam_stream.py' \
  >/dev/null 2>&1
echo "viewer: http://localhost:8090/  (容器 viewer_stream)"

: > "$LOG"
cd "$ROOT" || exit 1
# 现场可调旋钮（主人边看 8090 边改）：
#   GRASP_Z_OFFSET      抓取高度偏置，m，负=压低（默认 -0.02）
#   SERVO_LATERAL_GAIN  手眼横向伺服比例增益（默认 1.25，历史收敛档）
#   SERVO_MAX_ITER      横向迭代上限（默认沿用 worker 自身参数）
GRASP_Z_OFFSET="${GRASP_Z_OFFSET:-}"
SERVO_LATERAL_GAIN="${SERVO_LATERAL_GAIN:-}"
EXTRA_ENV=()
[ -n "$GRASP_Z_OFFSET" ] && EXTRA_ENV+=("SUPERMARKET_GRASP_Z_OFFSET_M=$GRASP_Z_OFFSET")
[ -n "$SERVO_LATERAL_GAIN" ] && EXTRA_ENV+=("SUPERMARKET_SERVO_LATERAL_GAIN=$SERVO_LATERAL_GAIN")

env SUPERMARKET_TASKS="$TASKS" SUPERMARKET_TASK_COUNT="$COUNT" SUPERMARKET_SEED="$SEED" \
  "${EXTRA_ENV[@]}" \
  setsid nohup bash "$SCRIPT" >> "$LOG" 2>&1 < /dev/null &
echo "launched pid=$! tasks=$TASKS count=$COUNT seed=$SEED log=$LOG"
echo "旋钮: GRASP_Z_OFFSET=${GRASP_Z_OFFSET:-默认-0.02}  SERVO_LATERAL_GAIN=${SERVO_LATERAL_GAIN:-默认}"
sleep 3
echo "--- 日志开头 ---"
head -n 5 "$LOG" 2>/dev/null || echo "(日志尚未生成)"
