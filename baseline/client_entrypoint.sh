#!/usr/bin/env bash
# 客户端容器入口脚本（替代 run_random5_autopilot.sh 里的内联 bash -lc）。
#
# 为什么单独成文件：内联在 docker run 的 "..." 里时，反斜杠续行、$ 转义、
# 引号三层嵌套极易出错，而且脚本一旦语法错就静默退出（实测客户端在
# "检测器就绪"之后直接 exit 0，ArUco 与编排器都没启动）。
#
# 由 run_random5_autopilot.sh 挂载进容器后执行：
#   bash /workspace/baseline/client_entrypoint.sh
set -u

BASELINE=/workspace/baseline
WEIGHTS="${MULTICLASS_WEIGHTS:-/workspace/baseline/debug_data/multiclass_train_v6_servo/confusion_finetune/weights/best.pt}"
CONF="${SUPERMARKET_DETECTION_CONFIDENCE:-0.15}"
ARUCO_SCRIPT="$BASELINE/official_baseline/examples/supermarket_sorting/perception/aruco_detect.py"

# ROS 的 setup.bash 会读 AMENT_TRACE_SETUP_FILES / COLCON_TRACE 等变量；
# 在 set -u 下它们是"未定义变量" → setup.bash 第 8 行直接报 unbound variable
# 退出（实测：脚本一 source 就死，ArUco/编排器根本没启动）。
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-}"
source /opt/ros/humble/setup.bash

echo "==> [client] 启动 YOLO 多类检测器"
python3 -u "$BASELINE/multiclass_detect.py" --weights "$WEIGHTS" --device "${DEVICE:-cuda}" \
  --confidence "$CONF" &
DET_PID=$!

cleanup() {
  kill "$DET_PID" "${ARUCO_PID:-0}" 2>/dev/null || true
}
trap cleanup EXIT

READY=0
for _ in $(seq 1 60); do
  if ! kill -0 "$DET_PID" 2>/dev/null; then
    echo "ERROR: 检测器提前退出" >&2
    exit 1
  fi
  if ros2 topic list --no-daemon 2>/dev/null | grep -qx /multiclass/detections; then
    READY=1
    break
  fi
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "ERROR: 检测器未在 60 秒内创建 /multiclass/detections" >&2
  exit 1
fi
echo "==> [client] 检测器就绪"

# ArUco：传统视觉，不需要训练。必须 --image-topic-mode native，
# 否则读的是 3DGS 渲染图（里面没有 ArUco 格子）。
echo "==> [client] 启动 ArUco 货位识别（native 图）"
python3 -u "$ARUCO_SCRIPT" --cameras head --marker-size 0.03 --detect-scale 2 \
  --no-tf --image-topic-mode native > /tmp/aruco_detect.log 2>&1 &
ARUCO_PID=$!
sleep 3
if kill -0 "$ARUCO_PID" 2>/dev/null; then
  echo "==> [client] ArUco 节点存活 pid=$ARUCO_PID"
else
  echo "WARN: ArUco 节点未存活，仅失去货位吸附（不影响抓取）" >&2
  tail -5 /tmp/aruco_detect.log >&2 || true
fi

echo "==> [client] 启动编排器"
python3 -u "$BASELINE/inventory_competition_executor.py" \
  --execute --confirm random5 \
  --map-timeout "${MAP_TIMEOUT:-240}" \
  --mapping-max-speed "${MAPPING_SPEED:-0.60}" \
  --cruise-max-speed "${CRUISE:-1.4}" \
  --obstacle-distance "${OBSTACLE_DIST:-0.35}" \
  --min-samples 5 \
  --min-confidence "${SUPERMARKET_MAP_CONFIDENCE:-0.25}" \
  --map-path "$BASELINE/debug_data/inventory_map_autopilot.json"
echo "==> [client] 编排器退出 code=$?"
