#!/usr/bin/env bash
# Local development: official 45-product random scene + five-target plan-only queue.
# No action worker is started by this script.
set -euo pipefail

ROOT="${ROOT:-/mnt/e/workspace/claude_workplace/揭榜挂帅}"
IMAGE_SERVER="${IMAGE_SERVER:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"
SEED="${SUPERMARKET_SEED:-11}"
DOMAIN="${ROS_DOMAIN_ID:-99}"

# Prevent fixed-baseline values from leaking into the random run.
unset SUPERMARKET_FIXED_BASELINE SUPERMARKET_TASKS SUPERMARKET_FIXED_TARGET_SLOT \
  SUPERMARKET_FIXED_TARGET_WORLD SUPERMARKET_FIXED_TARGET_WORLD_TOLERANCE || true

# Terminal 1: keep this process running as the official Server with the local
# five-target selector and the existing development-only sensor patches.
docker run --rm -it --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e MUJOCO_GL="${MUJOCO_GL:-glfw}" \
  -e SUPERMARKET_HEADLESS="${SUPERMARKET_HEADLESS:-1}" \
  -e SUPERMARKET_ENABLE_RENDER="${SUPERMARKET_ENABLE_RENDER:-1}" \
  -e SUPERMARKET_ENABLE_LIDAR="${SUPERMARKET_ENABLE_LIDAR:-1}" \
  -e SUPERMARKET_USE_GS="${SUPERMARKET_USE_GS:-0}" \
  -e SUPERMARKET_RANDOMIZE=1 \
  -e SUPERMARKET_RANDOMIZE_OBSTACLES=1 \
  -e SUPERMARKET_SEED="$SEED" \
  -e SUPERMARKET_TASK_SELECTION_SEED="$SEED" \
  -e SUPERMARKET_TASK_COUNT=5 \
  -e SUPERMARKET_EXPECTED_TARGET_COUNT=5 \
  -e MAX_JOBS="${MAX_JOBS:-1}" \
  -e TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}" \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v supermarket_sorting_cache:/root/.cache \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  -v "$ROOT/baseline/patches/discoverse/envs/simulator.py:/workspace/supermarket_sorting_task/discoverse/envs/simulator.py:ro" \
  -v "$ROOT/baseline/patches/examples/ros2/mmk2_ros2.py:/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py:ro" \
  "$IMAGE_SERVER" bash -lc \
  'cd /workspace/supermarket_sorting_task && source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/random_server_bootstrap.py'

# Terminal 2, after Terminal 1 has printed the five-target task:
# docker run --rm -it --network host --ipc host \
#   -e ROS_DOMAIN_ID="$DOMAIN" \
#   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
#   -v "$ROOT/baseline:/workspace/baseline:ro" \
#   "$IMAGE_CLIENT" \
#   bash -lc "source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/competition_executor.py --duration 0 --expected-target-count 5 --detection-topic /multiclass/detections --no-zero --slot-lock-frames 5 --aruco-hold-seconds 0.8"
