#!/usr/bin/env bash
# 单测 server 启动（从 run_random5_autopilot.sh 原样提取，保证 odom 等数据流一致）
# 用法: bash test_server_start.sh <SEED>
SEED="${1:-7}"
set -uo pipefail
ROOT=/mnt/e/workspace/claude_workplace/揭榜挂帅
DOMAIN=99
HEADLESS="${SUPERMARKET_HEADLESS:-1}"
USE_GS="${SUPERMARKET_USE_GS:-1}"
GS_BATCH_RENDER="${SUPERMARKET_GS_BATCH_RENDER:-1}"
GS_HEAD_ONLY="${SUPERMARKET_GS_HEAD_ONLY:-1}"
GS_ASYNC="${SUPERMARKET_GS_ASYNC:-1}"
RANDOMIZE_OBSTACLES="${SUPERMARKET_RANDOMIZE_OBSTACLES:-1}"
X11_DISPLAY="${X11_DISPLAY:-172.25.192.1:1}"
CRUISE=1.4
OBSTACLE_DIST=0.35
DETECTION_CONFIDENCE=0.15
REMOVE_CORRIDOR_OBSTACLES="${SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES:-0}"
TASKS_SPEC="${SUPERMARKET_TASKS:-}"
SCORE_ENABLED="${SUPERMARKET_ENABLE_SCORE:-0}"
TASK_ORDER_LOCKED=0
RENDER_FPS=6
GUI_RENDER_WIDTH=960
GUI_RENDER_HEIGHT=600
ARM_PREGRASP_Y=2.35
ARM_ODOM_STABLE_SAMPLES=5
PLAN_GATE_TIMEOUT=60
TRACK_STABILITY_WINDOW=8
docker run -d --name obstacle_test_server --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY="$X11_DISPLAY" \
  -e MUJOCO_GL="${MUJOCO_GL:-glfw}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT="${LIBGL_ALWAYS_INDIRECT:-0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e SUPERMARKET_HEADLESS="$HEADLESS" \
  -e SUPERMARKET_ENABLE_RENDER="${SUPERMARKET_ENABLE_RENDER:-1}" \
  -e SUPERMARKET_ENABLE_LIDAR="${SUPERMARKET_ENABLE_LIDAR:-1}" \
  -e SUPERMARKET_USE_GS="$USE_GS" \
  -e SUPERMARKET_GS_BATCH_RENDER="$GS_BATCH_RENDER" \
  -e SUPERMARKET_GS_HEAD_ONLY="$GS_HEAD_ONLY" \
  -e SUPERMARKET_GS_ASYNC="$GS_ASYNC" \
  -e SUPERMARKET_PUBLISH_ARUCO_NATIVE="${SUPERMARKET_PUBLISH_ARUCO_NATIVE:-1}" \
  -e SUPERMARKET_ARUCO_NATIVE_CAMERAS="${SUPERMARKET_ARUCO_NATIVE_CAMERAS:-head}" \
  -e SUPERMARKET_RENDER_FPS="$RENDER_FPS" \
  -e SUPERMARKET_ARM_PREGRASP_Y="$ARM_PREGRASP_Y" \
  -e SUPERMARKET_ARM_ODOM_STABLE_SAMPLES="$ARM_ODOM_STABLE_SAMPLES" \
  -e SUPERMARKET_PLAN_GATE_TIMEOUT="$PLAN_GATE_TIMEOUT" \
  -e SUPERMARKET_TRACK_MIN_CONFIDENCE="$DETECTION_CONFIDENCE" \
  -e SUPERMARKET_TRACK_STABILITY_WINDOW="${SUPERMARKET_TRACK_STABILITY_WINDOW:-8}" \
  -e SUPERMARKET_GUI_RENDER_WIDTH="${GUI_RENDER_WIDTH:-960}" \
  -e SUPERMARKET_GUI_RENDER_HEIGHT="${GUI_RENDER_HEIGHT:-600}" \
  -e SUPERMARKET_RANDOMIZE=1 \
  -e SUPERMARKET_RANDOMIZE_OBSTACLES="$RANDOMIZE_OBSTACLES" \
  -e SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES="$REMOVE_CORRIDOR_OBSTACLES" \
  -e SUPERMARKET_GS_BATCH_RENDER="$GS_BATCH_RENDER" \
  -e SUPERMARKET_SEED="$SEED" \
  -e SUPERMARKET_TASK_SELECTION_SEED="$SEED" \
  -e SUPERMARKET_TASKS="$TASKS_SPEC" \
  -e SUPERMARKET_ENABLE_SCORE="$SCORE_ENABLED" \
  -e SUPERMARKET_TASK_COUNT=1 \
  -e SUPERMARKET_EXPECTED_TARGET_COUNT=1 \
  -e SUPERMARKET_TASK_ORDER_LOCKED="$TASK_ORDER_LOCKED" \
  -e MAX_JOBS="${MAX_JOBS:-2}" \
  -e TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}" \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  -v supermarket_sorting_cache:/root/.cache \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  -v "$ROOT/baseline/patches/discoverse/envs/simulator.py:/workspace/supermarket_sorting_task/discoverse/envs/simulator.py:ro" \
  -v "$ROOT/baseline/official_baseline/examples/supermarket_sorting/obstacle_layout.py:/workspace/supermarket_sorting_task/examples/supermarket_sorting/obstacle_layout.py:ro" \
  -v "$ROOT/baseline/patches/examples/ros2/mmk2_ros2.py:/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py:ro" \
  -v "$ROOT/baseline/patches/examples/ros2/mmk2_ros2.py:/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py:ro" \
  "$IMAGE_SERVER" bash -lc \
  'cd /workspace/supermarket_sorting_task && source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/random_server_bootstrap.py'
echo "test server up"
