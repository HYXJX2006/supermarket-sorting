#!/usr/bin/env bash
# Fixed single-item full-chain validation.
# Runs the complete real simulation sequence and stops on the first failure.
set -eo pipefail

IMAGE_SERVER="${IMAGE_SERVER:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"
ROOT="${ROOT:-/mnt/e/workspace/claude_workplace/揭榜挂帅}"
DOMAIN="${ROS_DOMAIN_ID:-99}"
SERVER_NAME="${SERVER_NAME:-fixed_full_chain_server}"
DISPLAY_VALUE="${DISPLAY_VALUE:-172.25.192.1:0.0}"
# GUI rendering can slow actuator convergence; keep each stage timeout explicit.
SAFE_POSE_TIMEOUT="${SAFE_POSE_TIMEOUT:-60}"
NAV_TIMEOUT="${NAV_TIMEOUT:-300}"
DEPLOY_TIMEOUT="${DEPLOY_TIMEOUT:-180}"
CREEP_TIMEOUT="${CREEP_TIMEOUT:-180}"
CLOSE_TIMEOUT="${CLOSE_TIMEOUT:-30}"
LIFT_TIMEOUT="${LIFT_TIMEOUT:-60}"
RETREAT_TIMEOUT="${RETREAT_TIMEOUT:-120}"
DELIVERY_TIMEOUT="${DELIVERY_TIMEOUT:-600}"
PLACE_TIMEOUT="${PLACE_TIMEOUT:-60}"
BASELINE="/workspace/baseline"

fail_cleanup() {
  status=$?
  if [[ -n "${SERVER_NAME:-}" ]]; then
    docker logs --tail 240 "$SERVER_NAME" 2>&1 || true
    docker rm -f "$SERVER_NAME" >/dev/null 2>&1 || true
  fi
  exit "$status"
}
trap fail_cleanup EXIT INT TERM

docker rm -f "$SERVER_NAME" >/dev/null 2>&1 || true

echo "=== start X11 GUI Server ==="
docker run -d --name "$SERVER_NAME" --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
  -e DISPLAY="$DISPLAY_VALUE" -e MUJOCO_GL=glfw -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 -e SUPERMARKET_HEADLESS=0 \
  -e SUPERMARKET_ENABLE_RENDER=1 -e SUPERMARKET_ENABLE_LIDAR=1 \
  -e SUPERMARKET_USE_GS=0 -e SUPERMARKET_FIXED_BASELINE=1 \
  -e SUPERMARKET_RANDOMIZE=0 -e SUPERMARKET_RANDOMIZE_OBSTACLES=0 \
  -e SUPERMARKET_TASKS=product_032 -e SUPERMARKET_GUI_RENDER_WIDTH=960 \
  -e SUPERMARKET_GUI_RENDER_HEIGHT=600 -e SUPERMARKET_RENDER_FPS=12 \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  -v "$ROOT/baseline/patches/discoverse/envs/simulator.py:/workspace/supermarket_sorting_task/discoverse/envs/simulator.py:ro" \
  -v "$ROOT/baseline/patches/examples/ros2/mmk2_ros2.py:/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py:ro" \
  "$IMAGE_SERVER" bash -lc 'cd /workspace/supermarket_sorting_task && source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/server_bootstrap.py'

sleep 10
if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME")" != "running" ]]; then
  echo "Server failed to stay running" >&2
  exit 3
fi
docker logs --tail 80 "$SERVER_NAME" 2>&1

run_client() {
  local name="$1"
  shift
  docker run --rm --name "$name" --network host --ipc host \
    -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
    -v "$ROOT/baseline:/workspace/baseline:ro" \
    "$IMAGE_CLIENT" bash -lc "source /opt/ros/humble/setup.bash && $*"
}

echo "=== safe_pose ==="
run_client fixed_full_safe_pose \
  "python3 -u $BASELINE/safe_arm_controller.py safe_pose --execute --confirm safe_pose --timeout $SAFE_POSE_TIMEOUT"

echo "=== navigate to D shelf ==="
run_client fixed_full_nav \
  "python3 -u $BASELINE/low_speed_navigator.py --route '1.92,2.475,1.5707963267948966;0.852,2.475,1.5707963267948966' --max-speed 0.30 --goal-tolerance 0.05 --timeout $NAV_TIMEOUT --ignore-obstacles"

if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME")" != "running" ]]; then
  echo "Server exited after navigation" >&2
  exit 4
fi

echo "=== publish fixed target and execute arm/base chain ==="
docker run --rm --name fixed_full_actions --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DEPLOY_TIMEOUT="$DEPLOY_TIMEOUT" -e CREEP_TIMEOUT="$CREEP_TIMEOUT" \
  -e CLOSE_TIMEOUT="$CLOSE_TIMEOUT" -e LIFT_TIMEOUT="$LIFT_TIMEOUT" \
  -e RETREAT_TIMEOUT="$RETREAT_TIMEOUT" -e DELIVERY_TIMEOUT="$DELIVERY_TIMEOUT" \
  -e PLACE_TIMEOUT="$PLACE_TIMEOUT" \
  -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  "$IMAGE_CLIENT" bash -lc '
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 -u /workspace/baseline/known_slot_plan_publisher.py \
  --target-id known_product_032 --kind kele --slot D/L2/C2 \
  --world 0.92 3.243 0.9235 \
  --navigation-goal 1.04 2.60 1.5707963267948966 &
PUB_PID=$!
trap "kill $PUB_PID 2>/dev/null || true" EXIT
sleep 1
python3 -u /workspace/baseline/grasp_deploy_controller.py --execute --confirm deploy --timeout $DEPLOY_TIMEOUT
echo "STAGE deploy=success"
python3 -u /workspace/baseline/grasp_creep_controller.py --execute --confirm creep --pickup-approach --speed 0.12 --timeout $CREEP_TIMEOUT
echo "STAGE creep=success"
python3 -u /workspace/baseline/close_gripper_controller.py --execute --confirm close_gripper --timeout $CLOSE_TIMEOUT --hold-seconds 0.8
echo "STAGE close_gripper=success"
python3 -u /workspace/baseline/lift_controller.py --execute --confirm lift --timeout $LIFT_TIMEOUT
echo "STAGE lift=success"
python3 -u /workspace/baseline/retreat_controller.py --execute --confirm retreat --timeout $RETREAT_TIMEOUT --distance 0.55
echo "STAGE retreat=success"
python3 -u /workspace/baseline/global_delivery_navigator.py --execute --confirm deliver_nav_global --timeout $DELIVERY_TIMEOUT
echo "STAGE deliver_nav_global=success"
python3 -u /workspace/baseline/place_controller.py --execute --confirm place --timeout $PLACE_TIMEOUT
echo "STAGE place=success"
'

echo "=== final safe_pose (收臂) ==="
run_client fixed_full_final_safe_pose \
  "python3 -u $BASELINE/safe_arm_controller.py safe_pose --execute --confirm safe_pose --timeout 60"

echo "=== full chain success ==="
docker logs --tail 120 "$SERVER_NAME" 2>&1
docker rm -f "$SERVER_NAME" >/dev/null 2>&1 || true
trap - EXIT INT TERM
