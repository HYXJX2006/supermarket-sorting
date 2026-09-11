#!/usr/bin/env bash
# 苹果(pingguo)单目标夹取标定：真实动作链，停在 lift，保留 3D 窗口供人工确认。
#
#   Server: MuJoCo native 实时窗口(USE_GS=0)，可见小车/苹果/ArUco
#   Client: 只读遥测记录器 + 已知货位发布 + deploy/creep/close_gripper/lift
#
# 用法：
#   bash baseline/run_pingguo_calibration.sh
# 常用覆盖：
#   PINGGUO_SLOT=D/L2/C2  PINGGUO_WORLD="0.92 3.243 0.9235"
#   STOP_AFTER=lift       # lift|close_gripper|creep
set -euo pipefail

ROOT="${ROOT:-/mnt/e/workspace/claude_workplace/揭榜挂帅}"
IMAGE_SERVER="${IMAGE_SERVER:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"
DOMAIN="${ROS_DOMAIN_ID:-99}"
SERVER_NAME="${SERVER_NAME:-pingguo_calib_server}"
TELEM_NAME="${TELEM_NAME:-pingguo_calib_telem}"
STAMP="$(date +%Y%m%d_%H%M%S)"
TELEM_DIR="${TELEM_DIR:-/workspace/baseline/debug_data/grasp_evaluation/pingguo_calib_${STAMP}}"
TELEM_DIR_HOST="${ROOT}/baseline/debug_data/grasp_evaluation/pingguo_calib_${STAMP}"

# 商品与货位：官方 layout 中苹果固定为 D/L3/C2 = product_035，
# 世界坐标 [0.92, 3.243, 1.224]（L3 高层，z=1.224）。
KIND="${PINGGUO_KIND:-pingguo}"
SLOT="${PINGGUO_SLOT:-D/L3/C2}"
WORLD="${PINGGUO_WORLD:-0.92 3.243 1.224}"
NAV_GOAL="${PINGGUO_NAV_GOAL:-1.04 2.60 1.5707963267948966}"
TARGET_ID="${PINGGUO_TARGET_ID:-product_035}"

DISPLAY_VALUE="${DISPLAY_VALUE:-172.25.192.1:0.0}"
USE_GS="${SUPERMARKET_USE_GS:-0}"
HEADLESS="${SUPERMARKET_HEADLESS:-0}"

SAFE_POSE_TIMEOUT="${SAFE_POSE_TIMEOUT:-90}"
NAV_TIMEOUT="${NAV_TIMEOUT:-300}"
DEPLOY_TIMEOUT="${DEPLOY_TIMEOUT:-180}"
CREEP_TIMEOUT="${CREEP_TIMEOUT:-180}"
CLOSE_TIMEOUT="${CLOSE_TIMEOUT:-40}"
LIFT_TIMEOUT="${LIFT_TIMEOUT:-60}"
STOP_AFTER="${STOP_AFTER:-lift}"
# 元数据窗口必须覆盖整条动作链。L3 高层 creep 单独就 >60s，若沿用旧的
# 40s 窗口，lift_controller 会在订阅到 grasp_goal_meta 之前失去发布源，
# 于是退回构造函数默认门禁 0.85，把已改好的类别门禁"吃掉"。
META_WINDOW="${META_WINDOW:-900}"
BASELINE="/workspace/baseline"
TELEM_SECONDS="${TELEM_SECONDS:-420}"

log() { printf '\n=== %s ===\n' "$*"; }

log "清理旧容器"
docker rm -f "$SERVER_NAME" "$TELEM_NAME" >/dev/null 2>&1 || true

# 关键：容器内 Python 是 3.10，会优先加载 baseline/__pycache__/*.cpython-310.pyc。
# baseline 以 :ro 挂载只阻止"写"新缓存，不阻止"读"旧缓存；一旦源码改过而 .pyc 陈旧，
# 容器会静默使用旧字节码，表现为"修复不生效"（源码 grep 正确也没用）。每次运行前清掉。
log "清理陈旧的 Python 3.10 字节码缓存"
find "$ROOT/baseline/__pycache__" -name '*.cpython-310.pyc' -delete 2>/dev/null || true

log "启动 Server（MuJoCo native 窗口，headless=$HEADLESS, use_gs=$USE_GS）"
docker run -d --name "$SERVER_NAME" --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
  -e DISPLAY="$DISPLAY_VALUE" -e MUJOCO_GL=glfw -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e SUPERMARKET_HEADLESS="$HEADLESS" \
  -e SUPERMARKET_ENABLE_RENDER=1 -e SUPERMARKET_ENABLE_LIDAR=1 \
  -e SUPERMARKET_USE_GS="$USE_GS" \
  -e SUPERMARKET_FIXED_BASELINE=1 -e SUPERMARKET_RANDOMIZE=0 \
  -e SUPERMARKET_RANDOMIZE_OBSTACLES=0 \
  -e SUPERMARKET_TASKS=product_035 \
  -e SUPERMARKET_ENABLE_SCORE=1 \
  -e SUPERMARKET_GUI_RENDER_WIDTH=960 -e SUPERMARKET_GUI_RENDER_HEIGHT=600 \
  -e SUPERMARKET_RENDER_FPS=12 \
  -e MAX_JOBS=2 -e TORCH_CUDA_ARCH_LIST=8.9 \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  -v supermarket_sorting_cache:/root/.cache \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  -v "$ROOT/baseline/patches/discoverse/envs/simulator.py:/workspace/supermarket_sorting_task/discoverse/envs/simulator.py:ro" \
  -v "$ROOT/baseline/patches/examples/ros2/mmk2_ros2.py:/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py:ro" \
  "$IMAGE_SERVER" bash -lc \
  'cd /workspace/supermarket_sorting_task && source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/calibration_server_bootstrap.py'

sleep 12
if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME")" != "running" ]]; then
  echo "Server 未能保持运行" >&2
  docker logs --tail 120 "$SERVER_NAME" 2>&1 || true
  exit 3
fi
docker logs --tail 40 "$SERVER_NAME" 2>&1

log "启动只读遥测记录器（裁判 ground-truth：gripped / finger_contacts / position_world）"
mkdir -p "$TELEM_DIR_HOST"
docker run -d --name "$TELEM_NAME" --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$ROOT/baseline:/workspace/baseline:rw" \
  "$IMAGE_CLIENT" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/debug_data/referee_target_monitor.py --output-dir $TELEM_DIR --duration $TELEM_SECONDS"

run_client() {
  local name="$1"; shift
  docker run --rm --name "$name" --network host --ipc host \
    -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
    -v "$ROOT/baseline:/workspace/baseline:ro" \
    "$IMAGE_CLIENT" bash -lc "source /opt/ros/humble/setup.bash && $*"
}

log "safe_pose 收臂"
run_client pingguo_calib_safe_pose \
  "python3 -u $BASELINE/safe_arm_controller.py safe_pose --execute --confirm safe_pose --timeout $SAFE_POSE_TIMEOUT"

log "导航到货架前"
run_client pingguo_calib_nav \
  "python3 -u $BASELINE/low_speed_navigator.py --route '1.92,2.475,1.5707963267948966;0.852,2.475,1.5707963267948966' --max-speed 0.30 --goal-tolerance 0.05 --timeout $NAV_TIMEOUT --ignore-obstacles"

if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME")" != "running" ]]; then
  echo "Server 在导航后退出" >&2; exit 4
fi

log "发布 $KIND 目标并执行 deploy → creep → close_gripper → $STOP_AFTER"
# creep 必须带 --object-stop-only：停止判据只依据物体几何（y 停止线），
# 不再叠加"货架物理前沿 front<=0.40m"门禁。苹果在 L3 高层，货位比 L2 更深，
# 底盘被货架挡住时 front 恒在 0.53m 附近、永远不满足 0.40，而名义 y 停止线
# 又要求末端再前进约 0.14m —— 两个条件互斥，会直接死锁到 worker 超时。
docker run --rm --name pingguo_calib_actions --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DEPLOY_TIMEOUT="$DEPLOY_TIMEOUT" -e CREEP_TIMEOUT="$CREEP_TIMEOUT" \
  -e CLOSE_TIMEOUT="$CLOSE_TIMEOUT" -e LIFT_TIMEOUT="$LIFT_TIMEOUT" \
  -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  "$IMAGE_CLIENT" bash -lc "
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 -u $BASELINE/known_slot_plan_publisher.py \
  --target-id $TARGET_ID --kind $KIND --slot $SLOT \
  --world $WORLD --navigation-goal $NAV_GOAL \
  --window-seconds $META_WINDOW &
PUB_PID=\$!
trap \"kill \$PUB_PID 2>/dev/null || true\" EXIT
sleep 1
python3 -u $BASELINE/grasp_deploy_controller.py --execute --confirm deploy --timeout \$DEPLOY_TIMEOUT
echo 'STAGE deploy=success'
python3 -u $BASELINE/grasp_creep_controller.py --execute --confirm creep --object-stop-only --pickup-approach --speed 0.12 --timeout \$CREEP_TIMEOUT
echo 'STAGE creep=success'
python3 -u $BASELINE/close_gripper_controller.py --execute --confirm close_gripper --timeout \$CLOSE_TIMEOUT --hold-seconds 0.8
echo 'STAGE close_gripper=success'
if [[ '$STOP_AFTER' == 'lift' ]]; then
  python3 -u $BASELINE/lift_controller.py --execute --confirm lift --timeout \$LIFT_TIMEOUT
  echo 'STAGE lift=success'
fi
"

log "动作链结束；窗口保持 30 秒供人工确认夹爪是否夹中"
echo "遥测目录（宿主机）: $TELEM_DIR_HOST"
sleep 30

echo
echo "================================================================"
echo "Server 容器仍在运行: $SERVER_NAME"
echo "查看窗口: 保持容器存活，观察 MuJoCo 画面"
echo "停止并清理: docker rm -f $SERVER_NAME $TELEM_NAME"
echo "遥测: $TELEM_DIR_HOST"
echo "================================================================"
