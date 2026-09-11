#!/usr/bin/env bash
# 苹果(pingguo)抓取可视化保持：跑通 safe_pose→nav→deploy→creep→close→lift 后，
# 不复位、不退出，让 MuJoCo 窗口长时间停留在"苹果已被夹住并抬起"的姿态供人工确认。
#
# 与 run_pingguo_calibration.sh 的差别仅两点：
#   1) 不 sleep 30 就退出，而是保持窗口 HOLD_SECONDS 秒；
#   2) 容器一律用 --restart no + 前台阻塞，避免 WSL 资源竞争导致 tangle。
# CLI 用法严格对齐标定脚本（已验证可跑通的那一套），不做改动。
set -uo pipefail

# ROOT 默认指向 WSL 原生盘副本，而非 9P 挂载的 /mnt/e。
# 动因：Server 启动瞬间从 /mnt/e 大批量读取模型/纹理会打爆 9P 通道，
# 触发 WSL 发行版重启（内核日志 p9io.cpp:258 AcceptAsync -> systemd-shutdow）。
ROOT="${ROOT:-/opt/jbgz}"
DOMAIN="${DOMAIN:-99}"
SLOT="${SLOT:-D/L3/C2}"
WORLD="${WORLD:-0.92 3.243 1.224}"
TARGET_ID="${TARGET_ID:-product_035}"
KIND="${KIND:-pingguo}"
# 用 WSLg 的 unix socket（DISPLAY=:0），不要走 TCP。
# VcXsrv 那条路依赖 172.25.192.1:6000，Windows 防火墙重启后会重新识别 WSL 虚拟网卡
# 并拦掉入站（实测外网全通但宿主 135/445/6000 全 CLOSED），诊断成本高。
DISPLAY_VALUE="${DISPLAY_VALUE:-:0}"
SERVER_NAME="${SERVER_NAME:-pingguo_hold_server}"
TELEM_NAME="${TELEM_NAME:-pingguo_hold_telem}"
HOLD_SECONDS="${HOLD_SECONDS:-1200}"
# HEADLESS=1 时不创建 MuJoCo 窗口，改用 OSMesa 离屏渲染，完全绕开图形栈。
# 用途：判定 WSL 发行版反复重启是否由 WSLg（Xwayland+Weston+RDP）引起。
HEADLESS="${HEADLESS:-0}"
# NO_GPU=1 时不向容器暴露 GPU（去掉 --gpus all）。
# 用途：内核日志每次都有大量 `dxgkio_query_adapter_info: Ioctl failed: -22`，
# 怀疑 GPU 直通通道在长时间仿真后异常并拖垮整个发行版，故做消融实验。
NO_GPU="${NO_GPU:-0}"
if [[ "$NO_GPU" == "1" ]]; then
  GPU_ARGS=""
else
  GPU_ARGS="--gpus all"
fi
if [[ "$HEADLESS" == "1" ]]; then
  MUJOCO_GL_VALUE="${MUJOCO_GL_VALUE:-osmesa}"
else
  MUJOCO_GL_VALUE="${MUJOCO_GL_VALUE:-glfw}"
fi
NAV_ROUTE="${NAV_ROUTE:-1.92,2.475,1.5707963267948966;0.852,2.475,1.5707963267948966}"
NAV_GOAL="1.0 2.5 3.1416"
META_WINDOW="${META_WINDOW:-900}"
SAFE_POSE_TIMEOUT="${SAFE_POSE_TIMEOUT:-120}"
NAV_TIMEOUT="${NAV_TIMEOUT:-180}"
DEPLOY_TIMEOUT="${DEPLOY_TIMEOUT:-90}"
CREEP_TIMEOUT="${CREEP_TIMEOUT:-150}"
CLOSE_TIMEOUT="${CLOSE_TIMEOUT:-60}"
LIFT_TIMEOUT="${LIFT_TIMEOUT:-60}"

IMAGE_SERVER="${IMAGE_SERVER:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"

STAMP="$(date +%Y%m%d_%H%M%S)"
TELEM_DIR_HOST="$ROOT/baseline/debug_data/grasp_evaluation/pingguo_hold_${STAMP}"
TELEM_DIR_MNT="/workspace/baseline/debug_data/grasp_evaluation/pingguo_hold_${STAMP}"
BASELINE="/workspace/baseline"

log() { echo "[$(date +%H:%M:%S)] $*"; }

log "预检：清掉同名容器"
docker rm -f "$SERVER_NAME" "$TELEM_NAME" pingguo_hold_meta pingguo_hold_actions >/dev/null 2>&1 || true

# 关键：容器内 Python 是 3.10，会优先加载 baseline/__pycache__/*.cpython-310.pyc。
# baseline 以 :ro 挂载只阻止"写"新缓存，不阻止"读"旧缓存；源码改过而 .pyc 陈旧时
# 容器会静默使用旧字节码，表现为"修复不生效"。每次运行前清掉。
log "清理陈旧的 Python 3.10 字节码缓存"
find "$ROOT/baseline/__pycache__" -name '*.cpython-310.pyc' -delete 2>/dev/null || true

log "校验 WSLg X11 通路（unix socket $DISPLAY_VALUE）"
# 注意不要写成 `docker run ... | tee log | grep -q PATTERN`。
# `grep -q` 命中后立即退出并关闭管道，tee 随即收到 SIGPIPE（退出码 141）；
# 在 `set -o pipefail` 下整条管道因此返回非零，`if ! ...` 会把"自检成功"误判为失败。
# 这里先把输出完整落盘，再单独判断。
if [[ "$HEADLESS" == "1" ]]; then
  log "HEADLESS=1，跳过 X11 自检（使用 $MUJOCO_GL_VALUE 离屏渲染）"
else
X11PROBE_LOG=/tmp/x11probe.log
: > "$X11PROBE_LOG"
docker run --rm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY="$DISPLAY_VALUE" \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  "$IMAGE_SERVER" bash -lc \
  "python3 -u /workspace/baseline/probe_x11_wslg.py" >> "$X11PROBE_LOG" 2>&1
X11_RC=$?
log "X11 自检退出码=$X11_RC"
cat "$X11PROBE_LOG"
if ! grep -q 'X11 握手成功' "$X11PROBE_LOG"; then
  echo "X11 通路自检失败，MuJoCo 窗口将不可见：" >&2
  cat "$X11PROBE_LOG" >&2 || true
  exit 5
fi
log "X11 通路正常"
fi

log "启动 Server（headless=$HEADLESS, MUJOCO_GL=$MUJOCO_GL_VALUE, use_gs=0）"
docker run -d --name "$SERVER_NAME" $GPU_ARGS --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
  -e DISPLAY="$DISPLAY_VALUE" -e MUJOCO_GL="$MUJOCO_GL_VALUE" -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e SUPERMARKET_HEADLESS="$HEADLESS" \
  -e SUPERMARKET_ENABLE_RENDER=1 -e SUPERMARKET_ENABLE_LIDAR=1 \
  -e SUPERMARKET_USE_GS=0 \
  -e SUPERMARKET_FIXED_BASELINE=1 -e SUPERMARKET_RANDOMIZE=0 \
  -e SUPERMARKET_RANDOMIZE_OBSTACLES=0 \
  -e SUPERMARKET_TASKS="$TARGET_ID" \
  -e SUPERMARKET_ENABLE_SCORE=1 \
  -e SUPERMARKET_GUI_RENDER_WIDTH=960 -e SUPERMARKET_GUI_RENDER_HEIGHT=600 \
  -e SUPERMARKET_RENDER_FPS=12 \
  -e MAX_JOBS=2 -e TORCH_CUDA_ARCH_LIST=8.9 \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
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
docker logs --tail 30 "$SERVER_NAME" 2>&1

log "启动只读遥测记录器（裁判 ground-truth：gripped / finger_contacts / position_world）"
mkdir -p "$TELEM_DIR_HOST"
docker run -d --name "$TELEM_NAME" --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$ROOT/baseline:/workspace/baseline:rw" \
  "$IMAGE_CLIENT" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 -u $BASELINE/debug_data/referee_target_monitor.py --output-dir $TELEM_DIR_MNT --duration 3600"

run_client() {
  local name="$1"; shift
  docker run --rm --name "$name" --network host --ipc host \
    -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e SUPERMARKET_FIXED_BASELINE=1 -e SUPERMARKET_RANDOMIZE=0 -e SUPERMARKET_USE_GS=0 \
    -v "$ROOT/baseline:/workspace/baseline:ro" \
    "$IMAGE_CLIENT" bash -lc "source /opt/ros/humble/setup.bash && $*"
}

log "safe_pose 收臂"
run_client pingguo_hold_safe_pose \
  "python3 -u $BASELINE/safe_arm_controller.py safe_pose --execute --confirm safe_pose --timeout $SAFE_POSE_TIMEOUT"

log "导航到货架前"
run_client pingguo_hold_nav \
  "python3 -u $BASELINE/low_speed_navigator.py --route '$NAV_ROUTE' --max-speed 0.30 --goal-tolerance 0.05 --timeout $NAV_TIMEOUT --ignore-obstacles"

if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME")" != "running" ]]; then
  echo "Server 在导航后退出" >&2; exit 4
fi

log "发布 $KIND 目标并执行 deploy → creep → close_gripper → lift（停在 lift，窗口保持）"
# creep 必须带 --object-stop-only：停止判据只依据物体几何（y 停止线），
# 不再叠加"货架物理前沿 front<=0.40m"门禁。苹果在 L3 高层，货位比 L2 更深，
# 底盘被货架挡住时 front 恒在 0.53m 附近、永远不满足 0.40，而名义 y 停止线
# 又要求末端再前进约 0.14m —— 两个条件互斥，会直接死锁到 worker 超时。
docker run --rm --name pingguo_hold_actions --network host --ipc host \
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
python3 -u $BASELINE/lift_controller.py --execute --confirm lift --timeout \$LIFT_TIMEOUT
echo 'STAGE lift=success'
"

log "动作链结束；MuJoCo 窗口保持在『苹果已夹住并抬起』姿态 $HOLD_SECONDS 秒"
echo "遥测目录（宿主机）: $TELEM_DIR_HOST"
echo "查看窗口: 保持容器存活，观察 MuJoCo 画面"
sleep "$HOLD_SECONDS"
