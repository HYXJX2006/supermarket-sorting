#!/usr/bin/env bash
# 通用单品抓取标定：safe_pose → nav → deploy → creep → close_gripper → lift。
#
# 由 run_pingguo_hold.sh 泛化而来，支持任意商品/货位。
#
# ⚠️ 关键约束：必须以「wsl.exe 会话存活」的方式启动（Bash 工具 run_in_background=true），
# 不要在外部用 setsid ... & disown 后立即返回。原因是 .wslconfig 的 vmIdleTimeout
# 默认为 60000 ms：最后一个 wsl.exe 会话断开满 60 秒后，WSL 会判定 VM 空闲并关闭
# 整台虚拟机，正在运行的仿真连同容器一起被杀。表象是 docker.sock EOF、容器全灭。
#
# 用法：
#   KIND=chengzi TARGET_ID=product_036 SLOT=D/L3/C3 WORLD="1.14 3.243 1.226" \
#   LABEL=chengzi bash run_item_grasp.sh
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"          # WSL 原生盘副本，避开 9P
DOMAIN="${DOMAIN:-99}"

KIND="${KIND:-pingguo}"
TARGET_ID="${TARGET_ID:-product_035}"
SLOT="${SLOT:-D/L3/C2}"
WORLD="${WORLD:-0.92 3.243 1.224}"
LABEL="${LABEL:-$KIND}"

# 导航终点。货架 D 各列 x 不同，默认按 C2 列（目标 x=0.92，终点用 0.852，
# 差值 0.068 是夹爪前伸补偿）。C1/C3 传 NAV_ROUTE 覆盖即可。
NAV_ROUTE="${NAV_ROUTE:-1.92,2.475,1.5707963267948966;0.852,2.475,1.5707963267948966}"
NAV_GOAL="${NAV_GOAL:-1.0 2.5 3.1416}"

# 批量标定用 headless（OSMesa 离屏，快且不占桌面）；要看窗口时设 HEADLESS=0。
HEADLESS="${HEADLESS:-1}"
NO_GPU="${NO_GPU:-0}"
if [[ "$NO_GPU" == "1" ]]; then GPU_ARGS=""; else GPU_ARGS="--gpus all"; fi
if [[ "$HEADLESS" == "1" ]]; then
  MUJOCO_GL_VALUE="${MUJOCO_GL_VALUE:-osmesa}"
  DISPLAY_VALUE="${DISPLAY_VALUE:-}"
else
  MUJOCO_GL_VALUE="${MUJOCO_GL_VALUE:-glfw}"
  DISPLAY_VALUE="${DISPLAY_VALUE:-:0}"
fi

HOLD_SECONDS="${HOLD_SECONDS:-45}"
META_WINDOW="${META_WINDOW:-900}"
# 拨动模式的截断点：STOP_AFTER=creep 时只跑 deploy+creep（夹爪保持张开），
# 用于「拨动改变商品朝向」类策略——闭爪/抬升会破坏拨动效果。
STOP_AFTER="${STOP_AFTER:-lift}"
# creep 接近速度。轻且形状不规则的商品（如 sanmingzhi 三角形截面、0.122 kg）
# 用 0.12 m/s 接近会在接触瞬间把商品推歪，表现为 SINGLE_FINGER 且 xy_shift 偏大。
CREEP_SPEED="${CREEP_SPEED:-0.12}"
# 拨转后的抓取：商品横在指间、creep 的 stop 线算不准会超时——这是预期的。
# TOLERATE_CREEP_TIMEOUT=1 时 creep 失败不中止，继续 close+lift（靠过行程硬咬）。
TOLERATE_CREEP_TIMEOUT="${TOLERATE_CREEP_TIMEOUT:-0}"
# 抓取姿态：side=水平侧夹（默认）/ top=倒置俯抓咬顶棱。
# top 用于 zhijin 等横向对距(85mm)>开口(80mm)的商品——咬棱距离与 yaw 无关，
# 免去拨动改朝向。咬棱原理同 maidong 预压：过行程 5mm 硬咬顶棱。
GRASP_ROT_MODE="${GRASP_ROT_MODE:-side}"
TOP_GRASP_DZ="${TOP_GRASP_DZ:-0.05}"     # top 咬棱高度（相对商品中心）
TOP_HOVER_DZ="${TOP_HOVER_DZ:-0.16}"     # top 悬停高度（商品正上方）
SAFE_POSE_TIMEOUT="${SAFE_POSE_TIMEOUT:-120}"
NAV_TIMEOUT="${NAV_TIMEOUT:-180}"
DEPLOY_TIMEOUT="${DEPLOY_TIMEOUT:-90}"
CREEP_TIMEOUT="${CREEP_TIMEOUT:-150}"
CLOSE_TIMEOUT="${CLOSE_TIMEOUT:-60}"
LIFT_TIMEOUT="${LIFT_TIMEOUT:-60}"

# 容器名支持外部指定（REUSE_SERVER=1 时必须传入与外部一致的 SERVER_NAME）。
SERVER_NAME="${SERVER_NAME:-grasp_${LABEL}_server}"
TELEM_NAME="${TELEM_NAME:-grasp_${LABEL}_telem}"
META_NAME="${META_NAME:-grasp_${LABEL}_meta}"
ACTIONS_NAME="${ACTIONS_NAME:-grasp_${LABEL}_actions}"

IMAGE_SERVER="${IMAGE_SERVER:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"

STAMP="$(date +%Y%m%d_%H%M%S)"
TELEM_REL="debug_data/grasp_evaluation/${LABEL}_${STAMP}"
TELEM_DIR_HOST="$ROOT/baseline/$TELEM_REL"
TELEM_DIR_MNT="/workspace/baseline/$TELEM_REL"
BASELINE="/workspace/baseline"

log() { echo "[$(date +%H:%M:%S)] [$LABEL] $*"; }

# 逐类参数标定支持：把外部传入的 SUPERMARKET_<KIND>_* / SUPERMARKET_* 覆盖项
# 透传给容器（grasp_geometry.py 的 _kind_geometry 会读取它们）。
# 这些 -e 放在脚本自身 -e 之前 —— docker 对同名变量取最后一个 -e，
# 因此脚本显式设置的 SUPERMARKET_TASKS / HEADLESS 等仍然优先。
mapfile -t _OV < <(env | grep -E '^SUPERMARKET_[A-Z0-9_]+=' || true)
ENV_ARGS=()
for _kv in "${_OV[@]}"; do ENV_ARGS+=(-e "$_kv"); done
log "几何覆盖项透传 ${#ENV_ARGS[@]} 个"

log "开始标定：kind=$KIND target=$TARGET_ID slot=$SLOT world=$WORLD headless=$HEADLESS"

# REUSE_SERVER=1 时复用外部已运行的 Server（拨动改朝向类策略必须如此：
# 每次重启 Server 都会因 SUPERMARKET_FIXED_BASELINE=1 重置场景，商品朝向
# 回到初始值，拨动成果全部丢失——v2/v3 已实测）。
if [[ "${REUSE_SERVER:-0}" == "1" ]]; then
  docker rm -f "$TELEM_NAME" "$META_NAME" "$ACTIONS_NAME" >/dev/null 2>&1 || true
else
  docker rm -f "$SERVER_NAME" "$TELEM_NAME" "$META_NAME" "$ACTIONS_NAME" >/dev/null 2>&1 || true
fi

# 容器 Python 是 3.10，会优先读 baseline/__pycache__/*.cpython-310.pyc。
# :ro 挂载只阻止写新缓存，不阻止读旧缓存；源码改了而 .pyc 陈旧时会静默用旧字节码。
find "$ROOT/baseline/__pycache__" -name '*.cpython-310.pyc' -delete 2>/dev/null || true

if [[ "$HEADLESS" != "1" ]]; then
  X11PROBE_LOG=/tmp/x11probe_${LABEL}.log
  : > "$X11PROBE_LOG"
  docker run --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY="$DISPLAY_VALUE" \
    -v "$ROOT/baseline:/workspace/baseline:ro" "$IMAGE_SERVER" \
    bash -lc "python3 -u /workspace/baseline/probe_x11_wslg.py" >> "$X11PROBE_LOG" 2>&1 || true
  if ! grep -q 'X11 握手成功' "$X11PROBE_LOG"; then
    log "X11 自检失败，降级为 headless"
    HEADLESS=1
    MUJOCO_GL_VALUE=osmesa
    DISPLAY_VALUE=""
    cat "$X11PROBE_LOG"
  fi
fi

if [[ "${REUSE_SERVER:-0}" == "1" ]]; then
  if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME" 2>/dev/null)" != "running" ]]; then
    log "REUSE_SERVER=1 但 $SERVER_NAME 未在运行，拒绝继续（避免场景重置）"
    exit 3
  fi
  log "复用已运行 Server：$SERVER_NAME"
else
log "启动 Server"
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
if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME" 2>/dev/null)" != "running" ]]; then
  log "Server 未能保持运行"
  docker logs --tail 60 "$SERVER_NAME" 2>&1 || true
  exit 3
fi
fi

log "启动遥测记录器（裁判 ground-truth）"
mkdir -p "$TELEM_DIR_HOST"
docker run -d --name "$TELEM_NAME" --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$ROOT/baseline:/workspace/baseline:rw" \
  "$IMAGE_CLIENT" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 -u $BASELINE/debug_data/referee_target_monitor.py --output-dir $TELEM_DIR_MNT --duration 900" >/dev/null

run_client() {
  local name="$1"; shift
  docker run --rm --name "$name" --network host --ipc host \
    -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e SUPERMARKET_FIXED_BASELINE=1 -e SUPERMARKET_RANDOMIZE=0 -e SUPERMARKET_USE_GS=0 \
    -v "$ROOT/baseline:/workspace/baseline:ro" \
    "$IMAGE_CLIENT" bash -lc "source /opt/ros/humble/setup.bash && $*"
}

log "safe_pose"
run_client "grasp_${LABEL}_safe_pose" \
  "python3 -u $BASELINE/safe_arm_controller.py safe_pose --execute --confirm safe_pose --timeout $SAFE_POSE_TIMEOUT"

log "导航"
run_client "grasp_${LABEL}_nav" \
  "python3 -u $BASELINE/low_speed_navigator.py --route '$NAV_ROUTE' --max-speed 0.30 --goal-tolerance 0.05 --timeout $NAV_TIMEOUT --ignore-obstacles"

if [[ "$(docker inspect --format '{{.State.Status}}' "$SERVER_NAME" 2>/dev/null)" != "running" ]]; then
  log "Server 在导航后退出"
  exit 4
fi

log "deploy → creep → close_gripper → lift"
# creep 必须带 --object-stop-only：停止判据只依据物体几何（y 停止线），
# 不叠加货架物理前沿 front<=0.40m 门禁。高层货位底盘被货架挡住时 front 恒在
# 0.53m 附近，与名义 y 停止线互斥，会直接死锁到 worker 超时。
docker run --rm --name "$ACTIONS_NAME" --network host --ipc host \
  "${ENV_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$DOMAIN" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DEPLOY_TIMEOUT="$DEPLOY_TIMEOUT" -e CREEP_TIMEOUT="$CREEP_TIMEOUT" \
  -e CLOSE_TIMEOUT="$CLOSE_TIMEOUT" -e LIFT_TIMEOUT="$LIFT_TIMEOUT" \
  -e AMENT_TRACE_SETUP_FILES= -e AMENT_PYTHON_EXECUTABLE= -e COLCON_TRACE= \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  "$IMAGE_CLIENT" bash -lc "
set -eo pipefail
export SUPERMARKET_GRASP_ROT_MODE=$GRASP_ROT_MODE
export SUPERMARKET_TOP_HOVER_DZ=$TOP_HOVER_DZ
export SUPERMARKET_TOP_GRASP_DZ=$TOP_GRASP_DZ
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
if [[ '$GRASP_ROT_MODE' == 'top' ]]; then
  echo 'STAGE creep=skipped（top 俯抓无水平接近，deploy 两段下探已完成）'
elif [[ '$TOLERATE_CREEP_TIMEOUT' == '1' ]]; then
  python3 -u $BASELINE/grasp_creep_controller.py --execute --confirm creep --object-stop-only --pickup-approach --speed $CREEP_SPEED --timeout \$CREEP_TIMEOUT \
    || echo 'STAGE creep=timeout（容忍：拨转后商品在指间，继续 close/lift）'
  echo 'STAGE creep=success'
else
  python3 -u $BASELINE/grasp_creep_controller.py --execute --confirm creep --object-stop-only --pickup-approach --speed $CREEP_SPEED --timeout \$CREEP_TIMEOUT
  echo 'STAGE creep=success'
fi
if [[ '$STOP_AFTER' == 'creep' ]]; then
  echo 'STAGE stopped_after=creep（拨动模式，跳过 close/lift）'
else
python3 -u $BASELINE/close_gripper_controller.py --execute --confirm close_gripper --timeout \$CLOSE_TIMEOUT --hold-seconds 0.8
echo 'STAGE close_gripper=success'
python3 -u $BASELINE/lift_controller.py --execute --confirm lift --timeout \$LIFT_TIMEOUT
echo 'STAGE lift=success'
fi
"

log "动作链结束；保持 $HOLD_SECONDS 秒后清理"
sleep "$HOLD_SECONDS"

log "遥测目录: $TELEM_DIR_HOST"
if [[ "${REUSE_SERVER:-0}" == "1" ]]; then
  docker rm -f "$TELEM_NAME" >/dev/null 2>&1 || true   # 保留 Server 给下一阶段
  log "REUSE 模式：保留 Server $SERVER_NAME"
else
  log "停止 Server"
  docker rm -f "$SERVER_NAME" "$TELEM_NAME" >/dev/null 2>&1 || true
fi
