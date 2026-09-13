#!/usr/bin/env bash
# 随机 5 目标自主分拣：一键拉起 WSL 3D 可视化 Server + 9类检测 + 自主执行编排
#   Server：MuJoCo native 画面(USE_GS=0)实时窗口，可见小车/商品/ArUco
#   Client：multiclass_detect + inventory_competition_executor --execute
#   （executor 会自行 spawn 巡游/抓取/配送的 low-level worker）
#
# 用法：
#   bash baseline/run_random5_autopilot.sh
# 常用覆盖：
#   SUPERMARKET_SEED=20260902 CRUISE_MAX_SPEED=1.2 MULTICLASS_WEIGHTS=/path/best.pt
#   SUPERMARKET_HEADLESS=1   # 不想看窗口时（省资源）
set -euo pipefail

ROOT="${ROOT:-/mnt/e/workspace/claude_workplace/揭榜挂帅}"
IMAGE_SERVER="${IMAGE_SERVER:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"
SEED="${SUPERMARKET_SEED:-11}"
DOMAIN="${ROS_DOMAIN_ID:-99}"
CRUISE="${CRUISE_MAX_SPEED:-1.4}"          # 回退：2.2 时急停惯性大，翻车
MAPPING_SPEED="${MAPPING_SPEED:-0.60}"     # 回退：1.00 过猛
OBSTACLE_DIST="${OBSTACLE_DIST:-0.35}"     # 配送段动态避障触发距离（比赛提速）
RANDOMIZE_OBSTACLES="${SUPERMARKET_RANDOMIZE_OBSTACLES:-1}"
REMOVE_CORRIDOR_OBSTACLES="${SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES:-0}"
TASK_COUNT="${SUPERMARKET_TASK_COUNT:-5}"
# 跳巡游扫描（比赛时限 420s，全扫描要 5-8 分钟）：用任务 kind + 45 槽位布局
# 真值直接定位目标货位并立即进入执行。
SKIP_SCAN="${SUPERMARKET_SKIP_SCAN:-0}"   # 暂缓：首个目标会落入 pickup-transit
# 货架横移模式走 5.5m 长距离（龟速 0.005m/s，实测卡死），需改导航层再启用。
# 现阶段用"扫描加速"（巡游 0.7 + 停留 1.5s）把扫描从 5-8 分钟压到 2-3 分钟。
TASKS_SPEC="${SUPERMARKET_TASKS:-}"
SCORE_ENABLED="${SUPERMARKET_ENABLE_SCORE:-1}"
TASK_ORDER_LOCKED="${SUPERMARKET_TASK_ORDER_LOCKED:-0}"
STOP_AFTER_IK="${SUPERMARKET_STOP_AFTER_IK:-0}"
STOP_AFTER_IK_ARG=""
if [[ "$STOP_AFTER_IK" == "1" ]]; then
  STOP_AFTER_IK_ARG=" --stop-after-ik"
fi
STOP_AFTER_CLOSE="${SUPERMARKET_STOP_AFTER_CLOSE:-0}"
STOP_AFTER_CLOSE_ARG=""
if [[ "$STOP_AFTER_CLOSE" == "1" ]]; then
  STOP_AFTER_CLOSE_ARG=" --stop-after-close"
fi
GRIP_THRESHOLD="${SUPERMARKET_GRIP_CLOSED_FEEDBACK_MAX:-0.85}"
SHUPIAN_CREEP_STOP_DY="${SUPERMARKET_SHUPIAN_CREEP_STOP_DY:-0.010}"
SHUPIAN_CREEP_Y_TOL="${SUPERMARKET_SHUPIAN_CREEP_Y_TOL:-0.025}"
HEADLESS="${SUPERMARKET_HEADLESS:-0}"      # 0=显示 X11 窗口
USE_GS="${SUPERMARKET_USE_GS:-1}"   # 1=3DGS 渲染（走 GS 管线，绕开崩过的原生 mjr_render）；相机图像可用          # 1=3DGS实时画面；0=MuJoCo native
GS_BATCH_RENDER="${SUPERMARKET_GS_BATCH_RENDER:-1}"
RENDER_FPS="${SUPERMARKET_RENDER_FPS:-6}"
GS_HEAD_ONLY="${SUPERMARKET_GS_HEAD_ONLY:-1}"   # 1=仅头部+第三人称走 GS；开 0(手眼也 GS) 会把异步渲染线程压满，相机帧率崩溃导致检测凑不齐槽位（实测 observed=0/45）
GS_ASYNC="${SUPERMARKET_GS_ASYNC:-1}"          # 1=异步渲染保帧率；第三人称已由 simulator 补丁纳入批量（两全）
ARM_PREGRASP_Y="${SUPERMARKET_ARM_PREGRASP_Y:-2.35}"
ARM_ODOM_STABLE_SAMPLES="${SUPERMARKET_ARM_ODOM_STABLE_SAMPLES:-5}"
PLAN_GATE_TIMEOUT="${SUPERMARKET_PLAN_GATE_TIMEOUT:-60}"
# 检测器允许较低分数进入跟踪；地图仍由 executor 的 min-confidence + 多帧稳定门禁控制。
DETECTION_CONFIDENCE="${SUPERMARKET_DETECTION_CONFIDENCE:-0.15}"
MAP_CONFIDENCE="${SUPERMARKET_MAP_CONFIDENCE:-0.25}"
# X11/VcXsrv：不使用 WSLg 的 :0 或 /tmp/.X11-unix。默认取 WSL NAT
# 网关地址作为 Windows 宿主机；优先使用 WSL 默认路由网关，
# /etc/resolv.conf 中的 nameserver 只是 DNS，不一定能承载 X11。
# 也可显式传 X11_DISPLAY=192.168.x.x:0.0。
# X11 显示：默认走 WSLg 的 unix socket（:0 + /tmp/.X11-unix 挂载）——
# 与 run_item_grasp.sh 同款，实测可靠。此前默认 VcXsrv TCP（NAT 网关:0.0）
# 在 MUJOCO_GL=glfw 下 GLX 握手失败（gladLoadGL error → Server 崩、odom 断流、
# executor 死等建图）。如坚持 VcXsrv 可显式传 X11_DISPLAY=192.168.x.x:0.0。
# 默认走 VcXsrv :1（TCP），不用 WSLg :0：
#   WSLg 的 RDP 共享内存通道损坏（weston 日志 rdp_allocate_shared_memory
#   I/O error）→ 窗口变成 [WARN:COPY MODE]，渲染不出来。
#   VcXsrv 用 :1 是因为 :0 已被 WSLg 占用（抢 :0 会静默绑定失败）。
# 启动前需在 Windows 侧运行 start_x.bat（VcXsrv :1 + 防火墙规则）。
X11_DISPLAY="${X11_DISPLAY:-172.25.192.1:1}"
SERVER_NAME="random5_autopilot_server"
CLIENT_NAME="random5_autopilot_client"

# 检测权重：默认使用 v6（v4 + 手眼伺服时代的混淆专项微调，修复
# chengzi/kouxiangtang/maidong 分类混淆）；v4 为回退。仍可通过
# MULTICLASS_WEIGHTS 显式覆盖。
if [[ -z "${MULTICLASS_WEIGHTS:-}" ]]; then
  PREFERRED_V6="$ROOT/baseline/debug_data/multiclass_train_v6_servo/confusion_finetune/weights/best.pt"
  if [[ -f "$PREFERRED_V6" ]]; then
    MULTICLASS_WEIGHTS="$PREFERRED_V6"
  else
  PREFERRED_V4="$ROOT/baseline/debug_data/multiclass_train_formal_v4/multiclass_random_finetune/weights/best.pt"
  if [[ -f "$PREFERRED_V4" ]]; then
    MULTICLASS_WEIGHTS="$PREFERRED_V4"
  else
    LATEST_FORMAL=$(find "$ROOT/baseline/debug_data" -type f -path '*/multiclass_train_formal_v4*/weights/best.pt' -printf '%T@ %p\n' 2>/dev/null | sort -nr | cut -d' ' -f2- | head -n 1 || true)
    LATEST_ANY=$(find "$ROOT/baseline/debug_data" -type f -path '*/multiclass*weights/best.pt' ! -path '*/multiclass_train_formal_v5/*' -printf '%T@ %p\n' 2>/dev/null | sort -nr | cut -d' ' -f2- | head -n 1 || true)
    if [[ -n "$LATEST_FORMAL" ]]; then
      MULTICLASS_WEIGHTS="$LATEST_FORMAL"
    elif [[ -n "$LATEST_ANY" ]]; then
      MULTICLASS_WEIGHTS="$LATEST_ANY"
    else
      MULTICLASS_WEIGHTS="/workspace/baseline/debug_data/multiclass_train_smoke/multiclass_smoke/weights/best.pt"
    fi
  fi
  fi
fi
# 挂载路径默认指向容器内 /workspace/baseline（MULTICLASS_WEIGHTS 若含该前缀则直接使用）
if [[ "$MULTICLASS_WEIGHTS" != /workspace/baseline/* ]]; then
  MULTICLASS_WEIGHTS="/workspace/baseline/${MULTICLASS_WEIGHTS#$ROOT/baseline/}"
fi

unset SUPERMARKET_FIXED_BASELINE SUPERMARKET_FIXED_TARGET_SLOT \
  SUPERMARKET_FIXED_TARGET_WORLD SUPERMARKET_FIXED_TARGET_WORLD_TOLERANCE || true

echo "==> 清理旧容器"
# 必须连裁判遥测容器一起清：漏了它会以 "container name already in use"
# 直接掐死脚本（本轮实测踩到），而 server 已经起来了 → 后面全空等。
docker rm -f "$SERVER_NAME" "$CLIENT_NAME" "${SERVER_NAME}_telem" >/dev/null 2>&1 || true

echo "# 容器 Python 是 3.10，会优先读 baseline/__pycache__/*.cpython-310.pyc。
# :ro 挂载只阻止写新缓存，不阻止读旧缓存；源码改了而 .pyc 陈旧时会静默用旧
# 字节码——夹空检测/门禁修复曾因此完全不生效（lift 照跑旧逻辑）。
find "$ROOT/baseline/__pycache__" -name '*.cpython-310.pyc' -delete 2>/dev/null || true

==> 启动 3D 可视化 Server (X11=$X11_DISPLAY, USE_GS=$USE_GS, HEADLESS=$HEADLESS)"
docker run -d --name "$SERVER_NAME" --gpus all --network host --ipc host \
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
  -e SUPERMARKET_TASK_COUNT="$TASK_COUNT" \
  -e SUPERMARKET_EXPECTED_TARGET_COUNT="$TASK_COUNT" \
  -e SUPERMARKET_TASK_ORDER_LOCKED="$TASK_ORDER_LOCKED" \
  -e MAX_JOBS="${MAX_JOBS:-2}" \
  -e TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}" \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  -v supermarket_sorting_cache:/root/.cache \
  -v "$ROOT/baseline:/workspace/baseline:ro" \
  -v "$ROOT/baseline/patches/discoverse/envs/simulator.py:/workspace/supermarket_sorting_task/discoverse/envs/simulator.py:ro" \
  -v "$ROOT/baseline/patches/examples/ros2/mmk2_ros2.py:/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py:ro" \
  "$IMAGE_SERVER" bash -lc \
  'cd /workspace/supermarket_sorting_task && source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/random_server_bootstrap.py'

TELEM_STAMP="$(date +%Y%m%d_%H%M%S)"
TELEM_REL="debug_data/random5_telem/${TELEM_STAMP}_seed${SEED}"
mkdir -p "$ROOT/baseline/debug_data/random5_telem"
echo "==> 启动裁判遥测记录器（商品侧 ground truth）：$TELEM_REL"
docker run -d --name "${SERVER_NAME}_telem" --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$ROOT/baseline:/workspace/baseline:rw" \
  "$IMAGE_CLIENT" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 -u /workspace/baseline/debug_data/referee_target_monitor.py --output-dir /workspace/baseline/$TELEM_REL --duration 1500"

echo "==> 启动 Client：检测 + 自主执行（executor 自动 spawn 巡游/抓取/配送 worker）"
# 现场调参透传：把调用者环境里所有 SUPERMARKET_* 一次性塞进容器。
# 此前只显式传了固定几个 -e，导致 launch_comp.sh 的 export 到不了容器，
# 调参看着"没生效"（实测 ARM_LATERAL_OFFSET=-0.20 只落地了默认 -2cm）。
# 同名 -e 后面的会覆盖前面，所以已有的显式项不受影响。
PASS_ENV=()
while IFS= read -r _kv; do
  [ -n "$_kv" ] && PASS_ENV+=(-e "$_kv")
done < <(env | grep -E '^SUPERMARKET_[A-Z0-9_]+=' || true)
echo "==> 透传 SUPERMARKET_* 调参项 ${#PASS_ENV[@]} 个"
docker run -d --name "$CLIENT_NAME" --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e SUPERMARKET_SEED="$SEED" \
  -e SUPERMARKET_RANDOMIZE_OBSTACLES="$RANDOMIZE_OBSTACLES" \
  -e SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES="$REMOVE_CORRIDOR_OBSTACLES" \
  -e SUPERMARKET_OBSTACLE_SEED="${SUPERMARKET_OBSTACLE_SEED:-$((SEED + 1000003))}" \
  -e SUPERMARKET_TASK_ORDER_LOCKED="$TASK_ORDER_LOCKED" \
  -e SUPERMARKET_GRIP_CLOSED_FEEDBACK_MAX="$GRIP_THRESHOLD" \
  -e SUPERMARKET_SHUPIAN_CREEP_STOP_DY="$SHUPIAN_CREEP_STOP_DY" \
  -e SUPERMARKET_SHUPIAN_CREEP_Y_TOL="$SHUPIAN_CREEP_Y_TOL" \
  -e SUPERMARKET_ARM_PREGRASP_Y="$ARM_PREGRASP_Y" \
  -e SUPERMARKET_ARM_ODOM_STABLE_SAMPLES="$ARM_ODOM_STABLE_SAMPLES" \
  -e SUPERMARKET_PLAN_GATE_TIMEOUT="$PLAN_GATE_TIMEOUT" \
  -e SUPERMARKET_ARM_LATERAL_OFFSET_M="${SUPERMARKET_ARM_LATERAL_OFFSET_M:-}" \
  -e SUPERMARKET_GRASP_Z_OFFSET_M="${SUPERMARKET_GRASP_Z_OFFSET_M:-}" \
  -e SUPERMARKET_SERVO_LATERAL_GAIN="${SUPERMARKET_SERVO_LATERAL_GAIN:-}" \
  -e SUPERMARKET_PITCH_DWELL_S="${SUPERMARKET_PITCH_DWELL_S:-1.0}" \
  -e SUPERMARKET_SKIP_SCAN="$SKIP_SCAN" \
  -e DISPLAY="$X11_DISPLAY" \
  -e MUJOCO_GL="${MUJOCO_GL:-glfw}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT="${LIBGL_ALWAYS_INDIRECT:-0}" \
  -e MAX_JOBS="${MAX_JOBS:-2}" \
  -e TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}" \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  "${PASS_ENV[@]}" \
  -v supermarket_sorting_cache:/root/.cache \
  -v "$ROOT/baseline:/workspace/baseline:rw" \
  "$IMAGE_CLIENT" bash -lc "
set -e
source /opt/ros/humble/setup.bash
python3 -u /workspace/baseline/multiclass_detect.py \
    --weights '$MULTICLASS_WEIGHTS' --device '${DEVICE:-cuda}' --confidence $DETECTION_CONFIDENCE &
DET_PID=\$!
trap 'kill \$DET_PID 2>/dev/null || true' EXIT
READY=0
for i in \$(seq 1 60); do
  if ! kill -0 \$DET_PID 2>/dev/null; then
    echo '检测器提前退出，拒绝启动编排器' >&2
    exit 1
  fi
  if ros2 topic list --no-daemon 2>/dev/null | grep -qx '/multiclass/detections'; then
    READY=1
    echo '检测器就绪：/multiclass/detections'
    break
  fi
  sleep 1
done
if [[ \$READY -ne 1 ]]; then
  echo '检测器未在 60 秒内创建 /multiclass/detections，拒绝启动编排器' >&2
  exit 1
fi
python3 -u /workspace/baseline/inventory_competition_executor.py \
    --execute --confirm random5${STOP_AFTER_IK_ARG}${STOP_AFTER_CLOSE_ARG} \
    --map-timeout ${MAP_TIMEOUT:-240} \
    --mapping-max-speed $MAPPING_SPEED \
    --cruise-max-speed $CRUISE \
    --obstacle-distance $OBSTACLE_DIST \
    --min-samples 5 \
    --min-confidence $MAP_CONFIDENCE \
    --map-path /workspace/baseline/debug_data/inventory_map_autopilot.json
"

echo "================================================================"
echo "Server 容器: $SERVER_NAME   Client 容器: $CLIENT_NAME"
echo "检测权重: $MULTICLASS_WEIGHTS"
echo "安全停止到 IK: $STOP_AFTER_IK   close后停车: $STOP_AFTER_CLOSE   目标复核: 已删除   扫描停留: ${SUPERMARKET_PITCH_DWELL_S:-4.0}s/层   巡航速度: $CRUISE m/s   巡游: $MAPPING_SPEED   机械臂安全线Y: $ARM_PREGRASP_Y   Odom稳定帧: $ARM_ODOM_STABLE_SAMPLES   Plan-only超时: $PLAN_GATE_TIMEOUT s   避障距离: $OBSTACLE_DIST   指定任务: ${TASKS_SPEC:-随机}   随机障碍: $RANDOMIZE_OBSTACLES   移除走廊障碍: $REMOVE_CORRIDOR_OBSTACLES   3DGS批量渲染: $GS_BATCH_RENDER   3DGS头部单路: $GS_HEAD_ONLY   异步GS: $GS_ASYNC   渲染FPS: $RENDER_FPS   任务数: $TASK_COUNT   裁判计分: $SCORE_ENABLED   顺序锁定: $TASK_ORDER_LOCKED   闭爪阈值: $GRIP_THRESHOLD   薯片creep_dy: $SHUPIAN_CREEP_STOP_DY   薯片Y容差: $SHUPIAN_CREEP_Y_TOL"
echo "  - X11 窗口应由 VcXsrv/XLaunch 承载：DISPLAY=$X11_DISPLAY"
echo "  - docker logs -f $CLIENT_NAME   查看编排/动作日志"
echo "  - docker logs -f $SERVER_NAME   查看 Server/任务日志"
echo "  - 停止: docker rm -f $SERVER_NAME $CLIENT_NAME"
echo "================================================================"
docker logs -f "$CLIENT_NAME"
