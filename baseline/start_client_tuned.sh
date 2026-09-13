#!/usr/bin/env bash
# 手动起 client（带现场调参），用于绕开 run_random5_autopilot.sh 的启动链路做参数验证。
# 用法：bash start_client_tuned.sh [LATERAL] [ZOFF] [GAIN]
set -u

LATERAL="${1:--0.2}"
ZOFF="${2:--0.02}"
GAIN="${3:-1.25}"

ROOT=/mnt/e/workspace/claude_workplace/揭榜挂帅
IMAGE=crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client
WEIGHTS=/workspace/baseline/debug_data/multiclass_train_v6_servo/confusion_finetune/weights/best.pt
NAME=random5_autopilot_client

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID=99 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e SUPERMARKET_ARM_LATERAL_OFFSET_M="$LATERAL" \
  -e SUPERMARKET_GRASP_Z_OFFSET_M="$ZOFF" \
  -e SUPERMARKET_SERVO_LATERAL_GAIN="$GAIN" \
  -e DISPLAY=172.25.192.1:1 \
  -e MUJOCO_GL=glfw \
  -e MAX_JOBS=2 \
  -e TORCH_CUDA_ARCH_LIST=8.9 \
  -e TORCH_EXTENSIONS_DIR=/root/.cache/torch_extensions \
  -v supermarket_sorting_cache:/root/.cache \
  -v "$ROOT/baseline:/workspace/baseline:rw" \
  "$IMAGE" bash -lc "
source /opt/ros/humble/setup.bash
python3 -u /workspace/baseline/multiclass_detect.py --weights $WEIGHTS --device cuda --confidence 0.15 &
DET=\$!
trap 'kill \$DET 2>/dev/null || true' EXIT
for i in \$(seq 1 60); do
  ros2 topic list --no-daemon 2>/dev/null | grep -qx /multiclass/detections && break
  sleep 1
done
python3 -u /workspace/baseline/inventory_competition_executor.py --execute --confirm random5 --map-timeout 240 \
  --mapping-max-speed 0.60 --cruise-max-speed 1.4 --obstacle-distance 0.35 \
  --min-samples 5 --min-confidence 0.25 \
  --map-path /workspace/baseline/debug_data/inventory_map_autopilot.json
"
sleep 15
echo "容器内 LATERAL 变量 = $(docker exec "$NAME" printenv SUPERMARKET_ARM_LATERAL_OFFSET_M)"
