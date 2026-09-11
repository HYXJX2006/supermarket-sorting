#!/usr/bin/env bash
# Known-slot navigation + plan-only IK integration test.
# The base is moved to the shelf observation pose first; no arm, gripper,
# lift, delivery, or placement command is sent by the plan-only chain.
set -eo pipefail

# ROS setup scripts use unset variables internally.  Source them outside the
# strict unset-variable phase, then restore the safer shell setting.
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${COLCON_TRACE:=}"
set +u
source /opt/ros/humble/setup.bash
set -u

BASELINE_DIR="${BASELINE_DIR:-/workspace/baseline}"
TARGET_ID="${TARGET_ID:-known_product_032}"
KIND="${KIND:-kele}"
SLOT="${SLOT:-D/L2/C2}"
WORLD="${WORLD:-0.92 3.243 0.9235}"
NAV_GOAL="${NAV_GOAL:-1.04 2.60 1.5707963267948966}"
# The final route waypoint is the physical D-shelf observation pose.  Keep
# it separate from the target metadata goal used by downstream planning.
ODOM_GOAL="${ODOM_GOAL:-0.852 2.475 1.5707963267948966}"
NAV_ROUTE="${NAV_ROUTE:-1.92,2.475,1.5707963267948966;0.852,2.475,1.5707963267948966}"
NAV_SPEED="${NAV_SPEED:-0.30}"
NAV_TIMEOUT="${NAV_TIMEOUT:-300}"
ODOM_TIMEOUT="${ODOM_TIMEOUT:-10}"
ODOM_TOLERANCE="${ODOM_TOLERANCE:-0.16}"
read -r WX WY WZ <<< "$WORLD"
read -r GX GY GYAW <<< "$NAV_GOAL"

PLANNER_PID=""
EXECUTOR_PID=""
PUBLISHER_PID=""
cleanup() {
  for pid in "$PUBLISHER_PID" "$EXECUTOR_PID" "$PLANNER_PID"; do
    if [[ -n "$pid" ]]; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

printf '[known-slot] navigating base to D observation pose: route=%s speed=%s\n' "$NAV_ROUTE" "$NAV_SPEED"
python3 -u "$BASELINE_DIR/low_speed_navigator.py" \
  --route "$NAV_ROUTE" \
  --max-speed "$NAV_SPEED" \
  --timeout "$NAV_TIMEOUT" \
  --ignore-obstacles

read -r OX OY OYAW <<< "$ODOM_GOAL"
printf '[known-slot] navigation reached; waiting for odom near (%.3f, %.3f)\n' "$OX" "$OY"
export KNOWN_SLOT_GX="$OX" KNOWN_SLOT_GY="$OY" KNOWN_SLOT_GYAW="$OYAW"
export KNOWN_SLOT_ODOM_TIMEOUT="$ODOM_TIMEOUT" KNOWN_SLOT_ODOM_TOLERANCE="$ODOM_TOLERANCE"
python3 - <<'PY'
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry

x_goal = float(os.environ["KNOWN_SLOT_GX"])
y_goal = float(os.environ["KNOWN_SLOT_GY"])
yaw_goal = float(os.environ["KNOWN_SLOT_GYAW"])
timeout = max(1.0, float(os.environ["KNOWN_SLOT_ODOM_TIMEOUT"]))
tolerance = max(0.02, float(os.environ["KNOWN_SLOT_ODOM_TOLERANCE"]))
state = {"message": None}

rclpy.init()
node = rclpy.create_node("known_slot_odom_gate")

def on_odom(message):
    state["message"] = message

node.create_subscription(Odometry, "/slamware_ros_sdk_server_node/odom", on_odom, 10)
deadline = time.monotonic() + timeout
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        message = state["message"]
        if message is None:
            continue
        pose = message.pose.pose
        distance = math.hypot(float(pose.position.x) - x_goal, float(pose.position.y) - y_goal)
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_error = (yaw_goal - yaw + math.pi) % (2.0 * math.pi) - math.pi
        print(
            f"[known-slot] odom gate: pos=({pose.position.x:.3f},{pose.position.y:.3f}) "
            f"dist={distance:.3f} yaw_err={yaw_error:.3f}",
            flush=True,
        )
        if distance <= tolerance and abs(yaw_error) <= 0.20:
            raise SystemExit(0)
    print("[known-slot] odom did not settle near navigation goal", flush=True)
    raise SystemExit(2)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

printf '[known-slot] odom settled; starting grasp planner and plan-only executor\\n'
python3 -u "$BASELINE_DIR/grasp_planner.py" &
PLANNER_PID=$!
python3 -u "$BASELINE_DIR/single_item_executor.py" \
  --plan-audit \
  --max-items 1 \
  --timeout "${PLAN_TIMEOUT:-120}" \
  --max-attempts 1 \
  --demo-target "{\"target_id\":\"$TARGET_ID\",\"kind\":\"$KIND\",\"slot\":\"$SLOT\",\"object_world\":[$WX,$WY,$WZ],\"navigation_goal\":[$GX,$GY,$GYAW],\"plan_only_allowed\":true,\"local_recheck_required\":false}" &
EXECUTOR_PID=$!
sleep "${PLANNER_STARTUP_DELAY:-1}"
python3 -u "$BASELINE_DIR/known_slot_plan_publisher.py" \
  --target-id "$TARGET_ID" --kind "$KIND" --slot "$SLOT" \
  --world "$WX" "$WY" "$WZ" \
  --navigation-goal "$GX" "$GY" "$GYAW" &
PUBLISHER_PID=$!

wait "$EXECUTOR_PID"
EXECUTOR_RC=$?
printf '[known-slot] plan-only executor exit=%s\\n' "$EXECUTOR_RC"
exit "$EXECUTOR_RC"
