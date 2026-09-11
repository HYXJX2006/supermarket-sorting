#!/usr/bin/env python3
"""DG-202606 单次 creep 接近控制器。

只控制底盘沿当前朝向低速前进到商品前方的停止线。
默认 plan-only；真实执行必须 --execute --confirm creep。
禁止闭合夹爪、抬升、撤出和配送。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String

from grasp_geometry import GraspGeometry, geometry_for_kind

TASK_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
# 真实动作 worker 在 Client 镜像中运行；该镜像默认没有 Server 的
# /workspace/supermarket_sorting_task。优先使用镜像内官方路径，缺失时
# 回退到随 baseline 一起挂载的官方源码副本，避免 deploy/creep 因导入失败退出。
if not (TASK_DIR / "mmk2_kdl.py").is_file():
    bundled_task_dir = Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"
    if (bundled_task_dir / "mmk2_kdl.py").is_file():
        TASK_DIR = bundled_task_dir
sys.path.insert(0, str(TASK_DIR))
from mmk2_kdl import MMK2Kdl  # noqa: E402

META_TOPIC = "/competition/grasp_goal_meta"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
CMD_TOPIC = "/cmd_vel"
STATUS_TOPIC = "/competition/creep_status"

CREEP_SPEED = 0.12
MAX_ANGULAR = 0.35
# 仅用于最终位置比较，吸收仿真浮点/消息离散误差；不改变实际停车线。
CREEP_Y_COMPARISON_EPSILON = 0.001
MIN_FRONT_CLEARANCE = 0.28
# In pickup-approach mode the shelf itself may stop the base before the
# nominal world-y stop line. Treat that as a valid physical stop only when
# the remaining end-effector distance is small and x/z alignment is checked.
PICKUP_PHYSICAL_STOP_FRONT = float(
    os.environ.get("SUPERMARKET_PICKUP_PHYSICAL_STOP_FRONT", "0.40")
)
PICKUP_PHYSICAL_STOP_MAX_REMAINING = float(
    os.environ.get("SUPERMARKET_PICKUP_PHYSICAL_STOP_MAX_REMAINING", "0.10")
)
PICKUP_PHYSICAL_STOP_Y_TOLERANCE = float(
    os.environ.get("SUPERMARKET_PICKUP_PHYSICAL_STOP_Y_TOLERANCE", "0.22")
)


def pickup_physical_stop_ready(
    *, pickup_approach: bool, front_min: float | None, remaining_distance: float
) -> bool:
    """Return whether the shelf's physical front edge can end creep safely."""
    return (
        pickup_approach
        and front_min is not None
        and front_min <= PICKUP_PHYSICAL_STOP_FRONT
        and 0.0 < remaining_distance <= PICKUP_PHYSICAL_STOP_MAX_REMAINING
    )


def wrap_to_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CreepController(Node):
    def __init__(
        self,
        execute: bool,
        confirm: str,
        timeout: float,
        speed: float,
        pickup_approach: bool = False,
        object_stop_only: bool = False,
    ) -> None:
        super().__init__("grasp_creep_controller")
        self.execute_enabled = execute and confirm == "creep"
        self.rejected = execute and confirm != "creep"
        self.timeout = max(1.0, float(timeout))
        self.speed = max(0.01, min(0.3, float(speed)))
        self.pickup_approach = bool(pickup_approach)
        self.object_stop_only = bool(object_stop_only)
        self.started_at = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.success = False
        self.target_id = ""
        self.kind = ""
        self.geometry: GraspGeometry = geometry_for_kind(None)
        self.object_y: float | None = None
        self.stop_y: float | None = None
        self.object_world: tuple[float, float, float] | None = None
        self.object_center_world: tuple[float, float, float] | None = None
        self.creep_target_world: tuple[float, float, float] | None = None
        self.ee_world: tuple[float, float, float] | None = None
        self.kdl = MMK2Kdl()
        self.joints: dict[str, float] = {}
        self.joint_state_seen = False
        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.front_min: float | None = None
        self.last_scan_at = 0.0
        self.goal_received = False

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"CREEP 控制器 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'} "
            f"pickup_approach={'on' if self.pickup_approach else 'off'} "
            f"object_stop_only={'on' if self.object_stop_only else 'off'}；"
            "只控制底盘低速接近，不控制机械臂和夹爪"
        )

    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            target_id = str(payload.get("target_id", ""))
            world = payload.get("object_world")
            if not target_id or not isinstance(world, list) or len(world) != 3:
                return
            self.object_world = (float(world[0]), float(world[1]), float(world[2]))
            self.kind = str(payload.get("kind", ""))
            self.geometry = geometry_for_kind(self.kind)
            yaw = self.yaw if self.yaw is not None else math.pi / 2.0
            forward = (math.cos(yaw), math.sin(yaw), 0.0)
            self.object_center_world = tuple(
                self.object_world[index] + self.geometry.surface_to_center_fwd * forward[index]
                for index in range(3)
            )
            deploy_offset = self.geometry.deploy_offset
            object_y = self.object_center_world[1]
            self.creep_target_world = (
                self.object_center_world[0] + deploy_offset[0],
                object_y + self.geometry.creep_stop_dy,
                self.object_center_world[2] + deploy_offset[2],
            )
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        if target_id != self.target_id:
            self.target_id = target_id
            self.object_y = object_y
            self.stop_y = self.creep_target_world[1] if self.creep_target_world else None
            self.goal_received = True
            self.get_logger().info(
                f"收到 creep 目标：target={self.target_id} kind={self.kind} stop_y={self.stop_y:.4f}"
            )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quaternion(pose.orientation)
        self._update_ee_world()

    def _on_joints(self, message: JointState) -> None:
        self.joint_state_seen = True
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }
        self._update_ee_world()

    def _footprint_to_world(self, point) -> tuple[float, float, float] | None:
        if self.x is None or self.y is None or self.yaw is None:
            return None
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (
            self.x + c * float(point[0]) - s * float(point[1]),
            self.y + s * float(point[0]) + c * float(point[1]),
            float(point[2]),
        )

    def _update_ee_world(self) -> None:
        if not self.joint_state_seen:
            self.ee_world = None
            return
        names = [f"right_arm_joint{i}" for i in range(1, 7)]
        if "slide_joint" not in self.joints or any(name not in self.joints for name in names):
            self.ee_world = None
            return
        q = [self.joints["slide_joint"]] + [self.joints[name] for name in names]
        try:
            _, transform = self.kdl.forward_kinematics(q, index="right")
            self.ee_world = self._footprint_to_world(transform[:3, 3])
        except Exception as exc:
            self.ee_world = None
            self.get_logger().warning(f"无法计算右臂末端世界坐标：{exc}")

    def _on_scan(self, message: LaserScan) -> None:
        values = [float(r) for r in message.ranges if math.isfinite(r) and r > 0.0]
        self.front_min = min(values) if values else None
        self.last_scan_at = time.monotonic()

    def _publish_status(self, state: str, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": 1,
                "state": state,
                "reason": reason,
                "target_id": self.target_id,
                "kind": self.kind,
                "stop_y": self.stop_y,
                "object_center_world": self.object_center_world,
                "creep_target_world": self.creep_target_world,
                "ee_world": self.ee_world,
                "grasp_geometry": self.geometry.to_dict(),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    def _stop(self, reason: str, success: bool = False) -> None:
        self.cmd_pub.publish(Twist())
        self.success = success
        self._publish_status("reached" if success else "failed", reason)
        self.get_logger().info(f"CREEP 停止：{reason}")
        self.done = True

    def _tick(self) -> None:
        if self.done:
            self.cmd_pub.publish(Twist())
            return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm creep")
            self.done = True
            return
        if time.monotonic() - self.started_at > self.timeout:
            self._stop("超时，自动停车")
            return
        if not self.goal_received or self.stop_y is None:
            self.cmd_pub.publish(Twist())
            self._publish_status("waiting", "等待 grasp_goal_meta")
            self._periodic_log("等待抓取目标，保持停车")
            return
        if self.x is None or self.y is None or self.yaw is None:
            self.cmd_pub.publish(Twist())
            self._periodic_log("等待 odom，保持停车")
            return
        if not self.pickup_approach:
            if time.monotonic() - self.last_scan_at > 1.0 or self.front_min is None:
                self.cmd_pub.publish(Twist())
                self._periodic_log("LaserScan 超时或无有效距离，保持停车")
                return
            if self.front_min < MIN_FRONT_CLEARANCE:
                self._stop(f"前方障碍过近 front={self.front_min:.3f}m，自动停车")
                return

        if self.ee_world is None:
            self.cmd_pub.publish(Twist())
            self._periodic_log("等待 joint_states/右臂末端坐标，保持停车")
            return

        # 官方动作骨架以已伸出的右臂末端 ee 为 creep 参考，
        # 不能让底盘本体追到商品的世界 y 坐标。
        distance = self.stop_y - self.ee_world[1]
        physical_front_stop = (
            False
            if self.object_stop_only
            else pickup_physical_stop_ready(
                pickup_approach=self.pickup_approach,
                front_min=self.front_min,
                remaining_distance=distance,
            )
        )
        if distance <= self.geometry.creep_position_tolerance[1] or physical_front_stop:
            # 到达 y 停止线时先完成底盘朝向对齐。若此时直接检查
            # 末端 x/z，底盘的残余偏航会被误判为抓取横向偏差。
            heading_error = wrap_to_pi(math.pi / 2.0 - self.yaw)
            if abs(heading_error) > 0.06:
                if not self.execute_enabled:
                    self.cmd_pub.publish(Twist())
                    self._periodic_log(
                        f"PLAN-ONLY creep：已到 y 停止线，等待朝向对齐 err={heading_error:.3f}"
                    )
                    return
                command = Twist()
                command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 2.0 * heading_error))
                self.cmd_pub.publish(command)
                self._periodic_log(
                    f"CREEP 已到 y 停止线，先对齐朝向 err={heading_error:.3f}"
                )
                return
            target = self.creep_target_world
            if target is None:
                self._stop("缺少 creep 三轴目标坐标")
                return
            errors = tuple(float(target[index] - self.ee_world[index]) for index in range(3))
            creep_tolerances = self.geometry.creep_position_tolerance
            # deploy 已经用相同目标姿态做过 FK 验证；creep 不应再
            # 用更严的 x/z 门限否定同一副已到位的机械臂。
            y_tolerance = (
                PICKUP_PHYSICAL_STOP_Y_TOLERANCE
                if physical_front_stop
                else creep_tolerances[1]
            )
            tolerances = (
                # 仿真底盘/末端反馈存在毫米级离散误差；保留
                # deploy 的安全门槛，同时避免 0.066m 这类 1mm 边界值
                # 把已正确到位的目标误判为失败。
                max(creep_tolerances[0], self.geometry.deploy_position_tolerance, 0.070),
                y_tolerance,
                max(creep_tolerances[2], self.geometry.deploy_position_tolerance),
            )
            if (
                abs(errors[0]) > tolerances[0]
                or abs(errors[1]) > tolerances[1] + CREEP_Y_COMPARISON_EPSILON
                or abs(errors[2]) > tolerances[2]
            ):
                self._stop(
                    ("货架物理前沿停止但末端未对准：" if physical_front_stop else "到达 y 停止线但末端未对准：")
                    + f"error_xyz=({errors[0]:.3f},{errors[1]:.3f},{errors[2]:.3f})m "
                    + f"tol_xyz=({tolerances[0]:.3f},{tolerances[1]:.3f},{tolerances[2]:.3f})m"
                )
                return
            self._stop(
                (
                    "已到货架物理前沿停止位"
                    if physical_front_stop
                    else "已到 creep 三轴停止线"
                )
                + f" error_xyz=({errors[0]:.3f},{errors[1]:.3f},{errors[2]:.3f})m",
                success=True,
            )
            return
        if not self.execute_enabled:
            self.cmd_pub.publish(Twist())
            self._periodic_log(
                f"PLAN-ONLY creep：ee_y={self.ee_world[1]:.3f}，停止线={self.stop_y:.3f}，距离={distance:.3f}"
            )
            return

        command = Twist()
        # 接近停止线时保留一个很小但可克服仿真底盘静摩擦的速度；
        # 否则速度随距离降到约 0.02m/s 后会停滞，直到 worker 超时。
        command.linear.x = min(self.speed, max(0.035, 0.5 * distance))
        command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 2.0 * wrap_to_pi(math.pi / 2.0 - self.yaw)))
        self.cmd_pub.publish(command)
        front_text = f"{self.front_min:.3f}" if self.front_min is not None else "ignored"
        self._periodic_log(
            f"CREEP 前进 base_y={self.y:.3f} ee_y={self.ee_world[1]:.3f} "
            f"stop={self.stop_y:.3f} dist={distance:.3f} front={front_text}"
        )

    def _periodic_log(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(text)
            self.last_log = now


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 creep 接近测试")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--speed", type=float, default=CREEP_SPEED, help="creep 速度上限（m/s），默认 0.12")
    parser.add_argument("--pickup-approach", action="store_true",
                        help="取货 creep 阶段忽略货架触发的 LaserScan 停车，仅按末端停止线停车")
    parser.add_argument("--object-stop-only", action="store_true",
                        help="取货阶段不使用货架前沿提前停车，只按物体停止线停车")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    if args.execute and args.confirm != "creep":
        print("拒绝执行：必须使用 --confirm creep")
        return 2
    rclpy.init()
    node = CreepController(
        args.execute,
        args.confirm,
        args.timeout,
        args.speed,
        args.pickup_approach,
        args.object_stop_only,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        result = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())
