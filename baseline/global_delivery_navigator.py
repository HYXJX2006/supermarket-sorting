#!/usr/bin/env python3
"""DG-202606 多航点配送导航器（分阶段绕障版）。

默认 plan-only；真实执行必须 --execute --confirm deliver_nav_global。
每个航点使用：对准 -> 前进 -> 障碍绕行 -> 恢复朝向 -> 到达切换。
不控制机械臂、夹爪和升降柱。
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
STATUS_TOPIC = "/competition/delivery_global_status"
DEFAULT_ROUTE = [[1.92, 1.90], [1.92, 2.475], [-0.50, 2.475], [-0.50, -0.70], [-0.90, -0.70], [-0.90, -2.80], [-1.88, -2.80]]
FINAL_YAW = -math.pi / 2.0

MAX_SPEED = 0.08
MAX_ANGULAR = 0.45
FRONT_BLOCKED = 0.45
FRONT_CLEAR = 0.60
DETOUR_FRONT_STOP = 0.32
SIDE_CLEARANCE = 0.38
MIN_SIDE_CLEARANCE = 0.30
WAYPOINT_TOLERANCE = 0.12
FINAL_WAYPOINT_TOLERANCE = 0.30
FINAL_YAW_TOLERANCE = 0.10
DETOUR_DISTANCE = 0.50
DETOUR_HEADING_TOLERANCE = 0.12
RECOVER_HEADING_TOLERANCE = 0.12
NO_PROGRESS_SECONDS = 8.0
DETOUR_COOLDOWN_SECONDS = 2.5
DETOUR_MAX_SECONDS = 60.0
DETOUR_BLOCKED_GRACE_SECONDS = 2.0
DETOUR_BACKUP_SPEED = 0.06
MAX_DETOURS_PER_WAYPOINT = 6


def wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


class GlobalDeliveryNavigator(Node):
    def __init__(
        self,
        execute: bool,
        confirm: str,
        timeout: float,
        route: list[list[float]],
        final_yaw: float = FINAL_YAW,
        ignore_obstacles: bool = False,
    ) -> None:
        super().__init__("global_delivery_navigator")
        self.execute_enabled = execute and confirm == "deliver_nav_global"
        self.rejected = execute and confirm != "deliver_nav_global"
        self.ignore_obstacles = bool(ignore_obstacles)
        self.timeout = max(1.0, float(timeout))
        self.route = [(float(x), float(y)) for x, y in route]
        self.final_yaw = float(final_yaw)
        self.started = time.monotonic()
        self.last_log = 0.0
        self.last_progress_at = self.started
        self.last_distance = float("inf")
        self.done = False
        self.success = False
        self.state = "align"
        self.waypoint_index = 0
        self.detour_side = 0
        self.detour_heading = 0.0
        self.detour_start: Pose | None = None
        self.detour_started_at = 0.0
        self.detour_count = 0
        self.detour_cooldown_until = 0.0
        self.detour_blocked_since = 0.0
        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.front: float | None = None
        self.left: float | None = None
        self.right: float | None = None
        self.last_scan = 0.0

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"全局配送导航 mode={'EXECUTE' if self.execute_enabled else 'PLAN-ONLY'} "
            f"waypoints={len(self.route)} ignore_obstacles={'on' if self.ignore_obstacles else 'off'}"
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quaternion(pose.orientation)

    def _on_scan(self, message: LaserScan) -> None:
        ranges = list(message.ranges)
        angle_min = float(message.angle_min)
        increment = float(message.angle_increment)

        def sector(center: float, half_width: float) -> float | None:
            values = []
            for index, distance in enumerate(ranges):
                angle = wrap(angle_min + index * increment)
                if abs(wrap(angle - center)) <= half_width:
                    if math.isfinite(distance) and distance > 0.0:
                        values.append(float(distance))
            return min(values) if values else None

        self.front = sector(0.0, math.radians(24.0))
        self.left = sector(math.pi / 2.0, math.radians(50.0))
        self.right = sector(-math.pi / 2.0, math.radians(50.0))
        self.last_scan = time.monotonic()

    def _status(self, state: str, reason: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "state": state,
                "reason": reason,
                "controller_state": self.state,
                "waypoint_index": self.waypoint_index,
                "waypoint_count": len(self.route),
                "detour_count": self.detour_count,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _publish(self, command: Twist, text: str) -> None:
        if not self.execute_enabled:
            command = Twist()
        self.cmd_pub.publish(command)
        self._log(text)

    def _stop(self, reason: str, success: bool = False) -> None:
        self.cmd_pub.publish(Twist())
        self.success = success
        self._status("reached" if success else "failed", reason)
        self.get_logger().info(f"全局配送导航停止：{reason}")
        self.done = True

    def _current_goal(self) -> tuple[float, float] | None:
        if self.waypoint_index >= len(self.route):
            return None
        return self.route[self.waypoint_index]

    def _distance_to_goal(self) -> float:
        goal = self._current_goal()
        if goal is None or self.x is None or self.y is None:
            return 0.0
        return math.hypot(goal[0] - self.x, goal[1] - self.y)

    def _begin_detour(self, reason: str, *, force_opposite: bool = False) -> bool:
        now = time.monotonic()
        if now < self.detour_cooldown_until:
            self._publish(Twist(), f"绕障冷却中，保持停车剩余={self.detour_cooldown_until - now:.1f}s")
            return False
        if self.detour_count >= MAX_DETOURS_PER_WAYPOINT:
            self._stop(f"当前航点绕障次数超过上限{MAX_DETOURS_PER_WAYPOINT}")
            return False

        left = self.left if self.left is not None else 0.0
        right = self.right if self.right is not None else 0.0
        if max(left, right) < MIN_SIDE_CLEARANCE:
            self._stop(f"绕障失败：左右侧均过窄 left={left:.3f}m right={right:.3f}m")
            return False

        # 选择更宽的一侧；若当前侧前方持续堵塞，强制换到相反侧。
        if force_opposite and self.detour_side in {-1, 1}:
            side = -self.detour_side
        elif abs(left - right) < 0.08:
            # 同一航点第二次绕障时换边，避免在对称障碍前反复走同一条死路。
            side = -self.detour_side if self.detour_count > 0 and self.detour_side in {-1, 1} else 1
        else:
            side = 1 if left >= right else -1
        self.detour_side = side
        assert self.yaw is not None
        self.detour_heading = wrap(self.yaw + self.detour_side * math.pi / 2.0)
        self.detour_start = Pose(self.x or 0.0, self.y or 0.0, self.yaw)
        self.detour_started_at = now
        self.detour_count += 1
        self.detour_blocked_since = 0.0
        self.state = "detour_turn"
        self.last_progress_at = now
        self._status("detour", reason)
        self.get_logger().warning(
            f"开始绕障#{self.detour_count}：side={'left' if self.detour_side > 0 else 'right'} "
            f"heading={self.detour_heading:.2f} left={left:.2f} right={right:.2f}"
        )
        return True

    def _detour_distance(self) -> float:
        '返回沿固定侧向绕障航向的有符号位移，避免斜向运动误判完成。'
        if self.detour_start is None or self.x is None or self.y is None:
            return 0.0
        dx = self.x - self.detour_start.x
        dy = self.y - self.detour_start.y
        return max(0.0, dx * math.cos(self.detour_heading) + dy * math.sin(self.detour_heading))

    def _tick(self) -> None:
        if self.done:
            self.cmd_pub.publish(Twist())
            return
        if self.rejected:
            self.get_logger().error("拒绝执行：必须使用 --execute --confirm deliver_nav_global")
            self.done = True
            return
        if time.monotonic() - self.started > self.timeout:
            self._stop("全局配送导航超时")
            return
        if self.x is None or self.y is None or self.yaw is None:
            self._publish(Twist(), "等待 odom，保持停车")
            return
        if self.ignore_obstacles:
            # 明确的无动态障碍单跑：不让静态场景边界/货架的 LaserScan
            # 把配送直线路径误判为动态障碍；随机障碍正式模式仍走原绕障逻辑。
            self.front = float("inf")
            self.left = float("inf")
            self.right = float("inf")
        elif time.monotonic() - self.last_scan > 1.0 or self.front is None:
            self._publish(Twist(), "等待 LaserScan，保持停车")
            return
        if not self.execute_enabled:
            self._publish(Twist(), "PLAN-ONLY 多航点路线已生成，保持停车")
            self.success = True
            self.done = True
            return

        if self.waypoint_index >= len(self.route):
            yaw_error = wrap(self.final_yaw - self.yaw)
            if abs(yaw_error) <= FINAL_YAW_TOLERANCE:
                self._stop("配送区最终朝向完成", True)
                return
            command = Twist()
            command.angular.z = max(-0.35, min(0.35, 1.3 * yaw_error))
            self._publish(command, f"最终朝向调整 error={yaw_error:.2f}")
            return

        now = time.monotonic()
        distance = self._distance_to_goal()
        if distance + 0.03 < self.last_distance:
            self.last_distance = distance
            self.last_progress_at = now

        if self.state == "detour_turn":
            if now - self.detour_started_at > DETOUR_MAX_SECONDS:
                self._stop(f"绕障转向超时>{DETOUR_MAX_SECONDS:.0f}s")
                return
            # 原地转向不会向障碍物前进，因此即使 front 较近也允许低速旋转。
            error = wrap(self.detour_heading - self.yaw)
            if abs(error) <= DETOUR_HEADING_TOLERANCE:
                self.state = "detour_drive"
                self.detour_start = Pose(self.x, self.y, self.yaw)
                self.last_progress_at = now
                self._status("detour_drive", "绕障方向对齐")
                return
            command = Twist()
            command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 1.4 * error))
            self._publish(command, f"绕障转向 error={error:.2f}")
            return

        if self.state == "detour_drive":
            if now - self.detour_started_at > DETOUR_MAX_SECONDS:
                self._stop(f"绕障前进超时>{DETOUR_MAX_SECONDS:.0f}s")
                return
            if self.front < DETOUR_FRONT_STOP:
                # 不能无限停车：携物状态下前方仍被挡住时，短暂停车确认，
                # 随后强制切换到另一侧重新绕行，避免原地卡死到超时。
                if self.detour_blocked_since <= 0.0:
                    self.detour_blocked_since = now
                blocked_for = now - self.detour_blocked_since
                if blocked_for >= DETOUR_BLOCKED_GRACE_SECONDS:
                    if self._begin_detour(
                        f"绕障侧前方持续堵塞 {blocked_for:.1f}s，切换相反侧",
                        force_opposite=True,
                    ):
                        return
                self._publish(
                    Twist(),
                    f"绕障期间前方过近 front={self.front:.3f}m，等待换边 {blocked_for:.1f}s",
                )
                return
            self.detour_blocked_since = 0.0
            if self._detour_distance() >= DETOUR_DISTANCE:
                self.state = "recover"
                self._status("recover", "绕障侧移完成，恢复目标方向")
                return
            command = Twist()
            command.linear.x = DETOUR_BACKUP_SPEED
            command.angular.z = max(-0.25, min(0.25, 1.0 * wrap(self.detour_heading - self.yaw)))
            self._publish(command, f"绕障前进 distance={self._detour_distance():.2f}/{DETOUR_DISTANCE:.2f}")
            return

        if self.state == "recover":
            goal = self._current_goal()
            assert goal is not None
            heading = math.atan2(goal[1] - self.y, goal[0] - self.x)
            error = wrap(heading - self.yaw)
            if abs(error) <= RECOVER_HEADING_TOLERANCE:
                self.state = "align"
                self.last_distance = self._distance_to_goal()
                self.last_progress_at = now
                self.detour_cooldown_until = now + DETOUR_COOLDOWN_SECONDS
                self._status("align", "绕障后目标方向恢复，进入冷却")
                return
            command = Twist()
            command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 1.4 * error))
            self._publish(command, f"绕障后恢复目标方向 error={error:.2f}")
            return

        # Normal waypoint tracking. The final delivery-base waypoint is a
        # safety region, not an exact point: once inside it, stop translating
        # and only correct the final yaw.
        waypoint_tolerance = (
            FINAL_WAYPOINT_TOLERANCE
            if self.waypoint_index == len(self.route) - 1
            else WAYPOINT_TOLERANCE
        )
        if distance <= waypoint_tolerance:
            self.waypoint_index += 1
            self.state = "align"
            self.detour_count = 0
            self.detour_cooldown_until = 0.0
            self.last_distance = float("inf")
            self.last_progress_at = time.monotonic()
            self._status(
                "waypoint_reached",
                f"航点{self.waypoint_index}到达 tolerance={waypoint_tolerance:.2f}",
            )
            self.get_logger().info(
                f"到达航点 {self.waypoint_index}/{len(self.route)} "
                f"tolerance={waypoint_tolerance:.2f}"
            )
            return
        if not self.ignore_obstacles and self.front < FRONT_BLOCKED:
            if now < self.detour_cooldown_until:
                self._publish(Twist(), f"绕障后仍有障碍，冷却中 front={self.front:.3f}m")
                return
            self._begin_detour(f"航点前方堵塞 front={self.front:.3f}m")
            return

        if self.state == "drive" and now - self.last_progress_at > NO_PROGRESS_SECONDS:
            if self._begin_detour("航点距离长时间无进展"):
                return
            return

        goal = self._current_goal()
        assert goal is not None
        heading = math.atan2(goal[1] - self.y, goal[0] - self.x)
        error = wrap(heading - self.yaw)
        command = Twist()
        if self.state == "align" and abs(error) > 0.20:
            command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 1.4 * error))
        else:
            self.state = "drive"
            # 目标方向误差较大时只旋转，避免底盘边转边横切到货架或障碍物。
            alignment = max(0.0, math.cos(error))
            command.linear.x = min(MAX_SPEED, max(0.0, 0.35 * distance * alignment))
            command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, 1.4 * error))
        self._publish(command, f"航点{self.waypoint_index + 1}/{len(self.route)} dist={distance:.2f} error={error:.2f} front={self.front:.2f}")

    def _log(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(text)
            self.last_log = now


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 多航点配送导航")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--route", default=json.dumps(DEFAULT_ROUTE))
    parser.add_argument("--final-yaw", type=float, default=FINAL_YAW, help="路线完成后的最终朝向（弧度）")
    parser.add_argument(
        "--ignore-obstacles",
        action="store_true",
        help="无动态障碍单跑：忽略 LaserScan 绕障判定；随机障碍正式模式不要使用",
    )
    args = parser.parse_args()
    if args.execute and args.confirm != "deliver_nav_global":
        print("拒绝执行：必须使用 --confirm deliver_nav_global")
        return 2
    try:
        route = json.loads(args.route)
    except Exception:
        print("路线JSON非法")
        return 2
    rclpy.init()
    node = GlobalDeliveryNavigator(
        args.execute,
        args.confirm,
        args.timeout,
        route,
        args.final_yaw,
        ignore_obstacles=args.ignore_obstacles,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        result = node.success
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())

