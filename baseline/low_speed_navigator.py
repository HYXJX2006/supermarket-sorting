#!/usr/bin/env python3
"""DG-202606 低速自主导航动作层。

当前阶段目标：从起点导航到货架观察区。
仅使用 odom + LaserScan，不读取 Server 内部布局，不控制机械臂。
任何异常、超时、通信中断或节点退出都会发布零速度。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
SCAN_TOPIC = "/slamware_ros_sdk_server_node/scan"
CMD_TOPIC = "/cmd_vel"
STATUS_TOPIC = "/competition/navigation_status"

# 与 competition_executor 的导航目标发布端保持匹配，接收晚启动时的缓存目标。
NAV_GOAL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def wrap_to_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def finite_min(values: list[float]) -> float | None:
    valid = [float(v) for v in values if math.isfinite(v) and v > 0.0]
    return min(valid) if valid else None


class LowSpeedNavigator(Node):
    def __init__(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
        *,
        max_speed: float,
        max_angular: float,
        obstacle_distance: float,
        goal_tolerance: float,
        yaw_tolerance: float,
        timeout: float,
        goal_topic: str | None,
        route: list[tuple[float, float, float]] | None = None,
        pickup_approach: bool = False,
        pickup_exact: bool = False,
        pickup_transit: bool = False,
        ignore_obstacles: bool = False,
        pickup_safe_distance: float = 0.30,
        pickup_lateral_tolerance: float = 0.08,
        drive_speed_gain: float = 0.35,
        avoid_clearance: float = 0.55,
        planned_route: bool = False,
        backup_speed: float = 0.15,
        backup_seconds: float = 2.0,
    ) -> None:
        super().__init__("low_speed_navigator")
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.goal_yaw = float(goal_yaw)
        self.route = list(route or [(self.goal_x, self.goal_y, self.goal_yaw)])
        self.route_index = 0
        if len(self.route) > 1 and goal_topic is None:
            # 当前控制目标必须从第一航点开始；否则会沿第一段方向行驶，
            # 却以最终航点计算距离，永远不会触发航点切换。
            self.goal_x, self.goal_y, self.goal_yaw = self.route[0]
        # 取货前航线是官方定义的货架外侧无障碍走廊，必须严格按航点
        # 行驶；不能让局部雷达把机器人拽入障碍区或货架侧边。
        self.strict_route = len(self.route) > 1 and goal_topic is None
        self.segment_start: tuple[float, float] | None = None
        self.goal_received = goal_topic is None
        self.goal_topic = goal_topic
        # In pickup-approach mode each dynamic goal is one axis-aligned
        # segment. Lock its initial travel heading so pose drift cannot turn a
        # pure east/west/north segment into diagonal chasing.
        self.pickup_heading_target: float | None = None
        # 目标货架接近阶段：货架会出现在雷达正前方，不能把货架本体
        # 当作需要绕开的动态障碍。该开关只由 approach_target worker 使用；
        # 取货前安全路线和配送路线仍保留 LaserScan 避障。
        self.pickup_approach = bool(pickup_approach)
        self.pickup_exact = bool(pickup_exact)
        self.pickup_transit = bool(pickup_transit)
        # 无障碍开发单跑：关闭雷达干预，但保留普通目标距离/航向控制。
        self.ignore_obstacles = bool(ignore_obstacles)
        # 货架/商品碰撞体会让底盘在安全边界外停止；接近模式允许
        # 在安全距离内结束导航，避免继续等待不可达的几何目标。
        self.pickup_safe_distance = max(0.05, float(pickup_safe_distance))
        self.pickup_lateral_tolerance = max(0.02, float(pickup_lateral_tolerance))
        self.planned_route = bool(planned_route)
        if self.pickup_approach and self.pickup_transit:
            raise ValueError("pickup_approach 与 pickup_transit 不能同时启用")
        self.max_speed = max(0.0, float(max_speed))
        self.max_angular = max(0.0, float(max_angular))
        self.obstacle_distance = max(0.1, float(obstacle_distance))
        self.goal_tolerance = max(0.01, float(goal_tolerance))
        self.yaw_tolerance = max(0.02, float(yaw_tolerance))
        self.timeout = max(1.0, float(timeout))
        # 通用直行推进比例增益与走廊避障参数：巡航段可调大 drive_speed_gain
        # 让长直 clear 段更快达速，转弯/末段仍由距离梯形自动减速。
        self.drive_speed_gain = max(0.05, float(drive_speed_gain))
        self.avoid_clearance = max(0.30, float(avoid_clearance))
        self.backup_speed = min(0.5, max(0.05, float(backup_speed)))
        self.backup_seconds = max(0.5, float(backup_seconds))

        self.x: float | None = None
        self.y: float | None = None
        self.yaw: float | None = None
        self.scan_received_at = 0.0
        self.last_scan_at = 0.0
        self.front_min: float | None = None
        self.left_min: float | None = None
        self.right_min: float | None = None
        self.finished = False
        self.success = False
        self.started_at = time.monotonic()
        self.last_log_at = 0.0
        self.last_command = Twist()
        self.last_goal_key = ""
        # 避障脱困状态：原地转向累计时长、倒车脱困截止时刻与转向方向。
        self.avoid_since: float | None = None
        self.backup_until: float | None = None
        self.backup_turn_sign = 1.0
        # 通用停滞脱困：任意方向长时间无位移时倒车重新定位。
        self._pos_history: list[tuple[float, float, float]] = []
        self._stuck_since: float | None = None
        self._stuck_backup_until: float | None = None
        # 绕障：保存最近一帧雷达原始数据与角度，用于选安全推进方向。
        self._ranges: list[float] = []
        self._scan_angle_min = 0.0
        self._scan_angle_inc = 0.0
        # pickup-approach 朝向滞回：避免在临界误差附近反复切换
        # “原地旋转/前进”，导致返程接近货架时停滞。
        self._pickup_heading_ready = False
        # 轴向接近越过目标后锁存反向方向，直到当前动态目标结束。
        # 不能仅按每一帧的剩余距离切换，否则会在目标容差边界来回掉头。
        self._pickup_reverse_latched = False

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        if goal_topic:
            self.create_subscription(PoseStamped, goal_topic, self._on_goal, NAV_GOAL_QOS)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._control_tick)

        self.get_logger().warning(
            f"低速导航启动：goal=({self.goal_x:.2f},{self.goal_y:.2f},{self.goal_yaw:.2f})，"
            f"max_speed={self.max_speed:.2f}，obstacle_distance={self.obstacle_distance:.2f}；"
            f"goal_topic={self.goal_topic or 'cli'}；"
            f"pickup_approach={'on' if self.pickup_approach else 'off'}；"
            f"pickup_transit={'on' if self.pickup_transit else 'off'}；"
            f"planned_route={'on' if self.planned_route else 'off'}；"
            "只控制底盘，不控制机械臂"
        )

    def _publish_status(self, state: str, reason: str = "") -> None:
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "state": state,
                "reason": reason,
                "goal": [round(self.goal_x, 4), round(self.goal_y, 4), round(self.goal_yaw, 4)],
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        if rclpy.ok():
            try:
                self.status_pub.publish(message)
            except RCLError:
                pass

    def _on_goal(self, message: PoseStamped) -> None:
        pose = message.pose
        self.goal_x = float(pose.position.x)
        self.goal_y = float(pose.position.y)
        self.goal_yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z),
            1.0 - 2.0 * (pose.orientation.z * pose.orientation.z),
        )
        goal_key = f"{self.goal_x:.4f},{self.goal_y:.4f},{self.goal_yaw:.4f}"
        # 编排器会周期性重发同一个目标，重复消息不能重置
        # pickup_heading_target / reverse latch；否则越点回收会被重新
        # 当成正向接近，在目标线附近反复掉头。
        new_goal = goal_key != self.last_goal_key
        if new_goal:
            self.route = [(self.goal_x, self.goal_y, self.goal_yaw)]
            self.route_index = 0
            self.pickup_heading_target = None
            self._pickup_heading_ready = False
            self._pickup_reverse_latched = False
            self.last_goal_key = goal_key
            self._publish_status("active", "收到新导航目标")
            self.get_logger().info(
                f"收到导航目标：({self.goal_x:.3f},{self.goal_y:.3f},yaw={self.goal_yaw:.3f})"
            )
        self.goal_received = True

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.yaw = yaw_from_quaternion(pose.orientation)
        if self.strict_route and self.segment_start is None:
            self.segment_start = (self.x, self.y)

    def _on_scan(self, message: LaserScan) -> None:
        ranges = list(message.ranges)
        if not ranges:
            return
        angle_min = float(message.angle_min)
        increment = float(message.angle_increment)
        if increment == 0.0:
            return

        def sector_min(center: float, half_width: float) -> float | None:
            values = []
            for index, distance in enumerate(ranges):
                angle = wrap_to_pi(angle_min + index * increment)
                error = abs(wrap_to_pi(angle - center))
                if error <= half_width:
                    values.append(float(distance))
            return finite_min(values)

        self.front_min = sector_min(0.0, math.radians(22.0))
        self.left_min = sector_min(math.pi / 2.0, math.radians(45.0))
        self.right_min = sector_min(-math.pi / 2.0, math.radians(45.0))
        self._ranges = ranges
        self._scan_angle_min = angle_min
        self._scan_angle_inc = increment
        self.scan_received_at = time.monotonic()
        self.last_scan_at = self.scan_received_at

    def _sector_min_dist(self, relative_angle: float, half_width: float) -> float | None:
        if not self._ranges or self.yaw is None:
            return None
        values = []
        for idx, dist in enumerate(self._ranges):
            ang_abs = self._scan_angle_min + idx * self._scan_angle_inc
            rel = wrap_to_pi(ang_abs - self.yaw)
            if abs(wrap_to_pi(rel - relative_angle)) <= half_width:
                if math.isfinite(dist) and dist > 0.02:
                    values.append(float(dist))
        return min(values) if values else None

    def _safe_heading(self, heading_error: float) -> float:
        """返回相对机器人的安全推进方向：近距离无障碍且尽量贴近目标方向。

        目标方向被障碍堵住时，沿最接近目标的可通行方向绕行，避免原地打转。
        绕行方向优先选择"前方障碍的反侧"，避免选到障碍更密集的一侧。
        """
        if not self._ranges or self.yaw is None:
            return heading_error
        # 前方近距离障碍的加权中心方向（相对机器人）。
        obs_center = 0.0
        obs_w = 0.0
        for idx, dist in enumerate(self._ranges):
            ang = wrap_to_pi(self._scan_angle_min + idx * self._scan_angle_inc - self.yaw)
            if math.isfinite(dist) and 0.02 < dist < 1.2 and abs(ang) <= math.radians(100):
                w = 1.0 / max(dist, 0.25)
                obs_center += ang * w
                obs_w += w
        if obs_w > 0:
            obs_center /= obs_w
        best = None
        best_score = None
        for deg in range(-150, 151, 10):
            a = math.radians(deg)
            close_dist = self._sector_min_dist(a, math.radians(12))
            if close_dist is None or close_dist < self.avoid_clearance:
                continue
            angle_cost = abs(wrap_to_pi(a - heading_error))
            away = 0.0
            if obs_w > 0 and abs(wrap_to_pi(a - obs_center)) > math.radians(90):
                away = 0.6
            score = -angle_cost + 0.3 * min(close_dist, 2.5) + away
            if best_score is None or score > best_score:
                best_score = score
                best = a
        return best if best is not None else heading_error

    def _publish(self, command: Twist) -> None:
        self.last_command = command
        if not rclpy.ok():
            return
        try:
            self.cmd_pub.publish(command)
        except RCLError:
            pass

    def stop(self, reason: str) -> None:
        self._publish(Twist())
        if not self.finished:
            state = "reached" if self.success else "failed"
            self._publish_status(state, reason)
            self.get_logger().info(f"导航停止：{reason}")
        self.finished = True

    def _control_tick(self) -> None:
        if self.finished:
            self._publish(Twist())
            return

        now = time.monotonic()
        if now - self.started_at > self.timeout:
            self.stop("超时，自动停车")
            return
        if not self.goal_received:
            self._publish(Twist())
            self._publish_status("waiting", "等待导航目标")
            self._periodic_log("等待 /competition/navigation_goal，保持停车")
            return
        if self.x is None or self.y is None or self.yaw is None:
            self._publish(Twist())
            self._periodic_log("等待 odom，保持停车")
            return
        if now - self.last_scan_at > 1.0 and not (
            self.pickup_approach or self.pickup_transit or self.ignore_obstacles
        ):
            self._publish(Twist())
            self._periodic_log("LaserScan 超时，保持停车")
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)
        arrival_tolerance = self.goal_tolerance
        transit_axis_done = False
        if self.pickup_transit:
            # Formal transit goals are axis-aligned. Once the robot crosses
            # the commanded axis coordinate, treat the waypoint as reached;
            # Euclidean distance alone would keep driving past it because the
            # heading is intentionally locked to the segment direction.
            arrival_tolerance = max(arrival_tolerance, 0.16)
            segment_dx = math.cos(self.goal_yaw)
            segment_dy = math.sin(self.goal_yaw)
            remaining_along_segment = dx * segment_dx + dy * segment_dy
            transit_axis_done = remaining_along_segment <= arrival_tolerance
        # pickup-approach 的目标点可能落在货架碰撞体内部。此时底盘
        # 的物理可达终点是货架外安全边界，而不是目标坐标本身。
        # 仅对无多段 route 的目标接近启用，避免跳过中间航点。
        pickup_arrival_tolerance = self.pickup_safe_distance + 0.05
        # pickup-approach 的宽松距离只能用于最终法向接近；横向对齐和
        # 机械臂预抓取安全线使用 pickup-exact，必须满足普通位置容差。
        segment_dx = math.cos(self.goal_yaw)
        segment_dy = math.sin(self.goal_yaw)
        lateral_error = abs(dx * (-segment_dy) + dy * segment_dx)
        normal_approach_axis = abs(abs(wrap_to_pi(self.goal_yaw)) - math.pi / 2.0) <= 0.20
        # 横向换列段（yaw=0/π）必须精确到目标 x；只有货架法向段
        # （yaw=±π/2）才允许在货架外安全距离提前结束。
        exact_arrival = self.pickup_exact or not normal_approach_axis
        safe_arrival = (
            self.pickup_approach
            and len(self.route) == 1
            and distance <= (self.goal_tolerance if exact_arrival else pickup_arrival_tolerance)
            and lateral_error <= self.pickup_lateral_tolerance
        )
        if safe_arrival:
            arrival_tolerance = pickup_arrival_tolerance
        if distance <= arrival_tolerance or transit_axis_done:
            yaw_error = wrap_to_pi(self.goal_yaw - self.yaw)
            if abs(yaw_error) <= self.yaw_tolerance:
                if self.route_index + 1 < len(self.route):
                    self.segment_start = (self.goal_x, self.goal_y)
                    self.route_index += 1
                    self.goal_x, self.goal_y, self.goal_yaw = self.route[self.route_index]
                    self._publish(Twist())
                    self._publish_status("active", f"已到达航点 {self.route_index}，进入下一段")
                    self.get_logger().info(
                        f"已到达航点 {self.route_index}/{len(self.route)}，"
                        f"切换到 ({self.goal_x:.3f},{self.goal_y:.3f})"
                    )
                    return
                self.success = True
                self.stop("已到达货架观察点并完成朝向对齐")
                return
            command = Twist()
            command.angular.z = max(
                -self.max_angular,
                min(self.max_angular, 1.6 * yaw_error),
            )
            self._publish(command)
            self._periodic_log(
                f"到达位置，正在对齐朝向 yaw={self.yaw:.2f} err={yaw_error:.2f}"
                + ("（已进入货架安全接近距离）" if safe_arrival else "")
            )
            return

        goal_heading = math.atan2(dy, dx)
        if self.pickup_approach or self.pickup_transit:
            if self.pickup_heading_target is None:
                # Executor supplies an axis-aligned goal_yaw for every formal
                # shelf-entry segment. Use that commanded axis directly;
                # deriving heading from a drifting pose creates diagonal
                # chasing and oscillation near the corridor boundary.
                self.pickup_heading_target = wrap_to_pi(self.goal_yaw)
            drive_heading = self.pickup_heading_target
        else:
            drive_heading = goal_heading
        heading_error = wrap_to_pi(drive_heading - self.yaw)
        command = Twist()

        front_blocked = (
            not self.pickup_approach
            and not self.pickup_transit
            and not self.ignore_obstacles
            and self.front_min is not None
            and self.front_min < self.obstacle_distance
        )
        if self.strict_route:
            # 官方取货前航线位于随机障碍区外侧。严格保持当前线段方向，
            # 同时用横向误差把底盘拉回航线，避免“朝终点”造成斜穿/漂移。
            if self.route_index == 0:
                segment_start = self.segment_start or (self.x, self.y)
            else:
                previous = self.route[self.route_index - 1]
                segment_start = (previous[0], previous[1])
            segment_end = self.route[self.route_index]
            segment_heading = math.atan2(
                segment_end[1] - segment_start[1],
                segment_end[0] - segment_start[0],
            )
            cross_track = (
                -math.sin(segment_heading) * (self.x - segment_start[0])
                + math.cos(segment_heading) * (self.y - segment_start[1])
            )
            heading_error = wrap_to_pi(segment_heading - self.yaw)
            if abs(heading_error) > 0.30:
                # 航点切换时先原地完成 90 度转向，禁止带着上一段的前进
                # 惯性冲出走廊；对齐后才恢复线段推进。
                command.linear.x = 0.0
                command.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, 2.0 * heading_error),
                )
                self._publish(command)
                self._periodic_log(
                    f"strict-route turn-in-place seg={self.route_index + 1}/{len(self.route)} "
                    f"yaw_err={heading_error:.2f} pos=({self.x:.2f},{self.y:.2f})"
                )
                return
            cross_correction = math.atan2(cross_track, 0.60)
            steer = wrap_to_pi(heading_error - 1.20 * cross_correction)
            alignment = max(0.0, math.cos(heading_error))
            command.linear.x = min(self.max_speed, 0.45 * distance * alignment)
            command.angular.z = max(
                -self.max_angular,
                min(self.max_angular, 1.8 * steer),
            )
            self._publish(command)
            self._periodic_log(
                f"strict-route pos=({self.x:.2f},{self.y:.2f}) dist={distance:.2f} "
                f"seg={self.route_index + 1}/{len(self.route)} cross={cross_track:.2f} "
                f"heading_err={heading_error:.2f} steer={steer:.2f}"
            )
            return

        if self.pickup_approach:
            # 这里是货架外侧到目标观察位的最后接近段：不读取 front/left/right
            # 做主动绕行，也不启用停滞倒车；只按 odom 直达目标并在容差内对齐 yaw。
            # 返程接近货架时必须先把底盘转正，避免一边横向靠近一边
            # 大角度转向，斜穿货架/中间挡板或跑到最右侧墙边。
            self.avoid_since = None
            self.backup_until = None
            self._stuck_since = None
            self._stuck_backup_until = None
            # 朝向控制使用滞回：首次接近时将误差收敛到 0.18 rad 以内；
            # 已进入推进状态后，只有误差重新超过 0.35 rad 才重新原地对齐。
            # 这样可以避免 0.30 rad 附近因 odom 噪声反复切换，造成原地抖动。
            # 轴向 pickup 航点可能因仿真步长/底盘惯性越过目标；
            # 越过后必须反向回收，否则锁定原方向会一直驶离目标。
            remaining_along = dx * segment_dx + dy * segment_dy
            if not self._pickup_reverse_latched and remaining_along < -self.goal_tolerance:
                # 已经越过目标：本目标内锁存反向回收，直到距离门禁触发。
                # 不在 remaining_along 穿越 +/- tolerance 时再次切回正向。
                self._pickup_reverse_latched = True
                self._periodic_log(
                    f"pickup-approach overshoot reverse latched pos=({self.x:.2f},{self.y:.2f}) "
                    f"remaining={remaining_along:.2f}"
                )
            desired_heading = wrap_to_pi(
                self.goal_yaw + math.pi if self._pickup_reverse_latched else self.goal_yaw
            )
            heading_changed = (
                self.pickup_heading_target is None
                or abs(wrap_to_pi(desired_heading - self.pickup_heading_target)) > 0.02
            )
            if heading_changed:
                self.pickup_heading_target = desired_heading
                self._pickup_heading_ready = False
            drive_heading = self.pickup_heading_target
            heading_error = wrap_to_pi(drive_heading - self.yaw)
            if not self._pickup_heading_ready:
                if abs(heading_error) > 0.18:
                    command.linear.x = 0.0
                    command.angular.z = max(
                        -self.max_angular,
                        min(self.max_angular, 1.8 * heading_error),
                    )
                    self._publish(command)
                    self._periodic_log(
                        f"pickup-approach turn-in-place pos=({self.x:.2f},{self.y:.2f}) "
                        f"dist={distance:.2f} heading_err={heading_error:.2f} "
                        "（先对齐朝向，忽略货架雷达障碍）"
                    )
                    return
                self._pickup_heading_ready = True
                self._periodic_log(
                    f"pickup-approach heading aligned err={heading_error:.2f}，恢复前进"
                )
            elif abs(heading_error) > 0.35:
                self._pickup_heading_ready = False
                command.linear.x = 0.0
                command.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, 1.8 * heading_error),
                )
                self._publish(command)
                self._periodic_log(
                    f"pickup-approach re-align turn-in-place pos=({self.x:.2f},{self.y:.2f}) "
                    f"dist={distance:.2f} heading_err={heading_error:.2f} "
                    "（误差超滞回上限，先对齐朝向）"
                )
                return
            alignment = max(0.0, math.cos(heading_error))
            command.linear.x = min(self.max_speed, 0.45 * distance * alignment)
            # Once a pickup segment is aligned, damp yaw correction heavily.
            # The previous 1.8 gain plus the hysteresis threshold caused the
            # differential base to overshoot, re-enter turn-in-place, and stall
            # on long east/north/west bypass segments.
            # 原夹 0.08 rad/s：一次对齐要 40s+（主人指出立省 2-4 分钟/轮）。
            # 0.4：与 max_angular 默认同量级，翻转/对齐 8s 内完成。
            _align_rate = float(os.getenv("SUPERMARKET_ALIGN_RATE", "0.4"))
            command.angular.z = max(
                -min(self.max_angular, _align_rate),
                min(min(self.max_angular, _align_rate), 0.5 * heading_error),
            )
            self._publish(command)
            self._periodic_log(
                f"pickup-approach pos=({self.x:.2f},{self.y:.2f}) dist={distance:.2f} "
                f"heading_err={heading_error:.2f} drive_heading={drive_heading:.2f}"
                "（锁定航点方向，忽略货架雷达障碍）"
            )
            return

        if self.pickup_transit:
            # Formal shelf-entry segments are axis-aligned. Do not translate
            # while turning: mixing angular correction with forward velocity
            # made the base drift into the south boundary and stall.
            self.avoid_since = None
            self.backup_until = None
            self._stuck_since = None
            self._stuck_backup_until = None
            if not self._pickup_heading_ready:
                if abs(heading_error) > 0.25:
                    command.linear.x = 0.0
                    command.angular.z = max(
                        -self.max_angular,
                        min(self.max_angular, 1.5 * heading_error),
                    )
                    self._publish(command)
                    self._periodic_log(
                        f"pickup-transit turn-in-place pos=({self.x:.2f},{self.y:.2f}) "
                        f"dist={distance:.2f} heading_err={heading_error:.2f}"
                    )
                    return
                self._pickup_heading_ready = True
                self._periodic_log(
                    f"pickup-transit heading aligned err={heading_error:.2f}，开始轴向直行"
                )
            elif abs(heading_error) > 0.80:
                self._pickup_heading_ready = False
                command.linear.x = 0.0
                command.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, 1.5 * heading_error),
                )
                self._publish(command)
                self._periodic_log(
                    f"pickup-transit re-align turn-in-place pos=({self.x:.2f},{self.y:.2f}) "
                    f"dist={distance:.2f} heading_err={heading_error:.2f}"
                )
                return
            command.linear.x = min(self.max_speed, self.drive_speed_gain * distance)
            # 轴向直行的航向微调同样提速（0.08→0.4，主人指示）
            command.angular.z = max(-0.4, min(0.4, 0.5 * heading_error))
            self._publish(command)
            self._periodic_log(
                f"pickup-transit pos=({self.x:.2f},{self.y:.2f}) dist={distance:.2f} "
                f"heading_err={heading_error:.2f}（轴向直行，忽略配送避障）"
            )
            return

        if self.ignore_obstacles:
            # 无障碍单跑只跳过 LaserScan 安全干预；这里仍按普通目标
            # 的距离和航向控制，不能复用 pickup-approach 的安全距离语义。
            self.avoid_since = None
            self.backup_until = None
            self._stuck_since = None
            self._stuck_backup_until = None
            alignment = max(0.0, math.cos(heading_error))
            command.linear.x = min(self.max_speed, self.drive_speed_gain * distance * alignment)
            command.angular.z = max(
                -self.max_angular,
                min(self.max_angular, 1.4 * heading_error),
            )
            self._publish(command)
            self._periodic_log(
                f"ignore-obstacles pos=({self.x:.2f},{self.y:.2f}) dist={distance:.2f} "
                f"heading_err={heading_error:.2f}"
            )
            return

        if front_blocked:
            # 前方有障碍：向较宽一侧转向；原地转向持续超 3s 仍未脱困则
            # 倒车 0.3m 拉开距离再重新转向，避免被障碍包夹原地打转。
            now = time.monotonic()
            if self.avoid_since is None:
                self.avoid_since = now
            left = self.left_min if self.left_min is not None else 0.0
            right = self.right_min if self.right_min is not None else 0.0
            turn_sign = 1.0 if left >= right else -1.0
            backup_mode = False
            if now - self.avoid_since > 3.0:
                if self.backup_until is None:
                    # backup_speed m/s * backup_seconds s = 后退距离（默认≈0.3m）
                    self.backup_until = now + self.backup_seconds
                    self.backup_turn_sign = turn_sign
                    self.get_logger().warning(
                        f"原地避障 {now - self.avoid_since:.1f}s 未脱困，倒车 0.3m 后重新转向"
                    )
                backup_mode = now < self.backup_until
            else:
                self.backup_until = None
            command = Twist()
            if backup_mode:
                command.linear.x = -self.backup_speed
                command.angular.z = self.backup_turn_sign * 0.25
            else:
                # 向较宽侧转向并小步前探；障碍过近时仅原地转，避免逼近。
                command.angular.z = turn_sign * self.max_angular
                command.linear.x = 0.15 if self.front_min > 0.30 else 0.0
            self._publish(command)
            self._periodic_log(
                f"前方障碍 front={self.front_min:.2f}m "
                f"{'倒车脱困' if backup_mode else '避障转向'}"
                f"{'左' if turn_sign > 0 else '右'}"
            )
            return

        # 前方无障碍，清除避障脱困状态。
        self.avoid_since = None
        self.backup_until = None

        # 通用停滞脱困：预规划配送路线已经由膨胀障碍 A* 保证可行，
        # 不再用倒车把底盘拉离路线；非预规划导航仍保留原逻辑。
        now = time.monotonic()
        if self.planned_route:
            self._stuck_since = None
            self._stuck_backup_until = None
        self._pos_history.append((now, self.x, self.y))
        cutoff = now - 2.5
        while self._pos_history and self._pos_history[0][0] < cutoff:
            self._pos_history.pop(0)
        if len(self._pos_history) >= 2:
            t0, x0, y0 = self._pos_history[0]
            tn, xn, yn = self._pos_history[-1]
            window_moved = math.hypot(xn - x0, yn - y0)
            if window_moved < 0.05 and tn - t0 >= 2.0:
                if self._stuck_since is None:
                    self._stuck_since = now
            else:
                self._stuck_since = None
        if not self.planned_route and self._stuck_since is not None and now - self._stuck_since > 2.0:
            if self._stuck_backup_until is None:
                self._stuck_backup_until = now + self.backup_seconds
                self.get_logger().warning(
                    f"前进停滞 {now - self._stuck_since:.1f}s，倒车 {self.backup_speed * self.backup_seconds:.2f}m 重新定位"
                )
            if now < self._stuck_backup_until:
                command = Twist()
                command.linear.x = -self.backup_speed
                command.angular.z = 0.2
                self._publish(command)
                return
            self._stuck_backup_until = None
            self._stuck_since = None

        # 绕障推进：目标方向被近距离障碍堵住时，转向最空旷且贴近目标的
        # 方向前进；front_blocked 分支已处理正面急障，这里覆盖侧向拥堵。
        target_clear = self._sector_min_dist(heading_error, math.radians(18))
        # A* 预规划段已经通过机器人膨胀障碍验证，严格沿当前航点
        # 方向推进；只有 front_blocked 分支才进入紧急绕障。
        steer = heading_error
        if not self.planned_route and (target_clear is None or target_clear < self.avoid_clearance + 0.1):
            steer = self._safe_heading(heading_error)
        alignment = max(0.0, math.cos(steer))
        command.linear.x = min(self.max_speed, self.drive_speed_gain * distance * alignment)
        command.angular.z = max(
            -self.max_angular,
            min(self.max_angular, 1.4 * steer),
        )
        self._publish(command)
        self._periodic_log(
            f"drive pos=({self.x:.2f},{self.y:.2f}) dist={distance:.2f} "
            f"heading_err={heading_error:.2f} steer={steer:.2f} front={self.front_min}"
        )

    def _periodic_log(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_log_at >= 1.0:
            self.get_logger().info(message)
            self.last_log_at = now


def parse_route(raw: str) -> list[tuple[float, float, float]]:
    """解析 x,y[,yaw];x,y[,yaw] 航点串；缺省 yaw 使用该段行驶方向。"""
    points: list[tuple[float, float, float]] = []
    for index, chunk in enumerate(raw.replace("|", ";").split(";"), start=1):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = [float(item.strip()) for item in chunk.split(",")]
        if len(values) not in (2, 3):
            raise ValueError(f"route waypoint {index} must be x,y or x,y,yaw")
        if len(values) == 2:
            if points:
                yaw = math.atan2(values[1] - points[-1][1], values[0] - points[-1][0])
            else:
                yaw = math.pi / 2.0
            values.append(yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"route waypoint {index} contains non-finite value")
        points.append((values[0], values[1], values[2]))
    if not points:
        raise ValueError("route must contain at least one waypoint")
    return points


def main() -> int:
    parser = argparse.ArgumentParser(description="DG-202606 低速自主导航")
    parser.add_argument("--goal-x", type=float, default=0.852)
    parser.add_argument("--goal-y", type=float, default=2.475)
    parser.add_argument("--goal-yaw", type=float, default=math.pi / 2.0)
    parser.add_argument("--max-speed", type=float, default=0.08)
    parser.add_argument("--max-angular", type=float, default=0.45)
    parser.add_argument("--obstacle-distance", type=float, default=0.40)
    parser.add_argument("--speed-gain", type=float, default=0.35,
                        help="通用直行推进比例增益：长直 clear 段调大可更快达速")
    parser.add_argument("--avoid-clearance", type=float, default=0.55,
                        help="避障方向选择要求的最短可通行距离(m)")
    parser.add_argument("--backup-speed", type=float, default=0.15)
    parser.add_argument("--backup-seconds", type=float, default=2.0)
    parser.add_argument("--planned-route", action="store_true",
                        help="预规划配送航点：沿航点直达，仅保留正面紧急避障")
    parser.add_argument("--goal-tolerance", type=float, default=0.10)
    parser.add_argument("--yaw-tolerance", type=float, default=0.10)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--goal-topic", default="", help="订阅 PoseStamped 导航目标；为空时使用命令行目标")
    parser.add_argument(
        "--pickup-approach",
        action="store_true",
        help="目标货架最后接近段：忽略 LaserScan 主动绕障、雷达超时停车和停滞倒车；仅用于已知安全取货接近路线",
    )
    parser.add_argument(
        "--pickup-safe-distance", type=float, default=0.30,
        help="pickup-approach 到达货架外安全边界的距离阈值(m)",
    )
    parser.add_argument(
        "--pickup-lateral-tolerance", type=float, default=0.08,
        help="pickup-approach 允许的横向误差阈值(m)",
    )
    parser.add_argument(
        "--pickup-exact", action="store_true",
        help="pickup-approach 严格按 goal-tolerance 到位，不使用货架安全距离放宽",
    )
    parser.add_argument(
        "--pickup-transit",
        action="store_true",
        help="首段起点到 E 货架过渡点：固定直行，不启用配送阶段避障、雷达超时停车或停滞倒车",
    )
    parser.add_argument(
        "--ignore-obstacles",
        action="store_true",
        help="无障碍开发单跑：关闭雷达避障/超时停车/脱困，但保留普通目标跟踪",
    )
    parser.add_argument(
        "--route",
        default="",
        help="分段航点串 x,y[,yaw];x,y[,yaw]；与 --goal-topic 同时使用时由动态目标覆盖",
    )
    args = parser.parse_args()

    try:
        route = parse_route(args.route) if args.route else None
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init()
    node = LowSpeedNavigator(
        args.goal_x,
        args.goal_y,
        args.goal_yaw,
        max_speed=args.max_speed,
        max_angular=args.max_angular,
        obstacle_distance=args.obstacle_distance,
        goal_tolerance=args.goal_tolerance,
        yaw_tolerance=args.yaw_tolerance,
        timeout=args.timeout,
        goal_topic=args.goal_topic or None,
        route=route,
        pickup_approach=args.pickup_approach,
        pickup_exact=args.pickup_exact,
        pickup_transit=args.pickup_transit,
        ignore_obstacles=args.ignore_obstacles,
        pickup_safe_distance=args.pickup_safe_distance,
        pickup_lateral_tolerance=args.pickup_lateral_tolerance,
        drive_speed_gain=args.speed_gain,
        avoid_clearance=args.avoid_clearance,
        backup_speed=args.backup_speed,
        backup_seconds=args.backup_seconds,
        planned_route=args.planned_route,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            node.get_logger().warning("收到中断，自动停车")
    finally:
        if rclpy.ok():
            node._publish(Twist())
        success = node.success
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
