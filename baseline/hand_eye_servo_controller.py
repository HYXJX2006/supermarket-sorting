#!/usr/bin/env python3
"""手眼伺服：creep 到位后、闭爪前，用右手眼 RGB 测量目标横向/高度偏差。

流程（对应"到达高度→左右对准→判断伸入深度→再抓"的分阶段方案）：
  1. creep 已把底盘带到停止线，手臂张开悬在罐前；
  2. 本 worker 抓一帧 /right_camera/color/image_raw，HSV 分割彩色目标
     （货架板白/灰、夹爪黑，饱和度团块即商品），取最靠近图像中心的团块；
  3. 质心相对图像中心的像素偏差 × 估计距离 → 米制 (dx, dz)；
     距离用已知商品宽度的表观尺寸反推：Z = fx * width / blob_width；
  4. 修正量写入结果文件 debug_data/servo_result.json，executor 读它决定
     是否回退重部署（deploy IK 把手臂对到修正后的世界坐标）。

设计约定：
  - 位置闭环的最后 5cm 必须在手腕坐标系里测量（2026-09-12 全天实测：
    开环链路检测残差 ±3cm，抓取容差 ±1.5cm，纯开环必然偶尔夹空）。
  - 图像中心 = 当前抓取瞄准方向（相机装在手腕上，随臂动），因此
    质心偏差即"手指-商品"相对偏差，无需知道商品绝对位置。
  - 测量失败（无团块/帧过旧）不算失败：输出 measured=false，executor
    继续盲闭——不比现状差。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, String

TASK_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
if not (TASK_DIR / "mmk2_kdl.py").is_file():
    bundled = Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"
    if (bundled / "mmk2_kdl.py").is_file():
        TASK_DIR = bundled
sys.path.insert(0, str(TASK_DIR))
from mmk2_kdl import MMK2Kdl  # noqa: E402

from grasp_geometry import geometry_for_kind  # noqa: E402

META_TOPIC = "/competition/grasp_goal_meta"
RGB_TOPIC = "/right_camera/color/image_raw"
INFO_TOPIC = "/right_camera/color/camera_info"
ODOM_TOPIC = "/slamware_ros_sdk_server_node/odom"
RESULT_DIR = Path("/workspace/baseline/debug_data")
RESULT_PATH = RESULT_DIR / "servo_result.json"


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class HandEyeServo(Node):
    def __init__(self, *, timeout: float, max_iterations: int,
                 lateral_tol_m: float, u_sign: float,
                 u0: float = 0.0, v0: float = 0.0) -> None:
        super().__init__("hand_eye_servo")
        self.timeout = float(timeout)
        self.max_iterations = int(max_iterations)
        self.lateral_tol_m = float(lateral_tol_m)
        self.u_sign = float(u_sign)
        # 光轴标定：相机光轴与抓取轴的固定像素偏移（在"必然成功"的标定
        # 位姿上测得的质心读数即对中参考，之后每次测量先扣除）。
        self.u0 = float(u0)
        self.v0 = float(v0)
        self.started_at = time.monotonic()

        self.kdl = MMK2Kdl()
        self.joints: dict[str, float] = {}
        self.x = self.y = self.yaw = None
        self.kind = ""
        self.target_world: tuple[float, float, float] | None = None

        self.fx = self.fy = self.cx = self.cy = None
        self.latest_image = None          # (wall_time, np.ndarray BGR)
        self.done = False

        self.create_subscription(String, META_TOPIC, self._on_meta, 10)
        self.create_subscription(Image, RGB_TOPIC, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_info, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        # 微调用执行器：关节1（肩部横摆→世界 x）与升降柱（世界 z 1:1）
        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray, "/right_arm_forward_position_controller/commands", 10)
        self.spine_cmd_pub = self.create_publisher(
            Float64MultiArray, "/spine_forward_position_controller/commands", 10)

    # ---------- callbacks ----------
    def _on_meta(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        world = payload.get("object_world")
        if isinstance(world, list) and len(world) == 3:
            self.kind = str(payload.get("kind", "")).strip().lower()
            self.target_world = (float(world[0]), float(world[1]), float(world[2]))

    def _on_info(self, message: CameraInfo) -> None:
        if len(message.k) >= 6:
            self.fx, self.fy = float(message.k[0]), float(message.k[4])
            self.cx, self.cy = float(message.k[2]), float(message.k[5])

    def _on_image(self, message: Image) -> None:
        if message.encoding not in ("bgr8", "rgb8"):
            return
        buf = np.frombuffer(message.data, dtype=np.uint8)
        h, w = message.height, message.width
        if buf.size < h * w * 3:
            return
        frame = buf.reshape(h, w, 3)
        if message.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.latest_image = (time.monotonic(), frame)

    def _on_joints(self, message: JointState) -> None:
        self.joints = {
            name: float(message.position[i])
            for i, name in enumerate(message.name)
            if i < len(message.position)
        }

    def _on_odom(self, message) -> None:
        p = message.pose.pose
        self.x, self.y = float(p.position.x), float(p.position.y)
        self.yaw = yaw_from_quaternion(p.orientation)

    # ---------- helpers ----------
    def _ee_world(self) -> tuple[float, float, float] | None:
        if not self.joints or self.x is None:
            return None
        names = [f"right_arm_joint{i}" for i in range(1, 7)]
        if "slide_joint" not in self.joints or any(n not in self.joints for n in names):
            return None
        q = [self.joints["slide_joint"]] + [self.joints[n] for n in names]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        try:
            _, transform = self.kdl.forward_kinematics(q, index="right")
            local = transform[:3, 3]
            return (
                self.x + c * float(local[0]) - s * float(local[1]),
                self.y + s * float(local[0]) + c * float(local[1]),
                float(local[2]),
            )
        except Exception:
            return None

    def _find_blob(self, frame: np.ndarray) -> tuple[float, float, float] | None:
        """饱和度团块质心 (u, v, 宽度px)。货架板白/灰、夹爪黑都被滤掉。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 60, 50), (180, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        h, w = mask.shape
        cx_img, cy_img = w / 2.0, h / 2.0
        best = None
        best_dist = 1e9
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < max(1500, h * w * 0.004):
                continue
            bw = float(stats[index, cv2.CC_STAT_WIDTH])
            bh = float(stats[index, cv2.CC_STAT_HEIGHT])
            # 形状过滤：目标罐是大块团（宽高比适中）；货架红条是细长条
            # （一条边远大于另一条）——实测红条混入会让伺服量到货架上去
            if min(bw, bh) < 60:
                continue
            aspect = max(bw, bh) / max(1.0, min(bw, bh))
            if aspect > 2.5:
                continue
            u, v = centroids[index]
            dist = math.hypot(u - cx_img, (v - cy_img) * 0.5)  # 横向优先
            if dist < best_dist:
                best_dist = dist
                best = (float(u), float(v), bw)
        return best

    # ---------- main ----------
    def run(self) -> int:
        result: dict[str, object] = {"measured": False, "ok": True}
        # 等相机内参 + 一帧新图（阶段开关打开后手眼才有流）
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and (self.fx is None or self.latest_image is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.fx is None or self.latest_image is None:
            result["reason"] = "camera_info 或图像流不可用（阶段开关未开？）"
            self._write_result(result)
            return 0

        geometry = geometry_for_kind(self.kind) if self.kind else None
        item_width = 0.065
        if geometry is not None and geometry.dimensions_m[0] > 0:
            item_width = min(geometry.dimensions_m[0], geometry.dimensions_m[1])

        iterations = []
        correction_total = [0.0, 0.0]
        converged = False
        standoff = 0.0
        for iteration in range(self.max_iterations):
            stamp, frame = self.latest_image
            age = time.monotonic() - stamp
            if age > 2.0:
                rclpy.spin_once(self, timeout_sec=0.2)
                stamp, frame = self.latest_image
                if time.monotonic() - stamp > 2.0:
                    break
            blob = self._find_blob(frame)
            # 诊断存图：标注团块框与图像中心，便于离线核对伺服量的是不是目标
            try:
                vis = frame.copy()
                if blob is not None:
                    u, v, wpx = blob
                    cv2.rectangle(vis, (int(u - wpx / 2), int(v - wpx / 2)),
                                  (int(u + wpx / 2), int(v + wpx / 2)), (0, 255, 0), 2)
                h, w = vis.shape[:2]
                cv2.line(vis, (int(w/2) - 20, int(h/2)), (int(w/2) + 20, int(h/2)), (0, 0, 255), 1)
                cv2.line(vis, (int(w/2), int(h/2) - 20), (int(w/2), int(h/2) + 20), (0, 0, 255), 1)
                RESULT_DIR.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(RESULT_DIR / "servo_frame.jpg"), vis)
            except Exception:
                pass
            if blob is None:
                result["reason"] = "画面内无合格目标团块"
                break
            u, v, width_px = blob
            if width_px < 8:
                break
            standoff = self.fx * item_width / width_px
            standoff = max(0.04, min(0.50, standoff))
            # 先扣除光轴标定偏移 (u0,v0)，再算偏差
            err_lateral = (u - self.cx - self.u0) * standoff / self.fx
            err_vertical = (v - self.cy - self.v0) * standoff / self.fy
            iterations.append({
                "u": round(u, 1), "v": round(v, 1), "width_px": round(width_px, 1),
                "standoff_m": round(standoff, 3),
                "lateral_err_m": round(err_lateral, 4),
                "vertical_err_m": round(err_vertical, 4),
            })
            if abs(err_lateral) <= self.lateral_tol_m and abs(err_vertical) <= 0.015:
                converged = True
                break
            # ---- 预抓取位横向校准（主人的分阶段方案第②步）----
            # 关节选择用数值 FK 灵敏度：j3 是水平面关节（dx=0.335, dz=0），
            # 关节1 不是（dx=0.128 且污染 y/z——用它修正无效，实测误差不变）。
            # j3 的 y 副作用由 creep 的绝对 y 停止线自动吸收（底盘前送补偿）。
            ee = self._ee_world()
            if ee is None:
                break
            q = [self.joints["slide_joint"]] + [
                self.joints[f"right_arm_joint{i}"] for i in range(1, 7)
            ]
            h = 0.02
            try:
                _, t0 = self.kdl.forward_kinematics(q, index="right")
                q3 = list(q)
                q3[3] += h   # j3
                _, t3 = self.kdl.forward_kinematics(q3, index="right")
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                wx0 = c * t0[0, 3] - s * t0[1, 3]
                wx3 = c * t3[0, 3] - s * t3[1, 3]
                sens = (wx3 - wx0) / h   # world x per j3 rad
            except Exception:
                sens = 0.0
            # 修正方向/幅度用实测标定：dj3=+0.15 时误差 +6cm（数值 FK 预言
            # +x 方向与实际相反）→ 经验灵敏度 -0.4 m/rad。
            # 阻尼 0.5：全额修正在罐子两侧 ±6cm 振荡不收敛（实测 +3.7/+6.0/-6.0），
            # 半量迭代 2-3 次收敛到 ±1cm。
            dj3 = max(-0.08, min(0.08, -err_lateral / 0.4 * 0.5))
            sens = -0.4
            if abs(dj3) < 1e-3:
                continue
            arm_msg = Float64MultiArray()
            arm_msg.data = [q[1], q[2], q[3] + dj3, q[4], q[5], q[6], 1.0]
            self.arm_cmd_pub.publish(arm_msg)
            correction_total[0] += dj3 * sens
            self.get_logger().info(
                f"横向校准 {iteration+1}: dj3={dj3:+.4f} "
                f"(err_lat={err_lateral*100:+.1f}cm, sens={sens:.3f})"
            )
            # 等关节真的到位：真正睡 1.2s（spin_once 会被高频回调立即打断，
            # 0.04s 后就测下一帧会读到未动的画面），且要求拿到比修正时刻
            # 新的图像帧
            t_move = time.monotonic()
            while time.monotonic() - t_move < 1.2 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
            while (self.latest_image and self.latest_image[0] < t_move
                   and time.monotonic() - t_move < 4.0 and rclpy.ok()):
                rclpy.spin_once(self, timeout_sec=0.05)
            continue

        result.update({
            "measured": True,
            "converged": converged,
            "standoff_m": round(standoff, 3) if iterations else 0.0,
            "iterations": iterations,
            "correction": [round(correction_total[0], 4), 0.0, round(correction_total[1], 4)],
            "corrected_world": (
                [round(v, 4) for v in self.target_world] if self.target_world else None
            ),
            "kind": self.kind,
            "ee_world": [round(v, 4) for v in (self._ee_world() or (0, 0, 0))],
        })
        self._write_result(result)
        return 0

    def _write_result(self, result: dict[str, object]) -> None:
        result["schema_version"] = 1
        result["wall_time"] = time.time()
        try:
            RESULT_DIR.mkdir(parents=True, exist_ok=True)
            RESULT_PATH.write_text(
                json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            self.get_logger().info(f"伺服结果已写入 {RESULT_PATH}: {json.dumps(result, ensure_ascii=False)[:300]}")
        except OSError as exc:
            self.get_logger().error(f"写入伺服结果失败：{exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="手眼伺服：抓前横向/高度偏差测量与修正")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--max-iterations", type=int, default=3)
    parser.add_argument("--lateral-tol-m", type=float, default=0.012)
    parser.add_argument("--u-sign", type=float,
                        default=float(os.getenv("SUPERMARKET_SERVO_U_SIGN", "-1.0")),
                        help="图像 u 轴到世界 x 轴的方向符号（一次实测标定）")
    parser.add_argument("--u0", type=float, default=float(os.getenv("SUPERMARKET_SERVO_U0", "5.9")),
                        help="光轴标定：对中参考 u 像素（2026-09-12 苹果标定位姿实测）")
    parser.add_argument("--v0", type=float, default=float(os.getenv("SUPERMARKET_SERVO_V0", "65.2")),
                        help="光轴标定：对中参考 v 像素（同上；fx≈767 cx=480 cy=300 自洽验证）")
    args = parser.parse_args()
    if args.execute and args.confirm != "servo":
        print("拒绝执行：必须使用 --confirm servo")
        return 2
    rclpy.init()
    node = HandEyeServo(
        timeout=args.timeout,
        max_iterations=args.max_iterations,
        lateral_tol_m=args.lateral_tol_m,
        u_sign=args.u_sign,
        u0=args.u0,
        v0=args.v0,
    )
    try:
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.run()
    except Exception:
        code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
