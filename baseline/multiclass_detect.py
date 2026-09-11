#!/usr/bin/env python3
"""9 类商品 RGB-D 检测节点。

复用官方 kele_detect.py 的相机、深度、odom、关节状态和世界坐标转换，
只把单类 kele 后端替换为 9 类 YOLO 权重。输出：
- /multiclass/detections: vision_msgs/Detection3DArray，world 坐标
- /multiclass/result_image: 带类别和世界坐标的调试图

不读取 Server 内部布局真值，不控制机器人。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import cv2
import numpy as np


TASK_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
# Client 镜像可能没有完整的官方 task tree；baseline 挂载中保留了官方副本。
# 优先使用镜像内路径，缺失时自动回退，避免依赖启动脚本额外拼接 PYTHONPATH。
if not (TASK_DIR / "perception" / "backends.py").is_file():
    bundled_task_dir = Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"
    if (bundled_task_dir / "perception" / "backends.py").is_file():
        TASK_DIR = bundled_task_dir
# Client fallback 还需要把 official_baseline 根目录加入 sys.path，
# 这样 kele_detect.py 才能导入 discoverse 包。
BUNDLED_ROOT = Path(__file__).resolve().parent / "official_baseline"
if BUNDLED_ROOT.is_dir():
    sys.path.insert(0, str(BUNDLED_ROOT))
PERCEPTION_DIR = TASK_DIR / "perception"
sys.path.insert(0, str(TASK_DIR))
sys.path.insert(0, str(PERCEPTION_DIR))

from backends import YoloBackend  # noqa: E402
from kele_detect import KeleDetectNode  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from vision_msgs.msg import Detection3DArray  # noqa: E402
from std_msgs.msg import String  # noqa: E402


CLASS_NAMES = [
    "chengzi",
    "heweidao",
    "kele",
    "kouxiangtang",
    "maidong",
    "pingguo",
    "sanmingzhi",
    "shupian",
    "zhijin",
]

# 仅用于 /multiclass/result_image 的短标签；检测消息、日志和 JSON 仍保留
# 完整英文类别名，避免影响 competition_executor 的类别匹配。
DISPLAY_NAMES = {
    "chengzi": "CH",
    "heweidao": "HWD",
    "kele": "KL",
    "kouxiangtang": "KXT",
    "maidong": "MD",
    "pingguo": "PG",
    "sanmingzhi": "SMZ",
    "shupian": "SP",
    "zhijin": "ZJ",
}
DISPLAY_SHOW_CONF = os.getenv("SUPERMARKET_DISPLAY_SHOW_CONF", "1").strip().lower() not in {"0", "false", "no", "off"}
DISPLAY_SHOW_WORLD = os.getenv("SUPERMARKET_DISPLAY_SHOW_WORLD", "0").strip().lower() in {"1", "true", "yes", "on"}
DISPLAY_FONT_SCALE = max(0.35, min(0.8, float(os.getenv("SUPERMARKET_DISPLAY_FONT_SCALE", "0.5"))))
DISPLAY_LINE_THICKNESS = max(1, min(3, int(os.getenv("SUPERMARKET_DISPLAY_LINE_THICKNESS", "2"))))
# 商品框应是局部目标；超大框通常来自墙面、地面或货架结构误检。
MAX_BOX_WIDTH_FRAC = max(0.10, min(0.80, float(os.getenv("SUPERMARKET_MAX_BOX_WIDTH_FRAC", "0.30"))))
MAX_BOX_HEIGHT_FRAC = max(0.15, min(0.90, float(os.getenv("SUPERMARKET_MAX_BOX_HEIGHT_FRAC", "0.45"))))
MAX_BOX_AREA_FRAC = max(0.02, min(0.50, float(os.getenv("SUPERMARKET_MAX_BOX_AREA_FRAC", "0.12"))))


class MultiClassYoloBackend(YoloBackend):
    CLASS_NAMES = CLASS_NAMES

    def detect(
        self,
        rgb,
        depth,
        K,
        T_cam_world=None,
    ):
        """使用后端保存的置信度阈值，避免 Ultralytics 默认阈值提前过滤。"""
        if self.model is None:
            return []

        results = self.model(rgb, conf=self.conf_thresh, verbose=False)[0]
        image_height, image_width = rgb.shape[:2]
        max_box_width = image_width * MAX_BOX_WIDTH_FRAC
        max_box_height = image_height * MAX_BOX_HEIGHT_FRAC
        max_box_area = image_width * image_height * MAX_BOX_AREA_FRAC
        detections = []
        for box in results.boxes:
            conf = float(box.conf.item())
            if conf < self.conf_thresh:
                continue
            cls_id = int(box.cls.item())
            if cls_id >= len(self.CLASS_NAMES):
                continue
            x0, y0, x1, y1 = map(int, box.xyxy[0].cpu().numpy())
            box_width = max(1, x1 - x0)
            box_height = max(1, y1 - y0)
            if (
                box_width > max_box_width
                or box_height > max_box_height
                or box_width * box_height > max_box_area
            ):
                continue
            detections.append(
                {
                    "class": self.CLASS_NAMES[cls_id],
                    "x": (x0 + x1) // 2,
                    "y": (y0 + y1) // 2,
                    "w": box_width,
                    "h": box_height,
                    "conf": conf,
                }
            )
        return self._suppress_duplicate_boxes(detections)

    def _suppress_duplicate_boxes(self, detections):
        """对同类别重叠框做一次轻量 class-aware NMS。

        Ultralytics 已经执行模型内部 NMS，但不同配置/训练权重仍可能在
        货架商品上产生重复框；这里在发布 ROS 检测前再做一次保守去重。
        """
        iou_threshold = float(os.getenv("SUPERMARKET_NMS_IOU", "0.45"))
        iou_threshold = min(0.95, max(0.05, iou_threshold))
        kept = []
        for candidate in sorted(detections, key=lambda item: item["conf"], reverse=True):
            duplicate = False
            for previous in kept:
                if candidate["class"] != previous["class"]:
                    continue
                if self._box_iou(candidate, previous) >= iou_threshold:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(candidate)
        return kept

    @staticmethod
    def _box_iou(left, right) -> float:
        lx0 = left["x"] - left["w"] / 2.0
        ly0 = left["y"] - left["h"] / 2.0
        lx1 = left["x"] + left["w"] / 2.0
        ly1 = left["y"] + left["h"] / 2.0
        rx0 = right["x"] - right["w"] / 2.0
        ry0 = right["y"] - right["h"] / 2.0
        rx1 = right["x"] + right["w"] / 2.0
        ry1 = right["y"] + right["h"] / 2.0
        inter_w = max(0.0, min(lx1, rx1) - max(lx0, rx0))
        inter_h = max(0.0, min(ly1, ry1) - max(ly0, ry0))
        inter = inter_w * inter_h
        area_left = max(0.0, lx1 - lx0) * max(0.0, ly1 - ly0)
        area_right = max(0.0, rx1 - rx0) * max(0.0, ry1 - ry0)
        union = area_left + area_right - inter
        return inter / union if union > 1e-9 else 0.0


class MultiClassDetectNode(KeleDetectNode):
    # WSL/MuJoCo 深度缓冲可能把无效背景写成 uint16 饱和值 65535（65.535 m）。
    # 该值经像素反投影会制造 y≈70 m 的伪目标，必须在进入世界坐标前剔除。
    MIN_VALID_DEPTH_M = 0.15
    MAX_VALID_DEPTH_M = 8.0
    MIN_VALID_PATCH_PIXELS = 3

    @classmethod
    def patch_depth_m(cls, depth_img, u, v, r=4):
        """读取有效深度，拒绝 0、NaN/Inf 和 uint16 饱和值等异常值。"""
        height, width = depth_img.shape[:2]
        y0, y1 = max(0, int(v) - r), min(height, int(v) + r + 1)
        x0, x1 = max(0, int(u) - r), min(width, int(u) + r + 1)
        patch = depth_img[y0:y1, x0:x1].astype(np.float32, copy=False)
        valid = patch[(patch >= cls.MIN_VALID_DEPTH_M * 1000.0)
                      & (patch <= cls.MAX_VALID_DEPTH_M * 1000.0)
                      & np.isfinite(patch)]
        if valid.size < cls.MIN_VALID_PATCH_PIXELS:
            return 0.0
        return float(np.median(valid)) * 1e-3

    def _capture_write_json(self, name: str, payload: dict) -> None:
        if not self.capture_enabled:
            return
        try:
            (self.capture_dir / name).write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        except Exception as exc:
            self.get_logger().warning(f"识别留档 JSON 写入失败：{exc}")

    def _capture_depth_preview(self, depth) -> np.ndarray:
        if depth is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        arr = np.asarray(depth)
        valid = arr[np.isfinite(arr) & (arr > 0) & (arr < 8000)]
        if valid.size == 0:
            return np.zeros((*arr.shape[:2], 3), dtype=np.uint8)
        gray = (np.clip(arr.astype(np.float32), 0, 8000) * (255.0 / 8000.0)).astype(np.uint8)
        return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)

    def _capture_resize(self, image: np.ndarray | None) -> np.ndarray | None:
        """按低占用设置缩小留档图，不改变检测器实际推理分辨率。"""
        if image is None:
            return None
        max_width = self.capture_max_width
        if max_width <= 0 or image.shape[1] <= max_width:
            return image
        scale = max_width / float(image.shape[1])
        height = max(1, int(round(image.shape[0] * scale)))
        return cv2.resize(image, (max_width, height), interpolation=cv2.INTER_AREA)

    def _capture_write_image(self, path: Path, image: np.ndarray | None, quality: int | None = None) -> bool:
        if image is None:
            return False
        image = self._capture_resize(image)
        params = []
        if path.suffix.lower() in {".jpg", ".jpeg"}:
            params = [cv2.IMWRITE_JPEG_QUALITY, int(quality or self.capture_jpeg_quality)]
        return bool(cv2.imwrite(str(path), image, params))

    def _capture_task_cb(self, msg: String) -> None:
        try:
            self.capture_task = json.loads(msg.data)
        except Exception:
            self.capture_task = {"raw": msg.data}

    def _capture_detection_cb(self, msg: Detection3DArray) -> None:
        records = []
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            p = result.pose.pose.position
            records.append({
                "class": str(result.hypothesis.class_id),
                "confidence": round(float(result.hypothesis.score), 5),
                "world": [round(float(p.x), 5), round(float(p.y), 5), round(float(p.z), 5)],
            })
        self.capture_detections = records
        self._capture_maybe_save(msg.header.stamp)

    def _capture_result_cb(self, msg: Image) -> None:
        try:
            self.capture_result = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"识别结果图转换失败：{exc}")
            return
        self._capture_maybe_save(msg.header.stamp)

    def _capture_save_frame(self, stamp, *, stage: str) -> None:
        if not self.capture_enabled or self.capture_rgb is None:
            return
        now = time.monotonic()
        self.capture_frame_index += 1
        stem = f"frame_{self.capture_frame_index:06d}"
        # 首帧可能尚未有结果图/深度图，使用同一时刻的 RGB 占位，JSON 明确标注阶段。
        result = self.capture_result if self.capture_result is not None else self.capture_rgb
        rgb_path = self.capture_dir / f"{stem}_rgb.jpg"
        result_path = self.capture_dir / f"{stem}_result.jpg"
        depth_path = self.capture_dir / f"{stem}_depth.jpg"
        rgb_ok = self._capture_write_image(rgb_path, self.capture_rgb)
        result_ok = self._capture_write_image(result_path, result)
        depth_ok = self._capture_write_image(
            depth_path, self._capture_depth_preview(self.capture_depth),
            quality=self.capture_depth_jpeg_quality,
        )
        self.capture_saved_rgb_frame = self.capture_pending_rgb_frame
        payload = {
            "frame_index": self.capture_frame_index,
            "source_rgb_frame": self.capture_saved_rgb_frame,
            "capture_stage": stage,
            "elapsed_s": round(now - self.capture_started_at, 4),
            "ros_stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "task": self.capture_task,
            "base_position": [round(float(v), 5) for v in self.base_pos] if self.base_pos is not None else None,
            "slide": round(float(self.slide), 5),
            "head": [round(float(v), 5) for v in self.head],
            "detections": self.capture_detections,
            "files": {
                "rgb": rgb_path.name if rgb_ok else None,
                "result": result_path.name if result_ok else None,
                "depth": depth_path.name if depth_ok else None,
            },
            "image_policy": {
                "max_width_px": self.capture_max_width,
                "jpeg_quality": self.capture_jpeg_quality,
                "depth_jpeg_quality": self.capture_depth_jpeg_quality,
            },
        }
        self._capture_write_json(f"{stem}.json", payload)
        if self.capture_frame_index == 1 or self.capture_frame_index % 10 == 0:
            self.get_logger().info(
                f"已保存识别帧 {stem} stage={stage} detections={len(self.capture_detections)} "
                f"dir={self.capture_dir}"
            )

    def _capture_save_first_frame(self, stamp) -> None:
        """在第一张真实 RGB 回调中立即留档，避免等机器人移动后才保存。"""
        if self.capture_frame_index == 0:
            self.capture_last_save = time.monotonic()
            self.capture_pending_rgb_frame = self.capture_rgb_input_frame
            self._capture_save_frame(stamp, stage="first_rgb")

    def _capture_maybe_save(self, stamp) -> None:
        if not self.capture_enabled or self.capture_rgb is None:
            return
        if not self.capture_pending:
            return
        save_stamp = self.capture_pending_stamp or stamp
        self.capture_pending = False
        self.capture_last_save = time.monotonic()
        self._capture_save_frame(save_stamp, stage=f"every_{self.capture_every_n}_rgb")

    def depth_cb(self, msg: Image):
        self.depth_callback_count += 1
        try:
            self.capture_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"识别深度图转换失败：{exc}")
        super().depth_cb(msg)

    def rgb_cb(self, msg: Image):
        """复用官方坐标链路，但仅在调试图上绘制紧凑短标签。"""
        self.rgb_callback_count += 1
        if self.rgb_callback_count in (1, 10) or self.rgb_callback_count % 300 == 0:
            self.get_logger().info(
                f"视觉输入状态：rgb={self.rgb_callback_count} depth={self.depth_callback_count} "
                f"detections={self.detection_publish_count}"
            )
        try:
            self.capture_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.capture_rgb_input_frame += 1
            # 首帧在推理前立即保存；之后每 N 张真实 RGB 输入设置一次待保存标记。
            self._capture_save_first_frame(msg.header.stamp)
            if (
                self.capture_frame_index > 0
                and self.capture_rgb_input_frame % self.capture_every_n == 0
            ):
                self.capture_pending = True
                self.capture_pending_stamp = msg.header.stamp
                self.capture_pending_rgb_frame = self.capture_rgb_input_frame
        except Exception as exc:
            self.get_logger().warning(f"识别原图转换失败：{exc}")
        if self.K is None or self._depth_msg is None:
            return
        T_cam_world = self.camera_world_tmat()
        if T_cam_world is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(self._depth_msg)
        dets = self.detector.detect(rgb, depth, self.K, T_cam_world)
        out = []
        vis = rgb.copy() if self.pub_res_img else rgb
        for d in dets:
            u, v = int(d["x"]), int(d["y"])
            depth_m = self.patch_depth_m(depth, u, v)
            if depth_m <= 0.0:
                continue
            p_cam = self.pixel_to_cam(u, v, depth_m)
            p_world = (T_cam_world @ np.array([p_cam[0], p_cam[1], p_cam[2], 1.0]))[:3]
            rec = {"class": d["class"], "conf": d.get("conf", 0.0), "world": p_world}
            out.append(rec)
            if not self.pub_res_img:
                continue
            w, h = int(d["w"]), int(d["h"])
            x0, y0 = max(0, u - w // 2), max(0, v - h // 2)
            x1, y1 = min(vis.shape[1] - 1, u + w // 2), min(vis.shape[0] - 1, v + h // 2)
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), DISPLAY_LINE_THICKNESS)
            label = DISPLAY_NAMES.get(d["class"], d["class"][:8])
            if DISPLAY_SHOW_CONF:
                label += f" {float(d.get('conf', 0.0)):.2f}"
            if DISPLAY_SHOW_WORLD:
                label += f" ({p_world[0]:.2f},{p_world[1]:.2f},{p_world[2]:.2f})"
            text_x = x0
            text_y = max(16, y0 - 6)
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, DISPLAY_FONT_SCALE, 1
            )
            box_x1 = min(vis.shape[1] - 1, text_x + text_width + 4)
            box_y0 = max(0, text_y - text_height - baseline - 2)
            cv2.rectangle(vis, (text_x, box_y0), (box_x1, text_y + 2), (0, 110, 0), -1)
            cv2.putText(
                vis, label, (text_x + 2, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                DISPLAY_FONT_SCALE, (255, 255, 255), 1, cv2.LINE_AA
            )
        self.detection_publish_count += len(out)
        self.publish_detections(out, msg.header.stamp)
        if self.pub_res_img:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))

    def __init__(self, checkpoint: str, device: str, confidence: float) -> None:
        # Parent supplies the official RGB-D and robot-state subscriptions.
        super().__init__(backend="blob", pub_res_img=True, device=device)
        self.detector = MultiClassYoloBackend(
            checkpoint,
            conf_thresh=confidence,
            device=device,
        )
        # Redirect the two parent publishers to the multi-class namespace.
        self.det_pub = self.create_publisher(
            Detection3DArray, "/multiclass/detections", 10
        )
        self.img_pub = self.create_publisher(
            Image, "/multiclass/result_image", 5
        )

        # 每次 detector 启动都清空“当前运行”目录，避免把上一次运动后的
        # 画面误认为本次识别首帧；历史数据仍保留在 recognition_runs 中。
        self.capture_enabled = os.getenv("SUPERMARKET_CAPTURE", "1") != "0"
        self.capture_interval = max(0.2, float(os.getenv("SUPERMARKET_CAPTURE_INTERVAL", "1.0")))
        # 留档默认采用中等缩略图与 JPEG，避免长时间运行迅速占满磁盘。
        self.capture_every_n = max(1, int(os.getenv("SUPERMARKET_CAPTURE_EVERY_N", "4")))
        self.capture_max_width = max(320, int(os.getenv("SUPERMARKET_CAPTURE_MAX_WIDTH", "960")))
        self.capture_jpeg_quality = min(95, max(35, int(os.getenv("SUPERMARKET_CAPTURE_JPEG_QUALITY", "55"))))
        self.capture_depth_jpeg_quality = min(90, max(30, int(os.getenv("SUPERMARKET_CAPTURE_DEPTH_JPEG_QUALITY", "45"))))
        self.capture_dir = Path(os.getenv(
            "SUPERMARKET_CAPTURE_DIR",
            "/workspace/baseline/debug_data/recognition_current",
        ))
        self.capture_dir.parent.mkdir(parents=True, exist_ok=True)
        if self.capture_enabled:
            # 当前目录可能包含长时间运行产生的数十万张图片；逐文件删除会
            # 阻塞检测器启动。通过同文件系统原子改名归档，立即释放当前目录，
            # 再创建空目录供本轮写入。归档目录不参与本轮识别。
            if self.capture_dir.exists() and any(self.capture_dir.iterdir()):
                archive_root = self.capture_dir.parent / "recognition_runs"
                archive_root.mkdir(parents=True, exist_ok=True)
                archive_dir = archive_root / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                try:
                    self.capture_dir.rename(archive_dir)
                    self.get_logger().info(f"识别留档旧目录已原子归档：{archive_dir}")
                except OSError as exc:
                    # 只读挂载或跨文件系统时不阻塞检测器启动；当前目录
                    # 保留旧文件，但本轮仍继续运行并通过 session.json 标识。
                    self.get_logger().warning(f"识别留档旧目录归档失败，继续复用：{exc}")
            self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.capture_frame_index = 0
        self.capture_rgb_input_frame = 0
        self.capture_pending = False
        self.capture_pending_stamp = None
        self.capture_pending_rgb_frame = 0
        self.capture_saved_rgb_frame = 0
        self.capture_last_save = 0.0
        self.capture_started_at = time.monotonic()
        self.capture_rgb = None
        self.capture_depth = None
        self.capture_result = None
        self.capture_detections = []
        self.capture_task = None
        self.rgb_callback_count = 0
        self.depth_callback_count = 0
        self.detection_publish_count = 0
        self.create_subscription(Image, "/multiclass/result_image", self._capture_result_cb, 5)
        self.create_subscription(Detection3DArray, "/multiclass/detections", self._capture_detection_cb, 10)
        task_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/supermarket_sorting/task", self._capture_task_cb, task_qos)
        self._capture_write_json("session.json", {
            "started_at": datetime.now().astimezone().isoformat(),
            "capture_dir": str(self.capture_dir),
            "capture_enabled": self.capture_enabled,
            "capture_every_n_rgb": self.capture_every_n,
            "capture_max_width_px": self.capture_max_width,
            "capture_jpeg_quality": self.capture_jpeg_quality,
            "capture_depth_jpeg_quality": self.capture_depth_jpeg_quality,
            "first_frame_policy": "save on first /head_camera/color/image_raw callback before inference",
            "display_labels": {
                "names": DISPLAY_NAMES,
                "show_confidence": DISPLAY_SHOW_CONF,
                "show_world_coordinates": DISPLAY_SHOW_WORLD,
                "font_scale": DISPLAY_FONT_SCALE,
                "line_thickness": DISPLAY_LINE_THICKNESS,
            },
            "read_only": True,
            "source_topics": {
                "rgb": "/head_camera/color/image_raw",
                "depth": "/head_camera/aligned_depth_to_color/image_raw",
                "result": "/multiclass/result_image",
                "detections": "/multiclass/detections",
                "odom": "/slamware_ros_sdk_server_node/odom",
            },
        })
        self.get_logger().info(
            f"识别留档={'on' if self.capture_enabled else 'off'} dir={self.capture_dir} "
            f"every_{self.capture_every_n}_rgb max_width={self.capture_max_width}px "
            f"jpeg={self.capture_jpeg_quality}（首帧立即保存，之后按帧抽样）"
        )
        self.get_logger().info(
            f"multiclass detector up: classes={CLASS_NAMES}, checkpoint={checkpoint}, "
            f"device={device}, confidence={confidence:.2f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="9 类商品 RGB-D YOLO 检测")
    parser.add_argument(
        "--weights",
        default="/workspace/baseline/debug_data/multiclass_train_smoke/multiclass_smoke/weights/best.pt",
    )
    parser.add_argument("--device", default="cuda", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--confidence", type=float, default=0.25)
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    node = MultiClassDetectNode(args.weights, args.device, args.confidence)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



