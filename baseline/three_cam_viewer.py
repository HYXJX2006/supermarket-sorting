#!/usr/bin/env python3
"""三相机实时查看器：head 全局视角 + 左手眼 + 右手眼，横排拼一个窗口。

供主人审查夹取效果用——MuJoCo 主窗口是自由视角，本查看器给三个机位的
真实感知渲染画面（3DGS batched cameras 的输出）。三个窗口合一，免得桌面
被铺满。cv2 窗口经 WSLg X11 显示。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
except Exception:
    _BRIDGE = None


def to_bgr(msg: Image) -> np.ndarray | None:
    try:
        if _BRIDGE is not None:
            return _BRIDGE.imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        pass
    # 兜底：手动转换（RGB8 / BGR8 常见编码）
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.width * msg.height * 3
    if arr.size < expected:
        return None
    frame = arr[:expected].reshape(msg.height, msg.width, 3)
    if "rgb" in msg.encoding.lower():
        frame = frame[:, :, ::-1]
    return frame.copy()


class ThreeCamViewer(Node):
    def __init__(self) -> None:
        super().__init__("three_cam_viewer")
        self.frames: dict[str, np.ndarray] = {}
        for name, topic in [
            ("HEAD", "/head_camera/color/image_raw"),
            ("LEFT", "/left_camera/color/image_raw"),
            ("RIGHT", "/right_camera/color/image_raw"),
        ]:
            self.create_subscription(Image, topic, self._make_cb(name), 2)

    def _make_cb(self, name: str):
        def callback(msg: Image) -> None:
            frame = to_bgr(msg)
            if frame is None:
                return
            h, w = frame.shape[:2]
            scale = 480.0 / max(h, 1)
            frame = cv2.resize(frame, (max(1, int(w * scale)), 480))
            cv2.putText(frame, name, (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0), 2, cv2.LINE_AA)
            self.frames[name] = frame
        return callback


def main() -> int:
    rclpy.init()
    node = ThreeCamViewer()
    import time
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    last_yield = 0.0
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
            now = time.monotonic()
            if now - last_yield >= 0.066:   # ~15fps 足够审查
                last_yield = now
                frames = node.frames
                if frames:
                    tiles = []
                    for name in ("HEAD", "LEFT", "RIGHT"):
                        f = frames.get(name)
                        if f is None:
                            f = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(f, f"{name}: no signal", (12, 34),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                        (0, 0, 255), 2, cv2.LINE_AA)
                        tiles.append(f)
                    canvas = np.hstack(tiles)
                    cv2.imshow("3-camera review (HEAD | LEFT | RIGHT)", canvas)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
