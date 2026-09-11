#!/usr/bin/env python3
"""三相机 MJPEG 网页流：head 全局 + 左手眼 + 右手眼，浏览器实时审查。

client 镜像的 OpenCV 是 headless 构建（cv2.imshow 不可用），故改用网页流：
三路画面横排拼合后以 MJPEG 推到 http://0.0.0.0:8090/ ，Windows 浏览器直接
打开 http://localhost:8090 即可实时观看（WSL2 localhost 自动转发）。
"""
from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
    _CV2 = True
except Exception:
    _CV2 = False

try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
except Exception:
    _BRIDGE = None

PORT = 8090


def to_bgr(msg: Image) -> np.ndarray | None:
    try:
        if _BRIDGE is not None:
            return _BRIDGE.imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        pass
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
        self.lock = threading.Lock()
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
            frame = cv2.resize(frame, (max(1, int(w * scale)), 480)) if _CV2 else frame
            with self.lock:
                self.frames[name] = frame
        return callback

    def compose(self) -> np.ndarray:
        """三路画面横排拼合（无信号的机位显示占位提示）。"""
        with self.lock:
            frames = dict(self.frames)
        tiles = []
        for name in ("HEAD", "LEFT", "RIGHT"):
            f = frames.get(name)
            if f is None:
                f = np.full((480, 640, 3), 40, dtype=np.uint8)
                if _CV2:
                    cv2.putText(f, f"{name}: no signal", (12, 34),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                (0, 0, 255), 2, cv2.LINE_AA)
            tiles.append(f)
        return np.hstack(tiles)


class StreamHandler(BaseHTTPRequestHandler):
    node: ThreeCamViewer

    def do_GET(self):  # noqa: N802
        if self.path != "/":
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                frame = self.node.compose()
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    time.sleep(0.1)
                    continue
                data = buf.tobytes()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                time.sleep(0.066)   # ~15fps
        except Exception:
            pass

    def log_message(self, *args):  # 静默访问日志
        pass


def main() -> int:
    rclpy.init()
    node = ThreeCamViewer()
    StreamHandler.node = node
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
    node.get_logger().info(f"三相机网页流: http://0.0.0.0:{PORT}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
