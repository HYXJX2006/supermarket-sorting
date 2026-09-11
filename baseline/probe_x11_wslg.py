#!/usr/bin/env python3
"""WSLg X11 通路自检：验证容器能否经 /tmp/.X11-unix/X0 建立 X11 会话。

背景：此前依赖 Windows 侧 VcXsrv + TCP 6000，但 Windows 防火墙在重启后会
重新识别 WSL 虚拟网卡，导致 172.25.192.1:6000 被拦（外网通、宿主端口全 CLOSED）。
WSLg 提供的是 unix domain socket，不过网络栈，因此不受防火墙影响，是更稳的通路。
"""
import socket
import struct
import sys

SOCK = "/tmp/.X11-unix/X0"


def probe_x11_unix() -> bool:
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(6)
        s.connect(SOCK)
    except Exception as exc:
        print(f"[FAIL] 无法连接 {SOCK}: {exc}")
        return False

    # X11 连接请求：byte-order 'l'(LSB) + pad + proto(11,0) + auth-name-len + auth-data-len + pad
    req = b"l" + b"\x00" + struct.pack("<HHHHH", 11, 0, 0, 0, 0)
    try:
        s.sendall(req)
        resp = s.recv(8)
    except Exception as exc:
        print(f"[FAIL] X11 握手收发失败: {exc}")
        return False
    finally:
        try:
            s.close()
        except Exception:
            pass

    if not resp:
        print("[FAIL] X11 服务端返回空响应")
        return False

    code = resp[0]
    if code == 0:  # Failed
        reason_len = struct.unpack("<H", resp[6:8])[0]
        print(f"[FAIL] X11 连接被拒绝(code=0) reason_len={reason_len}")
        return False
    if code == 2:  # Authenticate
        print("[WARN] X11 要求认证(code=2)，可能需要 xauth")
        return False

    major = struct.unpack("<H", resp[2:4])[0]
    minor = struct.unpack("<H", resp[4:6])[0]
    vendor_len = struct.unpack("<H", resp[6:8])[0]
    print(f"[OK] X11 握手成功 protocol={major}.{minor} vendor_len={vendor_len}")
    return True


def probe_graphics() -> None:
    try:
        import mujoco
        print(f"[OK] mujoco {mujoco.__version__}")
    except Exception as exc:
        print(f"[WARN] import mujoco 失败: {exc}")
    try:
        import glfw
        glfw.init()
        print(f"[OK] glfw {glfw.get_version_string()}")
        glfw.terminate()
    except Exception as exc:
        print(f"[WARN] glfw 初始化失败: {exc}")


if __name__ == "__main__":
    ok = probe_x11_unix()
    probe_graphics()
    sys.exit(0 if ok else 1)
