#!/usr/bin/env python3
"""启动比赛 Server 的开发包装器。

仅负责设置资源目录和开发同步参数，然后执行官方 Server 入口。
官方入口自行调用 rclpy.init()/shutdown()；不对 ROS2 API 做 monkey-patch。
"""

from __future__ import annotations

# Allow sibling development helpers mounted under /workspace/baseline to be imported.
import os as _bootstrap_os
import sys as _bootstrap_sys
_bootstrap_sys.path.insert(0, _bootstrap_os.path.dirname(__file__))

import os
import importlib.util
import sys


SERVER = "/workspace/supermarket_sorting_task/examples/supermarket_sorting/supermarket_sorting_server.py"
SERVER_DIR = "/workspace/supermarket_sorting_task/examples/supermarket_sorting"


def _disable_dev_sync() -> None:
    """让本地开发测试跳过实时同步；正式运行默认不触发。"""
    if os.getenv("SUPERMARKET_DEV_DISABLE_SYNC", "0") != "1":
        return
    from discoverse.robots_env.mmk2_base import MMK2Cfg
    MMK2Cfg.sync = False
    print("[bootstrap] development mode: cfg.sync=False", flush=True)




def main() -> int:
    os.environ.setdefault(
        "DISCOVERSE_ASSETS_DIR",
        os.path.join(SERVER_DIR, "models"),
    )
    os.chdir(SERVER_DIR)
    if SERVER_DIR not in sys.path:
        sys.path.insert(0, SERVER_DIR)

    _disable_dev_sync()
    import importlib.util
    import rclpy

    original_init = rclpy.init
    def init_idempotent(*args, **kwargs):
        if not rclpy.ok():
            return original_init(*args, **kwargs)
        return None
    rclpy.init = init_idempotent
    if not rclpy.ok():
        original_init()

    spec = importlib.util.spec_from_file_location("official_supermarket_server", SERVER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load official Server: {SERVER}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    from server_debug_spawn import patch_build_config
    patch_build_config(module)
    return int(module.main() or 0)


if __name__ == "__main__":
    # Launch a clean -c process which imports this wrapper as a normal module.
    # This avoids Python's nested __main__ + rclpy context interaction.
    loader = (
        "import importlib.util,sys; "
        "p=sys.argv[1]; spec=importlib.util.spec_from_file_location('bootstrap_entry',p); "
        "m=importlib.util.module_from_spec(spec); sys.modules[spec.name]=m; "
        "spec.loader.exec_module(m); raise SystemExit(m.main())"
    )
    os.execv(sys.executable, [sys.executable, "-c", loader, __file__])
