#!/usr/bin/env python3
"""固定单目标标定 Server：与 server_bootstrap 相同的固定基线，但启用裁判桥。

与 ``random_server_bootstrap.py`` 的区别：
  - 不强制 SUPERMARKET_RANDOMIZE=1，允许 SUPERMARKET_FIXED_BASELINE=1；
  - 同样启用 RefereeBridge，因此会发布
    ``/referee/gameinfo``、``/referee/target_state``（含 grounded truth
    的 ``gripped`` / ``finger_contacts`` / ``position_world``），
    供只读记录器取证，避免用控制器退出码冒充抓取成功。
"""
from __future__ import annotations

import importlib.util
import os
import signal
import sys
import threading
import traceback
from pathlib import Path

_BOOTSTRAP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_BOOTSTRAP_DIR))

SERVER_DIR = "/workspace/supermarket_sorting_task/examples/supermarket_sorting"
SERVER = SERVER_DIR + "/supermarket_sorting_server.py"


def _truthy(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def main() -> int:
    os.environ.setdefault("DISCOVERSE_ASSETS_DIR", os.path.join(SERVER_DIR, "models"))
    os.chdir(SERVER_DIR)
    if SERVER_DIR not in sys.path:
        sys.path.insert(0, SERVER_DIR)

    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    spec = importlib.util.spec_from_file_location("official_calib_server", SERVER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load Server: {SERVER}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    from server_debug_spawn import patch_build_config
    patch_build_config(module)

    config = module.build_config()
    exec_node = module.TaskMMK2ROS2(config)
    exec_node.reset()

    stop_event = threading.Event()
    previous_signal_handlers = {
        signal.SIGINT: signal.getsignal(signal.SIGINT),
        signal.SIGTERM: signal.getsignal(signal.SIGTERM),
    }

    def handle_signal(signum, _frame) -> None:
        print(f"[server] received signal {signum}; requesting orderly stop", flush=True)
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    exec_node._ros_stop_event = stop_event
    context = rclpy.get_default_context()
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(exec_node)

    referee = None
    referee_bridge = None
    if _truthy(os.getenv("SUPERMARKET_ENABLE_SCORE", "1")):
        referee_dir = _BOOTSTRAP_DIR / "official_baseline" / "examples" / "supermarket_sorting"
        sys.path.insert(0, str(referee_dir))
        from referee import Referee
        from referee_bridge import RefereeBridge

        referee_config = os.getenv(
            "SUPERMARKET_REFEREE_CONFIG",
            str(referee_dir / "referee_json" / "retail_referee_config.json"),
        )
        target_bodies = [str(item["id"]) for item in getattr(config, "task_targets", [])]
        object_bodies = [str(name) for name in getattr(config, "obj_list", [])]
        referee = Referee(exec_node.mj_model, target_bodies, object_bodies, referee_config)
        referee.reset(exec_node.mj_data)
        referee_bridge = RefereeBridge(referee, config, threading.Event())
        executor.add_node(referee_bridge)
        referee_bridge.publish_state(float(exec_node.mj_data.time), exec_node.mj_data)
        print(
            f"[server] referee enabled: targets={len(target_bodies)} "
            "topics=/referee/gameinfo,/referee/target_state,/referee/score",
            flush=True,
        )
    else:
        print("[server] referee disabled (SUPERMARKET_ENABLE_SCORE=0)", flush=True)

    def spin_node() -> None:
        try:
            while not stop_event.is_set() and rclpy.ok(context=context):
                executor.spin_once(timeout_sec=0.1)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception as exc:
            print(f"[server] spin thread stopped: {type(exc).__name__}: {exc}", flush=True)
            stop_event.set()

    spin_thread = threading.Thread(target=spin_node, name="ros-spin", daemon=True)
    pubtopic_thread = threading.Thread(
        target=exec_node.thread_pubros2topic,
        args=(24, stop_event),
        name="ros-topic-publisher",
        daemon=True,
    )
    spin_thread.start()
    pubtopic_thread.start()

    return_code = 0
    loop_count = 0
    try:
        last_referee_publish = -1.0
        while rclpy.ok(context=context) and not stop_event.is_set():
            if not exec_node.running:
                exec_node.running = True
            exec_node.step(exec_node.target_control)
            if referee is not None:
                referee.update(exec_node.mj_data)
                sim_time = float(exec_node.mj_data.time)
                if referee_bridge is not None and (
                    last_referee_publish < 0.0 or sim_time - last_referee_publish >= 0.1
                ):
                    referee_bridge.publish_state(sim_time, exec_node.mj_data)
                    last_referee_publish = sim_time
            loop_count += 1
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[server] simulation loop stopped: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
        return_code = 1
    finally:
        stop_event.set()
        exec_node.running = False
        pubtopic_thread.join(timeout=5.0)
        spin_thread.join(timeout=5.0)
        if spin_thread.is_alive():
            executor.shutdown()
            spin_thread.join(timeout=2.0)
        if referee_bridge is not None:
            try:
                executor.remove_node(referee_bridge)
                referee_bridge.destroy_node()
            except Exception as exc:
                print(f"[server] referee cleanup warning: {exc}", flush=True)
        try:
            executor.remove_node(exec_node)
        except Exception:
            pass
        try:
            exec_node.destroy_node()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)
        signal.signal(signal.SIGINT, previous_signal_handlers[signal.SIGINT])
        signal.signal(signal.SIGTERM, previous_signal_handlers[signal.SIGTERM])
        print(f"[server] lifecycle closed: loops={loop_count}", flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
