#!/usr/bin/env python3
"""Start the official random 3DGS Server without altering rclpy lifecycle.

The official Server owns rclpy.init()/shutdown(). This wrapper only selects a
reproducible five-target task queue and imports the official entry as a normal
module, avoiding nested __main__ execution and rclpy monkey-patching.
"""
from __future__ import annotations

# Allow sibling development helpers mounted under /workspace/baseline to be imported.
import os as _bootstrap_os
import sys as _bootstrap_sys
_bootstrap_sys.path.insert(0, _bootstrap_os.path.dirname(__file__))

import importlib.util
import json
import os
import random
import signal
import runpy
import sys
import traceback
import threading
from pathlib import Path

SERVER_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
SERVER = SERVER_DIR / "supermarket_sorting_server.py"
LAYOUT = SERVER_DIR / "retail_competition_layout.json"
ALLOWED_TASK_SHELVES = frozenset(("B", "C", "D", "E"))


class LifecycleStopEvent(threading.Event):
    """记录首次停止请求，区分 ROS Context 关闭和 worker 主动停止。"""

    def set(self) -> None:
        if not self.is_set():
            print("[server] lifecycle stop_event.set() caller:", flush=True)
            traceback.print_stack(limit=8)
        super().set()


def _truthy(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def int_env(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    return default if not raw else int(raw)


def choose_targets() -> list[dict[str, str]]:
    layout = json.loads(LAYOUT.read_text(encoding="utf-8"))
    products = [
        {
            "body": str(item["body"]),
            "kind": str(item["object_kind"]),
            "shelf": str(item.get("shelf", "")).upper(),
        }
        for item in layout
        if item.get("object_kind") and str(item.get("shelf", "")).upper() in ALLOWED_TASK_SHELVES
    ]
    if len(products) != 36:
        raise RuntimeError(f"expected 36 B/C/D/E products in layout, got {len(products)}")

    requested = os.getenv("SUPERMARKET_TASKS", "").strip()
    if requested and requested.lower() not in {"all", "*"}:
        bodies = [
            item.strip()
            for item in requested.replace(";", ",").split(",")
            if item.strip()
        ]
        by_body = {item["body"]: item for item in products}
        unknown = [body for body in bodies if body not in by_body]
        if unknown:
            raise ValueError("unknown SUPERMARKET_TASKS: " + ", ".join(unknown))
        selected = [by_body[body] for body in dict.fromkeys(bodies)]
    else:
        count = int_env("SUPERMARKET_TASK_COUNT", 5)
        if not 1 <= count <= len(products):
            raise ValueError(f"SUPERMARKET_TASK_COUNT must be in [1,45], got {count}")
        seed = int_env(
            "SUPERMARKET_TASK_SELECTION_SEED",
            int_env("SUPERMARKET_SEED", 11),
        )
        selected = random.Random(seed).sample(products, count)

    if not selected:
        raise RuntimeError("random task selection produced no targets")
    os.environ["SUPERMARKET_TASKS"] = ",".join(item["body"] for item in selected)
    return selected


def main() -> int:
    if os.getenv("SUPERMARKET_FIXED_BASELINE", "0").strip().lower() in {
        "1", "true", "yes", "on"
    }:
        raise RuntimeError("random bootstrap refuses SUPERMARKET_FIXED_BASELINE=1")
    os.environ.setdefault("SUPERMARKET_RANDOMIZE", "1")
    os.environ.setdefault("SUPERMARKET_RANDOMIZE_OBSTACLES", "1")
    os.environ.setdefault("SUPERMARKET_TASK_COUNT", "5")
    os.environ.setdefault("SUPERMARKET_EXPECTED_TARGET_COUNT", "5")
    os.environ.setdefault("DISCOVERSE_ASSETS_DIR", str(SERVER_DIR / "models"))

    selected = choose_targets()
    print(
        "[random-bootstrap] selected="
        + json.dumps(selected, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )
    os.chdir(SERVER_DIR)
    sys.path.insert(0, str(SERVER_DIR))
    # Import the official module without changing its rclpy module object.
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    spec = importlib.util.spec_from_file_location("official_random_server", str(SERVER))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load Server: {SERVER}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    if not rclpy.ok():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    print(f"[random-bootstrap] ROS context ready={rclpy.ok()}", flush=True)

    from server_debug_spawn import patch_build_config
    patch_build_config(module)

    # Do not call the official module.main() here: it owns a second lifecycle
    # boundary while two background threads are starting.  Keep one explicit
    # ROS context, one executor, and one owner for shutdown ordering.
    import threading
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    config = module.build_config()
    print(
        f"[random-bootstrap] after build_config ROS context ready={rclpy.ok()} "
        f"default={rclpy.get_default_context().ok()}",
        flush=True,
    )
    exec_node = module.TaskMMK2ROS2(config)
    exec_node.reset()

    stop_event = LifecycleStopEvent()
    previous_signal_handlers = {
        signal.SIGINT: signal.getsignal(signal.SIGINT),
        signal.SIGTERM: signal.getsignal(signal.SIGTERM),
    }

    def handle_signal(signum, _frame) -> None:
        print(f"[server] received signal {signum}; requesting orderly stop", flush=True)
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    reset_request = threading.Event()
    exec_node._ros_stop_event = stop_event
    context = rclpy.get_default_context()
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(exec_node)

    referee = None
    referee_bridge = None
    if _truthy(os.getenv("SUPERMARKET_ENABLE_SCORE", "1")):
        referee_dir = Path(__file__).resolve().parent / "official_baseline" / "examples" / "supermarket_sorting"
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
        referee_bridge = RefereeBridge(referee, config, reset_request)
        executor.add_node(referee_bridge)
        referee_bridge.publish_state(float(exec_node.mj_data.time), exec_node.mj_data)
        print(
            f"[server] referee enabled: targets={len(target_bodies)} "
            "topics=/referee/taskinfo,/referee/gameinfo,/referee/score "
            "service=/supermarket_sorting/reset_run",
            flush=True,
        )

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
    if getattr(exec_node.config, "lidar_s2_sim", False):
        print("[server] lidar enabled: /slamware_ros_sdk_server_node/scan (12 Hz)", flush=True)

    return_code = 0
    loop_count = 0
    try:
        last_referee_publish = -1.0
        while rclpy.ok(context=context) and not stop_event.is_set():
            if reset_request.is_set():
                exec_node.reset()
                if referee is not None:
                    referee.reset(exec_node.mj_data)
                reset_request.clear()
                if referee_bridge is not None:
                    referee_bridge.acknowledge_reset(exec_node.mj_data)
                print("[server] referee reset_run applied", flush=True)
            if not exec_node.running:
                print("[server] simulator running flag was cleared; keeping ROS Server alive", flush=True)
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
        print(
            f"[server] main loop exit condition: ros_ok={rclpy.ok(context=context)} "
            f"stop_event={stop_event.is_set()} running={exec_node.running}",
            flush=True,
        )
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[server] simulation loop stopped: {type(exc).__name__}: {exc}", flush=True)
        return_code = 1
    finally:
        # One owner, deterministic order: stop workers, join them, then
        # destroy the node and finally shut down the shared ROS context.
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
            except Exception as exc:
                print(f"[server] referee bridge remove warning: {type(exc).__name__}: {exc}", flush=True)
            try:
                referee_bridge.destroy_node()
            except Exception as exc:
                print(f"[server] referee bridge destroy warning: {type(exc).__name__}: {exc}", flush=True)
        try:
            executor.remove_node(exec_node)
        except Exception as exc:
            print(f"[server] executor remove warning: {type(exc).__name__}: {exc}", flush=True)
        try:
            exec_node.destroy_node()
        except Exception as exc:
            print(f"[server] destroy node warning: {type(exc).__name__}: {exc}", flush=True)
        try:
            executor.shutdown()
        except Exception as exc:
            print(f"[server] executor shutdown warning: {type(exc).__name__}: {exc}", flush=True)
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)
        signal.signal(signal.SIGINT, previous_signal_handlers[signal.SIGINT])
        signal.signal(signal.SIGTERM, previous_signal_handlers[signal.SIGTERM])
        print(
            f"[server] lifecycle closed: loops={loop_count} "
            f"pub_alive={pubtopic_thread.is_alive()} spin_alive={spin_thread.is_alive()}",
            flush=True,
        )
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())




