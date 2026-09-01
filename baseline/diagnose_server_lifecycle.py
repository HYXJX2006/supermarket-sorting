#!/usr/bin/env python3
"""开发诊断入口：只记录 ROS2 生命周期事件，不改变官方 Server 逻辑。"""
from __future__ import annotations

import os
import runpy
import sys
import traceback

import rclpy
from rclpy.context import Context

SERVER_DIR = "/workspace/supermarket_sorting_task/examples/supermarket_sorting"
SERVER = f"{SERVER_DIR}/supermarket_sorting_server.py"
os.chdir(SERVER_DIR)
sys.path.insert(0, SERVER_DIR)

_original_ok = rclpy.ok
_original_shutdown = rclpy.shutdown
_original_context_shutdown = Context.shutdown
_original_context_try_shutdown = Context.try_shutdown
_last_ok = None


def _trace_ok(*args, **kwargs):
    global _last_ok
    value = bool(_original_ok(*args, **kwargs))
    if value != _last_ok:
        _last_ok = value
        print(f"[lifecycle-trace] rclpy.ok -> {value}", flush=True)
    return value


def _trace_shutdown(*args, **kwargs):
    print("[lifecycle-trace] rclpy.shutdown called", file=sys.stderr, flush=True)
    traceback.print_stack(file=sys.stderr)
    return _original_shutdown(*args, **kwargs)


def _trace_context_shutdown(self, *args, **kwargs):
    print("[lifecycle-trace] Context.shutdown called", file=sys.stderr, flush=True)
    traceback.print_stack(file=sys.stderr)
    return _original_context_shutdown(self, *args, **kwargs)


def _trace_context_try_shutdown(self, *args, **kwargs):
    print("[lifecycle-trace] Context.try_shutdown called", file=sys.stderr, flush=True)
    traceback.print_stack(file=sys.stderr)
    return _original_context_try_shutdown(self, *args, **kwargs)


rclpy.ok = _trace_ok  # type: ignore[method-assign]
rclpy.shutdown = _trace_shutdown  # type: ignore[method-assign]
Context.shutdown = _trace_context_shutdown  # type: ignore[method-assign]
Context.try_shutdown = _trace_context_try_shutdown  # type: ignore[method-assign]
runpy.run_path(SERVER, run_name="__main__")
