#!/usr/bin/env python3
"""容器内 GS 渲染器 import 链诊断。"""
import sys

sys.path.insert(0, "/workspace/supermarket_sorting_task")

print("python:", sys.version.split()[0], flush=True)
try:
    import torch
    print("torch OK:", torch.__version__, "cuda_available:", torch.cuda.is_available(), flush=True)
except Exception as e:  # noqa: BLE001
    print("torch FAIL:", type(e).__name__, str(e)[:200], flush=True)

try:
    from gaussian_renderer.gs_renderer_mujoco import GSRendererMuJoCo  # noqa: F401
    print("gs_renderer OK", flush=True)
except Exception as e:  # noqa: BLE001
    print("gs_renderer FAIL:", type(e).__name__, str(e)[:400], flush=True)

try:
    import discoverse  # noqa: F401
    print("discoverse OK", flush=True)
except Exception as e:  # noqa: BLE001
    print("discoverse FAIL:", type(e).__name__, str(e)[:200], flush=True)
