#!/usr/bin/env python3
"""查官方 kele.pt 的 YOLO 版本（depth/width multiples 决定 n/s/m/l）。"""
import torch

paths = [
    "/workspace/supermarket_sorting_task/examples/supermarket_sorting/perception/checkpoints/kele.pt",
    "/tmp/ck/kele.pt",
]
for p in paths:
    try:
        ckpt = torch.load(p, map_location="cpu", weights_only=False)
        m = ckpt.get("model")
        cfg = getattr(m, "yaml", {}) or {}
        print("found:", p)
        print("  nc:", cfg.get("nc"), "| depth_multiple:", cfg.get("depth_multiple"),
              "| width_multiple:", cfg.get("width_multiple"))
        args = ckpt.get("train_args") or {}
        if isinstance(args, dict):
            print("  train model:", args.get("model", "?"))
        d, w = cfg.get("depth_multiple"), cfg.get("width_multiple")
        if d == 0.33 and w == 0.25:
            print("  => YOLOv8n")
        elif d == 0.33 and w == 0.50:
            print("  => YOLOv8s")
        elif d == 0.67 and w == 0.75:
            print("  => YOLOv8l")
        elif d == 1.00 and w == 1.00:
            print("  => YOLOv8x")
        elif d == 0.33 and w == 0.75:
            print("  => YOLOv8m")
        break
    except FileNotFoundError:
        continue
    except Exception as exc:  # noqa: BLE001
        print("加载失败:", type(exc).__name__, str(exc)[:200])
        break
