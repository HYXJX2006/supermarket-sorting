#!/usr/bin/env python3
"""Start the official random 3DGS Server without altering rclpy lifecycle.

The official Server owns rclpy.init()/shutdown(). This wrapper only selects a
reproducible five-target task queue and imports the official entry as a normal
module, avoiding nested __main__ execution and rclpy monkey-patching.
"""
from __future__ import annotations

import importlib.util
import json
import os
import random
import runpy
import sys
from pathlib import Path

SERVER_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting")
SERVER = SERVER_DIR / "supermarket_sorting_server.py"
LAYOUT = SERVER_DIR / "retail_competition_layout.json"


def int_env(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    return default if not raw else int(raw)


def choose_targets() -> list[dict[str, str]]:
    layout = json.loads(LAYOUT.read_text(encoding="utf-8"))
    products = [
        {"body": str(item["body"]), "kind": str(item["object_kind"])}
        for item in layout
        if item.get("object_kind")
    ]
    if len(products) != 45:
        raise RuntimeError(f"expected 45 products in layout, got {len(products)}")

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
    spec = importlib.util.spec_from_file_location("official_random_server", str(SERVER))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load Server: {SERVER}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    from server_debug_spawn import patch_build_config
    patch_build_config(module)
    return int(module.main() or 0)


if __name__ == "__main__":
    raise SystemExit(main())


