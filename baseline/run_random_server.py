#!/usr/bin/env python3
"""Import the lifecycle launcher as a normal module before starting ROS."""
from __future__ import annotations
import importlib.util
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
TARGET=HERE/'lifecycle_random_server_launcher.py'
spec=importlib.util.spec_from_file_location('bootstrap_test',str(TARGET))
if spec is None or spec.loader is None: raise RuntimeError(f'cannot load {TARGET}')
module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
raise SystemExit(module.main())