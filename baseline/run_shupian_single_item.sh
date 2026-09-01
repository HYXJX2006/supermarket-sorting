#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
exec python3 -u /workspace/baseline/single_item_executor.py \
  --execute --confirm single_item --manage-workers \
  --max-items 1 --timeout 1200 --max-attempts 1