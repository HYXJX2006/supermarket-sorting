#!/usr/bin/env python3
"""从 run_random5_autopilot.sh 提取已验证的 server 启动段 → test_server_start.sh"""
src = open('run_random5_autopilot.sh', encoding='utf-8').read()
lines = src.split('\n')
start = end = None
for i, ln in enumerate(lines):
    if 'docker run -d --name "$SERVER_NAME"' in ln and start is None:
        start = i
    if start is not None and 'random_server_bootstrap.py' in ln:
        end = i
        break
assert start is not None and end is not None, (start, end)
block = lines[start:end + 1]
block_txt = '\n'.join(block)
block_txt = block_txt.replace('docker run -d --name "$SERVER_NAME"', 'docker run -d --name obstacle_test_server')

out = []
for ln in block_txt.split('\n'):
    s = ln.strip()
    if s.startswith('-e SUPERMARKET_TASK_COUNT'):
        out.append('  -e SUPERMARKET_TASK_COUNT=1 ' + chr(92))
    elif s.startswith('-e SUPERMARKET_EXPECTED_TARGET_COUNT'):
        out.append('  -e SUPERMARKET_EXPECTED_TARGET_COUNT=1 ' + chr(92))
    else:
        out.append(ln)

header = '''#!/usr/bin/env bash
# 单测 server 启动（从 run_random5_autopilot.sh 原样提取，保证 odom 等数据流一致）
# 用法: bash test_server_start.sh <SEED>
SEED="${1:-7}"
set -uo pipefail
ROOT=/mnt/e/workspace/claude_workplace/揭榜挂帅
DOMAIN=99
HEADLESS="${SUPERMARKET_HEADLESS:-1}"
USE_GS="${SUPERMARKET_USE_GS:-1}"
GS_BATCH_RENDER="${SUPERMARKET_GS_BATCH_RENDER:-1}"
GS_HEAD_ONLY="${SUPERMARKET_GS_HEAD_ONLY:-1}"
GS_ASYNC="${SUPERMARKET_GS_ASYNC:-1}"
RANDOMIZE_OBSTACLES="${SUPERMARKET_RANDOMIZE_OBSTACLES:-1}"
X11_DISPLAY="${X11_DISPLAY:-172.25.192.1:1}"
CRUISE=1.4
OBSTACLE_DIST=0.35
DETECTION_CONFIDENCE=0.15
REMOVE_CORRIDOR_OBSTACLES="${SUPERMARKET_REMOVE_CORRIDOR_OBSTACLES:-0}"
TASKS_SPEC="${SUPERMARKET_TASKS:-}"
SCORE_ENABLED="${SUPERMARKET_ENABLE_SCORE:-0}"
TASK_ORDER_LOCKED=0
'''

open('test_server_start.sh', 'w', encoding='utf-8', newline='\n').write(
    header + '\n'.join(out) + '\necho "test server up"\n')
print('extracted', len(out), 'lines')
