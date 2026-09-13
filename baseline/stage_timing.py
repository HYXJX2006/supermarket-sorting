#!/usr/bin/env python3
"""抓取链各阶段耗时（相对时间 + 步间耗时）。

用法：
  bash -c 'docker logs random5_autopilot_client > /tmp/cl.log 2>&1; python3 stage_timing.py /tmp/cl.log'
"""
import re
import sys

PAT = re.compile(r"\[(INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[([a-z_]+)\]: (.*)")
WATCH = (
    "odom 已稳定", "plan-only IK", "开始动作阶段", "部署计划生成", "部署动作完成",
    "横向校准", "手眼伺服完成", "CREEP 停止", "闭爪执行中", "夹空检测",
    "抬升动作完成", "目标失败", "已放置", "高度闭环", "到达货架观察点",
    "目标导航到位", "预抓取",
)


def main() -> int:
    if len(sys.argv) > 1:
        src = open(sys.argv[1], encoding="utf-8", errors="replace")
    else:
        src = sys.stdin
    ev = []
    for line in src:
        m = PAT.match(line.strip())
        if m:
            ev.append((float(m.group(2)), m.group(3), m.group(4)))
    sel = [e for e in ev if any(k in e[2] for k in WATCH)]
    if not sel:
        print("(没抓到阶段事件；日志行数=%d)" % len(ev))
        return 0
    starts = [i for i, e in enumerate(sel) if "odom 已稳定" in e[2]]
    start = starts[-1] if starts else 0
    chain = sel[start:start + 45]
    t0 = chain[0][0]
    prev = None
    print("相对时间   步间耗时   阶段事件")
    for t, node, txt in chain:
        gap = ("+%5.1fs" % (t - prev)) if prev is not None else "        "
        print("%8.1fs %s   %s" % (t - t0, gap, txt[:100]))
        prev = t
    print("\n总计 %.1fs" % (chain[-1][0] - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
