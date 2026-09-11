#!/usr/bin/env bash
# WSL 运行状态监视器。
#
# 目的：WSL 发行版在仿真运行 30-40 秒后被强制重启，所有进程一并消失。本进程
# 同样会被杀掉，但已落到原生盘的内容可以保留崩溃前最后几秒的状态，用于判断
# 是否存在瞬时内存爆炸 / 进程数激增 / 交换区打满等情况。
#
# 输出：每 INTERVAL 秒一行，字段固定便于事后解析。
set -uo pipefail

OUT="${OUT:-/opt/jbgz/baseline/debug_data/memwatch.log}"
INTERVAL="${INTERVAL:-2}"

: > "$OUT"

while true; do
  TS=$(date +%s)
  MEM=$(free -m | awk '/^Mem:/{print $2","$3","$7}')
  SWAPU=$(free -m | awk '/^Swap:/{print $3}')
  UP=$(cut -d' ' -f1 /proc/uptime)
  NPROC=$(ps -e --no-headers 2>/dev/null | wc -l)
  DOCKER_N=$(docker ps -q 2>/dev/null | wc -l)
  echo "$TS mem_total,used,avail=$MEM swap_used=$SWAPU uptime=$UP procs=$NPROC docker=$DOCKER_N" >> "$OUT"
  sleep "$INTERVAL"
done
