#!/usr/bin/env bash
# 将 Windows 侧工作区（9P: /mnt/e）增量同步到 WSL 原生盘（/opt/jbgz）。
#
# 背景：反复观测到 WSL 发行版在 MuJoCo Server 启动瞬间被强制重启，内核日志为
#   Exception: Operation canceled @p9io.cpp:258 (AcceptAsync)
#   systemd-shutdow -> EXT4 unmount -> 重新 mount
# p9io.cpp 是 WSL 的 9P 文件共享服务（即 /mnt/* 通道）。仿真实时渲染会同时读入
# 大量模型/纹理/库文件，9P 被打爆后整个发行版重启（表象：docker.sock EOF、
# 容器瞬间消失）。因此仿真必须在 ext4 原生盘上运行。
#
# 本脚本只同步代码。路径全部硬编码，避免通过 bash -lc 传变量时被外层 shell
# 提前展开（曾导致 rsync 源退化为 "/"，把整个 Linux 根目录当输入）。
set -uo pipefail

SRC=/mnt/e/workspace/claude_workplace/揭榜挂帅
DST=/opt/jbgz

if [[ ! -d "$SRC/baseline" ]]; then
  echo "错误：源目录不存在 $SRC/baseline" >&2
  exit 2
fi

mkdir -p "$DST"
mkdir -p "$DST/baseline/debug_data"

echo "[$(date +%H:%M:%S)] 增量同步代码 $SRC -> $DST"
echo "  排除：debug_data / __pycache__ / *.pyc / official_baseline / .git / 环境 / *.tar"

rsync -a \
  --exclude 'baseline/debug_data/' \
  --exclude 'baseline/official_baseline/' \
  --exclude 'baseline/__pycache__/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude 'tmp_pdf_extract/' \
  --exclude '.git/' \
  --exclude '环境/' \
  --exclude '*.tar' \
  --exclude '*.bak' \
  --exclude '*.zip' \
  --exclude '*.7z' \
  "$SRC/" "$DST/"
RC=$?

echo "[$(date +%H:%M:%S)] rsync 退出码=$RC"

echo
echo "=== 核心文件校验 ==="
check() {
  local rel="$1" pat="$2"
  local p="$DST/$rel"
  if [[ ! -f "$p" ]]; then
    printf "MISS  %s\n" "$rel"; return
  fi
  local n
  n=$(grep -c -- "$pat" "$p" 2>/dev/null || true)
  printf "OK    %-45s %6s bytes  命中 '%s' x%s\n" "$rel" "$(stat -c%s "$p")" "$pat" "${n:-0}"
}
check "baseline/grasp_geometry.py"              "grip_gate_for_opening"
check "baseline/lift_controller.py"             "收到抓取元数据"
check "baseline/known_slot_plan_publisher.py"   "window_seconds"
check "baseline/run_pingguo_hold.sh"            "object-stop-only"
check "baseline/calibration_server_bootstrap.py" "Referee"

echo
echo "=== 目标体积 ==="
du -sh "$DST" 2>/dev/null
du -sh "$DST/baseline/debug_data" 2>/dev/null

mkdir -p "$DST/baseline/debug_data"

# 注意：不要在此清空 debug_data。其中 grasp_evaluation/ 存放着已采集的裁判
# 遥测（target_state.jsonl），递归删除会销毁证据。需要干净的遥测目录时，
# 由运行脚本按时间戳新建目录，不要靠同步脚本来清。
#
# debug_data 被整体排除以避开 6.5 GB 数据，但该目录下也存放着采集脚本
# （referee_target_monitor.py 等）。漏同步会让遥测容器启动即报
# "can't open file ... No such file or directory"，导致裁判真值全程为零。
echo
echo "=== 补同步 debug_data 下的脚本（*.py，不含数据）==="
find "$SRC/baseline/debug_data" -maxdepth 2 -name '*.py' -print0 2>/dev/null \
  | while IFS= read -r -d '' f; do
      rel="${f#"$SRC/baseline/debug_data/"}"
      mkdir -p "$DST/baseline/debug_data/$(dirname "$rel")"
      cp -p "$f" "$DST/baseline/debug_data/$rel"
      echo "  + $rel"
    done

echo
echo "=== 采集脚本校验 ==="
ls -la "$DST/baseline/debug_data/referee_target_monitor.py" 2>&1
ls -la "$DST/baseline/debug_data" 2>/dev/null | head -10
