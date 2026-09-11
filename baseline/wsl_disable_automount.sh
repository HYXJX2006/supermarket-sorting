#!/usr/bin/env bash
# 禁用 WSL 的 /mnt/* 自动挂载（9P 通道），用于排除 WSL 发行版反复重启的根因。
#
# 现象：仿真运行中 WSL 发行版被强制重启，内核日志固定为
#   Exception: Operation canceled @p9io.cpp:258 (AcceptAsync)
#   systemd-shutdow -> EXT4 unmount -> 重新 mount -> 全部容器消失（docker.sock EOF）
# 已排除的因素：
#   - 不是 OOM（free 显示可用内存充足，journal 无 OOM 记录）
#   - 不是代码位于 9P（把工作区搬到 /opt/jbgz 后仍然崩溃）
# 因此怀疑是 9P 服务自身（p9io）在无外部访问时失败。禁用 automount 可彻底
# 移除该通道，用于验证假设。
#
# 回滚：cp /etc/wsl.conf.bak-pre-noautomount /etc/wsl.conf 后 wsl --shutdown
set -euo pipefail

CONF=/etc/wsl.conf
BAK=/etc/wsl.conf.bak-pre-noautomount

if [[ ! -f "$BAK" ]]; then
  cp "$CONF" "$BAK"
  echo "已备份 $CONF -> $BAK"
else
  echo "备份已存在，保留原有备份：$BAK"
fi

python3 - "$CONF" <<'PY'
import re
import sys

path = sys.argv[1]
text = open(path, encoding="utf-8").read()

# 去掉已有的 [automount] 段（到下一个 [section] 或文件尾）
text = re.sub(r"\n?\[automount\][^\[]*", "\n", text, flags=re.S)
text = text.strip() + "\n\n[automount]\nenabled = false\n"

open(path, "w", encoding="utf-8").write(text)
print("已写入 automount 禁用配置")
PY

echo
echo "--- 当前 $CONF ---"
cat "$CONF"
echo "--- 备份 ---"
cat "$BAK"
