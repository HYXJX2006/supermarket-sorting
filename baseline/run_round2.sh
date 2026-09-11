#!/usr/bin/env bash
# 第二轮标定：验证 lift 竞态修复 + sanmingzhi 下探深度寻优。
#
# 依据是批量基线（见 memory 2026-09-10）：
#   heweidao  → 双指接触 151 帧、倾角 1.4°，说明已跨住，仅 lift 被默认门禁误拒；
#               修了 lift 元数据竞态后重跑应能通过。
#   sanmingzhi→ 开口中位 39.9 mm，而该商品 x 维 0.0650 m（65 mm），
#               开口远小于商品宽度 ⇒ 夹空了。它在 L1（z=0.5484）最底层，
#               怀疑 deploy_offset.dz=-0.010 的下探量不足，末端停在商品上方。
#               故扫描 DEPLOY_DZ，取更负（下探更深）的值对比。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（Bash 工具 run_in_background=true），
#    否则 vmIdleTimeout 会在 60 秒后关掉整台 WSL 虚拟机。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
NAV_YAW="1.5707963267948966"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-35}"

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

log "步骤 1/2：heweidao 重跑（验证 lift 竞态修复）"
KIND=heweidao \
TARGET_ID=product_029 \
SLOT=D/L1/C2 \
WORLD="0.92 3.243 0.5515" \
LABEL=heweidao_r2 \
NAV_ROUTE="1.92,2.475,$NAV_YAW;0.852,2.475,$NAV_YAW" \
  bash "$ROOT/baseline/run_item_grasp.sh" \
  > "$ROOT/baseline/debug_data/round2_heweidao.log" 2>&1
rc1=$?

# 判定：只看裁判真值
T1=$(grep -a '遥测目录: ' "$ROOT/baseline/debug_data/round2_heweidao.log" | tail -1 | sed 's/.*遥测目录: //')
if [[ -n "$T1" && -d "$T1" ]]; then
  echo "--- heweidao 裁判真值 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/analyse_pingguo_calibration.py --dir=${T1/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=0.5515" 2>&1 | tail -12
  echo "--- 评分 ---"
  docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" \
    crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client \
    bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=${T1/#$ROOT\/baseline/\/workspace\/baseline} --baseline-z=0.5515" 2>&1 | tail -4
else
  echo "heweidao 无遥测（rc=$rc1）"
fi

log "步骤 2/2：sanmingzhi 扫描 DEPLOY_DZ（下探深度）"
KIND=sanmingzhi \
TARGET_ID=product_028 \
SLOT=D/L1/C1 \
WORLD="0.70 3.243 0.5484" \
PARAM=DEPLOY_DZ \
VALUES="-0.060 -0.035 -0.010" \
NAV_X_OFFSET=0.068 \
  bash "$ROOT/baseline/run_param_sweep.sh" \
  > "$ROOT/baseline/debug_data/round2_sanmingzhi_sweep.log" 2>&1

echo
echo "================================================================"
echo "第二轮结束"
echo "================================================================"
tail -30 "$ROOT/baseline/debug_data/round2_sanmingzhi_sweep.log" 2>/dev/null
