#!/usr/bin/env bash
# 8 类商品抓取演示：GPU 真实渲染（--gpus all + GLFW/WSLg 窗口）。
#
# 每类完整链 safe_pose → nav → deploy → creep → close_gripper → lift，
# MuJoCo 窗口实时显示，动作结束后保持 HOLD_SECONDS 供观看，然后切换下一类。
# 所有抓取参数来自 grasp_geometry.py 已固化的逐类标定值（calibrated=True），
# 无需环境变量覆盖——本演示就是固化参数的端到端验收。
#
# 顺序（货架 D 为主，maidong 单独走 E 架路线）：
#   1 pingguo      D/L3/C2  球   70mm   （首例闭环验证）
#   2 kele         D/L2/C2  圆柱 53mm
#   3 shupian      D/L1/C3  圆柱 65mm
#   4 kouxiangtang D/L3/C1  圆柱 49mm
#   5 chengzi      D/L3/C3  球   74mm
#   6 heweidao     D/L1/C2  圆台 79mm  （lift 竞态修复的受益者）
#   7 sanmingzhi   D/L1/C1  棱柱 65mm  （surface_to_center_fwd 修正的受益者）
#   8 maidong      E/L2/C2  圆柱 65mm  0.643kg（grip_close 预压的受益者）
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true），
#    否则 vmIdleTimeout 会在 60 秒后关掉整台 WSL 虚拟机。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
NAV_YAW="1.5707963267948966"

# GPU 真实渲染：--gpus all + GLFW 窗口（WSLg），3DGS 高斯泼溅真实感画面
export NO_GPU=0
export HEADLESS=0          # run_item_grasp.sh 据此设 MUJOCO_GL=glfw、DISPLAY=:0
export USE_GS=1            # 3DGS 真实感渲染（商品用照片重建资产，非简单网格）
export HOLD_SECONDS="${HOLD_SECONDS:-20}"
export CREEP_SPEED=0.08    # 演示用稍慢的接近速度，画面更稳

NAV_D="1.92,2.475,$NAV_YAW"

# kind|target|slot|world|nav_route 终点x
ITEMS=(
  "pingguo|product_035|D/L3/C2|0.92 3.243 1.224|0.852"
  "kele|product_032|D/L2/C2|0.92 3.243 0.9235|0.852"
  "shupian|product_030|D/L1/C3|1.14 3.243 0.604|1.072"
  "kouxiangtang|product_034|D/L3/C1|0.70 3.243 1.229|0.632"
  "chengzi|product_036|D/L3/C3|1.14 3.243 1.226|1.072"
  "heweidao|product_029|D/L1/C2|0.92 3.243 0.5515|0.852"
  "sanmingzhi|product_028|D/L1/C1|0.70 3.243 0.5484|0.632"
  "maidong|product_041|E/L2/C2|1.805 3.243 0.956|1.737"
)

log() { echo "[$(date +%H:%M:%S)] === $* ==="; }

log "8 类商品 GPU 渲染演示开始（共 ${#ITEMS[@]} 类，约 35 分钟）"

PASS=0
for entry in "${ITEMS[@]}"; do
  IFS='|' read -r kind target slot world endx <<< "$entry"
  route="${NAV_D};${endx},2.475,$NAV_YAW"
  log ">>> [$kind] slot=$slot  world=$world"

  logf="$ROOT/baseline/debug_data/demo_${kind}.log"
  KIND=$kind TARGET_ID=$target SLOT=$slot WORLD="$world" LABEL="demo_$kind" \
  NAV_ROUTE="$route" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$logf" 2>&1
  rc=$?

  stages=$(grep -ac "STAGE .* = *success" "$logf" 2>/dev/null || grep -ac "STAGE" "$logf" 2>/dev/null)
  verdict=$(grep -a "STAGE lift=success" "$logf" >/dev/null 2>&1 && echo "✅ 完成" || echo "⚠️ 未完整")
  if [[ "$verdict" == "✅"* ]]; then PASS=$((PASS+1)); fi
  log "<<< [$kind] $verdict (stages=$stages, rc=$rc)  [累计通过 $PASS]"
done

log "演示结束：完整链通过 $PASS / ${#ITEMS[@]} 类"
