#!/usr/bin/env bash
# maidong 寻优：扫描 GRIP_CLOSE（闭爪指令，越小越紧）。
#
# 问题：maidong 夹住了（129 帧双指接触、闭爪后开口 66.1 mm ≈ 直径 65 mm），
# 但 0.643 kg 是九类最重，lift 只抬到 4.8 mm 商品就在指间下滑 ——
# 位置夹持的预压力不足以让摩擦力撑住重力。
#
# 机制：close_gripper 把 right_gripper 位置指令发给
# /right_arm_forward_position_controller/commands。GRIP_CLOSE=0.08（默认最紧档）
# 时指头闭到「贴住商品表面」（66 mm ≈ 65 mm + 1 mm 变形）—— 即几乎零预压。
# 指令调小（0.06 / 0.05 / 0.04）→ 目标位置更深 → 对商品产生预压紧力 →
# 摩擦力上限随正压力增大，0.643 kg 才撑得住。
#
# 对照：pingguo(0.251 kg)/chengzi(0.253 kg) 零预压也够用；maidong 是唯一
# 需要预压的类别 —— 这正是「每类商品自己的参数」的意义。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
IMAGE_CLIENT="crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-30}"
NAV_YAW="1.5707963267948966"

OUTDIR="$ROOT/baseline/debug_data/sweep_maidong_GRIP_CLOSE_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

{
  echo "=================================================================="
  echo "maidong 扫描 GRIP_CLOSE（闭爪指令，越小越紧/预压越大）"
  echo "货位 E/L2/C2  world=(1.805 3.243 0.956)  质量最大 0.643 kg"
  echo "输出=$OUTDIR"
  echo "=================================================================="
  printf '%-10s %10s %10s %10s %10s %9s  %s\n' GRIP_CLOSE SCORE RISE_m TILT_deg SHIFT_m GRIP% REASON
} | tee "$OUTDIR/summary.txt"

BEST=-999999; BESTV=""
for gc in 0.060 0.050 0.040; do
  echo "[$(date +%H:%M:%S)] >>> GRIP_CLOSE=$gc"
  log="$OUTDIR/gc_$gc.log"
  # grip_close 经 grasp_goal_meta 下发（known_slot_plan_publisher 读
  # grasp_geometry.py，后者支持 SUPERMARKET_<KIND>_* 环境变量覆盖），
  # 而 run_item_grasp.sh 会把环境里的 SUPERMARKET_* 透传进 actions 容器。
  export SUPERMARKET_MAIDONG_GRIP_CLOSE="$gc"
  KIND=maidong TARGET_ID=product_041 SLOT=E/L2/C2 \
  WORLD="1.805 3.243 0.956" LABEL="maidong_gc_$gc" \
  NAV_ROUTE="1.92,2.475,$NAV_YAW;1.737,2.475,$NAV_YAW" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$log" 2>&1
  unset SUPERMARKET_MAIDONG_GRIP_CLOSE

  T=$(grep -a '遥测目录: ' "$log" | tail -1 | sed 's/.*遥测目录: //')
  if [[ -z "$T" || ! -d "$T" ]]; then
    printf '%-10s %10s %10s %10s %10s %9s  %s\n' "$gc" n/a - - - - NO_TELEMETRY | tee -a "$OUTDIR/summary.txt"
    continue
  fi
  rel="${T#$ROOT/baseline/}"
  sc=$(docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" "$IMAGE_CLIENT" \
       bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=/workspace/baseline/$rel --baseline-z=0.956 --json" 2>/dev/null | tail -1)
  val=$(echo "$sc"  | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['score'])" 2>/dev/null || echo "n/a")
  rise=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['z_rise_m'])" 2>/dev/null || echo "-")
  tilt=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_tilt_deg'])" 2>/dev/null || echo "-")
  shf=$(echo "$sc"  | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_xy_shift_m'])" 2>/dev/null || echo "-")
  gr=$(echo "$sc"   | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['gripped_ratio']*100,1))" 2>/dev/null || echo "-")
  rs=$(echo "$sc"   | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['reason'])" 2>/dev/null || echo "?")
  printf '%-10s %10s %10s %10s %10s %9s  %s\n' "$gc" "$val" "$rise" "$tilt" "$shf" "$gr" "$rs" | tee -a "$OUTDIR/summary.txt"
  if [[ "$val" != "n/a" ]] && python3 -c "import sys;sys.exit(0 if float('$val')>float('$BEST') else 1)" 2>/dev/null; then
    BEST="$val"; BESTV="$gc"
  fi
done

{
  echo
  echo "=================================================================="
  echo "最优 GRIP_CLOSE = $BESTV   (score=$BEST)"
  echo "=================================================================="
} | tee -a "$OUTDIR/summary.txt"
