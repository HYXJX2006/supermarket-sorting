#!/usr/bin/env bash
# zhijin 俯抓（top 咬棱）扫描：TOP_GRASP_DZ = 末端咬棱高度（相对商品中心）。
#
# 背景：zhijin 172×85×88，最小对距 85 mm > 夹爪开口 80 mm，任何正交姿态
# 都无法包夹。俯抓改为咬顶面两条棱（棱距 = y 向对距 85 mm），
# 指距 80 mm 过行程 5 mm 硬咬 —— 原理同 maidong 预压。且咬棱距离与
# 商品 yaw 无关 ⇒ 不需要拨动改朝向。
#
# 关键配置：
#   SUPERMARKET_ZHIJIN_SURFACE_TO_CENTER_FWD=0
#     俯抓必须正对商品中心上方；侧抓的 RGB-D 表面点修正(0.0425)在此会让
#     deploy 点偏到商品前方 4.25 cm，指头咬不到棱。
#   SUPERMARKET_ZHIJIN_GRIP_FEEDBACK_MAX=1.2
#     两指咬 85 mm 棱距必然过行程（开口 80 mm），门禁必须放行。
#   GRASP_ROT_MODE=top，deploy 两段：悬停(商品上方 0.16)→下压到咬棱高度。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
IMAGE_CLIENT="crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-30}"
export GRASP_ROT_MODE=top
export SUPERMARKET_ZHIJIN_SURFACE_TO_CENTER_FWD=0.0
export SUPERMARKET_ZHIJIN_GRIP_FEEDBACK_MAX=1.2

NAV_YAW="1.5707963267948966"
OUTDIR="$ROOT/baseline/debug_data/sweep_zhijin_TOP_GRASP_DZ_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

{
  echo "=================================================================="
  echo "zhijin 俯抓扫描 TOP_GRASP_DZ（咬棱高度，相对商品中心）"
  echo "货位 D/L2/C1  world=(0.70 3.243 0.895)  顶面 z=0.939"
  echo "输出=$OUTDIR"
  echo "=================================================================="
  printf '%-10s %10s %10s %10s %10s %9s  %s\n' GRASP_DZ SCORE RISE_m TILT_deg SHIFT_m GRIP% REASON
} | tee "$OUTDIR/summary.txt"

BEST=-999999; BESTV=""
# 咬棱高度推算：指头长 72mm 竖直向下，指尖需低于顶棱(z=0.939)约 3cm，
# 末端 z ≈ 0.91+0.072 = 0.982 → dz ≈ 0.982-0.895 = 0.087
for dz in 0.060 0.087 0.115; do
  echo "[$(date +%H:%M:%S)] >>> TOP_GRASP_DZ=$dz"
  log="$OUTDIR/dz_$dz.log"
  TOP_GRASP_DZ=$dz LABEL="zhijin_top_$dz" \
  KIND=zhijin TARGET_ID=product_031 SLOT=D/L2/C1 \
  WORLD="0.70 3.243 0.895" \
  NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$log" 2>&1

  T=$(grep -a '遥测目录: ' "$log" | tail -1 | sed 's/.*遥测目录: //')
  if [[ -z "$T" || ! -d "$T" ]]; then
    printf '%-10s %10s %10s %10s %10s %9s  %s\n' "$dz" n/a - - - - NO_TELEMETRY | tee -a "$OUTDIR/summary.txt"
    continue
  fi
  rel="${T#$ROOT/baseline/}"
  sc=$(docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" "$IMAGE_CLIENT" \
       bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=/workspace/baseline/$rel --baseline-z=0.895 --json" 2>/dev/null | tail -1)
  val=$(echo "$sc"  | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['score'])" 2>/dev/null || echo "n/a")
  rise=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['z_rise_m'])" 2>/dev/null || echo "-")
  tilt=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_tilt_deg'])" 2>/dev/null || echo "-")
  shf=$(echo "$sc"  | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_xy_shift_m'])" 2>/dev/null || echo "-")
  gr=$(echo "$sc"   | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['gripped_ratio']*100,1))" 2>/dev/null || echo "-")
  rs=$(echo "$sc"   | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['reason'])" 2>/dev/null || echo "?")
  printf '%-10s %10s %10s %10s %10s %9s  %s\n' "$dz" "$val" "$rise" "$tilt" "$shf" "$gr" "$rs" | tee -a "$OUTDIR/summary.txt"
  if [[ "$val" != "n/a" ]] && python3 -c "import sys;sys.exit(0 if float('$val')>float('$BEST') else 1)" 2>/dev/null; then
    BEST="$val"; BESTV="$dz"
  fi
done

{
  echo
  echo "=================================================================="
  echo "最优 TOP_GRASP_DZ = $BESTV   (score=$BEST)"
  echo "=================================================================="
} | tee -a "$OUTDIR/summary.txt"
