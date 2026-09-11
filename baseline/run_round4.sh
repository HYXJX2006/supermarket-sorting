#!/usr/bin/env bash
# 第四轮：sanmingzhi 扫描 CREEP_SPEED（接近速度）。
#
# 依据：第三轮 CREEP_STOP_DY 扫描三个值全为 SINGLE_FINGER，但趋势在改善
#   （TILT 0.469→0.152→0.112，SHIFT 0.105→0.087→0.076，均在递减）。
# 同时观测到 xy_shift 高达 7.6–10.5 cm —— **商品被推走了**，说明问题出在
# 接触瞬间的推力，而非停止位置。
#
# sanmingzhi 是三角形截面的三明治（mesh_prism，0.122 kg，很轻），
# 用 0.12 m/s 接近极易把商品推歪，导致只剩单指接触。
# 故扫描更慢的接近速度。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（run_in_background=true）。
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
IMAGE_CLIENT="crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client"
export HEADLESS="${HEADLESS:-1}"
export HOLD_SECONDS="${HOLD_SECONDS:-35}"

NAV_YAW="1.5707963267948966"
OUTDIR="$ROOT/baseline/debug_data/sweep_sanmingzhi_CREEP_SPEED_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

{
  echo "=================================================================="
  echo "sanmingzhi 扫描 CREEP_SPEED（接近速度）  world=(0.70 3.243 0.5484)"
  echo "输出=$OUTDIR"
  echo "=================================================================="
  printf '%-10s %10s %10s %10s %10s %9s  %s\n' SPEED SCORE RISE_m TILT_deg SHIFT_m GRIP% REASON
} | tee "$OUTDIR/summary.txt"

BEST=-999999; BESTV=""
for spd in 0.060 0.030 0.015; do
  echo "[$(date +%H:%M:%S)] >>> CREEP_SPEED=$spd"
  log="$OUTDIR/speed_$spd.log"
  KIND=sanmingzhi TARGET_ID=product_028 SLOT=D/L1/C1 \
  WORLD="0.70 3.243 0.5484" LABEL="sanm_spd_$spd" \
  NAV_ROUTE="1.92,2.475,$NAV_YAW;0.632,2.475,$NAV_YAW" \
  CREEP_SPEED="$spd" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$log" 2>&1

  T=$(grep -a '遥测目录: ' "$log" | tail -1 | sed 's/.*遥测目录: //')
  if [[ -z "$T" || ! -d "$T" ]]; then
    printf '%-10s %10s %10s %10s %10s %9s  %s\n' "$spd" n/a - - - - NO_TELEMETRY | tee -a "$OUTDIR/summary.txt"
    continue
  fi
  rel="${T#$ROOT/baseline/}"
  sc=$(docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" "$IMAGE_CLIENT" \
       bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py --dir=/workspace/baseline/$rel --baseline-z=0.5484 --json" 2>/dev/null | tail -1)
  val=$(echo "$sc"  | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['score'])" 2>/dev/null || echo "n/a")
  rise=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['z_rise_m'])" 2>/dev/null || echo "-")
  tilt=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_tilt_deg'])" 2>/dev/null || echo "-")
  shf=$(echo "$sc"  | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_xy_shift_m'])" 2>/dev/null || echo "-")
  gr=$(echo "$sc"   | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['gripped_ratio']*100,1))" 2>/dev/null || echo "-")
  rs=$(echo "$sc"   | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['reason'])" 2>/dev/null || echo "?")
  printf '%-10s %10s %10s %10s %10s %9s  %s\n' "$spd" "$val" "$rise" "$tilt" "$shf" "$gr" "$rs" | tee -a "$OUTDIR/summary.txt"
  if [[ "$val" != "n/a" ]] && python3 -c "import sys;sys.exit(0 if float('$val')>float('$BEST') else 1)" 2>/dev/null; then
    BEST="$val"; BESTV="$spd"
  fi
done

{
  echo
  echo "=================================================================="
  echo "最优 CREEP_SPEED = $BESTV   (score=$BEST)"
  echo "=================================================================="
} | tee -a "$OUTDIR/summary.txt"
