#!/usr/bin/env bash
# 逐类抓取参数寻优：对指定商品扫描一个几何参数，按裁判真值评分选最优。
#
# 原理：grasp_geometry.py 的 _kind_geometry() 支持以环境变量覆盖任意类别参数
# （SUPERMARKET_<KIND>_<PARAM>）。run_item_grasp.sh 会把环境里的 SUPERMARKET_*
# 透传进容器，因此本脚本只需在调用前 export 对应变量即可。
#
# 评分只用裁判 ground-truth（见 score_grasp_telemetry.py），不看控制器退出码。
# 选出"能稳定夹住且夹得最正"的值，而不是仅仅"能抬起来"。
#
# ⚠️ 必须以「wsl.exe 会话存活」方式启动（Bash 工具 run_in_background=true），
#    否则 vmIdleTimeout 会在 60 秒后关掉整台 WSL 虚拟机。详见 run_item_grasp.sh。
#
# 用法：
#   KIND=pingguo TARGET_ID=product_035 SLOT=D/L3/C2 WORLD="0.92 3.243 1.224" \
#   PARAM=DEPLOY_DZ VALUES="-0.030 -0.010 0.010" bash run_param_sweep.sh
set -uo pipefail

ROOT="${ROOT:-/opt/jbgz}"
IMAGE_CLIENT="${IMAGE_CLIENT:-crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client}"

KIND="${KIND:?需要 KIND}"
TARGET_ID="${TARGET_ID:?需要 TARGET_ID}"
SLOT="${SLOT:?需要 SLOT}"
WORLD="${WORLD:?需要 WORLD，如 \"0.92 3.243 1.224\"}"
PARAM="${PARAM:?需要 PARAM，如 DEPLOY_DZ / CREEP_STOP_DY / GRIP_CLOSE}"
VALUES="${VALUES:?需要 VALUES，空格分隔，如 \"-0.03 -0.01 0.01\"}"
HOLD_SECONDS="${HOLD_SECONDS:-40}"
NAV_X_OFFSET="${NAV_X_OFFSET:-0.068}"
NAV_YAW="1.5707963267948966"

WX=$(echo "$WORLD" | cut -d' ' -f1)
WZ=$(echo "$WORLD" | cut -d' ' -f3)
ENDX=$(python3 -c "print(round($WX - $NAV_X_OFFSET, 4))")
ROUTE="1.92,2.475,$NAV_YAW;${ENDX},2.475,$NAV_YAW"

KIND_UPPER=$(echo "$KIND" | tr '[:lower:]' '[:upper:]')
OVERRIDE_VAR="SUPERMARKET_${KIND_UPPER}_${PARAM}"

OUTDIR="$ROOT/baseline/debug_data/sweep_${KIND}_${PARAM}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

{
  echo "================================================================"
  echo "参数寻优：kind=$KIND  param=$PARAM  values=[$VALUES]"
  echo "货位=$SLOT world=($WORLD)  导航终点x=$ENDX"
  echo "覆盖变量=$OVERRIDE_VAR"
  echo "输出目录=$OUTDIR"
  echo "================================================================"
} | tee "$OUTDIR/summary.txt"

printf '%-12s %10s %10s %10s %10s %9s  %s\n' \
  "$PARAM" SCORE RISE_m TILT_deg SHIFT_m GRIP% REASON | tee -a "$OUTDIR/summary.txt"

BEST_SCORE=-999999
BEST_VAL=""

for val in $VALUES; do
  export "$OVERRIDE_VAR=$val"
  echo "[$(date +%H:%M:%S)] >>> $KIND $PARAM=$val  ($OVERRIDE_VAR=$val)"

  logfile="$OUTDIR/${PARAM}_${val}.log"
  KIND="$KIND" TARGET_ID="$TARGET_ID" SLOT="$SLOT" WORLD="$WORLD" \
  LABEL="${KIND}_${PARAM}_${val}" NAV_ROUTE="$ROUTE" HOLD_SECONDS="$HOLD_SECONDS" \
    bash "$ROOT/baseline/run_item_grasp.sh" > "$logfile" 2>&1

  telem=$(grep -a '遥测目录: ' "$logfile" | tail -1 | sed 's/.*遥测目录: //')
  if [[ -z "$telem" || ! -d "$telem" ]]; then
    echo "[$(date +%H:%M:%S)]     无遥测目录，跳过评分"
    printf '%-12s %10s %10s %10s %10s %9s  %s\n' "$val" "n/a" "-" "-" "-" "-" NO_TELEMETRY \
      | tee -a "$OUTDIR/summary.txt"
    continue
  fi

  rel="${telem#$ROOT/baseline/}"
  sc=$(docker run --rm -v "$ROOT/baseline:/workspace/baseline:ro" "$IMAGE_CLIENT" \
       bash -lc "python3 -u /workspace/baseline/score_grasp_telemetry.py \
                 --dir=/workspace/baseline/$rel --baseline-z=$WZ --json" 2>/dev/null | tail -1)

  val_score=$(echo  "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['score'])" 2>/dev/null || echo "n/a")
  val_rise=$(echo   "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['z_rise_m'])" 2>/dev/null || echo "-")
  val_tilt=$(echo   "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_tilt_deg'])" 2>/dev/null || echo "-")
  val_shift=$(echo  "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['max_xy_shift_m'])" 2>/dev/null || echo "-")
  val_grip=$(echo   "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(round(d['gripped_ratio']*100,1))" 2>/dev/null || echo "-")
  val_reason=$(echo "$sc" | python3 -c "import sys,json;d=json.load(sys.stdin);print(d['reason'])" 2>/dev/null || echo "?")

  printf '%-12s %10s %10s %10s %10s %9s  %s\n' \
    "$val" "$val_score" "$val_rise" "$val_tilt" "$val_shift" "$val_grip" "$val_reason" \
    | tee -a "$OUTDIR/summary.txt"

  if [[ "$val_score" != "n/a" ]] && \
     python3 -c "import sys;sys.exit(0 if float('$val_score')>float('$BEST_SCORE') else 1)" 2>/dev/null; then
    BEST_SCORE="$val_score"
    BEST_VAL="$val"
  fi

  unset "$OVERRIDE_VAR"
done

{
  echo
  echo "================================================================"
  echo "最优取值：$PARAM = $BEST_VAL   (score=$BEST_SCORE)"
  echo "写回 grasp_geometry.py 时请同时置 calibrated=True"
  echo "汇总文件：$OUTDIR/summary.txt"
  echo "================================================================"
} | tee -a "$OUTDIR/summary.txt"
