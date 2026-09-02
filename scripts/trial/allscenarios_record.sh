#!/usr/bin/env bash
# 全シナリオ動作確認の録画。制御パラメータは1セットに固定(stop-only:
# multistep_planner.enabled=true + apply_stop_request=true, apply_foothold=false,
# edge_clearance=0, クロール)。world だけを変えて、これまで扱った地形を一通り
# 走らせて mp4 を残す。GIF 化は scripts/trial/allscenarios_gif.sh。
#
# 制御パラメータは触らない。yaml の multistep_planner を stop-only へ sed し、
# 正常終了・異常終了とも cp バックアップから復元する。
set -u
cd /home/takuya/work/mpc_dog
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
BAK="$(mktemp)"
cp "$YAML" "$BAK"
trap 'cp "$BAK" "$YAML"; rm -f "$BAK"' EXIT
sed -i 's/^\(        enabled: \)false\b/\1true/' "$YAML"
sed -i 's/^\(        apply_stop_request: \)false\b/\1true/' "$YAML"

one() { # world spawn dur vel tag
  local W=$1 SP=$2 D=$3 V=$4 TAG=$5
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -d "$SRC/models/${W}" ] && { [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"; }
  local lt="quadsdk_allrec_${TAG}"
  rm -rf "artifacts/logs/quadsdk_${lt}"
  SPAWN_X_M=$SP GAP_WORLD="${W}.xml" GAP_TAG="$lt" FORWARD_VEL_MPS=$V DURATION_S=$D \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "artifacts/logs/quadsdk_${lt}_run.log" 2>&1
  sleep 2
  local m; m=$(ls "artifacts/logs/quadsdk_${lt}/logs/"*.mp4 2>/dev/null | head -1)
  local dur; dur=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$m" 2>/dev/null)
  local C="artifacts/logs/quadsdk_${lt}/state_log.csv"
  local fx; fx=$(tail -1 "$C" 2>/dev/null | awk -F, '{printf "%.2f",$3}')
  local ms; ms=$(grep -c "multistep-stop] latching" "artifacts/logs/quadsdk_${lt}_run.log" 2>/dev/null); ms=${ms:-0}
  echo "REC ${TAG} world=${W} v=${V} -> x=${fx} mstop=${ms} mp4_dur=${dur:-BAD}"
}

run() { # world spawn dur vel tag  (mp4 が壊れていたら1回だけ再試行)
  one "$@"
  local TAG=$5
  local m; m=$(ls "artifacts/logs/quadsdk_allrec_${TAG}/logs/"*.mp4 2>/dev/null | head -1)
  ffprobe -v error -show_entries format=duration -of csv=p=0 "$m" >/dev/null 2>&1 || { echo "  -> mp4 BAD, retry once"; one "$@"; }
}

run flat_wide                     0.0  22 0.30 flat
run flat_gaps_2m                  0.0  34 0.30 gaps2m
run flat_repgap_s15g15n5          0.0  34 0.30 repgap_n5
run flat_trench_s09_15           -2.0  28 0.30 trench15
run flat_trench_s09_25           -2.0  28 0.30 trench25
run flat_trench_s09_30           -2.0  28 0.30 trench30
run flat_trench_s09_35           -2.0  28 0.30 trench35
run flat_trench_s09_50           -2.0  26 0.30 trench50
run flat_trench_s09_100          -2.0  26 0.30 trench100
run flat_repgap_s15g15n3_last100  0.0  36 0.30 composite
run flat_trench_s09_30           -2.0  24 0.50 trench30_v050
echo "ALLSC RECORD DONE"
