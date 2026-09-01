#!/usr/bin/env bash
# Step 11 (shadow, no control change): per future touchdown, enumerate the map
# cells that are reachable + safe + observed around the leg's hip, and record
# whether the selected foothold passes the same tests.
set -u
cd /home/takuya/work/mpc_dog
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
OUT=artifacts/step11
mkdir -p "$OUT"

run() { # world tag dur
  local W=$1 TAG=$2 DUR=$3
  echo "===== $TAG ($W) ====="
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -d "$SRC/models/${W}" ] && { [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"; }
  local d="$OUT/$TAG"; rm -rf "$d"; mkdir -p "$d"
  local lt="quadsdk_step11_${TAG}"; rm -rf "artifacts/logs/quadsdk_${lt}"
  MPCDOG_STEPDUMP_DIR="$PWD/$d" \
    GAP_WORLD="${W}.xml" GAP_TAG="$lt" FORWARD_VEL_MPS=0.3 DURATION_S=$DUR \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$d/run.log" 2>&1
  local C="artifacts/logs/quadsdk_${lt}/state_log.csv"
  [ -f "$C" ] && cp "$C" "$d/state_log.csv"
  local n=$(wc -l < "$d/step11_candidates.csv" 2>/dev/null || echo 0)
  local fx=$(tail -1 "$C" 2>/dev/null | awk -F, '{printf "%.2f",$3}')
  echo "  final x=$fx  step11_candidates lines=$n"
}

run flat_wide            flat  25
run flat_gaps_2m         g30   30
run flat_trench_s09_50   g50   25
run flat_trench_s09_100  g100  25
echo "STEP11 MEASURE DONE -> $OUT"
