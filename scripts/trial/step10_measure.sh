#!/usr/bin/env bash
# Step 10 (shadow, no control change): reconstruct the future touchdown-event
# list per leg from the current gait phase and compare with the actual contact
# transitions logged in state_log.csv. Runs flat / 30cm gaps / repeated 15cm.
set -u
cd /home/takuya/work/mpc_dog
# The repo .venv is Python 3.11 but ROS Jazzy's rclpy needs the system 3.12,
# so the CSV state-logger crashes if the venv shadows python3. Strip it.
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
OUT=artifacts/step10
mkdir -p "$OUT"

run() { # world tag dur
  local W=$1 TAG=$2 DUR=$3
  echo "===== $TAG ($W) ====="
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -e "$INST/models/${W}" ] || { [ -d "$SRC/models/${W}" ] && ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"; }
  local d="$OUT/$TAG"; rm -rf "$d"; mkdir -p "$d"
  local lt="quadsdk_step10_${TAG}"; rm -rf "artifacts/logs/quadsdk_${lt}"
  MPCDOG_STEPDUMP_DIR="$PWD/$d" \
    GAP_WORLD="${W}.xml" GAP_TAG="$lt" FORWARD_VEL_MPS=0.3 DURATION_S=$DUR \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$d/run.log" 2>&1
  local C="artifacts/logs/quadsdk_${lt}/state_log.csv"
  [ -f "$C" ] && cp "$C" "$d/state_log.csv"
  local n=$(wc -l < "$d/step10_gait_events.csv" 2>/dev/null || echo 0)
  local fx=$(tail -1 "$C" 2>/dev/null | awk -F, '{printf "%.2f",$3}')
  echo "  final x=$fx  step10_gait_events lines=$n"
}

run flat_wide             flat   25
run flat_gaps_2m          g30    30
run flat_repgap_s15g15n3  r15n3  30
echo "STEP10 MEASURE DONE -> $OUT"
