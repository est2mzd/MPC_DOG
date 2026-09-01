#!/usr/bin/env bash
# Step 13 - stopping-distance calibration. On a wide trench with edge_clearance
# enabled, the Phase 2B graceful stop latches ~1.9 m before the void and zeros
# cmd_vel. Walk in at v = 0.15 / 0.30 / 0.50 m/s (spawn pulled back so the robot
# reaches speed) and log through the deceleration. state_log has
# base_lin_vel_x_mps + base_pos_x_m; run.log has the "[safe-stop] latching" line.
set -u
cd /home/takuya/work/mpc_dog
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
OUT=artifacts/step13
mkdir -p "$OUT"
SCR=/tmp/claude-1000/-home-takuya-work-mpc-dog/f8678bee-2e6d-4ffb-b961-f3221713cee3/scratchpad
cp "$YAML" "$SCR/yaml.step13.bak"
restore() { cp "$SCR/yaml.step13.bak" "$YAML"; }
trap restore EXIT

W=flat_trench_1m
[ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
[ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"

restore
sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"

run() { # v spawn dur tag
  local V=$1 SP=$2 DUR=$3 TAG=$4
  local d="$OUT/$TAG"; rm -rf "$d"; mkdir -p "$d"
  local lt="quadsdk_step13_${TAG}"; rm -rf "artifacts/logs/quadsdk_${lt}"
  SPAWN_X_M=$SP GAP_WORLD="${W}.xml" GAP_TAG="$lt" FORWARD_VEL_MPS=$V DURATION_S=$DUR \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$d/run.log" 2>&1
  pkill -9 -f "mujoco_go2|ros2_control_node|local_planner_node|nmpc_controller|mjcf_to_grid_map_node|mujoco_estimator|mujoco_recorder|robot_driver_node|controller_manager" 2>/dev/null || true
  sleep 3
  local C="artifacts/logs/quadsdk_${lt}/state_log.csv"
  [ -f "$C" ] && cp "$C" "$d/state_log.csv"
  grep -n "latching graceful stop" "$d/run.log" | head -1 > "$d/latch.txt"
  local fx=$(tail -1 "$C" 2>/dev/null | awk -F, '{printf "%.2f",$3}')
  echo "  v=$V spawn=$SP final x=$fx  latch:$(cat "$d/latch.txt" | cut -c1-40)"
}

run 0.15 -2.0 40 v015
run 0.30 -2.0 32 v030
run 0.50 -2.0 26 v050
restore
echo "STEP13 MEASURE DONE -> $OUT"
