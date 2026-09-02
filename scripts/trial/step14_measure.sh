#!/usr/bin/env bash
# Step 14 - connect the multi-step foothold-sequence verdict to the graceful
# stop (opt-in). ON = multistep_planner.enabled:=true + apply_stop_request:=true
# (edge_clearance stays 0 so this isolates the multi-step planner).
set -u
cd /home/takuya/work/mpc_dog
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
OUT=artifacts/step14
mkdir -p "$OUT"
SCR=/tmp/claude-1000/-home-takuya-work-mpc-dog/f8678bee-2e6d-4ffb-b961-f3221713cee3/scratchpad
cp "$YAML" "$SCR/yaml.step14.bak"
restore() { cp "$SCR/yaml.step14.bak" "$YAML"; }
trap restore EXIT

run() { # world mode spawn dur tag   (mode: on|off)
  local W=$1 M=$2 SP=$3 D=$4 TAG=$5
  restore
  if [ "$M" = "on" ]; then
    sed -i 's/^\(        enabled: \)false\b/\1true/' "$YAML"
    sed -i 's/^\(        apply_stop_request: \)false\b/\1true/' "$YAML"
  fi
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -d "$SRC/models/${W}" ] && { [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"; }
  local d="$OUT/$TAG"; rm -rf "$d"; mkdir -p "$d"
  local lt="quadsdk_step14_${TAG}"; rm -rf "artifacts/logs/quadsdk_${lt}"
  SPAWN_X_M=$SP GAP_WORLD="${W}.xml" GAP_TAG="$lt" FORWARD_VEL_MPS=0.3 DURATION_S=$D \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$d/run.log" 2>&1
  pkill -9 -f "mujoco_go2|ros2_control_node|local_planner_node|nmpc_controller|mjcf_to_grid_map_node|mujoco_estimator|mujoco_recorder|robot_driver_node|controller_manager" 2>/dev/null || true
  sleep 3
  local C="artifacts/logs/quadsdk_${lt}/state_log.csv"
  [ -f "$C" ] && cp "$C" "$d/state_log.csv"
  read FX FZ FR < <(tail -1 "$C" 2>/dev/null | awk -F, '{print $3,$5,$6}')
  local MZ=$(awk -F',' 'NR>1 && $2>12{if($5<m||m==""){m=$5}}END{printf "%.3f",m}' "$C")
  local MSTOP=$(grep -c "multistep-stop] latching" "$d/run.log")
  local SLOW=$(grep -c "multistep-stop] SLOW" "$d/run.log")
  local V
  if python3 -c "exit(0 if (abs(${FR:-0})>0.8 or ${FZ:-0}<0.15 or ${MZ:-0}<0.15) else 1)"; then V=FELL
  elif python3 -c "exit(0 if ${FX:-0}>4.0 else 1)"; then V=CROSSED
  else V="SAFE-STOP"; fi
  printf "RESULT %-22s %-3s | x=%6.2f z=%.2f roll=%5.2f minz=%s | mstop=%s slow=%s | %s\n" \
    "$TAG" "$M" "${FX:-0}" "${FZ:-0}" "${FR:-0}" "${MZ:-0}" "$MSTOP" "$SLOW" "$V"
  restore
}

# multi-step planner ON (opt-in): wide voids must stop upright; small gaps must pass
for i in 1 2 3; do
  run flat_trench_s09_50  on -2.0 25 g50_on_$i
  run flat_trench_s09_100 on -2.0 25 g100_on_$i
done
run flat_trench_s09_35   on -2.0 30 g35_on
run flat_gaps_2m         on  0.0 35 g30_on
run flat_repgap_s15g15n3 on  0.0 30 r15_on

# feature OFF (default) -> must match Step 08
run flat_gaps_2m         off 0.0 35 g30_off
run flat_trench_s09_50   off -2.0 25 g50_off
echo "STEP14 MEASURE DONE -> $OUT"
