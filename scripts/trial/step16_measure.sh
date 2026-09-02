#!/usr/bin/env bash
# Step 16 - full regression / limit map. Sweep gap width x feature mode (+ a
# speed sub-sweep) and classify each run pass / slow / stop / fail. Crawl gait,
# edge_clearance stays 0 (the multi-step planner is the feature under test).
#
# feature modes:
#   off    - all multistep params false               (pre-Step-12 behaviour)
#   shadow - enabled=true                              (search + CSV, no control)
#   stop   - enabled + apply_stop_request              (Step 14)
#   apply  - enabled + apply_stop_request + apply_foothold  (Step 15)
set -u
cd /home/takuya/work/mpc_dog
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
OUT=artifacts/step16
mkdir -p "$OUT"
SCR=/tmp/claude-1000/-home-takuya-work-mpc-dog/f8678bee-2e6d-4ffb-b961-f3221713cee3/scratchpad
cp "$YAML" "$SCR/yaml.step16.bak"
restore() { cp "$SCR/yaml.step16.bak" "$YAML"; }
trap restore EXIT
CSV="$OUT/step16_runs.csv"
echo "world,gap_cm,mode,speed,iter,final_x,final_z,final_roll,min_z,mstop,slow,applied,verdict" > "$CSV"

run() { # world gap_cm mode speed spawn dur iter
  local W=$1 G=$2 M=$3 V=$4 SP=$5 D=$6 IT=$7
  local TAG="g${G}_${M}_v$(printf '%03d' "$(python3 -c "print(int(${V}*100))")")_$IT"
  restore
  if [ "$M" != "off" ]; then
    sed -i 's/^\(        enabled: \)false\b/\1true/' "$YAML"
  fi
  if [ "$M" = "stop" ] || [ "$M" = "apply" ]; then
    sed -i 's/^\(        apply_stop_request: \)false\b/\1true/' "$YAML"
  fi
  if [ "$M" = "apply" ]; then
    sed -i 's/^\(        apply_foothold: \)false\b/\1true/' "$YAML"
  fi
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -d "$SRC/models/${W}" ] && { [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"; }
  local d="$OUT/$TAG"; rm -rf "$d"; mkdir -p "$d"
  local lt="quadsdk_step16_${TAG}"; rm -rf "artifacts/logs/quadsdk_${lt}"
  local DUMPENV=()
  [ "$M" != "off" ] && DUMPENV=(MPCDOG_STEPDUMP_DIR="$PWD/$d")
  env "${DUMPENV[@]}" \
    SPAWN_X_M=$SP GAP_WORLD="${W}.xml" GAP_TAG="$lt" FORWARD_VEL_MPS=$V DURATION_S=$D \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$d/run.log" 2>&1
  pkill -9 -f "mujoco_go2|ros2_control_node|local_planner_node|nmpc_controller|mjcf_to_grid_map_node|mujoco_estimator|mujoco_recorder|robot_driver_node|controller_manager" 2>/dev/null || true
  sleep 3
  local C="artifacts/logs/quadsdk_${lt}/state_log.csv"
  [ -f "$C" ] && cp "$C" "$d/state_log.csv"
  read FX FZ FR < <(tail -1 "$C" 2>/dev/null | awk -F, '{print $3,$5,$6}')
  local MZ=$(awk -F',' 'NR>1 && $2>12{if($5<m||m==""){m=$5}}END{printf "%.3f",m}' "$C")
  local MSTOP=$(grep -c "multistep-stop] latching" "$d/run.log" 2>/dev/null || echo 0)
  local SLOW=$(grep -c "multistep-stop] SLOW" "$d/run.log" 2>/dev/null || echo 0)
  local NAPP=$(awk -F',' 'NR>1 && $5==1{n++}END{print n+0}' "$d/step15_footholds.csv" 2>/dev/null || echo 0)
  local VDT
  if python3 -c "exit(0 if (abs(${FR:-0})>0.8 or ${FZ:-0}<0.15 or ${MZ:-0}<0.15) else 1)"; then VDT=FAIL
  elif python3 -c "exit(0 if ${FX:-0}>4.0 else 1)"; then VDT=PASS
  elif [ "${MSTOP:-0}" -gt 0 ]; then VDT=STOP
  elif [ "${SLOW:-0}" -gt 0 ]; then VDT=SLOW
  else VDT=STALL; fi
  printf "%s,%s,%s,%s,%s,%.2f,%.2f,%.2f,%s,%s,%s,%s,%s\n" \
    "$W" "$G" "$M" "$V" "$IT" "${FX:-0}" "${FZ:-0}" "${FR:-0}" "${MZ:-0}" \
    "${MSTOP:-0}" "${SLOW:-0}" "${NAPP:-0}" "$VDT" >> "$CSV"
  printf "RUN %-26s | x=%6.2f roll=%5.2f minz=%s | mstop=%s slow=%s app=%s | %s\n" \
    "$TAG" "${FX:-0}" "${FR:-0}" "${MZ:-0}" "${MSTOP:-0}" "${SLOW:-0}" "${NAPP:-0}" "$VDT"
  restore
}

# ---- core: gap width x feature mode, v=0.30, crawl ----------------------
# off / stop / apply x3 each; shadow x1 (no control effect -> outcome == off,
# the run only confirms the search runs and writes its CSV).
for GW in 15:flat_trench_s09_15 25:flat_trench_s09_25 30:flat_trench_s09_30 \
          35:flat_trench_s09_35 50:flat_trench_s09_50 100:flat_trench_s09_100; do
  G=${GW%%:*}; W=${GW##*:}
  for IT in 1 2 3; do
    for M in off stop apply; do
      run "$W" "$G" "$M" 0.30 -2.0 26 "$IT"
    done
  done
  run "$W" "$G" shadow 0.30 -2.0 26 1
done

# ---- speed sub-sweep: gap 30 / 50, modes stop / apply, v=0.50 -----------
# (v=0.15 excluded: Step 13 showed the crawl does not sustain forward walking
#  at 0.15 m/s in this config.)
for GW in 30:flat_trench_s09_30 50:flat_trench_s09_50; do
  G=${GW%%:*}; W=${GW##*:}
  for IT in 1 2 3; do
    for M in stop apply; do
      run "$W" "$G" "$M" 0.50 -2.0 22 "$IT"
    done
  done
done

echo "STEP16 MEASURE DONE -> $OUT ($(wc -l < "$CSV") rows)"
