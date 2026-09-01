#!/usr/bin/env bash
# Re-run the 3 ambiguous cases from retune_verify (2x each) to separate a
# flaky NMPC-startup failure (robot never leaves x=-0.04) from a real fall.
# max_crossable_gap is already 0.54 in the yaml.
set -u
cd /home/takuya/work/mpc_dog
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SCR=/tmp/claude-1000/-home-takuya-work-mpc-dog/f8678bee-2e6d-4ffb-b961-f3221713cee3/scratchpad
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
YAML_BAK="$SCR/yaml.rerun.bak"; cp "$YAML" "$YAML_BAK"
restore(){ cp "$YAML_BAK" "$YAML"; }
trap restore EXIT
link(){ local W=$1
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"; }
run(){ local W=$1 LB=$2 DUR=$3 TAG=$4
  restore; sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"
  link "$W"
  local LOG="$SCR/rr_${TAG}.log"
  GAP_WORLD="${W}.xml" GAP_TAG="quadsdk_rr_${TAG}" FORWARD_VEL_MPS=0.3 DURATION_S=$DUR \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$LOG" 2>&1
  local CSV="artifacts/logs/quadsdk_quadsdk_rr_${TAG}/state_log.csv"
  local LATCH=$(grep -c "latching graceful stop" "$LOG")
  local NMPCFAIL=$(grep -c "NMPC solving fail" "$LOG")
  read FX FZ FR < <(tail -1 "$CSV" | awk -F, '{print $3,$5,$6}')
  local MAXX=$(awk -F',' 'NR>1{if($3>m||m==""){m=$3}}END{printf "%.2f",m}' "$CSV")
  local MINZ=$(awk -F',' 'NR>1 && $2>12{if($5<m||m==""){m=$5}}END{printf "%.3f",m}' "$CSV")
  local V
  if python3 -c "exit(0 if $MAXX<0.3 else 1)"; then V="NO-START(flaky)"
  elif python3 -c "exit(0 if (abs($FR)>0.8 or $FZ<0.15 or $MINZ<0.15) else 1)"; then V=FELL
  elif python3 -c "exit(0 if $FX>4.0 else 1)"; then V=CROSSED
  else V="SAFE-STOP"; fi
  printf "%-22s | latch=%-2s nmpcfail=%-4s | maxx=%5.2f finalx=%6.2f minz=%s roll=%5.2f | %s\n" \
    "$LB" "$LATCH" "$NMPCFAIL" "$MAXX" "$FX" "$MINZ" "$FR" "$V"
  restore
}
for i in 1 2; do
  run "flat_repgap_s25g35n2"        "r25/gap35 N=2 #$i"    35 r25g35_$i
  run "flat_repgap_s25g50n2"        "r25/gap50 N=2 #$i"    35 r25g50_$i
  run "flat_repgap_s15g15n3_last50" "15/15 x2 -> 50cm #$i" 40 last50_$i
done
echo "RERUN DONE"
