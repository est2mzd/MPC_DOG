#!/usr/bin/env bash
# Re-verify the Step 08 subset after max_crossable_gap 0.6 -> 0.54.
# Expect: <=0.30 m gaps still cross; the two 0.50 m FELL cases now SAFE-STOP;
# 100/1000 cm unchanged.
set -u
cd /home/takuya/work/mpc_dog
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SCR=/tmp/claude-1000/-home-takuya-work-mpc-dog/f8678bee-2e6d-4ffb-b961-f3221713cee3/scratchpad
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
YAML_BAK="$SCR/yaml.retune.bak"; cp "$YAML" "$YAML_BAK"
restore() { cp "$YAML_BAK" "$YAML"; }
trap restore EXIT

link() { local W=$1
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"
}
gen_rep() { local S=$1 G=$2 N=$3 LAST=${4:-0}
  local T
  if [ "$LAST" != "0" ]; then T="s$(printf %.0f $(echo "$S*100"|bc))g$(printf %.0f $(echo "$G*100"|bc))n${N}_last$(printf %.0f $(echo "$LAST*100"|bc))"
  else T="s$(printf %.0f $(echo "$S*100"|bc))g$(printf %.0f $(echo "$G*100"|bc))n${N}"; fi
  python3 src/trial/assets/gen_quadsdk_repeated_gap_world.py "$S" "$G" "$N" 2.0 1.0 "$T" 0.05 "$LAST" >/dev/null
  echo "flat_repgap_${T}"
}
run() { # world label ec dur tag
  local W=$1 LB=$2 EC=$3 DUR=$4 TAG=$5
  restore
  [ "$EC" = "1" ] && sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"
  link "$W"
  local LOG="$SCR/rt_${TAG}.log"
  GAP_WORLD="${W}.xml" GAP_TAG="quadsdk_rt_${TAG}" FORWARD_VEL_MPS=0.3 DURATION_S=$DUR \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$LOG" 2>&1
  local CSV="artifacts/logs/quadsdk_quadsdk_rt_${TAG}/state_log.csv"
  local LATCH=$(grep -c "latching graceful stop" "$LOG")
  read FX FZ FR < <(tail -1 "$CSV" | awk -F, '{print $3,$5,$6}')
  local MINZ=$(awk -F',' 'NR>1 && $2>12{if($5<m||m==""){m=$5}}END{printf "%.3f",m}' "$CSV")
  local V
  if python3 -c "exit(0 if (abs($FR)>0.8 or $FZ<0.15 or $MINZ<0.15) else 1)"; then V=FELL
  elif python3 -c "exit(0 if $FX>4.0 else 1)"; then V=CROSSED
  else V="SAFE-STOP"; fi
  printf "%-24s ec=%s | latch=%-2s | x=%7.2f minz=%s roll=%5.2f | %s\n" "$LB" "$EC" "$LATCH" "$FX" "$MINZ" "$FR" "$V"
  restore
}

echo "### must still cross (<=0.30 m)"
run flat_gaps_2m    "step03 0.3m ec0"     0 45 s03_ec0
run flat_gaps_2m    "step03 0.3m ec0.15"  1 45 s03_ec1
run flat_gaps_1p5m  "step04 0.3m ec0.15"  1 45 s04_ec1
run "$(gen_rep 0.15 0.15 2)"  "rep 15/15 N=2"        1 35 r1515n2
run "$(gen_rep 0.25 0.25 2)"  "rep 25/gap25 N=2"     1 35 r25g25
run "$(gen_rep 0.25 0.35 2)"  "rep 25/gap35 N=2"     1 35 r25g35
echo "### the two FELL cases -> expect SAFE-STOP now"
run "$(gen_rep 0.25 0.50 2)"      "rep 25/gap50 N=2"     1 35 r25g50
run "$(gen_rep 0.15 0.15 3 0.5)"  "15/15 x2 -> 50cm"     1 35 last50
echo "### wide trenches -> unchanged"
run flat_trench_1m   "single 100cm"   1 30 t100
run flat_trench_10m  "single 1000cm"  1 30 t1000
run "$(gen_rep 0.15 0.15 3 1.0)"  "15/15 x2 -> 100cm"    1 35 last100
echo "RETUNE VERIFY DONE"
