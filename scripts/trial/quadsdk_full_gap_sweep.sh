#!/usr/bin/env bash
# Full regression sweep of every gap terrain against the current code
# (Phase 2A + 3(A) + 2B + 4). Prints one line per run for a README table.
set -u
cd /home/takuya/work/mpc_dog
# The repo .venv is Python 3.11 but ROS Jazzy's rclpy needs the system 3.12,
# so the CSV state-logger crashes if the venv shadows python3. Strip it.
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SCR=/tmp/claude-1000/-home-takuya-work-mpc-dog/f8678bee-2e6d-4ffb-b961-f3221713cee3/scratchpad
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
YAML_BAK="$SCR/local_planner.yaml.fullsweep.bak"
cp "$YAML" "$YAML_BAK"
restore() { cp "$YAML_BAK" "$YAML"; }
trap restore EXIT

link() { local W=$1
  [ -e "$INST/worlds/${W}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${W}.xml.xacro" "$INST/worlds/${W}.xml.xacro"
  [ -e "$INST/models/${W}" ] || ln -sfn "$PWD/$SRC/models/${W}" "$INST/models/${W}"
}
gen_rep() { # strip gap n [last]
  local S=$1 G=$2 N=$3 LAST=${4:-0}
  local T
  if [ "$LAST" != "0" ]; then T="s$(printf %.0f $(echo "$S*100"|bc))g$(printf %.0f $(echo "$G*100"|bc))n${N}_last$(printf %.0f $(echo "$LAST*100"|bc))"
  else T="s$(printf %.0f $(echo "$S*100"|bc))g$(printf %.0f $(echo "$G*100"|bc))n${N}"; fi
  python3 src/trial/assets/gen_quadsdk_repeated_gap_world.py "$S" "$G" "$N" 2.0 1.0 "$T" 0.05 "$LAST" >/dev/null
  echo "flat_repgap_${T}"
}

run() { # world label ec dur tag  ; ec: 0 or 1(=>0.15)
  local W=$1 LB=$2 EC=$3 DUR=$4 TAG=$5
  restore
  [ "$EC" = "1" ] && sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"
  link "$W"
  local LOG="$SCR/fs_${TAG}.log"
  GAP_WORLD="${W}.xml" GAP_TAG="quadsdk_fs_${TAG}" FORWARD_VEL_MPS=0.3 DURATION_S=$DUR \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$LOG" 2>&1
  local CSV="artifacts/logs/quadsdk_quadsdk_fs_${TAG}/state_log.csv"
  local LATCH=$(grep -c "latching graceful stop" "$LOG")
  read FX FZ FR < <(tail -1 "$CSV" | awk -F, '{print $3,$5,$6}')
  local MINZ=$(awk -F',' 'NR>1 && $2>12{if($5<m||m==""){m=$5}}END{printf "%.3f",m}' "$CSV")
  local V
  if python3 -c "exit(0 if (abs($FR)>0.8 or $FZ<0.15 or $MINZ<0.15) else 1)"; then V=FELL
  elif python3 -c "exit(0 if $FX>4.0 else 1)"; then V=CROSSED
  else V="SAFE-STOP"; fi
  printf "%-22s ec=%s | latch=%-3s | x=%7.2f minz=%s roll=%5.2f | %s\n" "$LB" "$EC" "$LATCH" "$FX" "$MINZ" "$FR" "$V"
  restore
}

echo "### step03/04 baseline (edge_clearance:0)"
run flat_gaps_2m    "step03 0.3m sp2.0"   0 45 s03
run flat_gaps_1p5m  "step04 0.3m sp1.5"   0 45 s04

echo "### repeated 15cm/15cm, N varies (edge_clearance:0.15)"
for N in 2 3 4 5 6; do run "$(gen_rep 0.15 0.15 $N)" "15/15 N=$N" 1 35 r15_15_n$N; done

echo "### repeated g15, strip varies, N=2 (edge_clearance:0.15)"
for S in 0.25 0.35 0.50; do run "$(gen_rep $S 0.15 2)" "strip$(printf %.0f $(echo "$S*100"|bc))/15 N=2" 1 35 r${S}_15_n2; done

echo "### repeated strip25, gap varies, N=2 (edge_clearance:0.15)"
for G in 0.25 0.35 0.50; do run "$(gen_rep 0.25 $G 2)" "25/gap$(printf %.0f $(echo "$G*100"|bc)) N=2" 1 35 r25_${G}_n2; done

echo "### single trench, width varies (edge_clearance:0.15)"
run flat_gaps_2m    "single 30cm"    1 45 t30
run flat_trench_1m  "single 100cm"   1 30 t100
run flat_trench_10m "single 1000cm"  1 30 t1000

echo "### mixed: 15/15 x2 then last gap widens (edge_clearance:0.15)"
run "$(gen_rep 0.15 0.15 3 0.5)"  "15/15 x2 -> 50cm"   1 35 last50
run "$(gen_rep 0.15 0.15 3 1.0)"  "15/15 x2 -> 100cm"  1 35 last100
echo "FULL SWEEP DONE"
