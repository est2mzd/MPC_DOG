#!/usr/bin/env bash
# Step 09: terrain-grid + foothold measurement (NO control change).
# Runs single trenches of 15/25/30/35/50/100 cm with MPCDOG_STEP09_DIR set so
# local_footstep_planner dumps step09_map_cross_section.csv + step09_footholds.csv.
# The local_planner.yaml is NOT touched: this runs the repo exactly as it sits.
set -u
cd /home/takuya/work/mpc_dog
# The repo .venv is Python 3.11 but ROS Jazzy's rclpy needs the system 3.12,
# so the CSV state-logger crashes if the venv shadows python3. Strip it.
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
OUT=artifacts/step09
mkdir -p "$OUT"

# widths in cm -> metres
WIDTHS_CM="15 25 30 35 50 100"

for wcm in $WIDTHS_CM; do
  w=$(python3 -c "print($wcm/100)")
  tag="s09_${wcm}"
  world="flat_trench_${tag}"
  echo "===== ${wcm} cm trench ($world) ====="
  python3 src/trial/assets/gen_quadsdk_wide_trench_world.py "$w" 2.0 1.0 "$tag" 0.05 >/dev/null
  [ -e "$INST/worlds/${world}.xml.xacro" ] || ln -sfn "$PWD/$SRC/worlds/${world}.xml.xacro" "$INST/worlds/${world}.xml.xacro"
  [ -e "$INST/models/${world}" ]           || ln -sfn "$PWD/$SRC/models/${world}"           "$INST/models/${world}"

  dumpdir="$OUT/${tag}"
  rm -rf "$dumpdir"; mkdir -p "$dumpdir"
  logtag="quadsdk_step09_${tag}"
  rm -rf "artifacts/logs/quadsdk_${logtag}"

  MPCDOG_STEP09_DIR="$PWD/$dumpdir" \
    GAP_WORLD="${world}.xml" GAP_TAG="$logtag" FORWARD_VEL_MPS=0.3 DURATION_S=30 \
    bash scripts/trial/run_quadsdk_gap_1m.sh > "$dumpdir/run.log" 2>&1

  CSV="artifacts/logs/quadsdk_${logtag}/state_log.csv"
  MP4=$(ls -t artifacts/logs/quadsdk_${logtag}/logs/*.mp4 2>/dev/null | head -1)
  [ -f "$CSV" ] && cp "$CSV" "$dumpdir/state_log.csv"
  if [ -n "${MP4:-}" ] && [ -f "$MP4" ]; then
    ffmpeg -y -ss 10 -to 30 -i "$MP4" "$dumpdir/trim.mp4" -loglevel error
    bash scripts/trial/make_gif.sh "$dumpdir/trim.mp4" "$dumpdir/step09_${tag}.gif" 10 480 >/dev/null
  fi

  fx=$(tail -1 "$CSV" 2>/dev/null | awk -F, '{printf "%.2f",$3}')
  nrows_map=$(wc -l < "$dumpdir/step09_map_cross_section.csv" 2>/dev/null || echo 0)
  nrows_ft=$(wc -l < "$dumpdir/step09_footholds.csv" 2>/dev/null || echo 0)
  echo "  final x=$fx  map_csv_lines=$nrows_map  footholds_csv_lines=$nrows_ft"
done
echo "STEP09 MEASURE DONE -> $OUT"
