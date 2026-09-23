#!/usr/bin/env bash
# One baseline trial for each stair shape in the Step 24 matrix.
# The 15 cm, 30 cm, 4-step world is the existing Step 23 baseline and is not repeated here.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OVERLAY_SETUP="${QUADSDK_OVERLAY_SETUP:-}"
if [ -z "${OVERLAY_SETUP}" ] || [ ! -f "${OVERLAY_SETUP}" ]; then
  echo "QUADSDK_OVERLAY_SETUP must name install/setup.bash" >&2
  exit 2
fi

SUMMARY="${REPO_ROOT}/artifacts/logs/step24_baseline_shape_summary.csv"
if [ ! -f "${SUMMARY}" ]; then
  echo "order,world,height_m,tread_m,steps,angle_deg,duration_s,approach_x,ascent_x,top_x,descent_x,lower_x,success_x,pre_max_x,end_x,end_y,end_z,end_roll,end_pitch,nmpc_fail,verdict,log_dir,gif" > "${SUMMARY}"
fi

# order world height tread steps
specs=(
  "01 jp_stair_matrix_h10_d39_n04 0.10 0.39 4"
  "02 jp_stair_matrix_h10_d30_n04 0.10 0.30 4"
  "03 jp_stair_matrix_h12p5_d39_n04 0.125 0.39 4"
  "04 jp_stair_matrix_h12p5_d30_n04 0.125 0.30 4"
  "05 jp_stair_matrix_h15_d60_n04 0.15 0.60 4"
  "06 jp_stair_matrix_h15_d39_n04 0.15 0.39 4"
  "07 jp_stair_matrix_h15_d30_n01 0.15 0.30 1"
  "08 jp_stair_matrix_h15_d30_n02 0.15 0.30 2"
  "10 jp_stair_matrix_h15_d30_n06 0.15 0.30 6"
  "11 jp_stair_matrix_h15_d39_n06 0.15 0.39 6"
)

for spec in "${specs[@]}"; do
  read -r order world height tread steps <<<"${spec}"
  read -r lookat camera duration approach_x ascent_x top_x descent_x lower_x success_x <<<"$(python3 - "${height}" "${tread}" "${steps}" <<'PY'
import math
import sys
height = float(sys.argv[1])
tread = float(sys.argv[2])
steps = int(sys.argv[3])
top_run = 1.0
exit_run = 3.0
approach_x = 3.0
ascent_x = 3.0 + (steps - 1) * tread
top_x = ascent_x + top_run
descent_x = top_x + (steps - 1) * tread
lower_x = descent_x + 0.70
success_x = descent_x + 1.50
end_x = descent_x + exit_run
lookat = (1.0 + end_x) / 2.0
camera = max(4.5, (end_x - 0.2) / 1.35)
distance = success_x - 1.0
duration = max(70, math.ceil(distance / 0.10 * 1.6))
print(
    f"{lookat:.3f} {camera:.3f} {duration} "
    f"{approach_x:.3f} {ascent_x:.3f} {top_x:.3f} {descent_x:.3f} "
    f"{lower_x:.3f} {success_x:.3f}"
)
PY
)"
  tag="step24_${world}"
  log_dir="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}"
  run_log="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}_run.log"
  echo "SHAPE START order=${order} world=${world} duration=${duration}s success_x=${success_x}"
  pkill -9 -f "local_planner_node|nmpc_controller|ros2_control_node|mujoco_recorder|grid_map_filters_demo|mjcf_to_grid_map_node" 2>/dev/null || true
  sleep 2
  if ! pgrep -f "local_planner_node|ros2_control_node" >/dev/null; then
    rm -f /dev/shm/fastrtps_* || true
  fi
  sleep 1
  QUADSDK_OVERLAY_SETUP="${OVERLAY_SETUP}" \
    PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.10 \
    GAP_WORLD="${world}.xml" GAP_TAG="${tag}" DURATION_S="${duration}" HOLD_S=5 \
    CAMERA_DISTANCE_M="${camera}" CAMERA_LOOKAT_X_M="${lookat}" \
    bash "${REPO_ROOT}/scripts/trial/run_quadsdk_jp_stair.sh" > "${run_log}" 2>&1

  mp4="$(ls -t "${log_dir}"/logs/mujoco_go2_*.mp4 | head -1)"
  gif="${log_dir}/trial.gif"
  video_duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${mp4}")"
  gif_length="$(python3 - "${video_duration}" <<'PY'
import sys
print(f"{max(1.0, float(sys.argv[1]) - 8.0):.3f}")
PY
)"
  bash "${REPO_ROOT}/scripts/trial/make_gif.sh" \
    "${mp4}" "${gif}" 3 480 8 "${gif_length}"

  python3 - "${SUMMARY}" "${log_dir}/state_log.csv" "${order}" "${world}" \
    "${height}" "${tread}" "${steps}" "${duration}" \
    "${approach_x}" "${ascent_x}" "${top_x}" "${descent_x}" "${lower_x}" "${success_x}" \
    "${log_dir}" "${gif}" "${run_log}" <<'PY'
import csv
import math
import sys

(
    summary, state, order, world, height, tread, steps, duration,
    approach_x, ascent_x, top_x, descent_x, lower_x, success_x,
    log_dir, gif, run_log,
) = sys.argv[1:]
rows = list(csv.DictReader(open(state)))

def finite(row, key):
    value = float(row[key])
    return value if math.isfinite(value) else float("nan")

pre_rows = []
upright = False
for row in rows:
    t = finite(row, "sim_time_s")
    z = finite(row, "base_pos_z_m")
    roll = finite(row, "base_roll_rad")
    pitch = finite(row, "base_pitch_rad")
    pre_rows.append(row)
    if (not math.isnan(t) and t > 8.0 and z > 0.35) or (math.isnan(t) and z > 0.35):
        upright = True
    if upright and (z < 0.22 or abs(roll) > 0.80 or abs(pitch) > 0.80):
        break
pre_max_x = max(finite(row, "base_pos_x_m") for row in pre_rows)
last = rows[-1]
end_x = finite(last, "base_pos_x_m")
end_y = finite(last, "base_pos_y_m")
end_z = finite(last, "base_pos_z_m")
end_roll = finite(last, "base_roll_rad")
end_pitch = finite(last, "base_pitch_rad")
success = (
    pre_max_x >= float(success_x)
    and end_z > 0.20
    and abs(end_y) < 0.50
    and abs(end_roll) < 0.50
    and abs(end_pitch) < 0.50
)
nmpc_fail = sum("NMPC solving fail" in line for line in open(run_log, errors="replace"))
angle = math.degrees(math.atan(float(height) / float(tread)))
with open(summary, "a", newline="") as stream:
    csv.writer(stream).writerow([
        order, world, height, tread, steps, f"{angle:.1f}", duration,
        approach_x, ascent_x, top_x, descent_x, lower_x, success_x,
        f"{pre_max_x:.3f}", f"{end_x:.3f}", f"{end_y:.3f}", f"{end_z:.3f}",
        f"{end_roll:.3f}", f"{end_pitch:.3f}", nmpc_fail,
        "SUCCESS" if success else "FAIL", log_dir, gif,
    ])
print(
    f"SHAPE RESULT order={order} pre_max_x={pre_max_x:.3f} "
    f"success_x={success_x} end_y={end_y:.3f} end_z={end_z:.3f} "
    f"verdict={'SUCCESS' if success else 'FAIL'}"
)
PY
done
