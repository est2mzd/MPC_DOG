#!/usr/bin/env bash
# Repeat each failed Step 24 shape until that shape has 5 trials.
# A shape that already succeeded on the first trial is left at 1 trial.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OVERLAY_SETUP="${QUADSDK_OVERLAY_SETUP:-}"
SUMMARY="${REPO_ROOT}/artifacts/logs/step24_baseline_shape_summary.csv"
if [ -z "${OVERLAY_SETUP}" ] || [ ! -f "${OVERLAY_SETUP}" ]; then
  echo "QUADSDK_OVERLAY_SETUP must name install/setup.bash" >&2
  exit 2
fi
if [ ! -f "${SUMMARY}" ]; then
  echo "missing ${SUMMARY}" >&2
  exit 2
fi

mapfile -t failed < <(python3 - "${SUMMARY}" <<'PY'
import csv
import sys
rows = list(csv.DictReader(open(sys.argv[1])))
by_world = {}
for row in rows:
    by_world.setdefault(row["world"], []).append(row)
for world, group in by_world.items():
    if any(row["verdict"] == "SUCCESS" for row in group):
        continue
    if len(group) >= 5:
        continue
    first = group[0]
    start = len(group) + 1
    print(
        f"{first['order']} {world} {first['height_m']} {first['tread_m']} "
        f"{first['steps']} {start}"
    )
PY
)

if [ "${#failed[@]}" -eq 0 ]; then
  echo "SHAPE REPEAT none"
  exit 0
fi

for spec in "${failed[@]}"; do
  read -r order world height tread steps start_trial <<<"${spec}"
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
  for trial in $(seq "${start_trial}" 5); do
    trial_code="$(printf '%02d' "${trial}")"
    tag="step24_${world}_r${trial_code}"
    log_dir="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}"
    run_log="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}_run.log"
    echo "SHAPE REPEAT order=${order} world=${world} trial=${trial}/5"
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
      "${log_dir}" "${gif}" "${run_log}" "${trial}" <<'PY'
import csv
import math
import sys

(
    summary, state, order, world, height, tread, steps, duration,
    approach_x, ascent_x, top_x, descent_x, lower_x, success_x,
    log_dir, gif, run_log, trial,
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
    if t > 8.0 and z > 0.35:
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
        f"{order}r{int(trial):02d}", world, height, tread, steps, f"{angle:.1f}", duration,
        approach_x, ascent_x, top_x, descent_x, lower_x, success_x,
        f"{pre_max_x:.3f}", f"{end_x:.3f}", f"{end_y:.3f}", f"{end_z:.3f}",
        f"{end_roll:.3f}", f"{end_pitch:.3f}", nmpc_fail,
        "SUCCESS" if success else "FAIL", log_dir, gif,
    ])
print(
    f"SHAPE REPEAT RESULT order={order} trial={trial} pre_max_x={pre_max_x:.3f} "
    f"success_x={success_x} end_y={end_y:.3f} verdict={'SUCCESS' if success else 'FAIL'}"
)
PY
    if [ "$(tail -1 "${SUMMARY}" | cut -d, -f21)" = "SUCCESS" ]; then
      echo "SHAPE REPEAT STOP order=${order} trial=${trial} succeeded"
      break
    fi
  done
done
