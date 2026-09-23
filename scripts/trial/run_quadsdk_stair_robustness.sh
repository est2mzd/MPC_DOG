#!/usr/bin/env bash
# Repeat one fixed stair condition without changing planner parameters.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LABEL="${1:?usage: $0 LABEL [REPEATS]}"
REPEATS="${2:-5}"
OVERLAY_SETUP="${QUADSDK_OVERLAY_SETUP:-}"

if [ -z "${OVERLAY_SETUP}" ] || [ ! -f "${OVERLAY_SETUP}" ]; then
  echo "QUADSDK_OVERLAY_SETUP must name install/setup.bash" >&2
  exit 2
fi
if ! [[ "${LABEL}" =~ ^[a-z0-9_]+$ ]]; then
  echo "LABEL must contain only lowercase letters, digits, and underscores" >&2
  exit 2
fi
if ! [[ "${REPEATS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "REPEATS must be a positive integer" >&2
  exit 2
fi

PLANNER_YAML="${OVERLAY_SETUP%/setup.bash}/local_planner/share/local_planner/config/local_planner.yaml"
SUPPORT_MODE="$(awk '/foothold_support_check_mode:/ {print $2; exit}' "${PLANNER_YAML}")"
SUMMARY="${REPO_ROOT}/artifacts/logs/step23_stair_robustness_summary.csv"
if [ ! -f "${SUMMARY}" ]; then
  echo "label,support_mode,trial,max_x,end_x,end_y,end_z,end_roll,end_pitch,nmpc_fail,verdict,log_dir,gif" > "${SUMMARY}"
fi

for trial in $(seq 1 "${REPEATS}"); do
  trial_code="$(printf '%02d' "${trial}")"
  tag="step23_${LABEL}_r${trial_code}"
  log_dir="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}"
  run_log="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}_run.log"

  echo "ROBUSTNESS START label=${LABEL} support=${SUPPORT_MODE} trial=${trial}/${REPEATS}"
  pkill -9 -f "local_planner_node|nmpc_controller|ros2_control_node|mujoco_recorder|grid_map_filters_demo|mjcf_to_grid_map_node" 2>/dev/null || true
  sleep 2
  if ! pgrep -f "local_planner_node|ros2_control_node" >/dev/null; then
    rm -f /dev/shm/fastrtps_* || true
  fi
  sleep 1
  QUADSDK_OVERLAY_SETUP="${OVERLAY_SETUP}" \
    GAP_WORLD="jp_stair_matrix_h15_d30_n04.xml" \
    GAP_TAG="${tag}" DURATION_S=90 HOLD_S=5 \
    CAMERA_DISTANCE_M=6.370 CAMERA_LOOKAT_X_M=4.900 \
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

  python3 - "${SUMMARY}" "${log_dir}/state_log.csv" "${LABEL}" \
    "${SUPPORT_MODE}" "${trial}" "${log_dir}" "${gif}" "${run_log}" <<'PY'
import csv
import math
import sys

summary, state, label, support_mode, trial, log_dir, gif, run_log = sys.argv[1:]
rows = list(csv.DictReader(open(state)))

def finite(row, key):
    value = float(row[key])
    return value if math.isfinite(value) else float("nan")

max_x = max(finite(row, "base_pos_x_m") for row in rows)
last = rows[-1]
end_x = finite(last, "base_pos_x_m")
end_y = finite(last, "base_pos_y_m")
end_z = finite(last, "base_pos_z_m")
end_roll = finite(last, "base_roll_rad")
end_pitch = finite(last, "base_pitch_rad")
nmpc_fail = sum("NMPC solving fail" in line for line in open(run_log, errors="replace"))
success = (
    max_x >= 7.30
    and abs(end_y) < 0.50
    and end_z > 0.20
    and abs(end_roll) < 0.50
    and abs(end_pitch) < 0.50
)
if max_x < 1.50:
    verdict = "STARTUP_FAIL"
elif success:
    verdict = "SUCCESS"
else:
    verdict = "FAIL"
with open(summary, "a", newline="") as stream:
    csv.writer(stream).writerow([
        label, support_mode, trial, f"{max_x:.3f}", f"{end_x:.3f}",
        f"{end_y:.3f}", f"{end_z:.3f}", f"{end_roll:.3f}",
        f"{end_pitch:.3f}", nmpc_fail, verdict, log_dir, gif,
    ])
print(
    f"ROBUSTNESS RESULT label={label} support={support_mode} trial={trial} "
    f"max_x={max_x:.3f} end_y={end_y:.3f} nmpc_fail={nmpc_fail} "
    f"verdict={verdict}"
)
PY
done
