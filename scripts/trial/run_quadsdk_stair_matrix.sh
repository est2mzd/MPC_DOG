#!/usr/bin/env bash
# Run one height row of the up/down stair matrix with a shared binary and YAML.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HEIGHT_CODE="${1:?usage: $0 HEIGHT_CODE}"
OVERLAY_SETUP="${QUADSDK_OVERLAY_SETUP:-}"
TOP_RUN_M=1.00

case "${HEIGHT_CODE}" in
  15) TREAD_CODE=30; TREAD_M=0.30 ;;
  18) TREAD_CODE=26; TREAD_M=0.26 ;;
  20) TREAD_CODE=24; TREAD_M=0.24 ;;
  *) echo "height code must be 15, 18, or 20" >&2; exit 2 ;;
esac
if [ -z "${OVERLAY_SETUP}" ] || [ ! -f "${OVERLAY_SETUP}" ]; then
  echo "QUADSDK_OVERLAY_SETUP must name install/setup.bash" >&2
  exit 2
fi

SUMMARY="${REPO_ROOT}/artifacts/logs/step22_stair_matrix_summary.csv"
if [ ! -f "${SUMMARY}" ]; then
  echo "height_m,tread_m,steps,duration_s,exit_x,max_x,end_x,end_y,end_z,end_roll,end_pitch,nmpc_fail,verdict,log_dir,gif" > "${SUMMARY}"
fi

for steps in 2 4 6 8 10; do
  case "${steps}" in
    2) duration=70 ;;
    4) duration=90 ;;
    6) duration=110 ;;
    8) duration=130 ;;
    10) duration=150 ;;
  esac
  step_code="$(printf '%02d' "${steps}")"
  world="jp_stair_matrix_h${HEIGHT_CODE}_d${TREAD_CODE}_n${step_code}"
  tag="step22_h${HEIGHT_CODE}_d${TREAD_CODE}_n${step_code}"
  log_dir="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}"
  run_log="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}_run.log"
  read -r lookat camera exit_x <<<"$(python3 - "${steps}" "${TREAD_M}" "${TOP_RUN_M}" <<'PY'
import sys
n = int(sys.argv[1])
tread = float(sys.argv[2])
top_run = float(sys.argv[3])
exit_x = 3.0 + 2 * (n - 1) * tread + top_run
end_x = exit_x + 3.0
lookat = (1.0 + end_x) / 2.0
camera = max(4.5, (end_x - 0.2) / 1.35)
print(f"{lookat:.3f} {camera:.3f} {exit_x:.3f}")
PY
)"

  echo "MATRIX START height=${HEIGHT_CODE}cm tread=${TREAD_CODE}cm steps=${steps} duration=${duration}s"
  QUADSDK_OVERLAY_SETUP="${OVERLAY_SETUP}" \
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
  bash "${REPO_ROOT}/scripts/trial/make_gif.sh" "${mp4}" "${gif}" 3 480 8 "${gif_length}"

  python3 - "${SUMMARY}" "${log_dir}/state_log.csv" "${HEIGHT_CODE}" \
    "${TREAD_M}" "${steps}" "${duration}" "${exit_x}" "${log_dir}" "${gif}" "${run_log}" <<'PY'
import csv
import math
import sys

summary, state, height_code, tread, steps, duration, exit_x, log_dir, gif, run_log = sys.argv[1:]
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
target = float(exit_x) + 1.5
success = (
    max_x >= target
    and end_z > 0.20
    and abs(end_roll) < 0.50
    and abs(end_pitch) < 0.50
)
nmpc_fail = sum("NMPC solving fail" in line for line in open(run_log, errors="replace"))
with open(summary, "a", newline="") as stream:
    csv.writer(stream).writerow([
        f"{int(height_code) / 100:.2f}", f"{float(tread):.2f}",
        steps, duration, exit_x,
        f"{max_x:.3f}", f"{end_x:.3f}", f"{end_y:.3f}", f"{end_z:.3f}",
        f"{end_roll:.3f}", f"{end_pitch:.3f}", nmpc_fail,
        "SUCCESS" if success else "FAIL", log_dir, gif,
    ])
print(
    f"MATRIX RESULT height={height_code}cm tread={float(tread) * 100:.0f}cm "
    f"steps={steps} "
    f"max_x={max_x:.3f} target={target:.3f} end_z={end_z:.3f} "
    f"nmpc_fail={nmpc_fail} verdict={'SUCCESS' if success else 'FAIL'}"
)
PY
done
