#!/usr/bin/env bash
# Run flat, crossable-gap, stopping-gap, and stair checks with one installed
# planner binary and one installed YAML. This script never edits parameters
# between scenarios; only the world, spawn, duration, speed, and log tag change.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENARIO="${1:-}"
OVERLAY_SETUP="${QUADSDK_OVERLAY_SETUP:-}"

if [ -z "${OVERLAY_SETUP}" ] || [ ! -f "${OVERLAY_SETUP}" ]; then
  echo "ERROR: QUADSDK_OVERLAY_SETUP must name the tested install/setup.bash" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
source "${OVERLAY_SETUP}"
set -u

PLANNER_PREFIX="$(ros2 pkg prefix local_planner)"
PLANNER_YAML="${PLANNER_PREFIX}/share/local_planner/config/local_planner.yaml"
grep -Eq '^[[:space:]]+enabled:[[:space:]]+true' "${PLANNER_YAML}"
grep -Eq '^[[:space:]]+apply_stop_request:[[:space:]]+true' "${PLANNER_YAML}"
grep -Eq '^[[:space:]]+apply_foothold:[[:space:]]+false' "${PLANNER_YAML}"
grep -Eq 'foothold_support_check_mode:[[:space:]]+shadow' "${PLANNER_YAML}"
grep -Eq 'foothold_ik_check_mode:[[:space:]]+shadow' "${PLANNER_YAML}"
grep -Eq 'swing_terrain_check_mode:[[:space:]]+enforce' "${PLANNER_YAML}"

run_gap_world() {
  local world="$1" spawn="$2" duration="$3" speed="$4" tag="$5"
  local log_dir="${REPO_ROOT}/artifacts/logs/quadsdk_${tag}"
  mkdir -p "${log_dir}"
  SPAWN_X_M="${spawn}" GAP_WORLD="${world}.xml" GAP_TAG="${tag}" \
    FORWARD_VEL_MPS="${speed}" DURATION_S="${duration}" \
    bash "${REPO_ROOT}/scripts/trial/run_quadsdk_gap_1m.sh" \
    2>&1 | tee "${log_dir}/run.log"
}

case "${SCENARIO}" in
  flat)
    run_gap_world flat_wide 0.0 22 0.30 step19_unified_flat
    ;;
  cross)
    run_gap_world flat_gaps_2m 0.0 34 0.30 step19_unified_cross30
    ;;
  repeat)
    run_gap_world flat_repgap_s15g15n5 0.0 34 0.30 step19_unified_repeat15x5
    ;;
  stop)
    run_gap_world flat_trench_s09_100 -2.0 26 0.30 step19_unified_stop100
    ;;
  stair)
    GAP_TAG=step19_unified_stair \
      bash "${REPO_ROOT}/scripts/trial/run_quadsdk_jp_stair.sh"
    ;;
  *)
    echo "Usage: QUADSDK_OVERLAY_SETUP=<install/setup.bash> $0 {flat|cross|repeat|stop|stair}" >&2
    exit 2
    ;;
esac
