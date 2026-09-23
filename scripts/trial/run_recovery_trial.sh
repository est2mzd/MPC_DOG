#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENARIO="${1:?usage: $0 flat POSE TAG | stair auto TAG}"
POSE="${2:?usage: $0 flat POSE TAG | stair auto TAG}"
TAG="${3:?usage: $0 flat POSE TAG | stair auto TAG}"

if ! [[ "${TAG}" =~ ^[a-z0-9_]+$ ]]; then
  echo "TAG must contain lowercase letters, digits, and underscores" >&2
  exit 2
fi
if [ "${SCENARIO}" = "flat" ]; then
  case "${POSE}" in
    supine) ROLL="3.141592653589793" ;;
    left_side) ROLL="-1.5707963267948966" ;;
    right_side) ROLL="1.5707963267948966" ;;
    *) echo "flat pose must be supine, left_side, or right_side" >&2; exit 2 ;;
  esac
  WORLD="flat_wide.xml"
  SPAWN_X="0.0"
  SPAWN_Z="0.35"
  DURATION="${RECOVERY_DURATION_S:-38}"
  CAMERA_DISTANCE="${CAMERA_DISTANCE_M:-6.0}"
  CAMERA_LOOKAT_X="${CAMERA_LOOKAT_X_M:-1.0}"
  AUTOMATIC_RECOVERY="true"
elif [ "${SCENARIO}" = "stair" ] && [ "${POSE}" = "auto" ]; then
  ROLL="0.0"
  WORLD="jp_stair_matrix_h15_d30_n04.xml"
  SPAWN_X="1.0"
  SPAWN_Z="0.5"
  DURATION="${RECOVERY_DURATION_S:-110}"
  CAMERA_DISTANCE="${CAMERA_DISTANCE_M:-8.0}"
  CAMERA_LOOKAT_X="${CAMERA_LOOKAT_X_M:-4.5}"
  AUTOMATIC_RECOVERY="false"
else
  echo "scenario must be flat POSE or stair auto" >&2
  exit 2
fi

LOG_DIR="${REPO_ROOT}/artifacts/logs/recovery_modes/${TAG}"
mkdir -p "${LOG_DIR}"
RUN_LOG="${LOG_DIR}/console.log"
RECOVERY_CSV="${LOG_DIR}/recovery.csv"
STATE_CSV="${LOG_DIR}/state_log.csv"
VIDEO_DIR="${LOG_DIR}/video"
mkdir -p "${VIDEO_DIR}"

unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
export LD_LIBRARY_PATH="${REPO_ROOT}/artifacts/soname:/usr/local/lib:${LD_LIBRARY_PATH:-}"
set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u
export QUAD_LOGGER_SRC="${VIDEO_DIR}"

cleanup() {
  for pid in "${CMD_PID:-}" "${RECORDER_PID:-}" "${PLAN_PID:-}" "${RECOVERY_PID:-}" "${MUJOCO_PID:-}"; do
    [ -n "${pid}" ] || continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  pkill -INT -f "mujoco_recorder" 2>/dev/null || true
  for _ in $(seq 1 8); do
    pgrep -f "mujoco_recorder" >/dev/null 2>&1 || break
    sleep 1
  done
  pkill -9 -f "recovery_controller_node|ros2_control_node|mujoco_recorder|contact_state_publisher_node|mujoco_estimator|body_force_estimator_node|mjcf_to_grid_map_node|grid_map_filters_demo|nmpc_controller|local_planner_node|global_body_planner_node|rviz_interface_node|robot_driver_node|grid_map_visualization|topic_tools/relay|robot_state_publisher|static_transform_publisher|controller_manager/spawner" 2>/dev/null || true
}
trap cleanup EXIT

{
  echo "RECOVERY TRIAL scenario=${SCENARIO} pose=${POSE} tag=${TAG}"
  ros2 launch quad_utils quad_mujoco.py \
    gui:=false \
    world:="${WORLD}" \
    recording:=true \
    camera_track_robot:=false \
    camera_distance:="${CAMERA_DISTANCE}" \
    camera_lookat_x:="${CAMERA_LOOKAT_X}" \
    robot_configs:="[{\"name\":\"robot_1\",\"type\":\"go2\",\"controller\":\"inverse_dynamics\",\"init_pose\":\"-x ${SPAWN_X} -y 0.0 -z ${SPAWN_Z} -R ${ROLL} -P 0.0 -Y 0.0\"}]" &
  MUJOCO_PID=$!

  ready=0
  for second in $(seq 1 50); do
    if timeout 4 ros2 service call /robot_1/controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers "{}" 2>/dev/null |
      grep -q "name='joint_controller', state='active'"; then
      echo "joint_controller active after ${second}s"
      ready=1
      break
    fi
    sleep 1
  done
  if [ "${ready}" -ne 1 ]; then
    echo "joint_controller did not become active" >&2
    exit 1
  fi

  python3 "${REPO_ROOT}/src/trial/quadsdk_step01_baseline.py" \
    --robot-ns robot_1 \
    --duration-s "$((DURATION + 12))" \
    --csv-path "${STATE_CSV}" \
    --summary-csv-path "${LOG_DIR}/state_summary.csv" \
    --velocity-mps 0.10 &
  RECORDER_PID=$!

  if ros2 node list 2>/dev/null | grep -q "/robot_1/robot_driver"; then
    DRIVER_PARAMS="${LOG_DIR}/robot_driver_params.yaml"
    : >"${DRIVER_PARAMS}"
    for _ in $(seq 1 10); do
      if ros2 param dump /robot_1/robot_driver >"${DRIVER_PARAMS}" 2>/dev/null \
        && grep -q "robot_description" "${DRIVER_PARAMS}"; then
        break
      fi
      sleep 1
    done
    pkill -INT -f "/robot_driver_node" 2>/dev/null || true
    for _ in $(seq 1 10); do
      pgrep -f "/robot_driver_node" >/dev/null 2>&1 || break
      sleep 1
    done
    ros2 launch recovery_controller recovery_controller.launch.py \
      namespace:=robot_1 \
      driver_params_path:="${DRIVER_PARAMS}" \
      log_path:="${RECOVERY_CSV}" \
      automatic_recovery:="${AUTOMATIC_RECOVERY}" &
    RECOVERY_PID=$!
  else
    echo "robot_driver is absent, recovery node owns joint commands"
    ros2 run recovery_controller recovery_controller_node --ros-args \
      -r __ns:=/robot_1 \
      -p use_sim_time:=true \
      -p log_path:="${RECOVERY_CSV}" \
      -p automatic_recovery:="${AUTOMATIC_RECOVERY}" &
    RECOVERY_PID=$!
  fi

  for second in $(seq 1 30); do
    if timeout 3 ros2 topic echo --once /robot_1/recovery/status \
      std_msgs/msg/String >/dev/null 2>&1; then
      echo "recovery controller active after ${second}s"
      break
    fi
    sleep 1
  done

  if [ "${SCENARIO}" = "stair" ]; then
    timeout 5 ros2 topic pub --once \
      /robot_1/control/mode std_msgs/msg/UInt8 "data: 1" || true
    stood=0
    for second in $(seq 1 20); do
      if timeout 3 ros2 topic echo --once /robot_1/recovery/status \
        std_msgs/msg/String 2>/dev/null | grep -q '"pose": "upright"'; then
        echo "stood before walk after ${second}s"
        stood=1
        break
      fi
    done
    if [ "${stood}" -ne 1 ]; then
      echo "robot did not stand before the stair walk" >&2
    fi
  fi

  ros2 launch quad_utils quad_plan.py \
    robot_configs:='[{"name":"robot_1","type":"go2","controller_mode":"inverse_dynamics","reference":"twist","twist_input":"none"}]' &
  PLAN_PID=$!
  sleep 3
  timeout 5 ros2 topic pub --once \
    /robot_1/control/mode std_msgs/msg/UInt8 "data: 1" || true
  if [ "${SCENARIO}" = "stair" ]; then
    ros2 param set /robot_1/recovery_controller automatic_recovery true
    echo "automatic recovery armed after stand"
  fi

  timeout "${DURATION}" ros2 topic pub -r 50 \
    /robot_1/recovery/cmd_vel_input geometry_msgs/msg/Twist \
    "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
  CMD_PID=$!
  wait "${CMD_PID}" || true
  for _ in $(seq 1 25); do
    if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
} >"${RUN_LOG}" 2>&1

cleanup
trap - EXIT
wait "${RECORDER_PID}" 2>/dev/null || true

MP4="$(ls -t "${VIDEO_DIR}"/logs/mujoco_go2_*.mp4 2>/dev/null | head -1 || true)"
GIF="${LOG_DIR}/trial.gif"
if [ -n "${MP4}" ] && ffprobe -v error "${MP4}" >/dev/null 2>&1; then
  VIDEO_DURATION="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "${MP4}")"
  bash "${REPO_ROOT}/scripts/trial/make_gif.sh" \
    "${MP4}" "${GIF}" 3 480 0 "${VIDEO_DURATION}"
fi

python3 - "${SCENARIO}" "${POSE}" "${RECOVERY_CSV}" "${STATE_CSV}" "${LOG_DIR}/summary.json" "${GIF}" <<'PY'
import csv
import json
import math
import os
import sys

scenario, pose, recovery_path, state_path, summary_path, gif = sys.argv[1:]
recovery = list(csv.DictReader(open(recovery_path)))
if os.path.exists(state_path):
    state = list(csv.DictReader(open(state_path)))
    start_x = float(state[0]["base_pos_x_m"])
    end_x = float(state[-1]["base_pos_x_m"])
    distance_m = end_x - start_x
else:
    distance_m = float("nan")
last = recovery[-1]
modes = [row["mode"] for row in recovery]
operations = [row["operation"] for row in recovery]
upright_rows = [
    row for row in recovery
    if float(row["u"]) >= 0.82 and float(row["height_m"]) >= 0.22
]
summary = {
    "scenario": scenario,
    "initial_pose": pose,
    "samples": len(recovery),
    "attempts": max(int(row["attempt"]) for row in recovery),
    "recovery_started": "RECOVERY" in modes,
    "upright_reached": bool(upright_rows),
    "resume_reached": "RESUME" in modes or "walking" in operations,
    "distance_m": distance_m,
    "final_mode": last["mode"],
    "final_pose": last["pose"],
    "failure_reason": last["failure_reason"],
    "gif": gif if os.path.exists(gif) else "",
}
summary["success"] = (
    summary["upright_reached"]
    and summary["resume_reached"]
    and math.isfinite(summary["distance_m"])
    and summary["distance_m"] >= (2.0 if scenario == "flat" else 1.0)
)
with open(summary_path, "w") as stream:
    json.dump(summary, stream, indent=2, sort_keys=True)
print(json.dumps(summary, sort_keys=True))
PY

echo "Done: ${LOG_DIR}"
