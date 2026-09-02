#!/usr/bin/env bash
# Step 17 前方ジャンプの実行スクリプト(global_body_planner + FORCE_LEAP)。
#
# run_quadsdk_gap_gbpl.sh をベースに、次だけ変える:
#   - GBP に global_body_planner.jump_mode を注入(既定 force_leap)。
#     yaml を一時パッチし trap で必ず戻す(このブランチのコミット内容は不変)。
#   - CSV ロガーを quadsdk_step17_jump.py に差し替え(足先位置・実測接触・
#     関節 pos/vel/指令トルク・primitive_id・jump_phase を記録)。
#   - world / goal / spawn / duration を env で選べる(Stage 2〜6 兼用)。
#
# GBP-L は素のトロット歩容 + horizon 26 前提。main の twist 用クロール設定の
# ままだと NMPC が GBP body plan を追従できず横倒れするため、この実験の間だけ
# go2.yaml / local_planner.yaml を素の値へ一時パッチする(trap で復元)。
#
# 使い方の例:
#   Stage 2/3 (平地・短い前進ジャンプ):
#     GAP_WORLD=flat_wide.xml GOAL_X=1.6 DURATION_S=45 bash scripts/trial/run_step17_jump.sh
#   Stage 6 (0.30 m トレンチ越え):
#     GAP_WORLD=flat_trench_s09_30.xml GOAL_X=3.0 SPAWN_X_M=-1.5 DURATION_S=55 \
#       bash scripts/trial/run_step17_jump.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_NS="robot_1"

GAP_WORLD="${GAP_WORLD:-flat_wide.xml}"
STEP_TAG="${STEP_TAG:-step17_jump}"
LOG_DIR="${REPO_ROOT}/artifacts/logs/quadsdk_${STEP_TAG}"
OUT_DIR="${OUT_DIR:-${REPO_ROOT}/artifacts/step17/${STEP_TAG}}"
mkdir -p "${LOG_DIR}" "${OUT_DIR}"

GOAL_X="${GOAL_X:-1.6}"
GOAL_Y="${GOAL_Y:-0.0}"
SPAWN_X_M="${SPAWN_X_M:-0.0}"
JUMP_MODE="${JUMP_MODE:-force_leap}"          # off | auto | force_leap
JUMP_PRELOAD_FRACTION="${JUMP_PRELOAD_FRACTION:-0.4}"
JUMP_FRONT_LAND_FRACTION="${JUMP_FRONT_LAND_FRACTION:-0.5}"

# --- GBP-L 前提のトロット歩容へ一時パッチ(gbpl スクリプトと同じ) ---
GBPL_GAIT_PERIOD="${GBPL_GAIT_PERIOD:-0.36}"
GBPL_GAIT_DUTY="${GBPL_GAIT_DUTY:-0.5}"
GBPL_GAIT_PHASE="${GBPL_GAIT_PHASE:-[0.0, 0.5, 0.5, 0.0]}"
GBPL_HORIZON="${GBPL_HORIZON:-26}"
GBPL_FOOTHOLD_RADIUS="${GBPL_FOOTHOLD_RADIUS:-0.5}"
GBPL_MAX_PLANNING_TIME="${GBPL_MAX_PLANNING_TIME:-10.0}"
GBPL_NUM_LEAP_SAMPLES="${GBPL_NUM_LEAP_SAMPLES:-40}"
GBPL_MU="${GBPL_MU:-0.5}"                     # jump needs Fx<=mu*Fz; 0.25 slips

GO2_YAML="${REPO_ROOT}/external/quad-sdk/quad_utils/config/go2.yaml"
LP_YAML="${REPO_ROOT}/external/quad-sdk/local_planner/config/local_planner.yaml"
GBP_YAML="${REPO_ROOT}/external/quad-sdk/global_body_planner/config/global_body_planner.yaml"
CFG_BACKUP_DIR="$(mktemp -d)"
cp "${GO2_YAML}" "${CFG_BACKUP_DIR}/go2.yaml"
cp "${LP_YAML}"  "${CFG_BACKUP_DIR}/local_planner.yaml"
cp "${GBP_YAML}" "${CFG_BACKUP_DIR}/global_body_planner.yaml"
restore_cfg() {
  cp "${CFG_BACKUP_DIR}/go2.yaml" "${GO2_YAML}" 2>/dev/null || true
  cp "${CFG_BACKUP_DIR}/local_planner.yaml" "${LP_YAML}" 2>/dev/null || true
  cp "${CFG_BACKUP_DIR}/global_body_planner.yaml" "${GBP_YAML}" 2>/dev/null || true
  rm -rf "${CFG_BACKUP_DIR}"
}

python3 - "$GO2_YAML" "$LP_YAML" "$GBP_YAML" "$GBPL_GAIT_PERIOD" "$GBPL_GAIT_DUTY" \
  "$GBPL_GAIT_PHASE" "$GBPL_HORIZON" "$GBPL_FOOTHOLD_RADIUS" "$GBPL_MAX_PLANNING_TIME" \
  "$GBPL_NUM_LEAP_SAMPLES" "$JUMP_MODE" "$JUMP_PRELOAD_FRACTION" "$JUMP_FRONT_LAND_FRACTION" \
  "$GBPL_MU" <<'PY'
import re, sys
(go2, lp, gbp, period, duty, phase, horizon, frad, maxpt, nleap,
 jump_mode, preload_frac, front_frac, mu) = sys.argv[1:15]

s = open(go2).read()
s = re.sub(r'^(\s*period:\s*)[^\n#]*', rf'\g<1>{period} ', s, flags=re.M)
s = re.sub(r'^(\s*duty_cycles:\s*)\[[^\]]*\]', rf'\g<1>[{duty}, {duty}, {duty}, {duty}]', s, flags=re.M)
s = re.sub(r'^(\s*phase_offsets:\s*)\[[^\]]*\]', rf'\g<1>{phase}', s, flags=re.M)
s = re.sub(r'^(\s*foothold_search_radius:\s*)[^\n#]*', rf'\g<1>{frad} ', s, flags=re.M)
open(go2, 'w').write(s)

s = open(lp).read()
s = re.sub(r'^(\s*horizon_length:\s*)[0-9]+', rf'\g<1>{horizon}', s, flags=re.M)
open(lp, 'w').write(s)

s = open(gbp).read()
s = re.sub(r'^(\s*max_planning_time:\s*)[^\n#]*', rf'\g<1>{maxpt} ', s, flags=re.M)
s = re.sub(r'^(\s*num_leap_samples:\s*)[^\n#]*', rf'\g<1>{nleap} ', s, flags=re.M)
s = re.sub(r'^(\s*mu:\s*)[^\n#]*', rf'\g<1>{mu} ', s, flags=re.M)
# jump_mode / fractions を global_body_planner ブロックへ注入(無ければ num_leap_samples の後に足す)。
for key, val in (("jump_mode", jump_mode),
                 ("jump_preload_fraction", preload_frac),
                 ("jump_front_land_fraction", front_frac)):
    if re.search(rf'^\s*{key}:\s', s, flags=re.M):
        s = re.sub(rf'^(\s*{key}:\s*)[^\n#]*', rf'\g<1>{val} ', s, flags=re.M)
    else:
        s = re.sub(r'^(\s*num_leap_samples:.*\n)',
                   rf'\g<1>      {key}: {val}\n', s, flags=re.M)
open(gbp, 'w').write(s)
print(f"[step17 harness] temp-patched: period={period} duty={duty} horizon={horizon} "
      f"num_leap_samples={nleap} mu={mu} jump_mode={jump_mode} "
      f"preload_frac={preload_frac} front_land_frac={front_frac}")
PY

trap 'restore_cfg' EXIT
trap 'restore_cfg; trap - TERM; kill -TERM $$' TERM
trap 'restore_cfg; trap - INT;  kill -INT  $$' INT

DURATION_S="${DURATION_S:-45.0}"
STAND_SETTLE_S="${STAND_SETTLE_S:-8}"
PLAN_STARTUP_S="${PLAN_STARTUP_S:-6}"
JOINT_CONTROLLER_WAIT_TIMEOUT_S="${JOINT_CONTROLLER_WAIT_TIMEOUT_S:-40}"
CAMERA_DISTANCE_M="${CAMERA_DISTANCE_M:-6.0}"
CAMERA_LOOKAT_X_M="${CAMERA_LOOKAT_X_M:-1.5}"
RECORDER_DURATION_S="$(echo "${STAND_SETTLE_S} + ${PLAN_STARTUP_S} + ${DURATION_S}" | bc)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u
export QUAD_LOGGER_SRC="${LOG_DIR}"

ros2 launch quad_utils quad_mujoco.py \
  gui:=false \
  world:="${GAP_WORLD}" \
  recording:=true \
  camera_track_robot:=false \
  camera_distance:="${CAMERA_DISTANCE_M}" \
  camera_lookat_x:="${CAMERA_LOOKAT_X_M}" \
  robot_configs:="[{\"name\": \"robot_1\", \"type\": \"go2\", \"controller\": \"inverse_dynamics\", \"init_pose\": \"-x ${SPAWN_X_M} -y 0.0 -z 0.5\"}]" \
  &
MUJOCO_PID=$!

cleanup() {
  for pid in "${PLAN_PID:-}" "${MUJOCO_PID:-}"; do
    [ -n "${pid}" ] || continue
    kill -INT "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  done
  for _ in $(seq 1 10); do
    kill -0 "${PLAN_PID:-}" 2>/dev/null || kill -0 "${MUJOCO_PID:-}" 2>/dev/null || break
    sleep 1
  done
  for pid in "${PLAN_PID:-}" "${MUJOCO_PID:-}"; do
    [ -n "${pid}" ] || continue
    kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  done
  wait "${PLAN_PID:-}" "${MUJOCO_PID:-}" 2>/dev/null || true
  pkill -INT -f "mujoco_recorder" 2>/dev/null || true
  for _ in $(seq 1 8); do pgrep -f "mujoco_recorder" >/dev/null 2>&1 || break; sleep 1; done
  pkill -9 -f "ros2_control_node|rviz2|mujoco_recorder|contact_state_publisher_node|mujoco_estimator|body_force_estimator_node|mjcf_to_grid_map_node|grid_map_filters_demo|nmpc_controller|local_planner_node|global_body_planner_node|rviz_interface_node|robot_driver_node|grid_map_visualization|topic_tools/relay|robot_state_publisher|static_transform_publisher|controller_manager/spawner" 2>/dev/null || true
  restore_cfg
}
trap cleanup EXIT

echo "[$(date '+%T.%3N')] Waiting for joint_controller (timeout ${JOINT_CONTROLLER_WAIT_TIMEOUT_S}s)..."
JOINT_CONTROLLER_READY=0
for i in $(seq 1 "${JOINT_CONTROLLER_WAIT_TIMEOUT_S}"); do
  if timeout 5 ros2 service call "/${ROBOT_NS}/controller_manager/list_controllers" \
       controller_manager_msgs/srv/ListControllers "{}" 2>/dev/null \
       | grep -q "name='joint_controller', state='active'"; then
    echo "[$(date '+%T.%3N')] joint_controller active (~${i}s)"
    JOINT_CONTROLLER_READY=1
    break
  fi
  sleep 1
done
[ "${JOINT_CONTROLLER_READY}" -eq 1 ] || { echo "ERROR: joint_controller not active" >&2; exit 1; }

python3 "${REPO_ROOT}/src/trial/quadsdk_step17_jump.py" \
  --robot-ns "${ROBOT_NS}" \
  --duration-s "${RECORDER_DURATION_S}" \
  --csv-path "${OUT_DIR}/state_log.csv" \
  --summary-csv-path "${OUT_DIR}/trials_summary.csv" \
  --velocity-mps "0.0" &
RECORDER_PID=$!

echo "[$(date '+%T.%3N')] STAND"
ros2 topic pub --once "/${ROBOT_NS}/control/mode" std_msgs/msg/UInt8 "data: 1"
sleep "${STAND_SETTLE_S}"

echo "[$(date '+%T.%3N')] planning stack (gbpl, goal=[${GOAL_X}, ${GOAL_Y}], jump_mode=${JUMP_MODE})"
ros2 launch quad_utils quad_plan.py \
  leaping:=true \
  robot_configs:="[{\"name\": \"robot_1\", \"type\": \"go2\", \"controller_mode\": \"inverse_dynamics\", \"reference\": \"gbpl\", \"twist_input\": \"none\", \"goal_state\": [${GOAL_X}, ${GOAL_Y}]}]" \
  &
PLAN_PID=$!
sleep "${PLAN_STARTUP_S}"

echo "[$(date '+%T.%3N')] WALK"
ros2 topic pub --once "/${ROBOT_NS}/control/mode" std_msgs/msg/UInt8 "data: 2"

echo "[$(date '+%T.%3N')] running ${DURATION_S}s ..."
sleep "${DURATION_S}"

wait "${RECORDER_PID}" || true
echo "Done. CSV: ${OUT_DIR}/state_log.csv"
LATEST_MP4="$(ls -t "${LOG_DIR}"/logs/mujoco_go2_*.mp4 2>/dev/null | head -1)"
echo "Video (mp4): ${LATEST_MP4:-<not found>}"
[ -n "${LATEST_MP4:-}" ] && cp "${LATEST_MP4}" "${OUT_DIR}/clip.mp4" 2>/dev/null || true
echo "Next: bash scripts/trial/make_gif.sh \"${OUT_DIR}/clip.mp4\" \"${OUT_DIR}/clip.gif\""
