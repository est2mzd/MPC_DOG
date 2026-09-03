#!/usr/bin/env bash
# Step 17 前方ジャンプの実行スクリプト。
#
# 判断ロジック(RRT/global planner の穴判定)は使わない。穴シナリオも使わない。
# jump_mode:=force_leap のとき global_body_planner ノードが RRT を回さず、
# ロボットが静止した瞬間に「1 回のジャンプ」(PRELOAD->REAR_PUSH->FLIGHT->
# FRONT_LAND->SETTLE)を現在姿勢から組み立てて body_plan に流す。
# local_planner + NMPC + inverse_dynamics はそれを追従するだけ。
#
# 計測は CSV(quadsdk_step17_jump.py)。後脚 BL/BR の x/z を踏切前・飛翔中・
# 着地後で見る。前進距離は JUMP_TAKEOFF_VX を上げて伸ばす。
#
# 例:
#   その場ジャンプ:            bash scripts/trial/run_step17_jump.sh
#   少し前進:  JUMP_TAKEOFF_VX=0.6 bash scripts/trial/run_step17_jump.sh
#   ホップ高さ調整: JUMP_DZ=1.9 bash scripts/trial/run_step17_jump.sh
set -euo pipefail

# CSV ロガー(system python3 の rclpy)がプロジェクト .venv(python 3.11)を
# 拾って _rclpy_pybind11 の ABI 不一致で落ちるため、env 衛生をかける。
unset VIRTUAL_ENV || true
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_NS="robot_1"

WORLD="${WORLD:-flat_wide.xml}"               # 穴なし平地
STEP_TAG="${STEP_TAG:-step17_jump}"
LOG_DIR="${REPO_ROOT}/artifacts/logs/quadsdk_${STEP_TAG}"
OUT_DIR="${OUT_DIR:-${REPO_ROOT}/artifacts/step17/${STEP_TAG}}"
mkdir -p "${LOG_DIR}" "${OUT_DIR}"

SPAWN_X_M="${SPAWN_X_M:-0.0}"
JUMP_MODE="${JUMP_MODE:-force_leap}"          # off | auto | force_leap
JUMP_TAKEOFF_VX="${JUMP_TAKEOFF_VX:-0.0}"     # 前方離陸速度[m/s]。0=その場
# getRandomLeapAction は範囲内でサンプリングして実行可能な action を探す。
# 範囲をつぶす(min=max)と探索できず「could not build a valid jump action」に
# なるので、幅を残す。
JUMP_DZ_LO="${JUMP_DZ_LO:-1.1}"               # 鉛直インパルス下限[m/s]
JUMP_DZ_HI="${JUMP_DZ_HI:-1.6}"               # 鉛直インパルス上限[m/s]
JUMP_TS_LO="${JUMP_TS_LO:-0.20}"             # 踏切支持時間 下限[s]
JUMP_TS_HI="${JUMP_TS_HI:-0.30}"             # 踏切支持時間 上限[s]
JUMP_PRELOAD_FRACTION="${JUMP_PRELOAD_FRACTION:-0.4}"
JUMP_FRONT_LAND_FRACTION="${JUMP_FRONT_LAND_FRACTION:-0.5}"
GBPL_MU="${GBPL_MU:-0.6}"                     # 踏切で滑らないよう mu を上げる
JUMP_ATT_WEIGHT="${JUMP_ATT_WEIGHT:-25.0}"   # NMPC の roll/pitch 追従重み(既定 0.5)

# --- 一時パッチ(trap で復元) ---
# Step 17b G1: ジャンプ中は接触が primitive で上書きされるが、ホライズンに漏れる
# 非ジャンプステップも四脚接地にしたいので、gait をトロットではなく実質 STAND
# (duty≈0.98 / phase 全 0)にする。
GBPL_GAIT_PERIOD="${GBPL_GAIT_PERIOD:-0.36}"
GBPL_GAIT_DUTY="${GBPL_GAIT_DUTY:-0.98}"
GBPL_GAIT_PHASE="${GBPL_GAIT_PHASE:-[0.0, 0.0, 0.0, 0.0]}"
GBPL_HORIZON="${GBPL_HORIZON:-26}"
# Step 17b G3: ジャンプで一時的に増える位置誤差でも着地後 STAND へ確実に落ちるよう
# stand_pos_error_threshold を広げる。
STAND_POS_ERR_THRESH="${STAND_POS_ERR_THRESH:-0.15}"

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
  "$GBPL_GAIT_PHASE" "$GBPL_HORIZON" "$JUMP_MODE" "$JUMP_PRELOAD_FRACTION" \
  "$JUMP_FRONT_LAND_FRACTION" "$GBPL_MU" "$JUMP_TAKEOFF_VX" "$JUMP_DZ_LO" "$JUMP_DZ_HI" \
  "$JUMP_TS_LO" "$JUMP_TS_HI" "$JUMP_ATT_WEIGHT" "$STAND_POS_ERR_THRESH" <<'PY'
import re, sys
(go2, lp, gbp, period, duty, phase, horizon, jump_mode, preload_frac,
 front_frac, mu, tvx, dz_lo, dz_hi, ts_lo, ts_hi, attw, sper) = sys.argv[1:19]

s = open(go2).read()
s = re.sub(r'^(\s*period:\s*)[^\n#]*', rf'\g<1>{period} ', s, flags=re.M)
s = re.sub(r'^(\s*duty_cycles:\s*)\[[^\]]*\]', rf'\g<1>[{duty}, {duty}, {duty}, {duty}]', s, flags=re.M)
s = re.sub(r'^(\s*phase_offsets:\s*)\[[^\]]*\]', rf'\g<1>{phase}', s, flags=re.M)
# simple-model NMPC x_weights: idx 3,4 = roll,pitch position tracking weight.
# Keep every element float-formatted: a mixed int/float array is an
# RCLInvalidROSArgsError and aborts every node that loads go2.yaml.
attw_f = repr(float(attw))
def bump_att(m):
    vals = [repr(float(v.strip())) for v in m.group(1).split(',')]
    if len(vals) >= 5:
        vals[3] = attw_f
        vals[4] = attw_f
    return 'x_weights: [' + ', '.join(vals) + ']'
s = re.sub(r'x_weights:\s*\[([^\]]*)\]', bump_att, s, count=1)
open(go2, 'w').write(s)

s = open(lp).read()
s = re.sub(r'^(\s*horizon_length:\s*)[0-9]+', rf'\g<1>{horizon}', s, flags=re.M)
s = re.sub(r'^(\s*stand_pos_error_threshold:\s*)[^\n#]*', rf'\g<1>{sper} ', s, flags=re.M)
open(lp, 'w').write(s)

s = open(gbp).read()
s = re.sub(r'^(\s*mu:\s*)[^\n#]*', rf'\g<1>{mu} ', s, flags=re.M)
s = re.sub(r'^(\s*dz0_min:\s*)[^\n#]*', rf'\g<1>{dz_lo} ', s, flags=re.M)
s = re.sub(r'^(\s*dz0_max:\s*)[^\n#]*', rf'\g<1>{dz_hi} ', s, flags=re.M)
s = re.sub(r'^(\s*t_s_min:\s*)[^\n#]*', rf'\g<1>{ts_lo} ', s, flags=re.M)
s = re.sub(r'^(\s*t_s_max:\s*)[^\n#]*', rf'\g<1>{ts_hi} ', s, flags=re.M)
for key, val in (("jump_mode", jump_mode),
                 ("jump_preload_fraction", preload_frac),
                 ("jump_front_land_fraction", front_frac),
                 ("jump_takeoff_vx", tvx)):
    if re.search(rf'^\s*{key}:\s', s, flags=re.M):
        s = re.sub(rf'^(\s*{key}:\s*)[^\n#]*', rf'\g<1>{val} ', s, flags=re.M)
    else:
        s = re.sub(r'^(\s*num_leap_samples:.*\n)',
                   rf'\g<1>      {key}: {val}\n', s, flags=re.M)
open(gbp, 'w').write(s)
print(f"[step17 harness] temp-patched: gait period={period} duty={duty} phase={phase} "
      f"horizon={horizon} stand_err={sper} mu={mu} jump_mode={jump_mode} "
      f"dz=[{dz_lo},{dz_hi}] t_s=[{ts_lo},{ts_hi}] "
      f"preload_frac={preload_frac} front_land_frac={front_frac} att_w={attw}")
PY

trap 'restore_cfg' EXIT
trap 'restore_cfg; trap - TERM; kill -TERM $$' TERM
trap 'restore_cfg; trap - INT;  kill -INT  $$' INT

DURATION_S="${DURATION_S:-28.0}"
STAND_SETTLE_S="${STAND_SETTLE_S:-8}"
PLAN_STARTUP_S="${PLAN_STARTUP_S:-6}"
JOINT_CONTROLLER_WAIT_TIMEOUT_S="${JOINT_CONTROLLER_WAIT_TIMEOUT_S:-70}"
CAMERA_DISTANCE_M="${CAMERA_DISTANCE_M:-3.0}"   # 近い固定カメラ
CAMERA_LOOKAT_X_M="${CAMERA_LOOKAT_X_M:-0.2}"
RECORDER_DURATION_S="$(echo "${STAND_SETTLE_S} + ${PLAN_STARTUP_S} + ${DURATION_S}" | bc)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u
export QUAD_LOGGER_SRC="${LOG_DIR}"

ros2 launch quad_utils quad_mujoco.py \
  gui:=false \
  world:="${WORLD}" \
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

echo "[$(date '+%T.%3N')] planning stack (gbpl, jump_mode=${JUMP_MODE}, takeoff_vx=${JUMP_TAKEOFF_VX})"
ros2 launch quad_utils quad_plan.py \
  leaping:=true \
  robot_configs:="[{\"name\": \"robot_1\", \"type\": \"go2\", \"controller_mode\": \"inverse_dynamics\", \"reference\": \"gbpl\", \"twist_input\": \"none\", \"goal_state\": [5.0, 0.0]}]" \
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
