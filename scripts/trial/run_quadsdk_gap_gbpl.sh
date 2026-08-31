#!/usr/bin/env bash
# Quad-SDK 穴渡り: global_body_planner(GBP-L)版の実行スクリプト。
#
# run_quadsdk_gap_1m.sh(twist モード、cmd_vel 駆動)との違いだけを説明する:
#   - reference を "twist" ではなく "gbpl" にする → planning.py が
#     global_body_planner_node を起動する(planning.py: reference=='gbpl' でゲート)。
#   - 前進は cmd_vel ではなく **ゴール点** で与える。robot_configs の
#     "goal_state": [x, y] が global_body_planner.goal_state param になる
#     (quad_plan.py L30 / planning.py L114-121)。/clicked_point への publish は不要。
#   - leaping:=true(quad_plan.py の既定も true)で flight/leap プリミティブを許可。
#   - GBP-L は startup_delay 後に terrain_map + state/ground_truth を待って
#     自動でプランする(replanning: true で追従再計画)。local_planner の NMPC が
#     その global plan(LEAP_STANCE/FLIGHT/LAND_STANCE 含む)を追従する。
#
# 地形は run_quadsdk_gap_1m.sh と同じ flat_gaps_*(物理トレンチ + 本物の
# メッシュ穴)。GBP-L は terrain_map の traversability が穴帯で NaN/低値になるのを
# 見て、その帯を非踏破と判定し跳躍プランを立てる想定。
#
# 前提: ビルド済み(colcon build、--symlink-install)。global_body_planner も
# ビルドされていること。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

ROBOT_NS="robot_1"
GAP_WORLD="${GAP_WORLD:-flat_gaps_2m.xml}"       # flat_gaps_2m.xml (step03_1m) / flat_gaps_1p5m.xml (step04_1m)
GAP_TAG="${GAP_TAG:-step03_1m_gbpl}"

LOG_DIR="${REPO_ROOT}/artifacts/logs/quadsdk_${GAP_TAG}"
mkdir -p "${LOG_DIR}"

GOAL_X="${GOAL_X:-12.0}"                          # ゴールの x[m](y=0)。ロボットは x≈0 から +x へ
GOAL_Y="${GOAL_Y:-0.0}"
LEAPING="${LEAPING:-true}"                        # global planner の跳躍プリミティブ許可

# GBP-L は素のトロット歩容 + horizon 26 前提でチューニングされている。
# main の twist 用クロール設定(period 0.9 / horizon 40)のままだと GBP-L の
# body plan を local NMPC が追従できず初手で横倒れする(gbpl_run1 で確認)。
# → この実験の間だけ go2.yaml / local_planner.yaml を素の値へ一時パッチし、
#   trap で必ず元へ戻す(このブランチのコミット内容は twist と同一に保つ)。
GBPL_GAIT_PERIOD="${GBPL_GAIT_PERIOD:-0.36}"
GBPL_GAIT_DUTY="${GBPL_GAIT_DUTY:-0.5}"
GBPL_GAIT_PHASE="${GBPL_GAIT_PHASE:-[0.0, 0.5, 0.5, 0.0]}"
GBPL_HORIZON="${GBPL_HORIZON:-26}"
GBPL_FOOTHOLD_RADIUS="${GBPL_FOOTHOLD_RADIUS:-0.5}"
GBPL_MAX_PLANNING_TIME="${GBPL_MAX_PLANNING_TIME:-10.0}"   # global_body_planner.yaml: 1.0 -> 実験用に延長
GBPL_NUM_LEAP_SAMPLES="${GBPL_NUM_LEAP_SAMPLES:-30}"       # global_body_planner.yaml: 10 -> 増やす
GO2_YAML="${REPO_ROOT}/external/quad-sdk/quad_utils/config/go2.yaml"
LP_YAML="${REPO_ROOT}/external/quad-sdk/local_planner/config/local_planner.yaml"
GBP_YAML="${REPO_ROOT}/external/quad-sdk/global_body_planner/config/global_body_planner.yaml"
CFG_BACKUP_DIR="$(mktemp -d)"
cp "${GO2_YAML}" "${CFG_BACKUP_DIR}/go2.yaml"
cp "${LP_YAML}"  "${CFG_BACKUP_DIR}/local_planner.yaml"
cp "${GBP_YAML}" "${CFG_BACKUP_DIR}/global_body_planner.yaml"
restore_cfg() {
  [ -f "${CFG_BACKUP_DIR}/go2.yaml" ] && cp "${CFG_BACKUP_DIR}/go2.yaml" "${GO2_YAML}" || true
  [ -f "${CFG_BACKUP_DIR}/local_planner.yaml" ] && cp "${CFG_BACKUP_DIR}/local_planner.yaml" "${LP_YAML}" || true
  [ -f "${CFG_BACKUP_DIR}/global_body_planner.yaml" ] && cp "${CFG_BACKUP_DIR}/global_body_planner.yaml" "${GBP_YAML}" || true
  rm -rf "${CFG_BACKUP_DIR}"
}
python3 - "$GO2_YAML" "$LP_YAML" "$GBP_YAML" "$GBPL_GAIT_PERIOD" "$GBPL_GAIT_DUTY" "$GBPL_GAIT_PHASE" "$GBPL_HORIZON" "$GBPL_FOOTHOLD_RADIUS" "$GBPL_MAX_PLANNING_TIME" "$GBPL_NUM_LEAP_SAMPLES" <<'PY'
import re, sys
go2, lp, gbp, period, duty, phase, horizon, frad, maxpt, nleap = sys.argv[1:11]
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
open(gbp, 'w').write(s)
print(f"[gbpl harness] temp-patched: period={period} duty={duty} phase={phase} horizon={horizon} "
      f"foothold_radius={frad} max_planning_time={maxpt} num_leap_samples={nleap}")
PY
# 早期ガード。下で trap cleanup EXIT に差し替える(cleanup も restore_cfg を呼ぶ)。
# EXIT だけだと、外側から SIGTERM/SIGINT で殺されたとき config が一時パッチの
# まま残る。TERM/INT でも必ず戻すよう明示的にフックする。
trap 'restore_cfg' EXIT
trap 'restore_cfg; trap - TERM; kill -TERM $$' TERM
trap 'restore_cfg; trap - INT;  kill -INT  $$' INT
DURATION_S="${DURATION_S:-70.0}"                  # WALK 後、ゴールへ歩かせる実時間[s]
STAND_SETTLE_S="${STAND_SETTLE_S:-8}"             # STAND 後、プランナ起動前の待ち[s]
PLAN_STARTUP_S="${PLAN_STARTUP_S:-6}"             # プランナ起動後、WALK 前の待ち[s](GBP-L startup_delay=2 + 初回プラン余裕)
JOINT_CONTROLLER_WAIT_TIMEOUT_S="${JOINT_CONTROLLER_WAIT_TIMEOUT_S:-40}"
CAMERA_DISTANCE_M="${CAMERA_DISTANCE_M:-9.5}"     # 固定カメラ(run_quadsdk_gap_1m.sh と同じ較正値)
CAMERA_LOOKAT_X_M="${CAMERA_LOOKAT_X_M:-2.0}"

# CSV ロガーは STAND 送信前から記録(起立〜プラン〜WALK〜歩行の全区間)。
RECORDER_DURATION_S="$(echo "${STAND_SETTLE_S} + ${PLAN_STARTUP_S} + ${DURATION_S}" | bc)"

set +u
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

export QUAD_LOGGER_SRC="${LOG_DIR}"

# ==== MuJoCo シミュレータ起動 ====
ros2 launch quad_utils quad_mujoco.py \
  gui:=false \
  world:="${GAP_WORLD}" \
  recording:=true \
  camera_track_robot:=false \
  camera_distance:="${CAMERA_DISTANCE_M}" \
  camera_lookat_x:="${CAMERA_LOOKAT_X_M}" \
  robot_configs:='[{"name": "robot_1", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x 0.0 -y 0.0 -z 0.5"}]' \
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
  pkill -9 -f "ros2_control_node|rviz2|mujoco_recorder|contact_state_publisher_node|mujoco_estimator|body_force_estimator_node|mjcf_to_grid_map_node|grid_map_filters_demo|nmpc_controller|local_planner_node|global_body_planner_node|rviz_interface_node|robot_driver_node|grid_map_visualization|topic_tools/relay|robot_state_publisher|static_transform_publisher|controller_manager/spawner" 2>/dev/null || true
  restore_cfg
}
trap cleanup EXIT

# ==== コントローラマネージャの起動待ち ====
echo "[$(date '+%T.%3N')] Waiting for joint_controller to become active (timeout ${JOINT_CONTROLLER_WAIT_TIMEOUT_S}s)..."
JOINT_CONTROLLER_READY=0
for i in $(seq 1 "${JOINT_CONTROLLER_WAIT_TIMEOUT_S}"); do
  if timeout 5 ros2 service call "/${ROBOT_NS}/controller_manager/list_controllers" \
       controller_manager_msgs/srv/ListControllers "{}" 2>/dev/null \
       | grep -q "name='joint_controller', state='active'"; then
    echo "[$(date '+%T.%3N')] joint_controller is active (waited ~${i}s)"
    JOINT_CONTROLLER_READY=1
    break
  fi
  sleep 1
done
if [ "${JOINT_CONTROLLER_READY}" -ne 1 ]; then
  echo "[$(date '+%T.%3N')] ERROR: joint_controller did not become active within ${JOINT_CONTROLLER_WAIT_TIMEOUT_S}s" >&2
  exit 1
fi

# ==== 記録開始: CSV ロガー ====
python3 "${REPO_ROOT}/src/trial/quadsdk_step01_baseline.py" \
  --robot-ns "${ROBOT_NS}" \
  --duration-s "${RECORDER_DURATION_S}" \
  --csv-path "${LOG_DIR}/state_log.csv" \
  --summary-csv-path "${LOG_DIR}/trials_summary.csv" \
  --velocity-mps "0.0" &
RECORDER_PID=$!

# ==== 起立 ====
echo "[$(date '+%T.%3N')] Sending STAND (control/mode=1)"
ros2 topic pub --once "/${ROBOT_NS}/control/mode" std_msgs/msg/UInt8 "data: 1"
sleep "${STAND_SETTLE_S}"

# ==== プランニングスタック(global_body_planner + local_planner + NMPC)起動 ====
# reference:"gbpl" → planning.py が global_body_planner_node を起動。
# goal_state:[x,y] → global_body_planner.goal_state param。
echo "[$(date '+%T.%3N')] Launching planning stack (quad_plan.py, reference=gbpl, goal=[${GOAL_X}, ${GOAL_Y}], leaping=${LEAPING})"
ros2 launch quad_utils quad_plan.py \
  leaping:="${LEAPING}" \
  robot_configs:="[{\"name\": \"robot_1\", \"type\": \"go2\", \"controller_mode\": \"inverse_dynamics\", \"reference\": \"gbpl\", \"twist_input\": \"none\", \"goal_state\": [${GOAL_X}, ${GOAL_Y}]}]" \
  &
PLAN_PID=$!

sleep "${PLAN_STARTUP_S}"

# ==== WALK ====
# これで robot_driver が local_plan(= global plan を NMPC が追従したもの)に従う。
echo "[$(date '+%T.%3N')] Sending WALK (control/mode=2)"
ros2 topic pub --once "/${ROBOT_NS}/control/mode" std_msgs/msg/UInt8 "data: 2"

# ==== ゴールへ歩かせる(cmd_vel は送らない) ====
echo "[$(date '+%T.%3N')] Walking to goal for ${DURATION_S}s ..."
sleep "${DURATION_S}"

wait "${RECORDER_PID}"

echo "Done. CSV: ${LOG_DIR}/state_log.csv"
LATEST_MP4="$(ls -t "${LOG_DIR}"/logs/mujoco_go2_*.mp4 2>/dev/null | head -1)"
echo "Video (mp4): ${LATEST_MP4:-<not found>}"
echo "Next: bash scripts/trial/make_gif.sh \"${LATEST_MP4:-<mp4>}\" <output.gif>"
