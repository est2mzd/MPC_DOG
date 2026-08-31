#!/usr/bin/env bash
# Quad-SDK Step 01: 基準歩行トライアルの実行スクリプト。
#
# 背景: 2026-08-30時点までの調査で、以下2点が「歩けたり歩けなかったりする」
# 再現性問題の確認済みの原因として特定されている(詳細: agent_reports/step01/quad_sdk_step01_investigation.md)。
#   1. 起動シーケンス: 固定sleepでSTANDを送ると、実際に関節へトルクを伝える
#      ros2_controlの`joint_controller`がまだアクティブでないことがある
#      → controller_manager_msgs/srv/ListControllersをポーリングして待つ。
#   2. 地面サイズ: 既定のflat.xmlは地面が9m弱しかなく、10m規模の歩行試験では
#      端から落ちる → flat_wide.xml(同じ単純な形状のまま拡大)を使う。
# ただし、上記の修正だけでは「毎回必ず歩く」ことまでは保証されていない
# (2026-08-30の再検証で、同一条件でも転倒する試行を確認済み)。そのため
# このスクリプトは毎回、追従しない固定カメラでの録画(GIF化はconvert_to_gif.sh)と
# CSVログの両方を必ず生成する。歩行の成否は、CSVの数値だけでなくGIFを
# 目視して両方が一致して初めて判断すること(数値だけでの成功判定はしない)。
#
# 前提: chatgpt_instruction/quad_sdk_build_agent_handoff.md の手順でビルド済みであること。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

ROBOT_NS="robot_1"
GAP_WORLD="${GAP_WORLD:-flat_gaps_2m.xml}"   # flat_gaps_2m.xml (step03_1m) / flat_gaps_1p5m.xml (step04_1m)
GAP_TAG="${GAP_TAG:-step03_1m}"

LOG_DIR="${REPO_ROOT}/artifacts/logs/quadsdk_${GAP_TAG}"
mkdir -p "${LOG_DIR}"
FORWARD_VEL_MPS="${FORWARD_VEL_MPS:-0.3}"          # 前進速度指令[m/s]
DURATION_S="${DURATION_S:-25.0}"                   # cmd_velを送り続ける実時間[s]
STAND_SETTLE_S="${STAND_SETTLE_S:-8}"              # STAND送信後、プランナ起動前に待つ時間[s]
PLAN_STARTUP_S="${PLAN_STARTUP_S:-3}"              # プランナ起動後、WALK送信前に待つ時間[s]
JOINT_CONTROLLER_WAIT_TIMEOUT_S="${JOINT_CONTROLLER_WAIT_TIMEOUT_S:-40}"  # joint_controller起動待ちの上限[s]
CAMERA_DISTANCE_M="${CAMERA_DISTANCE_M:-9.5}"     # 固定カメラのlookat点からの距離[m]。
                                                    # 「前進方向が13m程度の幅になるように」との要望を
                                                    # 受け、地面の5m間隔目盛り線(flat_wide.xml.xacro)を
                                                    # 使って実測較正した値。較正方法: distance=10.75で
                                                    # 撮影した1フレームで、x=0/5/10の目盛り線の画面上
                                                    # ピクセルx座標を検出したところ、5mあたり約400px
                                                    # (=80px/m)、フル幅1280pxで可視幅は約16.0m。
                                                    # 距離と可視幅は比例する(elevation/azimuth固定のまま
                                                    # 距離のみ変える場合、カメラ視錐台と地面の交差形状が
                                                    # 相似的に拡大縮小するため)ため、比例定数
                                                    # k=16.0/10.75≈1.490[m/距離1単位]から
                                                    # 13m ÷ 1.490 ≈ 8.72と算出した。
CAMERA_LOOKAT_X_M="${CAMERA_LOOKAT_X_M:-2.0}"      # 固定カメラのlookat点のx方向オフセット[m]。
                                                    # 「カメラをもう少し右に」との要望を受け追加
                                                    # (mujoco_recorder_node.cppへ新規パラメータとして
                                                    # 追加、quad_mujoco.pyでlaunch引数化)。ロボットは
                                                    # x≈0付近から+x方向へ歩くため、lookat点を進行方向
                                                    # 側へずらすことで、画面内で右側に余裕ができる。
                                                    # 地面の5m間隔の目盛り線(flat_wide.xml.xacro)と
                                                    # 併用する。

# CSVロガーはSTAND送信"前"から記録を開始する(起立〜歩行移行の全区間を可視化するため)。
# そのため記録時間は「起立待ち+プランナ起動待ち+cmd_vel指令時間」の合計にする。
RECORDER_DURATION_S="$(echo "${STAND_SETTLE_S} + ${PLAN_STARTUP_S} + ${DURATION_S}" | bc)"

# ROS2の型付きパラメータ(geometry_msgs/Twist等)は"5"のような小数点なしの値を
# 整数と誤解釈してエラーになるため、常に小数表記で渡す。
DURATION_S_FLOAT="$(printf '%.3f' "${DURATION_S}")"
FORWARD_VEL_MPS_FLOAT="$(printf '%.3f' "${FORWARD_VEL_MPS}")"

set +u  # ROS2のsetup.bashは内部で未設定変数を参照するため一時的に無効化
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

export QUAD_LOGGER_SRC="${LOG_DIR}"  # quad_mujoco.pyのrecording:=trueの出力先をMPC_DOG側へ向ける

# ==== MuJoCoシミュレータ起動 ====
# world: flat_wide.xml — flat.xmlと同じ単純な直方体プリミティブ地面のまま、
#   範囲をx∈[-3,15], y∈[-5,5]に拡大したもの(external/quad-sdkへの追加ファイル)。
# camera_track_robot:=false + camera_distance: 録画カメラをロボットに追従させず
#   固定する。追従カメラだと実際に前進していても画面上は常に「その場」に見えて
#   しまい、録画だけでは前進を目視確認できない(quad_mujoco.py側にlaunch引数化
#   して追加、external/quad-sdkへの変更)。
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

# プロセスグループへSIGINT→(10秒待って)SIGKILLを送り、それでも残る子ノードを
# 名前パターンで強制killする。trap EXITでスクリプトがどう終了しても必ず呼ばれる。
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
  # プロセスグループkillだけでは一部の子ノードが終了しきらず、次の試行の
  # 記録を汚染する事象を確認済み。名前パターンでの強制killを保険として追加する。
  # 2026-08-30: このパターンにgrid_map_visualization/topic_tools relay(terrain_map)/
  # robot_state_publisher/static_transform_publisher/controller_manager spawnerが
  # 含まれておらず、試行のたびにこれらが残留し続けていたことが判明した(1セッションで
  # 121プロセスまで蓄積し、load averageが100超まで悪化。詳細はdocs参照)。以下へ追加。
  pkill -9 -f "ros2_control_node|rviz2|mujoco_recorder|contact_state_publisher_node|mujoco_estimator|body_force_estimator_node|mjcf_to_grid_map_node|grid_map_filters_demo|nmpc_controller|local_planner_node|global_body_planner_node|rviz_interface_node|robot_driver_node|grid_map_visualization|topic_tools/relay|robot_state_publisher|static_transform_publisher|controller_manager/spawner" 2>/dev/null || true
}
trap cleanup EXIT

# ==== コントローラマネージャの起動待ち ====
# 固定sleepではなく、joint_controller(ros2_control、関節へトルクを伝える
# コントローラ)が実際にactiveになるまでポーリングで待つ。固定sleepのままだと
# STANDがコントローラ未起動状態へ送られ、ロボットが一度も起立しない事象があった。
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

# ==== 記録開始:CSVロガー ====
# STAND送信"前"から記録を開始し、起立〜プランナ起動〜WALK移行の全区間を
# CSVに残す(この区間が見えないと、いつ・どの段階で転倒したか特定できない)。
python3 "${REPO_ROOT}/src/trial/quadsdk_step01_baseline.py" \
  --robot-ns "${ROBOT_NS}" \
  --duration-s "${RECORDER_DURATION_S}" \
  --csv-path "${LOG_DIR}/state_log.csv" \
  --summary-csv-path "${LOG_DIR}/trials_summary.csv" \
  --velocity-mps "${FORWARD_VEL_MPS}" &
RECORDER_PID=$!

# ==== ロボットを起立させる ====
# control/mode: 0=SAFETY(トルク0)、1=STAND(PD制御でノミナル姿勢)、2=WALK(local_plan追従)。
# 1を送らずに2だけ送ると起立前にWALKへ移行し不安定になる(公式tutorials/first-run/より)。
echo "[$(date '+%T.%3N')] Sending STAND (control/mode=1)"
ros2 topic pub --once "/${ROBOT_NS}/control/mode" std_msgs/msg/UInt8 "data: 1"

sleep "${STAND_SETTLE_S}"

# ==== プランニングスタック(local planner + NMPC)起動 ====
# reference: "twist"にしないとlocal_plannerはcmd_velを無視し、既定の"gbpl"
# (global body plannerの目標地点待ち)のまま静止し続ける
# (quad_utils/launch/planning.py: local_planner.use_twist_intput は reference=='twist' のときだけtrue)
echo "[$(date '+%T.%3N')] Launching planning stack (quad_plan.py)"
ros2 launch quad_utils quad_plan.py \
  robot_configs:='[{"name": "robot_1", "type": "go2", "controller_mode": "inverse_dynamics", "reference": "twist", "twist_input": "none"}]' \
  &
PLAN_PID=$!

sleep "${PLAN_STARTUP_S}"

# ==== ロボットをWALKモードへ切り替える ====
# これを送らないとplannerがGRF/軌道を計算していてもrobot_driverはSTANDのまま
# ノミナル姿勢へのPD制御を続け、cmd_velを送っても歩かない。
echo "[$(date '+%T.%3N')] Sending WALK (control/mode=2)"
ros2 topic pub --once "/${ROBOT_NS}/control/mode" std_msgs/msg/UInt8 "data: 2"

# ==== 一定速度指令 ====
# cmd_vel_publisher_node(quad_perf_tests)には既知の不具合(速度が実質ゼロの
# まま配信され続ける)があったため、生の`ros2 topic pub -r`を使う。
timeout "${DURATION_S_FLOAT}" ros2 topic pub -r 50 "/${ROBOT_NS}/cmd_vel" geometry_msgs/msg/Twist \
  "{linear: {x: ${FORWARD_VEL_MPS_FLOAT}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true

wait "${RECORDER_PID}"  # CSVロガーの自己終了を待つ。失敗してもtrapが後片付けする

echo "Done. CSV: ${LOG_DIR}/state_log.csv"
LATEST_MP4="$(ls -t "${LOG_DIR}"/logs/mujoco_go2_*.mp4 2>/dev/null | head -1)"
echo "Video (mp4): ${LATEST_MP4:-<not found>}"
echo "Next: bash scripts/trial/make_gif.sh \"${LATEST_MP4:-<mp4>}\" <output.gif>  # 固定カメラでの目視確認用GIFを作る"
