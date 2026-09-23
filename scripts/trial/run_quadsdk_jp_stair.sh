#!/usr/bin/env bash
# 日本家屋の階段シナリオ。起動順は run_quadsdk_gap_1m.sh と同じ。
# あのファイルは溝と平地の後方互換のため変更しない。時間と初期位置はこちらだけで決める。
#
# 胴体の初期 x は、最初の蹴上の 2 m 手前。
# 速度指令の終わりは、下りきって床の高さが 0 に戻った位置の 2 m 先。
# cmd_vel を送り続ける時間は、経路長 / 指令速度 × TIME_MARGIN。
# 平地の実測では指令の約 8 割しか進まないので、余裕は 2 倍。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# 8月29日の実行ファイルは Pinocchio 4.0 と MuJoCo 3.4 を要求する。
SONAME_DIR="${REPO_ROOT}/artifacts/soname"
if [ ! -f "${SONAME_DIR}/libpinocchio_parsers.so.4.0.0" ] || [ ! -f "${SONAME_DIR}/libmujoco.so.3.4.0" ]; then
  echo "ERROR: ${SONAME_DIR} に Pinocchio 4.0 と MuJoCo 3.4 の実体がありません" >&2
  exit 1
fi
unset VIRTUAL_ENV
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
hash -r 2>/dev/null || true
export LD_LIBRARY_PATH="${SONAME_DIR}:/usr/local/lib:${LD_LIBRARY_PATH:-}"

ROBOT_NS="robot_1"
GAP_WORLD="${GAP_WORLD:-jp_stair_10.xml}"
GAP_TAG="${GAP_TAG:-step18_stair}"

LOG_DIR="${REPO_ROOT}/artifacts/logs/quadsdk_${GAP_TAG}"
mkdir -p "${LOG_DIR}"
FORWARD_VEL_MPS="${FORWARD_VEL_MPS:-0.10}"          # 前進速度指令[m/s]
# gen_quadsdk_stair_world.py と同じ寸法。最初の蹴上は x=3。
# 下りは蹴上が N_STEPS-1 段で、その先で床の z が 0 に戻る。
FIRST_RISER_X_M="${FIRST_RISER_X_M:-3.0}"
APPROACH_M="${APPROACH_M:-2.0}"
TREAD_M="${TREAD_M:-0.24}"
N_STEPS="${N_STEPS:-10}"
LANDING_M="${LANDING_M:-2.0}"
EXIT_M="${EXIT_M:-2.0}"
TIME_MARGIN="${TIME_MARGIN:-2.0}"
COURSE_M="$(echo "${APPROACH_M} + ${N_STEPS} * ${TREAD_M} + ${LANDING_M} + (${N_STEPS} - 1) * ${TREAD_M} + ${EXIT_M}" | bc -l)"
# 距離 / 速度 × 余裕。環境変数 DURATION_S を渡したときだけ、その秒数を使う。
if [ -z "${DURATION_S:-}" ]; then
  DURATION_S="$(echo "${COURSE_M} / ${FORWARD_VEL_MPS} * ${TIME_MARGIN}" | bc -l)"
fi
STAND_SETTLE_S="${STAND_SETTLE_S:-8}"              # STAND送信後、プランナ起動前に待つ時間[s]
PLAN_STARTUP_S="${PLAN_STARTUP_S:-3}"              # プランナ起動後、WALK送信前に待つ時間[s]
JOINT_CONTROLLER_WAIT_TIMEOUT_S="${JOINT_CONTROLLER_WAIT_TIMEOUT_S:-40}"  # joint_controller起動待ちの上限[s]
# 未指定なら、最初の蹴上の APPROACH_M 手前。既定では x=1。
if [ -z "${SPAWN_X_M:-}" ]; then
  SPAWN_X_M="$(echo "${FIRST_RISER_X_M} - ${APPROACH_M}" | bc -l)"
fi
# 横から、機体の後方約 2 m と停止位置の先まで入る。
# distance 10.75 で可視幅約 16 m (k≈1.49)。距離 20 で可視幅約 30 m、lookat 13 で左端は約 x=-2。
CAMERA_DISTANCE_M="${CAMERA_DISTANCE_M:-20.0}"
CAMERA_LOOKAT_X_M="${CAMERA_LOOKAT_X_M:-13.0}"

# CSVロガーはSTAND送信"前"から記録を開始する(起立〜歩行移行の全区間を可視化するため)。
# そのため記録時間は「起立待ち+プランナ起動待ち+cmd_vel指令時間」の合計にする。
HOLD_S="${HOLD_S:-8}"  # 速度を出し終えたあと、零速度を送り続ける秒。
RECORDER_DURATION_S="$(echo "${STAND_SETTLE_S} + ${PLAN_STARTUP_S} + ${DURATION_S} + ${HOLD_S}" | bc)"
echo "[stair] spawn_x=${SPAWN_X_M} first_riser_x=${FIRST_RISER_X_M} approach_m=${APPROACH_M}"
echo "[stair] course_m=${COURSE_M} vel=${FORWARD_VEL_MPS} margin=${TIME_MARGIN} duration_s=${DURATION_S} hold_s=${HOLD_S}"

# ROS2の型付きパラメータ(geometry_msgs/Twist等)は"5"のような小数点なしの値を
# 整数と誤解釈してエラーになるため、常に小数表記で渡す。
DURATION_S_FLOAT="$(printf '%.3f' "${DURATION_S}")"
FORWARD_VEL_MPS_FLOAT="$(printf '%.3f' "${FORWARD_VEL_MPS}")"

set +u  # ROS2のsetup.bashは内部で未設定変数を参照するため一時的に無効化
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
if [ -n "${QUADSDK_OVERLAY_SETUP:-}" ]; then
  source "${QUADSDK_OVERLAY_SETUP}"
fi
set -u

export QUAD_LOGGER_SRC="${LOG_DIR}"  # quad_mujoco.pyのrecording:=trueの出力先をMPC_DOG側へ向ける

# 地形フィルタは install overlay の共通 filter_chain.yaml を使う。
# 平地、穴、階段の間で、このスクリプトはフィルタ値を変更しない。

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
  robot_configs:="[{\"name\": \"robot_1\", \"type\": \"go2\", \"controller\": \"inverse_dynamics\", \"init_pose\": \"-x ${SPAWN_X_M} -y 0.0 -z 0.5\"}]" \
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
  # mujoco_recorder は destructor で mp4 を finalize する(quad_mujoco.py の
  # コメント: "Ctrl+C produces a finalized mp4")。SIGINT なら moov を書いて
  # 有効な mp4 になるが、下の保険 pkill -9 が先に届くと moov 未書き込みで
  # 壊れる(短い試行で頻発)。保険 kill の前に、名前指定で SIGINT を送り、
  # プロセス消滅または最大8sまで待って finalize させる。
  pkill -INT -f "mujoco_recorder" 2>/dev/null || true
  for _ in $(seq 1 8); do pgrep -f "mujoco_recorder" >/dev/null 2>&1 || break; sleep 1; done
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
if [ -n "${DESCENT_VEL_MPS:-}" ]; then
  DESCENT_SWITCH_X_M="${DESCENT_SWITCH_X_M:?DESCENT_SWITCH_X_M is required with DESCENT_VEL_MPS}"
  echo "[$(date '+%T.%3N')] Descent speed ${DESCENT_VEL_MPS} m/s after x=${DESCENT_SWITCH_X_M} m"
  timeout "${DURATION_S_FLOAT}" python3 - "${ROBOT_NS}" "${FORWARD_VEL_MPS_FLOAT}" "${DESCENT_VEL_MPS}" "${DESCENT_SWITCH_X_M}" "${DURATION_S_FLOAT}" <<'PY' || true
import sys
import rclpy
from geometry_msgs.msg import Twist
from quad_msgs.msg import RobotState
from rclpy.node import Node

robot_ns, ascent_vel, descent_vel, switch_x, duration_s = sys.argv[1:]
ascent_vel = float(ascent_vel)
descent_vel = float(descent_vel)
switch_x = float(switch_x)
duration_s = float(duration_s)

class SplitSpeed(Node):
    def __init__(self):
        super().__init__("stair_split_speed")
        self.x = None
        self.switched = False
        self.pub = self.create_publisher(Twist, f"/{robot_ns}/cmd_vel", 10)
        self.create_subscription(RobotState, f"/{robot_ns}/state/ground_truth", self.on_state, 10)
        self.create_timer(0.02, self.on_timer)
        self.create_timer(duration_s, self.finish)

    def on_state(self, msg):
        self.x = msg.body.pose.position.x

    def on_timer(self):
        speed = ascent_vel
        if self.x is not None and self.x >= switch_x:
            speed = descent_vel
            if not self.switched:
                self.switched = True
                print(f"DESCENT SPEED x={self.x:.3f} vel={speed:.3f}", flush=True)
        twist = Twist()
        twist.linear.x = speed
        self.pub.publish(twist)

    def finish(self):
        rclpy.shutdown()

rclpy.init()
node = SplitSpeed()
rclpy.spin(node)
PY
else
  timeout "${DURATION_S_FLOAT}" ros2 topic pub -r 50 "/${ROBOT_NS}/cmd_vel" geometry_msgs/msg/Twist \
    "{linear: {x: ${FORWARD_VEL_MPS_FLOAT}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
fi

if [ "$(echo "${HOLD_S} > 0" | bc)" = "1" ]; then
  HOLD_S_FLOAT="$(printf '%.3f' "${HOLD_S}")"
  echo "[$(date '+%T.%3N')] Holding cmd_vel at zero for ${HOLD_S_FLOAT}s"
  timeout "${HOLD_S_FLOAT}" ros2 topic pub -r 50 "/${ROBOT_NS}/cmd_vel" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
fi

wait "${RECORDER_PID}"  # CSVロガーの自己終了を待つ。失敗してもtrapが後片付けする

echo "Done. CSV: ${LOG_DIR}/state_log.csv"
LATEST_MP4="$(ls -t "${LOG_DIR}"/logs/mujoco_go2_*.mp4 2>/dev/null | head -1)"
echo "Video (mp4): ${LATEST_MP4:-<not found>}"
echo "Next: bash scripts/trial/make_gif.sh \"${LATEST_MP4:-<mp4>}\" <output.gif>  # 固定カメラでの目視確認用GIFを作る"
