# ROS2経路 通信全体像 ros2/ 逐次解説シリーズの導入

## このファイルの役割(全体の中での位置づけ)

`external/Quadruped-PyMPC/ros2/`配下は、これまで読んできた`simulation.py`経路
(read_code_01〜15)とは**別の実行経路**です。`simulation.py`から一切呼ばれません。

**コードで確認した事実**:制御ロジック(`WBInterface`、`SRBDControllerInterface`、
`PeriodicGaitGenerator`など、read_code_01〜15で読んだクラス群)は、ROS2版でも
**まったく同じクラスがそのまま使われています**。ROS2版が追加しているのは
「制御ロジックを2つの独立したOSプロセスに分割し、その間をROS2のトピック通信で
つなぐ」という**通信の層だけ**です。中身(MPC、WBC、歩容生成)を新しく実装し
直してはいません。

このファイルは、ROS2版の個々のノード(`run_simulator.py`、`run_controller.py`、
`console.py`)を読む前に、**通信全体の関係図**を先にまとめるための導入文書です
(read_code_20のルールにより、個々のノードファイルではこの図を繰り返さず、
ここを見ればわかるようにする)。対象は
`external/Quadruped-PyMPC/ros2/run_simulator.py`(165行)、
`external/Quadruped-PyMPC/ros2/run_controller.py`(727行)、
`external/Quadruped-PyMPC/ros2/console.py`(415行)、および
`external/Quadruped-PyMPC/ros2/msgs_ws/src/dls2_interface/msg/*.msg`
(メッセージ型定義)です。

---

## 1. なぜ2つのプロセスに分かれているか

**コードで確認した事実**:`run_simulator.py`と`run_controller.py`は、それぞれ
独立して`python3`で起動する別々のスクリプトであり、それぞれが`rclpy.init()`→
`rclpy.spin()`という**自分自身のイベントループ**を持ちます(`run_simulator.py`
153〜161行目、`run_controller.py`715〜723行目)。2つのスクリプトの間で
Pythonオブジェクトや関数呼び出しは共有されず、**ROS2のトピック(pub/sub通信)
だけ**でやり取りします。

**設計上の解釈**:これは実機を想定した構成です。実機では「物理シミュレータ
(`run_simulator.py`)」の代わりに実際のロボットのハードウェアI/Fが同じ
トピックへ`/base_state`・`/blind_state`をpublishし、`/control_signal`・
`/trajectory_generator`をsubscribeする、という置き換えが可能になります
(実際にそのハードウェアI/F側のコードは対象リポジトリに無いため**未確認**)。
`simulation.py`経路は物理シミュレータと制御ロジックが同じPythonプロセス内で
直接関数呼び出しし合うため、この置き換えができません。

## 2. ノード構成図

```text
┌─────────────────────────────┐         ┌──────────────────────────────────┐
│   run_simulator.py           │         │   run_controller.py                │
│   (Simulator_Node)           │         │   (Quadruped_PyMPC_Node)           │
│                               │         │                                    │
│  MuJoCo物理シミュレーション   │         │  WBInterface / SRBDControllerIF   │
│  (env.step を実際に呼ぶ)     │         │  (read_code_01〜15と同じクラス)   │
│                               │         │  MuJoCoは運動学計算専用           │
│                               │         │  (env.step は呼ばない)            │
│  タイマー: 500Hz固定          │         │  ループ: /blind_state受信駆動      │
│  (SCHEDULER_FREQ)             │         │  (既定、USE_SCHEDULER=False)       │
└──────────────┬────────────────┘         └───────────────┬────────────────────┘
               │  publish                    subscribe     │
               │  /base_state (BaseState)  ───────────────▶│
               │  /blind_state (BlindState)───────────────▶│
               │                                            │
               │◀─────────────── subscribe    publish ─────┤
               │   /control_signal (ControlSignal)          │
               │◀────────────────────────────────────────── │
               │   /trajectory_generator (TrajectoryGenerator, 受信するがrun_simulator.py側で未使用)
               └────────────────────────────────────────────┘

                                              ▲
                                              │ subscribe
                                        /joy (sensor_msgs/Joy)
                                              │
                                   (ジョイスティック入力、外部ノードが
                                    publishする想定。そのpublisher側の
                                    コードは対象リポジトリに無い = 未確認)

                                        /time_debug (TimeDebug)
                                              │
                                        run_controller.pyがpublishするのみ
                                        (subscriberはコード中に無い = 未確認)
```

- `Quadruped_PyMPC_Node`が`console.py`の`Console`クラスを内部で生成し、
  別スレッド(`thread_console`)でターミナルからの対話入力を受け付けます
  (`run_controller.py` 264〜269行目)。これはROS2トピック通信ではなく、
  同一プロセス内のスレッド間でオブジェクトを直接共有する仕組みです。

## 3. トピック一覧表

| トピック名 | メッセージ型 | Publisher | Subscriber | 発行頻度/タイミング |
|---|---|---|---|---|
| `/base_state` | `BaseState` | `Simulator_Node` | `Quadruped_PyMPC_Node` | シミュレータ側タイマー、既定500Hz(`SCHEDULER_FREQ`) |
| `/blind_state` | `BlindState` | `Simulator_Node` | `Quadruped_PyMPC_Node` | 同上、`/base_state`と同じコールバック内で発行 |
| `/control_signal` | `ControlSignal` | `Quadruped_PyMPC_Node` | `Simulator_Node` | 制御ノードの制御ループ完了ごと(既定、`/blind_state`受信駆動) |
| `/trajectory_generator` | `TrajectoryGenerator` | `Quadruped_PyMPC_Node` | `Simulator_Node`(購読はしているが値を使うコードは無い) | 同上 |
| `/time_debug` | `TimeDebug` | `Quadruped_PyMPC_Node` | 無し(**未確認**、購読しているコードは対象リポジトリに無い) | 同上 |
| `joy` | `sensor_msgs/Joy` | 無し(**未確認**、publish元は対象リポジトリに無い) | `Quadruped_PyMPC_Node` | ジョイスティック入力時 |

**コードで確認した事実**:`run_simulator.py`は`/control_signal`(トルク)を
`get_torques_callback`で受け取り実際に使いますが(98〜105行目)、
`/trajectory_generator`は`get_trajectory_generator_callback`で受け取った
`joints_position`を`self.desired_joints_position`へ保存するだけで、その値を
**その後どこにも使っていません**(`compute_simulator_step_callback`は
`self.desired_tau`しか使わない)。つまり`TrajectoryGenerator`メッセージの
`joints_position`はROS2経路の中で発行はされるが、シミュレータ側で捨てられて
います。

**実装上の問題点**:`run_simulator.py`は`env.step(action=action)`でトルク
制御(`action`は各関節トルク)だけを行っており、`TrajectoryGenerator`が運ぶ
PDゲイン(`kp`/`kd`)や目標関節速度は使われません。`Quadruped_PyMPC_Node`側は
`self.tau`(最終トルク、read_code_12の`WBInterface.compute_stance_and_swing_torque`
が計算する値)をすでに`/control_signal`で送っているため、`/trajectory_generator`
は実機の低レベルPDコントローラ(このリポジトリには無い)向けの情報を運んでいる
だけで、`run_simulator.py`(MuJoCo側)では冗長という理解になります(**設計上の
解釈**)。

## 4. QoS(通信品質設定)について

**コードで確認した事実**:すべての`create_publisher`/`create_subscription`呼び出し
の第3引数は`1`です(例:`self.create_publisher(BaseState,"/base_state", 1)`)。
これはrclpyの簡易記法で、キュー長(depth)`1`の既定QoSプロファイル
(reliable、keep-last)を意味します。つまり「直近1件だけを保持し、それより
古い未消費メッセージは破棄する」設定です。ベストエフォート(取りこぼし許容)
への明示的な変更はコード中に見当たりません。

## 5. 起動時の共通処理:ROS2ワークスペースの自動sourceとプロセス再起動

`run_simulator.py`と`run_controller.py`の冒頭(共に1〜35行目)は、ほぼ同一の
以下の処理を持ちます。

```python
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
...
ros_ws = dir_path / "msgs_ws"
setup_bash = ros_ws / "install" / "setup.bash"

if not setup_bash.exists():
    print("Building the msgs first...")
    subprocess.run(["colcon", "build"], cwd=ros_ws, check=True)

if os.environ.get("QUADRUPED_PYMPC_ROS2_SOURCED") != "1":
    print("Sourcing ROS2 workspace and restarting script...")
    cmd = (
        f"source {shlex.quote(str(setup_bash))} && "
        "export QUADRUPED_PYMPC_ROS2_SOURCED=1 && "
        f"exec {shlex.quote(sys.executable)} "
        + " ".join(shlex.quote(arg) for arg in [str(Path(__file__).resolve()), *sys.argv[1:]])
    )
    os.execv("/bin/bash", ["bash", "-c", cmd])
```

- `ROS_LOCALHOST_ONLY`(環境変数、文字列`"1"`/`"0"`)：デフォルトで`"1"`に
  設定される(未設定時のみ、`setdefault`のため)。ROS2のDDS通信をローカル
  マシン内だけに限定するフェイルセーフ。ネットワーク越しの通信をしたい場合は
  事前に自分で`"0"`を設定しておく必要がある
- `ros_ws`(パス)：`ros2/msgs_ws`ディレクトリ。`dls2_interface`パッケージ
  (このファイルの4節で見るメッセージ型定義)のcolconワークスペース
- `setup_bash`が存在しない(まだ一度もビルドしていない)場合、`colcon build`を
  自動実行してメッセージ型をビルドする
- 環境変数`QUADRUPED_PYMPC_ROS2_SOURCED`が`"1"`でない場合、**同じスクリプトを
  bashサブシェル経由で自分自身に`os.execv`で再起動**する。これは、Pythonの
  `import rclpy`より前に、通常`source setup.bash`で行うシェル環境変数の設定
  (`PYTHONPATH`に`dls2_interface`のPythonバインディングを追加する、等)を
  済ませてから同じプロセスをやり直すためのテクニックである。2回目の起動では
  環境変数が`"1"`になっているため、この`if`は素通りしてそのまま続行する

**設計上の解釈**:通常のROS2ワークフローでは、ユーザーが手動で
`source install/setup.bash`してから`python3 run_controller.py`を実行するが、
このリポジトリは「`python3 run_controller.py`を直接実行するだけで、必要な
sourceとビルドを自動でやってくれる」ように、この自己再起動パターンで
利便性を高めていると考えられる。

## 6. 使われているメッセージ型の定義

対象:`external/Quadruped-PyMPC/ros2/msgs_ws/src/dls2_interface/msg/*.msg`

### `BaseState.msg`(`/base_state`)

```text
string frame_id
uint32 sequence_id
float64 timestamp
string robot_name
Pose pose            # position(3), orientation(4)
Screw velocity       # linear(3), angular(3)
Screw acceleration   # linear(3), angular(3)
bool[] stance_status
```

- `pose.position`(m)、`pose.orientation`(クォータニオン、無次元、要素順は
  `run_controller.py`の`get_base_state_callback`内のコメントより`[x,y,z,w]`)
- `velocity.linear`(m/s)、`velocity.angular`(rad/s)
- **コードで確認した事実**:`run_simulator.py`は`acceleration`と
  `stance_status`フィールドに**一度も値を代入していません**(134〜139行目)。
  この2つのフィールドはメッセージ定義上は存在するが、実際には常にゼロ初期化
  されたまま送られる未使用フィールドです

### `BlindState.msg`(`/blind_state`)

```text
string frame_id
uint32 sequence_id
float64 timestamp
string robot_name
string[] joints_name
float64[] joints_position
float64[] joints_velocity
float64[] joints_acceleration
float64[] joints_effort
float64[] joints_temperature
bool[] feet_contact
float64[] current_feet_positions
```

- `joints_position`(rad、長さ12)、`joints_velocity`(rad/s、長さ12)：
  `run_simulator.py`の141〜144行目で`self.env.mjData.qpos[7:]`/`qvel[6:]`から
  代入される
- **コードで確認した事実**:`joints_acceleration`・`joints_effort`・
  `joints_temperature`・`feet_contact`・`current_feet_positions`・
  `joints_name`は`run_simulator.py`側で**一切代入されていません**。実際に
  送信されているのは`joints_position`と`joints_velocity`だけです

### `ControlSignal.msg`(`/control_signal`)

```text
string frame_id
uint32 sequence_id
float64 timestamp
float64[] torques
uint64 signal_reconstruction_method
```

- `torques`(N·m、長さ12、順序`[FL(3), FR(3), RL(3), RR(3)]`)：
  `run_controller.py` 696〜698行目で`self.tau`(トルク制限クリップ後)から
  組み立てられる
- `signal_reconstruction_method`：`run_controller.py`側で代入コードが
  見当たらない(**未確認**、常にデフォルト値の`0`のまま送信されていると
  推測される)

### `TrajectoryGenerator.msg`(`/trajectory_generator`)

```text
string frame_id
uint32 sequence_id
float64 timestamp
Pose com_pose
Screw com_vel
Screw com_acc
float64[] joints_position
float64[] joints_velocity
float64[] joints_acceleration
float64[] joints_effort
float64[] kp
float64[] kd
float64[6] wrench
bool[] stance_legs
float64[] nominal_touch_down
float64[] touch_down
float64[] swing_period
float64[] normal_force_max
float64[] normal_force_min
```

- **コードで確認した事実**:`run_controller.py`が実際に代入しているのは
  `timestamp`・`joints_position`(rad、read_code_12の`pd_target_joints_pos`)・
  `joints_velocity`(rad/s、`pd_target_joints_vel`)・`kp`(無次元、
  `impedence_joint_position_gain`、既定`10.0`を12関節分)・`kd`(無次元、
  `impedence_joint_velocity_gain`、既定`2.0`を12関節分)だけです
  (700〜706行目)。`com_pose`/`com_vel`/`com_acc`/`joints_acceleration`/
  `joints_effort`/`wrench`/`stance_legs`/`nominal_touch_down`/`touch_down`/
  `swing_period`/`normal_force_max`/`normal_force_min`はメッセージ定義に
  存在するが、`run_controller.py`は一切代入していません。フィールド名から、
  このメッセージ型はもともとより高機能な(実機用の低レベルコントローラへ、
  接地脚判定や力の分配まで渡す)設計だったが、このROS2版では一部フィールド
  しか使っていないと考えられる(**設計上の解釈**)

### `TimeDebug.msg`(`/time_debug`)

```text
float64 time_mpc
float64 time_wbc
```

- `time_wbc`(秒)：`run_controller.py`の`self.loop_time`(`compute_control_callback`
  1回あたりの実測周期)
- `time_mpc`(秒)：`self.last_mpc_loop_time`(既定の同期MPC分岐では常に`0.0`
  のまま。後述のスレッド/プロセス版でのみ実測される)

### 未使用のメッセージ型:`FeetContactState.msg`・`Imu.msg`

**コードで確認した事実**:`dls2_interface`パッケージには`FeetContactState.msg`
(接地反力)と`Imu.msg`(IMUデータ)も定義されていますが、`run_controller.py`・
`run_simulator.py`・`console.py`のいずれからも`import`・使用されていません
(`grep`で確認、該当箇所なし)。将来の拡張、または他のリポジトリと共有している
メッセージパッケージの一部が単に未使用のまま残っていると考えられます
(**設計上の解釈**)。

## 7. `simulation.py`経路との対比

| 項目 | `simulation.py`経路 | ROS2経路 |
|---|---|---|
| プロセス数 | 1(単一Pythonプロセス) | 2(`Simulator_Node`、`Quadruped_PyMPC_Node`) |
| 物理シミュレーションと制御ロジックの関係 | 同じ`env`オブジェクトを直接共有 | 別プロセス、ROS2トピック経由でのみやり取り |
| 制御ループの駆動方式 | `for`ループ(`while`ではなく`range`ベース、read_code_01) | `Quadruped_PyMPC_Node`は既定で`/blind_state`受信駆動(`get_blind_state_callback`内で`compute_control_callback`を直接呼ぶ、`USE_SCHEDULER=False`のとき) |
| 制御ロジック本体 | `WBInterface`/`SRBDControllerInterface`を直接呼ぶ | 同じ`WBInterface`/`SRBDControllerInterface`を`Quadruped_PyMPC_Node`内で保持し、同様に呼ぶ |
| MuJoCoの役割(制御ノード側) | 物理シミュレーション(`env.step`で積分) | 運動学計算専用(`mujoco.mj_forward`のみ、`env.step`は呼ばれない。read_code_15の`InverseKinematicsNumeric`が持つ専用`QuadrupedEnv`と同じ発想) |
| 目標速度の入力元 | `QuadrupedEnv`内部の`base_vel_command_type`(既定`'human'`、キーボード) | `Quadruped_PyMPC_Node`は`joy`トピック(ジョイスティック)から`get_joy_callback`で直接`env._ref_base_lin_vel_H`等を書き換える。加えて`console.py`のターミナル対話コマンド(`ictp`)からも同じ変数をキーボードで書き換え可能 |

---

## 次に読むファイル一覧(このシリーズの続き)

1. `read_code_17_ros2_run_simulator.md` — `Simulator_Node`(物理シミュレータ側、
   構造が単純なのでこちらを先に読む)
2. `read_code_18_ros2_controller_init.md` — `Quadruped_PyMPC_Node.__init__`
   (制御ロジック側の初期化、既定OFFのスレッド/マルチプロセスMPC変種を含む)
3. `read_code_19_ros2_controller_callbacks.md` — `Quadruped_PyMPC_Node`の
   コールバック群と`compute_control_callback`(実際に毎周期動く本体)
4. `read_code_20_ros2_console.md` — `console.py`の`Console`(対話コマンドライン)

ユーザーからの指示により、これらを1つずつ確認を待たず連続して作成する。
