# ROS2 制御ノード(初期化) ros2/run_controller.py 逐次解説 (1/2: `__init__`)

## 通信上の位置づけ

```text
購読(subscribe): /base_state (BaseState), /blind_state (BlindState), joy (sensor_msgs/Joy)
配信(publish)  : /control_signal (ControlSignal), /trajectory_generator (TrajectoryGenerator),
                 /time_debug (TimeDebug)
```

トピック・メッセージ型の詳細は
[read_code_16](read_code_16_ros2_communication_overview.md)を参照してください。

## このファイルの役割(全体の中での位置づけ)

`run_controller.py`の`Quadruped_PyMPC_Node`が担当するのは、`simulation.py`
(read_code_01)の`run_simulation`ループが担っていた「歩容生成→フットホールド
計画→状態/参照値の集約→MPC→WBC→トルク計算」という制御ロジック**全体**です。
唯一の違いは、状態の入力元とトルクの出力先が、直接の関数呼び出しではなく
ROS2トピック(`/base_state`・`/blind_state`受信、`/control_signal`送信)に
なっていることです。

制御ロジック本体(`WBInterface`、`SRBDControllerInterface`)はread_code_01〜15で
読んだものと**同じクラスをそのまま使う**ため、このファイルではその内部処理を
再解説しません。ROS2版に固有の差分(通信、プロセス構成、既定OFFの
マルチスレッド/マルチプロセスMPC、リアルタイム優先度設定、対話コンソール)に
絞って解説します。

この章(1/2)では、モジュールレベルの定数群と`Quadruped_PyMPC_Node.__init__`
(130〜278行目)を扱います。コールバック本体(`compute_control_callback`等)は
次章`read_code_19_ros2_controller_callbacks.md`で扱います。

対象は`external/Quadruped-PyMPC/ros2/run_controller.py`(727行)です。冒頭の
起動処理(1〜41行目、ROS2ワークスペースの自動source)は
[read_code_16](read_code_16_ros2_communication_overview.md)の5節と同一のため
省略します。

---

## 63〜74行:プロセス優先度の設定

```python
pid = os.getpid()
print("PID: ", pid)
os.system("sudo renice -n -21 -p " + str(pid))
os.system("sudo echo -20 > /proc/" + str(pid) + "/autogroup")

# to reserve the core 4, 5 for the process, add in etc/default/grub
# GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=4-5" in etc/default/grub
# and then sudo update-grub
# and uncomment the lines below
#affinity_mask = {4, 5} 
#os.sched_setaffinity(pid, affinity_mask)

#for real time, launch it with chrt -r 99 python3 run_controller.py
```

- `renice -n -21`：このプロセスのniceレベルを`-21`(最高優先度に近い)へ変更
  しようとする。`sudo`権限が無ければ`os.system`は失敗するが、戻り値を
  チェックしていないため**エラーは無視されて処理が続行される**
  (**実装上の問題点**、失敗が静かに握りつぶされる)
- `autogroup`への書き込みも同様に、Linuxのスケジューラグループ優先度を
  上げようとする試み
- CPUコア固定(`sched_setaffinity`)はコメントアウトされており既定では
  実行されない。実行するにはGRUB設定で該当コアをOS全体のスケジューリング
  対象から外す(`isolcpus`)必要があるとコメントに明記されている
- 実時間スケジューリング(`chrt -r 99`)は、このスクリプト自体ではなく
  **起動コマンド側**で行う想定(コメントのみ、コード上の強制はない)

**設計上の解釈**:これらはすべて実機でのリアルタイム性確保を狙った
Linuxプロセス優先度のチューニングであり、シミュレーション時の挙動には
実質的な影響を与えない(MuJoCo自体はリアルタイム制約を持たないため)。

## 78〜127行:モジュールレベルの定数(既定値の一覧)

```python
USE_THREADED_MPC = False
USE_PROCESS_QUEUE_MPC = False
USE_PROCESS_SHARED_MEMORY_MPC = False
```

- 3つとも**既定`False`**。MPC計算を別スレッド/別プロセスに切り出す3種類の
  排他的な高度化オプションで、既定では**すべて無効**。既定で実際に動くのは
  この後(read_code_19)で見る、メインコールバック内の同期呼び出し
  (`else`分岐)だけです

```python
if(USE_PROCESS_SHARED_MEMORY_MPC):
    # 79〜113行目、共有メモリのレイアウト定義とヘルパー関数
    N_DBL = 75
    ...
```

- `USE_PROCESS_SHARED_MEMORY_MPC`が`False`のため、この`if`ブロック全体
  (共有メモリのペイロードレイアウト定数、`legsattr_to12`/`vec12_to_legsattr`
  ヘルパー関数)は**モジュール読み込み時に一度も実行されません**。関数定義
  すら行われないため、後段でこれらの関数名を使っているコード
  (`compute_mpc_process_shared_memory_callback`内)は、もし
  `USE_PROCESS_SHARED_MEMORY_MPC`を`True`に変えた場合にのみ意味を持ちます

```python
MPC_FREQ = 100 
RENDER_MUJOCO_VIEWER = False
RENDER_FREQ = 30

USE_SCHEDULER = False # This enable a call to the run function every tot seconds, instead of as fast as possible
SCHEDULER_FREQ = 250 # this is only valid if USE_SCHEDULER is True

USE_FIXED_LOOP_TIME = False # This is used to fix the clock time of periodic gait gen to 1/SCHEDULER_FREQ
USE_SATURATED_LOOP_TIME = True # This is used to cap the clock time of periodic gait gen to max 250Hz

USE_SMOOTH_VELOCITY = False
USE_SMOOTH_HEIGHT = True
```

| 定数 | 値(既定) | 意味 |
|---|---|---|
| `MPC_FREQ`(Hz) | `100` | 既定の同期MPC呼び出しのレート制限(read_code_19で使用) |
| `RENDER_MUJOCO_VIEWER`(`bool`) | `False` | **既定でビューアを開かない**(`Simulator_Node`とは対照的) |
| `RENDER_FREQ`(Hz) | `30` | ビューア描画レート(`RENDER_MUJOCO_VIEWER=True`の場合のみ意味を持つ) |
| `USE_SCHEDULER`(`bool`) | `False` | **既定でROS2タイマーによる定周期呼び出しを使わない**。代わりに`/blind_state`受信のたびに`compute_control_callback`を呼ぶ(read_code_19) |
| `SCHEDULER_FREQ`(Hz) | `250` | `USE_SCHEDULER=True`の場合のみ意味を持つ、既定では未使用 |
| `USE_FIXED_LOOP_TIME`(`bool`) | `False` | `True`なら`simulation_dt`を`1/SCHEDULER_FREQ`固定にする。既定は実測ループ時間を使う |
| `USE_SATURATED_LOOP_TIME`(`bool`) | `True` | 実測ループ時間が`0.005`秒を超えたら`0.005`秒に丸める(200Hz相当の下限保証) |
| `USE_SMOOTH_VELOCITY`(`bool`) | `False` | `get_base_state_callback`内の速度の指数移動平均平滑化。既定OFF |
| `USE_SMOOTH_HEIGHT`(`bool`) | `True` | 同、base高さ(z)だけは既定で平滑化**する** |

**実装上の問題点**:`USE_SCHEDULER`という同名の変数が`run_simulator.py`
(read_code_17)にも存在するが、値も意味も異なる
(`run_simulator.py`側は`True`固定かつファイル内未参照で死んでいる。
こちらの`run_controller.py`側は`False`固定かつ実際に分岐を切り替える)。
同名の変数がファイルをまたいで異なる意味を持つため、両方を同時に読む際に
混同しやすい。

---

## 130〜278行:`Quadruped_PyMPC_Node.__init__`

この関数の役割:ROS2の購読・配信・タイマーを登録し、`WBInterface`と
`SRBDControllerInterface`を生成して、制御ループが動き出せる状態を整える。

### 134〜143行:購読・配信・タイマーの登録

```python
self.subscription_base_state = self.create_subscription(BaseState,"/base_state", self.get_base_state_callback, 1)
self.subscription_blind_state = self.create_subscription(BlindState,"/blind_state", self.get_blind_state_callback, 1)
self.subscription_joy = self.create_subscription(Joy,"joy", self.get_joy_callback, 1)
self.publisher_control_signal = self.create_publisher(ControlSignal,"/control_signal", 1)
self.publisher_trajectory_generator = self.create_publisher(TrajectoryGenerator,"/trajectory_generator", 1)
self.publisher_time_debug = self.create_publisher(TimeDebug,"/time_debug", 1)
if(USE_SCHEDULER):
    self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_control_callback)
```

- `USE_SCHEDULER=False`(既定)のため、**このノードにはROS2タイマーが
  1つも作られません**。制御ループの駆動は、後述の
  `get_blind_state_callback`末尾での直接呼び出しに完全に依存します
  (read_code_19で詳説)

### 145〜165行:安全フラグと状態保持変数の初期化

```python
self.first_message_base_arrived = False
self.first_message_joints_arrived = False 

self.loop_time = 0.002
self.last_start_time = None
self.last_mpc_loop_time = 0.0

self.position = np.zeros(3)
self.orientation = np.zeros(4)
self.linear_velocity = np.zeros(3)
self.angular_velocity = np.zeros(3)
self.joint_positions = np.zeros(12)
self.joint_velocities = np.zeros(12)
self.feet_contact = np.zeros(4)

self.impedence_joint_position_gain = np.ones(12)*cfg.simulation_params['impedence_joint_position_gain']
self.impedence_joint_velocity_gain = np.ones(12)*cfg.simulation_params['impedence_joint_velocity_gain']
```

- `first_message_base_arrived`/`first_message_joints_arrived`(`bool`)：
  トピック受信済みかどうかのフラグ。初期値`False`。制御ループ本体
  (read_code_19の`compute_control_callback`冒頭)で、両方`True`になるまで
  早期`return`する安全策に使われる
- `self.loop_time`(秒)：`0.002`で初期化(実測前の仮の値)
- `self.position`/`self.orientation`/`self.linear_velocity`/`self.angular_velocity`：
  `/base_state`から`get_base_state_callback`で更新される状態のキャッシュ
  (m、無次元クォータニオン、m/s、rad/s)
- `self.joint_positions`/`self.joint_velocities`/`self.feet_contact`：
  `/blind_state`から`get_blind_state_callback`で更新されるキャッシュ
  (rad、rad/s、`bool`相当の`float`配列)
- `self.impedence_joint_position_gain`(無次元、長さ12)：
  `cfg.simulation_params['impedence_joint_position_gain']`(既定`10.0`)を
  12関節分に複製
- `self.impedence_joint_velocity_gain`(無次元、長さ12)：
  `cfg.simulation_params['impedence_joint_velocity_gain']`(既定`2.0`)を
  12関節分に複製
- これら2つのゲインは、後段(read_code_19)で`/trajectory_generator`の
  `kp`/`kd`フィールドとしてそのままpublishされる(実機の低レベルPD制御用)

### 169〜195行:MuJoco環境の構築(運動学計算専用)

```python
self.env = QuadrupedEnv(
    robot=cfg.robot,
    scene=cfg.simulation_params['scene'],
    sim_dt=cfg.simulation_params['dt'],
    base_vel_command_type="human"
)
self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
```

- `robot`：`cfg.robot`(既定`'go2'`)
- `scene`：`cfg.simulation_params['scene']`(既定`'flat'`)
- `sim_dt`(秒)：`cfg.simulation_params['dt']`(既定`0.002`)。ただし、
  read_code_19で確認する通り、この`env`に対しては`env.step`
  (物理積分)が一度も呼ばれず、`mujoco.mj_forward`(運動学の再計算のみ)
  だけが使われる。read_code_15の`InverseKinematicsNumeric`が持つ専用
  `QuadrupedEnv`と同じ「動かない計算機」としての使い方
- `base_vel_command_type="human"`：`run_simulator.py`と同様、この`env`
  自体の速度指令入力は使われないが、`_ref_base_lin_vel_H`等の変数は
  `get_joy_callback`と`console.py`から直接書き換えられる形で利用される

```python
self.feet_traj_geom_ids, self.feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
self.legs_order = ["FL", "FR", "RL", "RR"]
self.env.reset(random=False)
```

- `self.feet_traj_geom_ids`/`self.feet_GRF_geom_ids`：デバッグ描画
  (足先軌道やGRFベクトルのビジュアライズ)用のMuJoCo geom ID保持変数だが、
  `grep`で確認したところ**この後どこにも代入・使用されていません**
  (**実装上の問題点**、`RENDER_MUJOCO_VIEWER=False`が既定であることと
  整合的だが、`True`にしても機能しない未完成のデバッグ変数と考えられる)
- `self.env.reset(random=False)`：地形・初期姿勢を固定でリセット
  (`random=True`のようなランダム化オプションはここでは使われない)

```python
self.stand_up_and_down_actions = LegsAttr(*[np.zeros((1, int(self.env.mjModel.nu/4))) for _ in range(4)])
keyframe_id = mujoco.mj_name2id(self.env.mjModel, mujoco.mjtObj.mjOBJ_KEY, "down")
goDown_qpos = self.env.mjModel.key_qpos[keyframe_id]
self.stand_up_and_down_actions.FL = goDown_qpos[7:10]
self.stand_up_and_down_actions.FR = goDown_qpos[10:13]
self.stand_up_and_down_actions.RL = goDown_qpos[13:16]
self.stand_up_and_down_actions.RR = goDown_qpos[16:19]
```

- MuJoCoのXMLモデルに定義された`"down"`という名前のキーフレーム
  (伏せ姿勢の関節角度セット)から、初期の`stand_up_and_down_actions`
  (rad、`LegsAttr`)を取得する。これは`console.py`の`goUp`/`goDown`
  コマンド(read_code_20)が使う目標姿勢の初期値になる

### 198〜204行:制御ロジックの生成

```python
from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
from quadruped_pympc.interfaces.wb_interface import WBInterface

self.wb_interface = WBInterface(initial_feet_pos = self.env.feet_pos(frame='world'), legs_order = self.legs_order)
self.srbd_controller_interface = SRBDControllerInterface()
```

- `self.wb_interface`：read_code_06〜15で読んだ`WBInterface`そのもの。
  内部で`PeriodicGaitGenerator`、`FootholdReferenceGenerator`、
  `SwingTrajectoryController`、`TerrainEstimator`、`VelocityModulator`、
  `EarlyStanceDetector`、`InverseKinematicsNumeric`を保持する
  (これらの初期化処理自体はread_code_06で解説済みなので繰り返さない)
- `self.srbd_controller_interface`：read_code_07で読んだ
  `SRBDControllerInterface`そのもの
- **実装上の問題点**:`SRBDBatchedControllerInterface`が`import`されて
  いるが、`grep`で確認したところ**このファイル中で一度もインスタンス化・
  参照されていません**。死んだimportです

### 206〜220行:MPC⇄WBC間の共有変数の初期化

```python
self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_joints_pos = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_joints_vel = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_joints_acc = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_predicted_state = np.zeros(12)

self.best_sample_freq = self.wb_interface.pgg.step_freq
self.state_current = None
self.ref_state = None
self.contact_sequence = None
self.inertia = None
self.optimize_swing = None
```

- `self.nmpc_GRFs`(N)・`self.nmpc_footholds`(m)・`self.nmpc_joints_pos`(rad)・
  `self.nmpc_joints_vel`(rad/s)・`self.nmpc_joints_acc`(rad/s²)：
  MPCの最新解を保持する`self`変数として`__init__`時点でゼロ初期化。
  `simulation.py`経路には無い設計で、これは`USE_THREADED_MPC`等の
  非同期MPC変種で「MPCスレッドが書き込み、メインスレッドが読み出す」
  ための共有状態として使われる(既定の同期分岐でもこれらの`self`変数は
  そのまま使われる、read_code_19)
- `self.best_sample_freq`：`self.wb_interface.pgg.step_freq`
  (既定トロットなら`1.4`Hz、read_code_02)で初期化
- `self.state_current`/`self.ref_state`/`self.contact_sequence`/
  `self.inertia`/`self.optimize_swing`：いずれも`None`初期化。
  `compute_control_callback`が毎周期これらを更新する

### 222〜230行:トルクベクトルとトルク上限

```python
self.tau = LegsAttr(*[np.zeros((self.env.mjModel.nv, 1)) for _ in range(4)])
tau_soft_limits_scalar = 0.9
self.tau_limits = LegsAttr(
    FL=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.FL]*tau_soft_limits_scalar,
    FR=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.FR]*tau_soft_limits_scalar,
    RL=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.RL]*tau_soft_limits_scalar,
    RR=self.env.mjModel.actuator_ctrlrange[self.env.legs_tau_idx.RR]*tau_soft_limits_scalar)
```

- `tau_soft_limits_scalar`(無次元)：`0.9`固定。MuJoCoモデルが定義する
  アクチュエータの最大トルク(`actuator_ctrlrange`、N·m)の90%を実際の
  ソフトリミットとして使う。`simulation.py`(read_code_01)にも同様の
  トルクリミット処理があったが、そちらのスケール係数が`0.9`と同一かどうかは
  この章では未確認(read_code_01時点の記録では係数の値自体は明記していない)

### 232〜233行:起動直後は全脚接地固定

```python
# Let's start in FULL STANCE in any case
self.wb_interface.pgg.gait_type = 7 
```

- 歩容タイプを`7`(`GaitType.FULL_STANCE`、read_code_02で確認済みの
  enum値)に強制設定。起動直後は必ず「その場で全脚接地」の状態から始まり、
  `console.py`の`stw`コマンド(read_code_20)を打つまで歩き出さない
  安全策になっている

### 235〜269行:既定OFFのMPC並列化と、既定ONの対話コンソール

```python
if(USE_THREADED_MPC):
    ...
if(USE_PROCESS_QUEUE_MPC):
    ...
if(USE_PROCESS_SHARED_MEMORY_MPC):
    ...
```

- 3つとも既定`False`のため**すべてスキップされる**。もし有効化した場合の
  それぞれの役割(概要のみ、既定OFFのため深追いしない):
  - `USE_THREADED_MPC`：`compute_mpc_thread_callback`をデーモンスレッドで
    起動し、`self.state_current`等の`self`変数経由でメインループと
    非同期にMPCを回す
  - `USE_PROCESS_QUEUE_MPC`：MPC計算を別プロセスに切り出し、
    `multiprocessing.Queue`(各`maxsize=1`、つまり常に最新の1件だけ)で
    入出力をやり取りする
  - `USE_PROCESS_SHARED_MEMORY_MPC`：同じく別プロセスだが、出力側は
    `multiprocessing.shared_memory`+seqlock(排他ロック無しで一貫性を
    保つ手法)で、キューよりさらに低レイテンシな受け渡しを狙う設計

```python
from console import Console
self.console = Console(controller_node=self)
thread_console = threading.Thread(target=self.console.interactive_command_line)
thread_console.daemon = True
thread_console.start()
```

- こちらは既定で**必ず実行される**。`console.py`(read_code_20)の
  `Console`インスタンスを生成し、標準入力からの対話コマンドループを
  別デーモンスレッドで開始する。`self`(=`Quadruped_PyMPC_Node`インスタンス
  自体)を`controller_node`として渡すため、`Console`は
  `self.wb_interface`・`self.env`など、このノードの内部状態を直接
  書き換えられる

### 272〜277行:実機用ゲイン切り替え(コメントアウト、既定では無効)

```python
# Init for real robot and simulation gain, since real robot needs different values
#    self.wb_interface.stc.position_gain_fb = 100
#    self.wb_interface.stc.velocity_gain_fb = 10
#    self.wb_interface.stc.use_feedback_linearization = False
#    self.wb_interface.stc.use_friction_compensation = False
```

- コメントアウトされたまま残っている、実機投入時に手動で有効化することを
  想定したゲイン上書きコード。読み方から、シミュレーションと実機で
  スイング制御(read_code_13の`SwingTrajectoryController`)のフィードバック
  ゲインや、フィードバック線形化・摩擦補償の有無を変える運用を想定している
  ことがわかる(**設計上の解釈**、実際に実機側で使われている値は
  対象リポジトリに無いため**未確認**)

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `renice`/`autogroup`の`sudo`コマンドが失敗しても無視される
     (エラーハンドリング無し)
  2. `run_simulator.py`と同名だが意味の違う`USE_SCHEDULER`定数が存在し
     紛らわしい
  3. `self.feet_traj_geom_ids`/`self.feet_GRF_geom_ids`はデバッグ描画用に
     宣言されるが、どこにも使われていない
  4. `SRBDBatchedControllerInterface`がimportされるが、一度もインスタンス化
     されない(死んだimport)
- 確認できた重要な事実:
  - `WBInterface`と`SRBDControllerInterface`は、`simulation.py`経路
    (read_code_01〜15)と**まったく同じクラス**がそのまま使われる
  - 既定では`USE_SCHEDULER=False`、`USE_THREADED_MPC=False`、
    `USE_PROCESS_QUEUE_MPC=False`、`USE_PROCESS_SHARED_MEMORY_MPC=False`、
    `RENDER_MUJOCO_VIEWER=False`。制御ロジックの並列化やビューア表示は
    すべて既定でオフで、シンプルな単一スレッド同期処理が既定経路
  - 起動直後は歩容が強制的に`FULL_STANCE`(全脚接地)になり、`console.py`
    経由で明示的に歩行開始コマンドを打つまで歩き出さない
  - 制御ノード側の`QuadrupedEnv`はread_code_15の逆運動学専用環境と同様、
    物理積分ではなく運動学計算専用として使われる(次章で`mj_forward`の
    呼び出しを確認する)
- 次は`Quadruped_PyMPC_Node`のコールバック本体
  (`read_code_19_ros2_controller_callbacks.md`)に進みます。
