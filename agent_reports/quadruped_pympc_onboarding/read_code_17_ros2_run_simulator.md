# ROS2 物理シミュレータ ros2/run_simulator.py 逐次解説

## 通信上の位置づけ

```text
購読(subscribe): /control_signal (ControlSignal), /trajectory_generator (TrajectoryGenerator)
Publish   : /base_state (BaseState), /blind_state (BlindState)
```

トピック・メッセージ型の詳細、頻度、フィールドの使用状況は
[read_code_16](read_code_16_ros2_communication_overview.md)にまとめてあるので、
このファイルではその通信内容を実際に生成・消費するコードだけを見ます。

## このファイルの役割(全体の中での位置づけ)

`run_simulator.py`が担当するのは、「**MuJoCo物理シミュレーションを実際に1歩ずつ
進め(`env.step`)、その結果の状態をROS2トピックへpublishし、ROS2トピック経由で
受け取ったトルクをそのまま印加する**」ことだけです。

- 歩容生成・フットホールド計画・MPC・WBCといった制御ロジックは**一切持ちません**。
  それらはすべて`run_controller.py`(read_code_18・19)側の責務です。
- `simulation.py`(read_code_01)の`run_simulation`が持っていた「観測→制御ロジック
  →トルク計算→`env.step`」という一連の処理のうち、この`Simulator_Node`が担うのは
  「観測のpublish」と「受け取ったトルクでの`env.step`」だけです。制御ロジック部分は
  ネットワークの向こう側(`Quadruped_PyMPC_Node`)に切り出されています。

対象は`external/Quadruped-PyMPC/ros2/run_simulator.py`(165行)です。冒頭1〜35行目
の起動処理(ROS2ワークスペースの自動sourceと自己再起動)は
[read_code_16](read_code_16_ros2_communication_overview.md)の5節で説明済みなので
ここでは省略します。

---

## 56〜58行:モジュールレベル定数

```python
USE_SCHEDULER = True # Use the scheduler to compute the control signal
SCHEDULER_FREQ = 500 # Frequency of the scheduler
RENDER_FREQ = 30
```

- `USE_SCHEDULER`(`bool`)：`True`固定。実際にはこのファイル内で参照される
  箇所は無く(**実装上の問題点**、`run_controller.py`側にある同名の変数
  `USE_SCHEDULER`と紛らわしいが、こちらの`Simulator_Node`は常にタイマー駆動
  であり、この変数自体は死んでいる)
- `SCHEDULER_FREQ`(Hz)：`500`固定。MuJoCo物理シミュレーションを進める
  タイマーの周波数として使われる(69行目)
- `RENDER_FREQ`(Hz)：`30`固定。ビューア描画の上限頻度

---

## 62〜96行:`Simulator_Node.__init__`

この関数の役割:購読・配信するトピックとタイマーを登録し、MuJoCo環境を
物理シミュレーション用に構築する。

```python
class Simulator_Node(Node):
    def __init__(self):
        super().__init__('Simulator_Node')

        self.publisher_base_state = self.create_publisher(BaseState,"/base_state", 1)
        self.publisher_blind_state = self.create_publisher(BlindState,"/blind_state", 1)
        self.subscriber_control_signal = self.create_subscription(ControlSignal,"/control_signal", self.get_torques_callback, 1)
        self.subscriber_trajectory_generator = self.create_subscription(TrajectoryGenerator,"/trajectory_generator", self.get_trajectory_generator_callback, 1)

        self.timer = self.create_timer(1.0/SCHEDULER_FREQ, self.compute_simulator_step_callback)
```

- `self.timer`：`1.0/SCHEDULER_FREQ`秒(既定`1/500=0.002`秒)ごとに
  `compute_simulator_step_callback`を呼ぶROS2タイマー。**物理シミュレーションの
  進行は、このタイマーの実時間(壁時計)に従っており、`/control_signal`の受信
  頻度には従いません**(受信したトルクをその都度使うだけの、非同期な購読)

```python
        self.env = QuadrupedEnv(
            robot=cfg.robot,
            scene=cfg.simulation_params['scene'],
            sim_dt=1.0/SCHEDULER_FREQ,
            base_vel_command_type="human"
        )
        self.env.mjModel.opt.gravity[2] = -cfg.gravity_constant
        self.env.reset(random=False)
```

- `robot`：`cfg.robot`。既定`'go2'`
- `scene`：`cfg.simulation_params['scene']`。既定`'flat'`
- `sim_dt`(秒)：`1.0/SCHEDULER_FREQ`。既定`1/500=0.002`秒。MuJoCoの物理積分
  タイムステップが、タイマー周期と**同じ値に固定**されている(タイマーが遅延
  しても`sim_dt`自体は`0.002`のまま変わらない、固定ステップ積分)
- `base_vel_command_type`：`"human"`固定。ただし`Simulator_Node`は`env`の
  速度指令(`_ref_base_lin_vel_H`等)を一度も読み書きしないため、この設定は
  この`Simulator_Node`内では意味を持たない(速度指令を実際に書き換えるのは
  `run_controller.py`側の`joy`コールバックと`console.py`)
- `self.env.mjModel.opt.gravity[2]`(m/s²)：`-cfg.gravity_constant`
  (既定`-9.81`)で明示的に上書き

```python
        self.last_render_time = time.time()
        self.env.render()  
        self.env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        self.env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
```

- 起動直後に必ず1回`env.render()`してビューアを開く(`run_controller.py`側は
  フラグ`RENDER_MUJOCO_VIEWER`で切替可能だが、`Simulator_Node`側にはそのような
  切替フラグが無く、**常にビューアが開く**)
- 影(`mjRND_SHADOW`)と反射(`mjRND_REFLECTION`)の描画を無効化(描画負荷軽減)

```python
        self.desired_tau = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])
        self.desired_joints_position = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])
        self.desired_joints_velocity = LegsAttr(*[np.zeros((int(self.env.mjModel.nu/4), 1)) for _ in range(4)])
```

- `self.desired_tau`(N·m、`LegsAttr`、各脚`(3,1)`)：`/control_signal`から
  受け取ったトルクの保持先。初期値ゼロ
- `self.desired_joints_position`/`self.desired_joints_velocity`(rad/rad·s⁻¹)：
  `/trajectory_generator`から受け取った値の保持先。初期値ゼロ。**実装上の
  問題点**(read_code_16で既出):この2つは代入されるだけで、後段の
  `compute_simulator_step_callback`では一度も読まれません

---

## 98〜105行:`get_torques_callback`

この関数の役割:`/control_signal`メッセージからトルクを取り出し、4脚分に
分割して保持する。

```python
def get_torques_callback(self, msg):
    torques = np.array(msg.torques)
    self.desired_tau.FL = torques[0:3]
    self.desired_tau.FR = torques[3:6]
    self.desired_tau.RL = torques[6:9]
    self.desired_tau.RR = torques[9:12]
```

- `msg.torques`(N·m、長さ12)：`ControlSignal`メッセージのフィールド。
  順序は`[FL(3), FR(3), RL(3), RR(3)]`(`run_controller.py`側の組み立て順、
  [read_code_16](read_code_16_ros2_communication_overview.md)で確認済み)
- 受信のたびに即座に`self.desired_tau`を上書きするだけで、他の処理は行わない
  (このコールバック自体には周波数制限が無く、メッセージが届くたび毎回呼ばれる)

---

## 110〜117行:`get_trajectory_generator_callback`

この関数の役割:`/trajectory_generator`メッセージから目標関節角度を取り出し、
4脚分に分割して保持する(が、後段では使われない)。

```python
def get_trajectory_generator_callback(self, msg):
    joints_position = np.array(msg.joints_position)
    self.desired_joints_position.FL = joints_position[0:3]
    self.desired_joints_position.FR = joints_position[3:6]
    self.desired_joints_position.RL = joints_position[6:9]
    self.desired_joints_position.RR = joints_position[9:12]
```

- `get_torques_callback`と同じ構造で、`msg.joints_position`(rad、長さ12)を
  4脚に分割して`self.desired_joints_position`へ保持する
- **実装上の問題点**:このコールバックが受け取った値は`compute_simulator_step_callback`
  から一度も参照されません。`MjModel`側のトルク制御(`env.step(action=tau)`)
  だけで完結しており、位置制御(PD制御)は`Simulator_Node`内には実装されて
  いません。実機であればこの`joints_position`/`kp`/`kd`を使ってPD制御する
  低レベルコントローラが別途必要ですが、MuJoCoシミュレータ側のこのノードは
  その代わりを果たしていません

---

## 121〜150行:`compute_simulator_step_callback`

この関数の役割:MuJoCoを1ステップ進め、その結果の状態を`/base_state`・
`/blind_state`としてpublishする。タイマーによって500Hz(既定)で呼ばれる。

### 123〜128行:トルクの適用とステップ実行

```python
action = np.zeros(self.env.mjModel.nu)
action[self.env.legs_tau_idx.FL] = self.desired_tau.FL.reshape(-1)
action[self.env.legs_tau_idx.FR] = self.desired_tau.FR.reshape(-1)
action[self.env.legs_tau_idx.RL] = self.desired_tau.RL.reshape(-1)
action[self.env.legs_tau_idx.RR] = self.desired_tau.RR.reshape(-1)
self.env.step(action=action)
```

- `action`(N·m、shape`(nu,)`、`nu=12`)：直近に`get_torques_callback`で
  受信した`self.desired_tau`を、MuJoCoのアクチュエータ順序
  (`legs_tau_idx`)に並べ替えて格納
- `self.env.step(action=action)`：`simulation.py`(read_code_01)と同じ
  `QuadrupedEnv.step`。物理シミュレーションを`sim_dt`(既定`0.002`秒)だけ
  進める、この`Simulator_Node`内で**唯一の物理積分呼び出し**

**実装上の問題点**:`/control_signal`の受信とタイマーの発火は非同期です。
タイマーが500Hzで発火する一方、`/control_signal`は`run_controller.py`側の
制御周期(既定、`/blind_state`受信駆動、後述read_code_19)で送られてくるため、
必ずしも1タイマー周期ごとに新しいトルクが届くとは限りません。届いていなければ
`self.desired_tau`は**前回受信した値のまま**使われ続けます(明示的な
タイムアウト処理やゼロ埋めは無い)。

### 130〜144行:状態のpublish

```python
base_lin_vel = self.env.base_lin_vel(frame='world')
base_ang_vel = self.env.base_ang_vel(frame='base')
base_pos = self.env.base_pos

base_state_msg = BaseState()
base_state_msg.pose.position = base_pos
base_state_msg.pose.orientation = np.roll(self.env.mjData.qpos[3:7],-1)
base_state_msg.velocity.linear = base_lin_vel
base_state_msg.velocity.angular = base_ang_vel
self.publisher_base_state.publish(base_state_msg)

blind_state_msg = BlindState()
blind_state_msg.joints_position = self.env.mjData.qpos[7:].tolist()
blind_state_msg.joints_velocity = self.env.mjData.qvel[6:].tolist()
self.publisher_blind_state.publish(blind_state_msg)
```

- `base_lin_vel`(m/s、world座標系)、`base_ang_vel`(rad/s、base座標系)：
  read_code_01で確認した`QuadrupedEnv`の同名メソッドと同じ非対称な座標系
  (world/base)がここでもそのまま現れる
- `base_state_msg.pose.orientation`：`np.roll(qpos[3:7], -1)`。MuJoCoの
  `qpos[3:7]`は`[w,x,y,z]`順のクォータニオンであり、これを`roll(-1)`する
  ことで`[x,y,z,w]`順に変換している。[read_code_16](read_code_16_ros2_communication_overview.md)
  で述べた「DLS2側は`[x,y,z,w]`順」という事実の変換元がここで確認できる
- `blind_state_msg.joints_position`/`joints_velocity`：`qpos[7:]`(rad)/
  `qvel[6:]`(rad/s)。read_code_01で確認した`simulation.py`側の
  `joints_pos`(インデックス配列であって実角度ではない、という指摘)とは違い、
  ここは**実際の関節角度そのもの**をpublishしている
- `acceleration`・`stance_status`(`BaseState`)、`joints_acceleration`以下の
  各フィールド(`BlindState`)は未代入([read_code_16](read_code_16_ros2_communication_overview.md)で既出)

### 147〜150行:描画レート制限

```python
if time.time() - self.last_render_time > 1.0 / RENDER_FREQ:
    self.env.render()
    self.last_render_time = time.time()
```

- 物理シミュレーションは500Hzで進むが、描画は`RENDER_FREQ`(既定`30`Hz)に
  制限している。`simulation.py`(read_code_01)の`render_frequency`と同じ
  「壁時計の経過時間を見て間引く」パターン

---

## 153〜165行:`main`

この関数の役割:ROS2を初期化し、`Simulator_Node`を生成してスピンし続ける。

```python
def main():
    print('Hello from the gym_quadruped simulator.')
    rclpy.init()
    simulator_node = Simulator_Node()
    rclpy.spin(simulator_node)
    simulator_node.destroy_node()
    rclpy.shutdown()
```

- `rclpy.spin(simulator_node)`：ここでブロックし、以後はROS2のコールバック
  (タイマー・サブスクライバ)だけがこのプロセスのすべての処理を駆動する。
  `simulation.py`(read_code_01)のような明示的な`for`ループは存在しない

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `/trajectory_generator`から受信した`joints_position`は保存されるが、
     どこにも使われない(死んだデータパス)
  2. `USE_SCHEDULER`定数はこのファイル内で参照されない、事実上死んだ変数
  3. `/control_signal`受信とタイマー発火は非同期で、トルクが届かない周期では
     前回値がそのまま再利用される(タイムアウト処理なし)
- 確認できた重要な事実:
  - `Simulator_Node`は制御ロジックを一切持たず、「トルクを受けて物理を進め、
    状態をpublishするだけ」の薄いラッパーである
  - 物理シミュレーションは壁時計ベースの500Hzタイマーで駆動され、
    `/control_signal`の受信頻度には従わない
  - `BaseState`のクォータニオン順序変換(`[w,x,y,z]`→`[x,y,z,w]`)は
    このファイルで行われている
- 次は`Quadruped_PyMPC_Node`の初期化部分
  (`read_code_18_ros2_controller_init.md`)に進みます。
