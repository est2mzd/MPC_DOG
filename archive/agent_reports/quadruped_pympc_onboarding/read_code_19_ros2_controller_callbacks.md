# ROS2 制御ノード(コールバック本体) ros2/run_controller.py 逐次解説 (2/2)

## 通信上の位置づけ

[read_code_18](read_code_18_ros2_controller_init.md)と同じ`Quadruped_PyMPC_Node`
の続きです。トピック一覧は[read_code_16](read_code_16_ros2_communication_overview.md)
を参照してください。

## このファイルの役割(全体の中での位置づけ)

この章で扱うのは、`Quadruped_PyMPC_Node`が実際に**毎周期動く処理そのもの**です。
[read_code_18](read_code_18_ros2_controller_init.md)で準備した`WBInterface`と
`SRBDControllerInterface`を使い、「`/base_state`・`/blind_state`受信 →
状態集約 → MPC → WBC → `/control_signal`・`/trajectory_generator`送信」という
1制御周期分の処理を、`compute_control_callback`(既定OFFのMPC並列化変種を除けば
唯一の本体)が担います。

対象は`external/Quadruped-PyMPC/ros2/run_controller.py`の279〜727行目です。

---

## 279〜410行:既定OFFのMPC並列化コールバック3種(概要のみ)

[read_code_18](read_code_18_ros2_controller_init.md)で確認した通り、
`USE_THREADED_MPC`・`USE_PROCESS_QUEUE_MPC`・`USE_PROCESS_SHARED_MEMORY_MPC`
は既定すべて`False`のため、以下の3つのメソッドは**既定では一度も呼ばれません**。
概要と、見つかった問題点だけ記します。

### `compute_mpc_thread_callback`(279〜303行目)

この関数の役割:MPC計算を専用スレッドで無限ループさせ、`self.state_current`等
の`self`変数経由でメインスレッドとやり取りする(既定では起動されない)。

- `while True:`の中で`1.0/MPC_FREQ`(既定`0.01`秒=100Hz)ごとに
  `self.srbd_controller_interface.compute_control(...)`(read_code_11で読んだ
  メソッドそのもの)を呼び、結果を`self.nmpc_GRFs`等へ直接書き込む
- スレッド間の排他制御(ロック)は無い。メインスレッド側が同時に
  `self.nmpc_GRFs`を読んでいる最中に書き換わる可能性があり、
  レースコンディションが起こり得る(**実装上の問題点**、既定OFFのため
  実害は無いが、有効化する場合は注意が必要)

### `compute_mpc_process_queue_callback`(306〜350行目)

この関数の役割:MPC計算を別プロセスで無限ループさせ、`multiprocessing.Queue`
経由で入出力をやり取りする(既定では起動されない)。

- 入力・出力とも`maxsize=1`のキュー。「常に最新の1件だけを保持し、古い
  未処理データは`put_nowait`側で捨てられる」設計(read_code_16で見た
  ROS2トピックのQoS depth=1と同じ思想)
- プロセス優先度の設定([read_code_18](read_code_18_ros2_controller_init.md)
  で見た`renice`と同じコード)をこのプロセス内でも再度行う

### `compute_mpc_process_shared_memory_callback`(353〜409行目)

この関数の役割:MPC計算を別プロセスで無限ループさせ、共有メモリ+seqlock
(ロックフリーな一貫性保証)で出力だけをより低レイテンシに渡す
(既定では起動されない)。

```python
arr[IDX_GRF]  = legsattr_to12(nmpc_GRFs)
arr[IDX_FH]   = legsattr_to12(nmpc_footholds)
arr[IDX_JP]   = (nmpc_joints_pos if nmpc_predicted_state is not None else np.zeros(12).reshape(-1)[:12])
arr[IDX_JV]   = (nmpc_joints_pos if nmpc_predicted_state is not None else np.zeros(12).reshape(-1)[:12])
arr[IDX_JA]   = (nmpc_joints_pos if nmpc_predicted_state is not None else np.zeros(12).reshape(-1)[:12])
```

**実装上の問題点(コピー&ペーストのバグ)**:`IDX_JV`(関節速度のはずの領域)と
`IDX_JA`(関節加速度のはずの領域)に、どちらも`nmpc_joints_vel`/`nmpc_joints_acc`
ではなく**`nmpc_joints_pos`が代入されています**。変数名末尾を書き換え忘れた
典型的なコピー&ペーストミスです。`USE_PROCESS_SHARED_MEMORY_MPC=False`が
既定のため現在は実害がありませんが、もしこの変種を有効化すると、共有メモリ
経由で読み出される関節速度・加速度が常に関節角度の値になるという不具合が
発生します。

- `seq_out`(`multiprocessing.Value`、64bit整数)：偶数=安定、奇数=書き込み中、
  を表すseqlock方式のシーケンス番号。読み手側(read_code_19の
  `compute_control_callback`内、後述)は「書き込み前後でシーケンス番号が
  変わっていないか」を確認してから読み取ったデータを採用する

---

## 412〜434行:`get_base_state_callback`

この関数の役割:`/base_state`メッセージを受信し、base位置・姿勢・速度を
`self`へキャッシュする(既定で毎回、平滑化つき)。

```python
if(USE_SMOOTH_HEIGHT):
    self.position[2] = 0.5*self.position[2] + 0.5*np.array(msg.pose.position)[2]
else:
    self.position[2] = np.array(msg.pose.position)[2]
self.position[0:2] = np.array(msg.pose.position)[0:2]

if(USE_SMOOTH_VELOCITY):
    self.linear_velocity = 0.5*self.linear_velocity + 0.5*np.array(msg.velocity.linear)
else:
    self.linear_velocity = np.array(msg.velocity.linear)

self.orientation = np.roll(np.array(msg.pose.orientation), 1)
self.angular_velocity = np.array(msg.velocity.angular) 

self.first_message_base_arrived = True
```

- `USE_SMOOTH_HEIGHT=True`(既定)：base高さ(z、m)だけを1次の指数移動平均
  (係数`0.5`固定)で平滑化する。x/y位置と姿勢・角速度は平滑化しない
- `USE_SMOOTH_VELOCITY=False`(既定)：並進速度は平滑化せず、受信値を
  そのまま使う
- `self.orientation = np.roll(msg.pose.orientation, 1)`：受信した
  `[x,y,z,w]`順のクォータニオン([read_code_16](read_code_16_ros2_communication_overview.md)
  で確認した`run_simulator.py`側の変換の逆)を`roll(1)`して
  `[w,x,y,z]`順(MuJoCoの`qpos[3:7]`と同じ並び)に戻す
- コード中のコメント「For the angular velocity, mujoco is in the base
  frame, and DLS2 is in the world frame」：**コードで確認した事実として
  コメントは存在するが、実際の変換コードは無く、`msg.velocity.angular`を
  そのまま`self.angular_velocity`へ代入しているだけです**。コメントが
  示す座標系変換(world→base)は実装されていません(**実装上の問題点**、
  コメントと実装の不一致。この`self.angular_velocity`は次のブロックで
  `env.mjData.qvel[3:6]`へそのまま書き込まれるため、もしDLS2側が本当に
  world座標系で角速度を送っているなら、MuJoCo側はbase座標系として誤読
  することになる。実際のpublisher側の実装は対象リポジトリに無いため
  **未確認**)

---

## 437〜447行:`get_blind_state_callback`

この関数の役割:`/blind_state`メッセージを受信して関節状態をキャッシュし、
既定では**この関数自身が制御ループ全体を起動する**。

```python
def get_blind_state_callback(self, msg):
    self.joint_positions = np.array(msg.joints_position)
    self.joint_velocities = np.array(msg.joints_velocity)
    self.feet_contact = np.array(msg.feet_contact)
    self.first_message_joints_arrived = True
    if(not USE_SCHEDULER):
        self.compute_control_callback()
```

- `USE_SCHEDULER=False`(既定)のため、**`/blind_state`を受信するたびに
  `compute_control_callback`(この章の主役)が同期的に呼ばれます**。
  つまりこのノードの制御周期は、ROS2タイマーではなく「`Simulator_Node`が
  `/blind_state`をpublishする頻度」(既定500Hz、read_code_17)に
  実質的に従属しています
- `self.feet_contact`：`/blind_state`の`feet_contact`フィールドを保持する。
  ただし[read_code_17](read_code_17_ros2_run_simulator.md)で確認した通り、
  `run_simulator.py`側はこのフィールドに一度も値を代入していないため、
  **`self.feet_contact`は常に未初期化(デフォルト)のまま届く**
  (**実装上の問題点**、送信側と受信側のフィールド利用がかみ合っていない)

---

## 451〜469行:`get_joy_callback`

この関数の役割:ジョイスティック入力を受け取り、速度指令を直接書き換える。

```python
self.env._ref_base_lin_vel_H[0] = msg.axes[1]/3.5  # Forward/Backward
self.env._ref_base_lin_vel_H[1] = msg.axes[0]/3.5  # Left/Right
self.env._ref_base_ang_yaw_dot = msg.axes[3]/2.  # Yaw

if msg.buttons[8] == 1:
    ...
    os.system("kill -9 $(ps -u | grep -m 1 hal | grep -o \"^[^ ]* *[0-9]*\" | grep -o \"[0-9]*\")")
    os.system("pkill -f play_ros2.py") 
    exit(0)
```

- `msg.axes[1]`/`msg.axes[0]`/`msg.axes[3]`(`sensor_msgs/Joy`の`axes`配列、
  無次元、`-1.0`〜`1.0`)を、それぞれ`3.5`・`3.5`・`2.0`で割って
  `env._ref_base_lin_vel_H`(m/s)・`_ref_base_ang_yaw_dot`(rad/s)へ直接
  代入する。docstringコメントより、8BitDo Ultimate 2Cコントローラを
  対象に想定していることがわかる
- `msg.buttons[8]`が押されると、`hal`という名前を含むプロセスと
  `play_ros2.py`というプロセスを`kill`しようとする。この2つの名前は
  **対象リポジトリのコードには登場せず**、実機側(このリポジトリ外)の
  停止用スクリプト・プロセスを指していると考えられる(**未確認**)

---

## 473〜711行:`compute_control_callback`

この関数の役割:受信済みの状態から、状態/参照値の集約→MPC→WBC→トルク・
軌道メッセージの送信までの1制御周期を実行する。既定では
`get_blind_state_callback`から毎回呼ばれる、このノードの心臓部。

### 476〜491行:ループ時間の計測と安全チェック

```python
if(USE_FIXED_LOOP_TIME):
    simulation_dt = 1./SCHEDULER_FREQ
else:
    start_time = time.perf_counter()
    if(self.last_start_time is not None):
        self.loop_time = (start_time - self.last_start_time)
    self.last_start_time = start_time
    simulation_dt = self.loop_time
    if(USE_SATURATED_LOOP_TIME):
        if(simulation_dt > 0.005):
            simulation_dt = 0.005

if(self.first_message_base_arrived==False and self.first_message_joints_arrived==False):
    return
```

- `USE_FIXED_LOOP_TIME=False`(既定)のため、`simulation_dt`(秒)は
  **実測値**(前回呼び出しからの経過時間)を使う。`USE_SCHEDULER=False`と
  合わせて考えると、この`simulation_dt`は実質的に「`/blind_state`の
  受信間隔」を測っていることになる
- `USE_SATURATED_LOOP_TIME=True`(既定)：実測値が`0.005`秒(200Hz相当)を
  超えたら`0.005`秒に丸める。何らかの理由で受信間隔が開いた場合でも、
  下流の歩容位相更新(`PeriodicGaitGenerator`、read_code_02)が過大な
  `dt`で一気に進みすぎないようにする安全策と考えられる(**設計上の解釈**)
- 安全チェック：`self.first_message_base_arrived`と
  `self.first_message_joints_arrived`が**両方Falseの場合のみ**`return`する。
  **実装上の問題点**:条件が`and`のため、たとえば`/base_state`だけ届いて
  `/blind_state`がまだ届いていない状態(`first_message_base_arrived=True`、
  `first_message_joints_arrived=False`)でも、この関数はそのまま処理を
  続行してしまいます(本来は`or`にして「どちらか片方でも届いていなければ
  return」とすべきところと考えられる)。ただし実際には
  `get_blind_state_callback`からしかこの関数は呼ばれないため
  (`USE_SCHEDULER=False`の場合)、`/blind_state`は必ず届いた直後に
  呼ばれる形にはなる

### 494〜509行:受信済み状態でMuJoCoモデルを更新(運動学のみ)

```python
self.env.mjData.qpos[0:3] = copy.deepcopy(self.position)
self.env.mjData.qpos[3:7] = copy.deepcopy(self.orientation)
self.env.mjData.qvel[0:3] = copy.deepcopy(self.linear_velocity)
self.env.mjData.qvel[3:6] = copy.deepcopy(self.angular_velocity)
self.env.mjData.qpos[7:] = copy.deepcopy(self.joint_positions)
self.env.mjData.qvel[6:] = copy.deepcopy(self.joint_velocities)
self.env.mjModel.opt.timestep = simulation_dt
self.env.mjModel.opt.disableflags = 16 # Disable the collision detection
mujoco.mj_forward(self.env.mjModel, self.env.mjData)   
```

- **コードで確認した事実**:このブロックが、[read_code_18](read_code_18_ros2_controller_init.md)
  で予告した「`env.step`ではなく`mj_forward`だけを使う」ことの直接の
  証拠です。ROS2トピックで受信した状態(位置・姿勢・速度・関節角)を
  そのまま`mjData`へ書き込み、`mj_forward`(順運動学・ヤコビアン・
  質量行列などの姿勢依存量を再計算するだけの関数)を呼ぶだけで、
  MuJoCoの時間積分(接触力計算や運動方程式の解法)は一切行いません
- `self.env.mjModel.opt.disableflags = 16`：MuJoCoの衝突検出を無効化する
  フラグ値。この`env`インスタンスは実機/シミュレータ双方からの受信状態を
  そのまま反映するための「計算専用モデル」であり、それ自身が接触判定を
  する必要が無い(接触状態は`/blind_state`の`feet_contact`、あるいは
  `WBInterface`内部の歩容スケジュールから得る)ため、と考えられる
  (**設計上の解釈**)
- `self.env.mjModel.opt.timestep = simulation_dt`：実測ループ時間を
  MuJoCoモデルのタイムステップとして設定するが、`mj_step`を呼ばない
  以上、この値は`mj_forward`の計算結果には影響しない(**未確認**、
  `mj_forward`が`opt.timestep`を参照する内部処理が無いか完全には
  確認できていない)

### 512〜521行:現在状態の取得

```python
feet_pos = self.env.feet_pos(frame='world')
feet_vel = self.env.feet_vel(frame='world')
hip_pos = self.env.hip_positions(frame='world')
base_lin_vel = self.env.base_lin_vel(frame='world')
base_ang_vel = self.env.base_ang_vel(frame='base')
base_ori_euler_xyz = self.env.base_ori_euler_xyz
base_pos = self.env.base_pos
com_pos = self.env.com
```

- `simulation.py`(read_code_01)が`env`から直接取得していたのと**同じ
  メソッド群**。ここでの`self.env`はMuJoCo演算(`mj_forward`)によって
  上で書き込んだ受信状態から再計算された運動学量を返す

### 524〜531行:参照速度と慣性

```python
ref_base_lin_vel, ref_base_ang_vel = self.env.target_base_vel()

if(cfg.simulation_params['use_inertia_recomputation']):
    inertia = self.env.get_base_inertia().flatten()
else:
    inertia = cfg.inertia.flatten()
```

- `target_base_vel()`：`get_joy_callback`または`console.py`が書き換えた
  `_ref_base_lin_vel_H`/`_ref_base_ang_yaw_dot`から目標速度を計算する、
  read_code_01で確認済みの`gym_quadruped`側メソッド
- `use_inertia_recomputation=True`(既定)：`self.env.get_base_inertia()`で
  毎周期慣性テンソルを再計算する(`False`なら`cfg.inertia`固定値を使う)。
  `simulation.py`側と同じ設定名・同じ既定値

### 533〜553行:qpos/qvel、質量行列、ヤコビアン、ハイトマップ

```python
qpos, qvel = self.env.mjData.qpos, self.env.mjData.qvel
joints_pos = LegsAttr(FL=qpos[7:10], FR=qpos[10:13], RL=qpos[13:16], RR=qpos[16:19])

legs_mass_matrix = self.env.legs_mass_matrix
legs_qfrc_bias = self.env.legs_qfrc_bias
legs_qfrc_passive = self.env.legs_qfrc_passive

feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False)
feet_jac_dot = self.env.feet_jacobians_dot(frame='world', return_rot_jac=False)

legs_qvel_idx = self.env.legs_qvel_idx
legs_qpos_idx = self.env.legs_qpos_idx

heightmaps = None
```

- `joints_pos`：ここでは`qpos[7:10]`等の**実際の関節角度スライス**を
  直接使っており、read_code_01で指摘した`simulation.py`側の
  `joints_pos`(インデックス配列であって実角度ではない)問題は、この
  ROS2版には**存在しません**(実際に関節角度の値そのものを渡している)
- `heightmaps = None`：**実装上の問題点(構造的な制限)**。
  `simulation.py`(read_code_01)は`cfg.simulation_params['visual_foothold_adaptation']`
  の値に応じて実際に`HeightMap`オブジェクトを条件付きで構築していましたが、
  `run_controller.py`には`HeightMap`のimportや構築コードが**一切存在せず**、
  `heightmaps`は設定値に関わらず常に`None`で`update_state_and_reference`
  へ渡されます。既定設定(`visual_foothold_adaptation='blind'`)では
  `simulation.py`側でもどのみち`HeightMap`は使われないため挙動は一致しますが、
  仮に設定を`'height'`や`'vfa'`に変更しても、**ROS2経路ではハイトマップに
  基づく機能が原理的に有効化できません**(simulation.py経路でのみ有効化可能)

### 556〜580行:状態/参照値の集約とコンソールからの手動オフセット

```python
state_current, ref_state, contact_sequence, step_height, optimize_swing = \
    self.wb_interface.update_state_and_reference(com_pos, base_pos, base_lin_vel,
        base_ori_euler_xyz, base_ang_vel, feet_pos, hip_pos, joints_pos,
        heightmaps, legs_order, simulation_dt, ref_base_lin_vel, ref_base_ang_vel)

ref_state["ref_position"][2] += self.console.height_delta
ref_state["ref_orientation"][1] += self.console.pitch_delta
```

- `self.wb_interface.update_state_and_reference`：read_code_06で全文を
  読んだメソッドそのもの。引数の順序・意味は`simulation.py`側の呼び出しと
  同一なので、内部処理はここでは繰り返さない
- **コードで確認した事実**:読んだ直後の`ref_state`に対して、
  `console.py`(read_code_20)がユーザー操作で書き換える
  `self.console.height_delta`(m)・`self.console.pitch_delta`(rad)を
  **加算で上書き**している。これは`simulation.py`経路には存在しない、
  ROS2版だけの「対話コマンドによる目標姿勢の手動オフセット」機能。
  `WBInterface`自体はこのオフセットの存在を知らず、`Quadruped_PyMPC_Node`
  側が`ref_state`辞書を受け取った直後に外側から書き換えている

### 583〜650行:MPC呼び出しの4分岐

```python
if(USE_THREADED_MPC):
    self.state_current = state_current
    ...
elif(USE_PROCESS_QUEUE_MPC):
    ...
elif(USE_PROCESS_SHARED_MEMORY_MPC):
    ...
else:
    if time.time() - self.last_mpc_time > 1.0 / MPC_FREQ:
        self.nmpc_GRFs, self.nmpc_footholds, self.nmpc_joints_pos, \
        self.nmpc_joints_vel, self.nmpc_joints_acc, self.best_sample_freq, \
        self.nmpc_predicted_state = self.srbd_controller_interface.compute_control(
            state_current, ref_state, contact_sequence, inertia,
            self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq,
            optimize_swing)

        if(cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']):
            self.srbd_controller_interface.compute_RTI()
        self.last_mpc_time = time.time()
```

- 既定では`else`分岐のみが実行される。`self.srbd_controller_interface.compute_control`
  は**read_code_11で全文を読んだメソッドそのもの**であり、引数もほぼ同一
  (`simulation.py`経路との違いはread_code_11参照)
- **コードで確認した事実**:この`else`分岐にも、`1.0/MPC_FREQ`(既定
  `1/100=0.01`秒)による独自のレート制限があります。つまり
  `compute_control_callback`自体は`/blind_state`受信のたびに(既定500Hz
  相当で)呼ばれますが、その中のMPC計算部分だけは`self.last_mpc_time`を
  使って**約100Hzに間引かれています**。これは`quadruped_pympc_wrapper.py`
  (`compute_actions`)が`step_num % round(...)`というステップカウンタ方式で
  MPC呼び出しを間引いていたのとは異なる、**壁時計ベースの間引き方式**です
  (**設計上の解釈**:`simulation.py`経路はシミュレーション時刻
  ステップ数で間引くため決定論的だが、ROS2経路は実時間で間引くため、
  実行環境の負荷によって実際のMPC呼び出し頻度が変動しうる)
- `cfg.mpc_params['use_RTI']`は既定`False`のため、`compute_RTI()`は
  既定では呼ばれない(read_code_11で既出のRTI機能)

### 654〜677行:WBCによるトルク計算

```python
self.tau, pd_target_joints_pos, pd_target_joints_vel = \
    self.wb_interface.compute_stance_and_swing_torque(simulation_dt, qpos, qvel,
        feet_jac, feet_jac_dot, feet_pos, feet_vel, legs_qfrc_passive,
        legs_qfrc_bias, legs_mass_matrix, self.nmpc_GRFs, self.nmpc_footholds,
        legs_qpos_idx, legs_qvel_idx, self.tau, optimize_swing,
        self.best_sample_freq, self.nmpc_joints_pos, self.nmpc_joints_vel,
        self.nmpc_joints_acc, self.nmpc_predicted_state)
```

- read_code_12で全文を読んだ`WBInterface.compute_stance_and_swing_torque`
  そのもの。内部で`read_code_13`(スイング制御)・`read_code_14`
  (早期接地検知)・`read_code_15`(逆運動学)がそのまま呼ばれる

### 680〜691行:トルククリップとコンソールの立ち座り上書き

```python
for leg in ["FL", "FR", "RL", "RR"]:
    tau_min, tau_max = self.tau_limits[leg][:, 0], self.tau_limits[leg][:, 1]
    self.tau[leg] = np.clip(self.tau[leg], tau_min, tau_max)

if(self.console.isDown):
    pd_target_joints_pos = self.stand_up_and_down_actions
    pd_target_joints_vel.FL = pd_target_joints_vel.FL*0.0
    ...
```

- [read_code_18](read_code_18_ros2_controller_init.md)で見た
  `tau_limits`(アクチュエータ最大トルクの90%)で最終的にクリップする、
  `simulation.py`側と同種の安全策
- `self.console.isDown`(`bool`、[read_code_20](read_code_20_ros2_console.md)、
  既定`True`=伏せ状態)が`True`の間は、WBCが計算した`pd_target_joints_pos`
  を**丸ごと**`[read_code_18](read_code_18_ros2_controller_init.md)`で見た
  伏せ姿勢キーフレーム(`stand_up_and_down_actions`)に置き換え、目標速度は
  ゼロにする。**トルク自体(`self.tau`)は上書きされない**ため、実際に
  ロボットを支えるのはトルク指令であり、`pd_target_joints_pos`/`vel`は
  (実機の低レベルPD制御向けに送る)補助的な目標値という位置づけになる

### 696〜711行:メッセージのpublish

```python
control_signal_msg = ControlSignal()
control_signal_msg.torques = np.concatenate([self.tau.FL, self.tau.FR, self.tau.RL, self.tau.RR], axis=0).flatten().tolist()
self.publisher_control_signal.publish(control_signal_msg) 

trajectory_generator_msg = TrajectoryGenerator()
trajectory_generator_msg.timestamp = float(self.get_clock().now().nanoseconds)
trajectory_generator_msg.joints_position = np.concatenate([pd_target_joints_pos.FL, ...], axis=0).flatten().tolist()
trajectory_generator_msg.joints_velocity = np.concatenate([pd_target_joints_vel.FL, ...], axis=0).flatten().tolist()
trajectory_generator_msg.kp = (self.impedence_joint_position_gain).tolist()
trajectory_generator_msg.kd = (self.impedence_joint_velocity_gain).tolist()
self.publisher_trajectory_generator.publish(trajectory_generator_msg)

time_debug_msg = TimeDebug()
time_debug_msg.time_wbc = self.loop_time
time_debug_msg.time_mpc = self.last_mpc_loop_time
self.publisher_time_debug.publish(time_debug_msg)
```

- 各フィールドの値の出どころ(`self.tau`、`pd_target_joints_pos`/`vel`、
  `impedence_joint_position/velocity_gain`)はここまでの節で全て確認済み。
  フィールドの意味・単位は[read_code_16](read_code_16_ros2_communication_overview.md)
  の6節に一覧化してある
- `time_debug_msg.time_mpc`：`self.last_mpc_loop_time`。既定の同期`else`
  分岐では**この変数はどこにも代入されていない**ため、常に`__init__`時の
  初期値`0.0`のまま送信され続けます(**実装上の問題点**、`time_mpc`が
  非同期MPC変種(`USE_PROCESS_QUEUE_MPC`等)を使わない限り意味を持たない
  デバッグ用フィールド)

---

## 715〜727行:`main`

この関数の役割:ROS2を初期化し、`Quadruped_PyMPC_Node`を生成してスピンし続ける。

```python
def main():
    print('Hello from Quadruped-PyMPC ros interface.')
    rclpy.init()
    controller_node = Quadruped_PyMPC_Node()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()
```

- `run_simulator.py`(read_code_17)の`main`と対称的な構造。
  `rclpy.spin`に入った後は、コールバック(`get_base_state_callback`・
  `get_blind_state_callback`・`get_joy_callback`)だけがこのプロセスの
  すべての処理を駆動する

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `compute_mpc_process_shared_memory_callback`内で、共有メモリの
     関節速度領域(`IDX_JV`)と関節加速度領域(`IDX_JA`)に、どちらも
     誤って`nmpc_joints_pos`が代入されているコピー&ペーストのバグ
     (既定OFFのため無害だが、有効化すると速度・加速度が常に位置の値になる)
  2. `get_base_state_callback`のコメントは角速度のworld→base座標系変換を
     示唆しているが、実装コードは変換を一切行わず受信値をそのまま使っている
  3. `get_blind_state_callback`が受け取る`self.feet_contact`は、送信元
     (`run_simulator.py`)が値を代入していないため常に未初期化のまま
  4. `compute_control_callback`冒頭の安全チェックが`and`条件になっており、
     片方のトピックだけ届いていない状態でも処理が続行されうる
  5. `time_debug_msg.time_mpc`は既定の同期MPC分岐では常に`0.0`のまま
- 確認できた重要な事実:
  - 制御ループは既定でROS2タイマーではなく`/blind_state`の受信そのものに
    よって駆動される(`get_blind_state_callback`→`compute_control_callback`)
  - `Quadruped_PyMPC_Node`側の`QuadrupedEnv`は`mj_forward`のみを使う
    運動学専用のインスタンスであり、`mj_step`(物理積分)は一度も呼ばれない
  - MPC本体の呼び出しは`SRBDControllerInterface.compute_control`
    (read_code_11)そのものだが、壁時計ベースで`MPC_FREQ`(既定100Hz)に
    間引かれる、`simulation.py`側のステップカウンタ方式とは異なる間引き方式
  - `heightmaps`は設定値に関わらず常に`None`。`simulation.py`経路にあった
    条件付き`HeightMap`構築は、ROS2経路には実装されていない
  - `console.py`が`ref_state`の位置z・姿勢pitchへ、`WBInterface`の外側から
    直接オフセットを加算する
- 次は`console.py`の`Console`クラス
  (`read_code_20_ros2_console.md`)に進みます。これでROS2経路の3ファイル
  すべてが揃います。
