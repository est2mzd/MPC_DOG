# WBC(力→トルク変換) interfaces/wb_interface.py::compute_stance_and_swing_torque 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.compute_stance_and_swing_torque(...)   ← 本ファイル
          → self.stc.compute_swing_control_cartesian_space(...)  (swing_trajectory_controller.py、未読)
          → self.esd.update_detection(...)                       (early_stance_detector.py、未読)
          → self.ik.compute_solution(...)                        (inverse_kinematics_numeric_mujoco.py、未読)
```

read_code_06(`update_state_and_reference`)と同じ`WBInterface`クラスに属するメソッドだが、
呼ばれるタイミングが違う。read_code_06はMPCを解く**前**(状態集約)、この関数はMPCを解いた
**後**(read_code_07〜11のGRFを受け取ってから)呼ばれる。read_code_06の冒頭で予告した通り、
本ファイルが`WBInterface`の「2つ目の仕事」(WBC)を扱う章です。

## この関数の役割(全体の中での位置づけ)

`compute_stance_and_swing_torque`が担当するのは、「**MPCが計算したGRF(力)を、実際に
モーターへ送る関節トルクへ変換する**」ことです。

- 入力:read_code_11で得たGRF(`nmpc_GRFs`)・目標着地点(`nmpc_footholds`)、関節角度・速度、ヤコビアン
- 出力:4脚×3関節=12個の関節トルク、目標関節角度・速度(低レベルPD用)
- 立脚中の脚と遊脚中の脚で、全く異なる2種類の制御則を使う(下記)

対象は`external/Quadruped-PyMPC/quadruped_pympc/interfaces/wb_interface.py`の307〜487行です。

---

## 307〜354行:シグネチャとdocstring

```python
def compute_stance_and_swing_torque(
    self, simulation_dt: float, qpos: np.ndarray, qvel: np.ndarray,
    feet_jac: LegsAttr, feet_jac_dot: LegsAttr, feet_pos: LegsAttr, feet_vel: LegsAttr,
    legs_qfrc_passive: LegsAttr, legs_qfrc_bias: LegsAttr, legs_mass_matrix: LegsAttr,
    nmpc_GRFs: LegsAttr, nmpc_footholds: LegsAttr,
    legs_qpos_idx: LegsAttr, legs_qvel_idx: LegsAttr, tau: LegsAttr,
    optimize_swing: int, best_sample_freq: float,
    nmpc_joints_pos, nmpc_joints_vel, nmpc_joints_acc, nmpc_predicted_state,
    mujoco_contact: np.ndarray = None,
) -> LegsAttr:
```

- 引数はすべてread_code_01(simulation.py)・read_code_11(MPCの出力)で既出のもの。デフォルト値があるのは`mujoco_contact`(`None`)のみ
- docstringには`qpos`・`feet_pos`・`feet_vel`・`legs_mass_matrix`・`legs_qpos_idx`・`optimize_swing`の一部説明が欠けている(実引数には存在するのに記載がない、read_code_06の`update_state_and_reference`と同種の食い違い)

---

## 356〜361行:歩行周波数の最適化が適用された場合の再設定(既定では未実行)

```python
if optimize_swing == 1:
    self.pgg.step_freq = np.array([best_sample_freq])[0]
    self.frg.stance_time = (1 / self.pgg.step_freq) * self.pgg.duty_factor
    swing_period = (1 - self.pgg.duty_factor) * (1 / self.pgg.step_freq)
    self.stc.regenerate_swing_trajectory_generator(step_height=self.step_height, swing_period=swing_period)
```

- `optimize_swing`はread_code_06で確認した通り既定`0`(`optimize_step_freq=False`のため)。**このブロックは既定では実行されない**
- 実行される場合、`read_code_02`の歩容パラメータ(`step_freq`)・`read_code_03`の`stance_time`・スイング軌道生成器を、最適化された新しい歩行周波数で作り直す

---

## 364〜368行:早期接地検知の更新(未読の`EarlyStanceDetector`を呼ぶ)

```python
self.esd.update_detection(feet_pos, self.last_des_foot_pos, lift_off=self.frg.lift_off_positions,
    touch_down=nmpc_footholds, swing_time=self.stc.swing_time, swing_period=self.stc.swing_period,
    current_contact=self.current_contact, previous_contact=self.previous_contact,
    mujoco_contact=mujoco_contact, stc=self.stc)
```

- `self.esd`(`EarlyStanceDetector`、read_code_06の`__init__`で生成確認済み)の内部実装は未読
- 名前と引数から、「予定より早く地面に接触したこと」を検知し、スイング軌道制御に反映するための仕組みと考えられる(**設計上の解釈**)。内部の`hitmoments`/`hitpoints`が386行以降のスイング制御へ渡される。詳細は別途この専用ファイルを読む必要がある(未読、次の候補)

---

## 371〜375行:立脚トルクの計算

```python
tau.FL = -np.matmul(feet_jac.FL[:, legs_qvel_idx.FL].T, nmpc_GRFs.FL)
tau.FR = -np.matmul(feet_jac.FR[:, legs_qvel_idx.FR].T, nmpc_GRFs.FR)
tau.RL = -np.matmul(feet_jac.RL[:, legs_qvel_idx.RL].T, nmpc_GRFs.RL)
tau.RR = -np.matmul(feet_jac.RR[:, legs_qvel_idx.RR].T, nmpc_GRFs.RR)
```

- $\tau^{stance} = -J^\top F$ という式そのもの。`feet_jac.FL[:, legs_qvel_idx.FL]`で全身ヤコビアン(read_code_01で確認した`(3, nv)`)から該当脚の3列だけを取り出し、その転置にGRFを掛ける
- **重要な事実**:この式は**接地中・遊脚中を問わず4脚全部に対して無条件に実行される**。遊脚中の脚も一旦この式でトルクが計算されるが、直後(384行以降)の`if self.current_contact[leg_id]==0:`分岐で**上書きされる**ため、遊脚中の脚にとってこの371〜375行の計算結果は使われない。read_code_09〜11で見てきた「計算されるが捨てられる」パターンが、ここにも小規模な形で存在する
- 符号がマイナスなのは、ヤコビアンの転置が「関節トルク→足先の力」の関係(仮想仕事の原理)を表すため、逆に「地面へ押し返す力」を関節トルクに変換する際に符号が反転する、という力学的な理由による(**解釈**、厳密な導出はこのファイルの範囲では確認していない)
- 追加のPD項(フィードバック補正)は**無い**。これは`docs/pympc_2day`のような一部資料が「立脚トルクにPD項がある」と説明していた場合、実装と食い違うことになる点として過去のセッションで既に確認済み

---

## 377〜408行:遊脚トルクの計算

```python
self.stc.update_swing_time(self.current_contact, self.legs_order, simulation_dt)

des_foot_pos = LegsAttr(*[np.zeros((3,)) for _ in range(4)])
des_foot_vel = LegsAttr(*[np.zeros((3,)) for _ in range(4)])

for leg_id, leg_name in enumerate(self.legs_order):
    if self.current_contact[leg_id] == 0:
        tau[leg_name], des_foot_pos[leg_name], des_foot_vel[leg_name] = (
            self.stc.compute_swing_control_cartesian_space(
                leg_id=leg_id, q_dot=qvel[legs_qvel_idx[leg_name]],
                J=feet_jac[leg_name][:, legs_qvel_idx[leg_name]],
                J_dot=feet_jac_dot[leg_name][:, legs_qvel_idx[leg_name]],
                lift_off=self.frg.lift_off_positions[leg_name],
                touch_down=nmpc_footholds[leg_name],
                foot_pos=feet_pos[leg_name], foot_vel=feet_vel[leg_name],
                passive_force=legs_qfrc_passive[leg_name], h=legs_qfrc_bias[leg_name],
                mass_matrix=legs_mass_matrix[leg_name],
                early_stance_hitmoments=self.esd.hitmoments[leg_name],
                early_stance_hitpoints=self.esd.hitpoints[leg_name],
            )
        )
    else:
        des_foot_pos[leg_name] = nmpc_footholds[leg_name]
        des_foot_vel[leg_name] = des_foot_vel[leg_name] * 0.0

self.last_des_foot_pos = des_foot_pos
```

- `self.current_contact[leg_id] == 0`(遊脚中)の脚だけ、`self.stc.compute_swing_control_cartesian_space`(`SwingTrajectoryController`、未読)を呼んでトルクを計算し直す。ここで371〜375行の立脚式によるトルクが上書きされる
- 渡される引数から、この関数は「離陸位置(`lift_off`)から着地目標(`touch_down`=`nmpc_footholds`)までの軌道追従制御」を、現在の足位置・足速度・質量行列・遠心力/コリオリ力(`h`)・受動力・早期接地情報を使って計算していると考えられる(内部実装は未読)
- 接地中(`else`)の脚は、目標足位置を`nmpc_footholds`(MPCの着地点)、目標足速度を`0`とするだけで、実際のトルク計算はしない(371〜375行で既に決まっている)
- `self.last_des_foot_pos`：計算した目標足位置を次の周期のために保存する(365行目で`update_detection`に渡されていた`self.last_des_foot_pos`はここで更新される)

---

## 413〜418行:摩擦補償(既定で有効)

```python
if(self.stc.use_friction_compensation):
    tau.FL -= legs_qfrc_passive.FL
    tau.FR -= legs_qfrc_passive.FR
    tau.RL -= legs_qfrc_passive.RL
    tau.RR -= legs_qfrc_passive.RR
```

- `self.stc.use_friction_compensation`：`SwingTrajectoryController`のコンストラクタで`True`に固定されている(`config.py`のキーではなく、クラス内のハードコード値)
- **既定で有効**なため、立脚・遊脚を問わず**全4脚**のトルクから、関節の受動力(`legs_qfrc_passive`、摩擦等)を差し引く
- コメント「TODO fix this flag, is not only related to swing」の通り、フラグ名(`SwingTrajectoryController`が持つ)は「スイング専用」に見えるが、実際には立脚側のトルクにも適用される、という命名と実態のズレを開発者自身が認識している

---

## 421〜446行:目標関節角度・速度の計算(逆運動学)

```python
des_joints_pos = LegsAttr(*[np.zeros((3, 1)) for _ in range(4)])
des_joints_vel = LegsAttr(*[np.zeros((3, 1)) for _ in range(4)])
if cfg.mpc_params['type'] != 'kinodynamic':
    qpos_predicted = copy.deepcopy(qpos)
    temp = self.ik.compute_solution(qpos_predicted, des_foot_pos.FL, des_foot_pos.FR, des_foot_pos.RL, des_foot_pos.RR)
    des_joints_pos.FL = np.array(temp[0:3]).reshape((3,))
    ...
    des_joints_vel.FL = np.linalg.pinv(feet_jac.FL[:, legs_qvel_idx.FL]) @ des_foot_vel.FL
    ...
else:
    des_joints_pos = nmpc_joints_pos
    des_joints_pos = nmpc_joints_vel
```

- `mpc_params['type']`は既定`'nominal'`(`!= 'kinodynamic'`)のため、**`if`側が実行される**
- `self.ik.compute_solution(...)`(`InverseKinematicsNumeric`、未読)へ、現在の関節角度と4脚の目標足位置を渡し、対応する目標関節角度を数値的に逆算してもらう
- `des_joints_vel`は、ヤコビアンの疑似逆行列(`np.linalg.pinv`)を目標足速度に掛けて求める(コメント「本来は目標関節位置でのヤコビアンで計算すべき」という開発者自身のTODOあり、現在の実測ヤコビアンを流用している)

**実装上の問題点(明確なコピペミス、ただし到達しない経路)**:`else`節(`kinodynamic`のとき)の2行、

```python
des_joints_pos = nmpc_joints_pos
des_joints_pos = nmpc_joints_vel
```

は、明らかに2行目が`des_joints_vel = nmpc_joints_vel`であるべきところを、`des_joints_pos`に再代入してしまっている。結果として`kinodynamic`タイプでは`des_joints_pos`に速度の値が入り、`des_joints_vel`は423行目で初期化された全`0`のまま残る。ただし`mpc_params['type']`が既定`'nominal'`である限りこの`else`節自体に到達しないため、既定設定では影響しない

---

## 448〜469行:目標値の飽和(急激な変化を防ぐ)

```python
max_joints_pos_difference = 3.0
max_joints_vel_difference = 10.0

actual_joints_pos = LegsAttr(**{leg_name: qpos[legs_qpos_idx[leg_name]] for leg_name in self.legs_order})
actual_joints_vel = LegsAttr(**{leg_name: qvel[legs_qvel_idx[leg_name]] for leg_name in self.legs_order})

for leg in ["FL", "FR", "RL", "RR"]:
    joints_pos_difference = des_joints_pos[leg] - actual_joints_pos[leg]
    saturated_joints_pos_difference = np.clip(joints_pos_difference, -max_joints_pos_difference, max_joints_pos_difference)
    des_joints_pos[leg] = actual_joints_pos[leg] + saturated_joints_pos_difference

    joints_vel_difference = des_joints_vel[leg] - actual_joints_vel[leg]
    saturated_joints_vel_difference = np.clip(joints_vel_difference, -max_joints_vel_difference, max_joints_vel_difference)
    des_joints_vel[leg] = actual_joints_vel[leg] + saturated_joints_vel_difference

return tau, des_joints_pos, des_joints_vel
```

- `max_joints_pos_difference`(rad)：`3.0`固定。目標関節角度が現在値から一度に`±3.0`rad(約171度)以上離れないようにクリップする
- `max_joints_vel_difference`(rad/s)：`10.0`固定
- `des_joints_pos`/`des_joints_vel`：逆運動学で求めた目標値を、実測値からの差分としてクリップしてから返す。数値解の急激な飛びを抑える安全策
- 戻り値の`tau`は、371〜375行(立脚)+384〜404行(遊脚、上書き)+413〜418行(摩擦補償)を経た最終的な関節トルク。`des_joints_pos`/`des_joints_vel`は、read_code_10で見た`quadruped_pympc_wrapper.py`のコメントアウトされた低レベルPD制御ブロックへ渡すために用意された値(既定では未使用のまま破棄される、read_code_10のNN_quadruped_pympc_wrapper_walkthrough.mdで既に確認済み)

---

## 472〜487行:`reset`

この関数の役割:エピソード開始時に、歩容の位相と離地位置・現在接地状態をリセットする。

```python
def reset(self, initial_feet_pos: LegsAttr):
    self.pgg.reset()
    # self.frg.reset()
    # self.stc.reset()
    # self.terrain_computation.reset()
    self.frg.lift_off_positions = initial_feet_pos
    if cfg.simulation_params['visual_foothold_adaptation'] != 'blind':
        self.vfa.reset()
    self.current_contact = np.array([1, 1, 1, 1])
    return
```

- `self.pgg.reset()`(read_code_02で読んだメソッド)は実際に呼ばれるが、`self.frg.reset()`・`self.stc.reset()`・`self.terrain_computation.reset()`は**コメントアウトされており呼ばれない**
- `FootholdReferenceGenerator`(read_code_03)には、そもそも`reset`という名前のメソッド自体が定義されていない(コンストラクタ`__init__`しかない)。よってこのコメントは「呼んでも動かない」ではなく「そもそも対応するメソッドが無い」ことを示している(**未確認**、`TerrainEstimator`(read_code_04)・`SwingTrajectoryController`にも`reset`メソッドがあるかは未確認)
- `self.frg.lift_off_positions`を直接上書きすることで、着地点生成器の状態だけは手動でリセットしている
- `visual_foothold_adaptation`が`"blind"`でないときだけVFAをリセットする(既定では未実行)
- `self.current_contact`を全脚接地に戻す

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `kinodynamic`分岐(既定では到達しない)に`des_joints_pos = nmpc_joints_vel`という明確なコピペミスがある
  2. 立脚トルクの式(371〜375行)は遊脚中の脚にも一旦適用されるが、直後に上書きされ無駄になる
  3. `use_friction_compensation`という名前が「スイング専用」に見えるが、実際は立脚側にも適用される。開発者自身がTODOで認識済み
  4. `WBInterface.reset`が、`FootholdReferenceGenerator`・`SwingTrajectoryController`・`TerrainEstimator`のリセットをコメントアウトしており、実質`PeriodicGaitGenerator`の位相と`current_contact`・`lift_off_positions`しかリセットされない
- 確認できた重要な事実:
  - 立脚は$\tau=-J^\top F$(PD項なし)、遊脚は`SwingTrajectoryController`によるカルテシアン空間の軌道追従制御、という完全に異なる2つの制御則を`if current_contact==0`で切り替えている
  - 摩擦補償(`legs_qfrc_passive`の減算)は既定で全脚に適用される
  - 目標関節角度・速度は、最終的に実測値との差分を`±3.0`rad・`±10.0`rad/sでクリップしてから返される
- これで、simulation.pyから始まった標準経路(計画→状態集約→MPC→WBC)の主要な関数をひととおり読み終えました。未読で残っているのは、`stc`(`SwingTrajectoryController`)・`esd`(`EarlyStanceDetector`)・`ik`(`InverseKinematicsNumeric`)の内部実装です。
