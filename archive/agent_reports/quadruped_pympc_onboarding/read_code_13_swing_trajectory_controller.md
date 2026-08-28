# 遊脚制御 helpers/swing_trajectory_controller.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.compute_stance_and_swing_torque(...)  (read_code_12)
          → self.stc.update_swing_time(...)                      ← 本ファイル、毎周期
          → self.stc.compute_swing_control_cartesian_space(...)  ← 本ファイル、遊脚のときだけ
      → self.wb_interface.update_state_and_reference(...)  (read_code_06)
          → self.stc.check_apex_condition(...)             ← 本ファイル、VFA有効時のみ(既定OFF)
          → self.stc.check_full_stance_condition(...)       ← 本ファイル、VFA有効時のみ(既定OFF)
          → self.stc.check_touch_down_condition(...)        ← 本ファイル、optimize_step_freq有効時のみ(既定OFF)
```

`self.stc`は`WBInterface.__init__`(read_code_06)の中で`SwingTrajectoryController(...)`
として生成されるインスタンス。`update_swing_time`と`compute_swing_control_cartesian_space`
は毎周期(既定500Hz相当)呼ばれるが、`check_*`系の3メソッドはいずれもread_code_06で
確認した既定OFFの機能(VFA・歩行周波数最適化)からしか呼ばれない。

## このクラスの役割(全体の中での位置づけ)

`SwingTrajectoryController`が担当するのは、「**遊脚中の足を、離陸位置から目標着地点まで
滑らかな軌道で運ぶための制御則**」です。read_code_12で見た通り、立脚脚は単純な
$\tau=-J^\top F$だけで済むのに対し、遊脚脚はこのクラスが持つカルテシアン空間の
軌道追従制御(PD+フィードバック線形化)を使う。

対象は`external/Quadruped-PyMPC/quadruped_pympc/helpers/swing_trajectory_controller.py`
(410行)です。

---

## 1〜31行:`__init__`

この関数の役割:軌道生成器(scipy版または明示的軌道版)を選び、フィードバック係数と内部状態を初期化する。

```python
def __init__(self, step_height, swing_period, position_gain_fb, velocity_gain_fb, generator):
    self.generator = generator
    if self.generator == "scipy":
        from .swing_generators.scipy_swing_trajectory_generator import SwingTrajectoryGenerator
        self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)
    else:
        from .swing_generators.explicit_swing_trajectory_generator import SwingTrajectoryGenerator
        self.swing_generator = SwingTrajectoryGenerator(swing_period=swing_period, step_height=step_height)

    self.position_gain_fb = position_gain_fb
    self.velocity_gain_fb = velocity_gain_fb
    self.swing_period = swing_period
    self.swing_time = [0, 0, 0, 0]

    self.use_feedback_linearization = True
    self.use_friction_compensation = True
    self.rising_edge_detected = False
```

- `step_height`(m)：デフォルト値はなく必須引数。read_code_06で確認した通り、呼び出し元では`config.py`の`0.2 * hip_height`(Go2なら`0.056`)が渡される
- `swing_period`(秒)：デフォルト値はなく必須引数。呼び出し元では`(1-duty_factor)*(1/step_freq)`(既定`0.25`秒、read_code_09で計算済み)
- `position_gain_fb`(無次元、比例ゲイン)：デフォルト値はなく必須引数。呼び出し元では`config.py`の`swing_position_gain_fb`(既定`500`)
- `velocity_gain_fb`(無次元、微分ゲイン)：デフォルト値はなく必須引数。呼び出し元では`config.py`の`swing_velocity_gain_fb`(既定`10`)
- `generator`(文字列)：デフォルト値はなく必須引数。呼び出し元では`config.py`の`swing_generator`(既定`'scipy'`)。**既定では`if`側が選ばれ、`scipy_swing_trajectory_generator.py`の`SwingTrajectoryGenerator`が使われる**(未読、本章では扱わない)
- `self.swing_time`：4脚分の経過時間(秒)。初期値`[0,0,0,0]`
- `self.use_feedback_linearization`：`True`固定(コンストラクタ内のハードコード、`config.py`のキーではない)
- `self.use_friction_compensation`：`True`固定(read_code_12で確認済み、同じくハードコード)
- `self.rising_edge_detected`：`bool`。初期値`False`。`check_touch_down_condition`で使われる

---

## 33〜42行:`regenerate_swing_trajectory_generator`

この関数の役割:歩行周波数が変わったときに、軌道生成器を新しい`swing_period`で作り直す。

- read_code_12で確認した通り、`optimize_swing==1`のとき(既定`optimize_step_freq=False`のため実質未使用)だけ呼ばれる
- 中身は`__init__`と同じ生成ロジックの繰り返し(共通化されていない)

---

## 44〜90行:`compute_swing_control_cartesian_space`

この関数の役割:目標軌道との誤差から、遊脚の関節トルクをカルテシアン空間のPD制御+フィードバック線形化で計算する。

```python
def compute_swing_control_cartesian_space(
    self, leg_id, q_dot, J, J_dot, lift_off, touch_down, foot_pos, foot_vel,
    passive_force, h, mass_matrix, early_stance_hitmoments, early_stance_hitpoints
):
    des_foot_pos, des_foot_vel, des_foot_acc = self.swing_generator.compute_trajectory_references(
        self.swing_time[leg_id], lift_off, touch_down, early_stance_hitmoments, early_stance_hitpoints
    )

    err_pos = (des_foot_pos - foot_pos).reshape((3,))
    err_vel = (des_foot_vel - foot_vel).reshape((3,))

    accelleration = des_foot_acc + self.position_gain_fb * (err_pos) + self.velocity_gain_fb * (err_vel)
    accelleration = accelleration.reshape((3,))

    # Compute inertia matrix in task space.
    # Mass Matrix and centrifugal missing
    tau_swing = J.T @ (self.position_gain_fb * (err_pos) + self.velocity_gain_fb * (err_vel))
    if self.use_feedback_linearization:
        tau_swing += mass_matrix @ np.linalg.pinv(J) @ (accelleration - J_dot @ q_dot) + h

    return tau_swing, des_foot_pos, des_foot_vel
```

- 引数はすべてデフォルト値がなく必須。`leg_id`(整数、0〜3)、`q_dot`(rad/s)、`J`/`J_dot`(ヤコビアンとその微分)、`lift_off`/`touch_down`(m、軌道の始点・終点)、`foot_pos`/`foot_vel`(m, m/s、現在の足の状態)、`passive_force`(この関数内では未使用、後述)、`h`(N·m、遠心力・コリオリ力)、`mass_matrix`(関節空間の質量行列)、`early_stance_hitmoments`/`early_stance_hitpoints`(`EarlyStanceDetector`由来)
- `self.swing_generator.compute_trajectory_references(...)`(未読、`scipy_swing_trajectory_generator.py`)：今の経過時間から、目標の足位置・速度・加速度を計算する。離陸位置と着地目標を結ぶ軌道の、今この瞬間の点を返すと考えられる(**設計上の解釈**)
- `err_pos`/`err_vel`(m, m/s)：目標と実測の差
- `accelleration`：目標加速度に、位置誤差×`position_gain_fb`(500)と速度誤差×`velocity_gain_fb`(10)を加えたもの

**実装上の問題点(PD項が二重に入る、過去に指摘されていた内容の直接確認)**：

- `tau_swing`の最初の行は`J.T @ (Kp*err_pos + Kd*err_vel)`。これは「PD項をヤコビアン転置で関節トルクへ変換した」項
- `use_feedback_linearization`(既定`True`)が有効だと、`accelleration - J_dot@q_dot`を`mass_matrix @ pinv(J)`で関節トルクへ変換した項が**加算**される。しかしこの`accelleration`自体、すでに`Kp*err_pos + Kd*err_vel`を含んでいる(79行目)
- つまり最終的な`tau_swing`には、PDフィードバック項が**(1)`J.T@(...)`経由で1回、(2)フィードバック線形化の`accelleration`経由でもう1回**、合計2回加算されている。ゲインが同じ`position_gain_fb`/`velocity_gain_fb`のため、実質的にPDゲインが二重に効いていることになる
- コメント「Compute inertia matrix in task space. Mass Matrix and centrifugal missing」は、この`tau_swing`の最初の行が「本来は質量行列・遠心力項を含めた形にすべきだが、まだ入っていない」という開発者の認識を示しており、直後の`if`ブロックで別の形でこれらを補っている。しかし結果としてPD項が二重計上される構造になっていることまでは、コメントからは触れられていない
- `passive_force`引数はこの関数内で一度も使われていない(**実装上の問題点**、受け取るだけの未使用引数)

---

## 92〜114行:`compute_swing_control_joint_space`(呼び出し元が見つからない)

この関数の役割:関節空間で目標関節角度・速度・加速度との誤差からトルクを計算する、カルテシアン版とは別方式の遊脚制御。

```python
def compute_swing_control_joint_space(
    self, nmpc_joints_pos, nmpc_joints_vel, nmpc_joints_acc, qpos, qvel,
    legs_mass_matrix, legs_qfrc_bias, legs_qfrc_passive
):
    error_position = (nmpc_joints_pos - qpos).reshape((3,))
    error_velocity = (nmpc_joints_vel - qvel).reshape((3,))
    accelleration = nmpc_joints_acc.reshape((3,))

    tau_swing = self.position_gain_fb * error_position + self.velocity_gain_fb * error_velocity
    if self.use_feedback_linearization:
        tau_swing += (
            legs_mass_matrix @ (accelleration + self.position_gain_fb * error_position + self.velocity_gain_fb * error_velocity)
            + legs_qfrc_bias
        )
    return tau_swing, None, None
```

- こちらも同じ構造(PD項が`tau_swing`本体とフィードバック線形化の両方に現れる)で、`compute_swing_control_cartesian_space`と同種の二重計上が起きる
- **未確認**：read_code_12で読んだ`compute_stance_and_swing_torque`は`compute_swing_control_cartesian_space`だけを呼んでおり、この`compute_swing_control_joint_space`を呼んでいる箇所は、本パスで読んだ範囲(read_code_01〜12)には見つからなかった。`kinodynamic`タイプ(既定では使われない)向けの未使用コードである可能性があるが、確定はできない

---

## 116〜123行:`update_swing_time`

この関数の役割:遊脚中の脚の経過時間を積算し、接地したら0に戻す。

```python
def update_swing_time(self, current_contact, legs_order, dt):
    for leg_id, leg_name in enumerate(legs_order):
        if current_contact[leg_id] == 0:
            if self.swing_time[leg_id] < self.swing_period:
                self.swing_time[leg_id] = self.swing_time[leg_id] + dt
        else:
            self.swing_time[leg_id] = 0
```

- `current_contact`：4脚分の0/1配列。デフォルト値はなく必須引数
- `legs_order`：脚名の並び順。デフォルト値はなく必須引数
- `dt`(秒)：デフォルト値はなく必須引数。呼び出し元では`simulation_dt`(既定`0.002`)
- `self.swing_time[leg_id]`は`self.swing_period`(既定`0.25`秒)に達したら以降は増えなくなる(頭打ち)。read_code_12の`compute_swing_control_cartesian_space`はこの`swing_time`を軌道生成器へ渡すため、この頭打ちにより軌道は着地目標時刻以降そのまま止まった値を返し続けると考えられる(**設計上の解釈**)

---

## 125〜161行:歩容・VFAタイミング判定の3メソッド(既定OFFの機能からのみ呼ばれる)

```python
def check_apex_condition(self, current_contact, interval=0.02):
    apex = 0
    for leg_id in range(4):
        if current_contact[leg_id] == 0:
            if (self.swing_time[leg_id] > (self.swing_period / 2.0) - interval) and (
                self.swing_time[leg_id] < (self.swing_period / 2.0) + interval
            ):
                apex = 1
    return apex

def check_full_stance_condition(self, current_contact):
    stance = 1
    for leg_id in range(4):
        if current_contact[leg_id] == 0:
            stance = 0
    return stance

def check_touch_down_condition(self, current_contact, previous_contact, contact_sequence, lookahead=3):
    if np.all(current_contact == 1) and not np.all(previous_contact == 1):
        self.rising_edge_detected = True
    stable_stance = np.all(contact_sequence[:, 0:lookahead] == 1)
    next_leg_lift = not np.all(contact_sequence[:, lookahead] == 1)
    if self.rising_edge_detected and stable_stance and next_leg_lift:
        self.rising_edge_detected = False
        return 1
    else:
        return 0
```

- `check_apex_condition`：`interval`(秒)のデフォルト`0.02`。いずれかの脚が遊脚周期のちょうど中間点(頂点=apex)付近にいれば`1`を返す。read_code_06の`update_state_and_reference`内、VFA(既定OFF)のトリガー判定にのみ使われる
- `check_full_stance_condition`：4脚とも接地していれば`1`。VFAのリセット判定にのみ使われる(既定OFF)
- `check_touch_down_condition`：`lookahead`(ステップ数)のデフォルト`3`。「今まさに全脚接地に切り替わった(rising edge)」ことを記録しておき、その後ホライズンの先読みで「今後`lookahead`ステップは安定して接地が続くが、その次では誰かが浮く」という条件が揃ったときだけ`1`を返す。read_code_06で確認した通り`optimize_step_freq`(既定`False`)のときのみ呼ばれ、既定では未使用
- いずれも本パスの範囲では、既定のトロット歩行では呼ばれない(呼び出し元がすべて既定OFFの機能に属するため)

---

## 163〜410行:`__main__`ブロック(呼ばれない、かつ現在のAPIと食い違っている)

- このファイルを直接実行したときだけ動く、単体デモ用のコード。`simulation.py`からの呼び出し連鎖には含まれないため詳細な解説は保留とします
- 参考までに:`QuadrupedEnv(...)`の呼び出しに`hip_height=`・`legs_joint_names=`・`feet_geom_name=`という引数を渡しているが、read_code_01で確認した現在の`QuadrupedEnv.__init__`のシグネチャにはこれらの引数は無い。また`wb_interface.update_state_and_reference(...)`の呼び出しは9個の戻り値を受け取ろうとしているが、read_code_06で確認した現在の実装は5個しか返さない。**この`__main__`ブロックは、クラスの現在のAPIと食い違っており、そのままでは動かない**、read_code_03の`__main__`と同種の放置されたデモコードです

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `compute_swing_control_cartesian_space`(および同型の`compute_swing_control_joint_space`)で、PDフィードバック項が`tau_swing`本体とフィードバック線形化の`accelleration`の両方に現れ、実質二重に加算されている
  2. `passive_force`引数が受け取られるだけで使われていない
  3. `compute_swing_control_joint_space`の呼び出し元が、本パスで読んだ範囲には見つからない(未使用の疑い)
  4. `__main__`ブロックが現在のクラスAPI(`QuadrupedEnv`のコンストラクタ引数、`update_state_and_reference`/`compute_stance_and_swing_torque`の戻り値・引数の数)と食い違っており、動作しない状態で放置されている
- 確認できた重要な事実:
  - `use_feedback_linearization`・`use_friction_compensation`はどちらも`config.py`ではなくこのクラス内にハードコードされた`True`
  - 既定の軌道生成器は`config.py`の`swing_generator='scipy'`により`scipy_swing_trajectory_generator.py`(未読)
  - `check_apex_condition`・`check_full_stance_condition`・`check_touch_down_condition`は、いずれも既定OFFの機能(VFA・歩行周波数最適化)からしか呼ばれない
- 次は、read_code_12で呼び出しを確認した`EarlyStanceDetector`(早期接地検知)または`InverseKinematicsNumeric`(逆運動学)のどちらかに進みます。
