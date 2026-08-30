# Quad-SDK Step 01 の WBC(脚コントローラ / 逆動力学)の理論・コード・パラメータ

作成: 2026-08-30。`external/quad-sdk` の C++ ソース・YAML を実際に読んで
確認した内容。**【事実】=コードで確認済み**、**【推測】=未確認・仮説** として
節を分ける。関連: `agent_reports/quadsdk_step01_control_pipeline.md`(8〜9節)、
`agent_reports/quadsdk_step01_mpc.md`。

---

## 背景

`agent_reports/quadsdk_step01_control_pipeline.md` の8節は「Quad-SDK には
OCS2/legged_control のような独立した階層型 WBC ノードは無く、
`robot_driver_node` 内で選択される脚コントローラ(`inverse_dynamics`)が相当」
という要点だけで、**その脚コントローラが具体的にどういう計算をしているか**
(逆動力学の式、遊脚・接地脚の扱い、最終トルクの合成)には踏み込んでいなかった。

Step 01 では「WALK 移行直後(t≈17〜18秒)の一時的な姿勢の跳躍」が未調査のまま
残っている(handoff 4節)。これが MPC 側か WBC 側かを切り分けるには、
WBC が MPC の GRF 計画をどう関節トルクに変換しているかを押さえる必要がある。

## 目的

- `inverse_dynamics` 脚コントローラの**計算内容**(理論式 + コード対応)を明文化する
- 「接地脚」「遊脚」でトルクの求め方がどう違うかを整理する
- MPC の GRF 計画 → 関節トルク → MuJoCo アクチュエータの最終段までの
  合成式とゲインを1か所にまとめる
- どこが事実で、どこが未確認かを分ける

---

## 概要(何を・どう計算しているか)

- **実体**: `robot_driver_node` 内の `LegController` サブクラス
  `InverseDynamicsController`。`robot_configs` の `"controller": "inverse_dynamics"`
  で選択される。**独立ノードではない**。
- **入力**:
  - `local_plan`(`RobotPlan`。MPC 出力 = body 軌道 + GRF 系列 + 足先軌道)
  - 現在状態(`state/ground_truth`。`agent_reports/quadsdk_step01_sensing.md`)
- **やること(500 Hz)**:
  1. `local_plan` を現在時刻に**線形補間**して参照状態 `ref_state` と
     参照 GRF を得る(GRF は ZOH)
  2. **逆動力学**で各脚の feedforward トルク `τ_ff` を計算
     - **接地脚**: `τ = -J^T f`(接触ヤコビアン転置で GRF → 関節トルク)
     - **遊脚**: 全身動力学 + 接地脚の接触拘束 + 足先加速度追従を
       KKT ブロック系として解いて swing トルクを得る
  3. **遊脚のみ**: デカルト空間 PD(`swing_kp_cart` / `kd_cart`。go2 は 0)を
     関節空間へ写して `τ_ff` に加算
  4. 各関節に `MotorCommand` を出力:
     `pos_setpoint` / `vel_setpoint`(参照)、`torque_ff = τ`、
     `kp` / `kd`(接地: `stance_*`、遊脚: `swing_*`)
- **最終トルク**(ros2_control の `effort_controllers::QuadController`):
  ```
  τ_motor = clamp( kp·(pos_setpoint - pos) + kd·(vel_setpoint - vel) + torque_ff,
                   ±torque_lim )
  ```
- **go2 のゲイン**(go2.yaml): 接地・遊脚とも `kp = [60,60,60]`、`kd = [4,4,4]`、
  `swing_kp_cart = kd_cart = [0,0,0]`、`torque_lim = [33.5, 33.5, 50.0]`。
- **これは狭義の WBC(QP ベースの全身最適化)ではない**。MPC が決めた GRF を
  そのまま使い、逆動力学で関節トルクへ「翻訳」するだけ。最適化は解かない。

---

## 流れ(順を追って)

1. `robot_driver` の制御ループが `update_rate = 500 Hz`(robot_driver.yaml)で
   `updateControl()` を回す。
2. `control_mode_ == READY`(起立済み)なら
   `leg_controller_->computeLegCommandArray(last_robot_state_msg_,
   leg_command_array_msg_, grf_array_msg_)` を呼ぶ。
3. `InverseDynamicsController::computeLegCommandArray()`(下記「詳細」)が
   `local_plan` を補間 → 逆動力学 → `MotorCommand` 配列を作る。
   - `local_plan` が **0.1 s 以上古い**、または現在時刻が計画の時間範囲外なら
     `false` を返す。
4. `false` のとき `robot_driver` は各関節を `stand_joint_angles = [0, 0.8, -1.5]`
   への PD(`stand_kp = [60,60,60]`, `stand_kd = [2,2,2]`)へフォールバック。
5. `leg_command_array_pub_` → `control/joint_command`(`LegCommandArray`)、
   `grf_pub_` → `control/grfs`(`GRFArray`。ロガーが読む値)。
6. ros2_control の `joint_controller`(= `effort_controllers::QuadController`)が
   各関節の最終トルクを計算して MuJoCo アクチュエータへ書き込む
   (`agent_reports/quadsdk_step01_control_pipeline.md` 9節)。

### control/mode の実挙動(注意)

- `run_quadsdk_step01_baseline.sh` のコメントは
  `control/mode: 0=SAFETY, 1=STAND, 2=WALK` と書いているが、
  `robot_driver` の内部 enum は
  **`SIT=0, READY=1, SIT_TO_READY=2, READY_TO_SIT=3, SAFETY=4`**
  (`robot_driver.hpp:287-299`)。
- `controlModeCallback()` が状態遷移するのは `msg->data` が
  `READY(1)` / `SIT(0)` / `SAFETY(4)` のときだけ。
- したがって:
  - `data: 1` 送信 → `SIT → SIT_TO_READY → READY`(起立)。これは効く
  - `data: 2`(スクリプトが「WALK」として送る)→ どの分岐にも一致せず
    **無視される**(【事実】: コード上の分岐にヒットしない)
- 【推測】歩行への移行は実際には `control/mode` ではなく **`cmd_vel`** で
  起きている。`local_planner` は `control/mode` を購読しておらず、
  内部の `STAND / STEP` を `cmd_vel_.norm() > stand_cmd_vel_threshold(0.05)` で
  切り替える(`local_planner.cpp:331-341`)。`robot_driver` は `READY` の間
  `local_plan` を追従するだけ。つまり「起立(`data:1`)+ `cmd_vel` を流す」で
  歩き、`data:2` は実質不要。スクリプトのコメントは意図であって実装と一致しない。

---

## 詳細: 逆動力学の理論式

出所: `quad_utils/src/quad_kd2.cpp: computeInverseDynamics()`(`:753`)、
`getJacobianBodyAngVel()`(`:709`)。Pinocchio ベース(`model_`, `data_`)を
URDF(`robot_description`)から構築。

### 全身動力学

一般化座標 `q = [q_base(6, floating base); q_joint(12)]`、`nv = 18`:

```
M(q) q̈ + N(q, q̇) = S^T τ + Σ_{i=0}^{3} J_i(q)^T f_i
```

- `M(q)` … 質量行列(Pinocchio `data_.M`)
- `N(q, q̇)` … 非線形項 = コリオリ + 遠心力 + 重力(Pinocchio `data_.nle`)
- `S` … アクチュエーション選択行列(関節のみ駆動、base 6 自由度は非駆動)
- `J_i` … 脚 `i` の足先接触ヤコビアン(`LOCAL_WORLD_ALIGNED`、3×nv)
- `f_i` … 脚 `i` の GRF(`local_plan` から。world 系)

Pinocchio の関節順を quad-sdk の脚順(`FL, BL, FR, BR` × `abad, hip, knee`)へ
`jidx` で並べ替えてから全ブロック演算する。

### 接地脚: ヤコビアン転置写像

```
τ_stance = -J^T f_grf
```

(`quad_kd2.cpp:781`。`J` は base + 全 12 関節を含む 12×nv、GRF を一般化力へ。
接地脚の関節トルク成分だけを後で取り出す。)

### 遊脚: KKT ブロック系

未知は「base 加速度 `q̈_b`」「遊脚トルク `τ_swing`(12)」「接触拘束力 `λ`」。
既知は `M`, `N`, `τ_stance`、および足先加速度目標
`a = J_b q̈_b + J_l q̈_l`(`ref_foot_acceleration` から `J̇ q̇` を引いたもの)。

接地脚は接触拘束 `A q̈ = -Ȧ q̇`(`A` は接地脚の足先ヤコビアン)。

これを次のブロック線形系にまとめて `bdcSvd`(thin U/V の SVD)で解く
(`quad_kd2.cpp:844-871`):

```
[ -M_bb + M_bj J_l^{-1} J_b        -A_b^T ] [ q̈_b     ]   [ N_b + M_bj J_l^{-1}(a - J̇q̇)        ]
[ -M_jb + M_jj J_l^{-1} J_b        (遊脚 I)] [ τ_swing ] = [ N_j + M_jj J_l^{-1}(a - J̇q̇) - τ_stance,j ]
[ -A_b   + A_l J_l^{-1} J_b         0      ] [ λ       ]   [ Ȧq̇ + A_l J_l^{-1}(a - J̇q̇)         ]
```

- `J_l^{-1}` … 脚関節ブロック `J(:, 6:18)` の**ダンプ付き最小二乗逆**
  (`math_utils::sdlsInv`)
- 添字 `b` = base 6 自由度、`j` = 関節 12、`l` = 脚(関節)
- 遊脚に対応する対角ブロックには単位を入れて `τ_swing` を直接解に出す
- 解 `blk_sol` の `segment(6, 12)` が `τ_swing`

### トルクの取り出しと安全化

```
接地脚 i: τ(3i:3i+3) = τ_stance の関節成分
遊脚  i: τ(3i:3i+3) = τ_swing(3i:3i+3)
```

`τ` に inf / nan が混じったら `τ.setZero()`(`quad_kd2.cpp:865`)。

---

## 詳細: 脚コントローラのフロー(コード対応)

出所: `robot_driver/src/controllers/inverse_dynamics_controller.cpp`。

1. **鮮度チェック**: `last_local_plan_msg_` が NULL か、受信から 0.1 s 以上
   経過なら `return false`。
2. **参照補間**: `t_now = now - local_plan.state_timestamp`。
   `local_plan.states[i]` と `[i+1]` を `t_interp` で線形補間して
   `ref_state_msg_`(`quad_utils::interpRobotState`)。GRF は
   `grf_array_msg = local_plan.grfs[i]`(ZOH)。範囲外なら `return false`。
3. **GRF フィルタ**: `grf_exp_filter_const_ = 1.0`(ハードコード)なので
   実質フィルタ無し(`grf = 1.0·grf + 0.0·last_grf`)。
4. **接地モード**: `contact_mode[i] = ref_state_msg_.feet.feet[i].contact`。
5. **逆動力学**: `quadKD_->computeInverseDynamics(ref_foot_acceleration,
   grf_array, contact_mode, tau_array)`。
6. **デカルト PD(遊脚用)**:
   ```
   swing_cart_fb = swing_kp_cart ⊙ (ref_foot_pos - foot_pos)
                 + swing_kd_cart ⊙ (ref_foot_vel - foot_vel)
   ```
   を `getJacobianBodyAngVel()` のヤコビアン転置で関節空間へ写す。
   **go2 は `swing_kp_cart = kd_cart = 0` なのでこの項は 0**。
7. **MotorCommand 生成**(各脚 i、各関節 j):
   - `pos_setpoint = ref_state_msg_.joints.position[3i+j]`
   - `vel_setpoint = ref_state_msg_.joints.velocity[3i+j]`
   - `torque_ff = tau_array[3i+j]`(接地脚)/
     `tau_array[3i+j] + swing_cart_fb[3i+j]`(遊脚)
   - 接地脚: `kp = stance_kp[j]`, `kd = stance_kd[j]`
   - 遊脚: `kp = swing_kp[j]`, `kd = swing_kd[j]`

---

## 詳細: 最終トルクの合成(ros2_control)

出所: `quad_simulator/gazebo_plugins/src/controller_plugin.cpp`
(クラス名 `effort_controllers::QuadController`。パッケージ名に "gazebo" と
付くが**純粋な ros2_control コントローラ**。go2.yaml の
`joint_controller: type: effort_controllers/QuadController` がこれ)。

各関節について(`QuadController::update()`):

```
pos_error       = shortest_angular_distance(current_pos, pos_setpoint)   (関節リミット考慮)
vel_error       = vel_setpoint - current_vel
torque_feedback = kp · pos_error + kd · vel_error
torque_command  = clamp( torque_feedback + torque_ff, -torque_lim, +torque_lim )
```

- `torque_lim = motor_limits.torque[j]` = `[33.5, 33.5, 50.0]`(abad, hip, knee)
- BEMF エンベロープ(`motor_model_lb/ub`)は計算されるが
  `apply_motor_model = false` で**未適用**
- 最初のコマンド受信前は sit 姿勢 `[0, 1.36, -2.65]` を `hold_kp = 40`,
  `hold_kd = 2` で保持

---

## 初期パラメータ一覧(go2、Step 01 時点)

`go2.yaml`(`robot_driver`。ゲインは全て `[abad, hip, knee]`):

- `sit_kp = [10,10,10]`, `sit_kd = [1,1,1]`
- `stand_kp = [60,60,60]`, `stand_kd = [2,2,2]`
- `stance_kp = [60,60,60]`, `stance_kd = [4,4,4]`(接地脚)
- `swing_kp = [60,60,60]`, `swing_kd = [4,4,4]`(遊脚・関節空間)
- `swing_kp_cart = [0,0,0]`, `swing_kd_cart = [0,0,0]`(遊脚・デカルト。**無効**)
- `safety_kp = [0,0,0]`, `safety_kd = [2,2,2]`
- `stand_joint_angles = [0, 0.8, -1.5]`
- `sit_joint_angles = [0, 1.36, -2.65]`

`go2.yaml`(共通):

- `motor_limits.torque = [33.5, 33.5, 50.0]` N·m
- `motor_limits.speed = [30.0, 30.0, 20.06]` rad/s

`robot_driver.yaml`:

- `update_rate = 500.0` Hz、`publish_rate = 500.0` Hz
- `input_timeout = 0.2` s、`state_timeout = 0.1` s、`heartbeat_timeout = 0.2` s
- `filter_time_constant = 0.01` s
- `cmd_vel_filter_const = 0.10`、`cmd_vel_scale = 1.0`

`inverse_dynamics_controller.hpp`:

- `grf_exp_filter_const_ = 1.0`(フィルタ無し、ハードコード)

`controller_manager`(go2.yaml):

- `update_rate = 500`
- `joint_controller.type = effort_controllers/QuadController`

---

## 【事実】と【推測】

### 【事実】(コードで確認済み)

- 接地脚トルクは `τ_stance = -J^T f_grf`(ヤコビアン転置写像)。
- 遊脚トルクは「全身動力学 + 接地脚の接触拘束 + 足先加速度追従」の
  KKT ブロック線形系を `bdcSvd` で解いて得る。
- go2 は `swing_kp_cart = kd_cart = 0` なのでデカルト空間 PD は効かず、
  遊脚は関節 PD(`swing_kp/kd = 60/4`)+ 逆動力学 ff のみ。
- 最終トルク合成は `kp·e_pos + kd·e_vel + τ_ff` を `motor_limits.torque` で
  クランプ。BEMF モデルは無効。
- `local_plan` が 0.1 s 古いと脚コントローラは `false` を返し、
  `robot_driver` が `stand_joint_angles` への PD にフォールバックする。
- `robot_driver` の `control/mode` enum は `SIT=0 / READY=1 / SIT_TO_READY=2 /
  READY_TO_SIT=3 / SAFETY=4`。`controlModeCallback` は
  `READY` / `SIT` / `SAFETY` のみで遷移する。
- `effort_controllers::QuadController` の実体は
  `quad_simulator/gazebo_plugins/src/controller_plugin.cpp`。

### 【推測】(未確認)

- **`run_quadsdk_step01_baseline.sh` の `data: 2`(WALK)は無視されている**
  可能性が高い。歩行移行は `cmd_vel` 経由で `local_planner` が
  `STEP` に入ることで起きると読める。実機/実行時に
  `ros2 topic echo /robot_1/control/mode` と挙動を突き合わせての確認は未実施。
- **`ref_foot_acceleration` の中身**: `interpRobotState` が
  `local_plan` の足先加速度をどう補間して `ref_state_msg_.feet[*].acceleration`
  に入れているかは未精査。ゼロに近い場合、遊脚 KKT 系は実質「重力補償 +
  接地拘束のみ」になる。
- **`sdlsInv` のダンピング係数**と `bdcSvd` の数値条件(特異姿勢近傍での
  トルクの跳ね)は未評価。WALK 移行直後の姿勢跳躍(handoff 4節)がここに
  由来する可能性はあるが未検証。
- **`grf_exp_filter_const_ = 1.0` 固定**の妥当性(GRF に補間ノイズが乗った
  ときの平滑化が無い)は未評価。
- ros2_control の `QuadController` が実際にロードされているか
  (`ros2 control list_controllers` での確認)は本調査では未実施。
  handoff の `joint_controller` 起動待ち修正が効いていることから
  ロード自体はされているとみられる。

---

## その後(この WBC の位置づけと次に見るべき点)

- **Step 01 の結論**: WBC は MPC の GRF 計画を逆動力学で関節トルクへ翻訳して
  いるだけで、独自の最適化はしていない。転倒の根本原因は起動シーケンスと
  地面サイズであり(handoff 8節)、WBC の定式化ではなかった。
- **未解決**: WALK 移行直後(t≈17〜18 s)の姿勢跳躍が MPC 側か WBC 側かは
  切り分け未了。候補は (a) `local_plan` 切り替わり時の参照の不連続、
  (b) 遊脚 KKT 系の特異姿勢近傍での数値的跳ね、(c) 接地スケジュール切替時の
  `stance ↔ swing` ゲイン・トルク不連続。
- **ロガーとの対応**: ロガーの `grf_*` / `contact_*` 列はこの脚コントローラが
  出す `grf_array_msg`(= `local_plan` の GRF を ZOH + フィルタ 1.0)由来で、
  接地センサの実測ではない。
- **速度を上げる Step で最初に検討すべきパラメータ**:
  `stance_kp/kd`・`swing_kp/kd`、`torque_lim`、`grf_exp_filter_const_`、
  必要なら `swing_kp_cart/kd_cart` の有効化。いずれも現状は Step 01 の
  制約により未変更。

---

## ソース早見表(`external/quad-sdk/`)

- 脚コントローラ本体
  - `robot_driver/src/controllers/inverse_dynamics_controller.cpp`
    (`computeLegCommandArray`)
  - `robot_driver/include/robot_driver/controllers/inverse_dynamics_controller.hpp`
    (`grf_exp_filter_const_`)
  - `robot_driver/src/controllers/leg_controller.cpp` /
    `.../leg_controller.hpp`(基底、ゲイン `init()`)
- 逆動力学の計算
  - `quad_utils/src/quad_kd2.cpp`
    - `computeInverseDynamics`(`:753`。接地 `-J^T f` / 遊脚 KKT 系)
    - `getJacobianBodyAngVel`(`:709`。足先ヤコビアン)
  - `quad_utils/include/quad_utils/quad_kd2.hpp`
- robot_driver の制御ループ・モード
  - `robot_driver/src/robot_driver.cpp`
    (`updateControl` `:756`、`controlModeCallback` `:359`、
    SAFETY/SIT/READY の分岐)
  - `robot_driver/include/robot_driver/robot_driver.hpp`(`SIT=0` ほか enum、`:287`)
  - `robot_driver/src/robot_driver_utils.cpp`(`loadMotorCommandMsg`)
- 最終トルク合成(ros2_control)
  - `quad_simulator/gazebo_plugins/src/controller_plugin.cpp`
    (`effort_controllers::QuadController::update`)
  - `quad_simulator/gazebo_plugins/controller_plugin.xml`(plugin 宣言)
- パラメータ
  - `quad_utils/config/go2.yaml`(`robot_driver` ゲイン、`motor_limits`、
    `controller_manager`)
  - `robot_driver/config/robot_driver.yaml`(レート、タイムアウト、フィルタ係数)
- 起動
  - `quad_utils/launch/robot_driver.py` / `quad_mujoco_bringup.py`
    (`launch_robot_driver`)
