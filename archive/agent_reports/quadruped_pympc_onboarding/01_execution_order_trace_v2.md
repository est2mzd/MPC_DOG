# 01 — Quadruped-PyMPC 実行順序トレース（nominal構成）

日付: 2026-08-25

対象: `external/Quadruped-PyMPC`（`mpc_params['type'] = 'nominal'`、デフォルト構成）

関連: [AGENTS.md](../../AGENTS.md) の「Canonical execution path」節  
ファイル単位の要約は `AGENTS.md`、本ファイルは関数呼び出し単位の詳細を扱う。

---

## 1. 目的と調査範囲

`external/Quadruped-PyMPC` の主要ファイルを実際に読み、1制御サイクルの処理順序を、ファイル名・クラス名・関数名・役割つきで時系列に整理した。

目的は、コードを改造する前に「どこから、どの順番で読めば全体を追えるか」を固定することである。

### 読んだファイル

- `simulation/simulation.py`
- `quadruped_pympc/quadruped_pympc_wrapper.py`
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/interfaces/srbd_controller_interface.py`
- `quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py`
  - `compute_control()` 全体
- `quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py`
  - クラス構造のみ

---

## 2. 実行周期の区分

処理は、次の3種類に分かれる。

| 区分 | 実行タイミング | 主な処理 |
|---|---|---|
| 初期化 | 起動時に1回 | MuJoCo環境、MPC、歩容・着地点・遊脚制御器などの生成 |
| 毎シミュレーションステップ | 制御ループごと | 観測、歩容更新、接地列・参照生成、トルク計算、`env.step()` |
| MPC周期のみ | `mpc_frequency` に基づく間引き時 | OCPパラメータ設定、NLP求解、GRF・着地点抽出 |

---

## A. 起動・初期化フェーズ

ループへ入る前に、`simulation/simulation.py::run_simulation()` の中で1回だけ実行される。

| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| A1 | `QuadrupedEnv` | `__init__`（`QuadrupedEnv(...)`） | MuJoCoのロボット・地形環境を生成 | `gym_quadruped`（外部） |
| A2 | `QuadrupedEnv`（`env`） | `reset`（`env.reset(random=False)`） | シミュレーション初期状態にリセット | `simulation/simulation.py` |
| A3 | `QuadrupedPyMPC_Wrapper` | `__init__` | 以下のコントローラ一式を生成して束ねる | `quadruped_pympc/quadruped_pympc_wrapper.py` |
| A3-1 | `SRBDControllerInterface` | `__init__` | `mpc_params['type']`を見て`Acados_NMPC_Nominal`などの具象コントローラを選択・生成。acadosソルバーをビルドまたはロード | `interfaces/srbd_controller_interface.py` |
| A3-2 | `WBInterface` | `__init__` | 以下のヘルパー一式を生成 | `interfaces/wb_interface.py` |
| A3-2-1 | `PeriodicGaitGenerator` | `__init__` | duty factor、step frequency、gait typeを使う歩容位相管理器を生成 | `helpers/periodic_gait_generator.py` |
| A3-2-2 | `FootholdReferenceGenerator` | `__init__` | 着地点の基準生成器を生成 | `helpers/foothold_reference_generator.py` |
| A3-2-3 | `SwingTrajectoryController` | `__init__` | 遊脚軌道生成器とPD制御器を生成 | `helpers/swing_trajectory_controller.py` |
| A3-2-4 | `TerrainEstimator` | `__init__` | 地形勾配・高さ推定器を生成 | `helpers/terrain_estimator.py` |
| A3-2-5 | `InverseKinematicsNumeric` | `__init__` | 数値逆運動学ソルバーを生成 | `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py` |
| A3-2-6 | `VelocityModulator` / `EarlyStanceDetector` | `__init__` | 速度指令の補正器／早期接地検知器を生成 | `helpers/velocity_modulator.py` / `helpers/early_stance_detector.py` |

初期化後、`QuadrupedPyMPC_Wrapper` がSRBDコントローラと`WBInterface`を保持し、毎ステップの処理をまとめて呼び出す。

---

## B. 制御ループ

以下を1シミュレーションステップごとに繰り返す。

### B0. プラント観測

場所: `simulation/simulation.py` L172–205

MuJoCo環境から、次の観測値を読み出す。

- `env.feet_pos`
- `env.feet_vel`
- `env.hip_positions`
- `env.base_lin_vel`
- `env.base_ang_vel`
- その他、姿勢、関節状態、Jacobian、脚質量行列など

これらから、後段で使用する状態量

$$
(p, v, R, \omega, p_i, J_i, M_i, q, \dot q)
$$

を用意する。

### B1. `QuadrupedPyMPC_Wrapper.compute_actions(...)`

場所: `quadruped_pympc/quadruped_pympc_wrapper.py::compute_actions()` L50

ここが、観測値を受け取って最終的な関節トルクを返すまでの入口である。

---

### B1-1. 状態・参照・接地予定の更新

関数: `WBInterface.update_state_and_reference(...)`  
場所: `interfaces/wb_interface.py` L108

この処理は、MPC求解の間引き条件より前に呼ばれる。

| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| 1 | `TerrainEstimator` | `compute_terrain_estimation(...)` | 地形のroll、pitch、高さを推定 | `helpers/terrain_estimator.py` |
| 2 | `WBInterface` | dict構築 | `state_current`を組み立てる。CoM位置、速度、姿勢、脚位置、関節角などを格納 | `interfaces/wb_interface.py` |
| 3 | `VelocityModulator` | `modulate_velocities(...)` | `vm.activated`時のみ、危険姿勢時の速度指令を補正 | `helpers/velocity_modulator.py` |
| 4 | `PeriodicGaitGenerator` | `update_start_and_stop(...)` | 該当時のみ、静止時に歩容を止めるか判定 | `helpers/periodic_gait_generator.py` |
| 5 | `PeriodicGaitGenerator` | `run(simulation_dt, step_freq)` | 歩容位相を1シミュレーションステップ進める | `helpers/periodic_gait_generator.py` |
| 6 | `PeriodicGaitGenerator` | `compute_contact_sequence(...)` | MPCホライズン分の接地シーケンスを生成 | `helpers/periodic_gait_generator.py` |
| 7 | `FootholdReferenceGenerator` | `update_lift_off_positions(...)` | 離地位置を更新 | `helpers/foothold_reference_generator.py` |
| 8 | `FootholdReferenceGenerator` | `update_touch_down_positions(...)` | 着地位置を更新 | `helpers/foothold_reference_generator.py` |
| 9 | `FootholdReferenceGenerator` | `compute_footholds_reference(...)` | Raibertヒューリスティックで着地基準点を計算 | `helpers/foothold_reference_generator.py` |
| 10 | `VisualFootholdAdaptation` | `compute_adaptation(...)` / `get_footholds_adapted(...)` | VFA有効時のみ、高さマップに応じて着地点を補正 | `helpers/visual_foothold_adaptation.py` |
| 11 | `WBInterface` | dict構築 | `ref_state`を組み立てる。参照位置、速度、姿勢、4脚の着地参照などを格納 | `interfaces/wb_interface.py` |
| 12 | `SwingTrajectoryController` | `check_touch_down_condition(...)` | step frequency最適化の実行タイミングを判定し、`optimize_swing`を生成 | `helpers/swing_trajectory_controller.py` |

戻り値:

```text
state_current
ref_state
contact_sequence
step_height
optimize_swing
```

---

### B1-2. OCP求解

この処理は、`mpc_frequency`に基づいて間引かれる。

場所: `quadruped_pympc_wrapper.py` L134

```python
if step_num % round(1 / (mpc_frequency * simulation_dt)) == 0:
```

| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| 1 | `SRBDControllerInterface` | `compute_control(...)` | `mpc_params['type']`に対応する具象コントローラへ処理を委譲 | `interfaces/srbd_controller_interface.py` L85 |
| 2 | `Acados_NMPC_Nominal` | `compute_control(...)` | 実際のNLP求解処理 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` L1138 |
| 2-a | `Acados_NMPC_Nominal` | `perform_scaling(state, reference, constraint)` | 状態・参照をスケーリング | 同上 L1116 |
| 2-b | `Acados_NMPC_Nominal` | `set(j, "yref", yref)` | 各ステージの位置・速度・姿勢・4脚着地点・GRF参照をacadosへ設定 | 同上 L1166以降 |
| 2-c | `Acados_NMPC_Nominal` | `set(j, "p", param)` | 接地フラグ、摩擦係数、外乱推定、慣性、質量などをステージパラメータとして設定 | 同上 L1330 |
| 2-d | `Acados_NMPC_Nominal` | `set(0, "lbx"/"ubx", state_acados)` | 現在状態を初期状態制約として設定 | 同上 L1419 |
| 2-e | `Acados_NMPC_Nominal` | `set_warm_start(...)` | 有効時のみ、ウォームスタート値を設定 | 同上 L1048 |
| 2-f | `Acados_NMPC_Nominal` | `set_stage_constraint(...)` | footholdまたはstability制約が有効な場合、着地点・安定性制約を設定 | 同上 L562 |
| 2-g | `Acados_NMPC_Nominal` | `acados_ocp_solver.solve()` | NLPを求解。RTI使用時は2段階目のみ | 同上 L1445 / L1450 |
| 2-h | `Acados_NMPC_Nominal` | `get(0, "u")` / `get(j, "x")` | 第0段のGRFと次の着地点だけを取り出す。Receding Horizonとして使用 | 同上 L1455以降 |
| 3 | `SRBDControllerInterface` | `compute_RTI()` | `use_RTI`時のみ、次回のための線形化を行い、フィードバック遅延を低減 | `interfaces/srbd_controller_interface.py` |

戻り値:

```text
nmpc_GRFs
nmpc_footholds
nmpc_joints_pos
nmpc_joints_vel
nmpc_joints_acc
best_sample_freq
nmpc_predicted_state
```

---

### B1-3. 歩容周波数の最適化

条件: `optimize_step_freq = True`

関数:

```text
SRBDBatchedControllerInterface.optimize_gait(...)
```

ファイル:

```text
interfaces/srbd_batched_controller_interface.py
```

複数の歩容候補に対してバッチOCPを解き、最良のstep frequencyを選ぶ。

---

### B1-4. GRF・着地点から関節トルクへの変換

関数: `WBInterface.compute_stance_and_swing_torque(...)`  
場所: `interfaces/wb_interface.py` L307

| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| 1 | `SwingTrajectoryController` | `regenerate_swing_trajectory_generator(...)` | `optimize_swing == 1`時のみ、最適step frequencyを遊脚軌道へ反映 | `helpers/swing_trajectory_controller.py` |
| 2 | `EarlyStanceDetector` | `update_detection(...)` | 早期接地を検知してreflex処理に使用 | `helpers/early_stance_detector.py` |
| 3 | `WBInterface` | 直接計算 | 立脚脚について $\tau=-J^TF$ を計算し、`nmpc_GRFs`を関節トルクへ変換 | `interfaces/wb_interface.py` L372–375 |
| 4 | `SwingTrajectoryController` | `update_swing_time(...)` | 各遊脚の経過時間を更新 | `helpers/swing_trajectory_controller.py` |
| 5 | `SwingTrajectoryController` | `compute_swing_control_cartesian_space(...)` | 遊脚についてCartesian PDとfeedback linearizationでトルクを計算 | `helpers/swing_trajectory_controller.py` |
| 6 | `WBInterface` | `tau -= legs_qfrc_passive` | 摩擦補償が有効な場合に実行 | `interfaces/wb_interface.py` |
| 7 | `InverseKinematicsNumeric` | `compute_solution(...)` | 目標脚位置から目標関節角を数値逆運動学で求める | `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py` |
| 8 | `WBInterface` | `np.linalg.pinv(J) @ des_foot_vel` | 目標足先速度から目標関節速度を計算 | `interfaces/wb_interface.py` |
| 9 | `WBInterface` | クリップ処理 | 目標関節角・速度の急変を制限 | `interfaces/wb_interface.py` L448–468 |

戻り値:

```text
tau
des_joints_pos
des_joints_vel
```

---

### B2. 観測値の保存

場所: `quadruped_pympc_wrapper.py` L206–243

`self.quadrupedpympc_observables`へ、次のような観測値を保存する。

- `ref_base_height`
- `nmpc_GRFs`
- `swing_time`
- その他の制御・可視化用観測値

保存した値は`get_obs()`から外部取得できる。

### B3. トルク制限とMuJoCoへの適用

場所: `simulation/simulation.py` L237–251

1. `np.clip(tau, tau_min, tau_max)`でトルク上限を適用する。
2. `env.step(action=action)`でMuJoCoを1ステップ進める。

`env.step()`が、計算した制御入力を物理シミュレーションへ適用し、実際にロボット状態を変化させる箇所である。

### B4. 履歴保存・描画・エピソード終了処理

場所: `simulation/simulation.py` L253–332

- `quadrupedpympc_wrapper.get_obs()`で観測値を取得し、ログへ追加する。
- 一定周期で`plot_swing_mujoco(...)`または`env.render()`を呼び出す。
- ステップ数上限または終了条件に達すると、次の順で次エピソードへ移る。

```text
env.reset(random=True)
→ quadrupedpympc_wrapper.reset(...)
→ WBInterface.reset(...)
→ B1で使用するヘルパー群の内部状態をクリア
```

---

## 3. 全体像

```text
起動時
  QuadrupedEnv生成
  → env.reset(random=False)
  → QuadrupedPyMPC_Wrapper生成
       ├─ SRBDControllerInterface生成
       │    └─ Acados_NMPC_Nominal・acadosソルバー生成
       └─ WBInterface生成
            └─ 歩容・着地点・遊脚・地形・IKなどのヘルパー生成

毎シミュレーションステップ
  MuJoCoから観測取得
  → WBInterface.update_state_and_reference(...)
       地形推定
       → state_current生成
       → 速度補正
       → 歩容位相更新
       → contact_sequence生成
       → 離地・着地位置更新
       → 着地点参照生成
       → 任意でVFA
       → ref_state生成

  → mpc_frequency周期のみ
       SRBDControllerInterface.compute_control(...)
       → Acados_NMPC_Nominal.compute_control(...)
            scaling
            → yref設定
            → param設定
            → 初期状態制約
            → warm start
            → ステージ制約
            → solve
            → 第0段GRF・次着地点抽出

  → 任意でSRBDBatchedControllerInterface.optimize_gait(...)

  → WBInterface.compute_stance_and_swing_torque(...)
       立脚: -J^T F
       → 遊脚: Cartesian PD・feedback linearization
       → IKで目標関節角
       → Jacobian疑似逆行列で目標関節速度
       → クリップ

  → トルク上限制限
  → env.step(action=action)
  → ログ保存・描画

終了条件成立
  env.reset()
  → wrapper.reset()
  → WBInterface.reset()
```

---

## 4. 1制御サイクルの入出力サマリ

| 段階 | 主な入力 | 主な出力 |
|---|---|---|
| プラント観測 | MuJoCo内部状態 | ベース・足先・関節・Jacobian・質量行列など |
| 状態・参照更新 | 観測値、速度指令、歩容内部状態 | `state_current`、`ref_state`、`contact_sequence` |
| NMPC | 状態、参照、接地列、慣性など | GRF、着地点、予測状態など |
| 立脚・遊脚制御 | GRF、着地点、Jacobian、関節状態 | `tau`、目標関節角、目標関節速度 |
| プラント適用 | 制限後トルク | 次シミュレーションステップの状態 |

---

## 5. 次にやること（未着手）

他の`mpc_params['type']`について、nominalとの差分トレースはまだ行っていない。

- `input_rates`
- `lyapunov`
- `kinodynamic`
- `sampling`

必要になった段階で、本フォルダへ`02_xxx.md`として追加する。
