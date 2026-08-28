# 01 — Quadruped-PyMPC 実行順序トレース（nominal構成）

日付: 2026-08-25
対象: `external/Quadruped-PyMPC`（`mpc_params['type'] = 'nominal'`、デフォルト構成）
関連: [AGENTS.md](../../AGENTS.md) の「Canonical execution path」節（ファイル単位の要約はそちら、本ファイルは関数呼び出し単位の詳細）

## 何をしたか

`external/Quadruped-PyMPC` の主要ファイルを実際に読み、1制御サイクルの処理順序を
ファイル名・関数名・役割つきで時系列に整理した。目的は、このコードベースを
改造する前に「どこから読めば全体が追えるか」を固定すること。

読んだファイル:
- `simulation/simulation.py`
- `quadruped_pympc/quadruped_pympc_wrapper.py`
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/interfaces/srbd_controller_interface.py`
- `quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py`（`compute_control` 全体）
- `quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py`（クラス構造のみ）

## A. 起動・初期化フェーズ（ループに入る前、1回だけ）

すべて `simulation/simulation.py::run_simulation()` の中で順に実行される。

| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| A1 | `QuadrupedEnv` | `__init__`（`QuadrupedEnv(...)`） | MuJoCoのロボット・地形環境を生成 | `gym_quadruped`（外部） |
| A2 | `QuadrupedEnv`（`env`） | `reset`（`env.reset(random=False)`） | シミュレーション初期状態にリセット | `simulation/simulation.py` |
| A3 | `QuadrupedPyMPC_Wrapper` | `__init__` | 以下を生成しコントローラ一式を束ねる | `quadruped_pympc/quadruped_pympc_wrapper.py` |
| A3-1 | `SRBDControllerInterface` | `__init__` | `mpc_params['type']` を見て `Acados_NMPC_Nominal` 等の具象コントローラを選択・生成（＝acadosソルバーのビルド/ロード） | `interfaces/srbd_controller_interface.py` |
| A3-2 | `WBInterface` | `__init__` | 下記のヘルパー一式を生成 | `interfaces/wb_interface.py` |
| A3-2-1 | `PeriodicGaitGenerator` | `__init__`（`PeriodicGaitGenerator(...)`） | 歩容（duty factor, step freq, gait type）の位相管理器を生成 | `helpers/periodic_gait_generator.py` |
| A3-2-2 | `FootholdReferenceGenerator` | `__init__`（`FootholdReferenceGenerator(...)`） | 着地点の基準生成器 | `helpers/foothold_reference_generator.py` |
| A3-2-3 | `SwingTrajectoryController` | `__init__`（`SwingTrajectoryController(...)`） | 遊脚軌道＋PD制御器 | `helpers/swing_trajectory_controller.py` |
| A3-2-4 | `TerrainEstimator` | `__init__`（`TerrainEstimator()`） | 地形勾配・高さ推定器 | `helpers/terrain_estimator.py` |
| A3-2-5 | `InverseKinematicsNumeric` | `__init__`（`InverseKinematicsNumeric()`） | 逆運動学ソルバー | `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py` |
| A3-2-6 | `VelocityModulator` / `EarlyStanceDetector` | `__init__`（`VelocityModulator()` / `EarlyStanceDetector(...)`） | 速度指令の補正／早期接地検知 | `helpers/velocity_modulator.py` / `early_stance_detector.py` |

## B. 制御ループ（1シミュレーションステップごとに繰り返し）

### B0. プラント観測（`simulation.py` 内、L172–205）
`env.feet_pos/feet_vel/hip_positions/base_lin_vel/base_ang_vel/...` などを読み出し、
状態量 $(p, v, R, \omega, p_i, J_i, M_i, q, \dot q)$ を用意する。

### B1. `QuadrupedPyMPC_Wrapper.compute_actions(...)` 呼び出し
`quadruped_pympc_wrapper.py::compute_actions` (L50) が全体の入口。

#### B1-1. 状態・参照の更新 — `WBInterface.update_state_and_reference(...)` (`wb_interface.py` L108)
| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| ① | `TerrainEstimator` | `self.terrain_computation.compute_terrain_estimation(...)` | 地形のroll/pitch/高さを推定 | `helpers/terrain_estimator.py` |
| ② | `WBInterface` | （dict構築） | `state_current` を組み立て（CoM位置、速度、姿勢、脚位置/関節角） | `interfaces/wb_interface.py` |
| ③ | `VelocityModulator` | `self.vm.modulate_velocities(...)`（`vm.activated`時のみ） | 危険姿勢時に速度指令を補正 | `helpers/velocity_modulator.py` |
| ④ | `PeriodicGaitGenerator` | `self.pgg.update_start_and_stop(...)`（該当時のみ） | 静止時に歩容を止める判定 | `helpers/periodic_gait_generator.py` |
| ⑤ | `PeriodicGaitGenerator` | `self.pgg.run(simulation_dt, step_freq)` | 歩容位相を1step進める | `helpers/periodic_gait_generator.py` |
| ⑥ | `PeriodicGaitGenerator` | `self.pgg.compute_contact_sequence(...)` | ホライズン分の接地シーケンスを生成 | `helpers/periodic_gait_generator.py` |
| ⑦ | `FootholdReferenceGenerator` | `self.frg.update_lift_off_positions(...)` | 離地位置を更新 | `helpers/foothold_reference_generator.py` |
| ⑧ | `FootholdReferenceGenerator` | `self.frg.update_touch_down_positions(...)` | 着地位置を更新 | `helpers/foothold_reference_generator.py` |
| ⑨ | `FootholdReferenceGenerator` | `self.frg.compute_footholds_reference(...)` | Raibertヒューリスティックで着地基準点を計算 | `helpers/foothold_reference_generator.py` |
| ⑩ | `VisualFootholdAdaptation` | `self.vfa.compute_adaptation(...)` / `get_footholds_adapted(...)`（VFA有効時のみ） | 高さマップに応じて着地点を補正 | `helpers/visual_foothold_adaptation.py` |
| ⑪ | `WBInterface` | （dict構築） | `ref_state`（参照位置・速度・姿勢・4脚の着地参照など）を組み立て | `interfaces/wb_interface.py` |
| ⑫ | `SwingTrajectoryController` | `self.stc.check_touch_down_condition(...)` | step frequency最適化の実行タイミング判定（`optimize_swing`） | `helpers/swing_trajectory_controller.py` |

戻り値: `state_current, ref_state, contact_sequence, step_height, optimize_swing`

#### B1-2. OCP求解 — 条件付き（`mpc_frequency` に基づき間引き）
`quadruped_pympc_wrapper.py` L134: `if step_num % round(1/(mpc_frequency*simulation_dt)) == 0:`

| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| ① | `SRBDControllerInterface` | `compute_control(...)` (L85) | typeに応じて具象コントローラへ委譲 | `interfaces/srbd_controller_interface.py` |
| ② | `Acados_NMPC_Nominal` | `compute_control(...)` (L1138) | 実際のNLP求解本体 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-a | `Acados_NMPC_Nominal` | `self.perform_scaling(state, reference, constraint)` (L1116) | 状態・参照のスケーリング | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-b | `Acados_NMPC_Nominal` | ループ内 `self.acados_ocp_solver.set(j, "yref", yref)` (L1166〜) | 各ステージの参照（位置・速度・姿勢・4脚着地・GRF）をacadosに設定 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-c | `Acados_NMPC_Nominal` | `self.acados_ocp_solver.set(j, "p", param)` (L1330) | 接地フラグ・摩擦係数・外乱推定・慣性・質量などのパラメータを設定 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-d | `Acados_NMPC_Nominal` | `self.acados_ocp_solver.set(0, "lbx"/"ubx", state_acados)` (L1419) | 現在状態を初期状態制約として設定 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-e | `Acados_NMPC_Nominal` | `self.set_warm_start(...)`（有効時） (L1048) | ウォームスタート値を設定 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-f | `Acados_NMPC_Nominal` | `self.set_stage_constraint(...)`（foothold/stability制約有効時） (L562) | 着地点・安定性制約を設定 | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-g | `Acados_NMPC_Nominal` | `self.acados_ocp_solver.solve()` (L1445/1450) | NLPを解く（RTIなら2段階目のみ） | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ②-h | `Acados_NMPC_Nominal` | `self.acados_ocp_solver.get(0, "u")` → GRF抽出、`get(j, "x")` → 次着地点抽出 (L1455〜) | 第0段のGRFと次の着地点だけを取り出す（＝Receding Horizon） | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` |
| ③ | `SRBDControllerInterface` | `self.compute_RTI()` (`use_RTI`時のみ) | 次回のための線形化（フィードバック遅延低減） | `interfaces/srbd_controller_interface.py` |

戻り値: `nmpc_GRFs, nmpc_footholds, nmpc_joints_pos, nmpc_joints_vel, nmpc_joints_acc, best_sample_freq, nmpc_predicted_state`

#### B1-3. 歩容周波数の最適化 — 条件付き（`optimize_step_freq=True` の場合のみ）
`interfaces/srbd_batched_controller_interface.py::SRBDBatchedControllerInterface.optimize_gait(...)` —
複数の歩容候補でバッチOCPを解き、最良の step frequency を選ぶ。

#### B1-4. GRF/着地点 → 関節トルク — `WBInterface.compute_stance_and_swing_torque(...)` (`wb_interface.py` L307)
| 順 | クラス | 関数 | 役割 | ファイル |
|---|---|---|---|---|
| ① | `SwingTrajectoryController` | `self.stc.regenerate_swing_trajectory_generator(...)`（`optimize_swing==1`時のみ） | 最適step freqを反映 | `helpers/swing_trajectory_controller.py` |
| ② | `EarlyStanceDetector` | `self.esd.update_detection(...)` | 早期接地（reflex）を検知 | `helpers/early_stance_detector.py` |
| ③ | `WBInterface` | 直接計算（L372–375） | 立脚脚: $\tau = -J^{T}F$（`nmpc_GRFs` を関節トルクへ変換） | `interfaces/wb_interface.py` |
| ④ | `SwingTrajectoryController` | `self.stc.update_swing_time(...)` | 遊脚の経過時間を更新 | `helpers/swing_trajectory_controller.py` |
| ⑤ | `SwingTrajectoryController` | 各脚ループ: `self.stc.compute_swing_control_cartesian_space(...)`（遊脚のみ） | Cartesian PD + feedback linearization で遊脚トルクを計算 | `helpers/swing_trajectory_controller.py` |
| ⑥ | `WBInterface` | （摩擦補償が有効なら）`tau -= legs_qfrc_passive` | 摩擦補償 | `interfaces/wb_interface.py` |
| ⑦ | `InverseKinematicsNumeric` | `self.ik.compute_solution(...)` | 目標脚位置から目標関節角を逆運動学で解く | `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py` |
| ⑧ | `WBInterface` | `np.linalg.pinv(J) @ des_foot_vel` | 目標関節速度を計算 | `interfaces/wb_interface.py` |
| ⑨ | `WBInterface` | クリップ処理（L448–468） | 目標関節角/速度の急変を制限 | `interfaces/wb_interface.py` |

戻り値: `tau, des_joints_pos, des_joints_vel`

### B2. 観測値の保存（`quadruped_pympc_wrapper.py` L206–243）
`self.quadrupedpympc_observables` に `ref_base_height`, `nmpc_GRFs`, `swing_time` 等を格納
（`get_obs()` で外部から取得可能）。

### B3. トルク制限・アクチュエータ適用（`simulation.py` L237–251）
- `np.clip(tau, tau_min, tau_max)` でトルク上限を適用
- `env.step(action=action)` — MuJoCoを1ステップ進める（実際の物理シミュレーション）

### B4. 履歴保存・描画・エピソード終了処理（`simulation.py` L253–332）
- `quadrupedpympc_wrapper.get_obs()` で観測を取得しログへ追加
- 一定周期で `plot_swing_mujoco(...)` / `env.render()`（可視化）
- ステップ数上限 or 終了条件で `env.reset(random=True)` → `quadrupedpympc_wrapper.reset(...)` →
  `WBInterface.reset(...)`（B1のヘルパー群の内部状態をクリア）で次エピソードへ

## 全体像（時系列サマリ）

```
起動: env生成 → QuadrupedPyMPC_Wrapper生成（acadosソルバー含む）
  ↓
[毎シムステップ]
  観測取得
    → WBInterface.update_state_and_reference
         (地形推定 → 速度補正 → 歩容位相更新 → 接地列生成 → 着地点更新/参照計算 → VFA)
    → [mpc_frequency周期のみ] SRBDControllerInterface.compute_control
         → Acados_NMPC_Nominal.compute_control
              (yref設定 → param設定 → 初期状態制約 → warm start → 制約設定 → solve → GRF/着地点抽出)
    → [任意] SRBDBatchedControllerInterface.optimize_gait
    → WBInterface.compute_stance_and_swing_torque
         (立脚トルク=-J^T F → 遊脚PD/線形化 → IKで目標関節角 → クリップ)
    → トルク制限 → env.step()  ← 実際にロボットが動く
    → 観測ログ・描画
  ↓
終了条件で env.reset() + wrapper.reset()
```

## 次にやること（未着手）

- 他の `mpc_params['type']`（`input_rates`, `lyapunov`, `kinodynamic`, `sampling`）の
  差分トレースはまだ行っていない。必要になったら本フォルダに `02_xxx.md` として追加する。
