# Code Map and Call Graph

関数の呼出順と、標準設定で無効な経路の正本である。境界データと周期は[02](02_System_Architecture_and_Dataflow.md)。1行索引は[D](appendices/D_File_Function_Index.md)。学習の入口は[00](00_README.md)。

## 1. 主要ファイル

| ファイル | 主責務 |
|---|---|
| `simulation/simulation.py` | MuJoCo loop、状態取得、Torque入力 |
| `quadruped_pympc/quadruped_pympc_wrapper.py` | Controller全体統合 |
| `quadruped_pympc/interfaces/wb_interface.py` | 状態・参照、Gait/Foothold、Stance/Swing |
| `quadruped_pympc/interfaces/srbd_controller_interface.py` | MPC選択、入出力整形 |
| `quadruped_pympc/helpers/periodic_gait_generator.py` | 接地列 |
| `quadruped_pympc/helpers/foothold_reference_generator.py` | Nominal foothold |
| `quadruped_pympc/helpers/terrain_estimator.py` | 地形姿勢・高さ |
| `quadruped_pympc/helpers/visual_foothold_adaptation.py` | 地形適応Foothold（非blind） |
| `quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py` | MPC力学モデル |
| `quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py` | OCP・Solver |
| `quadruped_pympc/helpers/swing_trajectory_controller.py` | Swing追従Torque |
| `quadruped_pympc/helpers/early_stance_detector.py` | Early contact/reflex |
| `quadruped_pympc/config.py` | Robot、MPC、Gait、Simulation設定 |
| `gym_quadruped/quadruped_env.py` | MuJoCo API、User command、State API |

境界データの正本は[02](02_System_Architecture_and_Dataflow.md)。指令の正本は[03](03_User_Command_and_Reference_Generation.md)。関数1行索引は[D](appendices/D_File_Function_Index.md)。

## 2. 標準経路のCall graph

既定は`mpc_params['type']='nominal'`、`visual_foothold_adaptation='blind'`、`optimize_step_freq=False`、`use_RTI=False`、`reflex_trigger_mode=False`。

```text
run_simulation
├─ env.reset → QuadrupedEnv._sample_ref_vel
├─ env.render  ※render=Trueのときだけ _key_callback を登録
│  毎step:
├─ QuadrupedEnv getters（feet_pos, com, qpos, J, ...）
├─ env.target_base_vel
└─ QuadrupedPyMPC_Wrapper.compute_actions
   ├─ WBInterface.update_state_and_reference
   │  ├─ TerrainEstimator.compute_terrain_estimation
   │  ├─ state_current 組み立て
   │  ├─ VelocityModulator.modulate_velocities
   │  ├─ PeriodicGaitGenerator.run(simulation_dt)   ※戻り値は捨て、位相だけ進める
   │  ├─ PeriodicGaitGenerator.compute_contact_sequence
   │  ├─ FootholdReferenceGenerator.update_lift_off_positions
   │  ├─ FootholdReferenceGenerator.update_touch_down_positions
   │  ├─ FootholdReferenceGenerator.compute_footholds_reference
   │  ├─ 地形roll/pitchで ref_base_lin_vel を回転
   │  └─ ref_state 組み立て
   ├─ if step_num % 5 == 0:
   │  └─ SRBDControllerInterface.compute_control
   │     └─ Acados_NMPC_Nominal.compute_control
   └─ WBInterface.compute_stance_and_swing_torque
      ├─ stance: -J.T @ GRF
      ├─ SwingTrajectoryController.update_swing_time
      ├─ SwingTrajectoryController.compute_swing_control_cartesian_space  ※遊脚
      ├─ tau -= legs_qfrc_passive
      └─ InverseKinematicsNumeric.compute_solution  ※tauには未使用
├─ np.clip(tau, 0.9 * actuator_ctrlrange)
├─ action[legs_tau_idx.*] = tau.*
└─ QuadrupedEnv.step → mjData.ctrl = action → mujoco.mj_step
```

`joints_pos`について: `simulation.py`は

```python
joints_pos = LegsAttr(FL=legs_qvel_idx.FL, ...)
```

としてqvel indexを渡す。`state_current['joint_*']`に入るが、nominal MPCは読まない。

## 3. 理論とファイル

| 理論 | 正本コード |
|---|---|
| Trot位相 | `periodic_gait_generator.py` |
| Foothold heuristic | `foothold_reference_generator.py` |
| SRBD dynamics | `centroidal_model_nominal.py` |
| Cost/constraints | `centroidal_nmpc_nominal.py` |
| GRF output mask | `srbd_controller_interface.py` |
| Stance torque | `wb_interface.py` |
| Swing torque | `swing_trajectory_controller.py` |
| MuJoCo step | `gym_quadruped/quadruped_env.py` |

## 4. 標準設定で無効、または到達不能な経路

| 経路 | 無効化条件 | 根拠 |
|---|---|---|
| `_key_callback` | `render=False`、またはviewer未起動 | 登録は`render()`内だけ |
| `simulation_params['mode']` | どのPython経路からも未参照 | `config.py`の定義のみ |
| `HeightMap` / `VisualFootholdAdaptation` | `'blind'` | `simulation.py`、`wb_interface.py` |
| `pgg.update_start_and_stop` | `start_and_stop_activated=False` | 既定False。TrueにするのはROS2 console |
| `SRBDBatchedControllerInterface.optimize_gait` | `optimize_step_freq=False` | Wrapperがオブジェクトを作らない |
| `compute_RTI` | `use_RTI=False` | Wrapperの条件分岐 |
| Early stance / reflex | `reflex_trigger_mode=False` → `activated=False` | `early_stance_detector.py` |
| `compute_swing_control_joint_space` | `wb_interface`から未呼出 | 定義のみ |
| 関節PD加算 | コメントアウト | `quadruped_pympc_wrapper.py` |
| `nmpc_predicted_state`をIK初期値に使う行 | コメントアウト | `wb_interface.py` |
| sampling / input_rates / lyapunov / kinodynamic | `type='nominal'` | `srbd_controller_interface.py` |
| `use_DDP`, `use_integrators`, `use_warm_start`, `use_foothold_constraints`, 安定制約 | いずれもFalse | `config.py` |
| `use_nonuniform_discretization` | False。接触列は`dt=0.02`, N=12 | `wb_interface.py` |
| 非ゼロ`external_wrenches` | Wrapperが未渡し | 既定`zeros(6,)` |

## 5. 注意すべき実装点

- `WBInterface`は一般的な全身QP-WBCと同一ではない。正本は[10](10_Stance_and_Swing_Control.md)。
- 遊脚GRFの3段（力学Gate / 摩擦常時 / 出力Mask）の正本は[09](09_MPC_Output_and_Receding_Horizon.md) §6。OCP内等式ゼロは無い。
- 関節PD加算は**実装あり・標準無効**（Wrapperでコメントアウト）。
- `blind`設定では地形知覚・穴回避はない。
- Gait frequency最適化は設定で無効が既定。
- コメントやDocstringの次元がコード更新に追随していない可能性がある。
- wrapper観測分岐の `ref_foot_FL_constraints` は typo。実キーと制御経路は[03](03_User_Command_and_Reference_Generation.md)。

## 6. Cursor確認課題

このCall graphを静的解析で再生成し、実際に未使用・条件分岐で到達不能な経路を明示する。本章はその再生成結果である。設定やコミットが変わったら差分を取り直す。
