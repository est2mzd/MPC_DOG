# Log 03: 標準実行経路 / Call graph

対応プロンプト: 標準MuJoCoシミュレーションが通るPythonファイルと関数を確定する。本文未修正。

標準条件: `config.py` の `type='nominal'`, `gait='trot'`, `visual_foothold_adaptation='blind'`, `optimize_step_freq=False`, `use_RTI=False`, `reflex_trigger_mode=False`。`run_simulation` 既定 `render=True`, `base_vel_command_type='human'`。

## A. 実際の Call graph

初期化:

```text
simulation.py :: if __name__
  -> import quadruped_pympc.config as cfg
  -> run_simulation(qpympc_cfg=cfg)
       -> QuadrupedEnv.__init__
       -> QuadrupedEnv.reset(random=False)
            -> _sample_ref_vel()
            -> _set_ground_friction()
       -> [render=True] QuadrupedEnv.render()  # key_callback登録
       -> QuadrupedPyMPC_Wrapper.__init__
            -> SRBDControllerInterface.__init__
                 -> Acados_NMPC_Nominal.__init__
                      -> Centroidal_Model_Nominal.export_robot_model()
                      -> AcadosOcpSolver(...)
            -> WBInterface.__init__
                 -> PeriodicGaitGenerator / FootholdReferenceGenerator /
                    SwingTrajectoryController / TerrainEstimator /
                    InverseKinematicsNumeric / VelocityModulator /
                    EarlyStanceDetector
```

毎 `dt=0.002`（500 Hz）:

```text
run_simulation loop
  -> env.target_base_vel(frame='world')
  -> env.com / base_pos / base_lin_vel / base_ori_euler_xyz / ...
  -> QuadrupedPyMPC_Wrapper.compute_actions
       -> WBInterface.update_state_and_reference
            -> TerrainEstimator.compute_terrain_estimation
            -> [vm.activated] VelocityModulator.modulate_velocities
            -> [start_and_stop_activated] PeriodicGaitGenerator.update_start_and_stop  # 標準False
            -> PeriodicGaitGenerator.run
            -> PeriodicGaitGenerator.compute_contact_sequence
            -> FootholdReferenceGenerator.update_lift_off/touch_down
            -> FootholdReferenceGenerator.compute_footholds_reference
            -> [not blind] VisualFootholdAdaptation  # 標準不通
       -> [step_num % 5 == 0] SRBDControllerInterface.compute_control
            -> Acados_NMPC_Nominal.compute_control
                 -> perform_scaling / set yref / set p / solve / get u0,x
       -> [optimize_step_freq] SRBDBatchedControllerInterface.optimize_gait  # 標準不通
       -> WBInterface.compute_stance_and_swing_torque
            -> SwingTrajectoryController / EarlyStanceDetector / IK
  -> np.clip(tau, 0.9*ctrlrange)
  -> env.step(action)  # mj_step
```

## B. 標準実行経路

| 順序 | ファイル | クラス・関数 | 実行条件 | 入力 | 出力 | 次の処理 |
|---|---|---|---|---|---|---|
| 1 | `simulation.py` | `run_simulation` | 起動時 | `cfg` | env, wrapper | reset |
| 2 | `config.py` | モジュール読込 | import時 | なし | `mpc_params`, `simulation_params` | 全下流 |
| 3 | `quadruped_env.py` | `QuadrupedEnv.__init__` | 1回 | robot/scene/dt | `mjModel`/`mjData` | reset |
| 4 | `quadruped_env.py` | `reset` | 1回/episode | `random=False` | qpos keyframe, 指令初期化 | loop |
| 5 | `quadruped_pympc_wrapper.py` | `QuadrupedPyMPC_Wrapper.__init__` | 1回 | 初期足位置 | controller群 | loop |
| 6 | `simulation.py` | 主loop | 毎0.002 s | `mjData` | 状態ベクトル | wrapper |
| 7 | `quadruped_env.py` | `target_base_vel` | 毎step | `_ref_*_H` | `ref_base_lin/ang_vel` W | wrapper |
| 8 | `wb_interface.py` | `update_state_and_reference` | 毎step | 状態+指令 | `state_current`, `ref_state`, `contact_sequence` | MPC |
| 9 | `periodic_gait_generator.py` | `run` + `compute_contact_sequence` | 毎step | `dt`, `step_freq` | `(4,12)` 接触列 | Foothold/MPC |
| 10 | `foothold_reference_generator.py` | `compute_footholds_reference` | 毎step | hip, vel | `ref_feet_pos` | `ref_state` |
| 11 | `centroidal_nmpc_nominal.py` | `compute_control` | 100 Hz | 状態/参照/接触 | GRF, footholds | 低レベル |
| 12 | `wb_interface.py` | `compute_stance_and_swing_torque` | 毎step | GRF, footholds, J | `tau` | clip |
| 13 | `simulation.py` | `np.clip` | 毎step | `tau` | 0.9 `ctrlrange` | `env.step` |
| 14 | `quadruped_env.py` | `step` | 毎step | `action` `(12,)` | 次`mjData` | 次周期 |

## C. 標準設定で通らない経路

| 機能 | ファイル・関数 | 無効条件 | 用途 |
|---|---|---|---|
| VFA / HeightMap | `visual_foothold_adaptation.py`, `HeightMap` | `visual_foothold_adaptation=='blind'` | 地形適応 |
| start/stop gait | `update_start_and_stop` | `start_and_stop_activated=False`（ROS2 consoleのみTrue） | 停止時full stance |
| 周波数最適化 | `SRBDBatchedControllerInterface.optimize_gait` | `optimize_step_freq=False` | 候補周波数バッチ |
| RTI | `compute_RTI` | `use_RTI=False` | 遅延短縮 |
| DDP | `create_ocp_solver_description` | `use_DDP=False` | 別NLP |
| 積分補償 | `use_integrators` | False | 定常偏差 |
| Foothold制約 | `set_stage_constraint` | `use_foothold_constraints=False` | 足領域 |
| Static/ZMP安定 | 同上 | 両方False | 支持多角形 |
| 手動warm start | `set_warm_start` | `use_warm_start=False` | 足位置初期化 |
| Reflex | `EarlyStanceDetector` は呼ばれるが trigger は False | `reflex_trigger_mode=False` | 早期接地 |
| 関節PD加算 | wrapper内コメントアウト | 常時無効 | インピーダンス |
| sampling / lyapunov / kinodynamic / input_rates | factory | `type!='nominal'` | 別MPC |
| キー入力 | `_key_callback` | `render=False` なら未登録 | 手動指令 |

## D. 動的生成・Factory

`SRBDControllerInterface.__init__` が `mpc_params['type']` で切替:

- `nominal` → `Acados_NMPC_Nominal`
- `input_rates` → `Acados_NMPC_InputRates`
- `lyapunov` → `Acados_NMPC_Lyapunov`
- `kinodynamic` → `Acados_NMPC_KinoDynamic`
- `sampling` → `Sampling_MPC`

`WBInterface.__init__` は `simulation_params['gait']` で `gait_params` を選び `PeriodicGaitGenerator` を生成する。
