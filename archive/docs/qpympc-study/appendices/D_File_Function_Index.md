# File and Function Index

1行索引である。詳細は正本章へリンクする。

| ファイル | クラス/関数 | 入力 | 出力・役割 | 正本 |
|---|---|---|---|---|
| `simulation/simulation.py` | `run_simulation` | Config, `base_vel_command_type` | Main loop | [16](../16_Code_Map_and_Call_Graph.md) |
| `quadruped_env.py` | `_sample_ref_vel` | command type, ranges | `_ref_base_lin_vel_H`, `_ref_base_ang_yaw_dot` | [03](../03_User_Command_and_Reference_Generation.md) |
| `quadruped_env.py` | `_key_callback` | Keycode | 同上を加算。viewer時のみ | [03](../03_User_Command_and_Reference_Generation.md) |
| `quadruped_env.py` | `heading_orientation_SO3` | yaw | `(3,3)` H→W | [03](../03_User_Command_and_Reference_Generation.md) |
| `quadruped_env.py` | `target_base_vel` | Heading速度・Yaw | World並進、`[0,0,ψ̇]` | [03](../03_User_Command_and_Reference_Generation.md) |
| `quadruped_env.py` | `step` | `action (12,)` N·m | `mj_step`、次状態 | [11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md) |
| `quadruped_env.py` | `reset` | `random`, 任意`qpos` | keyframe/`mj_step`、摩擦上書き | [01](../01_MuJoCo_Go2_Plant_Model.md) |
| `quadruped_env.py` | `_set_ground_friction` | 接線μ | 床と足`geom_friction` | [01](../01_MuJoCo_Go2_Plant_Model.md) |
| `quadruped_env.py` | `com` | `mjModel`/`mjData` | `(3,)` W。公式CoMは[F](F_Open_Questions.md) | [01](../01_MuJoCo_Go2_Plant_Model.md) |
| `quadruped_env.py` | `get_base_inertia` | `mj_fullM` | `(3,3)`。frameは[F](F_Open_Questions.md) | [01](../01_MuJoCo_Go2_Plant_Model.md) |
| `quadruped_pympc_wrapper.py` | `compute_actions` | 状態・参照 | `tau`。MPCは100 Hz | [02](../02_System_Architecture_and_Dataflow.md) |
| `quadruped_pympc_wrapper.py` | `get_obs` 分岐 | 観測名 | `ref_feet_constraints` は typo キー。制御経路は使わない | [A](A_Variable_Dictionary.md) |
| `wb_interface.py` | `update_state_and_reference` | 状態・速度指令 | `state_current`, `ref_state`, `contact_sequence` | [03](../03_User_Command_and_Reference_Generation.md) |
| `terrain_estimator.py` | `compute_terrain_estimation` | base, yaw, lift-off | roll/pitch/height | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| `velocity_modulator.py` | `modulate_velocities` | 指令、feet、hip | 補正後指令 | [03](../03_User_Command_and_Reference_Generation.md) |
| `periodic_gait_generator.py` | `run` | `dt`, `step_freq` | 位相更新、`contact (4,)` | [04](../04_Gait_Generator_and_Contact_Schedule.md) |
| `periodic_gait_generator.py` | `compute_contact_sequence` | dts, lengths | `(4,N)`接地列。位相は復元 | [04](../04_Gait_Generator_and_Contact_Schedule.md) |
| `foothold_reference_generator.py` | `update_lift_off_positions` | 接触エッジ、feet | `lift_off_positions` | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| `foothold_reference_generator.py` | `update_touch_down_positions` | 接触エッジ、feet | `touch_down_positions` | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| `foothold_reference_generator.py` | `compute_footholds_reference` | 状態・回転前xy速度 | Nominal footholds | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| `visual_foothold_adaptation.py` | `compute_adaptation` | Heightmaps | Adapted foothold。非blind | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| `srbd_controller_interface.py` | `compute_control` | State/ref/contact | Mask後GRF、Foothold | [09](../09_MPC_Output_and_Receding_Horizon.md) |
| `centroidal_model_nominal.py` | モデル定義 | x/u/p | xdot | [06](../06_Centroidal_SRBD_Model.md) |
| `centroidal_nmpc_nominal.py` | `set_weight` | — | `Q`,`R`対角。`config.py`に無い | [07](../07_MPC_Formulation.md) |
| `centroidal_nmpc_nominal.py` | `create_friction_cone_constraints` | \(\mu\), GRF | Focchi 20式。接触で切らない | [07](../07_MPC_Formulation.md) |
| `centroidal_nmpc_nominal.py` | `perform_scaling` | state, reference | 原点ずらし | [09](../09_MPC_Output_and_Receding_Horizon.md) |
| `centroidal_nmpc_nominal.py` | `compute_control` | State/ref/contact | `u[0]`, foothold, `x[0:24]` | [09](../09_MPC_Output_and_Receding_Horizon.md) |
| `wb_interface.py` | `compute_stance_and_swing_torque` | GRF/Foothold/全身量 | `tau`, IK目標 | [10](../10_Stance_and_Swing_Control.md) |
| `swing_trajectory_controller.py` | `update_swing_time` | `current_contact`, `dt` | 遊脚時計。立脚で0 | [10](../10_Stance_and_Swing_Control.md) |
| `swing_trajectory_controller.py` | `compute_swing_control_cartesian_space` | 軌道・状態 | Swing torque（PD二重） | [10](../10_Stance_and_Swing_Control.md) |
| `swing_trajectory_controller.py` | `compute_swing_control_joint_space` | 関節目標 | 未呼出 | [16](../16_Code_Map_and_Call_Graph.md) |
| `early_stance_detector.py` | `update_detection` | 足状態・Contact | 標準では無効 | [16](../16_Code_Map_and_Call_Graph.md) |
| `inverse_kinematics_numeric_mujoco.py` | `compute_solution` | `qpos`, 足目標 | 関節目標。tau未使用 | [10](../10_Stance_and_Swing_Control.md) |
