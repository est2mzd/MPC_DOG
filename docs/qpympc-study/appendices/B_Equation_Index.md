# Equation Index

| 数式 | 正本章 | 対応コード |
|---|---|---|
| MuJoCo全身運動 | [01](../01_MuJoCo_Go2_Plant_Model.md) | MuJoCo engine |
| SRBD並進 | [06](../06_Centroidal_SRBD_Model.md) | `centroidal_model_nominal.forward_dynamics` |
| SRBD回転 | [06](../06_Centroidal_SRBD_Model.md) | 同上 |
| 足位置Gate | [06](../06_Centroidal_SRBD_Model.md) | 同上。結合の意味は[08](../08_Gait_MPC_Coupling.md) |
| 遊脚GRF 3段 | [09](../09_MPC_Output_and_Receding_Horizon.md) | Mask + 力学 + 摩擦 |
| MPC cost | [07](../07_MPC_Formulation.md) | `create_ocp_solver_description` |
| 基準GRF | [07](../07_MPC_Formulation.md) | `compute_control` |
| 摩擦錐 | [07](../07_MPC_Formulation.md) | `create_friction_cone_constraints` |
| Heading→World速度 | [03](../03_User_Command_and_Reference_Generation.md) | `target_base_vel` |
| Trot接地列 | [04](../04_Gait_Generator_and_Contact_Schedule.md) | `periodic_gait_generator` |
| Foothold heuristic | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) | `compute_footholds_reference` |
| Receding先頭入力 | [09](../09_MPC_Output_and_Receding_Horizon.md) | `get(0,"u")` |
| GRF出力Mask | [09](../09_MPC_Output_and_Receding_Horizon.md) | `SRBDControllerInterface.compute_control` |
| Jacobian転置 | [10](../10_Stance_and_Swing_Control.md) | `compute_stance_and_swing_torque` |
| Swing Cartesian PD | [10](../10_Stance_and_Swing_Control.md) | `compute_swing_control_cartesian_space` |
| Speed/frequency/stride | [12](../12_Speed_Frequency_Duty_and_Stride.md) | Gait/Foothold複数ファイル |
| \(L_{footprint}=v/f\) | [12](../12_Speed_Frequency_Duty_and_Stride.md) | 理論。FRGは \(T_{st}v/2\) |
| Rough-terrain feasibility | [13](../13_Feasibility_on_Rough_Terrain.md) | 推奨統合条件。交差関数は未実装 |
| Torque clip | [11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md) | `0.9 * actuator_ctrlrange` |
| \(s_i\equiv0\) | [06](../06_Centroidal_SRBD_Model.md) | `1*0` |