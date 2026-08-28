# Variable Dictionary

変数のshape・単位・frame・生成元の正本である。物語的説明は各章へリンクする。未確定のframeは「コード上の解釈」と書き、確定は[F](F_Open_Questions.md)へ置いた。

数式記号との対応も本章にだけ横断掲載する。

| 数式 | コード変数 | 正本章 |
|---|---|---|
| \(v_H^{ref}\) | `_ref_base_lin_vel_H` | [03](../03_User_Command_and_Reference_Generation.md) |
| \(R_H^W\) | `heading_orientation_SO3` | [03](../03_User_Command_and_Reference_Generation.md) |
| \(\dot\psi^{ref}\) | `_ref_base_ang_yaw_dot` | [03](../03_User_Command_and_Reference_Generation.md) |
| \(v_W^{ref}\) | `ref_base_lin_vel`（`target_base_vel`直後） | [03](../03_User_Command_and_Reference_Generation.md) |
| \(\phi_i,f,d\) | `_phase_signal`, `step_freq`, `duty_factor` | [04](../04_Gait_Generator_and_Contact_Schedule.md) |
| \(c_{i,k}\) | `contact_sequence[i,k]` | [04](../04_Gait_Generator_and_Contact_Schedule.md) |
| \(\Delta p_{ref}\) | `vel_offset` | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| \(\Delta p_{err}\) | `error_compensation` | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| \(T_{stance}\) | `frg.stance_time` | [05](../05_Foothold_Reference_and_Terrain_Adaptation.md) |
| \(u_0^*\) | `acados_ocp_solver.get(0,"u")` | [09](../09_MPC_Output_and_Receding_Horizon.md) |
| \(F_i^{MPC}\) | `control[12:]` の脚ブロック | [09](../09_MPC_Output_and_Receding_Horizon.md) |
| \(F_i^{cmd}\) | `nmpc_GRFs.*` | [09](../09_MPC_Output_and_Receding_Horizon.md) |
| \(F_i^{act}\),\(\lambda\) | `feet_contact_state(..., True)` / `mjData.contact` | [11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md) |
| \(\tau_i^{stance}\) | `tau.*`（立脚） | [10](../10_Stance_and_Swing_Control.md) |
| \(J_i\) | `feet_jac.*[:, legs_qvel_idx.*]` | [10](../10_Stance_and_Swing_Control.md) |
| \(R_{BW}\) | `b_R_w` | [06](../06_Centroidal_SRBD_Model.md) |
| \(s_i\) | `stance_proximity_*`（標準 `1*0` → 0） | [06](../06_Centroidal_SRBD_Model.md) |
| \(F_{ext}\) | `p[13:16]`（標準0） | [06](../06_Centroidal_SRBD_Model.md) |
| \(Q,R\) | `set_weight()` の対角 | [07](../07_MPC_Formulation.md) |
| \(L_{footprint}\) | \(v/f\)（コード変数なし） | [12](../12_Speed_Frequency_Duty_and_Stride.md) |
| \(\tau^{limited}\) | `np.clip(..., 0.9*ctrlrange)` | [11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md) |

| 変数 | 意味 | Shape | 単位 | Frame | 生成元 | 主使用先 | 更新周期 |
|---|---|---|---|---|---|---|---|
| `_ref_base_lin_vel_H` | 目標並進速度 | `(3,)` | m/s | H | `_sample_ref_vel` / `_key_callback` | `target_base_vel()` | resetまたはキー |
| `_ref_base_ang_yaw_dot` | 目標Yaw速度 | scalar | rad/s | z | 同上 | `target_base_vel()` | 同上 |
| `ref_base_lin_vel` | 目標並進（変換直後） | `(3,)` | m/s | W | `target_base_vel()` | VM、Foothold（回転前xy） | 500 Hzで読む |
| `ref_base_lin_vel`（上書き後） | 地形回転後の並進 | `(3,)` | m/s | 地形付き | `update_state_and_reference` | `ref_state['ref_linear_velocity']` | 500 Hz |
| `ref_base_ang_vel` | 目標角速度 | `(3,)` | rad/s | `[0,0,ψ̇]` | `target_base_vel()` | VM、`ref_state` | 500 Hzで読む |
| `com_pos` | `env.com`の戻り | `(3,)` | m | W | `QuadrupedEnv.com`。公式の物理CoM一致は[F](F_Open_Questions.md) | `state_current['position']` | 500 Hz |
| `base_pos` | Base原点 | `(3,)` | m | W | `mjData.qpos[0:3]` | Foothold、地形、`ref_pos` | 500 Hz |
| `base_lin_vel` | Base並進速度 | `(3,)` | m/s | W | `qvel[0:3]` | `state_current`、Foothold | 500 Hz |
| `base_ang_vel` | Base角速度 | `(3,)` | rad/s | B（コード上の解釈） | `base_ang_vel(frame='base')` → `qvel[3:6]` | `state_current` | 500 Hz |
| `base_ori_euler_xyz` | Euler姿勢 | `(3,)` | rad | SciPy `xyz` | `qpos[3:7]` | 地形、Foothold、`state_current` | 500 Hz |
| `feet_pos.*` | 足位置 | 各`(3,)` | m | W | `geom_xpos` | Gait、Foothold、MPC、Swing | 500 Hz |
| `feet_vel.*` | 足速度 | 各`(3,)` | m/s | W | `J @ qvel` | Swing | 500 Hz |
| `hip_pos.*` | Hip位置 | 各`(3,)` | m | W | `body.xpos` of `*_hip` | VM、Foothold | 500 Hz |
| `feet_jac.*` | 足並進Jacobian | 各`(3, nv)`、Go2は`(3,18)` | 混在 | W | `mj_jac` | Stance/Swing | 500 Hz |
| `feet_jac_dot.*` | Jacobian時間微分 | 各`(3,18)` | 混在 | W | `mj_jacDot` | Swing | 500 Hz |
| `qpos` | Configuration | `(nq,)`、Go2は`(19,)` | 混在 | MuJoCo | `mjData.qpos` | IK | 500 Hz |
| `qvel` | 一般化速度 | `(nv,)`、Go2は`(18,)` | 混在 | MuJoCo | `mjData.qvel` | Jacobian、Swing | 500 Hz |
| `inertia` | 慣性9要素 | `(9,)` | kg·m² | コード上は`mj_fullM[3:6,3:6]`。厳密frameは[F](F_Open_Questions.md) | `get_base_inertia().flatten()` | MPC parameter | 500 Hz計算、MPCは100 Hz |
| `joints_pos` | 名前は関節角だが現行はqvel index | 各脚 length-3 int | なし | なし | `simulation.py`: `legs_qvel_idx` | `state_current['joint_*']`。nominal未使用 | 500 Hz |
| `legs_mass_matrix.*` | 脚質量行列 | 各`(3,3)` | kg | 関節 | `mj_fullM`の脚ブロック | Swing FB線形化 | 500 Hz |
| `legs_qfrc_bias.*` | コリオリ・重力 | 各`(3,)` | N·m | 関節 | `qfrc_bias` | Swing | 500 Hz |
| `legs_qfrc_passive.*` | 受動力 | 各`(3,)` | N·m | 関節 | `qfrc_passive` | `tau -= passive` | 500 Hz |
| `contact_sequence` | 将来予定接触 | `(4, horizon)`、既定`(4,12)` | 0/1 | なし | `compute_contact_sequence` | MPC、`current_contact` | 500 Hz生成 |
| `current_contact` | 現在予定接触 | `(4,)` | 0/1 | なし | `contact_sequence[:,0]` | Mask、Stance/Swing | 500 Hz |
| `_phase_signal` | 脚位相 | `(4,)` | 無次元 0–1 | なし | `PeriodicGaitGenerator.run` | 接地判定、observable | 500 Hz |
| `state_current` | MPC現在状態 | dict | 混在 | 主にW。角速度B | `update_state_and_reference` | MPC | 500 Hz |
| `ref_state` | MPC参照 | dict | 混在 | 並進速度は地形付き | 同上 | MPC | 500 Hz |
| `ref_foot_FL` 等 | 足参照 | `(1,3)` | m | W | `ref_feet_pos.*.reshape((1,3))` | MPC yref | 500 Hz |
| `ref_foot_constraints_FL` 等 | 足制約 | blindでは`None` | — | W | VFAまたは`None` | MPC（使用は設定依存） | 500 Hz |
| `terrain_roll` / `terrain_pitch` | 推定傾斜 | scalar | rad | H差分から算出 | `TerrainEstimator` | `ref_orientation`、速度回転 | 500 Hz（内部フィルタ） |
| `terrain_height` | 推定地形高さ | scalar | m | W z | 4足zの平均をフィルタ | `ref_position[2]` | 500 Hz |
| `robot_height` | 推定ロボット高さ | scalar | m | — | 同上。`base_pos[2]`への代入はコメントアウト | 現状ほぼ未使用 | 500 Hz |
| `ref_feet_pos.*` | Nominal foothold | 各`(3,)` | m | W。zはlift-off z | `compute_footholds_reference` | `ref_state` | 500 Hz |
| `lift_off_positions.*` | 離地位置 | 各`(3,)` | m | W（遊脚中はH保持） | `update_lift_off_positions` | Swing、地形推定の足 | 接触エッジ |
| `nmpc_GRFs.*` | Mask後目標GRF | 各`(3,)` | N | W | `SRBDControllerInterface` | Stance | 100 Hz更新 |
| `nmpc_footholds.*` | Touchdown/stance位置 | 各`(3,)` | m | W | 同上 | Swing、IK | 100 Hz更新 |
| `nmpc_predicted_state` | 予測状態の先頭24 | `(24,)` | 混在 | W（decenter後） | `get(k,"x")[0:24]` | コメントアウトされたIK拡張 | 100 Hz。tau未使用 |
| `des_foot_pos.*` | 足先目標 | 各`(3,)` | m | W | Swingまたは`nmpc_footholds` | IK | 500 Hz |
| `des_joints_pos.*` | 目標関節角 | 各`(3,)` | rad | 関節 | IK | コメントアウトされたPD | 500 Hz計算、プラント未使用 |
| `tau.*` | 脚関節トルク | 各`(3,)` | N·m | 関節 | Stance/Swing | clip → `action` | 500 Hz |
| `action` | 全actuator | `(nu,)`、Go2は`(12,)` | N·m | actuator順 FL,FR,RL,RR × hip,thigh,calf | `simulation.py` | `env.step` | 500 Hz |
| `mjData.ctrl` | Plant入力 | `(12,)` | N·m | 同上 | `QuadrupedEnv.step` | MuJoCo | 500 Hz |
| `mjData.sensordata` | XMLセンサ連結 | `(25,)` | 混在 | sensor | MuJoCo。標準未読 | なし | — |
| `geom_friction`（実行時） | 床と足 | 各`(3,)` 接線/ねじり/転がり | — | geom | `reset`→`_set_ground_friction`。XML足`0.8 0.02 0.01`ではない | 接触ソルバー | episode |
| \(F^{act}\) / 実GRF | MuJoCo接触力 | 各脚`(3,)` | N | 接触→W | `feet_contact_state(ground_reaction_forces=True)` | **viewer専用**。トルク・MPCへ戻さない | render時 |
| `mjData.contact` | 接触列 | 可変 | 混在 | 接触 | `mj_step` | ログ用。制御切替は使わない | 500 Hz |
| 実接地フラグ | 足geom接触 | `(4,)` | 0/1 | なし | `feet_contact_state()` | 表示。`current_contact`（予定）とは別 | 500 Hz |

観測分岐だけ typo がある: wrapper `get_obs` が `ref_foot_FL_constraints` を読む。実キーは `ref_foot_constraints_FL`。制御経路は正しい。[E](E_Corrections_and_Clarifications.md) §12。

## 注意

- `base_ang_vel`のFrameと`com`の公式は、コード更新時に再確認する。Docstringだけで判断しない。
- 旧説明「`_ref_base_lin_vel_H`の生成元はKey callbackだけ」は[E](E_Corrections_and_Clarifications.md) §11。
