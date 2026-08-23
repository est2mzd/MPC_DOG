# Stance and Swing Control

## 1. 結論

立脚脚はMPC GRFをJacobian転置で関節トルクへ変換し、遊脚脚はMPC Footholdを終点とするCartesian軌道追従トルクを使う。同じ`current_contact`が両者を切り替える。

本章がStance/Swingトルク、摩擦補償、標準設定で無効なESD/関節空間Swing/関節PDの正本である。`action`組立と`mj_step`は[11](11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。無効経路一覧は[16](16_Code_Map_and_Call_Graph.md)。

`WBInterface`は一般的な全身QP-WBCと同一ではない。

## 2. 境界入出力

| 入力 | shape | 単位 | frame | 周期 |
|---|---|---|---|---|
| `nmpc_GRFs.*` | 各`(3,)` | N | W | 100 Hz更新、500 Hzで使用 |
| `nmpc_footholds.*` | 各`(3,)` | m | W | 同上 |
| `feet_jac.*[:, qvel_idx]` | 各`(3,3)` | 混在 | W | 500 Hz |
| `feet_pos` / `feet_vel` | 各`(3,)` | m, m/s | W | 500 Hz |
| `current_contact` | `(4,)` | 0/1 | なし | 500 Hz |
| `legs_qfrc_bias` / `legs_qfrc_passive` / `mass_matrix` | `(3,)` / `(3,)` / `(3,3)` | N·m / N·m / kg | 関節 | 500 Hz |

| 出力 | shape | 単位 | frame | 次の使用先 |
|---|---|---|---|---|
| `tau.*` | 各`(3,)` | N·m | 関節 | clip → `action` |
| `des_joints_pos/vel` | 各`(3,)` | rad, rad/s | 関節 | 標準ではプラント未使用 |

対応コード: `WBInterface.compute_stance_and_swing_torque()`。

## 3. 立脚制御

足先速度は、

\[
\dot x_i=J_i(q)\dot q_i
\]

仮想仕事より、

\[
\tau_i=J_i^\mathsf T F_i
\]

実装の力符号では、Mask後の指令を使う。

\[
\tau_i^{stance}=-J_i^\mathsf T F_i^{\mathrm{command}}
\]

立脚では \(c_{i,0}=1\) のため \(F_i^{\mathrm{command}}=F_i^{MPC}\)。記号の正本は[09](09_MPC_Output_and_Receding_Horizon.md) §6。

| 数式 | コード変数 |
|---|---|
| \(J_i\) | `feet_jac.FL[:, legs_qvel_idx.FL]` 等 |
| \(F_i^{\mathrm{command}}\) | `nmpc_GRFs.FL` 等（Mask後） |
| \(\tau_i^{stance}\) | `tau.FL` 等 |

これは一般的な全身QP-WBCではない。接触力再配分、全身加速度、関節制約を別QPで再最適化せず、MPC GRFを脚ごとに写像する簡潔なWBC相当処理である。

対応コード: `wb_interface.py` の `tau.FL = -np.matmul(...)` ブロック。

## 4. Swing軌道

入力：

- `frg.lift_off_positions`
- `nmpc_footholds`のTouchdown位置
- `stc.swing_period`（`(1-duty_factor)/step_freq`）
- `step_height`（既定`0.2 * hip_height`）

出力：

\[
p_d(t),\quad\dot p_d(t),\quad\ddot p_d(t)
\]

| 数式 | コード変数 |
|---|---|
| \(p_d,\dot p_d,\ddot p_d\) | `swing_generator.compute_trajectory_references` の戻り |

対応コード: `helpers/swing_generators/` と `SwingTrajectoryController.compute_swing_control_cartesian_space()`。

`update_swing_time()`は遊脚中に`swing_time += simulation_dt`、立脚で0に戻す。周期500 Hz。

## 5. Cartesian tracking

\[
\ddot p_{cmd}
=
\ddot p_d
+K_p(p_d-p)
+K_d(\dot p_d-\dot p)
\]

既定値は`swing_position_gain_fb=500`、`swing_velocity_gain_fb=10`である。

実装は次の2段であり、**同じPDが二重に入る**。

1. \(\tau \leftarrow J^\top(K_p e_p + K_d e_v)\)
2. `use_feedback_linearization=True`（標準ON）なら \(\ddot p_{cmd}=\ddot p_d+K_p e_p+K_d e_v\) を使い \(\tau \mathrel{+} M J^{+}(\ddot p_{cmd}-\dot J\dot q)+h\)

処理順: 全脚に先に \(-J^\top F\)（遊脚はMask後 \(F=0\)）→ 遊脚が `tau` を上書き → 全脚 `tau -= qfrc_passive`。

`passive_force`は引数にあるが関数本体では使わない。摩擦補償は次節。

| 数式 | コード変数 |
|---|---|
| \(K_p\) | `position_gain_fb` |
| \(K_d\) | `velocity_gain_fb` |
| \(M,h\) | `mass_matrix`, `h`（`legs_qfrc_bias`） |
| \(\tau^{swing}\) | 遊脚の`tau[leg_name]` |

対応コード: `swing_trajectory_controller.py` の `compute_swing_control_cartesian_space()`。

`compute_swing_control_joint_space()`は定義のみで、`wb_interface`から呼ばれない。

## 6. 摩擦補償

`stc.use_friction_compensation=True`（既定）のとき、立脚・遊脚のあと

```text
tau.* -= legs_qfrc_passive.*
```

を全脚に適用する。フラグ名はswingだが、実装は全脚である。

対応コード: `WBInterface.compute_stance_and_swing_torque()`。

## 7. Early stance / Reflex

`simulation_params['reflex_trigger_mode']=False`のとき`EarlyStanceDetector.activated=False`である。`update_detection()`はhitpointsを毎周期`None`に戻す。標準経路ではReflexは動かない。

有効時の説明をここに詳述しない。到達条件は[16](16_Code_Map_and_Call_Graph.md)。これは固定Gait scheduleを完全再計画する機能ではなく、局所的なReflexである。

## 8. IKと関節目標

足先目標から`InverseKinematicsNumeric.compute_solution()`で`des_joints_pos`を計算し、Jacobian擬似逆で`des_joints_vel`を作る。500 Hzで計算する。

標準Wrapperでは関節PD加算がコメントアウトされているため、`des_joints_*`はプラント入力にならない。`nmpc_predicted_state`を`qpos`へ入れる行もコメントアウトである。

関節PDを有効化するのは **推奨改善** であり、現行標準経路には無い。

対応コード: `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py`、`quadruped_pympc_wrapper.py` のコメントアウトブロック。

## 9. 対応コード

- `interfaces/wb_interface.py`: `compute_stance_and_swing_torque()`
- `helpers/swing_trajectory_controller.py`
- `helpers/swing_generators/`
- `helpers/early_stance_detector.py`（標準では無効）
- `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py`

## 10. Cursor確認課題

Swing torque式をコードから完全に再構成し、Feedforward、PD、Passive force、Bias forceの符号を単体テストで確認する。
