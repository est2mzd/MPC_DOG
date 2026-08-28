# Log 13: Stance / Swing torque End-to-End

対応プロンプト: `nmpc_GRFs` / `nmpc_footholds` / `current_contact` から各脚 `tau` と関節目標まで。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

標準: `type='nominal'`, `swing_generator='scipy'`, `reflex_trigger_mode=False`, `use_feedback_linearization=True`, `use_friction_compensation=True`。周期 500 Hz。MPC解は 100 Hz hold。

## 処理境界

| 順序 | 入力 | 処理内容 | 関数 | 出力 | Shape | 単位 | Frame | 次の使用先 |
|---|---|---|---|---|---|---|---|---|
| 1 | `nmpc_GRFs`, `nmpc_footholds` | wrapperが保持したMPC出力 | `QuadrupedPyMPC_Wrapper.compute_actions` | 同左 | 脚ごと(3,) | N / m | W | WBC |
| 2 | `current_contact` | PGG先頭列 | `WBInterface.update_state_and_reference` | `(4,)` | (4,) | 0/1 | なし | 切替 |
| 3 | `qpos`,`qvel` | Plant状態 | `env.mjData` | 全身 | (19,),(18,) | rad, rad/s | 混在 | J, IK |
| 4 | 足geom | `mj_jac` | `QuadrupedEnv.feet_jacobians(frame='world')` | `feet_jac.*` | (3,18) | — | W並進 | 列抽出 |
| 5 | 同上 | `mj_jacDot` | `feet_jacobians_dot(frame='world')` | `feet_jac_dot.*` | (3,18) | — | W | 遊脚FF |
| 6 | `feet_jac`, `qvel` | 足速度 | `env.feet_vel(frame='world')` | `feet_vel.*` | (3,) | m/s | W | 遊脚PD |
| 7 | `mj_fullM` の脚ブロック | 脚慣性 | `env.legs_mass_matrix` | (3,3)×4 | (3,3) | kg·m²相当 | 関節 | 遊脚FF |
| 8 | `qfrc_bias[qvel_idx]` | 重力+Coriolis+遠心 | `env.legs_qfrc_bias` | (3,)×4 | (3,) | N·m | 関節 | `+h` |
| 9 | `qfrc_passive[qvel_idx]` | 減衰・ばね等 | `env.legs_qfrc_passive` | (3,)×4 | (3,) | N·m | 関節 | 全脚補償 |
| 10 | `best_sample_freq` | 標準では不実行 | `if optimize_swing==1` | `step_freq`, `swing_period` | scalar | Hz, s | — | 軌道再生成 |
| 11 | 足状態、接触 | Reflex更新 | `EarlyStanceDetector.update_detection` | 標準は全None | — | — | — | 軌道（無効） |
| 12 | `J_i`, `F_i` | 全脚に立脚写像 | `tau.LEG = -J[:,idx].T @ F` | `tau.*` | (3,) | N·m | 関節 | 遊脚が上書き |
| 13 | `current_contact`, `dt` | 遊脚時計 | `STC.update_swing_time` | `swing_time[4]` | list | s | — | 軌道 |
| 14 | lift-off, TD, `swing_time` | 3次スプライン | `SwingTrajectoryGenerator.compute_trajectory_references` | `p_d,v_d,a_d` | (3,)×3 | m, m/s, m/s² | W | Cartesian |
| 15 | 軌道と現在足 | 遊脚トルク | `compute_swing_control_cartesian_space` | `tau`, `des_foot_pos/vel` | (3,) | N·m / m / m/s | 関節 / W | 摩擦補償 |
| 16 | `nmpc_footholds` | 立脚の足目標 | `else` 分岐 | `des_foot_pos=TD`, `des_foot_vel=0` | (3,) | m, m/s | W | IK |
| 17 | `tau`, `qfrc_passive` | 全脚 `tau -= passive` | WBC 414–418 | `tau` | (3,) | N·m | 関節 | clip |
| 18 | `des_foot_pos` 4脚 | 数値IK 5回 | `InverseKinematicsNumeric.compute_solution` | 12関節角 | (12,) | rad | 関節 | 分割 |
| 19 | `J_i^+`, `des_foot_vel` | 関節速度 | `pinv(J) @ v_d` | `des_joints_vel` | (3,) | rad/s | 関節 | 飽和 |
| 20 | 現在関節との差 | ±3 rad / ±10 rad/s | `np.clip` | 飽和後目標 | (3,) | rad, rad/s | 関節 | 標準sim未使用 |
| 21 | `tau` | `0.9*ctrlrange` | `simulation.py` | `action` | (12,) | N·m | アクチュエータ | `env.step` |

---

## 1. 立脚制御

| # | 確認項目 | 結果 |
|---|---|---|
| 1 | 立脚判定 | `current_contact[leg_id]==1`。ただし `-J.T@F` は**全脚に先に適用**。遊脚は `F` が既に0（上流mask）で、その後swingが `tau` を上書き |
| 2 | MPC GRF名 | `nmpc_GRFs.FL/FR/RL/RR` |
| 3 | 単位とFrame | N、World。docstringどおり |
| 4 | Jacobian取得元 | `env.feet_jacobians(frame='world', return_rot_jac=False)` → `mujoco.mj_jac` |
| 5 | Jacobian shape | 生: `(3, nv=18)`。使用: `(3,3)` |
| 6 | 脚3関節Index | `legs_qvel_idx.LEG`。Go2は hip, thigh, calf。qvel は FL 6–8, FR 9–11, RL 12–14, RR 15–16–17 |
| 7 | Jacobian Frame | World並進。`return_rot_jac=False` なので回転行なし |
| 8 | 実際のコード | `tau.FL = -np.matmul(feet_jac.FL[:, legs_qvel_idx.FL].T, nmpc_GRFs.FL)`（他脚同様） |
| 9 | 負号 | `F_MPC` はロボットが受けるGRF。足が地面へ出す力は `-F`。仮想仕事 \(\tau=J^\top F_{\mathrm{ee}}\) の \(F_{\mathrm{ee}}=-F^{MPC}\) |
| 10 | Gravity補償 | 立脚写像には**無い**。重力支持はMPC GRF側 |
| 11 | Coriolis/Bias | 立脚写像には**無い** |
| 12 | Joint damping補償 | 立脚写像には無い。後段で全脚 `tau -= qfrc_passive`（減衰・ばねの打消し） |
| 13 | saturation前 | 上の `tau` + 遊脚上書き + passive補償。clipは `simulation.py` |
| 14 | Contact切替 | ブレンドなし。`c=0` になった瞬間にswingが上書き。`c=1` なら `-J.T@F` が残る |

\[
\tau_i^{stance}=-J_i(q)^\mathsf T F_i^{MPC}
\]

| 数式項 | コード変数 | Shape | 単位 | Frame | 生成元 |
|---|---|---|---|---|---|
| \(J_i\) | `feet_jac.LEG[:, legs_qvel_idx.LEG]` | (3,3) | — | W→関節 | `mj_jac` |
| \(F_i^{MPC}\) | `nmpc_GRFs.LEG` | (3,) | N | W | MPC + `*c_0` |
| \(\tau_i^{stance}\) | `tau.LEG`（上書き前） | (3,) | N·m | 関節 | 上式 |
| \(q\) の脚部分 | `qpos[legs_qpos_idx]` | (3,) | rad | 関節 | Plant。Jの中に入る |

使用する \(J\) は浮遊ベース列を捨てた3関節分だけである。全身 `(3,18)` ではない。

| 項目 | 現行WBInterface | 一般的なQP-WBC |
|---|---|---|
| 接触力 | MPC GRFを脚ごとに写像 | QPで再配分 |
| 動力学 | 立脚は写像のみ。遊脚は脚3×3近似 | 全身 \(M\ddot q+h=\tau+J^\top F\) |
| 拘束 | なし | 摩擦、トルク、接触をQP制約 |
| 最適化 | なし | 加速度・力を最小二乗/QP |
| 切替 | `current_contact` で脚ごと上書き | 同じ接触集合をQPへ |
| ベースDoF | Jから除外 | 浮遊ベースを含む |

---

## 2. 遊脚軌道

標準Generatorは `scipy`（`CubicSpline`, 端点 clamped = 端点速度0）。

| # | 項目 | 結果 |
|---|---|---|
| 1 | Lift-off | `frg.lift_off_positions[leg]`。立脚→遊脚エッジで現在`feet_pos`を記録 |
| 2 | Touchdown | `nmpc_footholds[leg]`。MPC抽出の次TD。`frg.touch_down_positions` はコメントアウト |
| 3 | Swing開始位相 | `current_contact` が1→0。`swing_time` は立脚中0、遊脚で `+=dt` |
| 4 | Swing終了位相 | `current_contact` が0→1で `swing_time=0`。Gaitの \(\phi=d\) が予定終了。実接触ではない |
| 5 | Swing period | `(1-duty_factor)/step_freq` = `(1-0.74)/1.35 ≈ 0.1926` s |
| 6 | 残り時間 | 変数なし。実効は `swing_period - swing_time[leg]`。`>=period` なら時計停止 |
| 7 | Step height | `0.2*hip_height=0.056` m。scipyでは中間点の相対高さ |
| 8 | Generator | 標準 `scipy`。`explicit` は3次Bezier2本。ESD引数はscipyだけ使う |
| 9 | 目標足位置 | `des_foot_pos` = スプライン(t) |
| 10 | 目標足速度 | `des_foot_vel` = 1階微分 |
| 11 | 目標足加速度 | `des_foot_acc`。関数戻りには出さず、内部で `accelleration` に使う |
| 12 | 連続性 | scipyは5点C2スプライン、端点速度0。毎周期 `createCurve` を呼び直す |
| 13 | 水平 | x,y を lift-off→中間→TD で補間 |
| 14 | 鉛直 | 中間で `stepHeight` を足す。Reflex時は別制御点 |

`explicit` の前半制御点 `z=self.step_height` は **絶対高さ** であり、`lift_off[2]+h` ではない。標準経路では使わない。

| 出力 | コード変数 | Shape | 単位 | Frame | 次の使用先 |
|---|---|---|---|---|---|
| 目標位置 | `des_foot_pos` / 戻り1 | (3,) | m | W | Cartesian, IK |
| 目標速度 | `des_foot_vel` / 戻り2 | (3,) | m/s | W | Cartesian, `pinv(J)` |
| 目標加速度 | `des_foot_acc` | (3,) | m/s² | W | `accelleration` のみ |
| 立脚時位置 | `nmpc_footholds` | (3,) | m | W | IK（軌跡ではない） |
| 立脚時速度 | 0 | (3,) | m/s | W | IK速度 |

---

## 3. 遊脚Cartesian制御

コード（`compute_swing_control_cartesian_space`）:

\[
e_p=p_d-p,\quad e_v=\dot p_d-\dot p
\]

\[
\ddot p_{\mathrm{cmd}}
=
\ddot p_d+K_p e_p+K_d e_v
\]

\[
\tau^{PD}=J^\top(K_p e_p+K_d e_v)
\]

\[
\tau^{ID}=M J^{+}(\ddot p_{\mathrm{cmd}}-\dot J\dot q)+h
\]

\[
\tau^{swing}=\tau^{PD}+\tau^{ID}
\quad(use\_feedback\_linearization=True)
\]

| 数式項 | コード変数 | Shape | 単位 | Frame | 生成元 |
|---|---|---|---|---|---|
| \(p_d,\dot p_d,\ddot p_d\) | `des_foot_pos/vel/acc` | (3,) | m, m/s, m/s² | W | scipyスプライン |
| \(p,\dot p\) | `foot_pos`,`foot_vel` | (3,) | m, m/s | W | env |
| \(K_p\) | `position_gain_fb` | scalar | 1/s² | — | 500 |
| \(K_d\) | `velocity_gain_fb` | scalar | 1/s | — | 10 |
| \(\ddot p_{cmd}\) | `accelleration` | (3,) | m/s² | W | 上式 |
| \(J\) | `feet_jac[:, qvel_idx]` | (3,3) | — | W | `mj_jac` |
| \(J^{+}\) | `np.linalg.pinv(J)` | (3,3) | — | — | numpy |
| \(\dot J\) | `J_dot` 同抽出 | (3,3) | — | W | `mj_jacDot` |
| \(\dot q\) | `q_dot=qvel[qvel_idx]` | (3,) | rad/s | 関節 | Plant |
| \(M\) | `mass_matrix` | (3,3) | — | 関節 | `mj_fullM` の脚ブロック |
| \(h\) | `h` (`legs_qfrc_bias`) | (3,) | N·m | 関節 | `qfrc_bias`（重力+Coriolis+遠心） |
| passive | 引数`passive_force` | (3,) | N·m | 関節 | **関数内未使用** |
| Feedforward | `M J+(a_d - Jdot qdot) + h` 相当 | (3,) | N·m | 関節 | ID項。ただし \(a_{cmd}\) にPDを含む |
| Feedback | `J.T@(Kp e + Kd ev)` と \(a_{cmd}\) 内のPD | (3,) | N·m | 関節 | **PDが二重** |
| 符号 | `+h`, `+` ID, `J.T` は正 | — | — | — | 立脚の負号とは別 |
| Clip | 関数内なし | — | — | — | 外側で `tau` と関節目標 |

`10` の2段説明はコードと一致する。完全な単一式は \(\tau=J^\top K e + M J^{+}(\ddot p_d+Ke-\dot J\dot q)+h\) であり、PDが二回入る。

`M` は浮遊ベース連成を捨てた3×3近似である。

---

## 4. IKと関節目標

| # | 項目 | 結果 |
|---|---|---|
| 1 | IK入力 | 現在`qpos`全文（予測base代入はコメントアウト）と4脚`des_foot_pos` |
| 2 | IK出力 | 12関節角 `q_joint`。失敗判定なし。常に5回反復後の値 |
| 3 | Joint order | 出力 `[FL hip,thigh,calf, FR..., RL..., RR...]`。分割は `temp[0:3]`… |
| 4 | 位置目標 | `des_joints_pos`。現在角との差を ±3 rad で飽和 |
| 5 | 速度目標 | `pinv(J_i) @ des_foot_vel`。差を ±10 rad/s で飽和。IKのJacobianではない |
| 6 | Pseudo-inverse | 速度は脚ごと `pinv(3×3)`。IK内部は減衰 `inv(J^T J + 1e-3 I) J^T` と `total_err*100` |
| 7 | IK failure | なし。残差チェックなし |
| 8 | 標準MuJoCoの使用先 | **プラントに入れない**。計算して捨てる |
| 9 | Joint PDへ | wrapper 201–203行がコメントアウト。標準では使わない |
| 10 | 実機Interface | `ros2/run_controller.py` が `pd_target_joints_pos/vel` と `kp/kd` を `TrajectoryGenerator` としてpublish。トルクも別publish |

`compute_swing_control_joint_space` は定義のみ。`wb_interface` から呼ばれない。

kinodynamic分岐は `des_joints_pos = nmpc_joints_vel` と上書きしており、標準では到達しない。

---

## 5. Early stance / Reflex

標準 `reflex_trigger_mode=False` → `activated=False`。毎周期 hitpoints=None, hitmoments=-1。軌道は通常スプライン。

有効時（標準外）:

| 項目 | 結果 |
|---|---|
| 検出状態 | `feet_pos`, `last_des_foot_pos`, lift-off, TD, `swing_time`, `swing_period` |
| 予定Contact | `current_contact==1` ならearly stanceをリセット |
| 実Contact | `geom_contact` モードのみ `mujoco_contact` |
| Tracking error | `tracking` モード: `\|p_d-p\|/\|p_{td}-p_{lo}\|` と絶対距離 |
| 閾値 | 相対0.3、絶対0.1 m、終端0.07 sは無視 |
| 遷移 | False→Trueで hitpoint/hitmoment 記録。Gait列は変えない |
| Swing torque | scipyが hitpoint からTDへ曲線を作り直す |
| Stance切替 | `current_contact` が1になれば通常立脚。ESDは接触列を書き換えない |
| Gait schedule | **変更しない** |
| 局所処理か | はい。次周期の歩高さブースト（別フラグ）も局所 |

---

## 資料照合

### `10_Stance_and_Swing_Control.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| 立脚 `-J.T F`、遊脚Cartesian | 正しい | なし |
| 同じ`current_contact`で切替 | 正しい | 全脚に先に立脚写像する順序は未記載 |
| Jは`[:, qvel_idx]` | 正しい | (3,18)→(3,3)の明示が弱い |
| QP-WBCではない | 正しい | なし |
| 軌道入出力 | 正しい | scipy 5点・clampedは未記載 |
| \(\ddot p_{cmd}\) と2段実装 | 正しい | PD二重は本文に無い |
| `passive`未使用、全脚摩擦補償 | 正しい | なし |
| ESD標準無効 | 正しい | なし |
| IKは計算するがPD無効 | 正しい | ROS2ではpublishする事実が無い |
| 関節空間swing未呼出 | 正しい | なし |

### `appendices/B_Equation_Index.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| Jacobian転置 → 10 / `compute_stance_and_swing_torque` | 正しい |  |
| Swing Cartesian PD → 10 / cartesian関数 | 不完全 | ID項・摩擦補償・PD二重が索引に無い |

### `appendices/D_File_Function_Index.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| `compute_stance_and_swing_torque` | 正しい |  |
| cartesian / joint_space未呼出 | 正しい |  |
| ESD標準無効 → 16 | 正しい | 有効時の局所性は10にある |
| IKはtau未使用 | 正しい（標準sim） | ROS2では関節目標を送る |
| swing_generators / `update_swing_time` | 記載なし | 不完全 |

未確認: `qfrc_passive` のXML内訳（減衰係数の数値）。`M` 3×3近似の定量誤差。explicit Generatorの絶対高さzが意図かバグか。
