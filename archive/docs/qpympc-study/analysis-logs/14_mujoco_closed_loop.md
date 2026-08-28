# Log 14: Torque → MuJoCo Action → 次状態

対応プロンプト: `tau.*` から `mjData.ctrl`、`mj_step` 後の `qpos/qvel/contact/実GRF`、次周期 `state_current` まで。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

標準: `simulation.py` の `run_simulation`、`sim_dt=0.002`、1 step = 1 `mj_step`、PD加算はコメントアウト。

## 処理境界（全体）

| 順序 | 上流出力 | Shape | 単位 | Frame | 下流入力 | 更新周期 |
|---|---|---|---|---|---|---|
| 1 | `tau.FL/FR/RL/RR`（WBC直後） | 各`(3,)` | N·m | 関節 | clip | 500 Hz |
| 2 | clip後 `tau.*` | 各`(3,)` | N·m | 関節 | `action[legs_tau_idx.*]` | 500 Hz |
| 3 | `action` | `(12,)` | N·m | actuator順 | `env.step(action)` → `mjData.ctrl` | 500 Hz |
| 4 | `mjData.ctrl` | `(12,)` | N·m | 同上 | `mujoco.mj_step` | 1回/制御周期 |
| 5 | 接触 \(\lambda\) | 接触ごと | N | 接触frame | `qacc` | `mj_step`内部 |
| 6 | `mjData.qpos` | `(19,)` | 混在 | §下記 | 次loopの状態読取 | 500 Hz |
| 7 | `mjData.qvel` | `(18,)` | 混在 | §下記 | 同上 | 500 Hz |
| 8 | `mjData.contact` | 可変 | — | — | ESD（標準無効）、viewer GRF | 500 Hz |
| 9 | `com`,`base_*`,`feet_*`,`J` | 各種 | 各種 | W/B | `update_state_and_reference` | 500 Hz |
| 10 | `state_current` | dict | 混在 | W/B | 次回MPC（100 Hzでsolve） | 500 Hz生成 |

---

## 1. Torque assembly

| # | 項目 | 結果 |
|---|---|---|
| 1 | 各脚Torque shape | WBC戻りは各`(3,)`。初期化だけ `np.zeros((nv,1))=(18,1)` だが初回上書き |
| 2 | 脚順 | `legs_order=["FL","FR","RL","RR"]`。代入もこの順 |
| 3 | `legs_tau_idx` | `QuadrupedEnv.__init__`。各脚の `robot_cfg.leg_joints`（hip, thigh, calf）の `tau_idx` |
| 4 | Action格納順 | `action[legs_tau_idx.FL]=tau.FL` 等。Go2では index 0–11 が FL→FR→RL→RR |
| 5 | Joint order | 脚内 hip, thigh, calf |
| 6 | Actuator一致 | XML `<motor>` 順と `legs_tau_idx` は一致（Go2 default `leg_joints`） |
| 7 | dtype | `np.zeros(nu)` は **float64**。`action_space` は float32。代入はキャストなし |
| 8 | 単位 | N·m。`gainprm=1`, `gear=1` なので `ctrl=τ` |

```python
action = np.zeros(env.mjModel.nu)          # (12,) float64
action[env.legs_tau_idx.FL] = tau.FL
...
self.mjData.ctrl = action                  # QuadrupedEnv.step
```

| Action index | 脚 | Joint | Torque変数 | Actuator名 | ctrlrange [N·m] |
|---|---|---|---|---|---|
| 0 | FL | `FL_hip_joint` | `tau.FL[0]` | `FL_hip` | ±23.7 |
| 1 | FL | `FL_thigh_joint` | `tau.FL[1]` | `FL_thigh` | ±23.7 |
| 2 | FL | `FL_calf_joint` | `tau.FL[2]` | `FL_calf` | ±45.43 |
| 3 | FR | `FR_hip_joint` | `tau.FR[0]` | `FR_hip` | ±23.7 |
| 4 | FR | `FR_thigh_joint` | `tau.FR[1]` | `FR_thigh` | ±23.7 |
| 5 | FR | `FR_calf_joint` | `tau.FR[2]` | `FR_calf` | ±45.43 |
| 6 | RL | `RL_hip_joint` | `tau.RL[0]` | `RL_hip` | ±23.7 |
| 7 | RL | `RL_thigh_joint` | `tau.RL[1]` | `RL_thigh` | ±23.7 |
| 8 | RL | `RL_calf_joint` | `tau.RL[2]` | `RL_calf` | ±45.43 |
| 9 | RR | `RR_hip_joint` | `tau.RR[0]` | `RR_hip` | ±23.7 |
| 10 | RR | `RR_thigh_joint` | `tau.RR[1]` | `RR_thigh` | ±23.7 |
| 11 | RR | `RR_calf_joint` | `tau.RR[2]` | `RR_calf` | ±45.43 |

ctrlrange根拠: `01` / `mjModel.actuator_ctrlrange`。`forcerange` は `[0,0]` かつ `forcelimited=False` でこのclipには未使用。

---

## 2. Torque saturation

\[
\tau^{cmd}=\operatorname{clip}(\tau,\,s\tau_{min},\,s\tau_{max})
\]

| 数式 | コード |
|---|---|
| \(s\) | `tau_soft_limits_scalar = 0.9` |
| \(\tau_{min},\tau_{max}\) | `env.mjModel.actuator_ctrlrange[legs_tau_idx.*]` |
| \(\tau\) | clip前の `tau.LEG` |
| \(\tau^{cmd}\) | clip後の `tau.LEG` → `action` |

| 項目 | 結果 |
|---|---|
| 関数 | `np.clip(tau[leg], tau_min, tau_max)`。脚ごと3成分 |
| ctrlrange取得元 | `mjModel.actuator_ctrlrange`（XML motor） |
| 係数 | 0.9 固定。設定キーではない |
| 上下限 | hip/thigh ±21.33、calf ±40.887 |
| 周期 | 毎制御周期（500 Hz）。MPC内外どちらでも同じ |
| clip前 | WBC + 摩擦補償後 |
| clip後 | `action` に入る値 |
| saturation flag | **無い**。飽和したかはどこにも残さない |
| MPC内Torque limit | **無い**。OCPはGRF制限のみ。姿勢依存の \(\tau=J^\top F\) 実現可能性は見ない |

MuJoCoが `ctrllimited` なら `mj_step` 内でさらにフル `ctrlrange` へclipし得る。その場合もソフト限界 0.9 が先に効く。`ctrllimited` の実行時値は本ログでは未再読（`01` は ctrlrange の存在を記録）。

---

## 3. MuJoCo入力

| # | 項目 | 結果 |
|---|---|---|
| 1 | `mjData.ctrl` | `QuadrupedEnv.step`: `self.mjData.ctrl = action` |
| 2 | `mj_step` | 直後 `mujoco.mj_step(self.mjModel, self.mjData)` |
| 3 | step回数 | **1制御周期 = 1 `env.step` = 1 `mj_step`** |
| 4 | timestep | `simulation_params['dt']=0.002` → `mjModel.opt.timestep` |
| 5 | Frame skip | **無し**。`nstep` ループ無し |
| 6 | Render | 壁時計 30 Hz。`time.time()` 間隔。sim時刻同期ではない |
| 7 | Reset | 起動時 `reset(random=False)`。episode終了時 `reset(random=True)` + wrapper.reset |
| 8 | Keyframe | `mj_resetDataKeyframe(..., 0)` = XML `home`。`random=False` はノイズ無し。`random=True` は関節・xy・姿勢に乱数 |

`step_num` は `env.step` 末尾で +1。MPC更新は `step_num % 5 == 0`。

---

## 4. 目標GRFと実GRF

| 値 | コード変数 | 生成元 | 単位 | Frame | 物理的意味 |
|---|---|---|---|---|---|
| OCP生GRF | `optimal_GRF` / `control[12:]` | `solver.get(0,"u")` | N | W* | 予測モデル上の目標力 |
| 指令GRF | `nmpc_GRFs.*` | 上 × `current_contact` | N | W | `-J.T@F` に使う力。遊脚0 |
| 写像に使うGRF | 同じ `nmpc_GRFs.*` | 同上 | N | W | 足が受ける目標GRF |
| 実接触力 | `mj_contactForce` の和 | MuJoCo接触ソルバー | N | 接触→W（`R_c.T @ f[:3]`） | 拘束が生んだ力 |
| gym表示GRF | `feet_contact_forces.*` | `feet_contact_state(ground_reaction_forces=True)` | N | W（指定可） | 足–地面接触の合計。**viewerのみ** |
| 実接触判定 | `contact_state.LEG` (bool) | 足bodyとworldの接触 | 0/1 | なし | 表示・終了判定。制御切替には未使用 |
| 予定接触 | `current_contact` | PGG `contact_sequence[:,0]` | 0/1 | なし | MaskとStance/Swing |

\*OCP内部は原点相対だが力は並進不変。

確認:

1. **MPC GRFをMuJoCoへ直接入力しているか?** いいえ。入るのは12トルクだけ。
2. **実GRFを次MPCへFeedbackしているか?** いいえ。`state_current` にGRFキーは無い。
3. **実GRFは状態推定や接触判定に使うか?** 標準制御では使わない。`current_contact` はPGG。`mjData.contact` はESDへ渡るが `reflex=False` で無効。viewerだけ実GRFを読む。
4. **目標と実GRFの誤差補償は?** 無い。積分補償も標準オフ。
5. **Torque saturationをMPCが認識するか?** 否。flagも無く、clip後τはOCPに戻らない。

---

## 5. Joint PD

| 項目 | 結果 |
|---|---|
| 計算コード | wrapper 201–203行。コメントアウト |
| Kp / Kd | `impedence_joint_position_gain=10`, `impedence_joint_velocity_gain=2`。読まれるが加算されない |
| \(q_d\) | `des_joints_pos`（IK） |
| \(\dot q_d\) | `des_joints_vel`（`pinv(J)@v_d`） |
| FF加算 | 実装案は `tau += Kp e + Kd ev`。標準では走らない |
| 標準sim | **無効** |
| 別Interface | ROS2 `run_controller.py` が `pd_target_joints_*` と kp/kd を `TrajectoryGenerator` でpublish。トルクも別途publish |

「Joint PDがない」= 標準simは関節角サーボを足さない。
「閉ループがない」ではない。胴体誤差 → MPC GRF → `-J.T F` / swing → τ → 接触 → 次状態、という閉ループがある。E §1 と同じ区別。

---

## 6. Feedback loop（変数を省略せず）

```text
tau.FL/FR/RL/RR
→ np.clip(..., 0.9*actuator_ctrlrange)
→ action[legs_tau_idx.*]
→ QuadrupedEnv.step: mjData.ctrl = action
→ mujoco.mj_step
→ mjData.qpos (19,), mjData.qvel (18,), mjData.contact
→ 次周期:
     env.com → com_pos
     env.base_pos → base_pos
     env.base_lin_vel(world) → base_lin_vel
     env.base_ang_vel(base) → base_ang_vel
     env.base_ori_euler_xyz
     env.feet_pos(world) / feet_vel / hip_positions
     env.feet_jacobians / jacobians_dot
     env.legs_mass_matrix / qfrc_bias / qfrc_passive
     env.target_base_vel() → ref_base_lin_vel, ref_base_ang_vel
→ WBInterface.update_state_and_reference(...)
     → state_current['position']=com_pos+com_pos_offset_w
     → state_current['linear_velocity']=base_lin_vel
     → state_current['orientation']=base_ori_euler_xyz
     → state_current['angular_velocity']=base_ang_vel
     → state_current['foot_*']=feet_pos.*
→ step_num%5==0 のとき SRBDControllerInterface.compute_control(state_current, ...)
```

実接触力とPGG接触はここでは合流しない。

`qpos` 索引: 0:3 base位置、3:7 quat wxyz、7:19 12関節。
`qvel` 索引: 0:3 並進W、3:6 角速度（`base`読取は qvel[3:6]）、6:18 12関節。

---

## 資料照合

### `11_Joint_Torque_and_MuJoCo_Closed_Loop.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| 最終出力は12トルク。MPC GRFは直接入れない | 正しい | なし |
| assemblyコードと脚順 | 正しい | dtype float64 / 初期(18,1)は未記載 |
| clip = 0.9 ctrlrange。forcerange未使用 | 正しい | saturation flag無しは未記載 |
| 1 step = 1 mj_step、dt=0.002 | 正しい | なし |
| 目標GRF vs λ | 正しいが表が短い | 5種の区別は本ログ |
| PDなし≠閉ループなし | 正しい | なし |
| 関節PDコメントアウト | 正しい | ROS2利用は未記載 |
| Feedbackは次状態再取得 | 正しい | 実GRF非フィードバックを明示するとよい |

### `01_MuJoCo_Go2_Plant_Model.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| ctrlrange hip/thigh ±23.7、calf ±45.43 | 正しい（本ログは再XML未読、01に従う） |  |
| action 12、ctrl=τ | 正しい |  |
| 1回 mj_step、Keyframe home、初回 random=False | 正しい | episode中の reset(random=True) は11側 |
| 実GRFはviewer、トルク計算未使用 | 正しい |  |
| Plant→制御の契約表 | 正しい | `mjData.contact` を次MPCへは渡さない点は一致 |

### `appendices/A_Variable_Dictionary.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| `tau` / `action` / `mjData.ctrl` | 正しい |  |
| `nmpc_GRFs` = Mask後目標 | 正しい |  |
| 実GRF / `feet_contact_forces` / `mjData.contact` | **未掲載** | 不完全 |
| `current_contact` は予定接触 | 正しい | 実接触との混同防止は十分 |

### `appendices/E_Corrections_and_Clarifications.md`

| 記載 | 判定 | 差分 |
|---|---|---|
| §1 PD必須ではない。閉ループはGRF→τ | 正しい | なし |
| §10 MPC GRFをMuJoCoへ直接入力しない | 正しい | 本ログで再確認 |

未確認: `actuator_ctrllimited` の実行時値。`mj_contactForce` の符号が「足が受ける力」か「地面が受ける力」か（viewer表示には使うが制御未使用）。
