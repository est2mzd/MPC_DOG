# Joint Torque and MuJoCo Closed Loop

## 1. 結論

最終出力は12関節トルクである。MuJoCoはそのトルクと接触から実際のGRF・次状態を求める。MPC GRFは目標であり、MuJoCo GRFを直接設定していない。

本章がclip、`action`組立、`mj_step`の正本である。トルク生成は[10](10_Stance_and_Swing_Control.md)。Go2の`nq/nv/nu`と`ctrlrange`数値は[01](01_MuJoCo_Go2_Plant_Model.md)。

## 2. Torque assembly

各脚3成分の\(\tau\)をMuJoCo actuator indexへ格納する。

```python
action = np.zeros(env.mjModel.nu)  # Go2: (12,)
action[legs_tau_idx.FL] = tau.FL
action[legs_tau_idx.FR] = tau.FR
action[legs_tau_idx.RL] = tau.RL
action[legs_tau_idx.RR] = tau.RR
```

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|---|
| `tau.*` | 各`(3,)` | N·m | 関節 | `action` | `(12,)` | N·m | actuator FL,FR,RL,RR × hip,thigh,calf |
| `legs_tau_idx.*` | 各3 index | なし | なし | | | | |

アクチュエータ順はFL, FR, RL, RR × hip, thigh, calf。周期500 Hz。

対応コード: `simulation/simulation.py` の `run_simulation()`。

## 3. Saturation

Simulationは`actuator_ctrlrange`の一定割合でClipする。Go2の`actuator_forcerange`は`[0,0]`かつ`forcelimited=False`であり、このclipには使わない。数値は[01](01_MuJoCo_Go2_Plant_Model.md)。

\[
\tau^{limited}
=
\operatorname{clip}
(\tau,s\tau_{min},s\tau_{max})
\]

| 数式 | コード変数 |
|---|---|
| \(s\) | `tau_soft_limits_scalar = 0.9` |
| \(\tau_{min},\tau_{max}\) | `env.mjModel.actuator_ctrlrange[legs_tau_idx.*]` |

`s=0.9`は安全余裕であり、MPC内部の姿勢依存Torque feasibilityを保証するものではない。

対応コード: `simulation/simulation.py`。

## 4. MuJoCo step

```python
mjData.ctrl = action
mujoco.mj_step(model, data)
```

1回の`env.step`につき1回の`mj_step`である。`sim_dt=0.002`なら500 Hz。

MuJoCoは[01](01_MuJoCo_Go2_Plant_Model.md) §3 の全身式を解き、次の`qpos`,`qvel`,`contact`を更新する。\(\tau\) は `mjData.ctrl` ← `action`。\(\lambda\) は接触ソルバーであり指令GRFではない。

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|---|
| `action` | `(12,)` | N·m | actuator | `qpos`, `qvel` | `(19,)`, `(18,)` | 混在 | MuJoCo |
| | | | | `mjData.contact` | 可変 | 混在 | 接触 |

対応コード: `gym_quadruped/quadruped_env.py` の `QuadrupedEnv.step()`。全身式の説明は[01](01_MuJoCo_Go2_Plant_Model.md)にもある。

## 5. 目標GRFと実GRF

| 値 | 生成元 | 意味 |
|---|---|---|
| \(F^{MPC}\) | Centroidal OCP の `u[12:24]` | 内部目標GRF。遊脚でも等式ゼロではない |
| \(F^{cmd}\) | Mask \(c_{i,0}F^{MPC}\) | Stanceが使う指令。遊脚は0。[09](09_MPC_Output_and_Receding_Horizon.md) §6 |
| \(\lambda\) | MuJoCo contact solver | 実際の接触拘束力。viewer専用。制御へ戻さない |

差が生じる原因はTorque saturation、モデル誤差、Jacobian、足滑り、Soft contact、遅延、Swing/stance遷移などである。

## 6. PDなしで立てる理由

関節角PDで姿勢を固定する代わりに、胴体高さ・姿勢誤差をMPCがGRFへ変換し、`-J.T @ F`で関節を駆動する閉ループが存在する。

\[
\text{body error}
\rightarrow F^{MPC}
\xrightarrow{\times c_{i,0}} F^{cmd}
\rightarrow\tau
\rightarrow\text{contact}
\rightarrow\text{body state}
\]

したがって「PDがない」ことと「閉ループ制御がない」ことは同じではない。旧誤解の理由は[E](appendices/E_Corrections_and_Clarifications.md) §1。

## 7. 関節PDの位置付け

Wrapperには、

\[
\tau\mathrel{+}
=
K_p(q_d-q)+K_d(\dot q_d-\dot q)
\]

を模擬するコードがあるが、標準経路ではコメントアウトされている。実機ではモータ内部または低レベルControllerへ関節目標・Feedforward torqueを送る構成が一般的である。有効化は **推奨改善**。

対応コード: `quadruped_pympc_wrapper.py` のコメントアウトブロック。ゲインキーは`impedence_joint_position_gain`と`impedence_joint_velocity_gain`。

## 8. Feedback

MuJoCoの次状態から、Base/CoM速度、姿勢、足位置、Jacobian、接触を再取得し、次の制御周期へ戻す。境界表とFeedbackの正本は[02](02_System_Architecture_and_Dataflow.md)。

## 9. 対応コード

- `simulation/simulation.py`: Torque clip、Action assembly
- `gym_quadruped/quadruped_env.py`: `step()`
- `quadruped_pympc_wrapper.py`: Optional joint PD

## 10. Cursor確認課題

目標GRFと`env.feet_contact_state(ground_reaction_forces=True)`の実GRFを同時記録し、各位相の追従誤差を評価する。
