# MPC Output and Receding Horizon

## 1. 結論

MPCはホライズン全体の状態・入力系列を求めるが、実行するのは先頭段の制御だけである。次周期に新しい状態で解き直す。

本章がreceding、先頭`u`、遊脚GRFの3段扱い、Mask、`perform_scaling`、遊脚足teleport、`nmpc_predicted_state`の正本である。OCPコストと制約の定式化は[07](07_MPC_Formulation.md)。接触を力学へ入れる意味は[08](08_Gait_MPC_Coupling.md)。SRBD式は[06](06_Centroidal_SRBD_Model.md)。

標準`nominal`の状態は30次元（胴体12 + 足12 + 積分6）、入力は24次元である。

## 2. 内部解

\[
x_0^*,x_1^*,\ldots,x_N^*
\]

\[
u_0^*,u_1^*,\ldots,u_{N-1}^*
\]

標準入力のIndexは、

| Index | 内容 | Shape | 単位 | Frame |
|---:|---|---|---|---|
| `0:3` | FL足先速度 | `(3,)` | m/s | W |
| `3:6` | FR足先速度 | `(3,)` | m/s | W |
| `6:9` | RL足先速度 | `(3,)` | m/s | W |
| `9:12` | RR足先速度 | `(3,)` | m/s | W |
| `12:15` | FL GRF | `(3,)` | N | W |
| `15:18` | FR GRF | `(3,)` | N | W |
| `18:21` | RL GRF | `(3,)` | N | W |
| `21:24` | RR GRF | `(3,)` | N | W |

対応コード: `centroidal_model_nominal.py` の `Centroidal_Model_Nominal.__init__`（`self.inputs`）。

## 3. 求解前処理

### 3.1 `perform_scaling`

名前はscalingだが、実体は現在`state['position']`を原点へ平行移動する処理である。

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|---|
| `state`, `reference` | dict | 混在 | W | 同キー。positionを0にし、足と`ref_position`/`ref_foot_*`から同じベクトルを引く | dict | 混在 | 原点相対W |

対応コード: `Acados_NMPC_Nominal.perform_scaling()`。

### 3.2 遊脚足のteleport

`contact_sequence[i,0]==0`の脚は、OCP初期状態の足位置を`reference['ref_foot_*'][0]`に置き換える。

対応コード: `Acados_NMPC_Nominal.compute_control()` の `if FL_contact_sequence[0] == 0:` ブロック。

### 3.3 外部wrench

`mpc_params['external_wrenches_compensation']=True`でも、Wrapperは`external_wrenches`を渡さない。標準経路に内部推定器はない。既定は`zeros(6,)`。なぜフラグがTrueのままかは[F](appendices/F_Open_Questions.md)。

## 4. 先頭入力

\[
u_0^*=\mathrm{solver.get}(0,u)
\]

```python
control = solver.get(0, "u")
optimal_GRF = control[12:]
```

| 数式 | コード変数 |
|---|---|
| \(u_0^*\) | `control` `(24,)` |
| \(F^{MPC}\) | `optimal_GRF` `(12,)` |

足先速度の先頭値をそのままSwing actuatorへ送るのではなく、予測状態から次のTouchdown Footholdを抽出し、Swing controllerの終点にする。

対応コード: `Acados_NMPC_Nominal.compute_control()`。

## 5. 外部出力

`SRBDControllerInterface.compute_control()`が整形する。

| 出力 | shape | 単位 | frame | 備考 |
|---|---|---|---|---|
| `nmpc_GRFs.*` | 各`(3,)` | N | W | 下節のMask後 |
| `nmpc_footholds.*` | 各`(3,)` | m | W | decenter後。立脚は現在足、遊脚は次TDまたは参照 |
| `nmpc_predicted_state` | `(24,)` | 混在 | W（decenter後） | `get(k,"x")[0:24]`。`dt<=0.02`なら`k=2` |
| `nmpc_joints_*` | `None` | — | — | `nominal`では未使用 |
| `best_sample_freq` | scalar | Hz | — | 標準では入力の`pgg.step_freq`を返す |

Wrapper初期化の`nmpc_predicted_state = np.zeros(12)`は、初回solve前のプレースホルダである。solve後は`(24,)`。IKへの代入行はコメントアウト。[10](10_Stance_and_Swing_Control.md)。

Kinodynamic型では関節目標も存在する。標準`nominal`では作らない。

## 6. 遊脚GRFの3段（正本）

遊脚の目標GRFは、1つの等式では決まらない。標準`nominal`では次の3段が同時に存在する。

| 段 | 何が起きるか | 遊脚 \(c_i=0\) のとき | 対応コード |
|---|---|---|---|
| 1. 力学Gate | \(\dot v,\dot\omega\) に \(c_i F_i\) だけが入る | 胴体予測へ寄与しない | `centroidal_model_nominal.forward_dynamics` |
| 2. OCP制約・コスト | 摩擦錐と \(F_z\in[0,mg]\) は**全脚常時**。入力コストの参照は遊脚 \(F_z^{ref}=0\) | 等式 \(F_i=0\) ではない。摩擦は残る | `create_friction_cone_constraints`, `set_weight`, yref |
| 3. 出力Mask | 先頭接触を掛ける | 指令は必ず0 | `SRBDControllerInterface.compute_control` |

\[
F_i^{\mathrm{command}}=c_{i,0}F_i^{\mathrm{MPC}}
\]

| 数式 | コード変数 | shape | 単位 | frame | 周期 |
|---|---|---|---|---|---|
| \(c_{i,0}\) | `current_contact[i]`（`contact_sequence[:,0]`） | `(4,)` の1要素 | 0/1 | なし | 500 Hz生成、Maskは100 Hz |
| \(F_i^{MPC}\) | `control[12:]` の脚ブロック | `(3,)` | N | W | 100 Hz |
| \(F_i^{\mathrm{command}}\) | `nmpc_GRFs.FL` 等 | `(3,)` | N | W | 100 Hz。非更新時は保持 |

したがって「指令がゼロ」と「OCP内部で厳密ゼロ」は同じではない。旧説明の理由は[E](appendices/E_Corrections_and_Clarifications.md) §4。設計意図（なぜ等式ゼロにしないか）は[F](appendices/F_Open_Questions.md)。

明示的に \(0\le F_{z,i,k}\le c_{i,k}F_{z,max}\) を入れるのは **推奨改善** であり、標準OCPには無い。[08](08_Gait_MPC_Coupling.md) §7。

対応コード: `srbd_controller_interface.py` の `SRBDControllerInterface.compute_control()`。摩擦とyrefは[07](07_MPC_Formulation.md)。

## 7. 100 Hzと500 Hz

MPCを100 Hz、MuJoCo/低レベルを500 Hzで動かす場合、5つのSimulation stepに1回MPCを更新する。

```text
step_num % round(1 / (mpc_frequency * simulation_dt)) == 0
# 100 * 0.002 → 5
```

間の周期は直近のGRF/Footholdを使い、現在Jacobianでトルクを再計算する。周期表と閉ループ境界の正本は[02](02_System_Architecture_and_Dataflow.md)。

対応コード: `quadruped_pympc_wrapper.py` の `QuadrupedPyMPC_Wrapper.compute_actions()`。

## 8. 対応コード

- `quadruped_pympc_wrapper.py`: MPC更新条件
- `interfaces/srbd_controller_interface.py`: 出力整形・Mask
- `centroidal_nmpc_nominal.py`: `perform_scaling`, Solver解取得・Foothold抽出

## 9. Cursor確認課題

MPC solve時刻、GRF保持期間、Torque計算時刻をログ化し、実際のZero-order hold構造を時系列図にする。
