# Experiments and Research Roadmap

実験仕様の正本である。詳細な合格表と優先度の根拠は調査証跡 [analysis-logs/21](analysis-logs/21_experiment_research_roadmap.md) にある。本文へログを転記しない。

## 1. 原則

- 一度に変更する主変数は1群にする。
- wrapper HEAD、PyMPC tree識別子、Config、Random seed、Sceneを保存する。[00](00_README.md)
- 成功だけでなく失敗率と最悪条件を記録する。
- 目標GRFと実GRFを区別する。[11](11_Joint_Torque_and_MuJoCo_Closed_Loop.md)

合格条件に根拠のない数値は書かない。使うのは Baseline比、ロボット仕様値以内、solver deadline以内、転倒なし、constraint violationなし、実験で決める暫定値。

## 2. 実験段階

段階3（低速Trot、`flat`、摩擦一点、固定指令）を以降のBaseline比の基準にする。ディスク既定の `perlin` と `friction_coeff=(0.5,1.0)` のまま段階3を取ると歩行と不整地が交ざる。

| 段階 | 目的 | 変更する変数だけ | 合格の型 |
|---:|---|---|---|
| 1 Full stance | 4脚支持 | `gait='full_stance'`、指令0 | 転倒なし。\(\tau\) は 0.9 ctrlrange 以内 |
| 2 Swing単脚 | 遊脚軌道 | 評価窓のみ | 転倒なし。\(\tau\) 定格内 |
| 3 低速Trot | 閉ループ基準 | 小さい一定 \(v_x\) | 転倒なし。この記録がBaseline比 |
| 4 速度Step | 加減速と歩幅clip | \(v^{ref}(t)\) のみ | 転倒なし。数値は暫定 |
| 5 Frequency/Duty | 幾何と歩行 | L4の `f`×`d` のみ | 転倒なしの集合を記録 |
| 6 Weight | 追従と力 | L2の**1軸ずつ** | Baseline比のトレードオフ |
| 7 Friction | 計画μと床μ | 床μ **または** MPC `mu` の一方 | 転倒なし下限は暫定 |
| 8 External disturbance | 外力復帰 | Plant `xfrc_applied` のみ。補償はゼロ | 転倒なし |
| 9 段差 | 盲歩行 | L9 `scene` のみ。VFA=blind | 転倒なしを要求しない。flat比 |
| 10 穴・飛び石 | 3集合欠如の観測 | scene。VFAはまだOFF | 挙動を記録したこと |
| 11 Solver stress | statusと時間 | horizon / QP iter / 急指令の1つ | 実測deadline。失敗時は前回GRF |
| 12 Gradient vs Sampling | 方式差 | `type` だけ。Cost完全一致は不可 | §4の公平条件 |
| 13 Domain randomization | 過適合 | 環境因子のみ。制御θ固定 | 平均と最悪 |
| 14 Sim-to-Real準備 | 実機前 | 制御は段階3相当 | 自動合格なし。人が上限を決める |

ログは観測専用。設計は§8。未実装の間は目視+終了理由と宣言する。

## 3. 1変数群（同時変更禁止）

| ID | レイヤー | このコードでの実体 |
|---|---|---|
| L1 | Prediction model | SRBD / Jax力学、外力 |
| L2 | Cost | `set_weight`、Sampling Q/R |
| L3 | Constraint | 摩擦、足箱、安定、遊脚F=0 |
| L4 | Gait | type, `step_freq`, `duty_factor`, offset |
| L5 | Foothold | FRG clip、VFA |
| L6 | Swing | Kp/Kd、`step_height` |
| L7 | Low-level | clip 0.9、関節PD（sim無効） |
| L8 | State estimation | 現行は完全状態 |
| L9 | Terrain perception | `scene`、HeightMap、VFA |

禁止例: VFAとfreqとQを同時変更。Sampling切替と同時に duty を 0.65 へ「合わせる」（それは比較条件の固定であり改善実験ではない）。

## 4. Gradient vs Sampling の公平比較

`type` 以外を揃える。**現行Costは一致しない**（足位置Q、GRFのR無効、積分なし、足固定）。報告に「同一Cost」と書かない。[F](appendices/F_Open_Questions.md)

揃えるもの: 同じXMLと質量/慣性定数、同じseedと `reset(random=False)`、同じ \(v^{ref}(t)\) と `ref_z`、同じ外側PGG（`optimize_step_freq=False`、Sampling adaptive不使用）、同じsceneと一点摩擦、同じ評価時間、同じ 0.9 clip と VM。

勾配の慣性再計算は比較時に両方オフにして `config.inertia` へ揃える（標準ONだと勾配だけ姿勢依存）。Hardwareと wall-clock（1周期solve、周期超過率）を記録する。最初の地形は flat + 段階3指令。

## 5. 評価指標

\[
E_v=\frac1T\int\|v-v^{ref}\|^2dt,\qquad
E_{ori}=\frac1T\int(\phi^2+\theta^2)dt
\]

加えて Torque peak、Energy proxy、Slip、Impact、Fall rate、Solver failure rate。変数の生成元は[A](appendices/A_Variable_Dictionary.md)。

## 6. 研究候補の優先

| 優先 | 候補 | 最初に必要な段階 | 注 |
|---:|---|---|---|
| 1 | \(v,f,d\) Feasibility envelope | 5 | 制御非変更 |
| 2 | 遊脚GRFの明示ゼロ制約 | 3 | L3のみ。[09](09_MPC_Output_and_Receding_Horizon.md) |
| 3 | Reachability / Timing foothold | 3–5 | 標準未実装。[13](13_Feasibility_on_Rough_Terrain.md) |
| 4 | GRF Residual（オフライン）と Safe stopping | 3+7 | 停止はsimオフ |
| 5 | Gradient vs Sampling、Domain randomization | 3 と §4、7+9 | 方式比較は改善ではない |
| 6 | Terrain-aware TD timing | 10の失敗記録 | Planner層が無い |
| 7 | Outer-loop tuning | 3+6 | tuner未実装。[15](15_Automatic_Tuning_and_Sim_to_Real.md) |

## 7. Cursor実験プロンプト

```text
Baselineを変更せず、実験用のログ追加だけを提案してください。
評価変数の生成元、shape、単位、保存周期を表にし、既存制御周期へ影響しない設計にしてください。
```

## 8. Baseline固定とログ・テスト

制御式、重み、Gait、clip、`mj_step`入力を変えない。ログとテストは観測専用。実装は依頼があるまで行わない。

### 8.1 Baselineの固定方法

| 項目 | 固定値 |
|---|---|
| wrapper HEAD | 実験開始時の `mpc_dog` HEAD。作業開始時記録は `3adfad9`。[00](00_README.md) |
| Quadruped-PyMPC | git外。zip参考 `cc145a2` または treeハッシュ |
| gym-quadruped | 1.1.5 |
| Go2 XML | `gym_quadruped/robot_model/go2/go2.xml` |
| Config | `type='nominal'`, trot 1.35/0.74, `dt=0.002`, MPC 100 Hz, blind, `optimize_step_freq=False` |
| Terrain | 歩行基準は **`flat`**。ディスク既定 `perlin` は段階9用 |
| 摩擦 | **一点**。範囲 `(0.5,1.0)` は段階7 |
| Initial | `reset(random=False)` |
| 時間 | 段階3は 20 s（暫定） |
| Viewer | 再現は **off** |
| 指令 | 固定 \(v_x\)。`human`+キーは使わない |

変更禁止面: `WBInterface`、`SRBDControllerInterface`、`Acados_NMPC_Nominal.compute_control` の入力生成と出力Mask、`compute_stance_and_swing_torque`、`action`組立、`0.9 * ctrlrange` clip。

### 8.2 ログ追加の設計

制御ループの後ろ、`env.step`の前後にcopyだけを置く。新しい`if`でトルクや接触を書き換えない。

| 変数 | 生成元 | shape | 単位 | Frame | 推奨保存周期 |
|---|---|---|---|---|---|
| `qpos` / `qvel` | `mjData` | `(19,)`, `(18,)` | 混在 | MuJoCo | 500 Hzまたは100 Hz間引き |
| `com`, `base_pos`, Euler | env getters | `(3,)` | m / rad | W / SciPy xyz | 100 Hz |
| `_ref_base_lin_vel_H`, `ref_base_lin_vel` | env / `target_base_vel` | `(3,)` | m/s | H / W | 指令変更時と100 Hz |
| `contact_sequence[:,0]`, 実contact | PGG / `feet_contact_state` | `(4,)` | 0/1 | なし | 500 Hz。意味が違う |
| `nmpc_GRFs.*` | `SRBDControllerInterface` | 各`(3,)` | N | W | 100 Hz |
| 実GRF | `feet_contact_state(ground_reaction_forces=True)` | 各`(3,)` | N | W | 100 Hz。トルク計算には使わない |
| `nmpc_footholds.*`, `feet_pos.*` | MPC / env | 各`(3,)` | m | W | 100 Hz |
| `tau.*`, `action`, saturation flag | WBC / clip | `(3,)` / `(12,)` / bool | N·m | 関節 / actuator | 500 Hz |
| Solver status, solve time | acados | scalar | — / s | なし | 100 Hz |
| `geom_friction` 足と床 | reset直後 | `(3,)` | — | geom | episodeごと |

正本の変数は[A](appendices/A_Variable_Dictionary.md)。

### 8.3 テスト追加の設計

| 種類 | 何を固定するか | 制御コードへの侵入 |
|---|---|---|
| I/O契約 | `action.shape==(12,)`, clip後が`0.9*ctrlrange`内, `contact_sequence.shape==(4,12)` | なし |
| 周期 | `step_num % 5 == 0`のときだけMPC更新 | なし |
| 無効経路 | 標準でVFA、`optimize_step_freq`、RTI、ESD、関節PDが呼ばれない | なし |
| Plant定数 | `nq==19`, `nv==18`, `nu=12`, センサ16, hip/thigh ±23.7, calf ±45.43 | なし |

最初の実装単位は「episode終了後にnpzへ書くhook」と shape/周期のpytest。新しい制御分岐を足さない。
