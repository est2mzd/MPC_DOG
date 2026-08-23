# Automatic Tuning and Sim-to-Real

## 1. 結論

物理制約と安全判断を完全自動化するのは難しいが、シミュレーションでMPC重み、Gait timing、Swing gainの候補を大量評価して絞ることは可能である。Sampling MPCの入力探索とController重みの外側探索は別物である。

## 2. 既存の自動化（標準ON / 標準OFF / 未実装）

どれも \(Q,R\) の外側探索ではない。Samplingの入力探索とOuter-loopは別物である。

| 分類 | 項目 | 有効条件 | 備考 |
|---|---|---|---|
| 標準ON | 姿勢に応じた慣性再計算 | `use_inertia_recomputation=True` | `get_base_inertia()` → MPC `p` |
| 標準ON | Foothold位置のMPC最適化 | `use_foothold_optimization=True` | 足速度入力。地形安全集合の保証ではない。[05](05_Foothold_Reference_and_Terrain_Adaptation.md) |
| 実装あり・標準OFF | 候補Step frequencyのBatched評価 | `optimize_step_freq=True` | Wrapperがbatchedオブジェクトを作る。中身は[12](12_Speed_Frequency_Duty_and_Stride.md) §6 |
| 実装あり・標準OFF | GPU Sampling / MPPI | `mpc_params['type']='sampling'` | 標準は`nominal`。Costは勾配と一致しない |
| 実装あり・標準OFF | Integral action | `use_integrators=True` | 状態ベクトルには常にある。補償はオフ |
| 実装あり・実質未使用 | External wrench | フラグはTrueだが Wrapper が `zeros(6,)` | [09](09_MPC_Output_and_Receding_Horizon.md) §3.3 |
| 未実装（Outer） | \(Q,R\) / Swing gain のEpisode探索 | — | 本章§3は **推奨改善** |
| 未実装（同定） | Residual / adaptive dynamics | — | Lyapunov等の制約拡張であり、標準の適応同定ではない |

`batched_simulations.py` は同一configの並列Episodeであり、tunerではない。

## 3. 外側のAuto-tuning（推奨改善）

現行コードにOuter-loop tunerはない。次は **推奨改善** である。

調整ベクトルを、

\[
\theta=
[
Q_v,
Q_{angle},
Q_{foot},
R_F,
R_{footVel},
f,
d,
K_p^{swing},
K_d^{swing}
]
\]

とする。各候補でMuJoCo Episodeを実行し、

\[
J_{outer}
=
w_1E_v
+w_2E_{angle}
+w_3E_{slip}
+w_4E_{impact}
+w_5E_{energy}
+w_6N_{fall}
\]

を評価する。

探索器候補：

- Bayesian optimization
- CMA-ES
- Optuna/TPE
- Population-based search
- RLによるGain/Residual調整

## 4. 自動化の順序

1. 安全上のHard limitは固定する。
2. Full stanceで姿勢重みを探索する。
3. 低速TrotでSwing/Gaitを探索する。
4. 速度範囲を広げる。
5. Domain randomizationを加える。
6. 最悪条件と平均性能を両方評価する。
7. 実機は低速度・保守Marginから開始する。

## 5. Domain randomization

標準で乱れているのは `reset` の摩擦サンプルと `scene` 幾何だけである。次の一式は **推奨改善** / 未実装である。

対象：

- 質量・CoM・慣性誤差
- 摩擦係数
- 地面高さ・傾斜
- 状態推定Noise
- 通信・Torque遅延
- Motor strength
- 外力・Payload

調整値が単一Simulation条件へ過適合しないようにする。

## 6. 人が残す判断

- 安全制約の意味とMargin
- 転倒・衝撃の許容基準
- 実機試験の段階的拡大
- Sensor異常・通信喪失時の停止
- 評価関数に現れない不自然な挙動

## 7. Cursor確認課題

Outer-loopは未実装である。設計する場合は[18](18_Experiments_and_Research_Roadmap.md)の段階3・6のあと。`batched_simulations.py`は同一config並列であり、差分tunerではない。