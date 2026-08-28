# Open Questions

公開コードだけでは最終確定できない、または研究対象として残る項目である。実装事実として確定した項は本文と[E](E_Corrections_and_Clarifications.md)へ移した。

## 実機・同定

- Go2実機で使用した最終MPC重みとSwing gain
- 実床面ごとの摩擦係数
- State estimatorの遅延・Noise
- Unitree low-level motor controllerの実効帯域
- Torque commandから実Torqueまでの遅延
- Payload・Batteryによる変化

## MPC実装（意図・妥当性。事実は本文）

- 遊脚GRFを明示ゼロ制約にしない**設計意図**（コードコメントからは確定できない）。実装事実の3段は[09](../09_MPC_Output_and_Receding_Horizon.md) §6
- Solver failure時Fallbackの全実機安全性
- 積分状態の離散化・更新方法の妥当性
- Euler angle modelの高姿勢角限界
- Foot velocity decisionの外部利用方法
- `external_wrenches_compensation=True` なのに Wrapper が `zeros(6,)` を渡す**意図**。内部推定は無い（[E](E_Corrections_and_Clarifications.md) §26）
- `QuadrupedEnv.com` は全bodyについて `body_mass[i] * subtree_com[i]` を足して割る。これがMuJoCoの物理的総CoMと一致するかは未検証
- `base_ang_vel(frame='base')` は `qvel[3:6]` を返す。MuJoCo freejointの公式定義との一致は未検証
- `get_base_inertia()` の `mj_fullM` ブロック `[3:6,3:6]` を flatten した `(9,)` の、MPC慣性パラメータとしての厳密なframe意味

## Gait/Foothold

- Frequency候補最適化の実機適用範囲（機能自体は標準OFF。中身は[12](../12_Speed_Frequency_Duty_and_Stride.md) §6）
- 速度ごとのGait切替Envelopeの数値境界（自動切替は未実装）
- VFAと残りSwing時間の整合検査は**未実装**（確定）。残時間制約を足した場合の挙動は未検証
- 安全Footholdがない場合の減速・停止。simでは `start_and_stop` がオフ。連動Plannerは無い
- Contact timingを変更する上位PlannerとのInterface

## Plant / XML

- gym-quadruped同梱`go2.xml`とMuJoCo Menagerie `unitree_go2/go2.xml`の同一性。このワークスペースにMenagerie checkoutはない
- MJX版Go2との接触・摩擦差。本スタックにMJX経路がない
- `env.com`の総和が物理的総CoMと一致するか（上節と重複）
- XML`diaginertia`と`config.inertia`、`mj_fullM[3:6,3:6]`の3者の関係
- `cone="elliptic"`かつ足`condim=6`の実行接触が、MPCの\(\mu=0.42\)ピラミッド近似とどの程度ずれるか

## 研究上の検証

- GradientとSampling MPCを**同一Costに揃えた場合**の公平比較。現行はCostが一致しない（足位置Q、GRFのR、積分、足固定）
- SRBD誤差を支配する要因
- MuJoCo目標/実GRF差のResidual学習
- Auto-tuned重みのSim-to-Real再現性。Outer-loop tuner自体が未実装
- Reachability制約追加による計算時間増加

各項目は、実装事実・実験結果・推測を分けて追記する。
