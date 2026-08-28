# Feasibility on Rough Terrain

## 1. 結論

地形上安全なFootholdを選ぶだけでは不十分である。その位置が脚可動域内で、残りSwing時間までに到達でき、選択したGait・速度で胴体を支持できる必要がある。

次の3集合の交差は**理論上の必要条件**である。標準コードに交差を取る関数は無く、到達不能でも目標をそのまま使う。標準は`blind`（HeightMapなし）。VFAは optional（`height` / `vfa`）。`vfa` は未公開 `virall` に依存する。

## 2. 3つの集合（理論。標準未実装）

Touchdown位置は、

\[
p_{td}\in
\mathcal S_{terrain}\cap
\mathcal R_{kinematic}\cap
\mathcal R_{timing}
\]

を満たす必要がある。

### 地形安全集合

\[
\mathcal S_{terrain}
=
\{p\mid
\text{穴でない、傾斜・面積・端部余裕が許容}
\}
\]

### 運動学可到達集合

\[
\mathcal R_{kinematic}
=
\{p\mid q_{min}\le IK(p)\le q_{max}\}
\]

### Timing可到達集合

\[
\mathcal R_{timing}
=
\{p\mid
\|p-p_{lo}\|
\le
v_{foot,max}T_{swing,remaining}
\}
\]

## 3. 不整合の例

Nominal Footholdが穴にあるため、安全位置へずらした結果、予定Touchdown時刻までに足が届かない場合がある。

\[
\frac{\|p_{td}^{safe}-p_{lo}\|}
{T_{swing,remaining}}
>
v_{foot,max}
\]

この場合、位置変更だけでは解決しない。

## 4. 対応の順序

1. 安全集合内でFootholdを小さく変更する。
2. 目標速度を下げ、必要歩幅を短くする。
3. Step frequencyを変える。
4. Duty factorを変え、Swing/stance時間を再配分する。
5. Touchdown時刻または位相をずらす。
6. Gaitを変更する。
7. 安全位置がなければ停止する。

## 5. Plannerに必要な出力

不整地Plannerは位置だけでなく、少なくとも次を整合させる。

\[
\{
p_{td,i},
t_{td,i},
c_i(t),
v_{base}^{feasible}
\}
\]

| 出力 | 意味 | Shape | 単位 | Frame |
|---|---|---|---|---|
| \(p_{td,i}\) | 着地点 | `(3,)` | m | W |
| \(t_{td,i}\) | 着地時刻 | scalar | s | なし |
| \(c_i(t)\) | 接地予定 | `(4,)` 時系列 | 0/1 | なし |
| \(v_{base}^{feasible}\) | 実現可能な胴体速度 | `(3,)` | m/s | 指令境界 |

## 6. Quadruped-PyMPCの範囲

標準構成は、固定または候補選択されたGait TimingからNominal Footholdを作り、VFAとMPCで位置を調整する。速度、Gait type、frequency、Duty、接地時刻、地形を完全同時最適化しない。

そのため大きな穴や飛び石地形では、上位の速度・接地再計画が必要になる。

## 7. 研究課題

- Terrain-aware timing adjustment
- Reachability-aware foothold constraints
- Gait/velocity/Foothold joint optimization
- Contact-implicit MPC
- Failure-aware stopping policy

## 8. Cursor確認課題

交差制約は未実装と確定している。追加する場合は[18](18_Experiments_and_Research_Roadmap.md)の研究候補（Reachability / Timing）として、段階10の失敗記録のあと。