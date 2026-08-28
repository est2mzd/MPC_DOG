# Speed, Frequency, Duty Factor and Stride

## 1. 結論

目標速度、Gait周波数、Duty factor、接地点間隔、脚の胴体相対運動は連成する。定常速度では平均水平GRFは小さくできるが、足が地面に固定される限り、高速・低周波では脚の相対移動量とSwing速度が過大になる。

## 2. \(v=fL\)の正しい意味

\[
L_{footprint}=\frac{v}{f}
\]

ここで\(L_{footprint}\)は、同じ脚の連続する地面上の接地点間隔である。Touchdown時に脚を胴体から\(L_{footprint}\)前へ伸ばす意味ではない。

## 3. 5 m/s、2 Hzの例

周期は、

\[
T=1/f=0.5\ \mathrm{s}
\]

同じ脚の接地点間隔は、

\[
L_{footprint}=vT=2.5\ \mathrm m
\]

例の \(d=0.65\) は bound / `full_stance` の既定である。**標準Trotは \(d=0.74\)**。

\[
T_{stance}=d/f=
\begin{cases}
0.325\ \mathrm s & d=0.65\\
0.370\ \mathrm s & d=0.74
\end{cases}
\]

接地中に胴体が足に対して移動する距離は \(L_{stance}=v T_{stance}\) で、0.65なら 1.625 m、0.74なら 1.85 m。どちらもGo2の脚可動域には非現実的である。

FRGが使う先送りは \(L_{footprint}\) ではなく \(L_{stance}/2\) である。[05](05_Foothold_Reference_and_Terrain_Adaptation.md)。

## 4. StanceとSwing

\[
T_{stance}=\frac{d}{f},
\qquad
T_{swing}=\frac{1-d}{f}
\]

周波数を上げると地面上の印間隔 \(L_{footprint}=v/f\) は短くなる。水平方向の平均足先速度 \(\|p_{td}-p_{lo}\|_{xy}/T_{swing}\) は、clip無しなら \(f\) に依存しない（分子・分母が同じ比率で縮む）。鉛直ステップ、ピーク速度、Foothold clip時は \(f\) で変わり得る。

\[
\bar v_{foot}
=
\frac{\|p_{td}-p_{lo}\|}{T_{swing}}
\]

したがって「周波数を上げれば常に容易」ではない。鉛直と水平を分けて見る。

## 5. 定常速度と加速度

平地・損失なしの定常速度では、

\[
\dot v=0,\qquad\sum_iF_{x,i}\approx0
\]

であり、大きな平均推進力は不要である。しかし運動学的には、胴体が接地足の上を通過し、遊脚を次の接地点へ戻す必要がある。

| 状態 | 主な難しさ |
|---|---|
| 定常高速 | 脚可動域、関節速度、Swing速度、着地衝撃 |
| 急加減速 | 上記＋水平GRF、摩擦、Pitch、Torque saturation |

## 6. 周波数候補評価

**標準は `optimize_step_freq=False`。** 有効なのは Wrapper が `SRBDBatchedControllerInterface` を作るときだけ。[16](16_Code_Map_and_Call_Graph.md)。

有効時の実装事実:

- 候補 \(\mathcal F=\{1.4,2.0,2.4\}\)。標準Trotの 1.35 は含まれない。
- 評価が変えるのは接触列。**候補ごとに Foothold を再計算しない**。
- 目的は \(J_{MPC}\) ではなく \(J_{MPC}+\)周波数penalty（勾配側は \(3(f-1.4)^2\)、Sampling側は別係数）。
- 選んだ \(f\) は `optimize_swing==1` のときだけ `pgg.step_freq` / `frg.stance_time` / `stc.swing_period` に反映する。
- Sampling adaptive の Jax PGG duty は 0.65 で、外側Trot 0.74 とずれ得る。

\[
f^*
=
\arg\min_{f\in\mathcal F}
\bigl(J_{MPC}(f;v^{ref})+\ell_{freq}(f)\bigr)
\]

これは目標速度の決定ではなく、その速度を実現しやすいCadenceの選択である。旧「Foothold込みの \(J_{MPC}\) 最小」の理由は[E](appendices/E_Corrections_and_Clarifications.md) §24。

Sampling経路の公平比較条件は[18](18_Experiments_and_Research_Roadmap.md) §4。Cost一致は現行不可。[F](appendices/F_Open_Questions.md)。

## 7. 実現可能速度域

同じ周波数が対応できる速度には範囲がある。

\[
v
\in
\mathcal V
(f,d,L_{kin},\dot q_{max},\tau_{max},\mu,\text{gait})
\]

低速から高速まで同一Trot設定で対応するのではなく、周波数・Duty・Gaitを変更する必要がある。

## 8. 対応コード

- `config.py`: `gait_params`, `step_freq_available`, `optimize_step_freq=False`
- `helpers/foothold_reference_generator.py`: `stance_time/2 * ref_velocity`
- `interfaces/srbd_batched_controller_interface.py`（標準はオブジェクト未生成）
- Sampling の gait adaptive（標準`type='nominal'`では未到達）

## 9. Cursor確認課題

速度×周波数×DutyのGridで、必要Stance相対移動量、Swing平均速度、IK可到達性を計算し、Go2の実現可能Envelopeを可視化する。