# Gait–MPC Coupling

## 1. 結論

「Gait scheduleをMPCへ渡す」とは、各予測段でどの脚の力を利用でき、どの足を動かせるかを示す既知パラメータ\(c_{i,k}\)をMPCの運動方程式へ入れることである。Trot位相は最適化変数ではないため、MPCが逆相を選ぶことはない。

## 2. 固定Schedule OCP

\[
\min_{x,u}J(x,u)
\]

subject to

\[
x_{k+1}=f(x_k,u_k;c_k^{Trot})
\]

セミコロン右の\(c_k^{Trot}\)はGait Generatorが事前に決める。

対応コード: `periodic_gait_generator.py` の `compute_contact_sequence()` と `centroidal_nmpc_nominal.py` の `param` 設定。接地列の正本は[04](04_Gait_Generator_and_Contact_Schedule.md)。

## 3. 並進・回転への接触Gate

\[
m\dot v=mg+\sum_i c_iF_i+F_{ext}
\]

\(F_{ext}\) は標準0。回転の完全形（\(R_{BW}\)、\(\tau_{ext}\)）は[06](06_Centroidal_SRBD_Model.md) §6。ここでは接触Gateだけを残す。\(g\) は加速度ベクトル \([0,0,-g]\) であり、06の \(\dot v=(1/m)(\sum c_i F_i+F_{ext})+g\) と同じ。

\[
I\dot\omega
=
R_{BW}
\left(
\sum_i c_i(p_i-p_{CoM})\times F_i
\right)
-\omega\times I\omega
\]

位相Aが`[1,0,0,1]`なら、FLとRRだけが胴体予測を支持できる。FR/RLの力は \(\dot v,\dot\omega\) に入らない。

これは**力学だけのGate**である。同じ遊脚GRFには、摩擦錐と入力コストが残る。3段の正本は[09](09_MPC_Output_and_Receding_Horizon.md) §6。SRBD式の正本は[06](06_Centroidal_SRBD_Model.md)。

対応コード: `centroidal_model_nominal.py` の接触Gate。

## 4. 足位置への接触Gate

完全な足Gateは[06](06_Centroidal_SRBD_Model.md) §7 の \(\dot p_i=(1-c_i)(1-s_i)v_{foot,i}\) である。標準では \(s_i\equiv0\)。

- 立脚\(c_i=1\)：足位置固定。
- 遊脚\(c_i=0\)：足位置を最適化可能（`use_foothold_optimization=True`）。

よって同一段で、ある脚を「力学にGRFを出す立脚」と「自由に移動する遊脚」の両方にはできない。

対応コード: `centroidal_model_nominal.py` の足速度Gate。

## 5. acados実装

各段`j`に、

```python
param = np.array([
    FL_contact_sequence[j],
    FR_contact_sequence[j],
    RL_contact_sequence[j],
    RR_contact_sequence[j],
    mu,
    ...
])
solver.set(j, "p", param)
```

と設定する。`p`はSolverが変更する`u`ではない。

| 入力 | shape | 単位 | frame | 出力 |
|---|---|---|---|---|
| `contact_sequence` | `(4,12)` | 0/1 | なし | 各段`p`の先頭4要素 |
| `mu` | scalar | なし | なし | `p`の摩擦係数 |

対応コード: `Acados_NMPC_Nominal.compute_control()`。

## 6. 遊脚GRF

出力Mask \(F_i^{\mathrm{command}}=c_{i,0}F_i^{\mathrm{MPC}}\) を含む3段の正本は[09](09_MPC_Output_and_Receding_Horizon.md) §6である。OCP内部の等式ゼロは**無い**。

## 7. より明示的な推奨制約

次は **推奨改善** であり、現行標準OCPには無い。

\[
0\le F_{z,i,k}\le c_{i,k}F_{z,max}
\]

これと摩擦錐を組み合わせれば、\(c=0\)で\(F_x=F_y=F_z=0\)をOCP内でも保証できる。

## 8. Schedule不要となる別方式

| 方式 | 接触の扱い | 特徴 |
|---|---|---|
| 固定Schedule | 外部の\(c_{i,k}\) | 高速、標準実装 |
| Mixed-integer | \(c_{i,k}\in\{0,1\}\)も最適化 | 高計算負荷 |
| Contact-implicit | 距離と力の相補性 | 非線形で難しい |

Contact-implicitでは、

\[
F_z\ge0,\quad\phi\ge0,\quad F_z\phi=0
\]

により接触を決める。その場合のみ、固定Trot scheduleを外せる。これは現行コードに無い別方式である。

## 9. 低レベルとの共有

同じ`current_contact`が、

- GRF出力Mask
- `-J.T @ F`を使う立脚脚
- Cartesian swing controlを使う遊脚脚

を決める。MPCと低レベルが別位相を使わないことが重要である。

## 10. 対応コード

- `helpers/periodic_gait_generator.py`
- `interfaces/wb_interface.py`
- `interfaces/srbd_controller_interface.py`
- `controllers/gradient/nominal/centroidal_model_nominal.py`
- `controllers/gradient/nominal/centroidal_nmpc_nominal.py`

## 11. Cursor確認課題

§7の推奨制約を入れる案は、Baseline固定のあと[18](18_Experiments_and_Research_Roadmap.md)の研究候補として扱う。