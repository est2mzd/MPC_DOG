# MPC Formulation

## 1. 結論

MPCは、既知の現在状態・参照状態・接地列のもとで、将来の胴体運動、足位置、GRFを最適化する。標準設定は12段、20 ms刻み、0.24 s先まで予測する。

## 2. 最適化問題

実装は acados `LINEAR_LS` である。`W=\mathrm{blkdiag}(Q,R)`、終端は `W_e=Q`。**別の \(Q_N\) はない**。旧 \(Q_N\) 表記の理由は[E](appendices/E_Corrections_and_Clarifications.md) §20。

\[
\min_{x,u}
\sum_{k=0}^{N-1}
\left(
\|x_k-x_k^{ref}\|_Q^2+
\|u_k-u_k^{ref}\|_R^2
\right)
+\|x_N-x_N^{ref}\|_Q^2
\]

subject to

\[
x_{k+1}=f_{\mathrm{ERK}}(x_k,u_k,p_k),\qquad x_0=x_{\mathrm{meas}}
\]

および摩擦錐（標準で常時）。Foothold/安定制約は標準OFF。

| 数式 | コード |
|---|---|
| \(Q,R\) | `set_weight()` の対角。`ocp.cost.W` |
| 終端 \(Q\) | `ocp.cost.W_e = Q` |
| \(y=[x;u]\), \(y_e=x\) | `LINEAR_LS` |
| \(f\) | `centroidal_model_nominal.forward_dynamics` + ERK |

## 3. 参照

`ref_state`から次を設定する。

- CoM/Base高さ
- 線速度
- Roll/PitchとYaw参照
- 角速度
- 各脚Foothold

基準鉛直GRFは、立脚数\(n_s\)に対して、

\[
F_{z,i}^{ref}
=
\begin{cases}
mg/n_s & c_i=1\\
0 & c_i=0
\end{cases}
\]

とする。これは固定解ではなく、入力コストの参照である。

## 4. 既定重み

数値の正本は `Acados_NMPC_Nominal.set_weight()` である。`config.py` には無い。索引は[C](appendices/C_Parameter_Index.md)。症状からの使い方は[14](14_MPC_and_Controller_Tuning.md)。

| Cost項 | Index | コード変数 | 既定対角 | 単位 | Frame | 参照 |
|---|---|---|---|---|---|---|
| 位置 | x 0:3 | `Q_position` | `[0,0,1500]` | コスト対角 | W | `ref_position` |
| 速度 | x 3:6 | `Q_velocity` | `[200,200,200]` | コスト対角 | W | `ref_linear_velocity` |
| 姿勢 | x 6:9 | `Q_base_angle` | `[500,500,0]` | コスト対角 | SciPy xyz | `ref_orientation` |
| 角速度 | x 9:12 | `Q_base_angle_rates` | `[20,20,50]` | コスト対角 | Base | `ref_angular_velocity` |
| 足位置×4 | x 12:24 | `Q_foot_pos` | `[300,300,300]`×4 | コスト対角 | W | `ref_foot_*` |
| 積分6 | x 24:30 | `Q_*_integral` | 50, 10,10,10, 10,10 | コスト対角 | — | 0 |
| 足速度×4 | u 0:12 | `R_foot_vel` | `[1e-4,1e-4,1e-5]`×4 | コスト対角 | W | 0 |
| GRF×4 | u 12:24 | `R_foot_force` | `[0.001]*3`×4 | コスト対角 | W | Fx=Fy=0, Fz=`mg/n_s * c_i` |

`use_integrators=False` でも積分コストは残る。GRF rate 重み \(R_{\dot F}\) は **nominal未実装**（`type='input_rates'` 専用）。[E](appendices/E_Corrections_and_Clarifications.md) §22。

重要な効果：

- 速度重み増加：速度追従は強いがGRF・姿勢変動が増え得る。
- Roll/Pitch重み増加：胴体を水平に保つが力配分が攻撃的になり得る。
- GRF重み増加：力を抑えるが追従性が低下する。
- Foot position重み増加：Foothold参照に忠実だが地形・姿勢最適化の自由度が減る。
- Foot velocity重み増加：足運びは滑らかだが遠い着地点へ届きにくい。

## 5. 摩擦錐

Focchi線形錐である。各脚5式×4=20。`j=0` を含む全入力段。**接触状態で無効化しない**。遊脚にも \(F_z\in[0,mg]\) が残る。[09](09_MPC_Output_and_Receding_Horizon.md) §6。

脚あたり（\(n\) 法線、\(t,b\) 接線）:

\[
(-\mu n+t)^\top F \le 0,\quad
(-\mu n+b)^\top F \le 0,\quad
(\mu n+b)^\top F \ge 0,\quad
(\mu n+t)^\top F \ge 0
\]

\[
0\le F_z\le mg
\]

| 数式 | コード |
|---|---|
| \(\mu\) | `mpc_params['mu']=0.42`（`p`） |
| 上下限 | `constr_uh_friction` / `lh`。箱外は ±1000 |
| \(F_{z,\max}\) | `grf_max = m g`。\(c_i\) でゼロ化しない |

Plant側は楕円錐・実行時摩擦上書きであり、この \(\mu\) とは別。[01](01_MuJoCo_Go2_Plant_Model.md)。

## 6. Foothold・安定性制約

次は **標準OFF** である。有効条件を満たすときだけ構築する。

| 制約 | フラグ | 標準 |
|---|---|---|
| Foothold箱 + slack | `use_foothold_constraints` | False |
| Static stability | `use_static_stability` | False |
| ZMP | `use_zmp_stability` | False |
| Lyapunov | `type='lyapunov'` | 別コントローラ |

制約を厳しくすると安全余裕は増すが、OCPが実行不能になりやすい。

## 7. Soft constraint

標準経路では foothold/stability を作らないため **slack変数は無い**。有効時のみ `zl=zu=1000`, `Zl=Zu=1`。

\[
g(x,u)\le s,\qquad s\ge0,\qquad \rho_1 s+\rho_2 s^2
\]

は optional 経路の一般形である。Penaltyが大きすぎるとHardに近づく。

## 8. Solver

- CasADi：記号モデルと微分
- acados：OCP構築・NLP solve
- HPIPM：内部QP
- SQP/SQP-RTI/DDP：Solver方式

`num_qp_iterations=1`は高速だが、難しい非線形問題では収束余裕が小さい。

## 9. Failure/Fallback

`status in {1,4}` のとき、実装は `optimal_GRF = previous_optimal_GRF` のあと solver `reset()` する。その直前の `mg/n_s` 代入は直後に上書きされ **死文**。旧「または基準鉛直」の理由は[E](appendices/E_Corrections_and_Clarifications.md) §21。

実機での安全性は[F](appendices/F_Open_Questions.md)。

`use_warm_start=False`。明示的な shift 関数は無い。acadosが内部解を保持し得る。

## 10. 対応コード

- `controllers/gradient/nominal/centroidal_nmpc_nominal.py`
  - `create_ocp_solver_description()`
  - `set_weight()`
  - `create_friction_cone_constraints()`
  - `set_stage_constraint()`
  - `compute_control()`
- `config.py`: `mpc_params`

## 11. Cursor確認課題

§4の重みと `set_weight()` の差分を、コード更新時に再照合する。SamplingとのCost差は[F](appendices/F_Open_Questions.md)。