# Log 10: nominal gradient MPC の OCP 再構成

対応プロンプト: Cost / Dynamics / Constraints / Slack / Solver / yref / warm start / failure。本文未修正。

標準フラグ: `use_RTI=False`, `use_DDP=False`, `use_warm_start=False`, `use_integrators=False`, `use_foothold_constraints=False`, `use_static_stability=False`, `use_zmp_stability=False`, `use_foothold_optimization=True`, `num_qp_iterations=1`, `solver_mode='balance'`, `N=12`, `dt=0.02`。

\[
\min_{x,u,s} J=\sum_{k=0}^{N-1}\bigl(\|x_k-x_k^{ref}\|_Q^2+\|u_k-u_k^{ref}\|_R^2+\ell_{slack,k}\bigr)+\|x_N-x_N^{ref}\|_Q^2
\]

subject to

\[
x_{k+1}=f_{\mathrm{ERK}}(x_k,u_k,p_k),\quad x_0=x_{\mathrm{meas}}
\]

\[
h_{\mathrm{fric}}(u_k,p_k)\in[l_h,u_h]
\]

標準では foothold/stability slack 制約は構築されない。\(s\) は標準OCPに無い。

## 1. Cost

`LINEAR_LS`。`W=blkdiag(Q,R)`, `W_e=Q`（終端は同じQ。別 \(Q_N\) はない）。
`y=[x;u]`, `y_e=x`。設定関数: `set_weight()` と `create_ocp_solver_description()`。参照設定: `compute_control` の `solver.set(j,"yref",yref)`。

| Cost項 | 状態/入力Index | コード変数 | Weight | Reference | 設定関数 |
|---|---|---|---|---|---|
| Stage + Terminal 位置 | x 0:3 | `Q_position` | `[0,0,1500]` | `ref_position` | `set_weight` / yref |
| Stage + Terminal 速度 | x 3:6 | `Q_velocity` | `[200,200,200]` | `ref_linear_velocity` | 同上 |
| Stage + Terminal 姿勢 | x 6:9 | `Q_base_angle` | `[500,500,0]` | `ref_orientation` | 同上 |
| Stage + Terminal 角速度 | x 9:12 | `Q_base_angle_rates` | `[20,20,50]` | `ref_angular_velocity` | 同上 |
| Stage + Terminal 足位置×4 | x 12:24 | `Q_foot_pos` | `[300,300,300]`×4 | `ref_foot_*` | 同上 |
| Stage + Terminal z積分 | x 24 | `Q_com_position_z_integral` | 50 | 0 | 同上 |
| Stage + Terminal vx積分 | x 25 | `Q_com_velocity_x_integral` | 10 | 0 | 同上 |
| Stage + Terminal vy積分 | x 26 | 同上 y | 10 | 0 | 同上 |
| Stage + Terminal vz積分 | x 27 | 同上 z | 10 | 0 | 同上 |
| Stage + Terminal roll積分 | x 28 | `Q_roll_integral_integral` | 10 | 0 | 同上 |
| Stage + Terminal pitch積分 | x 29 | `Q_pitch_integral_integral` | 10 | 0 | 同上 |
| Stage 足速度×4 | u 0:12 | `R_foot_vel` | `[1e-4,1e-4,1e-5]`×4 | 0（未代入） | 同上 |
| Stage GRF×4 | u 12:24 | `R_foot_force` | `[0.001]*3`×4 | Fx=Fy=0, Fz=`mg/n_s * c_i` | yref[44,47,50,53] |
| Slack linear | — | `zl`,`zu` | 1000 | — | foothold/stability構築時のみ |
| Slack quadratic | — | `Zl`,`Zu` | 1 | — | 同上 |

区別:

- Stage cost: `W` on `k=0..N-1`
- Terminal cost: `W_e=Q`（入力項なし）
- State tracking: 上表 x
- Input tracking: 足速度（参照0）とGRF
- Foot position: x 12:24
- Foot velocity: u 0:12
- GRF: u 12:24
- GRF rate: **未実装**（`type='input_rates'` の別コントローラ）
- Integral state: x 24:30。標準は補償オフだがコストは在る
- Slack penalty: 標準経路では制約がfrictionのみのため slack 変数は作られない（`num_state_cstr=0`）

## 2. Hard constraint

摩擦は `create_friction_cone_constraints`。各脚5式×4=20。`j=0` も含め全入力段。接触で無効化しない。

脚あたり（Focchi線形錐）:

| 制約 | 数式（概略） | 対象 | 下限 | 上限 | 接触との関係 |
|---|---|---|---|---|---|
| 摩擦1 | \((-\mu n+t)^\top F\) | 各脚GRF | `-1000` | 0 | なし（常時） |
| 摩擦2 | \((-\mu n+b)^\top F\) | 同上 | `-1000` | 0 | なし |
| 摩擦3 | \((\mu n+b)^\top F\) | 同上 | 0 | `1000` | なし |
| 摩擦4 | \((\mu n+t)^\top F\) | 同上 | 0 | `1000` | なし |
| 法線 | \(F_z\) | 同上 | `grf_min=0` | `grf_max=m g` | **cでゼロ化しない** |
| 初期状態 | \(x_0=x_{meas}\) | 全状態 | eq | eq | 遊脚足は参照へteleport |
| 動力学 | ERK | x,u,p | — | — | pのcでGate |

`x`/`u` の box bound は設定しない。

## 3. Soft constraint

標準: なし。

`use_foothold_constraints` または stability が True のときだけ、frictionの後の `h` に slack（`zl=zu=1000`, `Zl=Zu=1`）。

| 制約 | Slack | Linear | Quadratic | 違反時 |
|---|---|---|---|---|
| （標準）なし | — | — | — | — |
| （非標準）Foothold箱 | あり | 1000 | 1 | 足が領域外 |
| （非標準）支持多角形 | あり | 1000 | 1 | COM/ZMPが外 |

## 4. Costだけで誘導される条件

- 遊脚 \(F_z^{ref}=0\)。等式 \(F=0\) ではない
- 遊脚 \(F_x,F_y\) 参照0
- 足速度参照0（小さいR）
- 水平位置 x,y の状態重み0 → 位置は速度経由でのみ誘導
- Yaw状態重み0
- 積分状態を0へ

## 5. 出力後処理（OCP外）

- 遊脚初期足のteleport（solve前）
- `perform_scaling` / decenter
- `optimal_GRF=u0[12:]`
- 次TD foothold抽出 + H frameで `±0.15` clip（constraintなし時）
- `SRBDControllerInterface` の `F *= current_contact`
- status 1 or 4: `optimal_GRF=previous_optimal_GRF` のあと `reset()`。その直前の `mg/n_s` 代入は直後に上書きされ死文
- 非更新周期（500 Hzのうち4/5）: wrapperが前回GRF/footholdを保持
- `simulation.py` が `tau` を `0.9*ctrlrange` clip

## 6. Solver

| 項目 | 標準値 |
|---|---|
| Solver type | acados OCP |
| NLP | SQP |
| QP | PARTIAL_CONDENSING_HPIPM, `BALANCE` |
| Integrator | ERK |
| Horizon | 12 |
| dt | 0.02 s（`tf=0.24`） |
| NLP iter | `num_qp_iterations=1` |
| Hessian | GAUSS_NEWTON |
| LM | 1e-3 |
| Warm start | `use_warm_start=False`。acadosは前回内部解を保持し得る（明示shiftなし） |
| Status | `status==1 or 4` を失敗扱い |
| Failure | 前回GRF + solver reset。遊脚footholdは参照 |

## 特に確認した7点

1. **遊脚GRFはOCP内で厳密ゼロか?** いいえ。動力学Gateと参照0と出力Maskのみ。摩擦は遊脚にも `Fz∈[0,mg]` を課す。
2. **法線上限はcontactでゼロ化か?** いいえ。常に `mg`。
3. **摩擦錐は全脚常時か?** はい。
4. **Foothold constraintは標準で有効か?** いいえ。`False`。
5. **Stability/ZMP/Lyapunovは標準で有効か?** いいえ。Lyapunovは別 `type`。
6. **GRF rate weightは実装されているか?** nominalには無い。
7. **文書の既定重みは一致するか?**

| 資料 | 記載 | コード | 判定 |
|---|---|---|---|
| `07` 構造（LS + ERK + 摩擦） | 概要 | 一致 | 正しいがIndex不足 |
| `07` 既定重み | 「set_weightに直接」 | 上表 | 不完全（数値未掲） |
| `07` 摩擦式 | \(\|F_x\|\le\mu F_z\) | 線形4辺+Fz | 不完全（線形化の符号付き4不等式） |
| `07` Slack一般論 | 導入できる | 標準では未構築 | 不完全 |
| `07` Failure | 前回GRFまたは基準鉛直 | 前回GRFが勝ち、mg/n_sは死文 | 誤り（「または」は実装と違う） |
| C Velocity weight | `[200,200,200]` | 一致 | 正しい |
| C Height | z=1500 | 一致 | 正しい |
| C Base angle | `[500,500,0]` | 一致 | 正しい |
| C Angular rate | `[20,20,50]` | 一致 | 正しい |
| C Foot pos | `[300,300,300]` | 一致 | 正しい |
| C Foot vel | "small" | `[1e-4,1e-4,1e-5]` | 不完全 |
| C GRF | 0.001 | 一致 | 正しい |
| C Reflex | "tracking" | `False` | 誤り |
| C Foothold opt/const | True / False | 一致 | 正しい |
