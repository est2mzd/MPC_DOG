# 数式 → ブロック

## 1. 結論

理解すべき式は、リポジトリ横断で **少数の役割** に落ちる。同じ記号でも実装が違う場合は、ブロック ID と出典を両方書く。式の展開とコード変数は既存ノートが正本である。

## 2. 記述の区別

- **理論**: 実装を説明する式。出典ノートをリンクする。
- **実装事実**: その式がどの関数に対応するか。
- 両リポで式が違うときは並べて書く。混ぜない。

## 3. B03 ゲイト

理論（PyMPC）: [qpympc-study/04](../qpympc-study/04_Gait_Generator_and_Contact_Schedule.md)

\[
\phi_i \leftarrow (\phi_i + \Delta t\, f) \bmod 1,\qquad
c_i =
\begin{cases}
1 & \phi_i < d \\
0 & \phi_i \ge d
\end{cases}
\]

| 記号 | 意味 | コード（実装事実） |
|---|---|---|
| \(\phi_i\) | 脚位相 | `_phase_signal[i]` |
| \(f\) | step frequency | `step_freq`（trot 1.35 Hz） |
| \(d\) | duty | `duty_factor`（trot 0.74） |
| \(c_i\) | 接地 | `contact[i]` |
| \(c_{i,k}\) | ホライズン接地 | `contact_sequence[i,k]` |

LC の gait は OCS2 `ModeSchedule`（イベント時刻 + mode 列）。式は tree 内に無い。B03 の第一実装は上式にする（方針）。

## 4. B04 足場

理論: [qpympc-study/05](../qpympc-study/05_Foothold_Reference_and_Terrain_Adaptation.md)

大意（実装のヒューリスティック）:

\[
p_{td,i}^{ref} = p_{hip,i} + v\,\frac{T_{stance}}{2} + \text{誤差補償}
\]

正確な項（`vel_offset`, `error_compensation`, 旋回オフセット）は 05 章が正本。LC に同等の xy 足場式は無い。

## 5. B05 地形

4足の lift-off 位置から地形平面の roll/pitch/高さを出す幾何である。標準は `roll_activated=False` で roll=0、pitch と高さはローパスフィルタ（実装事実）。式の係数は `terrain_estimator.py`。

## 6. B06 状態推定（LC）

理論: [legged_control/04](../legged_control/04_State_Estimation.md)

並進だけ線形 KF。予測は IMU 加速度、観測は接地足の FK（滑らない仮定）。

\[
\hat x \in \mathbb{R}^{18}
=
[p_b^\mathsf T,\ v_b^\mathsf T,\ p_{f,1}^\mathsf T,\ldots,p_{f,4}^\mathsf T]^\mathsf T
\]

出力は rbd `(36,)`。姿勢は IMU、関節はエンコーダ直読（実装事実）。PyMPC 標準経路にこの式は無い。

## 7. B07 予測モデル

### 7.1 PyMPC SRBD（第一正本にする式）

理論: [qpympc-study/06](../qpympc-study/06_Centroidal_SRBD_Model.md)

状態（基本24）:

\[
x
=
[p_{CoM}^\mathsf T,\ v_{CoM}^\mathsf T,\ \Theta^\mathsf T,\ \omega^\mathsf T,\
p_{FL}^\mathsf T,\ p_{FR}^\mathsf T,\ p_{RL}^\mathsf T,\ p_{RR}^\mathsf T]^\mathsf T
\]

入力:

\[
u
=
[v_{f,FL}^\mathsf T,\ldots,v_{f,RR}^\mathsf T,\
F_{FL}^\mathsf T,\ldots,F_{RR}^\mathsf T]^\mathsf T
\]

並進（理論）:

\[
\dot p = v,\qquad
m\dot v = \sum_i c_i F_i + mg + F_{ext}
\]

回転は Base 慣性と接触モーメント。接触 \(c_i\) は最適化変数ではなくパラメータ \(p\)（実装事実）。

### 7.2 LC Full Centroidal（参考。tree 外）

理論: [legged_control/05](../legged_control/05_NMPC.md)

\[
x = [h_{com}^\mathsf T,\ q_b^\mathsf T,\ q_j^\mathsf T]^\mathsf T,\quad
u = [f_c^\mathsf T,\ v_j^\mathsf T]^\mathsf T
\]

\(h_{com}\) は正規化 centroidal 運動量。関節が運動量に残る。**SRBD ではない**（既定 `centroidalModelType=0`）。B07 の第一実装に使わない（方針）。

## 8. B08 NMPC

### 8.1 PyMPC

理論: [qpympc-study/07](../qpympc-study/07_MPC_Formulation.md)

\[
\min_{x,u}
\sum_{k=0}^{N-1}
\bigl(\|x_k-x_k^{ref}\|_Q^2 + \|u_k-u_k^{ref}\|_R^2\bigr)
+ \|x_N-x_N^{ref}\|_Q^2
\]

\[
x_{k+1}=f_{ERK}(x_k,u_k,p_k),\quad x_0=x_{meas}
\]

+ 摩擦錐。\(N=12\), \(dt=0.02\)。終端重みは \(Q\) と同じ（実装事実。別の \(Q_N\) は無い）。

先頭入力の後半が GRF。mask:

\[
F_i^{cmd} = c_{i,0}\, F_i^{MPC}
\]

[qpympc-study/09](../qpympc-study/09_MPC_Output_and_Receding_Horizon.md)。

### 8.2 LC

理論: [legged_control/05](../legged_control/05_NMPC.md)

連続時間 OCP + 多重射撃 SQP。ホライズン 1.0 s、`sqp.dt=0.015`、反復1。WBC へ渡すのは **1点**。トルクは決定変数でない。

自分の `src/` の B08 第一版は 8.1 を包む（方針）。8.2 は比較用の式として残す。

## 9. B09 下位制御

### 9.1 Jacobian 転置（PyMPC）

理論: [qpympc-study/10](../qpympc-study/10_Stance_and_Swing_Control.md)

\[
\tau_i^{stance} = -J_i^\mathsf T F_i^{cmd}
\]

QP ではない。接触力の再配分をしない（実装事実）。

### 9.2 Weighted QP（LC）

理論: [legged_control/06](../legged_control/06_WBC.md)

\[
x_{wbc}
=
[\ddot q^\mathsf T\in\mathbb{R}^{18},\
F_c^\mathsf T\in\mathbb{R}^{12},\
\tau^\mathsf T\in\mathbb{R}^{12}]^\mathsf T
\]

運動方程式（理論）:

\[
M\ddot q - J^\mathsf T F_c - S^\mathsf T \tau + nle = 0
\]

これに摩擦、立脚ゼロ加速度、遊脚 PD、ベース加速度、接触力追従がタスクとして乗る。使う出力は \(\tau\) だけ（実装事実）。

B09 は **同じ穴に2式** を差し込む。混ぜて1つの式にしない（方針）。

## 10. B10 遊脚

PyMPC: 足の Cartesian 軌道（スプライン）+ PD、必要ならフィードバック線形化。ゲインは `swing_position_gain_fb=500`, `swing_velocity_gain_fb=10`（実装事実）。

LC: NMPC の望む足位置へ WBC が加速度タスクを書く。別途 `SwingTrajectoryPlanner` が z 制約を NMPC 側に供給する。

役割は近いが、所属が違う（独立コントローラ vs QP タスク）。B09 の選択に合わせて片方を使う（方針）。

## 11. B02 参照

### PyMPC

速度指令を H→W、地形で回転し、`ref_state` にする。位置の xy 参照は 0（実装事実。[qpympc-study/03](../qpympc-study/03_User_Command_and_Reference_Generation.md)）。

### LC

現在観測と指令から **2点**。速度経路では両端の運動量先頭を \(v_W\) で上書きする（実装事実。[legged_control/03](../legged_control/03_User_Command_and_Reference.md)）。

第一版の B02 は PyMPC 形（毎周期の1参照）にする。2点軌道は B08 を LC 型にしたときだけ要る（方針）。

## 12. B11 関節

PyMPC:

\[
\tau^{limited} = \mathrm{clip}(\tau,\ 0.9\,\tau_{lim})
\]

LC:

\[
\tau_{motor} = \tau_{ff} + K_d(\dot q^*-\dot q),\quad K_p=0,\ K_d=3
\]

sim 第一版は前者。後者は実機寄りの B11 バックエンド（方針）。

## 13. B12 安全

LC: \(|\phi_{roll}| \le \pi/2\) でなければ停止。PyMPC に対応式は無い。

## 14. 学習順（方針）

式を自分のコードに落とす順。ノートの章と一致させる。

1. B03 位相（4行でテストできる）
2. B07 並進 \(\dot v\)（接触力の和）
3. B09 の \(\tau=-J^T F\)
4. B08 のコストと摩擦錐
5. B06 KF（実機を意識したとき）
6. B09 の QP（必要になってから）

## 15. 次

ディレクトリ案。[08](08_Src_Layout.md)。
