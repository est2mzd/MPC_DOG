# Centroidal / Single Rigid Body Model

## 1. 結論

標準`nominal` MPCは12関節を予測せず、ロボットを質量・慣性を持つ胴体と4つの足作用点へ簡略化する。MuJoCo全身モデルはPlant、Centroidal/SRBDはMPC内部予測モデルである。

## 2. 状態

基本状態は24次元である。

\[
x=
\begin{bmatrix}
p_{CoM}\\
v_{CoM}\\
\Theta\\
\omega\\
p_{FL}\\
p_{FR}\\
p_{RL}\\
p_{RR}
\end{bmatrix}
\in\mathbb R^{24}
\]

実装はさらに高さ、速度、Roll/Pitchに関する6積分状態を持ち、acados上の状態次元は30となる。デフォルト`use_integrators=False`では積分補償は無効である。コード記号の`omega_*_integral`は`self.states`に接続されていない。

Indexの正本は[A](appendices/A_Variable_Dictionary.md)。概要:

| Index | コード変数 | Shape | 単位 | Frame |
|---|---|---|---|---|
| 0:3 | `com_position_*` | `(3,)` | m | W（`perform_scaling`後は原点相対） |
| 3:6 | `com_velocity_*` | `(3,)` | m/s | W |
| 6:9 | `roll,pitch,yaw` | `(3,)` | rad | SciPy xyz |
| 9:12 | `omega_*` | `(3,)` | rad/s | Base |
| 12:24 | `foot_position_*` ×4 | 各`(3,)` | m | W（原点相対） |
| 24:30 | 積分6 | `(6,)` | 混在 | — |

## 3. 入力

\[
u=
\begin{bmatrix}
v_{foot,FL}\\
v_{foot,FR}\\
v_{foot,RL}\\
v_{foot,RR}\\
F_{FL}\\
F_{FR}\\
F_{RL}\\
F_{RR}
\end{bmatrix}
\in\mathbb R^{24}
\]

前半12成分は足先速度、後半12成分は各脚3軸GRFである。

## 4. 固定パラメータ

各予測段で次を受け取る。

- 4脚の接触状態
- 摩擦係数
- Stance proximity
- Base位置・Yaw
- 外力・外モーメント
- 慣性
- 質量

これらは最適化変数ではなくacados parameter `p`である。

## 5. 並進運動

\[
\dot p=v
\]

\[
\dot v
=
\frac{1}{m}
\left(
\sum_i c_iF_i+F_{ext}
\right)+g
\]

接触状態\(c_i\)により、予定立脚脚の力だけが**胴体予測**へ寄与する。摩擦制約と入力コストは遊脚GRFにも残る。3段の正本は[09](09_MPC_Output_and_Receding_Horizon.md) §6。

| 数式 | コード変数 |
|---|---|
| \(c_i\) | `stanceFL` 等（`p[0:4]`） |
| \(F_i\) | `foot_force_*` |
| \(F_{ext}\) | `p[13:16]`。標準は0 |
| \(g\) | `[0,0,-config.gravity_constant]` |
| \(m\) | `p[28]`（`config.mass=15.019`） |

## 6. 回転運動

実装はWorldモーメントをBaseへ回し、Euler角速度写像を使う。

\[
\dot\Theta=E(r,p)^{-1}\omega
\]

\[
I\dot\omega
=
R_{BW}
\left(
\sum_i c_i(p_i-p_{CoM})\times F_i
+\tau_{ext}
\right)
-\omega\times I\omega
\]

| 数式 | コード変数 |
|---|---|
| \(R_{BW}\) | `b_R_w = Rx@Ry@Rz` |
| \(\omega\) | `states[9:12]`（Base） |
| \(\tau_{ext}\) | `p[16:19]`。標準は0 |
| \(E^{-1}\) | `inv(conj_euler_rates)` |

Euler角特異点や高角度運動には注意が必要である。妥当性は[F](appendices/F_Open_Questions.md)。

## 7. 足位置運動

\[
\dot p_i=(1-c_i)(1-s_i)v_{foot,i}
\]

- \(c_i=1\)：立脚なので足位置固定。
- \(c_i=0\)：遊脚なので足先速度で移動可能。
- \(s_i\)：Touchdown近傍でFoothold変更を抑えるStance proximity。標準コードは `1*0` のため **常に0**。
- `use_foothold_optimization=False` なら遊脚でも足速度入力を0にする。標準はTrue。

| 数式 | コード変数 |
|---|---|
| \(v_{foot,i}\) | `foot_velocity_*` |
| \(s_i\) | `stance_proximity_*`（`p[5:9]`、標準0） |

## 8. 省略しているもの

- 関節角・関節速度
- 関節トルク制限の姿勢依存性
- Link個別慣性
- Swing legの反作用
- モータ・減速機ダイナミクス
- 接触コンプライアンス
- センサ・通信遅延

これらの誤差の補償は、積分（標準OFF）、外力（標準は `zeros(6,)`）、Residual学習（未実装）、低レベルPD（sim無効）の役割になり得る。標準経路では補償しない。

## 9. 対応コード

- `controllers/gradient/nominal/centroidal_model_nominal.py`
  - `Centroidal_Model_Nominal.__init__()`
  - `forward_dynamics()`
  - `export_robot_model()`

## 10. Cursor確認課題

`states`、`states_dot`、`inputs`の各Indexを自動抽出し、[Variable Dictionary](appendices/A_Variable_Dictionary.md)と一致するかテストする。