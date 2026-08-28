# User Command and Reference Generation

Q2のブロック①と②である。境界サイズの正本は[02](02_System_Architecture_and_Dataflow.md)である。

## 1. 結論

ユーザーは「今どこにいるべきか」ではなく、「どの速さで胴体を動かしたいか」、または「odom上の1点へ行きたいか」を与える。`TargetTrajectoriesPublisher` は最新のNMPC観測を基準に、現在と最大1秒先の **2点** のcentroidal状態を作り、OCS2 `RosReferenceManager` へ送る。Gaitはこのノードでは触らない。

## 2. ブロック① ユーザー指令

### 2.1 背景

四足のモデルベース歩行では、MPCへ生のジョイスティックを渡さず、まず胴体の参照軌道にする。MIT Cheetah系もANYmal/OCS2系もこの分離を使う。本実装はOCS2の `TargetTrajectories` インタフェースに合わせ、速度指令とゴール指令を同じ2点軌道へ正規化する。

### 2.2 目的

- 人間の「前進・横歩き・旋回」を、NMPCが追従できる \(\mathbf{x}^{\mathrm{ref}}(t)\) の種にする。
- 指令と現在推定を切り離す。指令ノードはモータを直接駆動しない。
- ゴールと速度を別callbackにし、両方とも同じ publisher に乗せる。

### 2.3 入力経路

実装事実として3経路ある。

| 経路 | トピック | 中間ベクトル | shape |
|---|---|---|---|
| 速度 | `/cmd_vel` | `cmdVel` | `(4,)` = \([v_x, v_y, v_z, \dot\psi]\) |
| ゴール | `/move_base_simple/goal` | `cmdGoal` | `(6,)` = \([p_x, p_y, p_z, \psi, \theta, \phi]\) |
| 観測 | `legged_robot_mpc_observation` | `latestObservation_` | `SystemObservation`: `state (24,)`, `input (24,)`, `time`, `mode` |

`latestObservation_.time == 0` のあいだは両方の指令を捨てる。コントローラが最初のobservationを出すまで軌道は更新されない。

ゲームパッドを使う場合、`joy.yaml` は

- 軸1 → `linear.x`、scale 1.0
- 軸2 → `linear.y`、scale 0.8
- 軸0 → `angular.z`、scale \(\pi\)
- deadman ボタン 4

である。`linear.z` はゲームパッドからは出ないが、`cmdVelCallback` は `msg->linear.z` を読む。

対応コード: `legged_controllers/include/legged_controllers/TargetTrajectoriesPublisher.h` のコンストラクタ。起動は `legged_controllers/src/TargetTrajectoriesPublisher.cpp` の `main()`。joyは `config/joy.yaml`。

### 2.4 例

前進 0.5 m/s だけ欲しいとき、

\[
\mathrm{cmdVel} = [0.5,\ 0,\ 0,\ 0]^\mathsf T
\]

shapeは `(4,)` である。元スケッチの3成分だけでは `linear.z` が落ちる。

## 3. ブロック② 目標運動軌道

### 3.1 背景

NMPCのコストは \(\|\mathbf{x}-\mathbf{x}^{\mathrm{ref}}\|_Q^2\) である。参照が無いと、solverはその場の姿勢維持と入力正則化しか持たない。OCS2は参照を時間関数 `TargetTrajectories`（時刻列 + 状態列 + 入力列）として持ち、区間内は線形補間する。

本ノードは未来の密な軌道計画をしない。現在姿勢から指令速度を一定時間積分した **終端ポーズ** を1点足すだけである。細かい足運びはNMPC制約とWBCに任せる。

### 3.2 目的

- 指令速度をWorldの並進速度と、1秒後の水平位置・yawへ変換する。
- 高さ・roll・pitchを設定値へ固定し、ユーザーが傾けないようにする。
- 関節参照を `defaultJointState` の立位に固定する。遊脚軌道はここには無い。

### 3.3 速度経路のロジック

`cmdVelToTargetTrajectories(cmdVel, observation)` の手順である。

1. 現在ポーズを観測から取る。
   \[
   p_{\mathrm{cur}} = x[6:12] \in \mathbb{R}^6
   \]
2. 指令並進3成分を現在ZYXでWorldへ回す。
   \[
   v_{\mathrm{W}} = R_{\mathrm{ZYX}}(\psi,\theta,\phi)\, [v_x, v_y, v_z]^\mathsf T
   \]
   yaw速度 \(\dot\psi_{\mathrm{cmd}}\) は回転しない。
3. 積分時間は `TIME_TO_TARGET = mpc.timeHorizon = 1.0` s。
   \[
   \begin{aligned}
   p_x^+ &= p_{x,\mathrm{cur}} + v_{\mathrm{W},x}\, T \\
   p_y^+ &= p_{y,\mathrm{cur}} + v_{\mathrm{W},y}\, T \\
   p_z^+ &= h_{\mathrm{com}} \\
   \psi^+ &= \psi_{\mathrm{cur}} + \dot\psi_{\mathrm{cmd}}\, T \\
   \theta^+ &= 0,\quad \phi^+ = 0
   \end{aligned}
   \]
   a1では \(h_{\mathrm{com}}=0.3\) m。`v_{\mathrm{W},z}` は位置 \(p_z\) には使わない。
4. `targetPoseToTargetTrajectories` が2点軌道を組む。
   - \(t_0 = t_{\mathrm{obs}}\)、\(t_1 = t_{\mathrm{obs}}+T\)
   - 始点ポーズは現在xy/yaw、高さは \(h_{\mathrm{com}}\)、pitch/rollは0
   - 関節は両方とも `DEFAULT_JOINT_STATE` `(12,)`
   - 運動量6はいったんゼロ
   - 入力列は `(24,)×2` のゼロ。コスト側が重力補償入力に置き換える（[05](05_NMPC.md)）
5. 速度経路だけ、両端の状態先頭3を上書きする。
   \[
   x_0[0:3] = x_1[0:3] = v_{\mathrm{W}}
   \]
   これでNMPCは「今も1秒後も同じWorld並進速度」を追う。角運動量参照は0のままである。

対応コード: `TargetTrajectoriesPublisher.cpp` の `cmdVelToTargetTrajectories()`, `targetPoseToTargetTrajectories()`。

### 3.4 ゴール経路のロジック

`goalCallback` は Pose を `odom` へ TF し、quaternionを `eulerAngles(0,1,2)` で ZYX に分解する。`goalToTargetTrajectories` は

- 目標xyとyawをゴールから使う
- zは `COM_HEIGHT`、pitch/rollは0
- 到達時刻は並進距離 / `targetDisplacementVelocity` と \(|\Delta\psi|\) / `targetRotationVelocity` の大きい方
- a1では 0.5 m/s と 1.57 rad/s
- 状態先頭6（運動量）はゼロのまま。ゴールは位置追従である

対応コード: `TargetTrajectoriesPublisher.h` の `goalCallback`、`TargetTrajectoriesPublisher.cpp` の `goalToTargetTrajectories()`, `estimateTimeToTarget()`。上限速度は `reference.info`。

### 3.5 数式（軌道としてNMPCが見るもの）

\(t\in[t_0,t_1]\) でOCS2が線形補間する。

\[
\mathbf{x}^{\mathrm{ref}}(t)
=
\frac{t_1-t}{t_1-t_0}\mathbf{x}_0
+
\frac{t-t_0}{t_1-t_0}\mathbf{x}_1
\]

速度指令時、\(\mathbf{x}_0,\mathbf{x}_1\) の先頭3は同じ \(v_{\mathrm{W}}\) なので、参照CoM速度は区間内で一定である。位置参照だけが直線に進む。

### 3.6 例

現在 \(p=(0,0,0.3)\)、yaw 0、指令 \(v_x=0.5\) m/s。

| 点 | t [s] | \(v_{\mathrm{com}}\) [m/s] | 位置 [m] | 姿勢 [rad] | 関節 |
|---|---|---|---|---|---|
| 0 | \(t_{\mathrm{obs}}\) | (0.5, 0, 0) | (0, 0, 0.3) | (0,0,0) | default 12 |
| 1 | \(t_{\mathrm{obs}}+1\) | (0.5, 0, 0) | (0.5, 0, 0.3) | (0,0,0) | default 12 |

流れるデータ全体は、時刻2 + 状態 \(24\times2\) + 入力 \(24\times2\) = 98 scalar（時刻を除けば96）である。

### 3.7 Gaitはこのブロックに含まれない

`load_controller.launch` は別に `legged_robot_gait_command` を起動する。stance / trot / flying_trot などは `gait.info` の `modeSequence` と `switchingTimes` である。NMPCの接地制約はこの列を読む。詳細は[05](05_NMPC.md) §6。

## 4. 実装事実と理論の境界

- **実装事実**: 2点軌道、高さ固定、関節参照固定、速度はWorld回転後に積分。
- **理論**: 一定速度のホロノミック積分。スリップや踏み込み遅れは無視する。
- **推奨改善**: 指令とGaitを1ノードにまとめる案はREADME自身が書いている。現行は分離。
- **未実装**: 地形に合わせた \(h_{\mathrm{com}}\) 可変、速度の自動クリップ、局所障害物回避。
