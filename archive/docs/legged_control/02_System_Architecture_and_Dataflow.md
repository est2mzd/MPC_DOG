# System Architecture and Dataflow

## 1. 結論

標準経路は、ユーザーが胴体速度（または2Dゴール）を指令し、2点のcentroidal目標軌道を作り、線形Kalmanが現在剛体状態を推定し、OCS2 SQP-NMPCが未来の運動量・姿勢・関節・GRFを最適化し、`WeightedWbc` が現在瞬間の \(\ddot q, F_c, \tau\) を解き、12関節へハイブリッド指令を出す閉ループである。

型・単位・frame・**配列サイズ**付きの境界契約は本章を正本とする。各ブロックの背景と式は[03](03_User_Command_and_Reference.md)以降である。変数一覧は[Appendix A](appendices/A_Variable_Dictionary.md)である。

質問Q1への回答も本章である。元スケッチとの差分は§6。

## 2. 全体フロー

矢印横の注記は「変数名 / shape / 単位」である。Gaitは胴体指令と並列である。

指令経路（イベント）:

```
[ゲームパッド / キーボード / RViz]              [端末 gait 名]
          |         |                                  |
          | Twist   | PoseStamped                      | ModeSequenceTemplate
          | linear3 | 位置3 + 姿勢->ZYX 3              | 例 trot: 2 mode / 0.6 s
          | + wz    | = 6 scalar                       |
          | = 4     |                                  |
          v         v                                  v
   ① /cmd_vel   ①' /move_base_simple/goal     GaitReceiver / GaitSchedule
          |         |                                  |
          v         v                                  |
  ② cmdVelToTarget     ② goalToTarget                 |
     Trajectories          Trajectories                |
          |         |                                  |
          |         |  観測: SystemObservation         |
          |         |  state 24, input 24              |
          |         |  <- legged_robot_mpc_observation |
          |         |                                  |
          +----+----+                                  |
               |  TargetTrajectories                   |
               |  t: 2, x: (24,)x2, u: (24,)x2         | ModeSchedule
               |                                       | eventTimes + modeSequence
               +------------------+--------------------+
                                  v
                    ④ SqpMpc + MPC_MRT  100 Hz スレッド
```

制御閉ループ（500 Hz。NMPCは100 Hzのpolicyを読む）:

```
⑦ UnitreeHW / LeggedHWSim
  関節 q,dq (12,)+(12,)
  IMU quat (4,), w (3,), a (3,)
  接地 bool (4,)
          |
          v
③ KalmanFilterEstimate
  rbdState (36,)
          |
          v
CentroidalModelRbdConversions
  observation.state (24,), mode
          |                         \
          |                          \----> ⑤ の計測入力
          v
④ evaluatePolicy の1点
  x* (24,), u* (24,), plannedMode
          |
          v
⑤ WeightedWbc  500 Hz
  x_wbc (42,)  使うのは tail 12 = tau
          |
          v
⑥ HybridJointHandle.setCommand
  関節あたり 5 scalar
  q*, dq*, Kp=0, Kd=3, ff=tau
  x12 = 60 scalar
          |
          v
⑦ へ戻る --> ロボット 12 DoF --> ⑦ の read
④ は observation (state 24, input 24) を ② へも publish する
```

次段入力は上段出力と同じ変数である。

## 3. 境界ごとのデータ契約

| 上流 | 出力 | shape / 単位 / frame | 更新周期 | 下流入力 |
|---|---|---|---|---|
| joy / キーボード / 手動 publish | `/cmd_vel` | `Twist`: `linear`(3,) m/s（指令frame。実装はベースyawでWへ回転）、`angular.z` rad/s | イベント。joyは約10–50 Hz | `cmdVelCallback` |
| RViz 2D Nav Goal | `/move_base_simple/goal` | `PoseStamped`。内部で `(6,)`: xyz m + ZYX rad、odom | イベント | `goalCallback` |
| `legged_robot_gait_command` | gait template | mode列 + `switchingTimes`。trotは2 mode、周期 0.6 s | イベント | `GaitReceiver` → `GaitSchedule` |
| `cmdVelToTargetTrajectories` | `TargetTrajectories` | 時刻 2、状態 `(24,)×2`、入力 `(24,)×2`（入力はゼロ。状態先頭3は \(v_{\mathrm{W}}^{\mathrm{cmd}}\)） | `/cmd_vel` ごと | `RosReferenceManager` |
| `goalToTargetTrajectories` | 同上 | 状態先頭6（運動量）はゼロ。到達時刻は変位/上限速度 | goalごと | 同上 |
| `UnitreeHW::read` / `LeggedHWSim::readSim` | 関節、IMU、接地 | q `(12,)` rad、dq `(12,)` rad/s、quat `(4,)`、ω `(3,)` rad/s B、a `(3,)` m/s² B、contact `(4,)` bool | 500 Hz | `updateStateEstimation` |
| `KalmanFilterEstimate::update` | `rbdState_` | `(36,)`。配置は[04](04_State_Estimation.md) | 500 Hz | `computeCentroidalStateFromRbdModel` |
| 同上内部KF | `xHat_` | `(18,)`: 胴体位置3 + 速度3 + 足位置12、m と m/s、W | 500 Hz | `updateLinear` のみ使用 |
| `CentroidalModelRbdConversions` | `currentObservation_.state` | `(24,)` mixed。配置は§4 | 500 Hz | NMPC観測、目標軌道生成 |
| `evaluatePolicy` | `optimizedState` | `(24,)` 同配置 | 500 Hzで読む。中身は100 Hz更新のpolicy補間 | WBC `stateDesired`、`getJointAngles` |
| 同上 | `optimizedInput` | `(24,)`: GRF 12 N W + 関節速度 12 rad/s | 同上 | WBC `inputDesired`、`getJointVelocities` |
| 同上 | `plannedMode` | scalar mode id | 同上 | WBC接地フラグ |
| `WeightedWbc::update` | `qpSol` | `(42,)`: \(\ddot q\) 18 + \(F_c\) 12 N + \(\tau\) 12 N·m | 500 Hz | `x.tail(12)` だけが指令 |
| `setCommand` | ハイブリッド指令 | 関節あたり `{q*, dq*, Kp, Kd, ff}` = 5。全体 60 scalar | 500 Hz | `UnitreeHW::write` / `writeSim` |
| `UnitreeHW::write` | `LowCmd` 12モータ | q, dq, Kp, Kd, tau | 500 Hz | モータドライバ |
| `LeggedHWSim::writeSim` | 努力指令 12 | \(\tau = K_p(q^*-q)+K_d(\dot q^*-\dot q)+\tau_{\mathrm{ff}}\) N·m | sim周期 | Gazebo |

`currentObservation_.input` は `evaluatePolicy` のあと `optimizedInput` で上書きされ、observationトピックへ出る。推定は入力を作らない。

## 4. 主要ベクトルの中身

### 4.1 NMPC状態 \(\mathbf{x}\) `(24,)`

`task.info` の `initialState` コメントが正本である。

| index | 記号 | 意味 | 単位 |
|---|---|---|---|
| 0:3 | \(v_{\mathrm{com}}\) | 正規化並進運動量（質量で割ったCoM速度） | m/s |
| 3:6 | \(L/m\) | 正規化角運動量 | m²/s 相当 |
| 6:9 | \(p_b\) | 胴体位置 | m W |
| 9:12 | \((\psi,\theta,\phi)\) | yaw, pitch, roll | rad |
| 12:24 | \(q_j\) | 関節角 LF/LH/RF/RH × HAA/HFE/KFE | rad |

### 4.2 NMPC入力 \(\mathbf{u}\) `(24,)`

READMEと `initializeInputCostWeight()` が正本である。

| index | 記号 | 意味 | 単位 |
|---|---|---|---|
| 0:12 | \(f_c\) | 4脚GRF。脚順は `contactNames3DoF` | N W |
| 12:24 | \(v_j\) | 12関節速度 | rad/s |

`task.info` の `R` 後半は「足速度」で書かれているが、実装は関節速度コストへJacobianで写す。詳細は[05](05_NMPC.md)。

### 4.3 剛体状態 \(\mathbf{x}_{\mathrm{rbd}}\) `(36,)`

`StateEstimateBase` の書き込み位置から確定する。`n_q = 18`。

| index | 意味 | 単位 |
|---|---|---|
| 0:3 | ZYX | rad |
| 3:6 | 胴体位置 | m W |
| 6:18 | 関節角 12 | rad |
| 18:21 | 胴体角速度（globalへ変換済） | rad/s |
| 21:24 | 胴体並進速度 | m/s W |
| 24:36 | 関節速度 12 | rad/s |

### 4.4 WBC決定変数 \(\mathbf{x}_{\mathrm{wbc}}\) `(42,)`

\[
\mathbf{x}_{\mathrm{wbc}}
=
[\ddot q^\mathsf T\ (18),\
F_c^\mathsf T\ (12),\
\tau^\mathsf T\ (12)]^\mathsf T
\]

`LeggedController` が下流へ渡すのは \(\tau\) だけである。\(F_c\) と \(\ddot q\) はWBC内部解である。

## 5. 実行周期の関係

```
HW loop 500 Hz (周期 2 ms)                    NMPC thread 100 Hz (周期 10 ms)
----------------------------                  -------------------------------
read: q12, dq12, IMU, contact4
        |
        v
     Kalman  ->  x (24,)
        |                                     advanceMpc
        v                                     観測 x (24,)
 evaluatePolicy                               参照 (24,)x2
 最新policyを t,x で補間                      ModeSchedule
        |                                            |
        v                                            |
 WeightedWbc                                         v
 x* (24,), u* (24,), mode                     policy を共有
        |                                     (500 Hz側が読む)
        v
 write: tau 12 + q* 12 + dq* 12
```

NMPC非更新の 4 回の 500 Hz 周期では、同じpolicyを新しい \(t, x\) で評価する。JacobianとWBC制約は毎回の実測で作り直す。

## 6. 元スケッチから直した点

質問文のASCII図に対する実装事実である。

1. **サイズを明示した。** 指令は3速度ではなく `Twist` の4成分（`linear.z` も含む）。NMPCは胴体12次元ではなく24次元。WBC出力はトルク12だけではなく内部42次元。
2. **図はアスキーアートにした。** ブロック間の shape / 単位を矢印の注記に書いた。mermaid は描画が崩れるため使わない。
3. **各ブロックの処理をIPOのI/Oだけにしない。** ロジック本文は[03](03_User_Command_and_Reference.md)–[07](07_Joint_Control_and_Hardware.md)。本章は境界契約に集中する。
4. **Gaitを並列パスとして追加した。** 元図の①→⑦には無いが、接地制約の本体である。
5. **目標軌道は密な未来軌道ではない。** 2点だけを線形補間する。`head(3)` に速度指令を入れるのは `cmd_vel` 経路だけである。
6. **推定とNMPCは直列だが周期が違う。** 推定はNMPCの前段というより、500 Hzループの先頭である。
7. **低レベルは「主にKd」で合っている。** 実装は `Kp=0`, `Kd=3` 固定。`Kp(q^*-q)` は既定ではゼロである。

## 7. 標準実装にない上位機能

次は現行標準経路に存在しない。追加する場合は推奨改善であり、実装事実ではない。

- 目的地からの障害物回避ローカルプランナ
- 地形マップをNMPC制約へ入れる知覚（後継 `legged_perceptive`）
- NMPCによるGait自動選択
- 既定コントローラからの `HierarchicalWbc` 利用
- `FrictionConeConstraint::setSurfaceNormalInWorld`（未実装で例外）
