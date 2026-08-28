# Joint Control and Hardware

Q2のブロック⑥と⑦である。

## 1. 結論

低レベルは関節空間のハイブリッド指令である。WBCトルクを feedforward にし、位置ゲインは0、速度ゲインは3である。Gazeboでは同じ式をプラグイン内で計算し、実機ではUnitree LowLevel コマンドとしてモータ側PDへ渡す。12関節すべてが同じゲインである。

## 2. ブロック⑥ 低レベル関節制御

### 2.1 背景

全身QPのトルクだけを開ループで流すと、接触衝撃とモデル誤差で関節が暴れる。一方で高い関節PDはWBCの力制御と喧嘩する。本スタックは「低ゲインPD + トルクFF」で衝撃を抑えつつ、力はWBCに任せる。READMEもその意図を書いている。

### 2.2 目的

- \(\tau_{\mathrm{WBC}}\) をモータの主トルクにする。
- NMPCの \(q_j^*, \dot q_j^*\) を弱い速度フィードバックの目標にする。
- 位置フィードバックは既定で切る（\(K_p=0\)）。

### 2.3 指令の組み立て

`evaluatePolicy` のあと、

```
posDes = getJointAngles(optimizedState, info)        // (12,) rad
velDes = getJointVelocities(optimizedInput, info)    // (12,) rad/s
torque = x_wbc.tail(12)                              // (12,) N·m
```

各関節 \(j=0\ldots11\) へ

```
hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j))
```

`setCommand` の引数順は `(q*, dq*, Kp, Kd, ff)` である。

概念式は次である。実装の \(K_p\) は0なので、括弧の第2項は消える。

\[
\tau_{\mathrm{cmd}}
=
\tau_{\mathrm{WBC}}
+
K_p(q^*-q)
+
K_d(\dot q^*-\dot q)
=
\tau_{\mathrm{WBC}}
+
3\,(\dot q^*-\dot q)
\]

ユーザー原稿の「主にKd」は実装事実である。

関節順は

`LF_HAA, LF_HFE, LF_KFE, LH_HAA, LH_HFE, LH_KFE, RF_HAA, RF_HFE, RF_KFE, RH_HAA, RH_HFE, RH_KFE`

である。NMPC状態の関節ブロックと同じ LF/LH/RF/RH である。

流れるデータは関節あたり5スカラー、全体60スカラーである。

対応コード: `LeggedController::update()` 末尾、`HybridJointInterface.h` の `setCommand`。

### 2.4 コントローラ未接続時の安全

`UnitreeHW::read` は毎周期、全関節の ff と \(\dot q^*\) を0、`Kd=3` に戻してからコントローラが上書きする。コントローラが指令しなければダンピングだけが残る。Gazebo `readSim` は \(q^*=q\)、\(\dot q^*=\dot q\)、ゲイン0、ff 0 に戻す。

## 3. ブロック⑦ モータ / シミュレータ

### 3.1 背景

ros-control の役割は、制御器と「関節ハンドル」を切り、実機UDPとGazeboを同じ `HybridJointInterface` で動かすことである。ロボットを増やすときは `LeggedHW` を継承して `read/write` を書く、とREADMEが案内している。

### 3.2 実機 `UnitreeHW`

`LeggedHWLoop` が 500 Hz で `read → update → write` する。

`write()` は12モータへ

| `LowCmd` フィールド | 中身 |
|---|---|
| `q` | `posDes_` |
| `dq` | `velDes_` |
| `Kp` | `kp_`（既定0） |
| `Kd` | `kd_`（既定3） |
| `tau` | `ff_`（WBCトルク） |

のあと `Safety::PositionLimit` と `PowerProtect`（a1は `power_limit: 4`）を掛け、UDP送信する。

`read()` は `motorState[i].q, dq, tauEst`、IMU quat/gyro/acc、`footForce` をハンドルへ書く。Unitree脚順 FR/FL/RR/RL とコントローラの LF/LH/RF/RH は、関節名で `setupJoints` が対応付ける。

対応コード: `legged_unitree_hw/src/UnitreeHW.cpp`、`config/a1.yaml`、`legged_hw/src/LeggedHWLoop.cpp`。

### 3.3 Gazebo `LeggedHWSim`

`writeSim` は指令を `delay`（既定 9 ms）のFIFOに入れ、取り出した指令で努力を計算する。

\[
\tau
=
K_p(q^*-q)+K_d(\dot q^*-\dot q)+\tau_{\mathrm{ff}}
\]

既定ゲインでは \(\tau=\tau_{\mathrm{WBC}}+3(\dot q^*-\dot q)\)。これを `DefaultRobotHWSim::writeSim` がGazebo関節へ書く。

IMUはリンクのWorld姿勢と相対加速度から作り、接触は `ContactManager` のリンク名一致である。名前は `LF_FOOT, LH_FOOT, RF_FOOT, RH_FOOT`。

対応コード: `legged_gazebo/src/LeggedHWSim.cpp`、`config/default.yaml`。

### 3.4 閉ループ

モータ（またはGazebo）が次周期の \(q,\dot q\)、IMU、接地を返し、ブロック③へ戻る。植物側の次元は浮動ベース6 + 関節12である。制御が直接書くのは関節12だけである。

## 4. 実装事実と理論の境界

- **実装事実**: \(K_p=0\), \(K_d=3\) 固定。ロボット別チューニングは無い。実機はモータ内PD、simはプラグインPD。
- **理論**: 低ゲイン関節PDは接触剛性を下げ、トルクFFがWBCを実現する。
- **推奨改善**: ゲインを `task.info` へ出す。脚・関節ごとのKd。
- **未実装**: 電流ループやモータ慣性補償の明示モデル。それらはSDK/Gazebo側である。
