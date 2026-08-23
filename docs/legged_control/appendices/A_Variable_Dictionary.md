# Variable Dictionary

変数のshape・単位・frame・生成元の正本である。物語は各章へリンクする。

数式記号との対応も本章にだけ横断掲載する。

| 数式 | コード変数 | 正本章 |
|---|---|---|
| \(\mathrm{cmdVel}\) | `cmdVel` `(4,)` | [03](../03_User_Command_and_Reference.md) |
| \(v_{\mathrm{W}}\) | `cmdVelRot` | [03](../03_User_Command_and_Reference.md) |
| \(T\) | `TIME_TO_TARGET` = `mpc.timeHorizon` | [03](../03_User_Command_and_Reference.md) |
| \(\mathbf{x}\) | `currentObservation_.state` / `optimizedState` | [02](../02_System_Architecture_and_Dataflow.md), [05](../05_NMPC.md) |
| \(\mathbf{u}\) | `optimizedInput` | [05](../05_NMPC.md) |
| \(\mathbf{x}_{\mathrm{rbd}}\) | `measuredRbdState_` / `rbdState_` | [04](../04_State_Estimation.md) |
| \(\hat x\) | `xHat_` | [04](../04_State_Estimation.md) |
| \(c_i\) | `contactFlag_` / `getContactFlags` | [04](../04_State_Estimation.md), [05](../05_NMPC.md) |
| \(\mathbf{x}_{\mathrm{wbc}}\) | `WeightedWbc::update` の戻り | [06](../06_WBC.md) |
| \(\tau\) | `x.tail(12)` / `ff_` | [06](../06_WBC.md), [07](../07_Joint_Control_and_Hardware.md) |
| \(q^*,\dot q^*\) | `posDes`, `velDes` | [07](../07_Joint_Control_and_Hardware.md) |

| 変数 | 意味 | Shape | 単位 | Frame | 生成元 | 主使用先 | 更新周期 |
|---|---|---|---|---|---|---|---|
| `/cmd_vel` | 胴体速度指令 | Twist。使うのは4 | m/s, rad/s | 指令。並進はyawでWへ | joy / 手動 | `cmdVelCallback` | イベント |
| `cmdVel` | 上記の内部 | `(4,)` | 同上 | 同上 | callback | `cmdVelToTargetTrajectories` | 同上 |
| `cmdGoal` | ゴール内部 | `(6,)` | m, rad | odom | `goalCallback` | `goalToTargetTrajectories` | イベント |
| `TargetTrajectories` | NMPC参照 | t:2, x:`(24,)×2`, u:`(24,)×2` | 混在 | centroidal | 上記2関数 | `RosReferenceManager` | 指令ごと |
| `jointPos` / `jointVel` | 実測関節 | 各`(12,)` | rad, rad/s | 関節 | HW handle | 推定 | 500 Hz |
| IMU quat, ω, a | 姿勢と慣性 | 4+3+3 | —, rad/s, m/s² | base | IMU | 推定 | 500 Hz |
| `contactFlag` | 接地 | `(4,)` bool | — | — | 足力またはGazebo接触 | KF、mode | 500 Hz |
| `xHat_` | KF状態 | `(18,)` | m, m/s | W | `KalmanFilterEstimate` | `updateLinear` | 500 Hz |
| `rbdState_` | 剛体状態 | `(36,)` | 混在 | §4.3 of [02](../02_System_Architecture_and_Dataflow.md) | 推定 | centroidal変換、WBC | 500 Hz |
| `currentObservation_.state` | NMPC観測 | `(24,)` | 混在 | [02](../02_System_Architecture_and_Dataflow.md) §4.1 | `computeCentroidalStateFromRbdModel` | NMPC, 指令ノード | 500 Hz |
| `optimizedState` | NMPC状態解の現在値 | `(24,)` | 同上 | 同上 | `evaluatePolicy` | WBC, `posDes` | 500 Hz読 |
| `optimizedInput` | NMPC入力解の現在値 | `(24,)` | N, rad/s | GRFはW | 同上 | WBC, `velDes` | 500 Hz読 |
| `plannedMode` | 計画接地mode | scalar | — | — | 同上 | WBC `contactFlag_` | 500 Hz読 |
| `qpSol` | WBC解 | `(42,)` | 混在 | [06](../06_WBC.md) | `WeightedWbc` | `tail(12)` | 500 Hz |
| hybrid command | 関節指令 | 5×12 | rad, rad/s, N·m/rad, N·m/(rad/s), N·m | 関節 | `setCommand` | HW write | 500 Hz |
| `LowCmd` | 実機モータ指令 | 12モータ×5 | 同上 | モータ | `UnitreeHW::write` | UDP | 500 Hz |

脚の接触添字は `modelSettings().contactNames3DoF` に従う。`task.info` の `R` コメントは LF, RF, LH, RH。関節状態コメントは LF, LH, RF, RH。名前でハンドルを取るため実行は一致するが、配列添字を混同しないこと。
