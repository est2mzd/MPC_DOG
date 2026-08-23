# Packages and Control Loop

## 1. 結論

`legged_control` は ros-control の `LeggedController` を中核に、指令生成ノード、OCS2 NMPC、線形Kalman推定、WBC、ハイブリッド関節指令を組み合わせる。実機は 500 Hz のハードウェアループ、NMPCは別スレッド 100 Hz である。

パッケージの役割と周期は本章を正本とする。境界データの型は[02](02_System_Architecture_and_Dataflow.md)である。

## 2. パッケージ対応

| パッケージ | 役割 | このリポジトリでの主要ファイル |
|---|---|---|
| `legged_controllers` | 制御器プラグイン、目標軌道ノード | `LeggedController.cpp`, `TargetTrajectoriesPublisher.cpp` |
| `legged_interface` | OCS2 OCP組み立て | `LeggedInterface.cpp`, 制約・コスト |
| `legged_estimation` | 状態推定 | `LinearKalmanFilter.cpp`, `StateEstimateBase.cpp` |
| `legged_wbc` | 全身QP | `WeightedWbc.cpp`, `WbcBase.cpp`, `HierarchicalWbc.cpp` |
| `legged_hw` | 実機ループ | `LeggedHWLoop.cpp` |
| `legged_unitree_hw` | Unitree UDP read/write | `UnitreeHW.cpp` |
| `legged_gazebo` | Gazebo上の同一HWインタフェース | `LeggedHWSim.cpp` |
| `legged_common` | `HybridJointHandle`, 接地センサ | `HybridJointInterface.h` |
| `qpoases_catkin` | WBC用 qpOASES | ラッパのみ |
| OCS2（外部） | SQP-NMPC、Gait、centroidal dynamics | 本 repo にソース無し |

`legged_control/legged_control` はメタパッケージである。

## 3. 起動時に立つプロセス

`load_controller.launch` が次を同時に起動する。

| プロセス | パッケージ | 役割 |
|---|---|---|
| `controller_manager load .../legged_controller` | `controller_manager` | プラグインをロード。startは別サービス |
| `legged_robot_gait_command` | `ocs2_legged_robot_ros` | 端末から gait 名を送り `ModeSchedule` を更新 |
| `legged_robot_target` | `legged_controllers` | `/cmd_vel` と `/move_base_simple/goal` を `TargetTrajectories` に変換 |

対応コード: `legged_controllers/launch/load_controller.launch`。

ゲームパッドは任意で `joy_teleop.launch` が `/cmd_vel` を出す。軸は `joy.yaml`。

## 4. 2つの周期

### 4.1 ハードウェア / WBC ループ

実機は `LeggedHWLoop` が `loop_frequency` で `read → controller_manager.update → write` を回す。A1既定は 500 Hz。

| 項目 | 値 | 根拠 |
|---|---:|---|
| `loop_frequency` | 500 Hz | `legged_unitree_hw/config/a1.yaml` |
| `cycle_time_error_threshold` | 0.002 s | 同上 |
| `LeggedController::update` | 同じ 500 Hz | `controller_manager_->update` |
| 状態推定 | 500 Hz | `update()` の先頭で毎回 |
| WBC | 500 Hz | 同上 |
| 関節指令 | 500 Hz | `setCommand` |

Gazeboは `gazebo_ros_control` が同等の `readSim / writeSim` を回す。指令遅延の既定は `delay: 0.009` s。

### 4.2 NMPC スレッド

`setupMrt()` が別 `std::thread` を立て、`mpcDesiredFrequency_` で `mpcMrtInterface_->advanceMpc()` を呼ぶ。

| 項目 | 値 | 根拠 |
|---|---:|---|
| `mpc.timeHorizon` | 1.0 s | `task.info` |
| `mpc.mpcDesiredFrequency` | 100 Hz | 同上 |
| `sqp.dt` | 0.015 s | 同上。射撃間隔 |
| `sqp.sqpIteration` | 1 | RTIに近い1回SQP |
| 予測段数 | \(1.0/0.015 \approx 67\) | 設定からの導出 |

WBC側は毎周期 `updatePolicy()` のあと `evaluatePolicy(t, x)` で、最新policyを現在時刻に補間する。NMPCが遅れても前回policyを使い続ける。

`mpc.mrtDesiredFrequency` は `task.info` 上 1000 Hzだが、コメントが `Useless` であり、通常ループの周期には使わない。初期policy待ちの sleep にだけ使う。

対応コード: `legged_controllers/src/LeggedController.cpp` の `setupMrt()`, `update()`。設定は `legged_controllers/config/a1/task.info` の `mpc` と `sqp`。

## 5. `LeggedController::update` の呼出順

実装事実としての1周期は次である。

1. `updateStateEstimation` → `measuredRbdState_` (36)、`currentObservation_.state` (24)
2. `mpcMrtInterface_->setCurrentObservation`
3. `updatePolicy`
4. `evaluatePolicy` → `optimizedState` (24)、`optimizedInput` (24)、`plannedMode`
5. `wbc_->update` → `x` (42)。`torque = x.tail(12)`
6. `SafetyChecker`。失敗なら `stopRequest`
7. 12関節へ `setCommand(q*, dq*, 0, 3, τ)`
8. visualization と `legged_robot_mpc_observation` の publish

対応コード: `LeggedController::update()`。

## 6. 2種類のコントローラ

| プラグイン | 推定 | 用途 |
|---|---|---|
| `legged/LeggedController` | `KalmanFilterEstimate` | 実機と通常sim |
| `legged/LeggedCheaterController` | `FromTopicStateEstimate`（`/ground_truth/state`） | sim専用。READMEは実機禁止 |

両方とも WBC は `WeightedWbc` である。`HierarchicalWbc` はソースにあるが `init()` から呼ばれない。

## 7. ロボット差分

`ROBOT_TYPE` で `a1` / `go1` / `aliengo` を切り替える。NMPC次元は同じ24/24。変わるのはURDF、`comHeight`、初期関節、トルク上限である。

| ロボット | `comHeight` | 備考 |
|---|---:|---|
| a1, go1 | 0.3 m | `reference.info` |
| aliengo | 0.4 m | 同上 |

本章以降の数値例は a1 である。
