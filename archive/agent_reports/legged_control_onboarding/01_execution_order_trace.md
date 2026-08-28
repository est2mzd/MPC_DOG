# 01 — legged_control 実行順序トレース

日付: 2026-08-25
対象: `external/legged_control`（upstream commit `a7f381c0367e98e31c01336e678eef47e304d40d`、ROS1 + OCS2ベース）
関連: `notebook_legged/00_learning_map.ipynb`, `notebook_legged/01_packages_and_loop.ipynb`,
`notebook_legged/12_repository_code_walkthrough.ipynb`（既存の教育用notebookの記述をC++実ファイルに
当たって行番号レベルで裏取りしたもの。差異はなかった）

## 何をしたか

`external/legged_control` の主要C++ファイルを実際に読み、初期化から500Hz制御ループ・
100Hz MPCスレッドまでの呼び出し順を、ファイル名・関数名・役割つきで時系列に整理した。

読んで裏取りしたファイル:
- `legged_controllers/src/LeggedController.cpp`（`init`, `starting`, `update`, `updateStateEstimation`,
  `setupMpc`, `setupMrt`, `setupStateEstimate`）
- `legged_controllers/src/TargetTrajectoriesPublisher.cpp`（`cmdVelToTargetTrajectories`, `main`）
- `legged_estimation/src/LinearKalmanFilter.cpp`（コンストラクタの次元設定）
- `legged_interface/src/LeggedInterface.cpp`（`setupOptimalControlProblem`）
- `legged_wbc/src/WbcBase.cpp` / `WeightedWbc.cpp`（`update`, `formulate*Task`, qpOASES呼び出し）
- `legged_hw/src/LeggedHWLoop.cpp`（500Hzループ）
- `legged_examples/legged_unitree/legged_unitree_hw/src/legged_unitree_hw.cpp`（実機`main`）

## 重要な注意（このリポジトリ固有）

このプロジェクトでは **legged_control のROS1/OCS2版を実際にビルド・実行していない**。
以下のトレースは静的なコードリーディングによる呼び出し順の再構成であり、実行時計測ではない。
また、このリポジトリには `src/legged_control_mujoco/` という別実装（project所有のMuJoCo
実行アダプタ）が存在するが、これは以下でトレースする upstream ROS1/OCS2実装（`external/legged_control/`）
とは別物であり、OCS2のSQPをその場forceプランナーに、Pinocchio/qpOASES WBCをMuJoCoの
逆動力学に置き換えた別実装である。本ファイルの内容を「project実行結果」と混同しないこと。

## A. 初期化フェーズ（`LeggedController::init`、1回だけ）

`legged_controllers/src/LeggedController.cpp` L28–77。ros_controlのcontroller pluginとして
`init()` が1回呼ばれる。

| 順 | 関数 | 役割 |
|---|---|---|
| A1 | `setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose)` (L39, 実体 L200) | `LeggedInterface` を生成し `LeggedInterface::setupOptimalControlProblem(...)` を呼ぶ |
| A1-a | `legged_interface/src/LeggedInterface.cpp::setupOptimalControlProblem` (L80) | OCPを構築: `dynamicsPtr = LeggedRobotDynamicsAD`(L95), cost/constraints（摩擦・接触・衝突）を登録 |
| A2 | `setupMpc()` (L40, 実体 L206) | `SqpMpc` を生成、`GaitReceiver`・`RosReferenceManager` を同期モジュールとして登録 |
| A3 | `setupMrt()` (L41, 実体 L225) | `MPC_MRT_Interface` を生成し、`advanceMpc()` を回す別スレッド（100Hz、`mpcThread_`）を起動 |
| A4 | ハードウェアハンドル取得 (L52–63) | `HybridJointInterface`/`ContactSensorInterface`/IMUのハンドルを取得 |
| A5 | `setupStateEstimate(taskFile, verbose)` (L66, 実体 L253) | `KalmanFilterEstimate` を生成（`LinearKalmanFilter.cpp`） |
| A6 | `wbc_ = std::make_shared<WeightedWbc>(...)` (L69–71) | 全身QBソルバーを生成し `loadTasksSetting` でタスク重みを読み込む |
| A7 | `safetyChecker_ = std::make_shared<SafetyChecker>(...)` (L74) | 安全チェッカーを生成 |

`starting()` (L79–99): 初期状態を推定し、`mpcMrtInterface_->advanceMpc()` を
**最初のpolicyが届くまでブロッキングで**回してから `mpcRunning_ = true` にする
（＝制御ループは「未計画状態でのゼロ入力」を送らない設計）。

## B. 100Hz MPCスレッド（`setupMrt` が起動した別スレッド、ループ中ずっと並行動作）

`LeggedController.cpp` L231–249（`mpcThread_` のラムダ）:

| 順 | 関数 | 役割 |
|---|---|---|
| B1 | `mpcMrtInterface_->advanceMpc()` | 直近の観測・参照からOCS2 SQPを1回解き、新しいpolicy（状態・入力・mode軌道）を計算 |
| B2 | `executeAndSleep(..., mpcDesiredFrequency_)` | 実行後、設定周波数（既定100Hz）に合わせてスリープ |

参照側の入力は別ノード:
`legged_controllers/src/TargetTrajectoriesPublisher.cpp::main` (L92) が
`/cmd_vel` を購読し `cmdVelToTargetTrajectories(...)` (L67) で
$v_W = R_{ZYX}(\psi) v_{cmd}$, $p_{xy}^+ = p_{xy} + v_{W,xy} T$, $\psi^+ = \psi + \dot\psi T$
（`TIME_TO_TARGET` = `mpc.timeHorizon` 設定値）により2時刻分の目標軌道を作りpublishする。
歩容（stance/trot/...）は別途 `GaitReceiver` 経由で与えられ、MPC自身は歩容を選ばない。

## C. 500Hz 制御ループ（実機: `LeggedHWLoop`／Gazebo: `gazebo_ros_control`）

`legged_hw/src/LeggedHWLoop.cpp` L44 `update()` → `controllerManager_->update(...)` (L68) が
`LeggedController::update(time, period)` を呼ぶ。

### C0. 入口
`legged_hw/src/LeggedHWLoop.cpp::update()` (L44) が ros_control の `controllerManager_->update()`
経由で `LeggedController::update()` を毎周期（既定500Hz）呼ぶ。Gazebo実行時はこの周期を
`gazebo_ros_control` プラグインが担う（同じ`LeggedController`が呼ばれる）。

### C1. `LeggedController::update(time, period)` (`LeggedController.cpp` L101–144)

| 順 | 関数 | 役割 |
|---|---|---|
| ① | `updateStateEstimation(time, period)` (L103, 実体 L146) | 下記C2を実行し `measuredRbdState_` と `currentObservation_.state/mode` を更新 |
| ② | `mpcMrtInterface_->setCurrentObservation(currentObservation_)` (L106) | 最新の推定状態をMPC側と共有 |
| ③ | `mpcMrtInterface_->updatePolicy()` (L109) | Bスレッドが計算した最新policyを取り込む |
| ④ | `mpcMrtInterface_->evaluatePolicy(currentObservation_.time, state, optimizedState, optimizedInput, plannedMode)` (L114) | policy軌道を「現在時刻の1点」に評価（$x^*(t), u^*(t), mode(t)$） |
| ⑤ | `wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec())` (L120) | 下記C3のWBCを解く（戻り値は42変数：$\dot q_b(6)$相当の一般化加速度18＋接触力12＋トルク12） |
| ⑥ | `torque = x.tail(12)` (L123) | 末尾12個（関節トルク）だけを取り出す |
| ⑦ | `posDes = getJointAngles(optimizedState,...)`, `velDes = getJointVelocities(optimizedInput,...)` (L125–126) | MPC解から目標関節角度・角速度を取得 |
| ⑧ | `safetyChecker_->check(...)` (L129) | 失敗時は `stopRequest(time)` でコントローラ停止 |
| ⑨ | `hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j))` (L135) | 関節ごとに `(目標位置, 目標速度, Kp=0, Kd=3, トルクfeedforward)` を送る |
| ⑩ | `robotVisualizer_->update(...)` / `observationPublisher_.publish(...)` (L139–143) | 可視化・観測のpublish（制御には影響しない） |

### C2. 状態推定 — `updateStateEstimation` (`LeggedController.cpp` L146–183)
| 順 | 関数 | 役割 |
|---|---|---|
| ① | `stateEstimate_->updateJointStates(jointPos, jointVel)` (L174) | 関節センサ値を入力 |
| ② | `stateEstimate_->updateContact(contactFlag)` (L175) | 接地センサ値を入力 |
| ③ | `stateEstimate_->updateImu(...)` (L176) | IMU姿勢・角速度・線形加速度と共分散を入力 |
| ④ | `measuredRbdState_ = stateEstimate_->update(time, period)` (L177) | `KalmanFilterEstimate`（実体 `LinearKalmanFilter.cpp`）内部で線形カルマンフィルタを実行。状態次元 `numState_ = 6 + 3*numContacts`（4脚なら18）、観測次元 `numObserve_ = 2*3*numContacts + numContacts`（同28）。予測: $x_{k+1}=Ax_k+Bu_k$（$A_{p,v}=\Delta t I$ 等）で浮動base位置・速度と各脚のworld接地位置を推定 |
| ⑤ | `currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_)` (L180) | rigid-body状態（36次元相当）をOCS2のcentroidal状態（24次元）へ変換 |
| ⑥ | `currentObservation_.mode = stateEstimate_->getMode()` (L182) | 推定された現在の接地モードを反映 |

### C3. WBC（全身制御QP）— `WeightedWbc::update` (`legged_wbc/src/WeightedWbc.cpp` L11–48)
| 順 | 関数 | 役割 |
|---|---|---|
| ① | `WbcBase::update(...)` (L13, 実体 `WbcBase.cpp` L29) | `contactFlag_` を更新し、`updateMeasured(rbdStateMeasured)` / `updateDesired(stateDesired, inputDesired)` を呼んでPinocchioでヤコビアン・質量行列・非線形項を再計算 |
| ② | `formulateConstraints()` (L16, 実体 L50) | ハード制約 = `formulateFloatingBaseEomTask` + `formulateTorqueLimitsTask` + `formulateFrictionConeTask` + `formulateNoContactMotionTask`（`WbcBase.cpp` L97, 110, 142, 125） |
| ③ | `formulateWeightedTasks(stateDesired, inputDesired, period)` (L30, 実体 L54) | ソフトタスク = `formulateSwingLegTask * weightSwingLeg_` + `formulateBaseAccelTask * weightBaseAccel_` + `formulateContactForceTask * weightContactForce_`（既定重み: swingLeg=100, baseAccel=1, contactForce=0.01 相当。実値は `task.info` の `weight.*`） |
| ④ | `H = A_soft^T A_soft`, `g = -A_soft^T b_soft` (L31–32) | 二次コストを組み立て |
| ⑤ | `qpOASES::QProblem(...).init(H, g, A, ..., lbA, ubA, nWsr=20)` (L35–43) | qpOASESで単一QPを解く（`HierarchicalWbc` は同梱されているが既定では未使用） |
| ⑥ | `qpProblem.getPrimalSolution(qpSol.data())` (L46) | 解を取得（戻り値42次元: 一般化加速度18 + 接触力12 + トルク12）。solver return codeの分岐は無く、失敗時のフォールバックは未実装 |

## D. Plant（実機／Gazebo）

- 実機: `legged_examples/legged_unitree/legged_unitree_hw/src/legged_unitree_hw.cpp::main` (L42) →
  `LeggedHWLoop` を生成し、SCHED_FIFOスレッドで500Hzループを駆動、Unitree SDK経由でモータへ書き込む。
- シミュレーション: `legged_gazebo` の `gazebo_ros_control` プラグインが同じ `LeggedController` を
  ros_control経由で駆動する（`legged_unitree_hw` の代わりにGazeboがハードウェアI/Oを担当）。

## 全体像（時系列サマリ）

```
起動: LeggedController::init
  ├─ setupLeggedInterface → LeggedInterface::setupOptimalControlProblem (dynamics/cost/constraints登録)
  ├─ setupMpc            → SqpMpc生成 + GaitReceiver/RosReferenceManager登録
  ├─ setupMrt            → advanceMpc()スレッド起動 (100Hz)
  ├─ setupStateEstimate  → KalmanFilterEstimate生成
  └─ WeightedWbc生成 + SafetyChecker生成
  ↓
starting(): 初回policy到着までadvanceMpc()をブロッキング実行
  ↓
[500Hz、LeggedHWLoop / gazebo_ros_control が駆動]      [100Hz、別スレッドで並行]
LeggedController::update                                mpcMrtInterface_->advanceMpc()
  ① updateStateEstimation                                 (直近の観測+参照からSQPを1回解き
       (Kalman filter → centroidal state 24次元)             新しいpolicyを生成)
  ② setCurrentObservation
  ③ updatePolicy (Bの最新policyを取り込む)
  ④ evaluatePolicy(now) → x*(24), u*(24), mode
  ⑤ WeightedWbc::update
       (formulateConstraints + formulateWeightedTasks
        → qpOASES QP(42変数) → qdd(18), Fc(12), tau(12))
  ⑥ torque = tau
  ⑦ posDes/velDes をMPC解から取得
  ⑧ safetyChecker_->check (失敗ならcontroller停止)
  ⑨ setCommand(posDes, velDes, Kp=0, Kd=3, tau_ff)  ← 各関節へ
  ↓
Plant: Unitree実機 or Gazebo
  (関節センサ・IMU・接地センサ) ──────────────────→ 推定へ戻る
```

## 次にやること（未着手）

- `HierarchicalWbc`（コードには存在するが既定未使用）の内部トレースは行っていない。
- OCS2本体（SQPソルバーの内部反復、centroidal dynamicsの数式導出）はこのworkspaceに
  無いため、公開APIから分かる範囲に留めている。
- ROS2移行版・`src/legged_control_mujoco/` アダプタとの対応関係は本ファイルの対象外
  （`notebook_legged/15_ros_migration_logic_parity.ipynb` 側の話）。
