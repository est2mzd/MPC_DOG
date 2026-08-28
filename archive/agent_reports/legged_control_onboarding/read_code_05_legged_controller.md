# 制御ロジック本体(頭脳) legged_controllers/LeggedController 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
controller_manager(ros_control、外部)が controllers.yaml の
legged/LeggedController をロード
  → LeggedController::init(robot_hw, controller_nh)   ← 本ファイル、起動時1回
      → setupLeggedInterface / setupMpc / setupMrt      ← 本ファイル、起動時1回
      → setupStateEstimate / WeightedWbc生成 / SafetyChecker生成

controllerManager_->update(...)(read_code_01のLeggedHWLoop::update経由、
または read_code_03のGazebo経路)が毎周期呼ぶ:
  → LeggedController::starting(time)  ← アクティブ化時1回のみ
  → LeggedController::update(time, period)   ← 本ファイル、毎制御周期
      (a1/go1既定500Hz、aliengo既定800Hz、read_code_01)

setupMrt()が起動する独自スレッド mpcThread_ が並行して:
  → mpcMrtInterface_->advanceMpc()    ← 本ファイル、独立スレッドで
      既定100Hz(task.infoのmpcDesiredFrequency)ごとに、状態観測から
      OCS2のSQP-MPCを1回解く(OCS2内部、対象外・未確認)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`LeggedController`が担当するのは、「**状態推定→OCS2への現在状態の受け渡し→
最新のMPC解の評価→WBCによる関節トルク・目標角度への変換→ハードウェアへの
指令送信**」という、pympcでいう`WBInterface`+`SRBDControllerInterface`+
`quadruped_pympc_wrapper.py`を合わせたような**制御ロジックの中枢**です。

- OCS2自体のMPCソルバー内部(`SqpMpc`、`MPC_MRT_Interface`)は対象外・
  未確認。このファイルはOCS2への「入力の準備」と「出力の消費」を担当する
- **MPCの求解は、`update()`が呼ばれるros_controlの制御周期(500〜800Hz)とは
  別の、独立したバックグラウンドスレッド(`mpcThread_`)で非同期に行われる**。
  `update()`側は、そのスレッドが最後に計算し終えた最新のMPC解を
  「評価(補間)」して使うだけで、MPCの求解そのものを待つことはない
- 状態推定の詳細(`legged_estimation`)、MPCの問題定義
  (`legged_interface`)、WBCの内部(`legged_wbc`)はいずれも別ファイルの
  責務。このファイルはそれらを**呼び出して繋ぐ**役割に徹する

対象は`external/legged_control/legged_controllers/include/legged_controllers/LeggedController.h`
(84行)・`external/legged_control/legged_controllers/src/LeggedController.cpp`
(268行)です。

---

## `LeggedController.h` 27〜77行:クラス定義

```cpp
class LeggedController : public controller_interface::MultiInterfaceController<
    HybridJointInterface, hardware_interface::ImuSensorInterface, ContactSensorInterface> {
```

- `controller_interface::MultiInterfaceController<...>`(ros_control標準、
  外部)：テンプレート引数に列挙した複数のハードウェアインターフェース型
  ([read_code_02](read_code_02_legged_hw_interface.md)で読んだ3種)を
  同時に要求するコントローラ基底クラス。`init`内で`robot_hw->get<T>()`と
  呼べる保証を与える

主なメンバ変数(型・単位・役割):

| メンバ | 型 | 役割 |
|---|---|---|
| `leggedInterface_` | `std::shared_ptr<LeggedInterface>` | OCS2向けのロボットモデル・コスト・制約定義(未読、次章以降) |
| `currentObservation_` | `SystemObservation`(OCS2型) | 現在の時刻・状態・入力・モードをまとめた観測値 |
| `measuredRbdState_` | `vector_t`(`Eigen::VectorXd`) | 剛体力学(RBD)形式の実測状態 |
| `stateEstimate_` | `std::shared_ptr<StateEstimateBase>` | 状態推定器(未読、次々章) |
| `wbc_` | `std::shared_ptr<WbcBase>` | 全身制御器(未読、後章) |
| `mpc_` | `std::shared_ptr<MPC_BASE>`(OCS2型) | MPCソルバー本体(実体は`SqpMpc`) |
| `mpcMrtInterface_` | `std::shared_ptr<MPC_MRT_Interface>`(OCS2型) | MPCスレッドとメインスレッドの橋渡し(Model Reference/Real-Time interface) |
| `mpcThread_` | `std::thread` | MPCを非同期に回す専用スレッド |
| `mpcTimer_`/`wbcTimer_` | `benchmark::RepeatedTimer`(OCS2型) | MPC・WBCそれぞれの実行時間の統計を取るタイマー |

---

## `LeggedController.cpp` 28〜77行:`init`

この関数の役割:設定ファイルを読み込んでOCS2向けインターフェースを構築し、
ハードウェアハンドルを取得し、状態推定器・WBC・安全チェッカーを生成する。

```cpp
controller_nh.getParam("/urdfFile", urdfFile);
controller_nh.getParam("/taskFile", taskFile);
controller_nh.getParam("/referenceFile", referenceFile);
bool verbose = false;
loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
setupMpc();
setupMrt();
```

- `urdfFile`/`taskFile`/`referenceFile`(型`std::string`)：
  [read_code_01](read_code_01_legged_hw_loop.md)で見た`load_controller.launch`
  が設定するグローバルrosparam(`/urdfFile`等)から取得。実際のパスは
  `legged_controllers/config/<robot>/task.info`等
- `verbose`(型`bool`)：`task.info`内の`legged_robot_interface.verbose`
  キーから読み込む、OCS2独自の`.info`形式パーサ(`loadData::loadCppDataType`、
  外部)を使う

```cpp
auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                     "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
for (const auto& joint_name : joint_names) {
  hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
}
auto* contactInterface = robot_hw->get<ContactSensorInterface>();
for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
  contactHandles_.push_back(contactInterface->getHandle(name));
}
imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
```

- 関節順序は`LF→LH→RF→RH`(Left-Front→Left-Hind→Right-Front→Right-Hind)、
  各脚`HAA→HFE→KFE`の順で**ハードコード**されている。
  [read_code_04](read_code_04_unitree_hw.md)で見た`UnitreeHW`側の内部
  インデックス順(SDK基準)とは別に、**MPC/WBC側で使う正準の関節順序は
  この12個のリスト**であり、以降のOCS2状態ベクトルの関節角度成分は
  この順序に対応すると考えられる(**設計上の解釈**)
- `contactHandles_`は`leggedInterface_->modelSettings().contactNames3DoF`
  (未読の`LeggedInterface`から取得)の順序に従う。ハードコードではなく
  設定ファイル由来である点が関節名リストとは対照的
- `imuSensorHandle_`：IMU名は`"base_imu"`固定
  ([read_code_03](read_code_03_legged_hw_sim.md)・
  [read_code_04](read_code_04_unitree_hw.md)双方で登録名が`"base_imu"`
  だったことと一致)

```cpp
setupStateEstimate(taskFile, verbose);

wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
wbc_->loadTasksSetting(taskFile, verbose);

safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
```

- WBCの実装は**既定で`WeightedWbc`が選ばれる**(`legged_wbc`パッケージには
  `HierarchicalWbc`という別方式も存在するが、このファイルではimportされて
  いるだけで実際には使われていない、**実装上の注意点**、未使用import)
- `wbc_->loadTasksSetting(taskFile, verbose)`：WBCの各タスク(重み等)を
  `task.info`から読み込む(詳細は`legged_wbc`のファイルで扱う)

---

## `LeggedController.cpp` 79〜99行:`starting`

この関数の役割:コントローラがアクティブ化された瞬間に1回だけ呼ばれ、
初期状態を設定し、MPCの初回解が届くまで待ってからMPCスレッドを起動する。

```cpp
currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
updateStateEstimation(time, ros::Duration(0.002));
currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
currentObservation_.mode = ModeNumber::STANCE;

TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

mpcMrtInterface_->setCurrentObservation(currentObservation_);
mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
ROS_INFO_STREAM("Waiting for the initial policy ...");
while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
  mpcMrtInterface_->advanceMpc();
  ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
}
ROS_INFO_STREAM("Initial policy has been received.");
mpcRunning_ = true;
```

- 起動直後のモード(`currentObservation_.mode`)は**`ModeNumber::STANCE`
  (全脚接地)へ強制設定**される。pympc側の`WBInterface.pgg.gait_type=7`
  (`FULL_STANCE`)を起動時に強制していたのと同じ思想の安全策
- `while (!initialPolicyReceived() ...) { advanceMpc(); ... }`：
  **`starting()`自身が、このメインスレッドの中でブロッキングに
  `advanceMpc()`を呼び続ける**。この時点では`mpcRunning_`はまだ`false`
  なので、`setupMrt()`(`init()`時にすでに起動済み)のバックグラウンド
  スレッド側は`advanceMpc()`を呼ばずに待機している([後述](#leggedcontrollercpp-225251行setupmrt)、
  `mpcRunning_`フラグで制御)。つまり「最初の1回の解を得るまではメイン
  スレッドが直接ソルバーを回し、解が得られたらバックグラウンドスレッドへ
  バトンタッチする」という初期化専用の経路になっている
- 待機ループの間隔は`mrtDesiredFrequency_`(`task.info`で**`1000`Hz**、
  ただしコメントで`; [Hz] Useless`(=使われていない)と明記されている、
  **実装上の注意点**：設定ファイル自体が「この値は使われていない」と
  自認しているにもかかわらず、この`starting()`のポーリング間隔として
  実際に使われている。コメントとコードが食い違っている可能性がある)

---

## `LeggedController.cpp` 101〜144行:`update`

この関数の役割:状態推定→MPC解の評価→WBC→安全チェック→ハードウェア指令の
送信、という1制御周期分の処理を実行する。

```cpp
updateStateEstimation(time, period);
mpcMrtInterface_->setCurrentObservation(currentObservation_);
mpcMrtInterface_->updatePolicy();

vector_t optimizedState, optimizedInput;
size_t plannedMode = 0;
mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
```

- `mpcMrtInterface_->updatePolicy()`：バックグラウンドの`mpcThread_`が
  最後に計算し終えた最新の最適化結果(ポリシー)を取り込む(OCS2内部の
  スレッドセーフな受け渡し機構、**未確認**)
- `mpcMrtInterface_->evaluatePolicy(...)`：現在時刻・現在状態における
  最適状態・最適入力を、そのポリシー(通常は区分的な軌道・フィードバック
  ゲイン)から**補間して**取り出す。MPCが今この瞬間に新しく解いた値では
  なく、**直近に完了した最適化結果を現在時刻まで外挿・補間した値**である
  点がpympc(MPCが解いたその周期のGRFをそのまま使う)との構造上の違い

```cpp
currentObservation_.input = optimizedInput;
wbcTimer_.startTimer();
vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
wbcTimer_.endTimer();

vector_t torque = x.tail(12);
vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
```

- `wbc_->update(...)`(未読、`legged_wbc`)の戻り値`x`は末尾12要素が
  関節トルク(`torque = x.tail(12)`)。先頭の要素(浮動base加速度+接地力等と
  推測、**未確認**、`legged_wbc`のファイルで確認する)は使われない
- `posDes`(rad)・`velDes`(rad/s)は**WBCの出力ではなく、MPCが最適化した
  状態・入力(`optimizedState`/`optimizedInput`)から直接取り出される**。
  `centroidal_model::getJointAngles`/`getJointVelocities`
  (OCS2セントロイダルモデルのヘルパー関数)がその変換を担う

```cpp
if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
  ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
  stopRequest(time);
}

for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
  hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
}
```

**コードで確認した事実(重要)**：最終的な関節コマンドは
`setCommand(posDes(j), velDes(j), 0, 3, torque(j))`、すなわち
[read_code_02](read_code_02_legged_hw_interface.md)の
\(\tau=k_p(p_{des}-p)+k_d(v_{des}-v)+\tau_{ff}\)の式で言う
**\(k_p=0\)、\(k_d=3\)固定**です。つまり実際に送られる指令は
「位置フィードバックゲインはゼロ(位置追従は行わない)、速度フィード
バックゲインは固定値`3`(N·m·s/rad)だけの軽い減衰、そして
`torque(j)`(WBCが計算したトルク、すでに姿勢・接触力・タスク追従を
織り込み済み)がほぼ全てを担う」という設計になっています。
[read_code_04](read_code_04_unitree_hw.md)で見た`UnitreeHW::read`の
安全リセット値(`kd=3.`固定)と**数値が一致**しているのは偶然ではなく、
`update`が送る通常運転時の`kd`と同じ値を安全側デフォルトに採用している
ためと考えられます(**設計上の解釈**)。

- 安全チェック失敗時は`stopRequest(time)`(ros_control標準API、外部)を
  呼ぶが、**このループ自体はここで`return`せず、下のコマンド送信・
  可視化・publish処理までそのまま実行され続けます**(**実装上の注意点**：
  `stopRequest`が実際にいつコントローラを止めるかは呼び出し元
  (`controller_manager`)次第で、この関数内では即座に処理を打ち切らない。
  安全チェックに失敗した周期でも、失敗を検知する直前に計算した
  `posDes`/`velDes`/`torque`がそのままハードウェアへ送信される)

```cpp
robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
selfCollisionVisualization_->update(currentObservation_);
observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
```

- 可視化(RViz向けMarker等、OCS2提供)と、現在の観測値のROSトピック
  publish。コメント「Only needed for the command interface」から、
  この`observationPublisher_`は主に`TargetTrajectoriesPublisher`
  (未読、後章)のような外部ノードが現在状態を知るために使われると
  推測される(**設計上の解釈**)

---

## `LeggedController.cpp` 146〜183行:`updateStateEstimation`

この関数の役割:ハードウェアハンドルから関節・接触・IMUの生値を集め、
状態推定器へ渡して剛体状態を更新し、セントロイダル状態へ変換する。

```cpp
for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
  jointPos(i) = hybridJointHandles_[i].getPosition();
  jointVel(i) = hybridJointHandles_[i].getVelocity();
}
for (size_t i = 0; i < contacts.size(); ++i) {
  contactFlag[i] = contactHandles_[i].isContact();
}
```

- `hybridJointHandles_[i].getPosition()`/`getVelocity()`：
  `HybridJointHandle`が継承する`JointStateHandle`
  ([read_code_02](read_code_02_legged_hw_interface.md))の読み取り側を
  使う。書き込み用の5値(`posDes_`等)とは別の、実測値
- **実装上の注意点**：`for (size_t i = 0; i < contacts.size(); ++i)`の
  `contacts`はこの関数内で`contact_flag_t contacts;`と宣言されただけの
  **未初期化のローカル変数**で、`.size()`は型`contact_flag_t`
  (固定長配列型と推測)のコンパイル時サイズを返すだけの用途。変数名が
  `contactFlag`(実際に値を書き込む側)と紛らわしいが、動作自体に問題は
  ない(**事実**、可読性の問題)

```cpp
stateEstimate_->updateJointStates(jointPos, jointVel);
stateEstimate_->updateContact(contactFlag);
stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
measuredRbdState_ = stateEstimate_->update(time, period);
currentObservation_.time += period.toSec();
scalar_t yawLast = currentObservation_.state(9);
currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
currentObservation_.mode = stateEstimate_->getMode();
```

- `stateEstimate_`(未読、`legged_estimation`)へジョイント・接触・IMUの
  生データを渡し、`update(time, period)`で剛体状態(`measuredRbdState_`)を
  得る
- `currentObservation_.time += period.toSec()`：シミュレーション/実機の
  絶対時刻ではなく、**周期の積算**で時刻を進めている
- `state(9)`をヨー角として扱い、`angles::shortest_angular_distance`で
  前回値からの最短角度差分だけ進める、という**連続ヨー角の積算**
  ([read_code_01](read_code_01_legged_hw_loop.md)で見たGazebo側の
  `angles::shortest_angular_distance`の使い方と同じパターン、
  \(\pm\pi\)をまたぐ不連続を避けるための定番の手法)

---

## `LeggedController.cpp` 185〜198行:デストラクタ

この関数の役割:MPCスレッドを安全に停止させ、MPC・WBCの実行時間統計を
標準エラー出力へ表示する。

```cpp
controllerRunning_ = false;
if (mpcThread_.joinable()) { mpcThread_.join(); }
std::cerr << "### MPC Benchmarking\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
...
```

- コントローラ終了時、MPC・WBCそれぞれの最大・平均実行時間(ミリ秒)を
  表示する。pympc側には無い、実行時間の診断機能

---

## `LeggedController.cpp` 200〜223行:`setupLeggedInterface`・`setupMpc`

この関数の役割(`setupLeggedInterface`):`LeggedInterface`(未読、OCS2向け
問題定義)を構築する。

```cpp
leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
```

この関数の役割(`setupMpc`):OCS2のSQPソルバーを構築し、歩容コマンド受信
(`GaitReceiver`)・目標軌道受信(`RosReferenceManager`)をMPCに接続する。

```cpp
mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
...
auto gaitReceiverPtr = std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
rosReferenceManagerPtr->subscribe(nh);
mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
```

- `SqpMpc`(OCS2、外部)：**Sequential Quadratic Programming**による
  MPCソルバー。pympcのacadosに相当する、実際の求解エンジン(内部は
  **未確認**)
- `GaitReceiver`：[read_code_01](read_code_01_legged_hw_loop.md)で見た
  `load_controller.launch`の`legged_robot_gait_command`ノード(OCS2の
  `ocs2_legged_robot_ros`パッケージ提供、外部)からROS経由で歩容
  切り替えコマンドを受信し、`SwitchedModelReferenceManager`
  (未読、次章以降)の歩容スケジュールへ反映する。pympc側の
  `PeriodicGaitGenerator`が内部変数を直接いじられていたのとは異なり、
  こちらは**ROSトピック経由の非同期コマンド**という設計
- `RosReferenceManager`：同じく`load_controller.launch`の
  `legged_target_trajectories_publisher`ノード
  (`legged_controllers/src/TargetTrajectoriesPublisher.cpp`、未読)から
  目標軌道(速度指令等)を受信する

---

## `LeggedController.cpp` 225〜251行:`setupMrt`

この関数の役割:MPCと現在スレッドを繋ぐMRTインターフェースを構築し、
バックグラウンドでMPCを回し続ける専用スレッドを起動する。

```cpp
mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
mpcTimer_.reset();

controllerRunning_ = true;
mpcThread_ = std::thread([&]() {
  while (controllerRunning_) {
    try {
      executeAndSleep(
          [&]() {
            if (mpcRunning_) {
              mpcTimer_.startTimer();
              mpcMrtInterface_->advanceMpc();
              mpcTimer_.endTimer();
            }
          },
          leggedInterface_->mpcSettings().mpcDesiredFrequency_);
    } catch (const std::exception& e) {
      controllerRunning_ = false;
      ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      stopRequest(ros::Time());
    }
  }
});
setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
```

- `executeAndSleep(...)`(OCS2、外部)：与えたラムダを実行し、
  `mpcDesiredFrequency_`(a1の`task.info`で**`100`Hz**、pympc側の
  `MPC_FREQ=100`と同じ値)に相当する周期になるよう残り時間だけ
  スリープする、というヘルパー関数と考えられる(**設計上の解釈**、
  内部は未確認)
- `mpcRunning_`が`true`になる(=`starting()`が初回解の取得に成功した後)
  まで、このスレッドは`advanceMpc()`を呼ばずに空ループ(`executeAndSleep`
  自体は毎周期呼ばれ続ける)する。`starting()`との協調はこのフラグ1つで
  行われている
- `setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_)`：
  MPCスレッドの優先度を`task.info`の`threadPriority`(確認できた値は
  `50`、[read_code_01](read_code_01_legged_hw_loop.md)で見た
  `LeggedHWLoop`のメイン制御スレッド優先度`95`より**低い**)に設定する。
  「リアルタイム制約の厳しいハードウェア読み書きループを、MPC計算より
  優先する」という意図が読み取れる(**設計上の解釈**)

---

## `LeggedController.cpp` 253〜263行:`setupStateEstimate`・`LeggedCheaterController`

```cpp
void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(...);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(...);
}
```

- `LeggedController`(既定)は`KalmanFilterEstimate`(線形カルマンフィルタ、
  未読)を使う
- `LeggedCheaterController`(`controllers.yaml`に`legged_cheater_controller`
  として別途登録される、[read_code_01](read_code_01_legged_hw_loop.md)の
  `load_controller.launch`の`cheater`引数で切り替え可能)は、実際の状態
  推定を行わず`FromTopicStateEstimate`(未読、ROSトピックから直接状態を
  取得、Gazeboの正解値を想定と推測)を使う。pympcには無い、**状態推定器の
  精度切り分け専用のデバッグ用コントローラ**

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `starting()`の待機ループが`mrtDesiredFrequency_`(`task.info`上
     `; [Hz] Useless`と自己申告)を実際に使っており、設定ファイルの
     コメントと実装が食い違っている
  2. `wbc_`は既定`WeightedWbc`だが、`HierarchicalWbc`もimportされている
     だけで未使用
  3. 安全チェック失敗時、`stopRequest`を呼んでも`update`関数はその場で
     打ち切られず、計算済みの指令がそのまま送信され続ける
  4. `updateStateEstimation`内の`contacts`変数は値を保持しない、サイズ
     取得だけに使われる紛らわしい変数
- 確認できた重要な事実:
  - MPCは`update()`の制御周期(500〜800Hz)とは非同期の専用スレッドで
    既定100Hzで回り続け、`update()`側は最新の解を補間評価するだけ
  - 最終的な関節コマンドは`kp=0`・`kd=3`固定+WBC計算済みトルクの
    フィードフォワードで構成される。位置フィードバック(`kp`)は実質的に
    無効化されている
  - `LeggedCheaterController`という、状態推定を迂回してデバッグする
    専用モードが用意されている(pympcには類似機能なし)
  - 関節順序は`LF→LH→RF→RH`×`HAA→HFE→KFE`で固定
- 次は、`updateStateEstimation`が呼ぶ状態推定器本体
  (`legged_estimation`パッケージ、`StateEstimateBase`・
  `LinearKalmanFilter`・`FromTopicEstimate`)を読みます。
