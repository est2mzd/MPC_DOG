# 速度指令・安全チェック legged_controllers/TargetTrajectoriesPublisher・SafetyChecker 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
[起動] load_controller.launch(read_code_01) の
  legged_target_trajectories_publisher ノード
  → TargetTrajectoriesPublisher.cpp の main()   ← 本ファイル、起動時1回
      → TargetTrajectoriesPublisher コンストラクタ
          → /cmd_vel(geometry_msgs/Twist)を購読
          → /move_base_simple/goal(geometry_msgs/PoseStamped)を購読
          → <robotName>_mpc_observation を購読(LeggedControllerがpublish、read_code_05)

/cmd_vel受信のたびに:
  → cmdVelToTargetTrajectories(...)  ← 本ファイル、外部入力(キーボード/
      ジョイスティック等、対象リポジトリ外)の頻度による
      → TargetTrajectoriesRosPublisher(OCS2、外部)経由でpublish
          → RosReferenceManager(read_code_05のsetupMpc)が受信
              → SwitchedModelReferenceManager(read_code_08)の
                目標軌道として反映される

LeggedController::update(...)(read_code_05)が毎制御周期:
  → safetyChecker_->check(...)   ← 本ファイル、毎制御周期
```

## このファイル/クラスの役割(全体の中での位置づけ)

この章は`legged_controllers`パッケージの残り2つのコンポーネントを扱い、
`external/legged_control`の制御パイプライン本体の読解を完了させます。

- `TargetTrajectoriesPublisher`：**独立したROSノード**として、
  `/cmd_vel`(速度指令、`teleop_twist_keyboard`等の外部ツール想定)や
  `/move_base_simple/goal`(RViz上の「2D Nav Goal」等)を、OCS2が理解する
  `TargetTrajectories`(時刻・状態・入力の目標軌道)へ変換してpublishする。
  pympcでは`QuadrupedEnv`が速度指令を内部に保持し、`console.py`や
  `get_joy_callback`が直接その変数を書き換えていましたが、legged_control
  は**速度指令の生成自体を、コントローラ本体から独立したROSノードに
  切り出している**という設計上の違いがあります
- `SafetyChecker`：`LeggedController::update`が毎周期呼ぶ、**唯一の
  安全チェック**。中身は非常に単純です(後述)

対象は`external/legged_control/legged_controllers/include/legged_controllers/TargetTrajectoriesPublisher.h`
(98行)・`src/TargetTrajectoriesPublisher.cpp`(115行)、
`include/legged_controllers/SafetyChecker.h`(34行)です。

---

## `TargetTrajectoriesPublisher.h` 20〜96行:クラス定義とコールバック登録

```cpp
class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;
  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix,
                              CmdToTargetTrajectories goalToTargetTrajectories, CmdToTargetTrajectories cmdVelToTargetTrajectories)
```

- `goalToTargetTrajectories_`/`cmdVelToTargetTrajectories_`
  (型`std::function<...>`)：目標生成のロジック自体はコンストラクタの
  引数として**関数ポインタ形式で外から注入**される(実体は
  `TargetTrajectoriesPublisher.cpp`側の自由関数、後述)。クラス自体は
  「購読して、注入された変換関数を呼んで、publishする」という骨組み
  だけを持つ、汎用的な設計

```cpp
auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (latestObservation_.time == 0.0) { return; }
  geometry_msgs::PoseStamped pose = *msg;
  buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
  ...
};
goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
```

- `latestObservation_.time == 0.0`(初期値、[read_code_05](read_code_05_legged_controller.md)の
  `setupStateEstimate`で`currentObservation_.time=0`と初期化される
  ことに対応)である間は、目標をまだ一度も受信していない
  (`LeggedController`が起動していない)と判断して無視する、pympc側の
  `first_message_base_arrived`等と同種の安全チェック
- `buffer_.transform(pose, pose, "odom", ros::Duration(0.2))`(TF2、
  外部)：受け取った目標姿勢を`odom`座標系へ変換する。0.2秒以内に変換が
  得られなければ例外を捕捉して無視する

```cpp
auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
  ...
  vector_t cmdVel = vector_t::Zero(4);
  cmdVel[0] = msg->linear.x; cmdVel[1] = msg->linear.y; cmdVel[2] = msg->linear.z; cmdVel[3] = msg->angular.z;
  const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
  targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
};
cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
```

- 標準的なROSの`geometry_msgs/Twist`(`/cmd_vel`)を購読する。**publisher
  側(キーボードテレオペ・ジョイスティック等)は対象リポジトリに無く**、
  `teleop_twist_keyboard`のような外部ROSパッケージや、
  [read_code_04](read_code_04_unitree_hw.md)で見た`/joy`
  (`UnitreeHW::updateJoystick`)を購読して`/cmd_vel`へ変換する別ノード
  (対象リポジトリに無い)を想定していると考えられる(**未確認**)。
  pympc側は速度指令の入力元(キーボード/ジョイスティック)が`env`
  クラス自身に埋め込まれていたのに対し、legged_controlは**標準的な
  ROSナビゲーションスタックの規約(`/cmd_vel`)に準拠**しており、
  他のROSナビゲーションツールと組み合わせやすい設計になっています

---

## `TargetTrajectoriesPublisher.cpp` 21〜49行:目標軌道の共通ロジック

この関数の役割(`estimateTimeToTarget`):目標位置までの並進距離と目標
回頭角から、そこに到達するまでの所要時間を見積もる。

```cpp
const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
return std::max(rotationTime, displacementTime);
```

- `TARGET_ROTATION_VELOCITY`(rad/s、`reference.info`)：a1の実際の値
  **`1.57`**(≈90°/s)
- `TARGET_DISPLACEMENT_VELOCITY`(m/s、`reference.info`)：a1の実際の値
  **`0.5`**
- 並進・回頭のうち**時間のかかる方**を採用する(同時に動く前提で、
  遅い方に合わせる)

この関数の役割(`targetPoseToTargetTrajectories`):目標姿勢1点から、
現在時刻→目標到達時刻の2点だけを持つ`TargetTrajectories`を組み立てる。

```cpp
vector_t currentPose = observation.state.segment<6>(6);
currentPose(2) = COM_HEIGHT;
currentPose(4) = 0;
currentPose(5) = 0;
stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;
```

- `observation.state.segment<6>(6)`：セントロイダル状態の`[6,12)`区間が
  「base位置+ZYX姿勢」であることがここから確認できる(先頭6要素は
  正規化運動量と推測される、**未確認**、`legged_interface`側の状態
  レイアウトの完全な確認はしていない)
- **コードで確認した事実**：目標状態の高さ(`(2)`)は常に`COM_HEIGHT`
  (`reference.info`、a1で**`0.3`**m)固定、ピッチ(`(4)`)・ロール
  (`(5)`)は常に`0`固定です。[read_code_08](read_code_08_switched_model_reference_manager.md)〜
  [read_code_11](read_code_11_precomputation_cost_initializer.md)で
  繰り返し確認した「地形は常に平坦、傾斜への追従機構が無い」という
  legged_control全体の設計方針が、目標生成のこの最も上流の場所にも
  一貫して現れています
- 目標入力(`inputTrajectory`)はコメント「次元を合わせるためだけで、
  実際には使われない」通り全ゼロ。実際の目標入力(接地力)は
  [read_code_11](read_code_11_precomputation_cost_initializer.md)の
  `weightCompensatingInput`が別途計算する

この関数の役割(`cmdVelToTargetTrajectories`):現在の速度指令を、
`TIME_TO_TARGET`(`task.info`の`mpc.timeHorizon`、a1で**`1.0`**秒)秒間
延長した位置・姿勢を目標地点とする2点軌道を作る。

```cpp
vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);
target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
...
trajectories.stateTrajectory[0].head(3) = cmdVelRot;
trajectories.stateTrajectory[1].head(3) = cmdVelRot;
```

- `cmdVel`(base座標系の速度指令)を現在の姿勢(`zyx`)でworld座標系へ
  回転させてから、`timeToTarget`(=**MPCのホライズン長そのもの、`1.0`秒**)
  だけ進めた位置を目標地点とする、**単純な直線外挿(dead reckoning)**
- `trajectories.stateTrajectory[0/1].head(3)`：状態ベクトルの先頭3要素
  (正規化運動量の並進成分と推測される、**未確認**)にも、この
  `cmdVelRot`(world座標系速度)を目標値として埋め込んでいる。位置だけ
  でなく**速度も同時に目標として与える**ことで、MPCが「そこに着くころ
  には指令速度で動いていてほしい」という意図を伝えていると考えられる
  (**設計上の解釈**)

---

## `TargetTrajectoriesPublisher.cpp` 92〜115行:`main`

```cpp
nodeHandle.getParam("/referenceFile", referenceFile);
nodeHandle.getParam("/taskFile", taskFile);
loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);
ros::spin();
```

- `DEFAULT_JOINT_STATE`(rad、`reference.info`の`defaultJointState`)：
  目標状態の関節角度部分は、**MPCの求解結果ではなく、常にこの固定の
  既定関節角**が使われる(スタンス姿勢の基準値、a1の実際の値は
  `LF_HAA=-0.20`等12関節分。`WbcBase`の`swingLegTask`が使う
  MPC側の関節角とは異なる、**目標生成側だけで使う固定値**)
- `main`自体は`ros::spin()`でブロックするだけの、典型的なROSノードの
  構造(read_code_01・[read_code_17〜20](../quadruped_pympc_onboarding/read_code_20_ros2_console.md)
  のROS2版とは異なり、ROS1では`spin()`1行で全コールバックが処理される)

---

## `SafetyChecker.h` 12〜31行:`SafetyChecker`

この関数の役割:MPCの求解結果を使う直前に、姿勢が転倒に近い状態でないか
だけを確認する。

```cpp
bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
  return checkOrientation(observation);
}
bool checkOrientation(const SystemObservation& observation) {
  vector_t pose = getBasePose(observation.state, info_);
  if (pose(5) > M_PI_2 || pose(5) < -M_PI_2) {
    std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
    return false;
  }
  return true;
}
```

**コードで確認した事実(非常に限定的な安全チェック)**：

- `check`の引数`optimized_state`・`optimized_input`(MPCが今回新しく
  計算した最適解)は**どちらも未使用**(コメントアウトされた引数名)。
  実際にチェックされるのは`observation`(状態推定器が出した**現在の
  実測状態**)だけです。「MPCの新しい解が危険かどうか」ではなく
  「現在ロボットがすでに危険な姿勢になっていないか」だけを見ている、
  という点に注意が必要です
- チェック項目は`pose(5)`(ZYX姿勢の3番目=ロール角、
  [read_code_06](read_code_06_legged_estimation.md)の`quatToZyx`の
  戻り値の並びから推測、**設計上の解釈**)が\(\pm90°\)を超えていないか
  だけです。**ピッチ角のチェックは無く**、前後方向に転倒しかけている
  状態は検知できません
- 失敗時は`std::cerr`へメッセージを出すだけで、実際に停止させる処理
  ([read_code_05](read_code_05_legged_controller.md)で見た
  `stopRequest`の呼び出し)は呼び出し元(`LeggedController::update`)の
  責務です
- pympc側には、このような「姿勢が一定角度を超えたら停止」という
  明示的な安全チェックの仕組みは無かった(pympc側の安全策は
  [read_code_05](../quadruped_pympc_onboarding/read_code_05_velocity_modulator.md)
  の脚の伸びきりチェック(同シリーズ外参照になるため直接記載)のような、
  個別の限定的なものはあったが、姿勢全体を見る汎用のチェッカーは
  無かった)、という点でlegged_controlに追加で存在する仕組みです

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `SafetyChecker::check`は引数として渡される「新しいMPC解」を一切
     使わず、既知の実測姿勢だけをチェックする
  2. ロール角のみチェックし、ピッチ角のチェックが無い
- 確認できた重要な事実:
  - 速度指令(`/cmd_vel`)・ナビゲーション目標(`/move_base_simple/goal`)は
    コントローラ本体から独立したROSノードで生成され、標準的なROS
    ナビゲーションの慣習(`/cmd_vel`)に準拠している
  - 目標姿勢の高さ・ピッチ・ロールは常に固定値(`COM_HEIGHT`・`0`・`0`)。
    legged_control全体を通じて一貫している「地形は常に平坦」という
    設計方針が、目標生成の最上流にも現れている
  - 目標関節角度はMPCの計算結果ではなく、`reference.info`の
    `defaultJointState`という固定値が使われる
- これで、`external/legged_control`の主要な制御パイプライン
  (read_code_01〜14:実行ループ→ハードウェア抽象化→具体HW(実機/Gazebo)→
  コントローラ本体→状態推定→OCS2問題定義→歩容/スイング計画→接触制約→
  事前計算/コスト/初期化→WBCタスク定式化→WBC QP求解→速度指令/安全
  チェック)を一通り読み終えました。
