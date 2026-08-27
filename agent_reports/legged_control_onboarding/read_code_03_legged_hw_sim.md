# Gazeboシミュレーション側HW legged_gazebo/LeggedHWSim 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
[起動] roslaunch legged_unitree_description empty_world.launch
  → gazebo_ros_control(外部パッケージ)の GazeboRosControlPlugin が
    URDFの<robotSimType>指定に従い LeggedHWSim をプラグインとしてロード
      → LeggedHWSim::initSim(...)     ← 本ファイル、起動時1回

Gazeboの物理ステップイベント(内部、gazebo_ros_controlが購読)ごとに:
  → LeggedHWSim::readSim(...)         ← 本ファイル、毎物理ステップ
  → controllerManager_->update(...)   (gazebo_ros_control内部、外部、未確認)
  → LeggedHWSim::writeSim(...)        ← 本ファイル、毎物理ステップ
```

**事実**：read_code_01で確認した通り、`LeggedHWLoop`(独自スレッドで
`read`/`update`/`write`を回す仕組み)は実機経路専用であり、Gazebo経路では
使われません。かわりに`gazebo_ros_control`(外部パッケージ)が、Gazebo
本体の物理更新イベントに同期して同じ3ステップ(read→update→write)を
呼び出します。呼び出し頻度はGazeboの物理エンジンのタイムステップに従うため、
`loop_frequency`(read_code_01で見た実機側のyaml設定)とは**別の設定値**
(Gazebo world側のタイムステップ、対象リポジトリの`worlds/empty_world.world`
に依存、**未確認**)で決まります。

## このファイル/クラスの役割(全体の中での位置づけ)

`LeggedHWSim`が担当するのは、「**Gazeboの物理エンジンから関節・IMU・接触
センサの値を読み取り、ros_controlの4インターフェース(read_code_02)へ
反映し、逆にros_controlから来たハイブリッド指令をGazeboの関節へ適用する**」
ことです。pympc(Quadruped-PyMPC)における`run_simulator.py`の`Simulator_Node`
と同じ役割(物理シミュレーションと制御ロジックの橋渡し)を担いますが、
pympc側がPythonの`env.step`を明示的に1行呼んでいたのに対し、こちらは
Gazebo自体が物理演算を行い、このクラスは「Gazeboの内部状態⇄ros_control
インターフェース」の**変換層**に徹しています(物理演算そのものは持たない)。

対象は`external/legged_control/legged_gazebo/include/legged_gazebo/LeggedHWSim.h`
(97行)・`external/legged_control/legged_gazebo/src/LeggedHWSim.cpp`(238行)
です。基底クラス`gazebo_ros_control::DefaultRobotHWSim`(外部パッケージ)の
内部実装は対象外・未確認です。

---

## `LeggedHWSim.h` 51〜93行:データ構造とメンバ変数

```cpp
struct HybridJointData {
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};
struct HybridJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};
```

- `HybridJointData`：1関節あたりの現在のハイブリッド指令値
  (`HybridJointHandle`のポインタが指す実体)と、Gazebo側の関節ハンドル
  (`hardware_interface::JointHandle`、ros_control標準)を1つにまとめた構造体
- `HybridJointCommand`：`HybridJointData`と同じ5値に、タイムスタンプ
  `stamp_`(型`ros::Time`)を加えたもの。後述する遅延バッファで使う

```cpp
std::list<HybridJointData> hybridJointDatas_;
std::list<ImuData> imuDatas_;
std::unordered_map<std::string, std::deque<HybridJointCommand> > cmdBuffer_;
std::unordered_map<std::string, bool> name2contact_;
double delay_{};
```

- `cmdBuffer_`(型`std::unordered_map<関節名, std::deque<HybridJointCommand>>`)：
  関節ごとの指令履歴キュー。後述する通信遅延シミュレーションに使う
- `name2contact_`(型`std::unordered_map<接触リンク名, bool>`)：
  接触リンク名→接触中かどうかのマップ
- `delay_`(型`double`、秒)：関節指令の反映遅延時間。`gazebo/delay`
  パラメータから読み込み、無ければ`0.`。**実際の値は
  `legged_gazebo/config/default.yaml`で`0.009`秒**(`grep`で確認済み)

---

## `LeggedHWSim.cpp` 42〜75行:`initSim`

この関数の役割:基底クラスの初期化を呼んだ上で、ハイブリッド関節・IMU・
接触センサの各インターフェースをGazeboの実体と結びつけて登録する。

```cpp
bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
registerInterface(&hybridJointInterface_);
std::vector<std::string> names = ej_interface_.getNames();
for (const auto& name : names) {
  hybridJointDatas_.push_back(HybridJointData{.joint_ = ej_interface_.getHandle(name)});
  HybridJointData& back = hybridJointDatas_.back();
  hybridJointInterface_.registerHandle(HybridJointHandle(back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
  cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<HybridJointCommand>()));
}
```

- `ej_interface_`(基底`DefaultRobotHWSim`のメンバ、`EffortJointInterface`、
  外部)：基底クラスがGazeboの各関節に対してすでに用意している
  「エフォート(トルク)コマンド用ハンドル」をそのまま流用し、
  `HybridJointHandle`のコンストラクタへ渡す最初の引数
  (`JointStateHandle`、read_code_02参照)として再利用している。つまり
  読み取り(位置・速度・力)は基底クラスの仕組みに乗っかり、書き込み側だけ
  独自の5値ハイブリッド形式を追加している、という設計
- 関節ごとに`cmdBuffer_`へ空のキューを1つ用意しておく

```cpp
registerInterface(&imuSensorInterface_);
XmlRpc::XmlRpcValue xmlRpcValue;
if (!model_nh.getParam("gazebo/imus", xmlRpcValue)) {
  ROS_WARN("No imu specified");
} else {
  parseImu(xmlRpcValue, parent_model);
}
if (!model_nh.getParam("gazebo/delay", delay_)) {
  delay_ = 0.;
}
if (!model_nh.getParam("gazebo/contacts", xmlRpcValue)) {
  ROS_WARN("No contacts specified");
} else {
  parseContacts(xmlRpcValue);
}
```

- `gazebo/imus`・`gazebo/contacts`(rosparam)：`legged_gazebo/config/default.yaml`
  から読み込まれる。実際の設定は`imus.base_imu`(frame_id `base_imu`と
  各種共分散)、`contacts: ["LF_FOOT","LH_FOOT","RF_FOOT","RH_FOOT"]`
  (**事実**、脚の命名がpympc側の`FL/FR/RL/RR`ではなく`LF/LH/RF/RH`
  (Left-Front/Left-Hind/Right-Front/Right-Hind)である点に注意)

```cpp
contactManager_ = parent_model->GetWorld()->Physics()->GetContactManager();
contactManager_->SetNeverDropContacts(true);
```

- Gazebo物理エンジン(外部)の接触マネージャを取得し、「接触情報を
  絶対に破棄しない」設定にする。コードコメントより、これをしないと
  Gazebo GUIで`view→contacts`を有効にしないと`GetContacts()`が空を
  返すことがある、という既知の挙動を回避するための設定と分かる

---

## `LeggedHWSim.cpp` 77〜147行:`readSim`

この関数の役割:Gazeboの関節・IMU・接触状態を読み取り、ros_control側の
状態値へ反映する。あわせて、コントローラ未ロード時の安全な既定指令も
ここで設定する。

### 79〜92行:関節状態(基底クラスの実装を上書き)

```cpp
for (unsigned int j = 0; j < n_dof_; j++) {
  double position = sim_joints_[j]->Position(0);
  joint_velocity_[j] = (position - joint_position_[j]) / period.toSec();
  if (time == ros::Time(period.toSec())) {
    joint_velocity_[j] = 0;
  }
  if (joint_types_[j] == urdf::Joint::PRISMATIC) {
    joint_position_[j] = position;
  } else {
    joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
  }
  joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
}
```

- コード冒頭のコメント「`DefaultRobotHWSim::readSim`はバイアスのかかった
  関節速度を提供する」より、**基底クラスの速度計算をあえて使わず**、
  自前で位置の差分から速度を計算し直していることが分かる
- `joint_velocity_[j]`(rad/s)：`(今回の位置 - 前回保持していた位置) / period`
  という単純な後退差分。`time == ros::Time(period.toSec())`(シミュレーション
  開始直後の最初のステップ)のときだけ速度を`0`にリセットする
  (差分元が無意味な初期値になるのを避けるため、**設計上の解釈**)
- 回転関節では`angles::shortest_angular_distance`(外部、`angles`
  パッケージ)を使い、`joint_position_[j]`へ**最短角度差を積算**していく。
  単純代入ではなく積算にすることで、関節角度が`±π`をまたいで連続回転する
  ケースでの角度の巻き戻り(ラップアラウンド)を回避していると考えられる
  (**設計上の解釈**)
- `joint_effort_[j]`(N·m)：Gazebo側の関節力を直接取得

### 94〜131行:IMUと接触センサの読み取り

```cpp
ignition::math::Vector3d gravity = {0., 0., -9.81};
ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
```

- IMUの線形加速度は、Gazeboの相対加速度から重力加速度分
  (`-9.81`、リンク座標系へ回転させたもの)を**差し引いて**いる。実際の
  IMUセンサは重力を含めた加速度(静止時に鉛直方向に約9.81 m/s²を検出する)
  を出力するのが一般的だが、ここでは重力を除いた値になっている
  (**実装上の注意点、要検証**：一般的なIMUの仕様と逆になっている可能性が
  あり、この値を消費する`legged_estimation`側(未読)が重力込みを前提に
  しているとズレる恐れがある。コード中に意図の説明コメントは無く、
  正誤の判断は`legged_estimation`を読んでから改めて行う)

```cpp
for (const auto& contact : contactManager_->GetContacts()) {
  if (static_cast<uint32_t>(contact->time.sec) != (time - period).sec ||
      static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec) {
    continue;
  }
  ...
}
```

- 接触判定は、Gazeboが記録した全接触イベントの中から、**タイムスタンプが
  ちょうど`time - period`(1つ前の物理ステップの時刻)と一致するものだけ**
  を採用する。`time`(今回の`readSim`呼び出し時刻)そのものではなく1周期
  前の時刻を見ているのは、Gazeboの内部で物理ステップの接触計算が
  `readSim`呼び出しの前の周期分だけ完了している、というタイミングの
  ズレを補正するためと考えられる(**設計上の解釈**、Gazeboのコールバック
  順序の詳細は**未確認**)

### 133〜146行:安全な既定指令へのリセット

```cpp
// Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
for (auto& cmd : joint_effort_command_) { cmd = 0; }
for (auto& cmd : joint_velocity_command_) { cmd = 0; }
for (auto& joint : hybridJointDatas_) {
  joint.posDes_ = joint.joint_.getPosition();
  joint.velDes_ = joint.joint_.getVelocity();
  joint.kp_ = 0.;
  joint.kd_ = 0.;
  joint.ff_ = 0.;
}
```

- **コードで確認した事実(安全設計)**：`readSim`(=`read`→`update`→`write`
  サイクルの最初)は、**毎回**すべての関節のハイブリッド指令を
  「現在位置・現在速度を目標にし、ゲイン・フィードフォワードは全部ゼロ」
  にリセットしてから始めます。コメント通り、これは「コントローラが
  ロードされていない(=`writeSim`で新しい値が書き込まれない)ときに、
  ソフトリミット(関節可動域端の反発力等)が暴走的に振動するのを防ぐ」
  ための安全策です。もしこの周期で`controllerManager_->update()`が
  `legged/LeggedController`を実際に動かせば、この直後の`writeSim`までの
  間に新しい値で上書きされます

---

## `LeggedHWSim.cpp` 149〜167行:`writeSim`

この関数の役割:ハイブリッド指令を通信遅延バッファへ通し、遅延後の値から
実際のトルクを計算してGazeboの関節へ適用する。

```cpp
for (auto joint : hybridJointDatas_) {
  auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
  if (time == ros::Time(period.toSec())) {  // Simulation reset
    buffer.clear();
  }
  while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
    buffer.pop_back();
  }
  buffer.push_front(HybridJointCommand{.stamp_ = time, .posDes_ = joint.posDes_, ...});

  const auto& cmd = buffer.back();
  joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) + cmd.ff_);
}
DefaultRobotHWSim::writeSim(time, period);
```

- **コードで確認した事実(通信遅延シミュレーション)**：`cmdBuffer_`は
  関節ごとのFIFOキューで、新しい指令を`push_front`し、
  `stamp_ + delay_ < time`(=すでに遅延時間を超えて古くなった)指令だけを
  `pop_back`で捨てる。その上で**`buffer.back()`(まだ捨てられていない
  最古の指令)を実際に適用する**という実装です。これにより、
  「今計算した指令が、`delay_`秒後にようやくハードウェアへ反映される」
  という通信・処理遅延を意図的にシミュレートしています。
  **既定値`delay_=0.009`秒(約9ミリ秒)**。pympc(Quadruped-PyMPC)の
  MuJoCoシミュレーション(`simulation.py`/ROS2版とも)には、このような
  意図的な指令遅延の仕組みはありませんでした
- 適用する式は[read_code_02](read_code_02_legged_hw_interface.md)で
  先取りした通り
  \(\tau=k_p(p_{des}-p)+k_d(v_{des}-v)+\tau_{ff}\)
  で、`joint.joint_.setCommand(...)`によりGazeboの`EffortJointInterface`
  (基底クラス由来)へ最終トルクとして渡される
- `DefaultRobotHWSim::writeSim(...)`(基底クラス)を最後に呼び、他の
  (このクラスが上書きしていない)標準的な書き込み処理を委譲する

---

## `LeggedHWSim.cpp` 169〜232行:`parseImu`・`parseContacts`

この関数の役割(`parseImu`):rosparamのIMU設定をバリデーションしつつ読み、
Gazeboのリンクと結びつけたIMUハンドルを登録する。

- `frame_id`・`orientation_covariance_diagonal`(3要素)・
  `angular_velocity_covariance`(3要素)・`linear_acceleration_covariance`
  (3要素)の4項目が無いと`ROS_ERROR_STREAM`を出して**そのIMUの登録だけ
  スキップ**(`continue`)する。実際の値(`legged_gazebo/config/default.yaml`)
  は`orientation_covariance_diagonal=[0.0012,0.0012,0.0012]`・
  `angular_velocity_covariance=[0.0004,0.0004,0.0004]`・
  `linear_acceleration_covariance=[0.01,0.01,0.01]`(無次元の対角共分散、
  単位はそれぞれ対応するセンサ値の単位の2乗)
- 共分散の対角成分だけを3×3行列の対角要素へ詰め、非対角要素は`0`固定
  (センサノイズ間の相関は考慮しない設計)

この関数の役割(`parseContacts`):rosparamの接触リンク名リストを読み、
`name2contact_`マップと`ContactSensorHandle`を関節名ごとに登録する。

- `contactNames`(型`XmlRpc::XmlRpcValue`の配列)：`["LF_FOOT","LH_FOOT",
  "RF_FOOT","RH_FOOT"]`(a1等、実際の値)を1つずつ`name2contact_`へ
  `false`で初期登録し、`ContactSensorHandle(name, &name2contact_[name])`
  として`contactSensorInterface_`へ登録する

---

## `LeggedHWSim.cpp` 236〜237行:プラグイン登録マクロ

```cpp
PLUGINLIB_EXPORT_CLASS(legged::LeggedHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)
```

- `PLUGINLIB_EXPORT_CLASS`：`LeggedHWSim`を`gazebo_ros_control::RobotHWSim`
  インターフェースの実装として、pluginlib(ROSの動的プラグインロード機構、
  外部)に登録する
- `GZ_REGISTER_MODEL_PLUGIN`：**このリポジトリ独自のプラグインではなく、
  `gazebo_ros_control`パッケージが提供する汎用プラグイン
  `GazeboRosControlPlugin`をGazeboのモデルプラグインとして登録している**
  (`legged_hw_sim_plugins.xml`で見た`base_class_type="gazebo_ros_control::RobotHWSim"`
  という指定と合わせて理解すると、実際にロボットXacro/URDFの`<gazebo>`
  タグ側で`GazeboRosControlPlugin`をロードし、そのプラグインが今度は
  pluginlib経由で`legged_gazebo/LeggedHWSim`という名前の`RobotHWSim`
  実装(=このクラス)を選んでロードする、という**二段階のプラグイン
  ロード構造**になっている(**設計上の解釈**、Xacro側の該当タグは
  **未確認**)

---

## この章のまとめ

- 見つかった実装上の注意点(要検証):
  1. `readSim`のIMU線形加速度計算が、Gazeboの相対加速度から重力加速度を
     **差し引いて**おり、一般的なIMUの「静止時に重力分を検出する」仕様と
     逆向きになっている可能性がある(`legged_estimation`未読のため、
     消費側の前提と整合するかは**未確認**)
- 確認できた重要な事実:
  - `LeggedHWSim`は「コントローラ未ロード時は毎周期、現在値保持+
    ゲインゼロにリセットする」という安全な既定指令を`readSim`の末尾で
    必ず設定している
  - `writeSim`は`delay_`(既定`0.009`秒)による**通信遅延のシミュレーション**
    をFIFOバッファで実装しており、計算した指令がすぐには反映されない。
    pympc側のMuJoCoシミュレーションには無かった仕組み
  - 接触リンクの命名は`LF/LH/RF/RH`(pympc側の`FL/FR/RL/RR`とは異なる
    命名規則)
  - 実際のトルク計算式は`τ = kp*(posDes-p) + kd*(velDes-v) + ff`
    (read_code_02のハイブリッドインターフェースの実体)
- 次は、実機側のハードウェア実装`legged_unitree_hw/UnitreeHW.cpp`
  (pympcには存在しない、実ロボットSDKとの接続部分)を読みます。
