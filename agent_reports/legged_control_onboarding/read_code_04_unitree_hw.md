# 実機ハードウェア legged_unitree_hw/UnitreeHW 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
legged_unitree_hw.cpp の main() (read_code_01)
  → UnitreeHW::init(nh, robotHwNh)        ← 本ファイル、起動時1回
      → LeggedHW::init(...)(read_code_02、URDF読込・4IF登録)
      → setupJoints/setupImu/setupContactSensor  ← 本ファイル、起動時1回
      → UDP接続の確立(Unitree SDK、外部)

LeggedHWLoop::update()(read_code_01)が毎周期呼ぶ:
  → UnitreeHW::read(...)   ← 本ファイル、毎制御周期(既定500Hz/800Hz)
  → controllerManager_->update(...)
  → UnitreeHW::write(...)  ← 本ファイル、毎制御周期
```

## このファイル/クラスの役割(全体の中での位置づけ)

`UnitreeHW`が担当するのは、「**Unitree社製ロボット(A1/Go1/Aliengo)の低レベル
UDP通信SDK(`UNITREE_LEGGED_SDK`、外部、CANバス経由でモータードライバと通信)
を、`LeggedHW`(read_code_02)が定義した4種類のros_controlインターフェースへ
橋渡しする**」ことです。pympcには存在しない、**実ロボットとの通信を担う唯一の
経路**であり、[read_code_03](read_code_03_legged_hw_sim.md)の`LeggedHWSim`
(Gazebo側)と対になる、`LeggedHW`の**もう1つの具体的な実装**です。

- 制御ロジック(状態推定・MPC・WBC)は一切持ちません
- Unitree SDK自体(`UNITREE_LEGGED_SDK::UDP`・`Safety`)の内部実装は
  対象リポジトリの外(`legged_unitree_hw/include/unitree_legged_sdk_*`に
  ヘッダのみ同梱)であり、シグネチャ・使い方までは確認できるが内部動作は
  **未確認**として扱う

対象は
`external/legged_control/legged_examples/legged_unitree/legged_unitree_hw/include/legged_unitree_hw/UnitreeHW.h`
(97行)・同ディレクトリの`src/UnitreeHW.cpp`(209行)です。

---

## `UnitreeHW.h` 19〜33行:定数とデータ構造

```cpp
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};
```

- `CONTACT_SENSOR_NAMES`：接触センサ(実際には後述の通り力覚センサ閾値判定)
  の名前リスト。**`RF/LF/RH/LH`という命名**(Right-Front/Left-Front/
  Right-Hind/Left-Hind)で、[read_code_03](read_code_03_legged_hw_sim.md)の
  Gazebo側`default.yaml`の`contacts`リスト(`LF/LH/RF/RH`、順序は違うが同じ
  4文字の命名規則)と**同じ命名規則**を使っている。pympc側の`FL/FR/RL/RR`
  とは異なる、このリポジトリ独自の命名
- `UnitreeMotorData`：1関節あたりの状態(`pos_`/`vel_`/`tau_`)とコマンド
  (`posDes_`/`velDes_`/`kp_`/`kd_`/`ff_`)を1つにまとめた構造体。コマンド側
  5値は[read_code_02](read_code_02_legged_hw_interface.md)のハイブリッド
  インターフェースと同じ構成

---

## `UnitreeHW.cpp` 18〜58行:`init`

この関数の役割:URDF読み込み(基底クラス)の後、関節・IMU・接触センサを
セットアップし、UDP通信とロボット種別ごとの安全機構を初期化する。

```cpp
if (!LeggedHW::init(root_nh, robot_hw_nh)) { return false; }
robot_hw_nh.getParam("power_limit", powerLimit_);
setupJoints();
setupImu();
setupContactSensor(robot_hw_nh);

#ifdef UNITREE_SDK_3_3_1
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
#elif UNITREE_SDK_3_8_0
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
#endif
udp_->InitCmdData(lowCmd_);
```

- `powerLimit_`(型`int`、単位**未確認**、コメントや設定名から電流/電力の
  上限と推測)：rosparam`power_limit`。実際の値(a1/go1:`4`、aliengo:`7`、
  [read_code_01](read_code_01_legged_hw_loop.md)で確認済みの
  `<robot>.yaml`と同じファイル)
- `UNITREE_SDK_3_3_1`/`UNITREE_SDK_3_8_0`：コンパイル時マクロによる
  SDKバージョンの切り替え(A1/Aliengoは旧SDK、Go1は新SDKを使うと後段の
  ロボット種別分岐から分かる)
- **実装上の注意点**：`UNITREE_SDK_3_8_0`側のUDP接続先IPアドレス
  `"192.168.123.10"`とポート`8090`/`8007`が**ソースコードに直接
  ハードコード**されている。rosparam化されておらず、異なるネットワーク
  構成のロボットを使う場合はソースコードの書き換え・再ビルドが必要になる

```cpp
std::string robot_type;
root_nh.getParam("robot_type", robot_type);
#ifdef UNITREE_SDK_3_3_1
  if (robot_type == "a1") { safety_ = ...A1...; }
  else if (robot_type == "aliengo") { safety_ = ...Aliengo...; }
#elif UNITREE_SDK_3_8_0
  if (robot_type == "go1") { safety_ = ...Go1...; }
#endif
else {
  ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
  return false;
}
```

- プリプロセッサの`#ifdef`/`#elif`と、通常の実行時`if`/`else if`/`else`が
  組み合わさっている。コンパイル時にどちらか一方の`if`チェーンだけが
  実体化され、末尾の`else`(未知のロボット種別を`ROS_FATAL`で拒否する)は
  どちらのコンパイル結果にも共通して残る、という構造(**事実**、
  一見わかりにくいが正しく動作するC++の書き方)
- `safety_`(型`std::shared_ptr<UNITREE_LEGGED_SDK::Safety>`)：ロボット
  種別ごとの安全機構(SDK提供、後述の`write`で使う)

```cpp
joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(std::string("/contact"), 10);
```

- ワイヤレスリモコン(ジョイスティック)の入力と、生の足力センサ値を、
  それぞれROSトピック`/joy`・`/contact`として配信する準備。実際の配信は
  後述の`updateJoystick`/`updateContact`

---

## `UnitreeHW.cpp` 60〜96行:`read`

この関数の役割:UDP経由でモーター・IMU・足力の最新状態を取得し、
ros_controlインターフェースへ反映する。

```cpp
udp_->Recv();
udp_->GetRecv(lowState_);

for (int i = 0; i < 12; ++i) {
  jointData_[i].pos_ = lowState_.motorState[i].q;
  jointData_[i].vel_ = lowState_.motorState[i].dq;
  jointData_[i].tau_ = lowState_.motorState[i].tauEst;
}
```

- `udp_->Recv()`/`GetRecv(lowState_)`(SDK、外部)：UDPで届いた最新の
  ロボット状態パケットを`lowState_`(型`UNITREE_LEGGED_SDK::LowState`)へ
  取得する
- `jointData_[i].pos_`(rad)/`vel_`(rad/s)/`tau_`(N·m、`tauEst`=推定トルク)：
  12関節分をそのままコピー

```cpp
imuData_.ori_[0] = lowState_.imu.quaternion[1];
imuData_.ori_[1] = lowState_.imu.quaternion[2];
imuData_.ori_[2] = lowState_.imu.quaternion[3];
imuData_.ori_[3] = lowState_.imu.quaternion[0];
```

- SDKの`quaternion`は`[w,x,y,z]`順、`imuData_.ori_`は`[x,y,z,w]`順へ
  並べ替えている(**事実**、pympc側ROS2経路の`run_simulator.py`が行って
  いたのと同種のクォータニオン順序変換。ros_control標準の
  `ImuSensorHandle`が`[x,y,z,w]`順を要求するためと考えられる、
  **設計上の解釈**)

```cpp
for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
  contactState_[i] = lowState_.footForce[i] > contactThreshold_;
}
```

- `lowState_.footForce[i]`(単位**未確認**、SDK依存、力覚センサの生値と
  推測)が`contactThreshold_`(既定`40`、全ロボット共通、
  [read_code_01](read_code_01_legged_hw_loop.md)で確認済み)を超えたら
  接触と判定する**閾値ベースの接触検知**。Gazebo側
  ([read_code_03](read_code_03_legged_hw_sim.md))がGazebo物理エンジンの
  実接触イベントを使っていたのとは全く異なる、実機ならではのセンサ推定
  方式

```cpp
// Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
std::vector<std::string> names = hybridJointInterface_.getNames();
for (const auto& name : names) {
  HybridJointHandle handle = hybridJointInterface_.getHandle(name);
  handle.setFeedforward(0.);
  handle.setVelocityDesired(0.);
  handle.setKd(3.);
}
```

**実装上の問題点(Gazebo側との安全策の非対称性)**：
[read_code_03](read_code_03_legged_hw_sim.md)の`LeggedHWSim::readSim`は
`posDes_`・`velDes_`・`kp_`・`kd_`・`ff_`の**5値すべて**を安全な既定値
(現在位置・現在速度・ゲインゼロ)にリセットしていました。こちらの実機側
`read`は、**`ff_`(ゼロ)・`velDes_`(ゼロ)・`kd_`(`3.0`固定)の3つしか
リセットしません**。`posDes_`と`kp_`は**前回の値のまま保持**されます。
コントローラが正常に動いている間は`controllerManager_->update()`が
毎周期これらを新しい値で上書きするため実害はありませんが、もし
コントローラが異常終了・アンロードされた状態が続くと、**古い
`posDes_`(すでに実現された、あるいは危険な目標角度かもしれない)と
古い`kp_`(ゼロでない可能性がある)がそのまま使われ続け、`kd_=3.0`
という減衰項だけが効いた状態でモーターへ送信され続ける**ことになります。
Gazebo側は「コントローラ無し=現在位置保持・ゲインゼロ(完全に脱力に近い)」
という明確な安全側既定値でしたが、実機側は「コントローラ無し=古い位置
目標へ向けて、古いKp・固定Kd=3.0で追従し続けようとする」という、
より危険な可能性のある挙動になっています(**推測**、実際の被害の有無は
`safety_->PositionLimit`/`PowerProtect`(後述の`write`)がどこまで
緩和するか次第で、コード上の論理からの指摘であり実機での検証はしていない)。

```cpp
updateJoystick(time);
updateContact(time);
```

- ワイヤレスリモコン入力と足力センサ値を、それぞれ50Hzに間引いてROSへ
  publishする(詳細は後述)

---

## `UnitreeHW.cpp` 98〜110行:`write`

この関数の役割:ハイブリッド指令を安全機構でクリップし、UDP経由でモーター
コマンドとして送信する。

```cpp
for (int i = 0; i < 12; ++i) {
  lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
  lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
  lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
  lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
  lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
}
safety_->PositionLimit(lowCmd_);
safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
udp_->SetSend(lowCmd_);
udp_->Send();
```

- `jointData_[i]`の5値(`double`)を`float`にキャストして`lowCmd_`
  (SDK型)へコピーする。Gazebo側([read_code_03](read_code_03_legged_hw_sim.md))
  がホスト側(`writeSim`)で`τ=kp*(posDes-p)+kd*(velDes-v)+ff`を**計算して
  from トルクとして送信**していたのに対し、こちらは**5値をそのまま
  モーター側(SDK/ドライバのファームウェア)へ送り、実際のPD計算は
  モーター側で行われる**と考えられる(**設計上の解釈**、`lowCmd_`の
  各フィールド名`q`/`dq`/`Kp`/`Kd`/`tau`から妥当な推測だが、モーター
  ファームウェア側の実装は対象リポジトリの外で**未確認**)
- `safety_->PositionLimit(lowCmd_)`(SDK、外部)：関節角度指令を可動域内へ
  クリップすると推測される(**未確認**、関数名からの推測)
- `safety_->PowerProtect(lowCmd_, lowState_, powerLimit_)`(SDK、外部)：
  `powerLimit_`(既定`4`または`7`)を超えないようトルク指令を制限すると
  推測される(**未確認**)。これがpympc側で言う「MPC計算後のトルク
  クリップ」に相当する、**最終防衛ライン**の安全機構

---

## `UnitreeHW.cpp` 112〜169行:`setupJoints`・`setupImu`・`setupContactSensor`

この関数の役割(`setupJoints`):URDFの関節名文字列から脚・関節種別を判定し、
Unitree SDKの配列インデックスへ対応づけてハンドルを登録する。

```cpp
for (const auto& joint : urdfModel_->joints_) {
  if (joint.first.find("RF") != std::string::npos) { leg_index = UNITREE_LEGGED_SDK::FR_; }
  else if (joint.first.find("LF") != std::string::npos) { leg_index = UNITREE_LEGGED_SDK::FL_; }
  else if (joint.first.find("RH") != std::string::npos) { leg_index = UNITREE_LEGGED_SDK::RR_; }
  else if (joint.first.find("LH") != std::string::npos) { leg_index = UNITREE_LEGGED_SDK::RL_; }
  else { continue; }

  if (joint.first.find("HAA") != std::string::npos) { joint_index = 0; }
  else if (joint.first.find("HFE") != std::string::npos) { joint_index = 1; }
  else if (joint.first.find("KFE") != std::string::npos) { joint_index = 2; }
  else { continue; }

  int index = leg_index * 3 + joint_index;
  ...
}
```

**事実(3つ目の命名規則の存在)**：URDFの関節名は`RF/LF/RH/LH`
(Right-Front等、[read_code_03](read_code_03_legged_hw_sim.md)の接触名と
同じ)を使うが、Unitree SDK自体の配列インデックス定数は`FR_/FL_/RR_/RL_`
(Front-Right等、Front/Rear基準)という**別の命名**を使っている。この関数は
両者を対応づける変換表の役割を果たしている:

| URDF側(このリポジトリ) | SDK側インデックス |
|---|---|
| `RF`(Right-Front) | `FR_` |
| `LF`(Left-Front) | `FL_` |
| `RH`(Right-Hind) | `RR_`(Rear-Right) |
| `LH`(Left-Hind) | `RL_`(Rear-Left) |

- `HAA`/`HFE`/`KFE`：Hip Abduction-Adduction(股関節の開閉)、
  Hip Flexion-Extension(股関節の前後)、Knee Flexion-Extension(膝の
  屈伸)という、脚ロボット分野で一般的な関節種別の略称
- `index = leg_index * 3 + joint_index`：脚番号×3+関節番号で、
  SDKの`motorCmd`/`motorState`配列(12要素)上の位置を決める
- どちらのパターンにも一致しない関節(足首固定関節、URDF上の仮想関節等)は
  `continue`で無視される
- `hardware_interface::JointStateHandle`を作った上で、それを使って
  `HybridJointHandle`を作る、という2段階の登録は
  [read_code_03](read_code_03_legged_hw_sim.md)のGazebo側
  (`ej_interface_.getHandle`を再利用)と対応する構造

この関数の役割(`setupImu`)：IMUハンドルを登録し、共分散の対角成分に
固定値を設定する。

```cpp
imuData_.oriCov_[0] = 0.0012; imuData_.oriCov_[4] = 0.0012; imuData_.oriCov_[8] = 0.0012;
imuData_.angularVelCov_[0] = 0.0004; imuData_.angularVelCov_[4] = 0.0004; imuData_.angularVelCov_[8] = 0.0004;
```

- **実装上の注意点(設定値の重複)**：この共分散値(`0.0012`、`0.0004`)は、
  [read_code_03](read_code_03_legged_hw_sim.md)で見た
  `legged_gazebo/config/default.yaml`の`orientation_covariance_diagonal`・
  `angular_velocity_covariance`と**まったく同じ数値**ですが、Gazebo側は
  rosparamから読み込むのに対し、こちらはソースコードへ**直接ハードコード**
  されています。片方だけ値を変更すると2つの経路で共分散が食い違う
  (`linear_acceleration_covariance`に相当する設定はこの関数には無く、
  `UnitreeMotorData`同様に未設定=ゼロ初期化のままと考えられる、**未確認**)

この関数の役割(`setupContactSensor`)：接触閾値をrosparamから読み、
4本の脚の接触センサハンドルを登録する。

- `contactThreshold_`：rosparam`contact_threshold`(既定`40`、全ロボット
  共通)

---

## `UnitreeHW.cpp` 171〜207行:`updateJoystick`・`updateContact`

この関数の役割(`updateJoystick`)：ワイヤレスリモコンの生データを
`sensor_msgs/Joy`形式に変換し、50Hzで`/joy`へpublishする。

```cpp
if ((time - lastJoyPub_).toSec() < 1 / 50.) { return; }
...
xRockerBtnDataStruct keyData;
memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
sensor_msgs::Joy joyMsg;
joyMsg.axes.push_back(-keyData.lx);
joyMsg.axes.push_back(keyData.ly);
joyMsg.axes.push_back(-keyData.rx);
joyMsg.axes.push_back(keyData.ry);
...
```

- `lowState_.wirelessRemote`(SDKの生バイト列)を`memcpy`で
  `xRockerBtnDataStruct`(SDKヘッダ定義の構造体)へ直接型変換している
  (**事実**、Cスタイルの生メモリコピー、型安全性のチェックはSDK側にも
  このコードにも無い)
- `lx`/`rx`軸の符号を反転(`-keyData.lx`)：コメント「Pack as same as
  Logitech F710」より、Unitreeのリモコンの軸方向をLogitech F710
  ゲームパッドの軸方向に合わせるための符号調整と分かる
- 更新頻度は制御ループ本体(500〜800Hz)よりずっと低い**50Hz固定**
  (マジックナンバー、`1/50.`という同じ値が`updateContact`にも重複して
  書かれている)

この関数の役割(`updateContact`)：4脚分の生の足力センサ値を50Hzで
`/contact`へpublishする(デバッグ・監視用と考えられる)。

- `contactMsg.data`：`lowState_.footForce[i]`(整数、単位**未確認**)を
  そのまま4脚分並べる

---

## この章のまとめ

- 見つかった実装上の問題点・注意点:
  1. **Gazebo側との安全策の非対称性**：`LeggedHWSim::readSim`はコマンドの
     5値すべてを安全な既定値にリセットするが、`UnitreeHW::read`は
     `ff`・`velDes`・`kd`の3値しかリセットせず、`posDes`・`kp`は
     古い値のまま残る。コントローラ異常終了時の挙動が実機とGazeboで
     異なる可能性がある
  2. UDP接続先のIPアドレス・ポート(`192.168.123.10`等)がソースコードに
     ハードコードされ、rosparam化されていない
  3. IMU共分散値がGazebo側の設定ファイルとソースコードの2箇所に重複して
     ハードコードされている(値がずれるリスク)
  4. `updateJoystick`/`updateContact`の更新頻度`50`Hzがマジックナンバー
     として2箇所に重複している
- 確認できた重要な事実:
  - `RF/LF/RH/LH`(このリポジトリのURDF・接触名)、`FR_/FL_/RR_/RL_`
    (Unitree SDKの配列インデックス)という、**2つの異なる脚命名規則**が
    `setupJoints`の中で変換されている(pympc側の`FL/FR/RL/RR`も合わせると
    このワークスペース全体で3種類目の命名規則)
  - 接触検知は実機では力センサの閾値判定(既定閾値`40`)、Gazeboでは物理
    接触イベントの検出という、まったく異なる方式で実装されている
  - `write`が送るのは計算済みトルクではなく、5値のハイブリッド指令その
    ものであり、実際のPD計算はモーター側(ファームウェア、対象外)で
    行われると推測される(Gazebo側はホスト側で計算していたのと対照的)
  - `safety_->PositionLimit`/`PowerProtect`(SDK提供)が、実機における
    最終的なクリップ・電力保護を担う
- 次は、ros_controlのコントローラプラグイン本体、`legged_controllers/src/LeggedController.cpp`
  (状態推定→MPC→WBCを呼び出す「頭脳」)を読みます。
