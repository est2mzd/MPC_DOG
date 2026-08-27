# ハードウェア抽象化 legged_hw/LeggedHW + legged_common/hardware_interface 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
legged_unitree_hw.cpp の main()
  → legged::UnitreeHW::init(...) が内部で LeggedHW::init(...) を呼ぶ (継承、未読)
      → LeggedHW::init                          ← 本ファイル、起動時1回
          → loadUrdf(...)                        ← 本ファイル、起動時1回
          → registerInterface(4つ)               ← 本ファイル、起動時1回

LeggedHWLoop::update()(read_code_01) が毎周期呼ぶ
  hardwareInterface_->read(...) / ->write(...)
    → 実際の読み書きは LeggedHW を継承した具体クラス
      (UnitreeHW、LeggedHWSim)が実装する。LeggedHW自体は
      「どの型のインターフェースを持つか」の器を定義するだけで、
      read/writeの中身は持たない(純粋仮想ではないが、意味のある実装は
      継承先にある、**設計上の解釈**)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`LeggedHW`が担当するのは、「**ros_control(外部パッケージ)が要求する
`hardware_interface::RobotHW`を継承し、このロボット固有の4種類のハンドル
インターフェース(関節状態・IMU・ハイブリッド関節コマンド・接触センサ)を
用意して登録する**」ことです。

- 実際にCANバスやGazeboと通信して値を読み書きする処理は持ちません
  (継承先の`UnitreeHW`・`LeggedHWSim`の責務)
- `legged_common/hardware_interface`の2つのヘッダ
  (`HybridJointInterface`・`ContactSensorInterface`)は、`LeggedHW`が
  ros_controlの標準インターフェース(`JointStateInterface`・
  `ImuSensorInterface`)に加えて独自に定義した、このリポジトリ固有の
  インターフェース型です。これらの型定義自体もこのファイル群の役割に
  含めて解説します

対象は`external/legged_control/legged_hw/include/legged_hw/LeggedHW.h`
(59行)・`external/legged_control/legged_hw/src/LeggedHW.cpp`(34行)、
`external/legged_control/legged_common/include/legged_common/hardware_interface/HybridJointInterface.h`
(93行)、同ディレクトリの`ContactSensorInterface.h`(36行)です。

---

## `LeggedHW.h` 24〜56行:クラス定義

```cpp
class LeggedHW : public hardware_interface::RobotHW {
 public:
  LeggedHW() = default;
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
 protected:
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
  HybridJointInterface hybridJointInterface_;
  ContactSensorInterface contactSensorInterface_;
  std::shared_ptr<urdf::Model> urdfModel_;
 private:
  bool loadUrdf(ros::NodeHandle& rootNh);
};
```

- `hardware_interface::RobotHW`：ros_control(外部パッケージ)が定義する
  基底クラス。`registerInterface`等の仕組みを提供する(内部実装は
  **未確認**、対象リポジトリの外)
- `jointStateInterface_`(型`hardware_interface::JointStateInterface`、
  ros_control標準)：関節の位置(rad)・速度(rad/s)・トルク/力(N·m)を
  **読み取り専用**で公開するインターフェース
- `imuSensorInterface_`(型`hardware_interface::ImuSensorInterface`、
  ros_control標準)：IMUの姿勢・角速度・線形加速度を読み取り専用で公開
- `hybridJointInterface_`(型`HybridJointInterface`、このリポジトリ独自)：
  関節へ**コマンドを書き込む**ためのインターフェース。詳細は後述
- `contactSensorInterface_`(型`ContactSensorInterface`、このリポジトリ
  独自)：各脚の接触状態(`bool`)を読み取り専用で公開
- `urdfModel_`(型`std::shared_ptr<urdf::Model>`)：ロボットのURDF
  (ロボット記述XML)をパースしたモデル。他のパッケージ
  (`legged_interface`等)がロボットの運動学・質量パラメータを参照する
  ときの元データになると考えられる(**設計上の解釈**、実際の参照箇所は
  未確認)
- `protected`(非`private`)であることに注意：継承先の`UnitreeHW`・
  `LeggedHWSim`が、これらのメンバを直接読み書きしてハンドルを登録する
  設計になっている(継承ベースのテンプレートメソッドパターンに近い、
  **設計上の解釈**)

---

## `LeggedHW.cpp` 9〜21行:`init`

この関数の役割:URDFを読み込み、4種類のインターフェースをros_controlへ登録する。

```cpp
bool LeggedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }
  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);
  return true;
}
```

- `robot_hw_nh`引数はコメントアウト(`/*robot_hw_nh*/`)されており、この
  基底クラスの`init`では未使用(継承先の`UnitreeHW::init`が別途使う)
- `loadUrdf`が失敗すると`false`を返し、`init`全体が失敗扱いになる
  (呼び出し元の`main()`側で`try`/`catch`により`ROS_FATAL_STREAM`で
  検知される、read_code_01参照)
- `registerInterface`(ros_control標準API、外部)：各インターフェース
  オブジェクトのポインタをros_control(`hardware_interface::RobotHW`基底
  クラス内部)へ登録し、ros_controlのコントローラ側から
  `hw->get<HybridJointInterface>()`のような形で取得できるようにする

---

## `LeggedHW.cpp` 23〜31行:`loadUrdf`

この関数の役割:rosparamサーバーからURDF文字列を取得し、`urdf::Model`として
パースする。

```cpp
bool LeggedHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  rootNh.getParam("legged_robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}
```

- `urdfString`(型`std::string`)：rosparam`legged_robot_description`
  から取得。read_code_01の起動連鎖には出てこないが、Gazebo経路の
  `empty_world.launch`で`xacro`コマンドの出力としてこのパラメータに
  ロードされていた(pympc側read_code_16で見たROS2のトピック名解決に近い、
  「別ファイルがロードしたrosparamを、こちらが読む」という間接的な結合)
- `urdfModel_->initString(urdfString)`：URDF文字列をパースする
  (`urdf`パッケージ、外部、内部は**未確認**)
- **実装上の注意点**:`rootNh.getParam`の戻り値(取得成功/失敗の`bool`)を
  無視している。パラメータが存在しない場合、`urdfString`は空文字列の
  ままとなり、直後の`!urdfString.empty()`チェックで`false`が返るため
  実害は無いが、"パラメータが無かった"のか"パラメータはあったが空文字列
  だった"のかをログ上では区別できない

---

## `HybridJointInterface.h` 9〜92行:`HybridJointHandle`・`HybridJointInterface`

この関数の役割:1つの関節に対する「ハイブリッド(位置+速度+PDゲイン+
フィードフォワード)コマンド」の読み書きハンドルを定義する。

```cpp
class HybridJointHandle : public hardware_interface::JointStateHandle {
 public:
  HybridJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff)
      : JointStateHandle(js), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff) { ... }
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff) { ... }
  ...
 private:
  double* posDes_ = {nullptr};
  double* velDes_ = {nullptr};
  double* kp_ = {nullptr};
  double* kd_ = {nullptr};
  double* ff_ = {nullptr};
};
```

- `hardware_interface::JointStateHandle`(ros_control標準、外部)を継承する
  ことで、位置・速度・力(トルク)の**読み取り**機能をそのまま引き継ぐ
- `posDes_`(型`double*`、rad)：目標関節角度へのポインタ
- `velDes_`(型`double*`、rad/s)：目標関節角速度へのポインタ
- `kp_`(型`double*`、N·m/rad)：位置フィードバックゲイン
- `kd_`(型`double*`、N·m·s/rad)：速度フィードバックゲイン
- `ff_`(型`double*`、N·m)：フィードフォワードトルク
- コンストラクタは5つのポインタすべてが`nullptr`でないことを検証し、
  1つでも`nullptr`なら`hardware_interface::HardwareInterfaceException`
  を投げる
- `setCommand(pos_des, vel_des, kp, kd, ff)`：5値を一括で設定する

**コードで確認した事実(このハンドルの意味、`LeggedHWSim.cpp`で先取り確認)**：
最終的にハードウェア側(`LeggedHWSim::writeSim`)は、この5値から
実際のトルク指令を次の式で計算します(この式自体は後続ファイル
`legged_gazebo/LeggedHWSim.cpp`で詳しく扱う)。

\[
\tau = k_p (p_{des} - p) + k_d (v_{des} - v) + \tau_{ff}
\]

| 数式 | コード変数 | 意味 |
|---|---|---|
| \(p_{des}\) | `posDes_` | 目標関節角度 |
| \(v_{des}\) | `velDes_` | 目標関節角速度 |
| \(k_p\)、\(k_d\) | `kp_`、`kd_` | 位置・速度フィードバックゲイン |
| \(\tau_{ff}\) | `ff_` | フィードフォワードトルク(WBCが計算した値をそのまま渡すことが多いと推測される、**未確認**) |

つまりこのインターフェースは、pympc側(Quadruped-PyMPC)が`env.step`へ
純粋なトルクベクトルだけを渡していたのとは異なり、**関節ごとに
5つの値からなる「ハイブリッド指令」**を渡す設計になっています。
WBCが計算した最終トルクを「ゲインゼロ・フィードフォワードのみ」として
渡すことも、実機のUnitreeモーターのようにゲイン付きPD+フィードフォワード
として渡すことも、どちらも表現できる汎用的なインターフェースです。

```cpp
class HybridJointInterface
    : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};
```

- `hardware_interface::HardwareResourceManager<...>`(ros_control標準)：
  複数の`HybridJointHandle`(関節ごと)を名前で管理するコンテナ
- `hardware_interface::ClaimResources`：同じ関節を複数のコントローラが
  同時に「書き込み対象として占有」しようとするとエラーになる、
  排他制御ポリシー(ros_control標準の仕組み、コマンド系インターフェースに
  付けるのが通例)

---

## `ContactSensorInterface.h` 9〜35行:`ContactSensorHandle`・`ContactSensorInterface`

この関数の役割:1本の脚の接触状態(接地しているか否か)を読み取るための
ハンドルを定義する。

```cpp
class ContactSensorHandle {
 public:
  ContactSensorHandle(const std::string& name, const bool* isContact) : name_(name), isContact_(isContact) { ... }
  bool isContact() const { assert(isContact_); return *isContact_; }
 private:
  std::string name_;
  const bool* isContact_ = {nullptr};
};

class ContactSensorInterface
    : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};
```

- `isContact_`(型`const bool*`)：`const`ポインタであり**読み取り専用**
  (`HybridJointHandle`の書き込み用ポインタ群とは対照的)
- `hardware_interface::DontClaimResources`：読み取り専用インターフェース
  のため、排他制御(誰かが占有したら他が使えなくなる)は不要、という
  ポリシー。`HybridJointInterface`の`ClaimResources`との対比が、
  「書き込み系は排他、読み取り系は共有」というros_controlの標準的な
  設計方針を表している

- **設計上の解釈**:実際に`isContact_`が指す`bool`変数へどうやって値が
  書き込まれるか(センサ実測か、それとも推定値か)は、`UnitreeHW`・
  `LeggedHWSim`それぞれの実装次第(**未確認**、後続ファイルで確認する)

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `LeggedHW::loadUrdf`が`rootNh.getParam`の戻り値(取得成功/失敗)を
     無視しており、パラメータ未設定と空文字列設定を区別できない
- 確認できた重要な事実:
  - `LeggedHW`自体は「4種類のインターフェースを用意して登録する」だけの
    抽象基底クラスで、実際のセンサ読み取り・コマンド送信は継承先が行う
  - `HybridJointInterface`は、関節ごとに位置・速度・Kp・Kd・
    フィードフォワードトルクの5値からなる指令を渡す、汎用的なハイブリッド
    制御インターフェースであり、最終的なトルクは
    \(\tau=k_p(p_{des}-p)+k_d(v_{des}-v)+\tau_{ff}\)という式で計算される
    (`LeggedHWSim.cpp`で確認済み、詳細は後続ファイル)
  - `ContactSensorInterface`は読み取り専用で、`isContact_`の実体が
    センサ実測か推定かはこの章では確定できない
- 次は、この抽象インターフェースを実際に埋める具体的な実装
  (`legged_gazebo/LeggedHWSim.cpp`、Gazebo側=pympcの`Simulator_Node`に
  相当)を読みます。
