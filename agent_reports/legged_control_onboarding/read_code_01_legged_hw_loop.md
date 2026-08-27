# 実行ループの起点 legged_unitree_hw.cpp(main) + legged_hw/LeggedHWLoop 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
[起動] roslaunch legged_unitree_hw a1.launch(またはgo1.launch/aliengo.launch)
  → legged_unitree_hw.cpp の main()                              ← 本ファイル、起動時1回
      → legged::UnitreeHW::init(nh, robotHwNh)                    (未読、実機HW初期化)
      → legged::LeggedHWLoop コンストラクタ                       ← 本ファイル、起動時1回
          → 内部で std::thread を1本起動、while(loopRunning_) update() を無限ループ
              → hardwareInterface_->read(...)   ← 本ファイル、毎制御周期(a1/go1既定500Hz)
              → controllerManager_->update(...)  (ros_control、外部パッケージ、未確認)
              → hardwareInterface_->write(...)  ← 本ファイル、毎制御周期
```

**事実**:これは**実機ハードウェア経路**(`legged_unitree_hw`)の起動シーケンス
です。Gazeboシミュレーション経路は`legged_gazebo/LeggedHWSim`が
`gazebo_ros_control`(外部パッケージ、このリポジトリに含まれない)の
プラグインとしてロードされ、Gazebo自体の物理ステップイベントに合わせて
`read()`/`controllerManager_->update()`/`write()`相当の処理が呼ばれます。
つまり`LeggedHWLoop`(このファイルで読む、独自スレッドで動くループ)は
**実機経路専用**であり、Gazebo経路では使われません(Gazebo側の駆動方法は
別ファイルで扱う、**未確認**)。

## このファイル/クラスの役割(全体の中での位置づけ)

`LeggedHWLoop`が担当するのは、「**固定周波数で、ハードウェアの状態読み取り
(`read`)→ROS-Controlのコントローラ更新(`update`)→ハードウェアへの指令送信
(`write`)、という3ステップを永久に繰り返す**」ことだけです。

- 歩容計画・状態推定・MPC・WBCといった制御ロジックは一切持ちません。それらは
  すべて`controllerManager_->update()`が呼び出す、ロードされた
  ros_controlコントローラ(`legged/LeggedController`、後続ファイルで解説)側の
  責務です
- `main()`(`legged_unitree_hw.cpp`)は、このループを起動する前段の準備
  (ROSノード初期化、非同期スピナーの起動、具体的なハードウェア実装
  `UnitreeHW`の生成・初期化)だけを担当します

対象は`external/legged_control/legged_examples/legged_unitree/legged_unitree_hw/src/legged_unitree_hw.cpp`
(76行)と`external/legged_control/legged_hw/include/legged_hw/LeggedHWLoop.h`
(97行)・`external/legged_control/legged_hw/src/LeggedHWLoop.cpp`(86行)です。

---

## `legged_unitree_hw.cpp` 42〜76行:`main`

この関数の役割:ROSノードを初期化し、実機ハードウェアインターフェースを
生成・初期化した上で、制御ループ(`LeggedHWLoop`)を起動する。

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "legged_unitree_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try {
    std::shared_ptr<legged::UnitreeHW> unitreeHw = std::make_shared<legged::UnitreeHW>();
    unitreeHw->init(nh, robotHwNh);

    legged::LeggedHWLoop controlLoop(nh, unitreeHw);

    ros::waitForShutdown();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n" << "\t" << e.what());
    return 1;
  }
  return 0;
}
```

- `nh`(型`ros::NodeHandle`)：グローバル名前空間のノードハンドル
- `robotHwNh`(型`ros::NodeHandle`)：`"~"`(プライベート名前空間、
  例えば`/legged_unitree_hw/...`)のノードハンドル。`UnitreeHW::init`へ
  渡され、ロボット固有のパラメータ読み込みに使われる(**未確認**、
  `UnitreeHW`は未読)
- `ros::AsyncSpinner spinner(3)`：3スレッドの非同期スピナー。コメントに
  「サービスコールバック(コントローラのロード等)が(メインの)制御ループを
  ブロックしないよう、ROSのコールバック処理を別スレッドで回す」と明記されて
  いる
- `unitreeHw`(型`std::shared_ptr<legged::UnitreeHW>`)：実機用の具体的な
  ハードウェアインターフェース実装(未読、`legged::LeggedHW`を継承すると
  推測される、**設計上の解釈**)
- `unitreeHw->init(nh, robotHwNh)`：コメントより「1. rosparamから設定を
  取得、2. ハードウェアを初期化してros_controlと接続」の2つを行うと
  わかる(内部は未読)
- `legged::LeggedHWLoop controlLoop(nh, unitreeHw)`：この行がこのファイルの
  主役。コンストラクタの中でスレッドが起動し、**この行の実行が終わった
  時点で、すでにバックグラウンドで制御ループが回り始めている**(後述)
- `ros::waitForShutdown()`：メインスレッドはここでブロックし、ROSの
  シャットダウン信号を待つだけ。実際の処理はすべて`AsyncSpinner`のスレッド
  群と`LeggedHWLoop`が起動したスレッドが担う

**設計上の解釈**:`simulation.py`(Quadruped-PyMPCのpympc側read_code_01)の
`for`ループのような、メインスレッドが直接ステップを回す構造とは異なり、
ここではメインスレッドは「起動して待つだけ」で、実際の制御ステップは
別スレッドで動く。

---

## `LeggedHWLoop.h` 50〜95行:クラス定義

この関数の役割:制御ループの状態(周波数、タイミング、コントローラ
マネージャ、ハードウェアインターフェースへのポインタ)を保持する。

```cpp
class LeggedHWLoop {
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;
 public:
  LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface);
  ~LeggedHWLoop();
  void update();
 private:
  ros::NodeHandle nh_;
  double cycleTimeErrorThreshold_{}, loopHz_{};
  std::thread loopThread_;
  std::atomic_bool loopRunning_{};
  ros::Duration elapsedTime_;
  Clock::time_point lastTime_;
  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;
  std::shared_ptr<LeggedHW> hardwareInterface_;
};
```

- `loopHz_`(型`double`、Hz)：制御ループの目標周波数。rosparamの
  `loop_frequency`から読み込む。実際の値は`legged_examples/legged_unitree/legged_unitree_hw/config/<robot>.yaml`
  にあり、**a1・go1は`500`、aliengoは`800`**(実際に`grep`で確認済み)
- `cycleTimeErrorThreshold_`(型`double`、秒)：1周期の実測時間が目標周期
  からどれだけ超過したら警告するかの閾値。同yamlより**全ロボット共通で
  `0.002`**
- `loopThread_`(型`std::thread`)：制御ループを回す専用スレッド
- `loopRunning_`(型`std::atomic_bool`)：ループ継続フラグ。デストラクタで
  `false`にしてスレッドを止める
- `elapsedTime_`(型`ros::Duration`、秒)：直近の`update()`呼び出し間隔の
  実測値
- `lastTime_`(型`Clock::time_point`)：前回`update()`を実行した時刻
- `controllerManager_`(型`std::shared_ptr<controller_manager::ControllerManager>`)：
  ros_control(外部パッケージ)が提供するクラス。ロードされたコントローラ
  (`legged/LeggedController`等)の起動・停止・毎周期の`update()`呼び出しを
  一括管理する(内部実装は**未確認**、対象リポジトリの外)
- `hardwareInterface_`(型`std::shared_ptr<LeggedHW>`)：実際のハードウェア
  読み書きを行う抽象インターフェース(`LeggedHW`基底クラス、後続ファイルで
  読む)

---

## `LeggedHWLoop.cpp` 8〜42行:コンストラクタ

この関数の役割:ros_controlのコントローラマネージャを生成し、rosparamから
ループ周波数等を読み込み、専用スレッドで制御ループを起動する。

```cpp
LeggedHWLoop::LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("~");
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  if (error > 0) {
    std::string error_message = "could not retrieve one of the required parameters: ...";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  lastTime_ = Clock::now();

  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });
  sched_param sched{.sched_priority = threadPriority};
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN("Failed to set threads priority ...");
  }
}
```

- `controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_))`：
  ros_controlのコントローラマネージャに、このハードウェアインターフェース
  (`hardware_interface::RobotHW`を継承した`LeggedHW`)を紐づける。
  `controllers.yaml`(`legged_controllers/config/controllers.yaml`)で
  定義された`legged/LeggedController`等は、この`controllerManager_`経由で
  ロード・実行される
- `nhP.getParam("loop_frequency", loopHz_)`等：rosparamから3つの値を
  読み込む。**3つとも必須(デフォルト値なし)**で、読み込みに失敗すると
  `error`が加算され、1つでも失敗すれば`std::runtime_error`を投げてプロセスが
  落ちる
- `threadPriority`(型`int`、無次元)：a1/go1/aliengo共通で**`95`**
  (`grep`で確認済み)。POSIXリアルタイムスケジューリングの優先度
- `loopThread_ = std::thread(...)`：ラムダ式で`while(loopRunning_) update();`
  という無限ループを別スレッドで起動する。**このコンストラクタが返った
  時点で、すでに制御ループは実行され始めている**(スレッドの起動に
  `join`や同期待ちが無いため)
- `pthread_setschedparam(..., SCHED_FIFO, &sched)`：起動したスレッドに
  リアルタイムスケジューリングポリシー(`SCHED_FIFO`)と優先度`95`を設定
  しようとする。失敗しても(`sudo`権限が無い等)`ROS_WARN`を出すだけで
  処理は継続する(**実装上の注意点**：pympc側のROS2版`run_controller.py`が
  `sudo renice`の失敗を完全に無視していたのと同種の「権限不足時は警告のみで
  続行」というパターンがここにも見られる。ただしこちらは`ROS_WARN`で
  ユーザーに通知している分、pympc側より親切)

---

## `LeggedHWLoop.cpp` 44〜77行:`update`

この関数の役割:実際の制御周期1回分――経過時間の計測、ハードウェア読み取り、
コントローラ更新、ハードウェア書き込み、次周期までの待機――を実行する。

### 45〜60行:周期時間の計測と超過警告

```cpp
const auto currentTime = Clock::now();
const Duration desiredDuration(1.0 / loopHz_);

Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
elapsedTime_ = ros::Duration(time_span.count());
lastTime_ = currentTime;

const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
if (cycle_time_error > cycleTimeErrorThreshold_) {
  ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << ...);
}
```

- `desiredDuration`(型`Duration`(`double`秒)、秒)：`1.0/loopHz_`。
  a1/go1なら`1/500=0.002`秒、aliengoなら`1/800=0.00125`秒
- `elapsedTime_`(型`ros::Duration`、秒)：前回`update()`からの実測経過時間。
  ハードウェアの`read`/`write`とコントローラの`update`双方へこの実測値が
  そのまま渡される(固定値ではなく**実測ベース**、pympc側ROS2経路の
  `run_controller.py`が`USE_FIXED_LOOP_TIME`で選べたのと似た方式だが、
  こちらは実測一択で切替フラグは無い)
- `cycle_time_error`(秒)：実測経過時間から目標周期を引いた超過分。
  `cycleTimeErrorThreshold_`(既定`0.002`秒)を超えたら`ROS_WARN_STREAM`で
  警告するだけで、処理は継続する(異常終了しない)

### 62〜72行:Read → Control → Write

```cpp
hardwareInterface_->read(ros::Time::now(), elapsedTime_);
controllerManager_->update(ros::Time::now(), elapsedTime_);
hardwareInterface_->write(ros::Time::now(), elapsedTime_);
```

- `hardwareInterface_->read(...)`：`LeggedHW`(次ファイルで読む)経由で、
  実機(`UnitreeHW`)から関節角度・速度・トルク、IMU、足先接触センサ等の
  最新状態を取得し、ros_controlの`hardware_interface`群へ書き込む
- `controllerManager_->update(...)`：ロード済みの全アクティブコントローラ
  (既定では`joint_state_controller`と`legged/LeggedController`)の
  `update()`を順に呼ぶ。`legged/LeggedController`の内部(状態推定→MPC→WBC→
  関節指令計算)がここで実行される(**別ファイルで解説**)
- `hardwareInterface_->write(...)`：コントローラが計算した関節指令
  (トルク/位置/速度のハイブリッド指令、`legged_common`の
  `HybridJointInterface`経由と推測、**未確認**)を実機へ送信する

**コードで確認した事実**:`simulation.py`(pympc)の1ステップが
「観測→`compute_actions`→トルククリップ→`env.step`」という直列処理だったのに
対し、こちらは「`read`→`controllerManager_->update`→`write`」という
ros_controlの標準3段階パターンに従っている。制御ロジック
(状態推定・MPC・WBC)はすべて`update`呼び出しの中(=ロードされた
コントローラプラグインの内部)にカプセル化されており、この`LeggedHWLoop`
自体はその中身を一切知らない。

### 74〜76行:次周期までスリープ

```cpp
const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
std::this_thread::sleep_until(sleepTill);
```

- `sleepTill`：このステップの開始時刻(`currentTime`)から`desiredDuration`
  (目標周期)だけ進んだ絶対時刻。`std::this_thread::sleep_until`で
  その時刻まで待つ
- **実装上の注意点**：`update`の処理時間(`read`〜`write`の実行時間)が
  `desiredDuration`を超えていた場合、`sleepTill`はすでに過去の時刻になり
  `sleep_until`は即座に返る(待たない)。つまりこの実装は「処理落ちしても
  次周期を待たずにすぐ再実行する」動作になり、暴走的にループが詰まって
  いく可能性がある(上の`cycle_time_error`警告はこの状況を検知して知らせる
  だけで、防止はしない。**設計上の解釈**、実際に処理落ちが連鎖するかは
  実行時の負荷次第で**未確認**)

---

## `LeggedHWLoop.cpp` 79〜84行:デストラクタ

この関数の役割:ループフラグを落として、制御スレッドの終了を待ってから
破棄する。

```cpp
LeggedHWLoop::~LeggedHWLoop() {
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}
```

- `loopRunning_ = false`：`std::atomic_bool`への書き込み。次回`while`条件
  チェック時にループを抜ける(現在実行中の`update()`1回分は最後まで完走
  してから抜ける、`update()`の途中で強制中断はされない)
- `loopThread_.join()`：スレッドの終了を待つ。これによりプロセス終了時に
  ハードウェアへの書き込みが中途半端な状態で放置されることを防ぐ
  (**設計上の解釈**)

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. リアルタイムスレッド優先度の設定(`pthread_setschedparam`)が失敗しても
     `ROS_WARN`のみで処理が継続する(pympc側ROS2経路の`sudo renice`失敗と
     同種のパターンだが、こちらは警告ログが出る分親切)
  2. `update()`内のスリープは絶対時刻ベースで、処理が目標周期を超過すると
     待たずに即座に次周期へ突入する。処理落ちの連鎖を防ぐ仕組みは無く、
     警告(`cycle_time_error`)のみ
- 確認できた重要な事実:
  - `LeggedHWLoop`は**実機経路専用**であり、Gazebo経路
    (`legged_gazebo/LeggedHWSim`)はこのクラスを使わず、
    `gazebo_ros_control`(外部パッケージ)の駆動に従う
  - ループ周波数はロボットタイプごとに異なる(a1/go1:500Hz、
    aliengo:800Hz)。この値と`cycle_time_error_threshold`(全ロボット
    共通0.002秒)・`thread_priority`(全ロボット共通95)はすべて
    `legged_examples/legged_unitree/legged_unitree_hw/config/<robot>.yaml`
    から読み込まれる
  - 制御ロジック本体(状態推定・MPC・WBC)はこのファイルには一切無く、
    `controllerManager_->update()`が呼ぶ`legged/LeggedController`
    (ros_controlコントローラプラグイン)の内部にすべて存在する
- 次に読むべき候補:
  1. `legged_hw/LeggedHW.h`/`.cpp`(抽象ハードウェアインターフェース、
     `read`/`write`が実際に何を読み書きするかの土台)
  2. `legged_common/hardware_interface`(`HybridJointInterface`・
     `ContactSensorInterface`、実機/Gazebo共通のインターフェース型定義)
  3. `legged_controllers/src/LeggedController.cpp`(制御ロジック本体、
     状態推定→MPC→WBCを呼ぶ「頭脳」に相当)

呼び出し連鎖の自然な流れとしては1→2→3の順が妥当と考えられますが、
次にどれから読むか、ご指示をお願いします。
