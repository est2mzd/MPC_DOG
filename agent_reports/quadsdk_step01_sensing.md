# Quad-SDK Step 01 のセンシング(sensing / 状態推定)の仕組みとデータ構造

作成: 2026-08-30。`external/quad-sdk` の launch ファイルと C++ ソースを実際に
読んで確認した内容。**【事実】=コードで確認済み**、**【推測】=未確認・仮説**
として節を分ける。関連: `agent_reports/quadsdk_step01_control_pipeline.md`(5節)、
`agent_reports/quadsdk_step01_terrain_map.md`。

---

## 背景

Step 01 の記録ハーネス `src/trial/quadsdk_step01_baseline.py` は
`/{ns}/state/ground_truth` を主軸に CSV を書いている。転倒・不安定の調査でも、
「制御が見ている状態は本物か(推定誤差・遅延・ドリフトがあるのか)」が
論点になった。handoff でも「MUMPS の数値精度不足が転倒原因という証拠は
見つからなかった」「実際の原因は起動シーケンスと地面サイズだった」(8節)と
結論している。

そこで、**Step 01 のシミュレーションで「センシング」が具体的に何をしていて、
`local_planner` / `robot_driver` / ロガーが受け取る状態はどこから来るのか** を
コードで確認した結果をまとめる。

---

## 概要(何が・どこから・どんな処理・どこへ)

### 一言で

MuJoCo の物理真値を `mujoco_estimator` が1つの `RobotState` にまとめ直し、
`state/ground_truth` として 500 Hz で流す。**フィルタ推定(カルマン等)は
挟まらない**。制御もプランナもロガーも、この同じトピックを購読する。

### 全体図

```
[取得元]                    [処理: mujoco_estimator]           [出力]                 [行き先]

MuJoCo (mujoco_ros2_control)
 ├ /robot_1/odom ──────────▶ 座標系を整える                ┐
 │  浮遊ベースの pose・速度   (linear=world, angular=body)   │
 │                                                          │
 ├ /robot_1/joint_states ──▶ 関節名順を quad-sdk 脚順へ     ├▶ quad_msgs/RobotState ─┬▶ robot_driver (sim)
 │  関節 角度・角速度・トルク  リマップ(FL,BL,FR,BR)         │   /robot_1/            │   → そのまま制御状態
 │                                                          │   state/ground_truth  │   → WBC(inverse_dynamics)
 └ /robot_1/robot_description ▶ QuadKD2 で足先を FK 計算     ┘   (500 Hz, latch なし) ├▶ local_planner
    URDF(latched)                                                                    │   → 参照軌道の初期条件
                                                                                     │   → footstep の現在位置
                                                                                     ├▶ body_force_estimator
                                                                                     │   (+ local_plan)→ 外力推定
                                                                                     │   → robot_driver で補償
                                                                                     └▶ quadsdk_step01_baseline.py
                                                                                         → CSV 記録
```

### データ1つずつ(何が / どこから / どんな処理 / どこへ)

- **胴体の位置・姿勢・速度**
  - どこから: MuJoCo(`mujoco_ros2_control`)→ `/robot_1/odom`(`nav_msgs/Odometry`)
  - どんな処理: `mujoco_estimator` が座標系を揃えるだけ
    (`body.twist.linear` は world 系、`body.twist.angular` は body 系)。
    フィルタ・積分・補正はしない
  - どこへ: `RobotState.body.pose` / `RobotState.body.twist` → `state/ground_truth`
- **関節の角度・角速度・トルク**
  - どこから: MuJoCo → `/robot_1/joint_states`(`sensor_msgs/JointState`。
    ros2_control の `joint_state_broadcaster` も出す)
  - どんな処理: MuJoCo の関節名順 → quad-sdk の脚順
    (`FL, BL, FR, BR` × `abad, hip, knee`)にインデックスを並べ替えるだけ
  - どこへ: `RobotState.joints` → `state/ground_truth`
- **足先の位置・速度**
  - どこから: 上の関節角 + URDF(`/robot_1/robot_description`、latched)
  - どんな処理: `QuadKD2` で順運動学(FK)計算。センサ値ではなく計算値
  - どこへ: `RobotState.feet` → `state/ground_truth`
- **接地(どの脚が地面についているか)**
  - どこから: **専用のセンシングは無い**。`contact_state_publisher_node` は
    起動されるが入力が Gazebo 形式トピックで、MuJoCo では供給されない可能性が高い
    (【推測】参照)
  - どんな処理: 歩行中の接地判断はプランナ/コントローラが持つ接地スケジュール
  - どこへ: ロガーの `contact_*` 列は `control/grfs`(= `robot_driver` の出力)由来

### 処理されて「いない」もの(重要)

- `comp_filter` / EKF などのフィルタ推定は sim では**制御に使われない**。
  `robot_driver` は sim モードで `state/ground_truth` をそのまま制御状態にする
  (推定器オブジェクトは作られるが状態更新経路で呼ばれない。EKF はデバッグ用に
  並走できるだけで既定 off)
- `state/estimate` トピックは **sim では publish されない**(ハードウェア専用経路)

### だから何が言えるか

- ロガーが記録する `state/ground_truth` は **ほぼ MuJoCo 真値**。
  CSV の `base_pos_*` / `base_*_vel_*` / `base_roll/pitch/yaw` は信頼してよい。
  センシング起因の誤差・遅延・ドリフトは Step 01 にはほぼ無い
- 一方、ロガーの `contact_*` / `grf_*` 列は接地センサの実測ではなく
  `robot_driver` の出力である点に注意

---

## 流れ(順を追って)

### ステップ1: MuJoCo が生の物理量を出す

- `mujoco_ros2_control`(外部依存。このリポジトリには含まれない)が
  MuJoCo 物理エンジンを回し、
  - `/robot_1/odom`(`nav_msgs/Odometry`)… 浮遊ベースの world pose と速度
  - `/robot_1/joint_states`(`sensor_msgs/JointState`)… 関節 pos / vel / effort
  を publish する
- `joint_states` は ros2_control の `joint_state_broadcaster`(spawner で起動、
  `agent_reports/quadsdk_step01_control_pipeline.md` 9節)も出す

### ステップ2: `mujoco_estimator` が `RobotState` に整形する

`quad_simulator/mujoco_plugins/src/mujoco_estimator.cpp`:

1. `/robot_1/odom` と `/robot_1/joint_states` を購読してキャッシュ
2. `/robot_1/robot_description`(latched)を受け取り `QuadKD2`(運動学)を構築
3. 500 Hz タイマ `publishState()` で `quad_msgs/RobotState` を組み立てる:
   - `body.pose` = odom の pose(world 座標系)
   - `body.twist.linear` = odom の linear（**world 座標系**）
   - `body.twist.angular` = odom の angular（**body 座標系**。
     MuJoCo の free joint は `qvel[3:6]` が local frame という規約）
   - `joints` = MuJoCo の関節順 → quad-sdk の脚順にリマップ
     (`leg_<n>.joints.{abad,hip,knee}.name` パラメータ。無ければ
     フォールバック順 `{"8","0","1","9","2","3","10","4","5","11","6","7"}`)
   - `feet` = `updateDynamics()` + `fkRobotState()` で FK 計算した足先 pos/vel
4. publish 先:
   - `/robot_1/state/ground_truth`(world 系。angular だけ body 系)
   - `/robot_1/state/ground_truth_body_frame`(pose を恒等化、linear/angular
     とも body 系、足先も body 系)

> つまり "sensing" の実体はここだけで、やっているのは
> **真値の座標整形 + 関節順リマップ + 足先 FK**。カルマンフィルタや
> 相補フィルタは通っていない。

### ステップ3: `robot_driver` が状態を受け取る(sim 経路)

`robot_driver/src/robot_driver.cpp`:

- コンストラクタで `is_hardware_ == false` なら
  `robot_state_sub_`(`state/ground_truth` 購読)を作る
  → `robotStateCallback()` が `last_robot_state_msg_` に丸ごと格納
- `initStateEstimator()` は `estimator_id_ == "comp_filter"` で
  `CompFilterEstimator` を生成し `init()` する(オブジェクトは作られる)
- しかし `updateState()` の **sim 分岐(`else`)は基本的に何もせず `return true`**。
  例外は「`debug_estimator_` パラメータが有効 かつ `control_mode_ == READY`」の
  ときだけ EKF を**並走**させ、`ekf_estimate_msg_` に書く経路
  (制御には使われない、既定 off)
- `updateControl()` は `last_robot_state_msg_`(= `state/ground_truth` そのもの)を
  使って WBC 相当(`inverse_dynamics_controller`)を回す

### ステップ4: 他ノードも同じ `state/ground_truth` を購読する

- `local_planner` … 参照軌道の初期条件・footstep の現在位置基準
  (`agent_reports/quadsdk_step01_control_pipeline.md` 6節)
- `body_force_estimator_node` … `state/ground_truth` + `local_plan` から
  運動量残差ベースで外力を推定し `quad_msgs/BodyForceEstimate` を publish。
  `robot_driver` がこれを購読して外乱補償に使う
- `src/trial/quadsdk_step01_baseline.py`(ロガー)… CSV 記録

### ステップ5: 接地(contact)の扱い

- `contact_state_publisher_node`(`gazebo_scripts`)が
  `quad_mujoco_bringup.py: launch_contact_state_publisher()` で起動される
- ただし購読トピックは `gazebo/<toe>_contact_states`
  (型 `ros_gz_interfaces/msg/Contacts`)という **Gazebo 専用形式**
- ロガーが `contact_*` 列に入れているのは `control/grfs`(`quad_msgs/GRFArray`)の
  `contact_states` で、これは `robot_driver` の leg controller が
  `computeLegCommandArray()` の中で埋める値。**接地センサではなく
  コントローラ側が持っている接地状態**(local_plan 由来)

---

## 詳細:データ構造

### `quad_msgs/RobotState`(センシングの主出力)

- `header` … `stamp` / `frame_id`
- `body`(`quad_msgs/BodyState`)
  - `pose`(`geometry_msgs/Pose`)… 位置 [m] + クォータニオン姿勢
  - `twist`(`geometry_msgs/Twist`)… 並進速度 + 角速度
- `joints`(`sensor_msgs/JointState` 相当)
  - `name[]` / `position[]`(rad)/ `velocity[]`(rad/s)/ `effort[]`(N·m)
- `feet`(`quad_msgs/MultiFootState`)
  - `feet[4]` 各要素に `position` / `velocity` / `acceleration` / `contact`

座標系の規約(コード・`mujoco_plugins/README.md` で確認):

- `state/ground_truth`
  - `body.pose` … world
  - `body.twist.linear` … **world 系**
  - `body.twist.angular` … **body 系**
- `state/ground_truth_body_frame`
  - `body.pose` … 恒等(位置 0、姿勢単位クォータニオン)
  - `body.twist.linear` / `angular` … 両方 body 系
  - `feet[*].position` / `velocity` … body 系

### 脚順(ロガーの `LEG_ORDER` 問題と同じ)

- quad-sdk の脚順は `FL, BL, FR, BR`
  (`src/trial/quadsdk_step01_baseline.py:30` のコメント参照)
- `mujoco_estimator` の `quadsdk_joint_order_` は `go2.yaml` の
  `leg_<n>.joints.{abad,hip,knee}.name` から作られる。取得できないと
  ハードコードのフォールバック順になる

### `nav_msgs/Odometry`(`/robot_1/odom`、mujoco_estimator の入力)

- `pose.pose` … 浮遊ベースの world pose
- `twist.twist.linear` … world 系の並進速度
- `twist.twist.angular` … **body 系**の角速度(MuJoCo free-joint 規約)

### `quad_msgs/BodyForceEstimate`(`body_force_estimator_node` の出力)

- `joint_torques[]` … 運動量残差(外力の間接指標)
- 入力は `state/ground_truth` と `local_plan`。純粋な力センサではなく
  モデルベースの推定

### `quad_msgs/GRFArray`(`control/grfs`、ロガーが読む)

- `vectors[4]` … 各脚の接地力ベクトル [N]、world 系
- `points[4]` … 力の作用点
- `contact_states[4]` … bool(コントローラ側の接地判断)
- **これは `robot_driver` が「今こう出している」GRF であって、
  力センサの実測ではない**

---

## 【推測】未確認事項

- **`/robot_1/odom` の publisher と周波数**
  - `mujoco_ros2_control`(外部依存、このリポジトリに無い)が出しているはずだが、
    ソースを確認できていない。周波数・遅延・ノイズ有無は未計測
- **`contact_state_publisher_node` が MuJoCo で機能しているか**
  - 購読トピックが `gazebo/<toe>_contact_states`(`ros_gz_interfaces/Contacts`)で
    Gazebo 専用。MuJoCo 側がこれを publish しているとは考えにくい
  - もしそうなら、このノードが出す `GRFArray` は全ゼロ(または未 publish)。
    `ros2 topic echo` で要確認
  - ただし Step 01 の歩行は `local_plan` の接地スケジュールで回るため、
    このノードが空でも実害は出ていないと推測
- **`comp_filter` が sim で完全に休眠しているか**
  - `updateState()` の sim 分岐では `state_estimator_->loadSensorMsg()` /
    `updateOnce()` を呼んでいないので休眠と読めるが、実行時 introspection
    (`ros2 node info` / ログ)での確認はしていない
- **`state/ground_truth` の遅延**
  - `mujoco_estimator` は 500 Hz タイマで publish するが、`/clock` との
    同期ずれ・キューイング遅延の有無は未計測。ロガー側は各行の時刻を
    メッセージの `header.stamp` から取っている(`quadsdk_step01_baseline.py:123`)ので
    記録上の時刻ずれは小さいはず
- **`fall_time_s` 誤検出(handoff 4節)との関係**
  - これはセンシングの誤差ではなく、ロガー側の `FALL_HEIGHT_THRESHOLD_M`
    判定ロジックの問題(STAND 完了前の受動的な沈み込みを転倒と誤判定)。
    `state/ground_truth` の `base_pos_z` 自体は真値で正しい

---

## その後(このセンシングの意味と、次に見るべき点)

- **Step 01 の結論**: 制御が見ている状態は MuJoCo 真値。転倒・不安定の原因を
  「推定誤差」「フィルタのチューニング」に求める必要はない
  (handoff 8節・本ドキュメント概要と整合)。
- **ロガーの信頼性**: `state/ground_truth` 由来の CSV 列
  (`base_pos_*`、`base_*_vel_*`、`base_roll/pitch/yaw`)は真値なので信頼できる。
  一方 `contact_*` / `grf_*` 列は `robot_driver` の出力であって接地センサではない
  点に注意。
- **`agent_reports/quadsdk_step01_control_pipeline.md` 5.2 節の訂正**:
  同節は「`comp_filter` が `state/estimate` を出し、制御側がそれも参照する」と
  書いていたが、本調査で **sim では `state/estimate` は publish されず、
  制御は `state/ground_truth` 直結** であることが分かった。control_pipeline 側は
  追って修正する。
- **実機・ノイズ有り試験に進むとき最初に確認すべき点**:
  - `estimator:=comp_filter` が実際に効く経路(`is_hardware:=true` か
    `debug_estimator` 有効化)と、`state/estimate` を出す設定
  - IMU トピック(`topics.state.imu`)の供給元
  - MuJoCo での接地センシングを有効にする必要性
    (`contact_state_publisher` の MuJoCo 対応、または foot-force 相当の代替)

---

## ソース早見表(`external/quad-sdk/`)

- 真値整形(センシング本体)
  - `quad_simulator/mujoco_plugins/src/mujoco_estimator.cpp`(`publishState`, 500 Hz)
  - `quad_simulator/mujoco_plugins/include/mujoco_estimator/mujoco_estimator.hpp`
  - `quad_simulator/mujoco_plugins/README.md`(座標系規約の説明)
- 状態の受け取り・(未使用の)推定器
  - `robot_driver/src/robot_driver.cpp`(`robotStateCallback` / `updateState` の
    sim 分岐 / `initStateEstimator`)
  - `robot_driver/src/estimators/comp_filter_estimator.cpp`(sim では休眠)
  - `robot_driver/src/estimators/ekf_estimator.cpp`(デバッグ並走のみ)
  - `robot_driver/src/estimators/state_estimator.cpp`(基底)
- 接地
  - `quad_simulator/gazebo_scripts/src/contact_state_publisher.cpp`
    (Gazebo 形式トピック購読)
- 外力推定
  - `body_force_estimator/src/body_force_estimator.cpp`
- 起動
  - `quad_utils/launch/quad_mujoco_bringup.py`
    (`launch_mujoco_ground_truth` / `launch_contact_state_publisher`)
  - `quad_utils/launch/planning.py`(`launch_body_force_estimator`)
- 消費側
  - `src/trial/quadsdk_step01_baseline.py`(ロガー、`state/ground_truth` 購読)
