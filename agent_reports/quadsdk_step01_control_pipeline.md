# Quad-SDK Step 01 の制御パイプライン(map → sensing → MPC → WBC)

作成: 2026-08-30。`external/quad-sdk` の launch ファイルと C++ ソースを実際に
読んで確認した内容(推測ではなく実コードベース)。

## 0. なぜ `src/trial/quadsdk_step01_baseline.py` を読んでも流れが分からないのか

`quadsdk_step01_baseline.py` は**制御パイプラインの一部ではない**。
ROS2 トピックを外から購読して CSV に落とすだけの**受動的なロガー**であり、
map / sensing / MPC / WBC のどれも実装していない。購読しているのは出力側の
3トピックだけ(`src/trial/quadsdk_step01_baseline.py:93-95`):

- `/{ns}/control/grfs`(`quad_msgs/GRFArray`)
- `/{ns}/local_plan`(`quad_msgs/RobotPlan`)
- `/{ns}/state/ground_truth`(`quad_msgs/RobotState`)

制御の実体は全て `external/quad-sdk` の C++ ROS2 ノード群にあり、それらを起動
しているのは実行スクリプトの2つの `ros2 launch` 呼び出しである:

- `ros2 launch quad_utils quad_mujoco.py`
  - 場所: `scripts/trial/run_quadsdk_step01_baseline.sh:74-82`
  - 起動するもの: シミュレータ・状態推定・地形マップ・ros2_control・録画
- `ros2 launch quad_utils quad_plan.py`(→ `planning.py`)
  - 場所: `scripts/trial/run_quadsdk_step01_baseline.sh:156-159`
  - 起動するもの: local planner・NMPC・body force estimator

Quadruped-PyMPC 版(`step_01_baseline.py`)は単一プロセスの Python ループなので
1ファイルで流れが追えるが、Quad-SDK は多数の ROS2 ノードの集合体なので、
流れは「どのノードがどのトピックを介してつながっているか」でしか表現できない。

## 1. 全体像

```
 world xacro (flat_wide.xml.xacro)
        │  prepare_world: xacro 展開 → MJCF
        ▼
┌─────────────────────────────────────────────────────────────────────┐
│ MuJoCo 物理エンジン  (mujoco_ros2_control / ros2_control_node)        │
│   name = "controller_manager"  (quad_mujoco.py: launch_mujoco_world) │
└───────────────┬─────────────────────────────────────────▲───────────┘
                │ mjData(接地・関節・浮遊ベース)          │ アクチュエータ入力(トルク)
                ▼                                          │
   ┌──────────────────────────┐              ┌────────────────────────────┐
   │ [MAP]                    │              │ [ros2_control]             │
   │ mjcf_to_grid_map_node    │              │ joint_controller           │
   │ grid_map_filters(_demo)  │              │ (robot_driver の           │
   │  → /mapping/terrain_map  │              │  ros2_control コントローラ)│
   │  → relay → /r1/terrain_map│             │  トルク指令を MuJoCo へ書込 │
   └───────────┬──────────────┘              └────────────▲───────────────┘
               │ grid_map_msgs/GridMap                    │ control/joint_command
               │                                          │ (quad_msgs/LegCommandArray)
               ▼                                          │
   ┌──────────────────────────┐         ┌─────────────────┴──────────────┐
   │ [SENSING]                │         │ [WBC 相当] robot_driver_node   │
   │ mujoco_estimator         │         │  controller: inverse_dynamics  │
   │  → /r1/state/ground_truth │────────▶│  InverseDynamicsController     │
   │ contact_state_publisher  │  state  │  ・支持脚: GRF → Jᵀ → トルク    │
   │ body_force_estimator     │         │  ・遊脚:   逆動力学 + PD 追従   │
   │ (robot_driver 内 EKF/相補)│         │  出力: control/joint_command   │
   └───────────┬──────────────┘         │       control/grfs (適用GRF)   │
               │ state/ground_truth              └────────────▲───────────────┘
               ▼                                          │ local_plan
   ┌──────────────────────────────────────────────────────┴──────────────┐
   │ [LOCAL PLANNER]  local_planner_node                                 │
   │   local_footstep_planner.cpp : 地形マップ + 参照から着地点を決定     │
   │   local_planner.cpp          : ホライズン分の参照軌道を構築          │
   │        │                                                            │
   │        ▼  [MPC]  nmpc_controller (local_planner にリンクされる lib)  │
   │   nmpc_controller.cpp + quad_nlp.cpp : NLP 構築                      │
   │        → IPOPT (linear_solver = mumps) で解く                        │
   │        → 最適 GRF 系列 + 胴体状態軌道                                │
   │   出力: /r1/local_plan (quad_msgs/RobotPlan = 胴体軌道+GRF+足先計画) │
   └────────────────────────────────────────────────────────────────────┘
               ▲
               │ cmd_vel (参照指令)  ← `ros2 topic pub -r 50 /r1/cmd_vel`
               │ control/mode        ← STAND(1) / WALK(2)
        (シェルスクリプトが送信)
```

`r1` = `robot_1` 名前空間。global body planner は Step 01 では**動かない**(3節参照)。

## 2. 起動の2レイヤ

### レイヤ A: `quad_mujoco.py`(`run_quadsdk_step01_baseline.sh:74`)

`generate_launch_description()` の `OpaqueFunction` 群が順に実行される:

- `prepare_world`
  - `flat_wide.xml.xacro` を xacro 展開して `/tmp` に MJCF を書き、`world_path` に格納
- `launch_mujoco_world`
  - `mujoco_ros2_control` パッケージの `ros2_control_node` を起動
  - `name=controller_manager` / `namespace=robot_1`
  - これが MuJoCo 物理エンジン本体 + ros2_control のハードウェアインターフェース
- `launch_robot_mapping`
  - `/mapping` 名前空間で `mujoco_mapping.py` を include(→ 4節)
- `launch_robot_group`
  - `/robot_1` 名前空間で `quad_mujoco_bringup.py` を include
  - ここで sensing・robot_driver・ros2_control スポナー・TF が立ち上がる(→ 5〜7節)
- `launch_visualization`
  - `quad_visualization.py`(rviz)
  - `run_...sh` は quad-sdk 側の既定を `rviz:=false` に変更済みなので RViz は出ない
- `launch_recording`
  - `quad_utils/mujoco_recorder`。オフスクリーンで mp4 録画
  - 固定カメラ設定・目盛り焼き込みは MPC_DOG 側の改修

### レイヤ B: `quad_plan.py` → `planning.py`(`run_quadsdk_step01_baseline.sh:156`)

`STAND` 送信 → `STAND_SETTLE_S` 待機の**後**に起動する。`reference:=twist` を渡す:

- `launch_logging`
  - `logging:=true` 既定。quad_logger のバッグ記録ノード
- `launch_global_planner`
  - `reference != 'gbpl'` なので**何も起動しない**(→ 3節)
- `launch_twist_input_nodes`
  - `twist_input:=none` なのでキーボード/ジョイは無し
  - `cmd_vel` はシェルスクリプトが生 `ros2 topic pub` で送る
- `launch_local_planner`
  - `local_planner_node` を起動(→ 6節)
  - `local_planner.use_twist_input = (reference == 'twist')` が `true` になるのが要点
  - これが false だと local planner は `cmd_vel` を無視して global_plan 待ちになる
- `launch_body_force_estimator`
  - `body_force_estimator_node`(外力推定、→ 5節)

## 3. [参照] Global body planner — Step 01 では動かない

`planning.py: launch_global_planner()` は先頭で
`if LaunchConfiguration('reference') != 'gbpl': return []` する。
`run_quadsdk_step01_baseline.sh:157` は `reference:"twist"` を渡しているので
`global_body_planner_node` は起動しない。

つまり Step 01 では「目標地点 → RRT 大域経路」の段は**存在しない**。
参照は `cmd_vel`(一定前進速度 Twist)が local planner に直接入るだけ。
これがシェルスクリプトのコメント(`:152-154`)が「`reference` を `twist` に
しないと `gbpl` のまま静止し続ける」と書いている理由。

## 4. [MAP] 地形マップ

launch: `quad_utils/launch/mujoco_mapping.py`(`/mapping` 名前空間)

- `mjcf_to_grid_map_node`
  - パッケージ: `quad_utils`
  - MJCF world ジオメトリを読み、`grid_map_msgs/GridMap`(層名 `z`、解像度 0.05 m)を生成
  - `latch_grid_map_pub:=true` で latch publish
- `grid_map_filters`(exec 名 `grid_map_filters_demo`)
  - パッケージ: `quad_utils`
  - `filter_chain.yaml` のフィルタ列を適用し `/mapping/terrain_map` を出力
- `grid_map_visualization`
  - パッケージ: `grid_map_visualization`
  - RViz 用マーカー化(表示のみ)
- `static_transform_publisher`
  - パッケージ: `tf2_ros`
  - `world` → `map` の固定 TF

`/robot_1` 側へは `quad_mujoco_bringup.py: access_terrain_map()` が
`topic_tools/relay`(`terrain_map_relay`)で `/mapping/terrain_map` →
`/robot_1/terrain_map` に中継する。

`flat_wide.xml` は単純な直方体プリミティブ地面なので、この GridMap は実質「平ら」。
local footstep planner は接地可能高さの参照としてこれを使うが、平地では
ほぼ定数を返すだけ。

> ゾンビプロセス問題(handoff 3-3節)で名指しされた `grid_map_visualization` /
> `topic_tools/relay` / `static_transform_publisher` はここで起動されるノード。

## 5. [SENSING] 状態推定

### 5.1 真値ベース状態: `mujoco_estimator`

- launch: `quad_mujoco_bringup.py: launch_mujoco_ground_truth()`
- パッケージ: `quad_simulator/mujoco_plugins`、exec `mujoco_estimator`
- ソース: `quad_simulator/mujoco_plugins/src/mujoco_estimator.cpp`
- 動作:
  - `mujoco_ros2_control` が出す浮遊ベース odom と `joint_states` を購読
  - QuadKD2 の順運動学で足先位置・速度を補完して `quad_msgs/RobotState` を組み立て
  - **500 Hz** で publish する
    - `/robot_1/state/ground_truth`(world 座標系。`mujoco_estimator.cpp:77-78`)
    - `/robot_1/state/ground_truth_body_frame`(body 座標系)
- 規約:
  - `state/ground_truth` では `body.twist.linear` は world 系、
    `body.twist.angular` は body 系(Gazebo プラグインの規約を踏襲。
    `mujoco_plugins/README.md` に明記)

**`quadsdk_step01_baseline.py` が記録の主軸として読むのはこの
`state/ground_truth`。** 推定ではなくシミュレータ真値由来。

### 5.2 フィルタ推定(robot_driver 内)

- `quad_mujoco_bringup.py` は `estimator:=comp_filter`(既定)を robot_driver に渡す
- ソース:
  - `robot_driver/src/estimators/comp_filter_estimator.cpp`(既定)
  - `robot_driver/src/estimators/ekf_estimator.cpp`(代替)
  - `robot_driver/src/estimators/state_estimator.cpp`(基底)
- **ただし sim モードでは、このフィルタは制御に使われない**。
  `robot_driver` は sim では `state/ground_truth` を購読してそのまま制御状態に
  使い、`updateState()` の sim 分岐は推定器を呼ばない。`state/estimate` は
  sim では publish されない(EKF はデバッグ用に並走できるだけで既定 off)。
  詳細は `agent_reports/quadsdk_step01_sensing.md`。

### 5.3 接地・外力

- `contact_state_publisher_node`
  - パッケージ: `quad_simulator/gazebo_scripts`
  - 設計上の入力は Gazebo 形式トピック(`gazebo/<toe>_contact_states`)。
    MuJoCo では供給されない可能性が高く、機能しているか未確認
    (`agent_reports/quadsdk_step01_sensing.md` の【推測】参照)
  - 歩行で使われる接地は実質プランナ/コントローラ側の接地スケジュール由来
- `body_force_estimator_node`
  - パッケージ: `body_force_estimator`(`planning.py` で起動)
  - 出力: `quad_msgs/BodyForceEstimate`(運動量残差ベースの外力推定)
  - 用途: robot_driver での外乱補償

## 6. [LOCAL PLANNER] 局所プランナ + フットステップ

- launch: `planning.py: launch_local_planner()`
- ノード: `local_planner` パッケージ、exec `local_planner_node`、name `local_planner`
- パラメータ: `local_planner.yaml` + `nmpc_controller.yaml` + `local_planner_topics.yaml`
  + `go2.yaml`
- 入出力トピック(`local_planner/config/local_planner_topics.yaml`):
  - 入力
    - `terrain_map`(4節)
    - `state/ground_truth`(5.1)
    - `cmd_vel`
    - `control/mode`
    - `global_plan`(Step 01 では来ない)
  - 出力
    - **`local_plan`**(`quad_msgs/RobotPlan`)
    - `foot_plan_discrete`
    - `foot_plan_continuous`

ソース構成:

- `local_planner/src/local_planner_node.cpp`
  - ノードのエントリ・タイマループ
- `local_planner/src/local_planner.cpp`
  - 参照軌道(胴体)をホライズン分構築し NMPC を呼ぶ
  - `use_twist_input=true` のとき `cmd_vel` を積分して参照姿勢/速度を作る
- `local_planner/src/local_footstep_planner.cpp`
  - 地形マップと歩容位相から次の着地点(離散フットホールド)と連続遊脚軌道を生成

`local_plan`(`RobotPlan`)には胴体状態の時系列・各脚の GRF 系列・足先計画・
`compute_time`・`diagnostics.iterations`(IPOPT 反復数)・`diagnostics.cost` が入る。

> NMPC 解が失敗した tick では `publishLocalPlan()` が呼ばれず `local_plan` が
> publish されない。`quadsdk_step01_baseline.py` の `plan_age_s` はこの欠落を
> 「直近成功からの経過秒」として間接可視化している(`quadsdk_step01_baseline.py:155-173`)。

## 7. [MPC] NMPC(非線形モデル予測制御)

> 理論式・コスト・制約・IPOPT 設定・go2 の数値パラメータの詳細は
> `agent_reports/quadsdk_step01_mpc.md`。

- **独立ノードではない。** `nmpc_controller/CMakeLists.txt` は
  `add_library(nmpc_controller ...)`。`local_planner_node` にリンクされ、
  `local_planner.cpp` から関数呼び出しされるライブラリ
- ソース:
  - `nmpc_controller/src/nmpc_controller.cpp`
    - NMPC のラッパ。参照軌道・現在状態・接地スケジュールを受けて NLP を構成し、解を返す
  - `nmpc_controller/src/quad_nlp.cpp`
    - Ipopt の `TNLP` 実装(コスト・制約・ヤコビアン・ヘシアン)
    - `quad_nlp.cpp:1338` の `ip_data->iter_count()` が `diagnostics.iterations` の出所
  - `nmpc_controller/src/gen/`
    - 自動生成された微分コード
- ソルバ:
  - **IPOPT**
  - 線形ソルバは `nmpc_controller.cpp` で `"ma27"` → `"mumps"` に変更済み
    (HSL 未導入のため。handoff 5節)
  - `print_level` は診断用に `0` → `5` のまま
- 出力:
  - 予測ホライズンにわたる最適 GRF 系列と胴体状態軌道
  - これを `local_planner.cpp` が `RobotPlan` に詰めて `local_plan` として publish

モデルは単剛体(centroidal)ダイナミクス + 接触点での摩擦錐制約が中心
(`quad_nlp.cpp` 参照)。関節レベルの全身動力学は解かない — そこは次段の WBC 相当。

## 8. [WBC 相当] レッグコントローラ(robot_driver)

> 逆動力学の式(接地脚 `-Jᵀf` / 遊脚 KKT 系)・最終トルク合成・go2 のゲインの
> 詳細は `agent_reports/quadsdk_step01_wbc.md`。

Quad-SDK には OCS2/legged_control のような独立した階層型 WBC ノードは**ない**。
相当するのは `robot_driver_node` 内で選択されるレッグコントローラ。

- launch: `quad_mujoco_bringup.py: launch_robot_driver()` → `robot_driver.py`
  → `robot_driver` パッケージ exec `robot_driver_node`、name `robot_driver`
- コントローラ選択:
  - `robot_configs` の `"controller": "inverse_dynamics"`
    (`run_quadsdk_step01_baseline.sh:81`)
- 実装候補(`robot_driver/src/controllers/`):
  - `inverse_dynamics_controller.cpp` ← Step 01 で使うもの
  - `grf_pid_controller.cpp`
  - `leg_controller.cpp`(基底)
  - `joint_controller.cpp`(これは別物、→ 9節)
  - `learned_policy.cpp` など
- 入力:
  - `local_plan`(6節)
  - `state/ground_truth` または `state/estimate`(5節)
  - 接地状態(5.3)
  - `control/mode`(10節)
- 処理(`inverse_dynamics_controller.cpp`):
  - **支持脚**: `local_plan` の GRF を脚ヤコビアン転置 `Jᵀ` で関節トルクに写像
  - **遊脚**: `local_plan` の連続遊脚軌道を逆動力学 + 関節 PD で追従
  - `control/mode` に応じて STAND(ノミナル姿勢への PD)/ WALK(上記)を切替
- 出力:
  - `control/joint_command`(`quad_msgs/LegCommandArray`。各関節のトルク + 位置/速度目標 + ゲイン)
  - `control/grfs`(`quad_msgs/GRFArray`。実際に適用している GRF)
    ← `quadsdk_step01_baseline.py` が読む

トピック名は `robot_driver/config/robot_driver_topics.yaml` /
`quad_utils/config/topics_robot.yaml`。

## 9. [ros2_control] `joint_controller`

- 8節の `robot_driver` が出す `control/joint_command`(トルク等)を、
  MuJoCo のアクチュエータへ実際に書き込む ros2_control コントローラ
- 実装: `robot_driver/src/controllers/joint_controller.cpp`
- 起動: `quad_mujoco_bringup.py: spawn_controller_broadcasters()`
  - `joint_state_broadcaster` → `joint_controller` の順に spawner をチェーン起動
  - `OnProcessExit` で直列化(多ロボット時のロック競合対策)
- **これが handoff の起立失敗の根本原因**:
  - 固定 sleep で `STAND` を送ると `joint_controller` がまだ `active` でなく、
    トルクが床に落ちてロボットが起立しないことがあった
  - `run_quadsdk_step01_baseline.sh:117-126` で
    `controller_manager_msgs/srv/ListControllers` をポーリングして
    `state='active'` を待つよう修正済み

## 10. モード状態機械 `control/mode`

`std_msgs/UInt8` を `/robot_1/control/mode` に publish する:

- `0` = SAFETY
  - robot_driver: トルク 0
- `1` = STAND
  - robot_driver: PD 制御でノミナル姿勢を保持
- `2` = WALK
  - robot_driver: `local_plan` 追従(8節の支持脚/遊脚処理)

シェルスクリプトの順序(`run_quadsdk_step01_baseline.sh`):

- `joint_controller` が active になるまで待つ(`:117-126`)
- ロガー起動(`:135-140`)
- `STAND`(mode=1)送信(`:147`)→ `STAND_SETTLE_S` 待機
- `quad_plan.py` 起動(local planner + NMPC。`:156`)→ `PLAN_STARTUP_S` 待機
- `WALK`(mode=2)送信(`:167`)
- `cmd_vel` を 50 Hz で `DURATION_S` 秒間送信(`:172-173`)

`STAND` を挟まず `WALK` だけ送ると起立前に歩行へ移行して不安定化する
(公式 tutorial の記述、`:144-145` のコメント)。

## 11. トピック早見表(`robot_1` 名前空間)

- `/mapping/terrain_map`(`grid_map_msgs/GridMap`) — MAP
  - publisher: `grid_map_filters`
  - subscriber: `terrain_map_relay`
- `terrain_map`(`grid_map_msgs/GridMap`) — MAP → PLAN
  - publisher: `terrain_map_relay`
  - subscriber: `local_planner`
- `state/ground_truth`(`quad_msgs/RobotState`) — SENSING
  - publisher: `mujoco_estimator`
  - subscriber: `local_planner` / `robot_driver` / **logger**
- `state/estimate`(`quad_msgs/RobotState`) — SENSING
  - publisher: `robot_driver`(comp_filter)
  - subscriber: `robot_driver` 制御側
- `cmd_vel`(`geometry_msgs/Twist`) — 参照入力
  - publisher: シェル(`ros2 topic pub`)
  - subscriber: `local_planner`
- `control/mode`(`std_msgs/UInt8`) — モード
  - publisher: シェル(`ros2 topic pub`)
  - subscriber: `local_planner` / `robot_driver`
- `local_plan`(`quad_msgs/RobotPlan`) — PLAN / MPC 出力
  - publisher: `local_planner`(+ NMPC)
  - subscriber: `robot_driver` / **logger**
- `foot_plan_discrete` / `foot_plan_continuous`(`quad_msgs/*`) — PLAN 出力
  - publisher: `local_planner`
  - subscriber: `robot_driver`
- `control/joint_command`(`quad_msgs/LegCommandArray`) — WBC 出力
  - publisher: `robot_driver`
  - subscriber: `joint_controller`
- `control/grfs`(`quad_msgs/GRFArray`) — WBC 出力
  - publisher: `robot_driver`
  - subscriber: **logger**

## 12. ソース早見表(`external/quad-sdk/`)

- world 展開
  - `quad_utils/launch/quad_mujoco.py`(`prepare_world`)
  - `quad_simulator/quad_sim_scripts/worlds/flat_wide.xml.xacro`(MPC_DOG 追加)
- 物理 + ros2_control
  - `mujoco_ros2_control`(vendored)
  - `quad_utils/launch/quad_mujoco.py`(`launch_mujoco_world`)
- MAP
  - `quad_utils/launch/mujoco_mapping.py`
  - `quad_utils/src/mjcf_to_grid_map_node.*`
  - `quad_utils/src/grid_map_filters_demo.*`
- SENSING(真値)
  - `quad_simulator/mujoco_plugins/src/mujoco_estimator.cpp`
- SENSING(フィルタ)
  - `robot_driver/src/estimators/comp_filter_estimator.cpp`
  - `robot_driver/src/estimators/ekf_estimator.cpp`
  - `robot_driver/src/estimators/state_estimator.cpp`
- SENSING(接地/外力)
  - `quad_simulator/gazebo_scripts/src/contact_state_publisher.cpp`
  - `body_force_estimator/`
- GLOBAL PLAN(Step 01 は無効)
  - `global_body_planner/src/global_body_planner_node.cpp` ほか
- LOCAL PLAN
  - `local_planner/src/local_planner_node.cpp`
  - `local_planner/src/local_planner.cpp`
  - `local_planner/src/local_footstep_planner.cpp`
- MPC
  - `nmpc_controller/src/nmpc_controller.cpp`(lib、local_planner にリンク)
  - `nmpc_controller/src/quad_nlp.cpp`
- WBC 相当
  - `robot_driver/src/robot_driver_node.cpp`
  - `robot_driver/src/controllers/inverse_dynamics_controller.cpp`
- ros2_control
  - `robot_driver/src/controllers/joint_controller.cpp`
- 録画
  - `quad_utils/src/mujoco_recorder_node.cpp`(camera_lookat_x/y は MPC_DOG 追加)

## 13. まとめ:`quadsdk_step01_baseline.py` は図のどこを見ているか

`quadsdk_step01_baseline.py` は上記パイプラインの**出力3本だけ**を
外から購読している:

- `state/ground_truth`(5.1、SENSING 出力)
  - 記録の主軸(base 位置・姿勢・速度)
- `local_plan`(6 / 7、LOCAL PLAN + MPC 出力)
  - NMPC 診断値(compute_time・iterations・cost)
- `control/grfs`(8、WBC 相当の出力)
  - 各脚の接地力・接地フラグ

map(4節)・センシングのフィルタ側(5.2)・local footstep planner の内部・
NMPC の NLP・WBC のトルク計算は**一切通らない**。だからこのファイルを読んでも
`map → sensing → MPC → WBC` の流れは見えない。流れを追うなら本ドキュメントの
2節(起動レイヤ)→ 12節(ソース早見表)の順で `external/quad-sdk` を読む。
