# Quad-SDK Step 01: 元リポジトリからの変更点と実行方法

対象: `external/quad-sdk`(quad-sdk本体)。詳細な調査経緯は
`agent_reports/step01/quad_sdk_step01_investigation.md`を参照。本ドキュメントは要点のみ。

## 1. 元リポジトリ(`external/quad-sdk`)から変更する必要があったもの

- **`nmpc_controller/src/nmpc_controller.cpp`**
  - 変更: `linear_solver`を`"ma27"`→`"mumps"`
  - 理由: HSL(CoinHSL)ライブラリが本環境に未導入で、ma27を使うとNMPCが解けず即座に転倒する。MUMPSはIPOPT同梱のフォールバックソルバー

- **`local_planner/config/local_planner.yaml`**
  - 変更: `stand_cmd_vel_threshold`を`0.1`→`0.05`
  - 理由: コード側(`local_planner.cpp:331`)が`cmd_vel_.norm() > stand_cmd_vel_threshold_`という厳密な`>`比較のため、0.1 m/s指令時に閾値と同値になり歩行へ移行できなかった

- **`quad_utils/launch/quad_visualization.py`**
  - 変更: `rviz`引数の既定値を`true`→`false`
  - 理由: ヘッドレス記録時に不要なrviz2の起動を止める(`rviz:=true`を明示すれば従来通り起動可)

- **`quad_utils/launch/quad_mujoco.py`**
  - 変更: `camera_track_robot`・`camera_distance`・`camera_lookat_x`・`camera_lookat_y`をlaunch引数として追加
  - 理由: 録画カメラをロボットに追従させず固定できるようにする(追従カメラだと前進していても画面上は常に「その場」に見えてしまう)。距離・注視点も調整可能に

- **`quad_utils/src/mujoco_recorder_node.cpp`**
  - 変更: 上記`camera_lookat_x/y`パラメータを新規追加(既存の`camera_distance`等はパラメータ自体は元から存在)
  - 理由: 同上。**C++変更のため`colcon build --packages-select quad_utils`の再ビルドが必要**

- **`quad_simulator/quad_sim_scripts/worlds/flat_wide.xml.xacro`(新規ファイル)**
  - 内容: `flat.xml`と同じ単純な直方体プリミティブ地面のまま、範囲をx∈[-3,15]・y∈[-5,5]に拡大。5m間隔の目盛り線(衝突判定なし)も追加
  - 理由: 既定の`flat.xml`は地面が9m弱しかなく10m規模の歩行試験で端から落ちる。より詳細なメッシュ地形`big_flat.xml`は原因不明の不安定化を招いたため、単純な形状のまま拡大する方針にした

- **`quad_simulator/quad_sim_scripts/models/flat_wide/meshes/flat_wide.ply`(新規ファイル)**
  - 内容: 地形マップ生成ノード用のメッシュ(`flat.ply`と同じ8頂点直方体フォーマットで座標のみ書き換え)
  - 理由: 上記ワールドに対応する地形マップ入力

- **`quad_utils/CMakeLists.txt`, `quad_utils/package.xml`**
  - 変更: Pinocchioのリンク・rosdep依存関係をCMakeターゲットとして正しくexportするよう修正
  - 理由: ビルド時のリンクエラー修正(quad_sdk_build_agent_handoff.md参照、Step 01検証より前の準備作業)

**変更していないもの**: 上記以外の制御ロジック(MPC本体、WBC、歩容生成、状態推定等)は一切変更していない。

## 2. 実行に必要なコード(MPC_DOG側、新規作成)

- **`scripts/trial/run_quadsdk_step01_baseline.sh`**
  メイン実行スクリプト。MuJoCo起動→`joint_controller`起動待ち→CSV記録開始→STAND→歩行プランナ起動→WALK→cmd_vel送信、までの一連の流れを実行し、後片付けまで行う

- **`src/trial/quadsdk_step01_baseline.py`**
  上記から呼ばれるROS2ノード。`state/ground_truth`・`control/grfs`・`local_plan`を購読し、CSVへ記録する

- **`scripts/trial/make_gif.sh`**
  録画mp4をGIFへ変換する補助スクリプト。時刻(小数点1桁)とファイル名を画面に焼き込む

### 実行方法

```bash
cd /home/takuya/work/mpc_dog
DURATION_S=15 FORWARD_VEL_MPS=0.5 bash scripts/trial/run_quadsdk_step01_baseline.sh
```

- `FORWARD_VEL_MPS`: 前進速度指令[m/s](既定0.3)
- `DURATION_S`: cmd_velを送り続ける時間[秒](既定10.0)

出力:
- CSV: `artifacts/logs/quadsdk_step01/state_log.csv`(毎回上書きされるため、複数速度を試す場合は都度コピーすること)
- 録画mp4: `artifacts/logs/quadsdk_step01/logs/mujoco_go2_*.mp4`

GIF化:
```bash
bash scripts/trial/make_gif.sh <入力mp4> <出力gif> [fps] [幅px]
```

### 前提条件

- `chatgpt_instruction/quad_sdk_build_agent_handoff.md`の手順でquad-sdkがビルド済みであること
- 上記C++変更(`mujoco_recorder_node.cpp`)を反映させるには
  `colcon build --symlink-install --packages-select quad_utils --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release`
  を実行済みであること(既に実施・確認済み)

### 実行前の注意(プロセス残留)

`trap cleanup`によるプロセス終了処理が完全ではなく、稀に子ノードが残留することを確認している。
実行前後で以下のパターンに一致するプロセスが残っていないか確認することを推奨:

```bash
pgrep -af "quad_mujoco|quad_plan|ros2_control_node|rviz2|mujoco_recorder|contact_state_publisher_node|mujoco_estimator|body_force_estimator_node|mjcf_to_grid_map_node|grid_map_filters_demo|nmpc_controller|local_planner_node|global_body_planner_node|rviz_interface_node|robot_driver_node|grid_map_visualization|topic_tools/relay|robot_state_publisher|static_transform_publisher|controller_manager/spawner"
```
