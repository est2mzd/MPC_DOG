# Quad-SDK版：環境構築とStep 01前進歩行確認

## 0. 現在地

この文書は、一般的なQuad-SDKの紹介ではなく、`MPC_DOG`で実際に発生した問題と修正を反映した再現手順である。

```text
環境構築:            完了（ROS 2パッケージ36個をビルド済み）
Go2の自立:           確認済み
前進速度指令の反映:  確認済み
前進歩行:            確認済み（0.3 m/s指令、18.7秒で5.67 mの成功記録あり）
10 m正式合格:        未確認
標準Solver:          IPOPT + MUMPS
Coin-HSL / MA27:     使用しない
```

したがって、本書のStep 01は「一度も動いていないものを推測で動かす手順」ではない。一方、10 m以上を毎回転倒せず歩ける再現性までは確定していないため、短時間試験から順に確認する。

## 1. この文書の目的

Ubuntu 24.04／ROS 2 Jazzy環境でQuad-SDKを構築し、MuJoCo上のUnitree Go2へ前進速度指令を与えて、Step 01の基準動作を記録する。

対象プロジェクトは次の場所にあるものとする。

```text
/home/takuya/work/mpc_dog
```

Quad-SDKは次の版へ固定する。

```text
Repository: https://github.com/robomechanics/quad-sdk.git
Branch:     devel_ros2_review
Commit:     a3591a9f9e84aa9be3534ee0be107f0829ceb868
```

ビルドとStep 01確認が完了するまでは、`external/quad-sdk/.git`を削除しない。固定コミット、submodule、upstreamとの差分を確認するために必要である。自分のリポジトリへ完全に吸収する作業は、動作基準を保存した後の別作業とする。

## 2. Step 01の成功条件

正式な成功条件は次のとおりとする。

- ロボット：Go2
- シミュレータ：MuJoCo
- 参照入力：`reference="twist"`
- 前進速度指令：`0.3 m/s`
- 前進距離：`10 m以上`
- 転倒なし
- NMPCの連続失敗なし
- 同じ条件で再実行できること

理論上、`0.3 m/s`で10 m進むには最低でも約33.3秒かかる。立ち上がり時間を含め、正式試験では40秒以上の指令時間を使用する。

## 3. この環境で採用するSolver構成

本手順ではCoin-HSL／MA27を使用しない。

```text
Quad-SDK NMPC
  → IPOPT
  → MUMPS
```

理由は次のとおりである。

- 元コードはIPOPTの線形Solverに`ma27`を指定している。
- MA27はCoin-HSLに含まれ、対象PCには導入されていない。
- 無料Coin-HSLは個人ライセンスであり、チーム共有・再配布に向かない。
- MUMPSはこのPCのIPOPTで利用でき、実際にNMPCが解けることを確認済みである。
- MUMPSはCeCILL-Cライセンスで、条件を守れば再配布できる。

MUMPSで性能が不足した事実は、現時点では確認されていない。脚交換時の転倒原因をMUMPSと断定してはならない。

## 4. 前提環境

本手順は次の環境を前提とする。

| 項目 | 使用するもの |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy（aptでホストへインストール済み） |
| Python | `/usr/bin/python3` |
| Python環境 | ROS 2のビルド・実行にはuvを使わない |
| シミュレータ | MuJoCo |
| ロボットモデル | Go2 |
| Quad-SDK | `devel_ros2_review`固定コミット |

ROS 2 JazzyはUbuntuのsystem Pythonとaptパッケージを前提としている。プロジェクトの`.venv`には`rclpy`や`catkin_pkg`などが入っていないため、このROS 2ワークスペースを`uv run`でビルド・実行しない。

## 5. 環境構築

以下は新しくcloneする場合の手順である。すでに正常な`external/quad-sdk`がある場合はcloneをやり直さない。

### 5.1 Quad-SDKを取得する

# 目的：MPC_DOGのプロジェクトルートへ移動する

```bash
cd /home/takuya/work/mpc_dog
```

# 目的：Quad-SDK本体と必要なsubmoduleを同時に取得する

```bash
git clone --recurse-submodules \
  --branch devel_ros2_review \
  https://github.com/robomechanics/quad-sdk.git \
  external/quad-sdk
```

# 目的：調査済みのQuad-SDKコミットへ固定する

```bash
cd /home/takuya/work/mpc_dog/external/quad-sdk
git switch --detach a3591a9f9e84aa9be3534ee0be107f0829ceb868
```

# 目的：固定コミットが参照するsubmoduleを漏れなく取得する

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

この操作で、少なくとも次が取得される。

```text
external/mocap4ros2_optitrack
external/rbdl-orb
external/rbdl-orb/addons/urdfreader/thirdparty/urdfparser
external/unitree_sdk2
```

`rbdl-orb`や`unitree_sdk2`の空ディレクトリを先に手作業で作らない。空でない不完全なディレクトリが存在すると、submoduleのcloneが衝突する。

### 5.2 ROS 2ワークスペースへ配置する

# 目的：ROS 2がソースを探索する`ros2_ws/src`を作成して移動する

```bash
mkdir -p /home/takuya/work/mpc_dog/ros2_ws/src
cd /home/takuya/work/mpc_dog/ros2_ws/src
```

# 目的：`ros2_ws/src`から見た正しい相対パスでQuad-SDKを登録する

```bash
ln -sfnT ../../external/quad-sdk quad_sdk
```

ここで指定するリンク先は`./external/quad-sdk`ではない。リンクの基準位置は`ros2_ws/src`なので、正しい相対パスは`../../external/quad-sdk`である。

### 5.3 ROS 2と依存パッケージを準備する

# 目的：aptで導入したROS 2 Jazzyを現在のシェルで使用可能にする

```bash
source /opt/ros/jazzy/setup.bash
```

# 目的：既存のrosdepデータを更新する

```bash
rosdep update
```

`sudo rosdep init`で次が表示される場合、rosdepはすでに初期化されている。

```text
default sources list file already exists
```

この場合はファイルを削除せず、`rosdep update`だけを実行する。

# 目的：Quad-SDKの公式スクリプトでIpopt、ROS依存、RBDL、Unitree SDKなどを導入する

```bash
cd /home/takuya/work/mpc_dog/external/quad-sdk
chmod +x setup.sh
./setup.sh
```

`setup.sh`はQuad-SDKのリポジトリ直下で実行する。上位の`/home/takuya/work/mpc_dog`全体をrosdepへ渡すと、別リポジトリやバックアップ内の同名ROSパッケージまで検出する可能性がある。

`setup.sh`完了後に`/usr/local/lib`の共有ライブラリが見つからない場合に限り、次を検討する。エラーが出ていない段階で`.bashrc`へ追加する必要はない。

```bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
```

## 6. clean cloneへ必要なソース修正

この節は、固定した元コミットから変更する内容である。すでにMPC_DOG側で修正済みなら重ねて適用しない。

### 6.1 IPOPTの線形SolverをMA27からMUMPSへ変更する

# 目的：未導入のCoin-HSL／MA27ではなく、導入済みのMUMPSをIPOPTに使用させる

```bash
cd /home/takuya/work/mpc_dog/external/quad-sdk
sed -i \
  's/SetStringValue("linear_solver", "ma27")/SetStringValue("linear_solver", "mumps")/' \
  nmpc_controller/src/nmpc_controller.cpp
```

この変更がないと、コードは存在しないMA27を要求し、`NMPC solving fail`を繰り返す。

### 6.2 Pinocchioを下流ROSパッケージへ公開する

元の`quad_utils`公開ヘッダはPinocchioをincludeするが、CMakeターゲットとしての使用条件が下流へ十分に伝播しない。`quad_utils`単体がビルドできても、`gazebo_scripts`などで次のエラーが発生する。

```text
fatal error: pinocchio/multibody/model.hpp: No such file or directory
```

# 目的：Pinocchioのinclude／link条件と`quad_utils`ターゲットを下流パッケージへexportする

```bash
cd /home/takuya/work/mpc_dog/external/quad-sdk
git apply <<'PATCH'
diff --git a/quad_utils/CMakeLists.txt b/quad_utils/CMakeLists.txt
--- a/quad_utils/CMakeLists.txt
+++ b/quad_utils/CMakeLists.txt
@@ -151,8 +151,11 @@ target_link_libraries(rviz_interface_node
 #   ${tf2_geometry_msgs_LIBRARIES}
 # )
 
-ament_target_dependencies(quad_utils 
+target_link_libraries(quad_utils PUBLIC pinocchio::pinocchio)
+
+ament_target_dependencies(quad_utils PUBLIC
   rclcpp
   std_msgs
   nav_msgs
@@ -172,8 +175,7 @@ ament_target_dependencies(quad_utils 
   tf2
   tf2_geometry_msgs
-  cv_bridge
-  pinocchio
+  cv_bridge
 )
 
 ament_target_dependencies(mesh_to_grid_map_node rclcpp)
@@ -268,10 +270,12 @@ endif()
 
 # Install Libraries and Executables
-install(TARGETS quad_utils
+install(TARGETS quad_utils
+  EXPORT export_quad_utils
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
+  INCLUDES DESTINATION include
 )
 
 
@@ -312,8 +316,8 @@ install(
   DESTINATION share/${PROJECT_NAME}/rviz
 )
 
+ament_export_targets(export_quad_utils HAS_LIBRARY_TARGET)
 ament_export_include_directories(include)
-ament_export_libraries(quad_utils)
 ament_export_dependencies(
   rclcpp std_msgs nav_msgs nav2_msgs sensor_msgs geometry_msgs visualization_msgs
   grid_map_core grid_map_ros grid_map_pcl grid_map_msgs quad_msgs
diff --git a/quad_utils/package.xml b/quad_utils/package.xml
--- a/quad_utils/package.xml
+++ b/quad_utils/package.xml
@@ -27,6 +27,7 @@
   <depend>tf2_ros</depend>
   <depend>tf2_geometry_msgs</depend>
   <depend>eigen</depend>
+  <depend>pinocchio</depend>
 
   <depend>python_cmake_module</depend>
   <depend>rviz2</depend>
PATCH
```

`quad_utils/CMakeLists.txt`に`target_link_libraries(quad_utils PUBLIC pinocchio::pinocchio)`を二重に追加しない。

### 6.3 低速指令でも歩行へ移行できるようにする

Local Plannerは厳密な`>`比較で歩行開始を判定する。

```text
cmd_vel.norm() > stand_cmd_vel_threshold
```

速度指令と閾値がともに`0.1`の場合は歩行へ移行しない。

# 目的：0.1 m/s程度の低速試験でもSTANDからWALKへ移行できるようにする

```bash
cd /home/takuya/work/mpc_dog/external/quad-sdk
sed -i \
  's/stand_cmd_vel_threshold: 0.1/stand_cmd_vel_threshold: 0.05/' \
  local_planner/config/local_planner.yaml
```

Step 01の正式指令は`0.3 m/s`なので、この変更は0.3 m/s試験の必須条件ではない。ただし低速スモークテストとの一貫性を保つために適用する。

### 6.4 Step 01記録用のプロジェクトファイル

次のファイルはQuad-SDK upstreamではなく、MPC_DOG側で作成したStep 01用コードである。

```text
scripts/trial/run_quadsdk_step01_baseline.sh
src/trial/quadsdk_step01_baseline.py
scripts/trial/make_gif.sh
```

また、10 m試験で地面の端から落ちないよう、次の広い平面地形を使用する。

```text
external/quad-sdk/quad_simulator/quad_sim_scripts/worlds/flat_wide.xml.xacro
external/quad-sdk/quad_simulator/quad_sim_scripts/models/flat_wide/meshes/flat_wide.ply
```

これらは既存のMPC_DOGでソース管理する。環境構築のたびにシェルコマンドで再生成しない。

## 7. Quad-SDKをビルドする

### 7.1 初回の全体ビルド

# 目的：ROS 2ワークスペースへ移動する

```bash
cd /home/takuya/work/mpc_dog/ros2_ws
```

# 目的：ROS 2 Jazzyのsystem Python環境を有効にする

```bash
source /opt/ros/jazzy/setup.bash
```

# 目的：古いPython選択のCMakeキャッシュを破棄し、Release設定で全パッケージをビルドする

```bash
colcon build \
  --symlink-install \
  --cmake-clean-cache \
  --executor sequential \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release
```

`--executor sequential`は必須ではないが、PinocchioとEigenのコンパイル時メモリ使用量を抑え、失敗時の原因を追いやすくするため初回ビルドでは使用する。

`RBDL`や`unitree_sdk2`に対して`Python3_EXECUTABLE`が未使用という警告が出ても、それ自体はビルド失敗ではない。

### 7.2 ビルドした環境を有効にする

# 目的：ビルドしたQuad-SDKのROS 2パッケージとライブラリを現在のシェルへ追加する

```bash
source /home/takuya/work/mpc_dog/ros2_ws/install/setup.bash
```

### 7.3 NMPCコードを変更した場合の再ビルド

`nmpc_controller`は静的ライブラリとして`local_planner_node`へリンクされる。`nmpc_controller`だけをビルドしても、Local Plannerの実行ファイルには変更が反映されない。

# 目的：NMPCの変更と、それを静的リンクするLocal Plannerを一緒に再ビルドする

```bash
cd /home/takuya/work/mpc_dog/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build \
  --symlink-install \
  --packages-select nmpc_controller local_planner \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release
```

## 8. Step 01実行スクリプトが行う処理

`scripts/trial/run_quadsdk_step01_baseline.sh`は、次の順番で処理する。

1. ROS 2 Jazzyと`ros2_ws/install/setup.bash`をsourceする。
2. MuJoCoとGo2を起動する。
3. Joint Controllerの起動を待つ。
4. CSV記録ノードを起動する。
5. `control/mode=1`を送信し、STANDで立たせる。
6. `reference="twist"`でLocal Plannerを起動する。
7. `control/mode=2`を送信し、WALKへ切り替える。
8. `/robot_1/cmd_vel`へ前進速度を50 Hzで送信する。
9. 指定時間後に停止指令を送る。
10. CSVと動画を保存する。
11. `trap cleanup EXIT`で起動した子プロセスを終了する。

重要な点は次のとおりである。

- `reference="twist"`でなければ`cmd_vel`は使用されない。
- STANDのままでは歩かないため、WALKモード`2`が必要である。
- upstreamの`cmd_vel_publisher_node`は今回の試験で正常に指令を反映しなかったため、`ros2 topic pub -r 50`を使用する。
- `/clock`開始時の時刻ジャンプを避けるため、記録時間の管理は壁時計、`sim_time_s`はメッセージの`header.stamp`から計算する。
- GRF未受信の最初の行でもCSV列が変化しないよう、全脚の接触・GRF列を常に作成する。

## 9. Step 01を実行する

### 9.1 短時間スモークテスト

最初から40秒試験を行わず、5秒で起動・自立・初動を確認する。

# 目的：0.3 m/sの速度指令がPlannerへ入り、Go2が歩き始めることを短時間で確認する

```bash
cd /home/takuya/work/mpc_dog
DURATION_S=5 FORWARD_VEL_MPS=0.3 \
  bash scripts/trial/run_quadsdk_step01_baseline.sh
```

この段階では10 m到達を判定しない。次を確認する。

- Go2がSTAND姿勢を確立する。
- WALKへ切り替わる。
- 前方へ動き始める。
- 起動直後に転倒しない。
- `NMPC solving fail`が連続しない。
- 実行後にMuJoCoやPlannerのプロセスが大量に残らない。

### 9.2 中間確認

# 目的：複数回の脚交換を含む15秒間、前進歩行が継続することを確認する

```bash
cd /home/takuya/work/mpc_dog
DURATION_S=15 FORWARD_VEL_MPS=0.3 \
  bash scripts/trial/run_quadsdk_step01_baseline.sh
```

### 9.3 正式な10 m試験

# 目的：Step 01の正式条件である0.3 m/s・10 m以上の連続前進を試験する

```bash
cd /home/takuya/work/mpc_dog
DURATION_S=40 FORWARD_VEL_MPS=0.3 \
  bash scripts/trial/run_quadsdk_step01_baseline.sh
```

40秒で距離が不足する場合は、起動時のランプアップ時間を考慮して45秒へ延長する。

# 目的：正式試験の距離と転倒時刻を集計CSVで確認する

```bash
cd /home/takuya/work/mpc_dog
tail -n 2 artifacts/logs/quadsdk_step01/trials_summary.csv
```

合格行は少なくとも次を満たす。

```text
velocity_mps        = 0.3
walk_dist_x_m       >= 10.0
fall_time_s         = 空欄
```

前進距離は`0.3 × 実行時間`という指令上の理論値ではなく、`state/ground_truth`の最終x位置と初期x位置の差で判定する。

## 10. 出力ファイル

```text
artifacts/logs/quadsdk_step01/
├── state_log.csv
├── trials_summary.csv
└── logs/
    └── mujoco_go2_*.mp4
```

`state_log.csv`は毎回上書きされるため、条件比較に残す場合は次の試験を始める前に別名で保存する。

記録する主要項目は次のとおりである。

- base位置・姿勢・速度
- 4脚の接触状態
- 4脚のGRF
- Local Planの更新時刻
- NMPC計算時間
- NMPC反復回数
- NMPCコスト
- 転倒判定時刻

## 11. 現時点で確認できている結果

### 確認済みの事実

- Quad-SDKの36パッケージはビルド成功している。
- MA27からMUMPSへ変更後、連続していたNMPC失敗は解消した。
- Go2は安定して自立できるようになった。
- 生の`ros2 topic pub -r 50`で`cmd_vel`を送ると前進した。
- `0.3 m/s`指令の1試行で、18.7秒間に5.67 m前進し、転倒しなかった。
- 5.67 mは`0.3 × 18.7 = 5.61 m`に近く、速度指令が歩行へ反映されている。

### 正式合格として未確認の事項

- 提供された記録には、`0.3 m/s`で10 m以上を転倒なしで完走した確定ログがない。
- 40秒試験では、最初の脚交換付近で転倒した試行が記録されている。
- 同一条件での再現性は確定していない。

したがって、**前進動作は確認済みだが、Step 01の10 m正式条件は、`trials_summary.csv`で改めて合格を確認する必要がある**。

## 12. 問題が起きた場合の切り分け順序

### 12.1 完全に動かない

次を順番に確認する。

1. `reference="twist"`で起動しているか。
2. `/robot_1/cmd_vel`へ実際にメッセージが流れているか。
3. `control/mode=2`が送られているか。
4. Local Planが更新されているか。

### 12.2 立つが歩かない

次を確認する。

- STANDではなくWALKへ切り替わっているか。
- upstreamの`cmd_vel_publisher_node`ではなく、確認済みの`ros2 topic pub`を使っているか。
- `stand_cmd_vel_threshold`を超える速度指令になっているか。
- `cmd_vel`の受信後2秒以内に更新が続いているか。

### 12.3 NMPCが毎回失敗する

最初に、Solverが`ma27`のままになっていないか確認する。Coin-HSL未導入環境でMA27を指定すると解けない。

また、MUMPSへの変更後は`nmpc_controller`だけでなく`local_planner`も再ビルドする。

### 12.4 最初の脚交換付近で転倒する

MUMPSの性能不足と即断しない。次のデータを同じ時刻軸で確認する。

- 接触脚の切り替わり
- 各脚GRF、特にz方向
- baseのz、roll、pitch
- Local Planの更新間隔
- NMPC計算時間、反復回数、終了状態
- 実際の接触状態とPlannerの接触予定
- 起動順序とSTANDからWALKへ切り替えた時刻

原因候補には、起動タイミング、接触状態の不一致、GRFの急変、状態初期化、ゲイン・制約、MuJoCo側の接触なども含まれる。MA27との比較は、これらを記録した後に行う。

## 13. upstreamからの主要変更一覧

| ファイル | 変更 | 理由 |
|---|---|---|
| `nmpc_controller/src/nmpc_controller.cpp` | `ma27`→`mumps` | Coin-HSLなしでNMPCを解く |
| `quad_utils/CMakeLists.txt` | Pinocchioと`quad_utils`ターゲットをPUBLIC export | 下流パッケージのinclude失敗を修正 |
| `quad_utils/package.xml` | `pinocchio`依存追加 | rosdepと下流依存を明示 |
| `local_planner/config/local_planner.yaml` | 歩行開始閾値`0.1`→`0.05` | 低速試験で厳密な`>`判定を通す |
| `quad_utils/launch/quad_visualization.py` | RViz既定起動を無効化 | ヘッドレス記録では不要 |
| `quad_utils/launch/quad_mujoco.py` | カメラ設定引数を追加 | 固定カメラで移動量を視認する |
| `quad_utils/src/mujoco_recorder_node.cpp` | カメラ注視点を追加 | 録画位置を調整する |
| `flat_wide.xml.xacro` | 広い平面ワールド追加 | 10 m試験で地面端から落ちるのを防ぐ |
| `flat_wide.ply` | 対応する地形Mapメッシュ追加 | Plannerへ広い平面Mapを渡す |

## 14. 最終方針

Step 01では次を固定する。

```text
Global Body Planner: 使用しない
reference:           twist
制御モード:          WALK = 2
速度Publish:         ros2 topic pub -r 50
NMPC:                Quad-SDK独自実装
非線形Solver:        IPOPT
線形Solver:          MUMPS
Python:              /usr/bin/python3
地形:                flat_wide
記録:                CSV + MP4
```

まず5秒、次に15秒、最後に40～45秒へ延長する。正式合格は動画上の印象ではなく、`trials_summary.csv`の実測前進距離と転倒時刻で判定する。

## 15. 参考資料

- Quad-SDK installation: https://robomechanics.github.io/quad-sdk/latest/getting-started/installation/
- Quad-SDK repository: https://github.com/robomechanics/quad-sdk
- MUMPS license: https://mumps-solver.org/index.php?page=dwnld
- Ipopt documentation: https://coin-or.github.io/Ipopt/
- HSL licensing: https://www.hsl.rl.ac.uk/licensing.html
