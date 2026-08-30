# Quad-SDK：正しいビルド方法(検証済み)

## 状態

`colcon build --symlink-install`が成功することを確認済み。

```text
Summary: 36 packages finished [9.41s]
  2 packages had stderr output: gazebo_scripts quad_utils   # 警告のみ、ビルド停止なし
```

`Failed`/`Aborted`/`not processed`のパッケージは無し。`source install/setup.bash`後、`ros2 pkg list`で`quad_utils`, `quad_msgs`, `nmpc_controller`, `local_planner`, `global_body_planner`, `gazebo_scripts`を含む主要パッケージが見えることを確認済み。

以前の版(このファイルの旧内容)はデバッグ途中のAgent引き継ぎ資料でしたが、原因が判明し解決したため、このファイルは**手順書として書き直しました**。

## 前提

- Ubuntu 24.04
- ROS 2 Jazzy(apt、システムにインストール済み)。手順は`quad_sdk_install_ubuntu24_ros2_jazzy.md`参照
- `/home/takuya/work/mpc_dog/external/quad-sdk`にQuad-SDK本体(`devel_ros2_review`、commit `a3591a9f`)がsubmodule込みでclone済み
- `/home/takuya/work/mpc_dog/ros2_ws/src/quad_sdk`が`external/quad-sdk`へのシンボリックリンクとして存在

## 正しい手順

### 1. clone(submodule込み)

```bash
# 目的：Quad-SDK本体とsubmoduleを一度に取得する(submoduleだけ後から個別取得しない)
cd /home/takuya/work/mpc_dog/external
git clone --recurse-submodules https://github.com/robomechanics/quad-sdk.git quad-sdk
cd quad-sdk
git checkout a3591a9f9e84aa9be3534ee0be107f0829ceb868
git submodule sync --recursive
git submodule update --init --recursive
```

`--recurse-submodules`を付けずにcloneし、submoduleを後から個別に取得しようとすると、`external/rbdl-orb`や`external/unitree_sdk2`が空/中途半端な状態のまま残り、`setup.sh`内のRBDL・Unitree SDK2ビルドが`CMake Error: ... does not appear to contain CMakeLists.txt`で失敗する。**cloneをやり直す(submodule込みで最初から)のが正しい対処**であり、個別ビルド失敗を無視して進めるのは正攻法ではない。

### 2. setup.sh実行時の注意(rosdepのスキャン範囲)

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
cd /home/takuya/work/mpc_dog/external/quad-sdk
./setup.sh
```

`setup.sh`の最終行は`rosdep install --from-paths .. --ignore-src -r -y --rosdistro jazzy`で、`..`はquad-sdkの親、つまり`external/`全体を指す。このリポジトリでは`external/`直下に無関係な他の外部実装(`Quadruped-PyMPC`など)が同居しているため、同名パッケージが複数見つかりrosdepがクラッシュすることがある(例: `dls2_interface`が`Quadruped-PyMPC`と`Quadruped-PyMPC.zip-backup`の両方に存在し衝突)。

その場合は最後のrosdepだけ、スキャン範囲をquad-sdk自身に絞って手動で実行する。

```bash
# 目的：quad-sdk自身のpackage.xmlだけを対象にrosdepを実行し、external/内の無関係な他リポジトリを巻き込まない
cd /home/takuya/work/mpc_dog/external/quad-sdk
rosdep install --from-paths . --ignore-src -r -y --rosdistro jazzy
```

`setup.sh`自体は変更していない。

### 3. ワークスペースのシンボリックリンク

```bash
mkdir -p /home/takuya/work/mpc_dog/ros2_ws/src
ln -s /home/takuya/work/mpc_dog/external/quad-sdk /home/takuya/work/mpc_dog/ros2_ws/src/quad_sdk
```

### 4. quad_utilsのCMake修正(必須。この修正無しではPinocchio関連でビルド不可)

`external/quad-sdk/quad_utils/CMakeLists.txt`と`package.xml`に、以下の変更が入っている必要がある(このリポジトリでは適用済み)。

**問題**: 元の`quad_utils/CMakeLists.txt`は`find_package(pinocchio REQUIRED)`と`ament_target_dependencies(quad_utils ... pinocchio)`はあったが、`pinocchio::pinocchio`という現代的なCMake importedターゲットへ直接リンクしておらず、`quad_utils`を使う下流パッケージ(`gazebo_scripts`等)へPinocchioの必要な設定(includeパス等)が伝播しなかった。結果、`quad_utils`単体は通っても、`gazebo_scripts`が`quad_utils`の公開ヘッダー(`ros_utils.hpp`, `quad_kd2.hpp`)をincludeした時点で`fatal error: pinocchio/multibody/model.hpp: No such file or directory`で失敗していた。

**修正内容**:

```cmake
# quad_utils自身と、quad_utilsを使う後続パッケージへPinocchioの使用条件を渡す
target_link_libraries(quad_utils PUBLIC pinocchio::pinocchio)

# quad_utilsの公開ヘッダーが使用するROS依存関係を後続パッケージへ渡す
ament_target_dependencies(quad_utils PUBLIC
  rclcpp
  std_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  grid_map_core
  grid_map_ros
  grid_map_pcl
  grid_map_msgs
  quad_msgs
  pcl_msgs
  pcl_conversions
  Eigen3
  tf2
  tf2_geometry_msgs
  cv_bridge
)
# ↑ pinocchioは上の一覧から外し、target_link_libraries(... pinocchio::pinocchio)だけで扱う

# quad_utilsを後続ROSパッケージからCMakeターゲットとして利用できる形でinstallする
install(TARGETS quad_utils
  EXPORT export_quad_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 後続パッケージへquad_utilsのCMakeターゲットを公開する
ament_export_targets(export_quad_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs nav_msgs nav2_msgs sensor_msgs geometry_msgs visualization_msgs
  grid_map_core grid_map_ros grid_map_pcl grid_map_msgs quad_msgs
  pcl_msgs pcl_conversions Eigen3 tf2 tf2_geometry_msgs cv_bridge pluginlib filters
  pinocchio
)
# 旧来の ament_export_libraries(quad_utils) は削除(importedターゲットのexportと併用しない)
```

`package.xml`には次を追加する。

```xml
<!-- rosdepにPinocchioがquad_utilsの依存パッケージであることを伝える -->
<depend>pinocchio</depend>
```

**検証方法**: `quad_utils`を単体ビルド後、exportされたCMake設定に`pinocchio::pinocchio`が含まれることを確認する。

```bash
rg -n 'INTERFACE_LINK_LIBRARIES' \
  /home/takuya/work/mpc_dog/ros2_ws/install/quad_utils/share/quad_utils/cmake/export_quad_utilsExport.cmake
# → "pinocchio::pinocchio;rclcpp::rclcpp;..." が含まれていればOK
```

### 5. 全体ビルド(Pythonインタプリタの明示指定が必要)

```bash
cd /home/takuya/work/mpc_dog/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

**`-DCMAKE_BUILD_TYPE=Release`**: 公式インストール手順が推奨するフラグ。指定しないとビルド種別未指定
(実質デバッグビルド相当)になり、リアルタイムNMPCループの実行速度で不利。

**`-DPython3_EXECUTABLE=/usr/bin/python3`が必須の理由**: このマシンでは`uv`が管理するスタンドアロンPython(`~/.local/bin/python3.11` → `~/.local/share/uv/python/...`)が`PATH`上で`/usr/bin`より先に来ている。CMakeの`find_package(Python3)`はPATH順に走査して最初に見つかった`python3.X`実行系を採用するため、何も指定しないと`~/.local/bin/python3.11`が選ばれてしまい、`catkin_pkg`が入っていない(ROS2用のPythonパッケージは`/usr/bin/python3`側にaptで入っている)ため`ModuleNotFoundError: No module named 'catkin_pkg'`でビルド全体が失敗する。`uv`側のpythonにROS2用パッケージをpipで足す方法は採らず、システムpython3を明示指定する。

### 6. 動作確認

```bash
source /home/takuya/work/mpc_dog/ros2_ws/install/setup.bash
ros2 pkg list | grep -E "^(quad_utils|quad_msgs|nmpc_controller|local_planner|global_body_planner|gazebo_scripts)$"
```

6パッケージ全て表示されればOK。

## 無視してよい警告

以下はビルド停止原因ではない。

```text
CMake Warning (dev) ... Policy CMP0144 is not set ...（FLANN_ROOTの大文字小文字の警告）
** WARNING ** io features related to pcap will be disabled
'teleop_twist_keyboard'/'teleop_twist_joy' is in: /opt/ros/jazzy （underlayとの重複警告）
```

## この時点の構成

```text
/home/takuya/work/mpc_dog/
├── external/
│   └── quad-sdk/              # Quad-SDK本体(quad_utilsにCMake修正あり)
└── ros2_ws/
    ├── src/quad_sdk           # external/quad-sdkへのシンボリックリンク
    ├── build/                 # colconのビルド生成物
    ├── install/                # ビルドしたROS 2環境(36パッケージ)
    └── log/                    # colconのビルドログ
```

## 公式ドキュメントとの差分(2026-08-30、robomechanics.github.io/quad-sdk/latest/getting-started/installation/ より)

- **CoinHSL未導入**: 公式は`external/ipopt/coinhsl/`にCoinHSLソースを配置しないと`nmpc_controller`が
  リンク時に失敗すると明記。このリポジトリではHSLが無くMUMPSにフォールバックしており、実際にリンク・
  ビルドは成功している。ただしMUMPSはHSLよりソルバー収束性能が劣る可能性があり、NMPCの挙動が
  おかしい場合はまずこれを疑うこと。CoinHSLは学術ライセンスで別途入手が必要(未対応)。
- **`-DCMAKE_BUILD_TYPE=Release`が公式推奨コマンドに含まれる**: 指定しないとビルド種別未指定(実質
  デバッグビルド相当)になり、リアルタイムNMPCループの実行速度で不利。5節のビルドコマンドに追加済み。
- **ワークスペース構成**: 公式は`~/ros2_ws/src/quad-sdk`単独配置が前提。このリポジトリでは
  `external/`に`Quadruped-PyMPC`等が同居しており、これが2節のrosdep衝突(`dls2_interface`)の
  根本原因だったことを裏付ける。対応(`.`にスコープを絞る)は変更不要。

## 禁止事項(今後も継続)

- Python 3.11(uv管理)へ`catkin_pkg`やROSパッケージをpipで追加しない。system Python環境へ`uv sync`しない。
- Pinocchioのinclude pathを個別の下流パッケージへハードコードしない(`quad_utils`のCMakeターゲットexport経由で伝播させる)。
- ビルドが完了するまで`external/quad-sdk/.git`とsubmoduleのGit情報を削除しない。
