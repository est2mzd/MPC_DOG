# Quad-SDK：cloneからcolcon buildまで

## 目的

Quad-SDKの`devel_ros2_review`を固定コミットで取得し、submoduleと依存ライブラリをインストールする。その後、MPC_DOG内のROS 2ワークスペースからQuad-SDKを参照してビルドする。

## 前提

- Ubuntu 24.04
- ROS 2 Jazzyインストール済み
- `/home/takuya/work/mpc_dog/external/quad-sdk`がまだ存在しない
- `/home/takuya/work/mpc_dog/ros2_ws/src/quad_sdk`がまだ存在しない

## 実行コマンド

```bash
# 目的：Quad-SDKを配置するexternalディレクトリへ移動する
cd /home/takuya/work/mpc_dog/external

# 目的：devel_ros2_reviewブランチと全submoduleを取得する
git clone --branch devel_ros2_review --recurse-submodules https://github.com/robomechanics/quad-sdk.git quad-sdk

# 目的：取得したQuad-SDKのルートへ移動する
cd /home/takuya/work/mpc_dog/external/quad-sdk

# 目的：Quad-SDK本体を確認済みコミットへ固定する
git checkout a3591a9f9e84aa9be3534ee0be107f0829ceb868

# 目的：固定コミットに対応するsubmodule設定を反映する
git submodule sync --recursive

# 目的：固定コミットに対応する全submoduleを取得する
git submodule update --init --recursive

# 目的：ROS 2 Jazzyを現在のターミナルで有効にする
source /opt/ros/jazzy/setup.bash

# 目的：ROS依存パッケージの対応情報を更新する
rosdep update

# 目的：Quad-SDKが必要とするapt、Ipopt、Unitree SDK、ROS依存パッケージをインストールする
./setup.sh

# 目的：MPC_DOG内にROS 2標準構成のソースディレクトリを作成する
mkdir -p /home/takuya/work/mpc_dog/ros2_ws/src

# 目的：シンボリックリンクを作成するディレクトリへ移動する
cd /home/takuya/work/mpc_dog/ros2_ws/src

# 目的：ROS 2ワークスペースからQuad-SDK本体を絶対パスで参照できるようにする
ln -s /home/takuya/work/mpc_dog/external/quad-sdk quad_sdk

# 目的：colconの生成物をros2_ws配下へ出力するためワークスペースルートへ移動する
cd /home/takuya/work/mpc_dog/ros2_ws

# 目的：colconがROS 2 Jazzyのビルド環境を使用できるようにする
source /opt/ros/jazzy/setup.bash

# 目的：Quad-SDK全体をビルドしてros2_ws/build、install、logへ出力する
# -DPython3_EXECUTABLE: uv管理のpython3.11がPATH上で/usr/binより先に見つかりcatkin_pkg不足で
#   失敗するため、ROS2用パッケージが入っているシステムpython3を明示指定する
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 目的：ビルドしたQuad-SDKを現在のターミナルから使用可能にする
source /home/takuya/work/mpc_dog/ros2_ws/install/setup.bash
```

## この時点の構成

```text
/home/takuya/work/mpc_dog/
├── external/
│   └── quad-sdk/              # Quad-SDK本体
└── ros2_ws/
    ├── src/
    │   └── quad_sdk           # external/quad-sdkへのシンボリックリンク
    ├── build/                 # colconのビルド生成物
    ├── install/               # ビルドしたROS 2環境
    └── log/                   # colconのビルドログ
```
