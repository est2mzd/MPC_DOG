# 状態推定 legged_estimation(StateEstimateBase・KalmanFilterEstimate・FromTopicStateEstimate) 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedController::updateStateEstimation(time, period)(read_code_05)が毎周期:
  → stateEstimate_->updateJointStates(jointPos, jointVel)  ← 本ファイル、毎周期
  → stateEstimate_->updateContact(contactFlag)              ← 本ファイル、毎周期
  → stateEstimate_->updateImu(quat, ...)                     ← 本ファイル、毎周期
  → stateEstimate_->update(time, period)                     ← 本ファイル、毎周期
      (既定は KalmanFilterEstimate::update、
       legged_cheater_controller選択時は FromTopicStateEstimate::update)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`legged_estimation`が担当するのは、「**関節角度・関節速度・接触フラグ・
IMU(姿勢・角速度・線形加速度)という生のセンサ値から、OCS2のセントロイダル
モデルが必要とする剛体状態(base位置・姿勢・速度・角速度+関節角度・
角速度、`rbdState_`)を推定して返す**」ことです。pympcには存在しない
コンポーネントです(pympcはMuJoCoの完全な内部状態をそのまま読み取れる
ため、推定という工程自体が不要でした)。歩容計画・MPC・WBCは一切
担当しません。

- `StateEstimateBase`：状態のデータ構造(`rbdState_`のレイアウト)、
  クォータニオン→ZYXオイラー角変換、odometry/pose配信という**共通基盤**
  を持つ抽象基底クラス
- `KalmanFilterEstimate`：**既定**(`LeggedController::setupStateEstimate`)
  で使われる、IMU加速度の積分と脚の順運動学(足先位置)を組み合わせた
  線形カルマンフィルタ
- `FromTopicStateEstimate`：`LeggedCheaterController`が使う、外部
  トピック(Gazeboの正解値等)をそのまま状態として使うデバッグ用実装
  (フィルタ処理なし)

対象は`external/legged_control/legged_estimation/include/legged_estimation/StateEstimateBase.h`
(75行)・`src/StateEstimateBase.cpp`(74行)、
`include/legged_estimation/LinearKalmanFilter.h`(62行)・
`src/LinearKalmanFilter.cpp`(270行)、
`include/legged_estimation/FromTopiceEstimate.h`(31行)・
`src/FromTopicEstimate.cpp`(33行)です。

---

## `StateEstimateBase` :`rbdState_`のレイアウト

```cpp
rbdState_(vector_t::Zero(2 * info_.generalizedCoordinatesNum))
```

| 区間(先頭からのオフセット) | 内容 | 単位 |
|---|---|---|
| `[0,3)` | base姿勢(ZYXオイラー角) | rad |
| `[3,6)` | base位置 | m |
| `[6, 6+actuatedDofNum)` | 関節角度(12個) | rad |
| `[gc, gc+3)`(`gc=generalizedCoordinatesNum`) | base角速度(global座標系) | rad/s |
| `[gc+3, gc+6)` | base並進速度(global座標系) | m/s |
| `[6+gc, 6+gc+actuatedDofNum)` | 関節角速度(12個) | rad/s |

- `generalizedCoordinatesNum`(型`size_t`)：base6自由度+関節12自由度=`18`
  と推測される(**未確認**、`legged_interface`で確定させる)。したがって
  `rbdState_`の全長は`2*18=36`程度と推測される
- この配置は`updateAngular`(姿勢・角速度)・`updateLinear`(位置・速度)・
  `updateJointStates`(関節角度・角速度)という3つの`protected`関数で
  部分的に書き込まれる。**IMU由来(姿勢・角速度)とハードウェア由来
  (関節)は毎周期直接書き込まれるが、位置・並進速度だけは`update()`
  (継承先の実装)がカルマンフィルタ等で計算してから書き込む**、という
  役割分担になっている

```cpp
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) { ... }
```

この関数の役割:クォータニオンをZYXオイラー角(yaw-pitch-roll)へ変換する、
このパッケージ全体で使われる共通ヘルパー関数。

- `zyx(1) = std::asin(as)`の`as`はジンバルロック回避のため`.99999`に
  クランプされている(**事実**、ジンバルロック(ピッチ±90度付近)での
  数値的な発散を防ぐ一般的な工夫)

---

## `StateEstimateBase.cpp` 26〜55行:`updateJointStates`・`updateImu`・`updateAngular`・`updateLinear`

```cpp
void StateEstimateBase::updateImu(...) {
  ...
  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}
```

- IMUの角速度(`angularVelLocal`、base座標系)を、いったん「ZYXオイラー角の
  時間微分」へ変換し、それを今度は「global座標系の角速度ベクトル」へ
  変換し直すという**2段階変換**(OCS2の`ocs2_robotic_tools`提供、外部)。
  なぜ一度オイラー角微分を経由するのかはコード上明確ではないが、
  OCS2のセントロイダルモデルが角速度をこの表現で扱う設計になっている
  ためと考えられる(**設計上の解釈**、`legged_interface`側で裏付けを取る)
- `zyxOffset_`(既定`vector3_t::Zero()`)：姿勢の基準オフセット。この
  ファイル内で書き換える箇所は無く、既定では常にゼロ(**実装上の注意点**、
  将来のキャリブレーション用に用意されているが未使用の可能性)

```cpp
void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom) {
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    ...
    if (odomPub_->trylock()) { ... }
  }
}
```

- `publishRate`(Hz)：`200`固定(ハードコード)。odometry(`/odom`)と
  pose(`/pose`)を最大200Hzに間引いてpublishする
- `realtime_tools::RealtimePublisher`(ROS標準、外部)の`trylock()`：
  リアルタイムスレッド(この状態推定処理自体は500〜800Hzのメインループ内)
  からブロッキングせずに安全にpublishするための仕組み。ロックが取れない
  周期はそのpublishをスキップする(**事実**、リアルタイム制約を優先する
  設計)

---

## `LinearKalmanFilter.h` 20〜60行:`KalmanFilterEstimate`クラス定義

主な設定値(すべて`task.info`の`kalmanFilter`ブロックから
`loadSettings`で上書きされる。**a1の実際の設定値はC++側のデフォルト値と
完全に一致**していることを`grep`で確認済み):

| メンバ | 型 | 単位 | 既定/実際の値(a1) |
|---|---|---|---|
| `footRadius_` | `scalar_t` | m | `0.02` |
| `imuProcessNoisePosition_` | `scalar_t` | 無次元(プロセスノイズの重み係数) | `0.02` |
| `imuProcessNoiseVelocity_` | `scalar_t` | 無次元 | `0.02` |
| `footProcessNoisePosition_` | `scalar_t` | 無次元 | `0.002` |
| `footSensorNoisePosition_` | `scalar_t` | 無次元 | `0.005` |
| `footSensorNoiseVelocity_` | `scalar_t` | 無次元 | `0.1` |
| `footHeightSensorNoise_` | `scalar_t` | 無次元 | `0.01` |

- `numContacts_`(型`size_t`)：`info_.numThreeDofContacts + info_.numSixDofContacts`
  (脚の数、4本なら`4`と推測、**未確認**、`legged_interface`で確認する)
- `numState_ = 6 + dimContacts_`(`dimContacts_ = 3*numContacts_`)：
  カルマンフィルタの状態次元。「base位置3+base速度3」+「接触点ごとの
  足先位置3」。4脚なら`6+12=18`
- `numObserve_ = 2*dimContacts_ + numContacts_`：観測次元。
  「足先の相対位置(3×4)+相対速度(3×4)+足先高さ(4)」。4脚なら`12+12+4=28`

---

## `LinearKalmanFilter.cpp` 51〜156行:`update`(カルマンフィルタ本体)

この関数の役割:IMU加速度を積分入力、脚の順運動学から得る足先の相対位置・
速度・高さを観測値とする線形カルマンフィルタを1ステップ実行し、
base位置・速度を推定する。

### 状態空間モデル

\[
\hat{x} = [\,p_{base}\ (3),\ v_{base}\ (3),\ p_{foot,1..n}\ (3n)\,]^\top
\]

\[
\hat{x}_{k+1} = A\hat{x}_k + B\,a_{IMU}
\]

| 記号 | コード変数 | 意味 |
|---|---|---|
| \(A\) | `a_` | 状態遷移行列。位置の行に`dt`だけ速度を積分する項が入る(等速度モデル+外部入力) |
| \(B\) | `b_` | IMU加速度から位置・速度への入力行列(`0.5*dt²`/`dt`の標準的な2次積分) |
| \(a_{IMU}\) | `accel` | IMU線形加速度をworld座標系へ回転し、重力`(0,0,-9.81)`を加えたもの |

```cpp
vector3_t g(0, 0, -9.81);
vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;
```

- **事実**：ここでは重力`-9.81`を**加算**している。
  [read_code_03](read_code_03_legged_hw_sim.md)で指摘した
  「GazeboのIMU実装(`LeggedHWSim::readSim`)が重力分を`RelativeLinearAccel`
  から**引き算**していた」という実装と、ここで**足し戻す**処理が
  ちょうど対応しています。つまりGazebo側は「センサが重力を含まない
  真の並進加速度を返す」という前提で実装されており、この状態推定側も
  同じ前提(重力を後から足して真の並進加速度に戻す)で一貫しています。
  [read_code_03](read_code_03_legged_hw_sim.md)で「未確認・要検証」と
  していた点は、**この状態推定コードと整合している**ことが確認できました
  (一般的なIMUの生出力(重力込み)とは逆の慣習ですが、このリポジトリ内では
  Gazebo側とこの推定側で一貫しているため、少なくとも自己矛盾は無いと
  言えます)

### 観測モデルと接触依存のノイズ切り替え

```cpp
ps_.segment(3 * i, 3) = -eePos[i];
ps_.segment(3 * i, 3)[2] += footRadius_;
vs_.segment(3 * i, 3) = -eeVel[i];
```

- `eePos[i]`/`eeVel[i]`(m、m/s)：Pinocchio(外部、剛体動力学ライブラリ)の
  順運動学で計算した、**base座標系から見た**足先の位置・速度
  (`qPino`のbase位置成分をゼロにして計算しているため、base相対値になる、
  `qPino.segment<3>(3) = rbdState_.head<3>(); // Only set orientation,
  let position in origin.`というコメント通り)
- `ps_ = -eePos`：base→足先ベクトルの符号を反転させ、「足先から見た
  baseの相対位置」を擬似観測値として使う(足が接地していれば、この値の
  変化=baseの移動、という脚オドメトリの原理)。Z成分だけ`footRadius_`
  (足先の球体半径、`0.02`m)を加算し、足の中心ではなく接地点の高さに
  補正している

```cpp
bool isContact = contactFlag_[i];
scalar_t high_suspect_number(100);
q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
```

**コードで確認した事実(このフィルタの核心となる設計)**：接触していない
脚(`isContact=false`)については、プロセスノイズ`q`・観測ノイズ`r`の
該当ブロックを**100倍**に膨らませます。カルマンフィルタでは観測ノイズが
大きいほど、その観測値をほとんど信用しなくなる(逆に予測を信用する)ため、
この処理は実質的に「**遊脚中の足先位置の観測(脚オドメトリ)は信用せず、
接地中の足だけを頼りにbase位置を推定する**」という、脚ロボットの状態
推定における標準的な手法です(pympcには対応する仕組みはありません。
pympcはMuJoCoの真の状態を直接読めるため、そもそも状態推定という工程が
不要でした)。

### カルマンフィルタの更新式

```cpp
xHat_ = a_ * xHat_ + b_ * accel;
matrix_t pm = a_ * p_ * a_.transpose() + q;
matrix_t ey = y - c_ * xHat_;
matrix_t s = c_ * pm * c_.transpose() + r;
vector_t sEy = s.lu().solve(ey);
xHat_ += pm * c_.transpose() * sEy;
p_ = (matrix_t::Identity(numState_, numState_) - pm * c_.transpose() * (s.lu().solve(c_))) * pm;
p_ = (p_ + p_.transpose()) / 2.0;
```

- 標準的な線形カルマンフィルタの予測(`xHat_ = a_*xHat_+b_*accel`、
  `pm = a_*p_*aᵀ+q`)と更新(カルマンゲイン相当の`s.lu().solve(...)`、
  状態・共分散の補正)。`s.lu().solve(ey)`は逆行列を直接計算せずLU分解で
  線形方程式を解く、数値的に安定な実装
- `p_ = (p_ + p_.transpose()) / 2.0`：共分散行列`p_`を強制的に対称化する
  (数値誤差で非対称になるのを防ぐ、カルマンフィルタ実装の定石)

```cpp
//  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
//    p_.block(0, 2, 2, 16).setZero();
//    p_.block(2, 0, 16, 2).setZero();
//    p_.block(0, 0, 2, 2) /= 10.;
//  }
```

- コメントアウトされたブロック。x-y位置の共分散(左上2×2)がある閾値を
  超えたら、他の状態量との相関を切り離してx-y位置の不確実性を縮小する、
  という追加の補正だったと推測されるが、無効化されており理由の説明は
  コード中に無い(**未確認**、pympc側で頻出した「TODO付きで無効化された
  処理」と同種のパターン)

### 外部トピックによる補正(`updateFromTopic`)

```cpp
sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
```

- コンストラクタで`/tracking_camera/odom/sample`(トピック名から、
  Intel RealSense T265のようなトラッキングカメラの出力を想定していると
  考えられる、**設計上の解釈**)を常時購読する
- `updateFromTopic()`は、`update()`の最後で`topicUpdated_`が`true`の
  ときだけ呼ばれ、TFフレーム(`odom`⇄`base`⇄センサ)の変換を経て
  `xHat_`のbase位置・各接触点位置を**直接上書き**する
- **実装上の注意点**：このトピックを配信する外部ノードは対象リポジトリに
  無く(**未確認**)、配信されなければ`topicUpdated_`は永遠に`false`の
  ままで、この補正は単に呼ばれないだけで実害はない(オプトイン機能)

### `getOdomMsg`

```cpp
odom.pose.pose.orientation.x = quat_.x();
odom.pose.pose.orientation.y = quat_.y();
odom.pose.pose.orientation.z = quat_.z();
odom.pose.pose.orientation.w = quat_.w();
odom.pose.pose.orientation.x = quat_.x();
```

- **実装上の注意点(軽微な重複)**：最後の行で`orientation.x`への代入が
  **もう一度**行われている(コピー&ペーストの残骸と推測される)。同じ値を
  再代入しているだけなので実害はない

---

## `LinearKalmanFilter.cpp` 252〜268行:`loadSettings`

この関数の役割:`task.info`の`kalmanFilter`ブロックから7つのノイズ
パラメータを読み込む。

- OCS2独自の`.info`形式パーサ(`boost::property_tree`+`loadData::loadPtreeValue`)
  を使用。7つの値すべて実測(`grep`)で確認したところ、a1の`task.info`は
  C++側のデフォルト値と**完全に同じ数値**だった(意図的に「デフォルトと
  一致させている」のか、単に「デフォルトをそのまま設定ファイルへ書き
  写しただけ」なのかは**未確認**)

---

## `FromTopiceEstimate.h`・`FromTopicEstimate.cpp`:`FromTopicStateEstimate`

この関数の役割:フィルタ処理を一切行わず、外部トピックの姿勢・速度を
そのまま状態として使う、デバッグ用の状態推定器。

```cpp
sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
...
vector_t FromTopicStateEstimate::update(...) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();
  updateAngular(quatToZyx(...), ...);
  updateLinear(..., ...);
  publishMsgs(odom);
  return rbdState_;
}
```

- `/ground_truth/state`(トピック名から、Gazebo側の正解値配信プラグイン
  (このリポジトリには無い、外部)を想定していると考えられる、**未確認**)
  を購読し、その姿勢・角速度・位置・速度を`updateAngular`/`updateLinear`
  へそのまま渡すだけ。カルマンフィルタのような推定・フィルタ処理は
  一切無い
- 関節状態(`updateJointStates`)は基底クラスの実装がそのまま使われる
  (実際の関節エンコーダ値は`FromTopicStateEstimate`でも本物を使う、
  「base位置・姿勢・速度だけ正解値にすり替える」という設計)

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `getOdomMsg`で`orientation.x`への代入が2回重複している(実害なし)
  2. カルマンフィルタの共分散補正ブロック(x-y位置の相関切り離し)が
     コメントアウトされたまま、理由の説明なく放置されている
  3. `zyxOffset_`(姿勢の基準オフセット)は用意されているが、この
     パッケージ内では常にゼロのまま更新されない
- 確認できた重要な事実:
  - このパッケージが担う「センサ融合による状態推定」は、pympcには
    存在しない工程(pympcはシミュレータの真の状態を直接読めるため)
  - 接触フラグに応じてプロセス/観測ノイズを100倍に切り替える
    (遊脚中の脚オドメトリを信用しない)設計が、このカルマンフィルタの
    核心
  - Gazebo側のIMU実装([read_code_03](read_code_03_legged_hw_sim.md))が
    重力を差し引いていたのは、この状態推定側で重力を足し戻す設計と
    一貫していることが確認できた
  - `LeggedCheaterController`用の`FromTopicStateEstimate`は、外部の
    正解値トピックをそのまま使うだけの、フィルタ処理なしの実装
- 次は、OCS2向けのロボットモデル・コスト・制約定義をまとめる
  `legged_interface/LeggedInterface`(pympcの`centroidal_model_nominal.py`+
  `centroidal_nmpc_nominal.py`のOCP定義部分に相当)を読みます。
