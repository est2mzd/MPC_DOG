# State Estimation

Q2のブロック③である。

## 1. 結論

実機経路の推定は、IMU姿勢と関節角をそのまま使い、胴体並進位置・速度だけを線形Kalmanで復元する。出力は剛体状態 `(36,)` であり、コントローラがcentroidal状態 `(24,)` へ変換してNMPCへ渡す。Cheater経路は `/ground_truth/state` を読むだけで、実機禁止である。

## 2. 背景

浮動ベースには胴体xyzの絶対エンコーダが無い。IMUは姿勢と加速度、モータは関節角、足力センサは接地の有無を与える。Flayols et al., Humanoids 2017 の単純推定器（README参照[1]）に沿い、接地足が世界に対して滑らないという仮定で、相対足位置から胴体位置を観測する。

本実装のクラスは `KalmanFilterEstimate` である。視覚オドメトリ `/tracking_camera/odom/sample` が来たときだけ位置を上書きする任意経路がある。

## 3. 目的

- NMPCの初期状態 \(\mathbf{x}(t_0)=\mathbf{x}_0\) を毎周期用意する。
- WBCがPinocchioに渡す実測 \(q, v\) を用意する。
- yawの不連続（\(\pm\pi\)）を `angles::shortest_angular_distance` でつなぎ、参照との差が跳ねないようにする。
- 接地フラグをOCS2 mode番号へ写し、observation.mode にする。

## 4. センサ入力とサイズ

`LeggedController::updateStateEstimation()` がHWハンドルから読む。

| 信号 | shape | 単位 | 出典 |
|---|---|---|---|
| 関節位置 | `(12,)` | rad | `HybridJointHandle::getPosition` |
| 関節速度 | `(12,)` | rad/s | `getVelocity` |
| 接地 | `(4,)` bool | 無次元 | `ContactSensorHandle::isContact` |
| IMU姿勢 | quat `(4,)` | — | `ImuSensorHandle`。係数順 x,y,z,w |
| IMU角速度 | `(3,)` | rad/s | base |
| IMU並進加速度 | `(3,)` | m/s² | base。重力を含むセンサ値 |
| 各共分散 | 3×3 が3つ | 混在 | IMU。KF本体では主にodom publishに使う |

実機の接地は `footForce[i] > contact_threshold`。a1は閾値 40。Gazeboは接触マネージャの有無をboolにする。

対応コード: `LeggedController.cpp` の `updateStateEstimation()`、`UnitreeHW.cpp` の `read()`、`LeggedHWSim.cpp` の `readSim()`。

## 5. 剛体状態の組み立て

`StateEstimateBase` が `rbdState_` `(36,)` を埋める。

1. `updateJointStates`: index 6:18 に \(q_j\)、24:36 に \(\dot q_j\)。
2. `updateImu`:
   - quat → ZYX、`zyxOffset_` を引く
   - 局所ωをglobal角速度へ変換
   - index 0:3 にZYX、18:21 にglobal ω
3. `KalmanFilterEstimate::update` が位置と並進速度を決め、`updateLinear` が 3:6 と 21:24 に書く。

姿勢と関節はフィルタしない。フィルタ対象は胴体並進と足位置だけである。

対応コード: `legged_estimation/src/StateEstimateBase.cpp`、`LinearKalmanFilter.cpp`。

## 6. 線形Kalmanの数式

### 6.1 状態と観測

接触数 \(n_c=4\)。`dimContacts_ = 12`。

\[
\hat x \in \mathbb{R}^{18}
=
[p_b^\mathsf T,\ v_b^\mathsf T,\ p_{f,1}^\mathsf T,\ldots,p_{f,4}^\mathsf T]^\mathsf T
\]

\[
y \in \mathbb{R}^{28}
=
[p_{s,1}^\mathsf T,\ldots,p_{s,4}^\mathsf T,\
v_{s,1}^\mathsf T,\ldots,v_{s,4}^\mathsf T,\
h_1,\ldots,h_4]^\mathsf T
\]

- \(p_s \approx -p_{\mathrm{ee}}\)（ベース原点に胴体を置いたFKの足位置の符号反転）。zに `footRadius=0.02` m を足す
- \(v_s \approx -v_{\mathrm{ee}}\)
- \(h_i\): `feetHeights_`。通常0。視覚オドメトリ更新時だけ接地足の高さを覚える

### 6.2 予測

IMU加速度をWorldへ回し重力を足す。

\[
a_{\mathrm{W}} = R_{\mathrm{ZYX}}(\mathrm{quat})\, a_{\mathrm{B}} + [0,0,-9.81]^\mathsf T
\]

\[
\hat x \leftarrow A\hat x + B a_{\mathrm{W}},\quad
A
=
\begin{bmatrix}
I_3 & \Delta t I_3 & 0 \\
0 & I_3 & 0 \\
0 & 0 & I_{12}
\end{bmatrix},\quad
B
=
\begin{bmatrix}
\frac12\Delta t^2 I_3 \\
\Delta t I_3 \\
0
\end{bmatrix}
\]

プロセスノイズ \(Q\) は `imuProcessNoisePosition/Velocity` と `footProcessNoisePosition` でスケールする。遊脚の足位置プロセスノイズは 100 倍し、「この足は世界に固定されていない」と伝える。

### 6.3 更新

観測行列 \(C\in\mathbb{R}^{28\times18}\) は、概ね

\[
p_b - p_{f,i} \approx p_{s,i},\quad
v_b \approx v_{s,i},\quad
p_{f,i,z} \approx h_i
\]

を線形に書いたものである。標準Kalman更新を密行列のLUで解く。出力は \(\hat x[0:3]=p_b\)、\(\hat x[3:6]=v_b\) だけを剛体状態へ戻す。

対応コード: `KalmanFilterEstimate::update()`。ノイズは `task.info` の `kalmanFilter`。

### 6.4 視覚オドメトリ（任意）

`/tracking_camera/odom/sample` が来ると `topicUpdated_` が立ち、`updateFromTopic()` が `xHat_` の胴体位置と足位置をTF経由で上書きする。トピックが無ければこの枝は動かない。

## 7. centroidal状態への変換

コントローラ側である。

\[
\mathbf{x} = \texttt{computeCentroidalStateFromRbdModel}(\mathbf{x}_{\mathrm{rbd}})
\in\mathbb{R}^{24}
\]

この関数はOCS2 `CentroidalModelRbdConversions` にあり、本repo外である。やることは \(q,v\) から正規化centroidal運動量とZYXポーズ、関節角を組むことである。そのあと

```
state(9) = yawLast + shortest_angular_distance(yawLast, state(9))
```

でyawを連続化する。`mode` は `stanceLeg2ModeNumber(contactFlag)`。

対応コード: `LeggedController::updateStateEstimation()` 末尾。

## 8. Cheater推定

`FromTopicStateEstimate` は `/ground_truth/state` の pose / twist を `rbdState_` の姿勢・位置・速度へ直書きする。関節は直前の `updateJointStates` が残る。実機ではground truthが無い。

対応コード: `legged_estimation/src/FromTopicEstimate.cpp`。`LeggedCheaterController::setupStateEstimate()`。

## 9. 実装事実と理論の境界

- **実装事実**: 並進だけKF。姿勢はIMU。接地は閾値bool。出力36→24。
- **理論**: 接地足ゼロ速度・既知半径の線形観測。Flayols 2017系。
- **未確認**: `computeCentroidalStateFromRbdModel` の運動量定義の係数はOCS2ソース未照合。
- **推奨改善**: 遊脚ノイズ倍率 100 はマジックナンバー。接触力連続値を使っていない。
