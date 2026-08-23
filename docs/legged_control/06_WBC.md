# Whole-Body Control

Q2のブロック⑤である。

## 1. 結論

既定WBCは `WeightedWbc` である。NMPCが出した目標状態・入力と、推定した剛体状態を入力に、現在瞬間のQPをqpOASESで解く。決定変数は42次元で、コントローラが使うのは末尾12の関節トルクだけである。`HierarchicalWbc` は実装済みだが `LeggedController::init` は呼ばない。

## 2. 背景

Centroidal NMPCは慣性の縮約モデルである。実機は関節トルク、摩擦、遊脚加速度、トルク飽和を同時に満たさねばならない。Bellicoso et al., Humanoids 2016 の階層全身制御（README参照[4]）がこの隙間を埋める定番である。

本repoは同じタスク定義を `WbcBase` に置き、解法だけ2通り持つ。

| クラス | 解法 | 既定? |
|---|---|---|
| `WeightedWbc` | ハード制約 + 重み付き最小二乗。単一QP | はい |
| `HierarchicalWbc` | `HoQp` で null-space 階層 | いいえ |

READMEの「strict hierarchy」は `HierarchicalWbc` の説明である。走っているコードは加重和である。

## 3. 目的

現在時刻だけで次を両立する。

- 浮動ベースの運動方程式を満たす \(\ddot q, F_c, \tau\)
- トルク上限と摩擦錐（または遊脚ゼロ力）
- 立脚足の加速度ゼロ（滑らない）
- NMPCの胴体加速度とGRFに近づける
- 遊脚足をNMPC姿勢が意味する位置へPD加速する

NMPCのGRFは目標である。WBCの \(F_c\) は必要ならずらし、その結果の \(\tau\) を出す。

## 4. 決定変数と入出力

\[
\mathbf{x}_{\mathrm{wbc}}
=
[\ddot q^\mathsf T\in\mathbb{R}^{18},\
F_c^\mathsf T\in\mathbb{R}^{12},\
\tau^\mathsf T\in\mathbb{R}^{12}]^\mathsf T
\]

`numDecisionVars_ = generalizedCoordinatesNum + 3*numThreeDofContacts + actuatedDofNum = 18+12+12`。

| 入力 | shape | 由来 |
|---|---|---|
| `stateDesired` | `(24,)` | NMPC `optimizedState` |
| `inputDesired` | `(24,)` | NMPC `optimizedInput` |
| `rbdStateMeasured` | `(36,)` | 推定 |
| `mode` | scalar | NMPC `plannedMode` |
| `period` | s | 制御周期。ベース加速度タスクの \(\dot v_j\) 差分 |

| 出力 | shape | 使用 |
|---|---|---|
| `qpSol` | `(42,)` | 全体 |
| `torque` | `(12,)` | `x.tail(12)` のみ `setCommand` のff |

対応コード: `WbcBase.h` 先頭コメント、`WbcBase.cpp` コンストラクタ、`LeggedController::update()`。

## 5. 計測側と参照側の運動学

`updateMeasured` は推定 \(q,v\) でPinocchioを更新し、質量行列 \(M\)、非線形項 \(nle\)、足Jacobian \(J\) と \(\dot J\) を取る。ピンocchioの配置は `[p(3), zyx(3), q_j(12)]` で、`rbdState` の `[zyx, p, q_j]` とは順序が違う。コードが入れ替える。

`updateDesired` はNMPC状態から望ましい \(q,v\) を出し、centroidal運動量行列を更新する。遊脚タスクは「望む足位置・速度」をこのFKから取る。NMPCが関節を動かすと、望む足先が変わる。

対応コード: `WbcBase::updateMeasured()`, `updateDesired()`。

## 6. タスクの数式

各タスクは等式 \(A x = b\) と不等式 \(D x \le f\) である（`Task`）。

### 6.1 浮動ベース運動方程式（等式、18）

\[
M\ddot q - J^\mathsf T F_c - S^\mathsf T \tau + nle = 0
\]

行列形は

\[
[M,\ -J^\mathsf T,\ -S^\mathsf T]\, x = -nle
\]

\(S\) は関節選択（浮動6列が0、関節12が単位）。これは硬制約である。

### 6.2 トルク上限（不等式、24）

a1は脚内3関節とも 33.5 N·m。

\[
-\tau_{\lim} \le \tau \le \tau_{\lim}
\]

`torqueLimitsTask` は `(3,)` を4脚へ繰り返す。

### 6.3 立脚ゼロ運動（等式、\(3 n_{\mathrm{st}}\)）

接地足について

\[
J_i \ddot q = -\dot J_i v
\]

すなわち足並進加速度0。

### 6.4 摩擦と遊脚ゼロ力

遊脚: \(F_{c,i}=0\)（等式3）。

立脚: ピラミッド5不等式（円錐の内接）。

\[
F_z \ge 0,\quad
|F_x|\le \mu F_z,\quad
|F_y|\le \mu F_z
\]

コードの `frictionPyramic` は \(F_z\) に負号を付け、\(Dx\le0\) で書く。WBCの \(\mu\) は `frictionConeTask.frictionCoefficient=0.3` で、NMPC軟制約と同じ値である。形はNMPCの円錐（正則化付きノルム）とは違い、線形ピラミッドである。

### 6.5 遊脚PD（コスト、\(3 n_{\mathrm{sw}}\)）

\[
J_i\ddot q
=
k_p(p_i^*-p_i) + k_d(v_i^*-v_i) - \dot J_i v
\]

a1は \(k_p=350\)、\(k_d=37\)。\(p^*,v^*\) はNMPC望ましいFK、\(p,v\) は実測FK。

### 6.6 ベース加速度追従（コスト、6）

NMPC入力の関節速度差分から \(\ddot q_j \approx (v_j-v_j^{\mathrm{prev}})/\Delta t\) を作り、centroidal運動量行列の逆で浮動ベース加速度 \(\ddot q_b^*\) を復元し、\(\ddot q_{0:6}\) をそれに合わせる。

### 6.7 接触力追従（コスト、12）

\[
F_c = u^*[0:12]
\]

NMPC GRFそのものである。重みが小さいと、他タスクのためにGRFがずれる。

対応コード: `WbcBase.cpp` の `formulate*`。重みは `task.info` の `weight`。

## 7. `WeightedWbc` のQP

硬制約（6.1–6.4）を qpOASES の \(lbA \le A x \le ubA\) に積み、コストは

\[
\min_x
\ \big\|
W_{\mathrm{sw}} A_{\mathrm{sw}} x - W_{\mathrm{sw}} b_{\mathrm{sw}}
\big\|^2
+
\big\|
W_{\mathrm{base}} (\ddot q_b - \ddot q_b^*)
\big\|^2
+
\big\|
W_{F}(F_c - F_c^*)
\big\|^2
\]

a1重みは swingLeg 100、baseAccel 1、contactForce 0.01。遊脚追従が最も強く、NMPC GRFは弱い。だから「NMPCの力は目標、WBCは必要なら修正する」が実装どおりである。

`nWsr=20`、`options.setToMPC()`。失敗時のフォールバックは無い。`getPrimalSolution` をそのまま返す。

対応コード: `legged_wbc/src/WeightedWbc.cpp`。

trot（立脚2・遊脚2）の制約数の目安:

| 種類 | 行数 |
|---|---:|
| EoM 等式 | 18 |
| 遊脚ゼロ力 等式 | 6 |
| 立脚ゼロ運動 等式 | 6 |
| トルク箱 不等式 | 24 |
| 摩擦ピラミッド 不等式 | 10 |
| 合計 | 64 |
| 決定変数 | 42 |

## 8. `HierarchicalWbc`（未配線）

優先度は

0. EoM + トルク + 摩擦 + 立脚ゼロ運動
1. ベース加速度 + 遊脚
2. 接触力追従

`HoQp` が上位の null space で下位を解く。Bellicoso流に近い。`LeggedController` は `std::make_shared<WeightedWbc>(...)` だけである。

対応コード: `HierarchicalWbc.cpp`, `HoQp.cpp`。

## 9. 安全

`SafetyChecker` は観測ポーズの roll（`getBasePose` の index 5）が \(\pm\pi/2\) を超えたら失敗し、コントローラを止める。ピッチもトルクも見ない。

対応コード: `legged_controllers/include/legged_controllers/SafetyChecker.h`。

## 10. 実装事実と理論の境界

- **実装事実**: 既定は加重QP、42変数、出力はτ12、μ=0.3ピラミッド、Kp/Kdは遊脚タスク内部。
- **理論**: 階層WBC。README図はこちら。
- **推奨改善**: 失敗時の前回トルク保持。READMEと実装（Weighted vs Hierarchical）の表記を揃える。
