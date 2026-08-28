# NMPC

Q2のブロック④である。Gait結合も含む。

## 1. 結論

NMPCはOCS2 `SqpMpc` で、centroidalダイナミクスの最適制御を多重射撃SQPとして解く。状態24・入力24、ホライズン1.0 s、離散刻み15 ms、SQP反復1回、別スレッド100 Hzである。WBCへ渡すのは `evaluatePolicy` が現在時刻で取り出した **1点** の \(\mathbf{x}^*,\mathbf{u}^*\) と mode である。

ダイナミクス本体 `LeggedRobotDynamicsAD` はOCS2側である。本repoが組み立てるのはコスト、制約、参照、ソルバ設定である。

## 2. 背景

Sleiman et al., RAL 2021 と Grandia et al. の perceptive NMPC（README参照[2,3]）が、四足の全身運動をcentroidal運動量 + 関節 + 接触力で書く枠である。本スタックはそれを `ocs2_legged_robot` 経由で使い、ロボット固有のURDF・重み・摩擦・自己衝突を `LeggedInterface` に置く。

単一剛体(SRBD)ではなく、既定 `centroidalModelType = 0` の **FullCentroidalDynamics** である。関節運動がcentroidal運動量へ与える影響をモデルに残す。SRBDへ切り替える設定値 1 はあるが、a1/go1/aliengoの既定は0である。

## 3. 目的

予測ホライズン上で次を同時に決める。

- 胴体が目標2点軌道に沿うような運動量とポーズ
- 各脚のGRF（立脚）またはゼロ力（遊脚）
- 関節速度（足の相対運動の正則化を含む）
- gaitが指定した接地スケジュールを守ること
- 摩擦錐と自己衝突の余裕

12関節トルクはNMPCの決定変数ではない。トルクはWBCが現在瞬間に解く。

## 4. 最適制御問題

READMEの式を、本repoが渡す中身で具体化する。

\[
\begin{aligned}
\min_{u(\cdot)}\quad
&\phi(x(t_I)) + \int_{t_0}^{t_I} \ell(x,u,t)\,dt \\
\mathrm{s.t.}\quad
&x(t_0)=x_0 \\
&\dot x = f(x,u,t) \\
&g_1(x,u,t)=0 \\
&h(x,u,t)\ge 0
\end{aligned}
\]

| 記号 | 実装 | shape |
|---|---|---|
| \(x\) | centroidal状態 | `(24,)` |
| \(u\) | GRF + 関節速度 | `(24,)` |
| \(t_I-t_0\) | `mpc.timeHorizon` | 1.0 s |
| \(x_0\) | `currentObservation_.state` | `(24,)` |
| \(f\) | `LeggedRobotDynamicsAD` | OCS2 |
| 転写 | multiple shooting SQP + HPIPM | `sqp.dt=0.015`、反復1 |

対応コード: `LeggedInterface::setupOptimalControlProblem()`、`LeggedController::setupMpc()` の `SqpMpc`。設定は `task.info` の `mpc` と `sqp`。

### 4.1 ダイナミクス（理論 / OCS2側）

正規化centroidal運動量 \(h_{\mathrm{com}}=[m v_{\mathrm{com}}^\mathsf T,\ L^\mathsf T]^\mathsf T / m\) に対し、接触力の合力・合トルクが \(\dot h\) を駆動し、一般化座標は運動学で進む。READMEの定義は

\[
x=[h_{\mathrm{com}}^\mathsf T,\ q_b^\mathsf T,\ q_j^\mathsf T]^\mathsf T,\quad
u=[f_c^\mathsf T,\ v_j^\mathsf T]^\mathsf T
\]

である。ピンocchioのcentroidal運動量行列で \(h\) と \(q,\dot q\) を結ぶ。完全なODEは `ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD` にあり未照合である。

### 4.2 コスト

中間コストは二次追従である。ただし入力参照は軌道のゼロではなく、接地脚への重力補償である。

\[
\ell
=
\|x-x^{\mathrm{ref}}(t)\|_Q^2
+
\|u-u_{\mathrm{wc}}(c(t))\|_R^2
\]

`getStateInputDeviation()` が

```
xNominal = targetTrajectories.getDesiredState(time)
uNominal = weightCompensatingInput(info, contactFlags)
```

とする。`weightCompensatingInput` はOCS2ユーティリティ（本repo外）。立脚数で \(mg\) を分け、遊脚力は0、という標準実装である。

\(Q\) は `task.info` の対角。a1で大きい項は位置 \(Q_{xx,yy}=1000\)、高さ 1500、yaw 100、roll/pitch 300、水平速度 15である。関節は 2.5–5。

\(R\) ファイル値は「足速度」12次元だが、`initializeInputCostWeight()` が名目姿勢の足Jacobian \(J_b\) で関節速度ブロックへ写す。

\[
R_{v_j}
=
J_b^\mathsf T R_{\mathrm{task}} J_b
\]

GRFブロックはそのまま、スケール \(10^{-3}\) が掛かる。

対応コード: `legged_interface/include/legged_interface/cost/LeggedRobotQuadraticTrackingCost.h`、`LeggedInterface.cpp` の `getBaseTrackingCost()`, `initializeInputCostWeight()`。

### 4.3 制約

脚 \(i=0\ldots3\) ごとに、gaitの `contactFlags(t)[i]` で active が切り替わる。

| 名前 | 種類 | active | 式 | 次元 |
|---|---|---|---|---|
| `ZeroForce` | 等式 | 遊脚 | \(f_{c,i}=0\) | 3 |
| `ZeroVelocity` | 等式 | 立脚 | 足並進速度 \(\approx0\)。`positionErrorGain=0` なので位置フィードバック無し | 3 |
| `NormalVelocity` | 等式 | 遊脚 | \(\dot z = \dot z_{\mathrm{sw}}(t)\)。gain非ゼロなら \(z\) も | 1 |
| `FrictionCone` | 軟制約（既定） | 立脚 | \(\mu(F_z+F_g)-\sqrt{F_x^2+F_y^2+\varepsilon}\ge0\) | 1 |
| `selfCollision` | 状態軟制約 | 常時 | リンク対距離 \(\ge 0.05\) m | ペア数 |

摩擦の既定は \(\mu=0.3\)、`mu/delta` barrier 0.1 / 5.0。`useHardFrictionConeConstraint` は `LeggedInterface` コンストラクタ既定 false。地形法線回転 `setSurfaceNormalInWorld` は未実装で例外する。世界鉛直錐である。

遊脚zは `SwingTrajectoryPlanner` の spline CPGである。a1設定は

- liftOffVelocity 0.05 m/s
- touchDownVelocity −0.1 m/s
- swingHeight 0.08 m
- swingTimeScale 0.15 s（短いswingは高さ縮小）

`SwitchedModelReferenceManager::modifyReferences` は地形高さを **0** で渡す。不整地の真の標高は使わない。

対応コード: `LeggedInterface.cpp` の制約add、`constraint/*`、`SwingTrajectoryPlanner.cpp`、`LeggedRobotPreComputation.cpp`、`SwitchedModelReferenceManager.cpp`。

## 5. Receding horizon とWBCへの1点

NMPCスレッドは `advanceMpc()` でホライズン全体を解く。制御スレッドは

```
setCurrentObservation(currentObservation)
updatePolicy()
evaluatePolicy(t, x, optimizedState, optimizedInput, plannedMode)
currentObservation.input = optimizedInput
```

`useFeedbackPolicy=false` なので、評価はフィードバックゲインではなく軌道の補間である。WBCが見るのはこの瞬間の24+24+modeだけである。未来のGRF列はvisualizationには出るが、トルク計算には使わない。

対応コード: `LeggedController::update()`, `setupMrt()`, `setupMpc()`。

## 6. Gait結合

指令軌道と独立に、`GaitReceiver` が `GaitSchedule` を更新する。`modifyReferences(initTime, finalTime, ...)` がホライズン前後に伸ばした `ModeSchedule` を作り、swing plannerを更新する。

trotの定義（`gait.info`）:

| 区間 | mode | 接地 |
|---|---|---|
| 0.0–0.3 s | `LF_RH` | 左前・右後 |
| 0.3–0.6 s | `RF_LH` | 右前・左後 |

初期 `reference.info` は `STANCE` だけである。寝たまま `stance` と打つ必要は無い（README）。NMPCはmodeを最適化しない。与えられた列の上で力と運動を最適化する。

`plannedMode` はpolicy評価時刻のmodeで、WBCの `contactFlag_` になる。推定の `observation.mode` とは別に、計画modeがWBCを駆動する。

対応コード: `LeggedController::setupMpc()` の `GaitReceiver`、`SwitchedModelReferenceManager::modifyReferences()`、`config/*/gait.info`。`GaitReceiver` 本体はOCS2側。

## 7. 初期化

`starting()` は推定1回のあと、現在状態の1点軌道を参照に入れ、`initialPolicyReceived()` まで `advanceMpc()` を回す。その後 `mpcRunning_=true` で100 Hzスレッドが動き出す。

## 8. 実装事実と理論の境界

- **実装事実**: 状態24入力24、SQP 1回、100 Hz、摩擦は軟制約、地形高さ0、既定はFullCentroidal。
- **理論**: centroidal NMPC + スイッチド接触。トルクは内側ループ。
- **未確認**: `f(x,u)` の成分式、HPIPMのQPサイズ、`weightCompensatingInput` の正確な分配。
- **未実装**: 接触時刻の同時最適化、知覚地形、硬摩擦の既定利用。
