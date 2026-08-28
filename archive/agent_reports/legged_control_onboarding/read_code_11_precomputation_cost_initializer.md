# 事前計算・コスト・初期化 legged_interface(PreComputation・QuadraticTrackingCost・Initializer・SelfCollision) 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedInterface::setupPreComputation(...)(read_code_07)
  → LeggedRobotPreComputation コンストラクタ  ← 本ファイル、起動時1回

OCS2のSQPソルバー内部(未確認)が、MPCの各ステージ評価のたびに:
  → LeggedRobotPreComputation::request(...)   ← 本ファイル、
      MPC内部反復×ホライズンステージ数だけ(既定100Hzの各MPC呼び出し内)
      → 順運動学・セントロイダル動力学の再計算をキャッシュ
      → read_code_10のNormalVelocityConstraintCppAdが使う設定を準備

problemPtr_->costPtr->add("baseTrackingCost", ...)(read_code_07)
  → LeggedRobotStateInputQuadraticCost::getStateInputDeviation
      ← 本ファイル、コスト評価のたびに呼ばれる(頻度は上と同様)

LeggedInterface::setupOptimalControlProblem(...)
  → LeggedRobotInitializer  ← 本ファイル、SQPの初期軌道生成時(未確認、
      おそらくMPC呼び出しごとのウォームスタートに使われる)
```

## このファイル/クラスの役割(全体の中での位置づけ)

この章で扱うのは、`legged_interface`パッケージの残り4つのコンポーネント
です。

- `LeggedRobotPreComputation`：**事前計算キャッシュ**。順運動学・
  セントロイダル動力学の計算を1つのステージ評価につき1回だけ行い、
  コスト・制約クラス群がそれぞれ個別に再計算しなくて済むようにする
- `LeggedRobotQuadraticTrackingCost`：状態・入力の目標追従コスト
  (pympcの`create_stage_cost`相当)。目標入力を「体重を按分した接地力」
  とする点が特徴
- `LeggedRobotInitializer`：SQPソルバーへ渡す初期軌道(ウォームスタート)
  の生成規則
- `LeggedSelfCollisionConstraint`：[read_code_07](read_code_07_legged_interface.md)で
  見た自己干渉制約の、このリポジトリ独自の薄いラッパークラス

これで`legged_interface`パッケージ(OCS2向けの問題定義一式)を読み終えます。

対象は`external/legged_control/legged_interface/include/legged_interface/LeggedRobotPreComputation.h`
(78行)・`src/LeggedRobotPreComputation.cpp`(117行)、
`include/legged_interface/cost/LeggedRobotQuadraticTrackingCost.h`
(95行、`.cpp`なし)、
`include/legged_interface/initialization/LeggedRobotInitializer.h`
(65行)・`src/initialization/LeggedRobotInitializer.cpp`(66行)、
`include/legged_interface/constraint/LeggedSelfCollisionConstraint.h`
(29行、`.cpp`なし)です。

---

## `LeggedRobotPreComputation.cpp` 74〜113行:`request`

この関数の役割:このステージで何が必要とされているか(コスト/制約/近似)に
応じて、遊脚のZ速度制約設定とPinocchioの運動学・動力学キャッシュを更新する。

```cpp
if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
  return;
}
```

- コスト・制約・ソフト制約のいずれも要求されていなければ即座に`return`
  する早期リターン(無駄な計算を避ける)

```cpp
auto eeNormalVelConConfig = [&](size_t footIndex) {
  EndEffectorLinearConstraint::Config config;
  config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
  config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
  if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
    config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
    config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
  }
  return config;
};
if (request.contains(Request::Constraint)) {
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
  }
}
```

**コードで確認した事実([read_code_10](read_code_10_contact_constraints.md)で
予告した接続点の実体)**：`config.Av = (0, 0, 1)`と、World座標系のZ軸方向
だけを取り出す行列が**ハードコード**されています。地形の傾きに関わらず
「鉛直方向速度」として常にworld座標系のZ成分をそのまま使っており、
[read_code_10](read_code_10_contact_constraints.md)の摩擦錐(`t_R_w`が
常に単位行列)、[read_code_08](read_code_08_switched_model_reference_manager.md)の
地形高さ固定と一貫して、**このリポジトリ全体が傾斜地形に非対応**である
ことがここでも確認できます。

```cpp
if (request.contains(Request::Approximation)) {
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::updateGlobalPlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  updateCentroidalDynamics(pinocchioInterface_, info_, q);
  vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
  updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
} else {
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
}
```

- `Request::Approximation`(線形/2次近似が要求される、SQPの勾配・ヘシアン
  計算時)が含まれる場合は、ヤコビアン・動力学微分まで含めたフル計算を
  行う。含まれない場合(値の評価だけでよい場合)は、順運動学だけの
  軽量な計算に留める、という**計算量の使い分け**(**設計上の解釈**、
  pympc側には対応する明示的な使い分けの仕組みは無かった)

---

## `LeggedRobotQuadraticTrackingCost.h` 45〜91行:`LeggedRobotStateInputQuadraticCost`・`LeggedRobotStateQuadraticCost`

この関数の役割(`LeggedRobotStateInputQuadraticCost::getStateInputDeviation`):
現在の状態・入力が、目標状態・目標入力からどれだけ乖離しているかを
計算する(2次コストが最小化する対象)。

```cpp
std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                     const TargetTrajectories& targetTrajectories) const override {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  const vector_t xNominal = targetTrajectories.getDesiredState(time);
  const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
  return {state - xNominal, input - uNominal};
}
```

- `xNominal`(目標状態)：`TargetTrajectories`(未読、
  [read_code_12以降](read_code_12_wbc_base.md)または
  `TargetTrajectoriesPublisher`で扱う)から取得
- `uNominal`(目標入力)：`weightCompensatingInput(info_, contactFlags)`
  (OCS2 legged_robot、外部)。**接地中の脚だけで、ロボットの全体重を
  均等に按分した接地力**を目標入力とする。pympc側の
  [read_code_11](../quadruped_pympc_onboarding/read_code_11_compute_control.md)
  (同シリーズ外参照になるため直接記載:「GRFの目標値(z成分)は体重÷接地脚数
  で計算され、`yref`へ正しく反映される」)と**まったく同じ設計思想**の
  参照値です。ゼロを目標にするのではなく、「もし目標軌道通りに動けたら
  出るはずの、体を支えるのに必要な力」を目標にすることで、コストが
  無駄な力の変動を嫌う自然な挙動になると考えられます(**設計上の解釈**)

**実装上の注意点(未使用クラス)**：`LeggedRobotStateQuadraticCost`
(終端コスト用、コメント「State tracking cost used for the final time」)
というクラスも同じファイルに定義されていますが、`grep`で確認したところ
`external/legged_control`のどこからも**インスタンス化されていません**。
[read_code_07](read_code_07_legged_interface.md)で見た
`LeggedInterface::setupOptimalControlProblem`は`costPtr`(中間コスト)へ
`LeggedRobotStateInputQuadraticCost`だけを追加しており、OCS2の
`finalCostPtr`(終端コスト、あるとすれば)に相当する登録は行っていません。
つまりこのMPCには**明示的な終端コストが設定されていない**可能性が高い
(**推測**、OCS2側のデフォルト動作は未確認)。

---

## `LeggedRobotInitializer.cpp` 41〜62行:`compute`

この関数の役割:SQPソルバーへ渡す初期軌道の1ステップ分(次状態・入力)を
生成する。

```cpp
void LeggedRobotInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  input = weightCompensatingInput(info_, contactFlags);
  nextState = state;
  if (!extendNormalizedMomentum_) {
    centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
  }
}
```

- `input`の初期推定値も、コストと同じ`weightCompensatingInput`
  (体重按分接地力)を使う
- `nextState = state`：次ステージの状態の初期推定値は、単純に「今と
  同じ」(動かないと仮定した初期推定)
- `extendNormalizedMomentum_`：[read_code_07](read_code_07_legged_interface.md)の
  `LeggedInterface::setupOptimalControlProblem`で`true`固定
  (`constexpr bool extendNormalizedNomentum = true;`、**実装上の注意点**：
  変数名が`Nomentum`と誤字(`Momentum`が正しいスペル)になっている)で
  渡されるため、この`if`は**常に偽**となり、
  `getNormalizedMomentum(nextState, info_).setZero()`は実行されません。
  つまり初期推定の運動量成分は、現在の運動量がそのまま次ステージの
  初期推定として引き継がれます(ゼロクリアされない)

---

## `LeggedSelfCollisionConstraint.h`:自己干渉制約のラッパー

```cpp
class LeggedSelfCollisionConstraint final : public SelfCollisionConstraint {
 public:
  LeggedSelfCollisionConstraint(const CentroidalModelPinocchioMapping& mapping, PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<LeggedRobotPreComputation>(preComputation).getPinocchioInterface();
  }
};
```

- OCS2本体の`SelfCollisionConstraint`(外部)をほぼそのまま継承し、
  唯一`getPinocchioInterface`だけをオーバーライドして、
  `LeggedRobotPreComputation`(このステージで既にキャッシュ済みの
  Pinocchioモデル)を返すようにしている。**事実**：このファイルは
  [read_code_08](read_code_08_switched_model_reference_manager.md)・
  [read_code_09](read_code_09_swing_trajectory_planner.md)・
  [read_code_10](read_code_10_contact_constraints.md)の大半のファイルとは
  異なり、`legged`名前空間・qiayuan名義であり、このリポジトリで実際に
  書かれたコードと考えられます

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `LeggedRobotStateQuadraticCost`(終端コスト用クラス)が定義される
     だけで、どこからもインスタンス化されていない
  2. `LeggedInterface.cpp`の`extendNormalizedNomentum`という変数名に
     綴りの誤り(`Momentum`ではなく`Nomentum`)がある
  3. `extendNormalizedMomentum_`が常に`true`のため、
     `LeggedRobotInitializer`の運動量ゼロクリア分岐は常に実行されない
     (デッドコードではないが、実質的に意味を持たない`if`)
- 確認できた重要な事実:
  - `LeggedRobotPreComputation::request`内の`config.Av=(0,0,1)`
    ハードコードにより、[read_code_08](read_code_08_switched_model_reference_manager.md)・
    [read_code_10](read_code_10_contact_constraints.md)で見た
    「地形は常に平坦」という前提が、ここでも再確認できた
  - コストと初期化の両方が、目標入力として「体重を接地脚で按分した
    接地力」(`weightCompensatingInput`)を使う、pympcのGRF目標値計算と
    同じ設計思想が採られている
  - `Request`の種類(コスト/制約/近似)に応じて、Pinocchioの計算量を
    使い分ける最適化がされている
- これで`legged_interface`パッケージ(read_code_07〜11、OCS2向けの問題
  定義一式)を読み終えました。次は、MPCが出力した最適状態・入力を実際の
  関節トルクへ変換する`legged_wbc`パッケージ(`WbcBase`・`HoQp`・
  `WeightedWbc`・`HierarchicalWbc`、pympcの
  `WBInterface.compute_stance_and_swing_torque`に相当)を読みます。
