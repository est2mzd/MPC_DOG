# OCS2問題定義の組み立て legged_interface/LeggedInterface 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedController::init(...)(read_code_05)
  → setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose)
      → LeggedInterface コンストラクタ            ← 本ファイル、起動時1回
      → LeggedInterface::setupOptimalControlProblem(...)  ← 本ファイル、起動時1回
          → setupModel / setupReferenceManager / setupPreComputation
          → 動力学・コスト・制約をOCS2の OptimalControlProblem へ登録
  → mpc_ = SqpMpc(..., leggedInterface_->getOptimalControlProblem(), ...)
      (read_code_05、以後はOCS2内部がこの問題定義を毎周期解く)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`LeggedInterface`が担当するのは、「**OCS2のSQPソルバーに『何を解かせるか』
(動力学モデル・コスト関数・制約条件)を、`task.info`/`urdf`/`reference.info`
から組み立てて渡す**」ことです。pympcで言えば`centroidal_model_nominal.py`
(状態・入力・パラメータの次元定義)と`centroidal_nmpc_nominal.py`
(コスト・制約の設定、read_code_09)を合わせたような役割に相当します。

- 実際にSQPを解く処理(`SqpMpc`内部)は対象外・未確認
- 歩容スケジュール・スイング軌道計画の詳細は
  `SwitchedModelReferenceManager`・`SwingTrajectoryPlanner`
  (次章以降)に委譲されており、ここでは「それらを生成して問題に接続する」
  ところまでを扱う
- 個々の制約・コストクラス(`FrictionConeConstraint`等)の内部実装も
  対象外(それぞれ独立した`read_code`ファイルで扱う)。ここでは
  「**どの制約・コストが、どんな設定値で、何個(脚の数だけ等)登録される
  か**」という**組み立てのロジック**に焦点を当てる

対象は`external/legged_control/legged_interface/include/legged_interface/LeggedInterface.h`
(105行)・`external/legged_control/legged_interface/src/LeggedInterface.cpp`
(374行)です。

---

## `LeggedInterface.cpp` 38〜75行:コンストラクタ

この関数の役割:3つの設定ファイルの存在を検証し、`task.info`から
モデル・MPC・DDP・SQP・IPM・ロールアウトの各種設定を読み込む。

```cpp
boost::filesystem::path taskFilePath(taskFile);
if (boost::filesystem::exists(taskFilePath)) { ... } else { throw std::invalid_argument(...); }
```

- `taskFile`/`urdfFile`/`referenceFile`のいずれかが存在しなければ、
  その場で`std::invalid_argument`例外を投げてコンストラクタが失敗する
  (pympc側は設定ファイルが無い場合の明示的な検証は無かった、
  **設計上の解釈**:C++の起動時失敗はPythonよりも早期に検知する
  傾向がある)

```cpp
modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
```

- `ddpSettings_`/`ipmSettings_`も読み込まれるが、
  [read_code_05](read_code_05_legged_controller.md)で確認した通り実際に
  使われるソルバーは`SqpMpc`のみ。DDP・IPM設定は**読み込まれるだけで
  未使用**と考えられる(**推測**、pympc側の「サンプリングMPC・
  kinodynamic型は既定OFF」というパターンと同様、複数のソルバー方式を
  選べる設計だが実際に配線されているのはSQPだけ)
- `modelSettings_`(`ModelSettings`型、OCS2 legged_robot共通)の実際の値
  (a1の`task.info`)：`positionErrorGain=0.0`、
  `phaseTransitionStanceTime=0.1`(秒)、`verboseCppAd=true`、
  `recompileLibrariesCppAd=false`、`modelFolderCppAd=/tmp/legged_control/a1`
  (CppADの自動微分コード生成キャッシュ先。pympc側の「acadosの2層ビルド」
  に相当する、初回だけ時間のかかるコード生成の仕組みが、ここにも
  存在すると考えられる、**設計上の解釈**)

---

## `LeggedInterface.cpp` 80〜137行:`setupOptimalControlProblem`

この関数の役割:モデル・参照マネージャ・動力学・コスト・制約を1つの
`OptimalControlProblem`へ組み立てる、このクラスの中心的な処理。

```cpp
setupModel(taskFile, urdfFile, referenceFile, verbose);
initialState_.setZero(centroidalModelInfo_.stateDim);
loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
setupReferenceManager(taskFile, urdfFile, referenceFile, verbose);

problemPtr_ = std::make_unique<OptimalControlProblem>();
dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics", modelSettings_);
problemPtr_->dynamicsPtr = std::move(dynamicsPtr);
```

- `LeggedRobotDynamicsAD`(OCS2 legged_robotパッケージ、外部)：セントロイダル
  動力学モデル本体。pympcの`Centroidal_Model_Nominal`に相当するが、内部は
  対象外(自動微分(AD)ベースでヤコビアン等を生成する、CasADiに近い設計と
  推測される、**設計上の解釈**)

```cpp
problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

scalar_t frictionCoefficient = 0.7;
RelaxedBarrierPenalty::Config barrierPenaltyConfig;
std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
  const std::string& footName = modelSettings_.contactNames3DoF[i];
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);
  if (useHardFrictionConeConstraint_) {
    problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone", getFrictionConeConstraint(i, frictionCoefficient));
  } else {
    problemPtr_->softConstraintPtr->add(footName + "_frictionCone", getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));
  }
  problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", ...ZeroForceConstraint...);
  problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getZeroVelocityConstraint(*eeKinematicsPtr, i));
  problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", ...NormalVelocityConstraintCppAd...);
}
```

- コスト項は`"baseTrackingCost"`の**1つだけ**(状態・入力の目標追従、
  詳細は後述)
- `frictionCoefficient`(無次元)：ローカル変数の初期値`0.7`は
  **すぐに`loadFrictionConeSettings`の戻り値で上書き**される。
  実際の値(a1の`task.info`)は**`0.3`**
- `useHardFrictionConeConstraint_`(コンストラクタ引数、既定`false`)：
  既定では**ソフト制約**(`softConstraintPtr`、緩和バリア関数によるペナルティ)
  が使われ、**ハード不等式制約**(`inequalityConstraintPtr`、厳密な制約)
  は使われない。pympc側の「既定では摩擦錐制約(不等式)のみ有効、その他
  (着地点box制約等)は既定OFF」という状況とは異なり、こちらは摩擦錐
  自体も**既定ではソフト(ペナルティ)扱い**という違いがある
- 4脚それぞれについて、`_frictionCone`(摩擦錐、ソフト)・`_zeroForce`
  (遊脚中は接地力ゼロ、等式制約)・`_zeroVelocity`(接地中は足先速度ゼロ、
  等式制約)・`_normalVelocity`(スイング軌道追従、等式制約)の**4種類の
  制約**が登録される。命名パターン(`footName + "_xxx"`)から、脚ごとに
  独立した制約オブジェクトが4つ×4種類=16個生成されることが分かる

```cpp
problemPtr_->stateSoftConstraintPtr->add("selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));
setupPreComputation(taskFile, urdfFile, referenceFile, verbose);
rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);
constexpr bool extendNormalizedNomentum = true;
initializerPtr_ = std::make_unique<LeggedRobotInitializer>(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedNomentum);
```

- **自己干渉回避制約(`selfCollision`)がソフト制約として1つ追加される**。
  a1の`task.info`では**8組の実際のリンクペア**
  (`LF_calf/RF_calf`、`LH_calf/RH_calf`、`LF_calf/LH_calf`、
  `RF_calf/RH_calf`、および対応する4組の`_FOOT`ペア。左右対角
  (例:`LF`と`RH`)の組み合わせは含まれない)が設定されており、
  **既定で有効に機能する制約**である点がpympc側の多くの「既定OFF」
  機能とは対照的
- `TimeTriggeredRollout`(OCS2、外部)：SQPの内部でシステムを前進
  シミュレーションする際に使われるロールアウト方式(**未確認**、
  「時間トリガー式」という名前から、固定時間刻みでの前進積分と
  推測される)

---

## `LeggedInterface.cpp` 142〜153行:`setupModel`

この関数の役割:URDFからPinocchio(剛体動力学ライブラリ)モデルを構築し、
セントロイダルモデルの各種次元・パラメータ(`CentroidalModelInfo`)を作る。

```cpp
pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));
centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
    *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
    centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
    modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);
```

- `Pinocchio`(外部ライブラリ)：URDFから剛体動力学(質量・慣性・
  ヤコビアン等)を計算するオープンソースライブラリ。pympcのCasADiに近い
  役割だが、Pinocchioはロボット動力学に特化している
- `centroidal_model::loadCentroidalType(taskFile)`：セントロイダル
  モデルの種類(標準的な運動量ベース vs 完全な剛体力学ベース等の
  バリエーションがOCS2に存在すると推測される、**未確認**)を
  `task.info`から選択する
- `nq - 6`(`nq`はPinocchioの一般化座標数)：`base`の6自由度を除いた
  関節自由度数(=12)を`loadDefaultJointState`へ渡し、`reference.info`
  から関節の基準姿勢(スタンス時の既定関節角度)を読み込む

---

## `LeggedInterface.cpp` 158〜173行:`setupReferenceManager`・`setupPreComputation`

```cpp
auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));
```

- `SwingTrajectoryPlanner`の第2引数`4`はハードコードされた脚の数
  (**実装上の注意点**：`centroidalModelInfo_.numThreeDofContacts`
  ではなくリテラルの`4`が直書きされている。4脚ロボット以外を想定する
  場合はここを直す必要がある)
- `swing_trajectory_config`の実際の値(a1)：`liftOffVelocity=0.05`
  (m/s)、`touchDownVelocity=-0.1`(m/s)、`swingHeight=0.08`(m)、
  `swingTimeScale=0.15`(秒)。詳細は`SwingTrajectoryPlanner`本体を読む
  次章以降で扱う
- `SwitchedModelReferenceManager`・`SwingTrajectoryPlanner`自体の内部は
  この章では扱わず、次章で詳しく読む

```cpp
problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
    *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);
```

- `LeggedRobotPreComputation`(未読)：OCS2の「事前計算(PreComputation)」
  機構に登録される。SQPの各ステージで、コスト・制約が必要とする共通の
  中間結果(順運動学等)を一度だけ計算してキャッシュする役割と推測される
  (**設計上の解釈**、詳細は別ファイルで確認する)

---

## `LeggedInterface.cpp` 178〜203行:`loadGaitSchedule`

この関数の役割:`reference.info`から初期モードスケジュールとデフォルト
歩容テンプレートを読み込み、`GaitSchedule`を構築する。

```cpp
const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

const auto defaultGait = [defaultModeSequenceTemplate] {
  Gait gait{};
  gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
  std::for_each(..., [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
  gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
  return gait;
}();
```

- **実装上の注意点**：即時実行ラムダで`defaultGait`を計算しているが、
  この`defaultGait`変数はこの関数の中で**その後一度も使われていません**
  (`return`文は`initModeSchedule`と`defaultModeSequenceTemplate`から
  `GaitSchedule`を作るだけで、`defaultGait`は無関係)。計算だけして
  捨てられる、pympc側で何度も見られた「計算されるが使われない」パターン
  がここにも存在する
- `modelSettings_.phaseTransitionStanceTime`(既定`0.1`秒)が
  `GaitSchedule`のコンストラクタへ渡される。歩容モードが切り替わる際の
  遷移時間(モード境界での接地維持時間)と推測される(**未確認**)
- 実際の歩容パターン(`initialModeSchedule`/`defaultModeSequenceTemplate`
  の中身)は`reference.info`にあり、この章では中身を確認しない
  (次章の`SwitchedModelReferenceManager`で扱う)

---

## `LeggedInterface.cpp` 208〜252行:`initializeInputCostWeight`・`getBaseTrackingCost`

この関数の役割(`initializeInputCostWeight`):`task.info`の入力コスト
重み行列`R`のうち、関節速度の部分だけを現在の姿勢のヤコビアンで
タスク空間(足先の力・速度)から関節空間へ変換する。

```cpp
matrix_t rTaskspace(info.inputDim, info.inputDim);
loadData::loadEigenMatrix(taskFile, "R", rTaskspace);
matrix_t r = rTaskspace;
r.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) =
    base2feetJac.transpose() * rTaskspace.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) * base2feetJac;
return r;
```

- `rTaskspace`の先頭`totalContactDim`(=3×脚数)行/列は接地力
  (N)へのコスト、後半`actuatedDofNum`(12)行/列は**関節速度**への
  コストとして`task.info`に書かれているが、実際にOCS2へ渡す前に
  `base2feetJac`(base→足先ヤコビアン、初期姿勢で評価)を使って
  `JᵀRJ`という形へ変換している。つまり`task.info`の`R`は「タスク空間
  (足先の並進速度)での重み」として書かれ、コード側で関節空間の重みへ
  変換される、という設計(**事実**、コメントは無いがコードの構造から
  読み取れる)
- ヤコビアンは**`initialState_`(起動時の初期姿勢)1点だけで評価**され、
  以後の関節配置の変化には追従しない(**設計上の解釈**、コスト行列は
  MPCの実行中に再計算されない固定値と考えられる)

この関数の役割(`getBaseTrackingCost`):状態・入力の目標追従コスト
(`Q`・`R`の2次形式)を構築する。

- `Q`(状態コスト重み行列、`task.info`の`Q`ブロック)・`R`
  (`initializeInputCostWeight`で変換済み)を`LeggedRobotStateInputQuadraticCost`
  (`legged_interface/cost/`、未読)へ渡す。具体的な数値は`task.info`の
  `Q`/`R`ブロック(この章では未確認、必要なら別途参照)

---

## `LeggedInterface.cpp` 257〜333行:摩擦錐・ゼロ速度制約のヘルパー

```cpp
loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
```

- `frictionConeSoftConstraint`ブロックの実際の値(a1)：
  `frictionCoefficient=0.3`、`mu=0.1`、`delta=5.0`。pympc側の
  摩擦係数`mu=0.5`([過去のread_code_09](../quadruped_pympc_onboarding/read_code_09_centroidal_nmpc_nominal_setup.md)、
  同シリーズ外参照になるため数値だけ直接記載)より**低い(滑りやすい)
  設定**になっている

```cpp
auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
  EndEffectorLinearConstraint::Config config;
  config.b.setZero(3);
  config.Av.setIdentity(3, 3);
  if (!numerics::almost_eq(positionErrorGain, 0.0)) {
    config.Ax.setZero(3, 3);
    config.Ax(2, 2) = positionErrorGain;
  }
  return config;
};
```

- `positionErrorGain`(既定`0.0`、a1の`task.info`)がゼロなら`config.Ax`
  (位置誤差にかかるフィードバックゲイン行列)は設定されない
  (`if`を通らない)。つまり既定では「接地中は足先**速度**を厳密にゼロに
  する」制約だけで、位置誤差に応じた補正フィードバックは**既定では
  無効**(**事実**、`modelSettings_.positionErrorGain`を`0`以外に
  設定した場合のみZ方向の位置フィードバックが有効になる、Z成分だけに
  設定される理由はコード中に説明がなく**未確認**)

---

## `LeggedInterface.cpp` 338〜372行:`getSelfCollisionConstraint`

この関数の役割:`task.info`の`selfCollision`ブロックからリンクペア・
最小距離・バリアパラメータを読み込み、自己干渉回避のソフト制約を作る。

- 実際の値(a1)：`minimumDistance=0.05`(m)、`mu=1e-2`、`delta=1e-3`、
  監視するリンクペアは8組(前節で確認済み)
- `PinocchioGeometryInterface`(OCS2、外部)がリンク間の実際の距離計算を
  担う。この関数自体は設定ファイルの読み込みと組み立てだけを行う

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `loadGaitSchedule`内で計算される`defaultGait`変数が、計算される
     だけで一度も使われない(死んだ計算)
  2. `SwingTrajectoryPlanner`の脚数がハードコードされた`4`
  3. `getZeroVelocityConstraint`の位置誤差フィードバックがZ成分にしか
     対応しない理由が説明されていない(既定`positionErrorGain=0.0`の
     ため通常は無関係)
- 確認できた重要な事実:
  - 既定(`useHardFrictionConeConstraint_=false`)では摩擦錐制約も
    含めて**ほぼすべての不等式的制約がソフト制約(ペナルティ)**として
    扱われる。pympc側の「摩擦錐だけがハード不等式制約として有効」
    という構成とは設計思想が異なる
  - **自己干渉回避制約が、既定で8組の実リンクペアに対して有効**に
    機能している。pympc側で頻出した「既定OFF」の機能が多い中、
    これは対照的に既定で効いている安全機構
  - 摩擦係数は`0.3`(pympc側の`0.5`より低い設定)
  - 入力コストの重み行列`R`は、`task.info`上はタスク空間(足先速度)の
    重みとして書かれ、起動時の初期姿勢のヤコビアンで関節空間へ変換
    されて使われる。以後のMPC実行中は再計算されない
- 次は、歩容スケジュールとスイング軌道計画を管理する
  `SwitchedModelReferenceManager`・`SwingTrajectoryPlanner`
  (pympcの`PeriodicGaitGenerator`+`FootholdReferenceGenerator`+
  `SwingTrajectoryController`の計画部分に相当)を読みます。
