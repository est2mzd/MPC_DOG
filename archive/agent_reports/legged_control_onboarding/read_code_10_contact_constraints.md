# 接触関連の制約 legged_interface/constraint(FrictionCone・ZeroForce・EndEffectorLinear・ZeroVelocity・NormalVelocity) 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedInterface::setupOptimalControlProblem(...)(read_code_07)が
脚ごとに4種類の制約を OptimalControlProblem へ登録
  → problemPtr_->softConstraintPtr->add(..._frictionCone, ...)   ← 摩擦錐(既定ソフト)
  → problemPtr_->equalityConstraintPtr->add(..._zeroForce, ...)   ← 遊脚中は接地力ゼロ
  → problemPtr_->equalityConstraintPtr->add(..._zeroVelocity, ...) ← 接地中は足先速度ゼロ
  → problemPtr_->equalityConstraintPtr->add(..._normalVelocity, ...) ← 遊脚中はZ速度追従

OCS2のSQPソルバー内部(未確認)が、MPCの各反復・各ステージで:
  → isActive(time) で有効/無効を判定(接地状態に応じて自動切替)
  → getValue / getLinearApproximation / getQuadraticApproximation を呼ぶ
      ← 本ファイル群、MPCの内部反復回数だけ(未確認、既定100Hzの
         MPC呼び出し1回あたり複数回のSQPイテレーション×ホライズン
         ステージ数だけ評価されると推測される)
```

## このファイル/クラスの役割(全体の中での位置づけ)

この一群のファイルが担当するのは、「**接地力・足先速度に関する4種類の
制約を、OCS2のSQPソルバーが評価できる形(値・1階微分・2階微分)で提供する**」
ことです。pympcで言えば、`centroidal_nmpc_nominal.py`の
`create_friction_cone_constraints`(read_code_09、同シリーズ外参照になる
ため直接記載)に相当する部分ですが、legged_controlでは**制約ごとに
独立したC++クラス**として分離されています。

- 4種類の制約すべてに共通するのは、`SwitchedModelReferenceManager::getContactFlags`
  ([read_code_08](read_code_08_switched_model_reference_manager.md))を
  使って「今、この脚は接地しているか」を判定し、**接地状態に応じて
  自動的に有効/無効を切り替える**という設計です
- `EndEffectorLinearConstraint`は、`ZeroVelocityConstraintCppAd`・
  `NormalVelocityConstraintCppAd`が共通で使う**汎用の線形制約の土台**
  (「足先位置・速度の線形結合」という形の制約を表現する)であり、
  それ自体は接地状態の判定を持ちません

対象は`external/legged_control/legged_interface/{include,src}/constraint/`
配下の`FrictionConeConstraint`(149+210行)・`ZeroForceConstraint`
(69+76行)・`EndEffectorLinearConstraint`(94+115行)・
`ZeroVelocityConstraintCppAd`(77+81行)・`NormalVelocityConstraintCppAd`
(75+88行)です。すべて`ocs2::legged_robot`名前空間・
Farbod Farshidian名義のライセンスヘッダを持ちます
([read_code_08](read_code_08_switched_model_reference_manager.md)・
[read_code_09](read_code_09_swing_trajectory_planner.md)と同様、
OCS2本体由来と見られるコードです)。

---

## `FrictionConeConstraint`:摩擦錐

この関数の役割(制約式全体):接地力が摩擦円錐の内側に収まることを
強制する不等式相当の制約。

\[
h(t,x,u)=\mu\,(F_z+F_{grip})-\sqrt{F_x^2+F_y^2+\epsilon}\ \ge 0
\]

| 数式 | コード変数 | 意味 | 単位 |
|---|---|---|---|
| \(\mu\) | `config_.frictionCoefficient` | 摩擦係数 | 無次元、a1既定`0.3`(`task.info`) |
| \(F_{grip}\) | `config_.gripperForce` | グリッパー(吸着等)による補助把持力 | N、既定`0.0`(未使用) |
| \(\epsilon\) | `config_.regularization` | 数値正則化項 | N²、既定`25.0` |
| \(F_x,F_y,F_z\) | `localForce` | 接地力(地形座標系) | N |

```cpp
bool FrictionConeConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}
```

この関数の役割:この脚が接地中のときだけ、摩擦錐制約を有効にする
(遊脚中は接地力自体がゼロ制約(`ZeroForceConstraint`)で縛られるため、
摩擦錐は無関係になる)。

```cpp
void FrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w.setIdentity();
  throw std::runtime_error("[FrictionConeConstraint] setSurfaceNormalInWorld() is not implemented!");
}
```

**コードで確認した事実**：地形の傾きに応じて摩擦錐の向き(`t_R_w`、
world座標系→地形座標系の回転行列)を設定するための関数が用意されて
いますが、**呼ばれると即座に例外を投げる、未実装のスタブ**です。
`t_R_w`は結局`matrix3_t::Identity()`(コンストラクタ時の既定値)のまま
一切変わらず、摩擦錐は**常にworld座標系のZ軸を鉛直と仮定して**計算
されます。[read_code_08](read_code_08_switched_model_reference_manager.md)
で確認した「`terrainHeight`が常に`0.0`固定」という事実と合わせると、
legged_controlは**傾斜地形に対する摩擦錐の傾き補正を一切持たない**
(常に完全に水平な地面を仮定する)ことが、このファイルからも重ねて
確認できます。

```cpp
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim);
  ddhdudu.block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = coneDerivatives.d2Cone_du2;
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
  return ddhdudu;
}
```

- `hessianDiagonalShift`(既定`1e-6`)を対角成分から差し引く、という
  処理が2階微分(ヘシアン)の計算に入っている。これはSQPが内部で扱う
  2次近似(QP)が数値的に扱いやすい形(強凸性の確保等)になるようにする
  ための、ソルバー実装上の技法と考えられる(**設計上の解釈**、
  クラスのdocstringコメントにも「Hessianを厳密な凸2次近似にするための
  シフト」と明記されている)

---

## `ZeroForceConstraint`:遊脚中の接地力ゼロ制約

```cpp
bool ZeroForceConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}
vector_t ZeroForceConstraint::getValue(...) const {
  return centroidal_model::getContactForces(input, contactPointIndex_, info_);
}
```

- `FrictionConeConstraint`とちょうど**逆の接地判定**(`!`が付いている)。
  遊脚中(`isActive`が`true`)のときだけ有効になり、制約の値は「その脚の
  接地力そのもの」。OCS2の等式制約は`g(x,u)=0`という形で扱われると
  考えられ(**設計上の解釈**)、これは「遊脚中は接地力を厳密にゼロにする」
  という素直な制約です

---

## `EndEffectorLinearConstraint`:足先位置・速度の線形制約(共通の土台)

```cpp
vector_t EndEffectorLinearConstraint::getValue(...) const {
  vector_t f = config_.b;
  if (config_.Ax.size() > 0) { f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front(); }
  if (config_.Av.size() > 0) { f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state, input).front(); }
  return f;
}
```

\[
g(x_{ee}, v_{ee}) = A_x\,x_{ee} + A_v\,v_{ee} + b
\]

- `Ax`が空なら位置項を無視、`Av`が空なら速度項を無視する、という
  柔軟な設計(コメント通り「`g(x_{ee})`だけの制約にしたければ`Av`を
  空にする」)。`endEffectorKinematicsPtr_`(OCS2、自動微分(CppAd)ベースの
  足先運動学、**未確認**)が実際の位置・速度をPinocchioモデルから計算する

---

## `ZeroVelocityConstraintCppAd`:接地中の足先速度ゼロ制約

```cpp
bool ZeroVelocityConstraintCppAd::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}
```

- [read_code_07](read_code_07_legged_interface.md)の
  `getZeroVelocityConstraint`で見た設定(`Av=Identity(3)`、`Ax`は
  `positionErrorGain`が非ゼロのときだけZ成分に設定)をそのまま
  `EndEffectorLinearConstraint`(3制約)へ委譲する薄いラッパー。
  実質的な制約式は\(v_{ee}=0\)(`positionErrorGain=0.0`が既定のため
  `Ax`項は無効)で、**「接地中の足先は動かない」という素直な等式制約**

---

## `NormalVelocityConstraintCppAd`:遊脚中のZ速度追従制約(SwingTrajectoryPlannerとの接続点)

```cpp
bool NormalVelocityConstraintCppAd::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}
vector_t NormalVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);
  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}
```

- 遊脚中(`isActive`が`true`)のときだけ有効。`config`を毎回
  `LeggedRobotPreComputation`(次章で読む)から取得し直している点が
  `ZeroVelocityConstraintCppAd`(固定`config`)との違い

**コードで確認した事実([read_code_09](read_code_09_swing_trajectory_planner.md)の
「`getZpositionConstraint`/`getZvelocityConstraint`の呼び出し元は未確認」
という指摘への回答)**：`LeggedRobotPreComputation.cpp`を先取りして
確認したところ、この制約が使う`config.b`は次の式で計算されています。

```cpp
config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
if (settings_.positionErrorGain != 0.0) {
  config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
}
```

すなわち、`EndEffectorLinearConstraint`の`Av`(足先速度の1成分、地形法線
方向=既定でworld座標系のZ方向)と組み合わせて、
\(v_{ee,z} = \text{getZvelocityConstraint}(...)\)
(符号反転しているのは制約を`g=Av*v+b=0`の形で表現するため
\(v_{ee,z}-\dot{z}_{ref}=0\)と同じ意味になる、**事実**)という等式制約に
なります。**[read_code_09](read_code_09_swing_trajectory_planner.md)で
計画した3次スプラインのZ速度が、こうして遊脚中のMPC制約として実際に
使われている**ことがここで確認できました。`positionErrorGain`が既定
`0.0`のため、Z位置についての補正項は既定では効きません。

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `FrictionConeConstraint::setSurfaceNormalInWorld`は呼ばれると即座に
     例外を投げる未実装のスタブで、摩擦錐は常にworld座標系のZ軸基準で
     計算される(`t_R_w`は常に単位行列)
- 確認できた重要な事実:
  - 4つの制約(摩擦錐・ゼロ接地力・ゼロ速度・法線方向速度)はいずれも
    `referenceManagerPtr_->getContactFlags(time)`による接地判定で
    自動的に有効/無効が切り替わる、一貫した設計
  - [read_code_09](read_code_09_swing_trajectory_planner.md)の
    `SwingTrajectoryPlanner`が計画したZ速度(・`positionErrorGain≠0`なら
    Z位置も)は、`NormalVelocityConstraintCppAd`経由でMPCの等式制約
    として実際に使われている(未確認だった接続点がここで確定した)
  - `FrictionConeConstraint`はworld座標系のZ軸を鉛直と仮定する処理が
    ハードコードされており、傾斜地形への追従機構は無い
    ([read_code_08](read_code_08_switched_model_reference_manager.md)の
    「地形推定機能自体が無い」という結論と一貫している)
- 次は、これらの制約が共通で参照する`LeggedRobotPreComputation`
  (事前計算キャッシュ)と、コスト・初期化・自己干渉制約
  (`LeggedRobotQuadraticTrackingCost`・`LeggedRobotInitializer`・
  `LeggedSelfCollisionConstraint`)を読み、`legged_interface`パッケージを
  読み終えます。
