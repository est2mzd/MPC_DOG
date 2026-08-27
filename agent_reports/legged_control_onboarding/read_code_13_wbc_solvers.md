# WBCのQP求解 legged_wbc/HoQp・WeightedWbc・HierarchicalWbc 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedController::update(...)(read_code_05)が毎制御周期:
  → wbc_->update(...)
      既定: WeightedWbc::update(...)  ← 本ファイル、毎制御周期
          → WbcBase::update(...)(read_code_12の共通前処理)
          → formulateConstraints()(EOM+トルク上限+摩擦錐+接地拘束)
          → formulateWeightedTasks(...)(スイング+base加速度+接地力、重み付き)
          → qpOASES::QProblem を1回構築して solve
      既定未使用: HierarchicalWbc::update(...)
          → HoQp を3段(優先度)重ねて solve
```

## このファイル/クラスの役割(全体の中での位置づけ)

この章では、[read_code_12](read_code_12_wbc_base.md)で組み立てた
7種類のタスクを、実際に**1つの数値(関節トルク等)へ解く**部分を読みます。
legged_controlには**2つの異なるQP求解戦略**が用意されています。

- `WeightedWbc`(**既定**、[read_code_05](read_code_05_legged_controller.md)で
  確認済み)：物理的に絶対守るべきタスク(EOM・トルク上限・摩擦錐・
  接地拘束)を**QPのハード制約**にし、それ以外(スイング追従・base加速度
  追従・接地力追従)を**重み付き最小二乗のコスト**として**1本のQP**に
  まとめて解く
- `HierarchicalWbc`(既定では生成されない、`legged_wbc`にのみ存在)：
  すべてのタスクに優先順位を付け、`HoQp`(階層QP、零空間射影による
  段階的な求解)で**上位タスクを侵さない範囲で下位タスクを解く**、
  という、より厳密な優先度型の解き方

対象は`external/legged_control/legged_wbc/include/legged_wbc/HoQp.h`
(70行)・`src/HoQp.cpp`(163行)、`include/legged_wbc/WeightedWbc.h`
(25行)・`src/WeightedWbc.cpp`(75行)、
`include/legged_wbc/HierarchicalWbc.h`(18行)・
`src/HierarchicalWbc.cpp`(23行)です。`HoQp`もTask.h・WbcBase.hと同じく
`Ref: https://github.com/bernhardpg/quadruped_locomotion`という外部参照
コメント付きです。

---

## `WeightedWbc.cpp`(既定で実際に動く経路)

### 50〜52行:`formulateConstraints`

```cpp
Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}
```

この関数の役割:物理的に必ず満たすべき4つのタスクを`+`で連結し、1つの
「制約」として扱えるようにする。

### 54〜57行:`formulateWeightedTasks`

```cpp
Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}
```

この関数の役割:残り3つの「できれば満たしたいタスク」を、それぞれの
重みでスケールしてから連結する。

- `weightSwingLeg_`/`weightBaseAccel_`/`weightContactForce_`
  (`task.info`の`weight`ブロック)：a1の実際の値は
  **`swingLeg=100`、`baseAccel=1`、`contactForce=0.01`**。
  スイング脚の追従が最も重視され(`100`)、接地力の追従は最も軽視
  (`0.01`)されていることが分かる。**設計上の解釈**：接地力はもともと
  EOM・摩擦錐という厳しいハード制約下にあるため、MPCの計画値に強く
  こだわらせる必要が薄く、逆にスイング脚の位置精度は着地失敗に直結
  するため重視されている、と考えられる

### 11〜48行:`update`(QPの組み立てと求解)

```cpp
Task constraints = formulateConstraints();
...
A << constraints.a_,
     constraints.d_;
lbA << constraints.b_,
       -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
ubA << constraints.b_,
       constraints.f_;
```

- **コードで確認した事実**：等式制約`a_*x=b_`と不等式制約`d_*x<=f_`を、
  qpOASESが要求する「1本の線形制約行列`A`+上下限`lbA`/`ubA`」の形へ
  変換している。等式部分は`lbA=ubA=b_`(上下限を同じ値にすることで
  等式を表現)、不等式部分は`lbA=-∞`・`ubA=f_`(片側だけ制限)という、
  QPソルバーでは定番の書き方

```cpp
Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
Eigen::Matrix<...> H = weighedTask.a_.transpose() * weighedTask.a_;
vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
```

\[
\min_x \sum_i w_i\|a_i x - b_i\|^2 \iff \min_x \tfrac12 x^\top H x + g^\top x,\quad H=A_w^\top A_w,\ g=-A_w^\top b_w
\]

- 3つの重み付きタスクを連結した`weighedTask.a_`/`.b_`(既に`*weight`が
  掛けられている)から、二次計画の標準形(\(H\)、\(g\))を作る。これは
  「(重み付き)最小二乗をQPの目的関数に変換する」という定番の変形

```cpp
auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
qpOASES::Options options;
options.setToMPC();
options.printLevel = qpOASES::PL_LOW;
options.enableEqualities = qpOASES::BT_TRUE;
qpProblem.setOptions(options);
int nWsr = 20;
qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
```

- `qpOASES`(このリポジトリのサブパッケージ`qpoases_catkin`として
  vendorされている、外部ライブラリ)：アクティブセット法のQPソルバー
- `options.setToMPC()`：qpOASES組み込みのプリセットで、MPC用途向けに
  チューニングされた設定(反復回数と精度のバランス等、内部は**未確認**)
- `options.enableEqualities = BT_TRUE`：等式制約(`lbA==ubA`の行)を
  ソルバーに明示的に伝え、数値的な扱いを効率化するオプションと
  考えられる(**設計上の解釈**)
- `nWsr = 20`：ワーキングセット再計算(アクティブセット法の反復)の
  上限回数。pympc側の`nlp_solver_max_iter=1`
  ([過去のread_code_09](../quadruped_pympc_onboarding/read_code_09_centroidal_nmpc_nominal_setup.md)、
  同シリーズ外参照になるため直接記載:acadosのSQP内部反復は1回に
  制限されていた)とは異なり、**QPの反復回数には最大20回の余裕がある**
  (制御周期(500〜800Hz)の中で1回のWBC-QPを解く時間予算が、MPC(100Hz)
  より短い周期で毎回発生することを考えると、比較的余裕を持たせている
  設計と考えられる、**設計上の解釈**)

---

## `HoQp.cpp`(既定未使用、階層QPの仕組み)

この関数の役割(コンストラクタ):自分より優先度が高い問題
(`higherProblem`)の解の零空間の中でだけ、自分のタスクを解く。

```cpp
HoQp::HoQp(Task task, HoQp::HoQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  initVars();
  formulateProblem();
  solveProblem();
  buildZMatrix();
  stackSlackSolutions();
}
```

- 上位問題が無い(`higherProblem_==nullptr`)場合は、決定変数空間全体
  (単位行列の零空間=全体)から始める
- `stackedZPrev_`(上位問題までの零空間基底)と、自分のタスクの等式制約
  `task_.a_`を掛け合わせた核(カーネル)を`buildZMatrix`で計算し、それを
  「次の(より優先度の低い)問題が使える自由度」として渡す。これにより、
  **下位のタスクは上位のタスクの等式制約を一切崩せない**(数学的に
  零空間に制約されているため)という、優先度の厳密な保証が実現される
  (**事実**、階層QPの標準的な仕組み)
- 不等式制約についてはスラック変数(`stackedSlackVars_`)を導入し、
  「達成できない不等式はスラックで吸収しつつ、そのスラックをできるだけ
  小さくする」という緩和つきの扱いになっている(**設計上の解釈**、
  `buildHMatrix`でスラック変数に単位行列のコストが課されていることから)

```cpp
void HoQp::solveProblem() {
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  ...
  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  ...
}
```

- こちらも`WeightedWbc`と同じ`qpOASES`を使うが、**優先度レベルごとに
  個別のQPを解く**(3段階なら3回`qpOASES::QProblem`を構築して解く)ため、
  `WeightedWbc`(1回のQPで済む)より**計算コストが高い**と考えられます
  (**設計上の解釈**)

---

## `HierarchicalWbc.cpp`(既定未使用)

```cpp
Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
Task task2 = formulateContactForceTask(inputDesired);
HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
return hoQp.getSolutions();
```

- 優先度は3段階：**レベル0(最優先)**=EOM+トルク上限+摩擦錐+接地拘束
  (`WeightedWbc`の「ハード制約」と全く同じ内容)、**レベル1**=base加速度
  +スイング脚追従、**レベル2(最低優先)**=接地力追従
- `WeightedWbc`との違いは、レベル1・2が「重み付きの妥協」ではなく
  「レベル0を一切崩さない範囲で**可能な限り**満たす、優先度による
  順番」という点。数学的な保証はこちらの方が厳密ですが、その分、
  QPを複数回解く必要があり計算コストが増えます
- **実装上の注意点(再掲)**：[read_code_05](read_code_05_legged_controller.md)で
  確認した通り、`legged_controllers/src/LeggedController.cpp`は
  `HierarchicalWbc.h`をimportしているものの、実際にインスタンス化する
  コードはどこにも無く、**このクラスは既定では一切実行されません**

---

## この章のまとめ

- 見つかった実装上の注意点:
  - 特になし(このファイル群自体に新たな問題点は見つからなかった。
    `HierarchicalWbc`が未使用という点はread_code_05で既出)
- 確認できた重要な事実:
  - 既定で動くのは`WeightedWbc`(1本の重み付き最小二乗QP、ハード制約+
    3つの重み付きソフトタスク)。実際の重み(a1)は
    `swingLeg=100 > baseAccel=1 > contactForce=0.01`で、スイング脚の
    追従精度を最優先し、接地力の追従は軽視する設計
  - `HierarchicalWbc`(既定未使用)は、`HoQp`による厳密な優先度型QPを
    3段階(EOM等→base/スイング→接地力)で解く、より数学的に厳密だが
    計算コストの高い代替方式として用意されている
  - QPソルバーはどちらも`qpOASES`(vendored、`qpoases_catkin`)で
    共通。ワーキングセット反復上限は`WeightedWbc`側で`20`回
- これで`legged_wbc`パッケージ(read_code_12〜13)を読み終えました。
  次は、対話的な操作を担う`legged_controllers/TargetTrajectoriesPublisher`・
  `SafetyChecker`(pympcの`console.py`+安全チェック相当)を読み、
  legged_controlの主要な制御パイプライン一式の読解を完了します。
