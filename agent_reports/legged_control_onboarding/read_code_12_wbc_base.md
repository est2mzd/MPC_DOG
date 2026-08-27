# 全身制御タスクの定式化 legged_wbc/Task・WbcBase 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedController::update(...)(read_code_05)が毎制御周期:
  → wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec())
      (実体は WeightedWbc::update、次章。内部で WbcBase::update と
       formulateXxxTask() 群を呼ぶ)  ← 本ファイル、毎制御周期
      (a1/go1既定500Hz、aliengo既定800Hz、read_code_01)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`WbcBase`が担当するのは、「**MPCが出した最適状態・入力(base軌道・接地力の
計画)と、実測状態から、全身動力学的に矛盾の無い『タスク』(等式・不等式の
線形制約の集まり)を組み立てる**」ことです。pympcの
[WBInterface.compute_stance_and_swing_torque](../quadruped_pympc_onboarding/read_code_12_wb_interface_torque.md)
(同シリーズ外参照になるため直接記載:立脚は\(\tau=-J^\top F\)、遊脚は
カルテシアン空間PD制御、という2つの制御則を切り替えるだけの設計)に相当
する部分ですが、legged_controlは**もっと精緻**です。「まず浮遊base込みの
全身運動方程式(EOM)を等式制約として立て、そこに接地拘束・摩擦錐・
トルク上限・base加速度追従・スイング脚追従・接地力追従という複数の
タスクを重ねて、それら全体を1つの二次計画問題(QP)として解く」という、
**全身動力学ベースのタスク優先度型WBC**です。

- `Task`：等式`a_*x=b_`と不等式`d_*x<=f_`の組を表す、汎用のデータ構造。
  タスク同士の足し算(`+`)・重み付け(`*`)ができる
- `WbcBase`：決定変数`x`(base加速度6+接地力3×脚数+関節トルク12)を対象に、
  7種類のタスクを作る関数群を持つ。**実際にQPをどう解くか(重み付き1本の
  QPか、階層的なQPか)は持たない**(継承先の`WeightedWbc`/
  `HierarchicalWbc`、次章の責務)

対象は`external/legged_control/legged_wbc/include/legged_wbc/Task.h`
(69行)、`include/legged_wbc/WbcBase.h`(62行)・`src/WbcBase.cpp`
(271行)です。ライセンスヘッダのコメントに
`Ref: https://github.com/bernhardpg/quadruped_locomotion`とあり、
階層QP方式のWBCという設計自体は外部の参考実装に基づくと考えられます
(**事実**)。

---

## `Task.h` 17〜66行:`Task`構造体

```cpp
class Task {
 public:
  Task(matrix_t a, vector_t b, matrix_t d, vector_t f) : a_(...), d_(...), b_(...), f_(...) {}
  Task operator+(const Task& rhs) const { return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), ...}; }
  Task operator*(scalar_t rhs) const { return {rhs*a_, rhs*b_, rhs*d_, rhs*f_}; }
  matrix_t a_, d_;
  vector_t b_, f_;
};
```

- \(a\cdot x=b\)(等式)、\(d\cdot x\le f\)(不等式)という2種類の線形式を
  1つにまとめたデータ構造
- `operator+`：2つのタスクの行列・ベクトルを**縦に連結**する。複数の
  等式制約を「まとめて1つの大きい等式制約」として扱える
- `operator*`：全行列・ベクトルをスカラー倍する。**重み付きタスク**
  (次章の`WeightedWbc`)を作るのに使われる

---

## `WbcBase` の決定変数

```cpp
// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
```

| 区間 | 内容 | 次元(a1) |
|---|---|---|
| `[0, generalizedCoordinatesNum)` | 一般化座標の**加速度**(\(\dot u\)、base6+関節12) | 18 |
| `[gc, gc+3*numContacts)` | 各脚の接地力\(F\)(N、world座標系) | 12 |
| `[gc+3*numContacts, ...)` | 関節トルク\(\tau\)(N·m) | 12 |

- 合計次元は`18+12+12=42`。pympcのMPC状態(30次元)や入力(24次元)とは
  異なる、**WBC側だけが持つ独自の決定変数空間**であることに注意
  (MPCの決定変数とWBCの決定変数は別物)

---

## `WbcBase.cpp` 29〜43行:`update`

この関数の役割:接地脚数を数え、実測状態とMPCの目標状態の両方について
Pinocchioの運動学・動力学キャッシュを更新する(継承先の`update`から
呼ばれる共通の前処理)。

```cpp
contactFlag_ = modeNumber2StanceLeg(mode);
numContacts_ = std::count(contactFlag_で true の数);
updateMeasured(rbdStateMeasured);
updateDesired(stateDesired, inputDesired);
return {};
```

- `mode`(MPCが計画した現在時刻のモード番号)から接地フラグを得る。
  実測の接触センサ([read_code_04](read_code_04_unitree_hw.md)・
  [read_code_03](read_code_03_legged_hw_sim.md))ではなく、**MPCの計画上の
  接地状態**を使っている点に注意(**事実**、実測接触との整合性はここでは
  保証されない)
- 基底クラスの`update`自体は空のベクトル`{}`を返すだけで、実際のQP求解は
  継承先(`WeightedWbc`/`HierarchicalWbc`)が担う

---

## `WbcBase.cpp` 45〜81行:`updateMeasured`

この関数の役割:実測の剛体状態からPinocchioの質量行列・非線形項・
ヤコビアン(位置・速度)を計算してキャッシュする。

```cpp
pinocchio::crba(model, data, qMeasured_);
data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
```

- `pinocchio::crba`(複合剛体アルゴリズム、外部)：関節空間の質量行列
  \(M\)を計算する。CRBAは上三角部分しか埋めないため、直後の行で
  下三角へコピーして対称行列を完成させている(**事実**、Pinocchioの
  典型的な使い方)
- `pinocchio::nonLinearEffects`：コリオリ力・遠心力・重力をまとめた
  非線形項(`data.nle`)を計算する
- 全接地点(4脚)についてヤコビアン`j_`とその時間微分`dj_`を計算して
  キャッシュする。これらは後段の複数のタスク関数から再利用される
  (**設計上の解釈**：pympcのように毎回別々に計算するのではなく、
  1周期に1回だけ計算してタスク関数間で共有する効率化)

---

## `WbcBase.cpp` 97〜108行:`formulateFloatingBaseEomTask`(等式、最優先)

この関数の役割:浮遊base込みの全身運動方程式を等式制約として定式化する。

\[
M\dot u - J^\top F - S^\top \tau = -h(q,v)
\]

| 数式 | コード変数 | 意味 |
|---|---|---|
| \(M\) | `data.M` | 関節空間の質量行列(18×18) |
| \(\dot u\) | 決定変数の先頭18要素 | 一般化加速度 |
| \(J\) | `j_` | 4脚分の接地点ヤコビアン(12×18) |
| \(F\) | 決定変数の中間12要素 | 接地力 |
| \(S\) | `s`(このタスク内でのみ構築、`[0,I]`) | 関節選択行列(baseにはトルクが直接かからないことを表す) |
| \(\tau\) | 決定変数の末尾12要素 | 関節トルク |
| \(h(q,v)\) | `data.nle` | コリオリ・遠心力・重力 |

- `s`：先頭6列(base)がゼロ、残り12列(関節)が単位行列。「baseは
  アクチュエータで直接駆動されない(下位の関節トルクを介してのみ動く)」
  という浮遊base系の基本的な事実を表現している
- pympcの\(\tau=-J^\top F\)という**簡略化された立脚トルク公式**とは違い、
  こちらは**質量行列・コリオリ力まで含めた厳密な運動方程式**をQPの制約
  として毎周期解いている(**事実**、より動力学的に正確だが、Pinocchioの
  逆動力学計算が毎周期必要になる分、計算コストは高い)

---

## `WbcBase.cpp` 110〜123行:`formulateTorqueLimitsTask`(不等式)

```cpp
d.block(0, ..., actuatedDofNum, actuatedDofNum) = I;
d.block(actuatedDofNum, ..., actuatedDofNum, actuatedDofNum) = -I;
for (size_t l = 0; l < 2*actuatedDofNum/3; ++l) { f.segment<3>(3*l) = torqueLimits_; }
```

- \(\tau \le \tau_{max}\)と\(-\tau \le \tau_{max}\)(つまり\(|\tau|\le\tau_{max}\))
  を、`d`(不等式行列)への`+I`・`-I`の2ブロックで表現する典型的な書き方
- `torqueLimits_`(N·m、`task.info`の`torqueLimitsTask`)：a1の実際の値は
  **HAA・HFE・KFEいずれも`33.5`**(N·m、3要素の`vector_t`を12関節分
  繰り返し適用)

---

## `WbcBase.cpp` 125〜140行:`formulateNoContactMotionTask`(等式)

```cpp
a.block(3*j, 0, 3, gc) = j_.block(3*i, 0, 3, gc);
b.segment(3*j, 3) = -dj_.block(3*i, 0, 3, gc) * vMeasured_;
```

\[
J_{stance}\,\dot u = -\dot J_{stance}\,v
\]

- 接地中の脚(`contactFlag_[i]`が`true`)についてのみ、「その足先の加速度
  がゼロになる」という等式制約を立てる。足先速度\(v_{ee}=Jv\)を時間微分
  すると\(\dot v_{ee}=J\dot u+\dot J v\)となるため、\(\dot v_{ee}=0\)を
  課すと\(J\dot u=-\dot J v\)になる、という導出(**事実**、標準的な
  接地拘束の定式化)。[read_code_10](read_code_10_contact_constraints.md)で
  読んだMPC側の`ZeroVelocityConstraintCppAd`(足先**速度**をゼロにする
  制約)と対になる、WBC側は**足先加速度**をゼロにする制約、という違いが
  ある

---

## `WbcBase.cpp` 142〜172行:`formulateFrictionConeTask`(等式+不等式)

```cpp
// 遊脚: 接地力を強制的にゼロにする(等式)
a.block(3*j++, gc + 3*i, 3, 3) = I;  // (bはゼロ)

// 接地中: 線形化した摩擦錐(五角錐近似、不等式)
matrix_t frictionPyramic(5, 3);
frictionPyramic << 0, 0, -1,
                    1, 0, -frictionCoeff_,
                   -1, 0, -frictionCoeff_,
                    0, 1, -frictionCoeff_,
                    0,-1, -frictionCoeff_;
d.block(5*j++, gc + 3*i, 5, 3) = frictionPyramic;
```

**コードで確認した事実(MPCとWBCで摩擦錐の定式化方式が違う)**：
[read_code_10](read_code_10_contact_constraints.md)のMPC側
`FrictionConeConstraint`は\(\mu F_z-\sqrt{F_x^2+F_y^2+\epsilon}\ge0\)
という**滑らかな(2次錐に正則化項を加えた)非線形制約**でしたが、こちらの
WBC側は**5面の平面で近似した線形の摩擦錐(五角錐)** を使っています。
QP(2次計画問題、線形制約)として解く以上、非線形の錐制約をそのまま
扱えないための近似と考えられます(**設計上の解釈**)。摩擦係数の値
自体は両方とも`task.info`の別ブロック(MPC側`frictionConeSoftConstraint`、
WBC側`frictionConeTask`)から独立に読み込まれますが、a1では**どちらも
`0.3`**で一致していることを確認済みです。

- 五角錐の1行目`(0,0,-1)`：\(-F_z\le0\)、すなわち\(F_z\ge0\)
  (接地力は常に地面を押す方向=引っ張り力は禁止)
- 残り4行：\(\pm F_x \le \mu F_z\)、\(\pm F_y \le \mu F_z\)という、
  正方形(4面)で円形の摩擦円錐を近似する形

---

## `WbcBase.cpp` 174〜200行:`formulateBaseAccelTask`(等式)

この関数の役割:MPCが計画したセントロイダル運動量の変化率(次の目標状態
への遷移)から、逆算してbase(浮遊6自由度)の目標加速度を求め、それを
追従させる等式制約を立てる。

```cpp
vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
inputLast_ = inputDesired;
...
Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
centroidalMomentumRate.noalias() -= ADot * vDesired;
centroidalMomentumRate.noalias() -= Aj * jointAccel;
Vector6 b = AbInv * centroidalMomentumRate;
```

- `jointAccel`(rad/s²)：MPCの**今回の目標入力と前回の目標入力の差分**を
  `period`(制御周期)で割った、関節加速度の**有限差分近似**。**実装上の
  注意点**：`inputLast_`は`WbcBase`のコンストラクタで`Zero`初期化される
  ため、**起動直後の最初の1回はゼロとの差分**になり、不自然に大きい
  加速度になる可能性がある(**推測**、起動直後の過渡応答への影響は
  未確認)
- `getNormalizedCentroidalMomentumRate`(OCS2、外部)：MPCが計画した入力
  (接地力等)から、セントロイダル運動量(線形+角運動量)の変化率を計算する
- \(A\)(セントロイダル運動量行列)の**base成分の逆行列**
  (`computeFloatingBaseCentroidalMomentumMatrixInverse`、OCS2、外部)を
  使って、関節部分の寄与(`Aj*jointAccel`)と運動量行列自体の時間変化
  (`ADot*vDesired`)を差し引いた残りから、**base加速度を逆算**する
- この関数はpympcには存在しない発想の処理です。pympcはMPCが直接
  base位置・姿勢のホライズンを出力しそれをそのまま使っていましたが、
  legged_controlはMPCの出力(セントロイダル運動量変化)から**WBCが
  base加速度を再構築**するという、1段階余分な変換を挟んでいます
  (**設計上の解釈**、なぜこの変換が必要かはコード中に説明が無く、
  MPCの状態表現(運動量ベース)とWBCの決定変数(加速度ベース)の違いを
  埋めるためと推測される)

---

## `WbcBase.cpp` 202〜225行:`formulateSwingLegTask`(等式)

```cpp
vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
a.block(3*j, 0, 3, gc) = j_.block(3*i, 0, 3, gc);
b.segment(3*j, 3) = accel - dj_.block(3*i, 0, 3, gc) * vMeasured_;
```

\[
\ddot p_{ee}^{target} = k_p(p_{des}-p_{meas})+k_d(v_{des}-v_{meas})
\]

- 遊脚中の足について、カルテシアン空間のPD制御則で**目標加速度**を作り、
  それを`formulateNoContactMotionTask`と同じ導出(\(\dot v_{ee}=J\dot u+
  \dot J v\))でヤコビアンを介した等式制約に変換する
- `swingKp_`(N·m/... 実質的にはm/s²のPDゲイン、無次元的には(1/s²))・
  `swingKd_`：`task.info`の`swingLegTask`ブロックから読み込む、実際の
  a1の値は**`kp=350`、`kd=37`**。pympcの
  [SwingTrajectoryController](../quadruped_pympc_onboarding/read_code_13_swing_trajectory_controller.md)
  (同シリーズ外参照になるため直接記載:`swing_position_gain_fb=500`・
  `swing_velocity_gain_fb=10`)と桁は近いが、数値としては異なる
  (kp:350 vs 500、kd:37 vs 10)
- `posDesired`/`velDesired`はMPCが計画した状態(`pinocchioInterfaceDesired_`)
  から順運動学で計算した足先位置・速度。**MPCの状態には直接「足先の
  目標位置」という成分は無い**([read_code_09](read_code_09_swing_trajectory_planner.md)で
  見た通り、legged_controlのMPCはX/Y位置を明示的な計画変数として
  持たない)ため、ここでMPCが最適化した関節角度等から**順運動学で
  逆算**して初めて「目標足先位置」が得られる、という設計になっている
  (**設計上の解釈**)

---

## `WbcBase.cpp` 227〜238行:`formulateContactForceTask`(等式、最も優先度が低い)

```cpp
a.block(3*i, gc + 3*i, 3, 3) = I;
b = inputDesired.head(a.rows());
```

- 「WBCが決定する接地力`F`を、MPCが計画した接地力(`inputDesired`の
  先頭12要素)にできるだけ近づける」という、最も単純な追従タスク。
  他の等式制約(EOM等)と衝突する場合は、次章で見る重み付け/優先度に
  よってこのタスクが妥協される

---

## `WbcBase.cpp` 240〜268行:`loadTasksSetting`

`task.info`から`torqueLimitsTask`(N·m)・`frictionConeTask.frictionCoefficient`
(無次元、a1で`0.3`)・`swingLegTask.kp`/`kd`(a1で`350`/`37`)を読み込む。
継承先(`WeightedWbc::loadTasksSetting`)がこれを呼んだ上で、さらに
重み(`weight.swingLeg`等)を追加で読み込む(次章)。

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `formulateBaseAccelTask`の`jointAccel`は起動直後の最初の1回、
     ゼロ初期化された`inputLast_`との差分になり、不自然な値になる
     可能性がある
- 確認できた重要な事実:
  - WBCは全身動力学(質量行列・コリオリ力・ヤコビアン)を毎周期
    Pinocchioで計算し、pympcの\(\tau=-J^\top F\)より遥かに精緻な
    運動方程式ベースの制約としてQPに組み込んでいる
  - MPC側([read_code_10](read_code_10_contact_constraints.md))が
    滑らかな非線形摩擦錐を使うのに対し、WBC側は5面の線形近似(摩擦
    ピラミッド)を使う、**同じ物理量に対して2つの異なる数学的近似**が
    このリポジトリ内に共存している
  - MPCの出力(セントロイダル運動量ベース)を、WBCが使う加速度ベースの
    決定変数へ変換する`formulateBaseAccelTask`という追加の変換層が
    存在し、pympcには対応する処理が無い
  - スイング脚の目標位置・速度は、MPCの状態から順運動学で都度計算
    される(MPCがX/Y足先位置を直接持たないため)
- 次は、これらのタスクを実際にどう組み合わせてQPを解くか
  (`WeightedWbc`(既定、1本の重み付きQP)と`HierarchicalWbc`(既定未使用、
  `HoQp`による階層QP))を読みます。
