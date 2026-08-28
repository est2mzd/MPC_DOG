# スイング軌道(高さ)計画 legged_interface/constraint/SwingTrajectoryPlanner 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
SwitchedModelReferenceManager::modifyReferences(...)(read_code_08)が
MPCの求解サイクルごとに:
  → SwingTrajectoryPlanner::update(modeSchedule, terrainHeight=0.0)
      ← 本ファイル、既定100Hz相当(mpcDesiredFrequency)
      → 各脚・各モード区間ごとにZ方向の3次スプライン軌道を再構築

OCS2のコスト・制約評価(NormalVelocityConstraintCppAd等、未読、次章)が
MPCの内部反復のたびに:
  → SwingTrajectoryPlanner::getZpositionConstraint(leg, time)
  → SwingTrajectoryPlanner::getZvelocityConstraint(leg, time)
      ← 本ファイル、MPCソルバー内部の反復回数だけ(未確認)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`SwingTrajectoryPlanner`が担当するのは、「**各脚が遊脚中である区間について、
足先のZ方向(高さ)位置・速度の目標軌道(3次スプライン)を、歩容スケジュール
から自動的に組み立てる**」ことです。

**重要な事実(pympcとの大きな設計思想の違い)**：このクラスが計画するのは
**Z方向(高さ)だけ**です。足先のX/Y方向(どこに着地するか、pympcの
`FootholdReferenceGenerator`が担っていた役割)は、このクラスには一切
含まれていません。legged_controlでは、着地点のX/Y位置は
`SwingTrajectoryPlanner`のような独立した事前計画コンポーネントではなく、
**OCS2のMPC自身が動力学・コスト・制約(次章以降で読む
`ZeroVelocityConstraintCppAd`等)を通じて最適化の中で決定する**、と
考えられます(**設計上の解釈**、確定にはOCS2内部の未確認)。pympcが
「歩容計画→フットホールド計画(X/Y、Raibert発見的手法)→MPC」という
明確に分離された3段階だったのに対し、legged_controlは「歩容計画
(モードスケジュール)→**Z高さだけ**の事前計画→MPCがX/Y含め全体最適化」
という、より少ない段階数の構成です。

対象は
`external/legged_control/legged_interface/include/legged_interface/constraint/SwingTrajectoryPlanner.h`
(118行)・
`external/legged_control/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp`
(267行)です。このファイルも
[read_code_08](read_code_08_switched_model_reference_manager.md)と同じく
`ocs2::legged_robot`名前空間・Farbod Farshidian名義のライセンスヘッダを
持ちます。

---

## `SwingTrajectoryPlanner.h` 42〜47行:`Config`構造体

```cpp
struct Config {
  scalar_t liftOffVelocity = 0.0;
  scalar_t touchDownVelocity = 0.0;
  scalar_t swingHeight = 0.1;
  scalar_t swingTimeScale = 0.15;  // swing phases shorter than this time will be scaled down in height and velocity
};
```

| メンバ | 型 | 単位 | 既定(ヘッダ) | 実際の値(a1、`task.info`) |
|---|---|---|---|---|
| `liftOffVelocity` | `scalar_t` | m/s | `0.0` | `0.05` |
| `touchDownVelocity` | `scalar_t` | m/s | `0.0` | `-0.1` |
| `swingHeight` | `scalar_t` | m | `0.1` | `0.08` |
| `swingTimeScale` | `scalar_t` | 秒 | `0.15` | `0.15` |

- `liftOffVelocity`(正の値、上向き)：離陸(接地→遊脚)の瞬間の目標
  垂直速度
- `touchDownVelocity`(負の値、下向き)：着地(遊脚→接地)の瞬間の目標
  垂直速度。着地時に真下向きの速度を持たせることで、着地の瞬間の衝撃を
  和らげつつ確実に接地させる意図と考えられる(**設計上の解釈**)
- `swingTimeScale`：この時間より短いスイング区間では、高さ・離着陸速度を
  スケールダウンする(後述の`swingTrajectoryScaling`)

---

## `SwingTrajectoryPlanner.cpp` 66〜135行:`update`(3つのオーバーロード)

この関数の役割:歩容スケジュールと地形高さから、各脚・各モード区間の
Z方向スプライン軌道を再構築する。

```cpp
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, scalar_t terrainHeight) {
  const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
  feet_array_t<scalar_array_t> liftOffHeightSequence;
  liftOffHeightSequence.fill(terrainHeightSequence);
  feet_array_t<scalar_array_t> touchDownHeightSequence;
  touchDownHeightSequence.fill(terrainHeightSequence);
  update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}
```

- [read_code_08](read_code_08_switched_model_reference_manager.md)から
  実際に呼ばれるのはこの1引数(地形高さのスカラー1個)版。すべての脚・
  すべての時刻について、離陸高さ・着地高さの両方を**同じ`terrainHeight`
  (既定`0.0`)**で埋める。より詳細な2引数・3引数版(脚ごと・時刻ごとに
  異なる離陸/着地高さ、最大高さを指定できる)はこのファイル内には
  呼び出し元が無く、地形認識が実装された場合の**将来の拡張用インタフェース**
  と考えられる(**推測**、legged_controlには地形認識機能自体が無いことは
  [read_code_08](read_code_08_switched_model_reference_manager.md)で
  確認済み)

```cpp
for (int p = 0; p < modeSequence.size(); ++p) {
  if (!eesContactFlagStocks[j][p]) {  // for a swing leg
    const int swingStartIndex = startTimesIndices[j][p];
    const int swingFinalIndex = finalTimesIndices[j][p];
    checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

    const scalar_t swingStartTime = eventTimes[swingStartIndex];
    const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
    const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

    const CubicSpline::Node liftOff{swingStartTime, liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity};
    const CubicSpline::Node touchDown{swingFinalTime, touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity};
    const scalar_t midHeight = maxHeightSequence[j][p] + scaling * config_.swingHeight;
    feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);
  } else {  // for a stance leg
    const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
    const CubicSpline::Node touchDown{1.0, liftOffHeightSequence[j][p], 0.0};
    feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);
  }
}
```

- 遊脚区間(`!eesContactFlagStocks[j][p]`)は、`CubicSpline`
  (OCS2 legged_robotの`SplineCpg`、外部)へ「離陸時刻・離陸高さ・離陸速度」
  「最高点の高さ(`midHeight`)」「着地時刻・着地高さ・着地速度」の3点を
  与えて3次スプラインを構築する。この`midHeight`が実質的な
  「最大遊脚高さ」で、既定`terrainHeight=0.0`のとき
  \(\text{midHeight}=0+\text{scaling}\times 0.08\)(a1の`swingHeight`)
- 接地区間(`else`)は、離陸・着地とも同じ高さ(`liftOffHeightSequence`、
  実質的に地形高さそのもの)・速度ゼロの、実質的に**平坦な(高さが変化
  しない)ダミースプライン**を作る。コードコメント「時間を`0.0`→`1.0`と
  任意に設定しているのは、`CubicSpline`内部の`assert`が失敗するのを
  避けるため」より、`CubicSpline`が開始時刻と終了時刻の重複(同時刻)を
  許可しない実装になっており、接地区間には実際の時刻範囲が無い
  (常に高さ一定のはずの区間)ため、意味の無い仮の時刻`0.0`/`1.0`を
  与えているだけと分かる(**事実**)

---

## `SwingTrajectoryPlanner.cpp` 234〜236行:`swingTrajectoryScaling`

この関数の役割:スイング区間の時間長が短いほど、遊脚高さ・離着陸速度を
比例的に小さくするスケール係数を計算する。

```cpp
scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) {
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}
```

- スイング時間が`swingTimeScale`(`0.15`秒)以上あれば係数は`1.0`
  (スケールなし、フルの高さ・速度を使う)。それより短ければ、
  時間に比例して線形に縮小する
- **設計上の解釈**：高速なトロット等でスイング時間が短くなったとき、
  無理に元の高さ・速度で足を振ろうとすると動力学的に無理が生じる
  (間に合わない、過大なトルクが必要になる等)ため、あらかじめ縮小して
  実現可能性を高める安全策と考えられる。pympc側には対応する仕組みは
  無かった(pympcの`step_height`は歩容速度に関わらず固定)

---

## `SwingTrajectoryPlanner.cpp` 140〜229行:区間検出のヘルパー群

この関数の役割(`extractContactFlags`)：モードシーケンス(整数のモード
番号の列)を、脚ごとの接地フラグの時系列(`bool`の配列)へ展開する。

```cpp
const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
```

- `modeNumber2StanceLeg`(OCS2 legged_robot、外部)：モード番号から4脚の
  接地状態を取り出す変換関数。[read_code_08](read_code_08_switched_model_reference_manager.md)の
  `getContactFlags`と同じ関数を使っている

この関数の役割(`findIndex`)：ある遊脚フェーズについて、直前の離陸
(接地→遊脚に切り替わった)時刻のインデックスと、直後の着地(遊脚→接地に
切り替わる)時刻のインデックスを、前後の接地フェーズを線形探索して求める。

```cpp
int startTimesIndex = -1;
for (int ip = index - 1; ip >= 0; ip--) {
  if (contactFlagStock[ip]) { startTimesIndex = ip; break; }
}
int finalTimesIndex = numPhases - 1;
for (size_t ip = index + 1; ip < numPhases; ip++) {
  if (contactFlagStock[ip]) { finalTimesIndex = ip - 1; break; }
}
```

- 直前に接地フェーズが1つも見つからなければ`startTimesIndex`は
  初期値`-1`のまま、直後に接地フェーズが見つからなければ
  `finalTimesIndex`は`numPhases-1`のままになる

この関数の役割(`checkThatIndicesAreValid`)：見つけたインデックスが
不正(`-1`のまま、または範囲外)なら、詳細なログを出して例外を投げる。

```cpp
if (startIndex < 0) {
  ...
  throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
}
```

**コードで確認した事実(前章の疑問への回答)**：もしMPCが要求するホライズン
の**先頭または末尾がちょうど遊脚の途中**で、かつその前後に境界となる
接地フェーズが見つからない場合、この関数は**例外を投げてクラッシュ
します**。[read_code_08](read_code_08_switched_model_reference_manager.md)
で「なぜ`getModeSchedule`は要求ホライズンの前後に同じ幅の余白を追加して
取得するのか、理由の説明が無い」と指摘しましたが、ここでその理由が
裏付けられます。**前後に余白を持たせて歩容スケジュールを取得しておく
ことで、ホライズンの端でも必ず前後の接地フェーズが見つかるようにし、
この`throw`が発生するのを防いでいる**と考えられます(**設計上の解釈**、
両ファイルを合わせて読むことで裏付けが取れた)。

---

## `SwingTrajectoryPlanner.cpp` 50〜61行:`getZvelocityConstraint`・`getZpositionConstraint`

この関数の役割:指定した脚・時刻における、計画済みスプライン軌道上の
Z位置・Z速度を取得する。

```cpp
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].position(time);
}
```

- `lookup::findIndexInTimeArray`(OCS2、外部)で、指定時刻がどのモード
  区間に属するかを二分探索等で特定し、対応するスプラインから位置・速度を
  評価する
- 呼び出し元(このクラスの外)はこのファイルの範囲では確認できない
  (**未確認**、次章以降の制約クラスで使われると推測される)

---

## `SwingTrajectoryPlanner.cpp` 241〜263行:`loadSwingTrajectorySettings`

`task.info`の`swing_trajectory_config`ブロックから`Config`の4つの値を
読み込む(値は本章冒頭の表を参照)。

---

## この章のまとめ

- 見つかった実装上の注意点:
  - 特になし(このファイルはコメント・実装ともに比較的整理されている)
- 確認できた重要な事実:
  - **legged_controlには、pympcの`FootholdReferenceGenerator`に相当する
    「X/Y方向の着地点を事前計画するコンポーネント」が存在しない**。
    `SwingTrajectoryPlanner`が計画するのはZ方向(高さ)の軌道だけであり、
    X/Y方向はMPC自身の最適化に委ねられていると考えられる
  - スイング時間が短いほど遊脚高さ・離着陸速度を線形に縮小する
    `swingTrajectoryScaling`という、pympcには無い安全策がある
  - [read_code_08](read_code_08_switched_model_reference_manager.md)で
    未確認としていた「ホライズン前後の余白取得」の理由が、この章の
    `checkThatIndicesAreValid`の例外送出条件から裏付けられた
- 次は、実際にOCS2の制約として登録される
  `FrictionConeConstraint`・`ZeroForceConstraint`・
  `ZeroVelocityConstraintCppAd`・`NormalVelocityConstraintCppAd`
  (このZ軌道を実際に制約として使うと推測される)を読みます。
