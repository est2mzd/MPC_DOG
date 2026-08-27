# 歩容スケジュール管理 legged_interface/SwitchedModelReferenceManager 逐次解説

## 実行への結びつき(呼び出し連鎖)

```text
LeggedInterface::setupReferenceManager(...)(read_code_07)
  → SwitchedModelReferenceManager コンストラクタ  ← 本ファイル、起動時1回
      (GaitSchedule・SwingTrajectoryPlannerを受け取って保持)

OCS2のSQPソルバー内部(mpc_、外部、未確認)が、MPCを1回解くたびに:
  → SwitchedModelReferenceManager::modifyReferences(...)  ← 本ファイル、
      既定100Hz相当(mpcDesiredFrequency、read_code_05)
      → gaitSchedulePtr_->getModeSchedule(...)  (OCS2 legged_robot、外部)
      → swingTrajectoryPtr_->update(modeSchedule, terrainHeight)
          (read_code_09で読む SwingTrajectoryPlanner)

console.py相当の外部ノード(GaitReceiver、ROS経由、OCS2提供、外部)が
  → gaitSchedulePtr_->setModeSchedule(...) 経由で歩容を切り替える(条件付き)
```

## このファイル/クラスの役割(全体の中での位置づけ)

`SwitchedModelReferenceManager`が担当するのは、「**現在のMPCホライズンに
対応する歩容モードスケジュール(どの脚がいつ接地/遊脚か)を`GaitSchedule`
から切り出し、それを`SwingTrajectoryPlanner`(次章)へ伝える**」ことです。
pympcの`PeriodicGaitGenerator.compute_contact_sequence`(接地スケジュールを
ホライズン分先読みする処理)に相当します。

- 実際の歩容パターン(トロット等の位相定義)の管理・切り替え自体は
  `GaitSchedule`(OCS2 legged_robotパッケージ、外部)側が持つ。
  このクラスはそれをMPCのホライズンに合わせて呼び出す**橋渡し役**
- スイング軌道の高さ・速度計画は`SwingTrajectoryPlanner`(次章)の責務で、
  このクラスは`update`を呼んで**地形高さの前提(後述)を渡すだけ**
- **事実**：このファイルのライセンスヘッダは`Copyright (c) 2021,
  Farbod Farshidian`(OCS2/ETH Zurichの開発者名)であり、`namespace`も
  `legged`ではなく`ocs2::legged_robot`のままです。おそらくOCS2本体
  (`ocs2_legged_robot`パッケージ)のソースをこのリポジトリへそのまま
  コピーしたか、由来が近い実装と考えられます(**推測**)

対象は`external/legged_control/legged_interface/include/legged_interface/SwitchedModelReferenceManager.h`
(70行)・`external/legged_control/legged_interface/src/SwitchedModelReferenceManager.cpp`
(73行)です。

---

## `SwitchedModelReferenceManager.h` 46〜66行:クラス定義

```cpp
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);
  void setModeSchedule(const ModeSchedule& modeSchedule) override;
  contact_flag_t getContactFlags(scalar_t time) const;
  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }
  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }
 protected:
  void modifyReferences(...) override;
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};
```

- `ReferenceManager`(OCS2、外部)を継承する。OCS2のMPCソルバーが
  内部で「目標軌道」と「モードスケジュール」をこのインターフェース
  経由で取得する設計と考えられる(**設計上の解釈**)
- `gaitSchedulePtr_`(型`std::shared_ptr<GaitSchedule>`)：
  [read_code_05](read_code_05_legged_controller.md)で見た`GaitReceiver`
  (ROSトピック経由で歩容コマンドを受信するモジュール)が、この
  `GaitSchedule`インスタンスへ直接歩容を書き込む(コンストラクタで
  `getGaitSchedule()`を通じて共有ポインタが渡される、
  [read_code_05](read_code_05_legged_controller.md)の`setupMpc`参照)

---

## `SwitchedModelReferenceManager.cpp` 47〜57行:`setModeSchedule`・`getContactFlags`

```cpp
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}
```

- `setModeSchedule`：基底クラスへの反映と、`gaitSchedulePtr_`(実データを
  持つ側)への反映を**両方**行う。二重に保持している理由は、基底クラス
  `ReferenceManager`側はOCS2の他コンポーネントが参照する共通インタフェース、
  `gaitSchedulePtr_`側は歩容固有のロジック(周期性等)を持つ実体、という
  役割分担と考えられる(**設計上の解釈**)
- `getContactFlags(time)`：ある時刻の`ModeNumber`(整数のモード番号)を
  `modeNumber2StanceLeg`(OCS2 legged_robot、外部)で4脚分の接地フラグへ
  変換する。呼び出し元は本ファイルの範囲では確認できない(**未確認**、
  `legged_wbc`側で使われると推測される)

---

## `SwitchedModelReferenceManager.cpp` 62〜69行:`modifyReferences`

この関数の役割:MPCの現在の求解ホライズンに対応する範囲のモード
スケジュールを`GaitSchedule`から切り出し、スイング軌道計画を更新する。

```cpp
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}
```

- OCS2ソルバー内部(未確認)から、MPCの各求解サイクルで呼ばれると推測
  される(**設計上の解釈**、`ReferenceManager`の仮想関数のオーバーライド
  であること、`initTime`/`finalTime`という引数名から)
- `gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon)`：
  要求されたホライズン(`finalTime - initTime`)の**前後にさらに同じ幅だけ
  余分に**歩容スケジュールを取得する(合計3倍の時間幅)。この余白の理由は
  コード中に説明がなく、モード切り替えの補間や参照生成での前後参照の
  ためと推測される(**未確認**)

**コードで確認した事実(地形は常に平坦と仮定)**：`terrainHeight`は
**`0.0`固定でハードコード**されており、実際の地形情報は一切参照されて
いません。pympcの`TerrainEstimator`([過去のread_code_04](../quadruped_pympc_onboarding/read_code_04_terrain_estimator.md)、
同シリーズ外参照になるため直接記載:pympc側はpitch方向の地形傾斜だけ
推定し、rollは既定で無効化していた)と比べると、legged_controlは
**地形推定の仕組み自体を持たず、常に完全に平坦な地面を仮定**している
という、より単純な(あるいは、より限定的な)設計になっています。

---

## この章のまとめ

- 見つかった実装上の注意点:
  1. `getModeSchedule`が要求ホライズンの前後にさらに同じ幅を加えて
     取得しているが、その必要性の説明がコード中に無い
- 確認できた重要な事実:
  - `SwingTrajectoryPlanner`へ渡される地形高さは常に`0.0`固定であり、
    legged_controlには地形推定の仕組みが存在しない(pympcの
    `TerrainEstimator`に相当する機能が丸ごと無い)
  - このファイルはOCS2本体(Farbod Farshidian名義)由来と見られる
    コードで、`legged`名前空間ではなく`ocs2::legged_robot`名前空間の
    ままこのリポジトリに組み込まれている
  - 歩容の実際の切り替えは`GaitSchedule`(外部)が保持し、このクラスは
    MPCホライズンへの切り出しと`SwingTrajectoryPlanner`への通知を
    担うだけの薄い橋渡し
- 次は、実際にスイング軌道(高さ・速度)を計画する
  `SwingTrajectoryPlanner`(pympcの`FootholdReferenceGenerator`+
  `SwingTrajectoryController`の計画部分に相当)を読みます。
