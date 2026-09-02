# Step 17：Go2 前方ジャンプ（後脚踏切）— 実装記録

対象読者：この課題に初めて触れる人。
状態：**進行中。** プランナ側の土台（接触スケジュール・強制ジャンプモード・
サブフェーズ・前向き踏切）はコードとユニットテストで完成。物理シミュレーション
（Stage 2〜6）は未実施。事実・計測・推測を分けて書く。

関連：実装前分析は [step_17_forward_jump_code_analysis.md](./step_17_forward_jump_code_analysis.md)。

---

## 1. 背景

Go2 に幅 0.30 m・深さ 1.0 m の穴を「またぐ」のではなく「飛び越え」させたい。
理想は 8 段階：全脚接地で腰を落とす → 重心を後方へ → 前脚離地 → 後脚 2 本へ荷重集中 →
後脚で前上方へ踏切 → 四脚離地（飛翔）→ 前脚から着地 → 後脚着地で安定化。

分析（前掲）で、既存の「リープ」は実質「四脚接地スクワット →（運が良ければ）
四脚同時飛翔 → 四脚接地」であり、後脚だけの支持（REAR_PUSH）も前脚だけの着地
（FRONT_LAND）も**到達不能**（`local_footstep_planner.cpp` の後脚接触分岐がデッドコード）
だと判明した。GBP は点質量＋単一合力モデルで後脚荷重配分もピッチモーメントも
表現できず、踏切の水平力の向きは乱数。

---

## 2. 目的

課題 §6〜§13 のうち、まず「計測可能なジャンプ」の前提となるプランナ経路を作る：

1. `PRELOAD / REAR_PUSH / FLIGHT / FRONT_LAND / SETTLE` を**到達可能**な明示フェーズにする。
2. primitive ID を単一定義元に集約する。
3. `jump_mode`（OFF / AUTO / FORCE_LEAP）で「またぎ」ではなくジャンプを強制できるようにする。
4. 踏切の水平 GRF を**着地方向**へ向ける（乱数をやめる）。
5. 既存歩行・STAND・メッセージ互換を壊さない。

そのうえで Stage 2〜6 の MuJoCo 検証で、後脚踏切・四脚実離地・穴越え・安定着地を
**計測値**で確認する。

---

## 3. 元コードの問題（分析より要約）

| 問題 | 内容 | 根拠 |
|---|---|---|
| A | 通常接続がリープより先に採用され、`REACHED` で即終了 | `rrt.cpp:21-37` |
| B | 後脚だけの接触分岐が到達不能（デッドコード） | `local_footstep_planner.cpp:531-538`（旧） |
| C | GBP `Action` が胴体合力のみ。後脚荷重配分・ピッチモーメント不可 | `planning_utils.hpp:243-251` |
| D | 踏切の水平 GRF の向きが一様乱数 | `planning_utils.cpp:626-631`（旧） |
| E | NMPC・Inverse Dynamics が計画接触のみ使用 | `quad_nlp.cpp:1752` / `inverse_dynamics_controller.cpp:120-123` |
| 追加 | primitive ID が 3 ファイルに重複定義 | `planning_utils.hpp` / `local_footstep_planner.hpp` / `rviz_interface.hpp` |
| 追加 | NMPC simple model の脚別鉛直 GRF 上限 150 N/脚（後脚 2 本で 300 N ≈ 1.9 BW） | `go2.yaml:46-47` |

---

## 4. 変更したファイル（コミット単位）

### commit `6fef3f4` — reachable rear-push contact phase

| ファイル | 変更 |
|---|---|
| `quad_utils/include/quad_utils/primitive_ids.hpp`（新規） | primitive ID の単一定義元。0..3 は上流値を維持、4..7 が `PRELOAD/REAR_PUSH/FRONT_LAND/SETTLE`。`isJumpPrimitive()` / `primitiveHasContact()` を提供 |
| `global_body_planner/.../planning_utils.hpp` | `enum Phase` の値を `quad_utils::PRIM_*` から取る形へ。`PRELOAD..SETTLE` を追加 |
| `local_planner/.../local_footstep_planner.hpp` | 生の `const int LEAP_STANCE=1...` を廃し `static constexpr int ... = quad_utils::PRIM_*` へ。サブフェーズ定数追加 |
| `quad_utils/include/quad_utils/rviz_interface.hpp` `.../rviz_interface.cpp` | 同上。可視化の色分けにサブフェーズを追加 |
| `local_planner/src/local_footstep_planner.cpp` | `computeContactSchedule` のデッドコードを primitive → 接触表の `switch` に差し替え。`REAR_PUSH → {0,1,0,1}`、`FRONT_LAND → {1,0,1,0}`、`PRELOAD/SETTLE → {1,1,1,1}`、`FLIGHT → {0,0,0,0}` |
| `local_planner/test/test_footstep_planner.cpp` | Stage 1 ユニットテスト `ContactScheduleForwardJumpSubPhases` |

### commit `a300535` — forced-leap mode + jump sub-phases + directed takeoff

| ファイル | 変更 |
|---|---|
| `planning_utils.hpp` | `enum JumpMode {JUMP_OFF,JUMP_AUTO,JUMP_FORCE_LEAP}`。`PlannerConfig` に `jump_mode / jump_preload_fraction(0.4) / jump_front_land_fraction(0.5)`。`Action` に `bool is_jump` |
| `global_body_planner/src/global_body_planner.cpp` | `global_body_planner.jump_mode` パラメータ（`off`/`auto`/`force_leap`）を読み、config へマップ。`off` は `enable_leaping:=false` 相当。`jump_*_fraction` も宣言 |
| `global_body_planner/src/rrt.cpp` | `newConfig`：`jump_mode == FORCE_LEAP` のときは通常接続が `REACHED` でも即 return しない（リープを優先し、通常接続はフォールバック） |
| `global_body_planner/src/planning_utils.cpp` | `getRandomLeapAction`：`a.is_jump = (jump_mode == FORCE_LEAP)`。`refineStance`：ジャンプ踏切のとき水平 GRF を `s.vel` の水平方位へ向け、摩擦上限の 0.9 倍を使う（乱数をやめる）。`interpStateActionPair`：`is_jump` のとき leap stance を `PRELOAD→REAR_PUSH`、land stance を `FRONT_LAND→SETTLE` に時間分割してスタンプ |
| `global_body_planner/test/test_global_body_plan.cpp` | `JumpActionInterpEmitsSubPhases`：ジャンプアクションが 5 サブフェーズを正順で出し、通常リープは従来 ID のままであることを確認 |

---

## 5. 状態遷移（実装した接触スケジュール）

| primitive | ID | FL | RL | FR | RR | 意味 |
|---|--:|--:|--:|--:|--:|---|
| `PRELOAD` | 4 | 1 | 1 | 1 | 1 | 腰下げ・重心後方 |
| `REAR_PUSH` | 5 | 0 | 1 | 0 | 1 | 後脚のみで前上方踏切 |
| `FLIGHT` | 2 | 0 | 0 | 0 | 0 | 弾道飛行 |
| `FRONT_LAND` | 6 | 1 | 0 | 1 | 0 | 前脚先着地 |
| `SETTLE` | 7 | 1 | 1 | 1 | 1 | 後脚着地・安定化 |

脚順序 `0=FL 1=RL 2=FR 3=RR`。GBP がジャンプアクションを 1 個計画すると、
`interpStateActionPair` が leap stance の前半 `jump_preload_fraction` を PRELOAD、
残りを REAR_PUSH、land stance の前半 `jump_front_land_fraction` を FRONT_LAND、
残りを SETTLE としてスタンプする。これが `body_plan_msg.primitive_ids` に載り、
local planner の `computeContactSchedule` が上表の接触へ変換する。

---

## 6. 必要速度・インパルスの計算（推定・未検証）

m = 16.1 kg、g = 9.81、体重 W ≈ 158 N。同高着地の弾道近似 `L = 2 v_x v_z / g`。
目標飛距離 L = 0.45 m（穴 0.30 + 踏切余裕 0.075 + 着地余裕 0.075）で対称配分すると：

| 量 | 値 |
|---|---|
| 離陸速度 v_x ≈ v_z | ≈ 1.49 m/s |
| 飛翔時間 | ≈ 0.30 s |
| 水平インパルス J_x | ≈ 19.2 N·s（後脚合計 平均 F_x ≈ 128 N, T_push=0.15 s） |
| 鉛直インパルス J_z | ≈ 47.7 N·s（後脚合計 平均 F_z ≈ 318 N ≈ 2.0 W、ピーク ≈ 477 N ≈ 3.0 W） |
| 滑らない条件 F_x ≤ μ F_z | μ ≥ 0.40（現状 GBP 0.25 / NMPC 0.3 では滑る） |

**帰結**：NMPC simple model の脚別上限 150 N/脚（後脚 2 本 300 N）は必要ピーク
≈ 477 N に届かない。ジャンプ時は後脚 `u_ub` z の引き上げ、または T_push 延長・
飛距離縮小が要る。摩擦係数もジャンプ用に 0.4〜0.6 へ上げる必要がある。
→ **NMPC 側の変更（未実施）。**

---

## 7. NMPC への入力（現状）

`computeContactSchedule` が出す bool 4×H の `contact_schedule` を
`quad_nlp.cpp:updateContactSchedule` が `contact_sequence_` にコピー。
`REAR_PUSH` の列は `{0,1,0,1}` なので NMPC は前脚 GRF を 0 にゲートし、
接地している後脚 2 本へ名目 GRF `mass·g/num_contacts` を**等分**する。
前上方インパルスの非対称配分・後脚上限引き上げは**未実装**。

---

## 8. Inverse Dynamics からトルクまで（現状）

`inverse_dynamics_controller.cpp`：local plan を時間補間して参照状態を作り、
GRF は NMPC 出力の ZOH、接触は参照状態の `feet[i].contact`（＝計画接触）。
`computeInverseDynamics(acc, grf, contact_mode, tau)` でトルク。
接地脚は `stance_kp/kd` 位置保持、遊脚は Cartesian PD。
**実測接触は torque 経路に未導入**（問題 E は未対応）。

---

## 9. Stage 別試験結果

| Stage | 内容 | 状態 | 結果 |
|---|---|---|---|
| 1 | 接触スケジュール単体テスト | **完了** | `ContactScheduleForwardJumpSubPhases`（local_planner）＋ `JumpActionInterpEmitsSubPhases`（global_body_planner）＋ 既存 trot/STAND/flight 回帰。`colcon test`：**112 tests, 0 failures**（quad_utils / global_body_planner / local_planner） |
| 2 | 後脚荷重移動（平地・前脚を 50 ms 浮かす） | 未実施 | — |
| 3 | 後脚踏切（平地・後脚支持 100〜150 ms・小インパルス） | 未実施 | — |
| 4 | 小ホップ（平地・数 cm 全脚離地） | 未実施 | — |
| 5 | 短い前方ジャンプ（平地・0.10→0.20→0.30 m） | 未実施 | — |
| 6 | 穴越え（0.30 m 幅・1.0 m 深・FORCE_LEAP） | 未実施 | — |

Stage 2 以降は GBPL → NMPC → MuJoCo の追従ループが必要。既存ハーネス
`scripts/trial/run_quadsdk_gap_gbpl.sh` のヘッダに「main の twist 用クロール設定
（period 0.9 / horizon 40）のままだと GBP-L の body plan を local NMPC が追従できず
初手で横倒れする」と記録があり、素のトロット歩容へ一時パッチして実験している。
ジャンプの追従はさらに厳しいと見込まれ、Stage 2〜6 は反復チューニングを要する。

---

## 10. 成功・失敗の判定根拠

現時点で「計測値で確認できたジャンプ」は**まだ無い**。
Stage 1（接触スケジュール）のみ、ユニットテストで正しさを確認済み。
`primitive_id == FLIGHT` があるだけを成功としないという禁止事項を守り、
Stage 4/6 では実測接触が 30 ms 以上ゼロ・CoM が穴の反対側・着地後 2 s 転倒なし、
を CSV で確認してから成功と書く。

---

## 11. GIF と CSV の場所

- Stage 1：`ros2_ws/build/{local_planner,global_body_planner}` の gtest（アーティファクト無し、テストログのみ）。
- Stage 2 以降：未生成。生成時は `artifacts/step17/` 以下に `state_log.csv` と GIF を置く。

---

## 12. 残課題

1. **NMPC**：`REAR_PUSH` 区間で後脚 `u_ub` z を引き上げ、名目 GRF を後脚偏重に。μ をジャンプ用に上げる。
2. **CSV ロガー拡張**：現行 `quadsdk_step01_baseline.py` は body pose/vel、脚別 contact（計画）、脚別 GRF、NMPC 診断のみ。
   課題 §10 が要求する足先位置・関節 pos/vel/torque・`primitive_id`・`jump_phase`・**実測接触**を追加する経路が要る。
3. **実測接触**：MuJoCo 接触または `body_force_estimator` を購読し、着地検出（SETTLE 遷移）と CSV へ。
4. **Stage 2〜6 の run スクリプト**：`run_quadsdk_gap_gbpl.sh` を基に、平地世界＋`jump_mode:=force_leap`＋
   ステージ別パラメータ（REAR_PUSH 時間、目標飛距離）で走らせ、CSV → 判定 → GIF。
5. **GBP 力モデル（問題 C）**：現状は点質量のまま。後脚接地点まわりのピッチ発散が出る場合、
   `Action` へ後脚別 GRF ＋ 接地点 ＋ ピッチ拘束を追加（大改修、Stage 3 で必要性を判断）。
6. GBPL → NMPC 追従の安定化（歩容・horizon・重み）。

---

## 13. 実機投入前に必要な安全確認（未到達）

シミュレーションで Stage 6 まで通ってから記載する。最低限：関節トルク・速度の実測が
Go2 データシート上限内、着地衝撃、ロール・ピッチの最大、複数回試行の再現性、
プラン失敗時のフォールバック（STAND 復帰）。

---

## 14. 事実 / 計測 / 推測の分離

**事実（コード・テストで確認）**
- commit `6fef3f4` / `a300535` の変更内容は本文どおり。
- `colcon test`：112 tests, 0 failures（quad_utils / global_body_planner / local_planner、2026-09-03）。
- 既存の trot / STAND / flight 接触スケジュールのテストは不変で通る。
- `jump_mode` 既定 `auto` は上流挙動と一致（リープを is_jump 扱いしない）。

**計測**
- なし（Stage 2 以降未実施）。

**推測（未検証）**
- §6 のインパルス値と「μ を上げ後脚上限を上げれば 30 cm を飛べる」。
- GBPL → NMPC 追従がジャンプで発散するか否か。
- ピッチ発散が点質量モデルのままで許容できるか。
