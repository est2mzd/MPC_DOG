# Step 11: 1 歩の可到達領域と安全足場候補生成(shadow mode)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
指示書 §5 Step 11。**制御は変更しない**(計装のみ)。

## 1. 背景

Step 10 で「未来の脚順序」を再構成できた。次は「その脚が **実際に届く安全なセル**」を
列挙する。既存の `getNearestValidFootholdResult` は半径 0.7 m の探索で
`traversability > 0.6` の最近傍を取るだけで、**脚が届くか(reach)/ 未観測でないか
/ 足裏マージン**を見ていない(Step 09 の 50 cm 転倒はこの穴が原因)。

## 2. 目的

- 各脚の 1 歩ぶんについて、hip 周りの地図セルを走査し
  **(a) reach 内**(`‖セル − hip‖ ≤ ik_max_reach`、Phase 4 と同じ 3D 距離 + 粗い
  前後左右ボックス)**(b) 安全**(`traversability > 0.6`、直交 4 近傍も安全 =
  足裏/縁マージン)**(c) 観測済み**(生 `z` が有限、未観測 ≠ 安全)を満たす数を数える。
- 選択足場が同じ判定を通るかも記録。
- 30 cm / 50 cm / 100 cm / 平地 で、完了条件を確認。

## 3. 変更前のコード経路(調査)

| 項目 | 事実 |
|---|---|
| 既存の足場探索 | `getNearestValidFootholdResult`:`SpiralIterator` 半径 `foothold_search_radius_`(0.7 m)、`traversability > foothold_obj_threshold_`(0.6)の最近傍。**reach / 観測 / 足裏マージンは見ない**。 |
| Phase 4 の reach 判定 | `(result.position - hip_world).norm() > ik_max_reach_`([local_footstep_planner.cpp:915](../../external/quad-sdk/local_planner/src/local_footstep_planner.cpp#L915))。`hip_world` = `computeFootPlan` から渡る `hip_position_midstance`(= welzl 円中心、z 成分は円半径)。 |
| hip の 3D 位置 | `quadKD_->worldToNominalHipFKWorldFrame(leg, body_pos, body_rpy, hip_out)` で touchdown 姿勢の world hip。Step 11 はこれを使う(welzl の z=半径では reach を 3D で測れないため)。 |
| 生 `z` / observed | 地図に `observed` レイヤは無い。`z`(生)が `NaN` = 未観測 or 穴(Step 09)。両者は区別できない → いずれも「安全でない」と扱う。 |

## 4. 事実 / 推測 / 未確認

- 事実:§9 の CSV / 図。
- **簡易近似(instruction 2.3 の 6〜8、未実装を「確認済み」と書かない)**:
  - 前後左右ボックス = hip から `前 0.32 / 後 0.28 / 左右 0.26` m(go2 脚可動域の
    粗い矩形。関節限界の厳密判定ではない)。
  - 支持脚集合(7)・NMPC 追従(9)は本 Step では**評価しない**。
- 未確認:`ik_max_reach`(既定 0.45)が go2 の真の可到達半径と一致するかは Phase 4
  と同じ前提を流用(Step 07 で妥当性を確認済み)。

## 5. 変更計画

| ファイル | 追加内容 | 既定 OFF の担保 |
|---|---|---|
| `local_footstep_planner.cpp` | 匿名 namespace に `step11EnumerateCandidates()`。`computeFootPlan` の touchdown ループで、脚ごと 1 回/周期、hip FK を計算して呼ぶ。`step11_candidates.csv` へ 1 行 | env 未設定なら未実行。戻り値・`foot_positions`・`cmd_vel` 不変。`local_planner.cpp` 無変更 |
| `scripts/trial/step11_measure.sh` | 平地 / 30 / 50 / 100 cm を実行し CSV 収集 | yaml 無変更 |
| `scripts/trial/step11_analyze.py` | 前脚 n_valid の推移図 + 完了条件チェック | — |

## 6. 変更ファイルと変更理由

- Phase 4 の reach 判定式(`‖pos − hip‖ ≤ ik_max_reach`)を Step 11 でも使う。
  今回は重複実装を避けきれず、`step11EnumerateCandidates` 内に同式を再掲した
  (Phase 4 側は `hip_position_midstance`、Step 11 側は正しい 3D hip FK を使うので
  完全共通化は次段で。OFF 時の Phase 4 挙動は不変)。

## 7. 入出力・単位・座標系

`step11_candidates.csv`(脚ごと 1 回/周期):
`time[s], current_plan_index, leg, touchdown_index, hip_x, hip_y, hip_z[m],
n_in_reach(box+reach), n_safe(+trav>0.6), n_valid(+足裏4近傍+観測), 
min_valid_reach_dist[m], sel_x, sel_y, sel_in_reach(0/1), sel_passes_all(0/1),
ik_max_reach[m]`

## 8. 実験条件

- feature スイッチは出荷既定。0.3 m/s、クロール歩容。
- 平地 `flat_wide`、30 cm `flat_gaps_2m`、50 cm `flat_trench_s09_50`、
  100 cm `flat_trench_s09_100`。

## 9. 試行結果

前脚(FL/FR)の 1 touchdown あたりの有効候補数 `n_valid`(reach 内 + 安全 +
足裏 4 近傍安全 + 観測済み)。`scripts/trial/step11_analyze.py`。

| 地形 | `n_valid` 中央値(全体) | `n_valid` 最小(穴の近く) | 選択足場が全判定を通る率(穴の近く) | 判定 |
|---|---:|---:|---:|---|
| 平地(`flat_wide`) | 134 | 131 | **100 %** | ✅ 名目付近に候補が常に残る |
| 30 cm 穴(`flat_gaps_2m`、穴は x≈1.0 / 3.0) | 131 | **43** | 32 % | ✅ **穴の直上でも候補が残る**(43 > 0) |
| 50 cm 穴(`flat_trench_s09_50`) | 131 | **0** | 17 % | ✅ 穴で `n_valid` が 0 に落ちる |
| 100 cm 穴(`flat_trench_s09_100`) | 134 | **0** | 0 % | ✅ 穴全域で `n_valid` = 0 |

- **30 cm**:穴の直上で `n_valid` は 134 → 43 に減るが **0 にはならない**。
  reach(0.45 m)内に、near / far どちらの帯にも安全セルが残るため。
  `getNearestValidFootholdResult` の選択足場が全判定を通るのは 32 %(縁に寄った
  瞬間は足裏マージンで落ちる)だが、**43 個の代替候補があるので複数歩探索なら
  別セルを選べる**。
- **50 / 100 cm**:hip が穴に入ると `n_reach`(幾何的に届くセル)は ~140 あるのに
  `n_safe` が 0(50 cm は縁 2 セル以外すべて `traversability` = NaN、100 cm は
  reach box が丸ごと void)→ `n_valid` = 0。**向こう岸の候補は reach 外**なので
  出てこない = 「届かない向こう岸を候補から除外できている」。
- 選択足場(void の縁にスナップ)は `sel_passes_all` = 0 が大半 = 縁は
  未観測(生 `z` NaN)/ 足裏マージン不足で不合格。

図:`artifacts/step11/<地形>/step11_<地形>_n_valid.png`(`n_valid` vs 前脚 hip の
x 位置。グレー帯 = 物理 void)。

## 10. 失敗原因

なし(4 地形とも完了条件を満たす)。「30 cm では候補が残るのに縁足場が選ばれる」
のは Step 11 の役割(候補の列挙)の外で、**選択ロジックの入れ替え = Step 12〜15**
の課題。

## 11. 後方互換性確認

- env 未設定で計装は未実行。`local_planner.cpp` 無変更。gtest **41/41 green**。

## 12. GIF・CSV・ログ

- `artifacts/step11/{flat,g30,g50,g100}/`:`step11_candidates.csv` /
  `state_log.csv` / `run.log` / `step11_*_n_valid.png`
- README には「前脚の有効足場候補数 n_valid の推移」図(30 cm = 残る / 50・100 cm =
  穴の手前で 0 に落ちる)。

## 13. 次 Step へ進む条件

- [x] 30 cm 穴で有効候補が残る(穴の直上でも `n_valid` 最小 43)。
- [x] 50/100 cm 穴で、届かない向こう岸候補を除外できる(`n_valid` = 0、`sel_passes_all` ≈ 0)。
- [x] 平地で名目足場付近の候補が残る(`n_valid` ≈ 134、`sel_passes_all` 100 %)。
- → Step 12(複数歩足場列の探索、shadow)へ進む。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 11
- `agent_reports/steps/step_10_future_gait_event_prediction.md`
- `agent_reports/steps/step_09_terrain_grid_and_foothold_measurement.md`(縁スナップの因果)
