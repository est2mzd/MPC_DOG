# Step 10: 現在歩容から未来の脚順序を再構成(shadow mode)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
指示書 §5 Step 10。**制御は変更しない**(計装のみ)。

## 1. 背景

Step 09 で「50 cm の穴が落ちる因果」を数値で確定した。指示書のゴール(Step 14:
危険地点の M 歩手前で止まる)には、その前に「未来の脚順序 → 到達可能な足場候補 →
複数歩足場列 → 停止余裕」の連鎖が要る。Step 10 はその最初の 1 段:
**いまの歩容位相から、この先どの脚がどの順で着地するかを再構成する**。

## 2. 目的

- 現在の gait phase から未来 touchdown event 列(脚・ホライズン index・予測時刻)を
  出す計装を追加(既定 OFF、env `MPCDOG_STEPDUMP_DIR`)。
- 予測 touchdown と実接触遷移(`state_log.csv` の `contact_*` 立ち上がり)の
  **脚順一致**と**タイミング誤差**を、平地 / 30 cm 穴 / 連続 15 cm 穴で確認。

## 3. 変更前のコード経路(調査)

| 項目 | 事実 |
|---|---|
| 位相 | `computeContactSchedule()`([local_footstep_planner.cpp:181](../../external/quad-sdk/local_planner/src/local_footstep_planner.cpp#L181)):`phase = current_plan_index % period_`。 |
| 未来接触表 | `contact_schedule[i] = nominal_contact_schedule_[(i + phase) % period_]`(horizon 分タイル展開)。**位相 0 からではなく現在位相から**始めている。 |
| `nominal_contact_schedule_` | `setTemporalParams()` で `duty_cycles_` / `phase_offsets_` から 1 周期分を構築。 |
| touchdown event | `isNewContact(contact_schedule, i, j)` = `contact[i][j] && !contact[i-1][j]`。 |
| 実接触 | `state_log.csv` の `contact_FL/BL/FR/BR`(`True`/`False`)。立ち上がり = 実 touchdown。 |
| 脚順(コード) | `num_feet_` 順 = FL, BL, FR, BR(`kS09Legs`)。`state_log` の列順も同じ。 |

→ 「現在位相からの未来 touchdown 列」は `computeContactSchedule` の出力そのもの。
Step 10 の計装はその出力から event 列を**取り出して記録するだけ**(再導出しない)。

## 4. 事実 / 推測 / 未確認

- 事実:§3、§7 の CSV / 図。
- 推測:なし(位相・脚順はコードから直読)。
- 未確認:`node_->now()`(ROS クロック)と `state_log` の `sim_time_s` は原点が違う
  ため、絶対時刻の比較はしない。**脚順**と**touchdown 間隔**で誤差を見る(原点非依存)。

## 5. 変更計画

| ファイル | 追加内容 | 既定 OFF の担保 |
|---|---|---|
| `local_footstep_planner.cpp` | 計装ゲートを `step09Dir()` → `stepDumpDir()`(`MPCDOG_STEPDUMP_DIR` / 旧 `MPCDOG_STEP09_DIR`)。`computeFootPlan` に、毎周期・脚ごとに `isNewContact` の先頭数件を `step10_gait_events.csv` へ書く | env 未設定なら 1 行も動かない。戻り値・`foot_positions`・`cmd_vel` 不変。`local_planner.cpp` 無変更 |
| `scripts/trial/step10_measure.sh` | 平地 / 30 cm / 連続 15 cm を実行し CSV 収集 | yaml 無変更 |
| `scripts/trial/step10_analyze.py` | 脚順一致 + touchdown 間隔誤差 + 予測 vs 実の図 | — |

## 6. 変更ファイルと変更理由

- 計装のみ(env ガード)。予測 touchdown は `computeContactSchedule` 内部の
  `contact_schedule` からしか取り出せず、publish されていない。
- `stepDumpDir()` へ改名:Step 10 以降も同じ env で複数ファイルを吐くため。

## 7. 入出力・単位・座標系

`step10_gait_events.csv`(毎周期・脚ごと・先頭 4 event):
`time[s](ROS クロック), current_plan_index, phase(=index % period), period, dt[s],
leg(FL/BL/FR/BR), event_ordinal(0=次の touchdown), pred_touchdown_horizon_index,
pred_touchdown_time[s](=(current_plan_index + i)·dt)`

## 8. 実験条件

- feature スイッチは出荷既定。0.3 m/s、クロール歩容。
- 平地 `flat_wide`(25 s)、30 cm 穴 `flat_gaps_2m`(30 s)、連続 15 cm
  `flat_repgap_s15g15n3`(30 s)。

## 9. 試行結果

`scripts/trial/step10_analyze.py` の出力(予測 = 現在位相からの `computeContactSchedule`
出力、実 = `state_log.csv` の `contact_*` 立ち上がり、STAND 整定は除外):

| 地形 | 予測 脚順(1 周期) | 実 脚順 | 一致 | 予測 touchdown 間隔 | 実間隔 | 誤差 |
|---|---|---|---|---|---|---|
| 平地(`flat_wide`) | `FL→BR→FR→BL` | `FL→BR→FR→BL` | **YES** | 225 ms | 222 ms | 3 ms |
| 30 cm 穴(`flat_gaps_2m`) | `FL→BR→FR→BL` | `FL→BR→FR→BL` | **YES** | 225 ms | 225 ms | 0 ms |
| 連続 15 cm(`flat_repgap_s15g15n3`) | `FL→BR→FR→BL` | `FL→BR→FR→BL` | **YES** | 225 ms | 225 ms | 0 ms |

- **脚順序は 3 地形すべてで一致**(巡回順として。クロール歩容は 1 脚ずつ
  `FL→BR→FR→BL` の順、period 30 index × dt 0.03 s = 0.9 s / 4 脚 = 0.225 s 間隔)。
- 予測 touchdown 間隔と実接触間隔の差は **0〜3 ms**(= sim の 1 tick 未満)。
- 図:`artifacts/step10/<地形>/step10_<地形>_pred_vs_actual.png`(実接触の立ち上がりを
  脚別の色で並べたもの。予測 1 周期の脚順を下部に併記)。

**注意点(数値の読み方)**:
- 予測は毎周期 4 行(脚ごとに「その脚の次の touchdown」)出る。「次に着く脚」は
  `pred_touchdown_horizon_index` が最小の脚。1 周期の順 = 4 脚を index 昇順に並べたもの。
- `node_->now()`(ROS クロック)と `state_log` の `sim_time_s` は原点が違うので、
  **絶対時刻は比較せず**、脚順(巡回)と touchdown 間隔で照合した。
- 実データは STAND 整定(全脚同時接地)を含むため、`base_pos_x_m > 0.05` に
  なってから + 先頭数エッジを捨ててから照合。

## 10. 失敗原因

なし(3 地形とも脚順一致・間隔誤差 ≤3 ms)。

## 11. 後方互換性確認

- env 未設定で計装は 1 行も動かない。`local_planner.cpp` 無変更。gtest **41/41 green**。
- feature OFF 回帰は Step 09 で確認済み(instrumentation ビルドで step03 渡り切り /
  100 cm は既存どおり)。Step 10 の追加は同じ env ガード内で、制御パスに触れない。

## 12. GIF・CSV・ログ

- `artifacts/step10/{flat,g30,r15n3}/`:`step10_gait_events.csv` / `state_log.csv` /
  `run.log` / `step10_*_pred_vs_actual.png`
- README には「予測 touchdown ↔ 実接触」の図(最も分かりやすい 1 枚)。

## 13. 次 Step へ進む条件

- [x] 各脚の予測 touchdown と実接触遷移の誤差を記録できる(間隔誤差 ≤3 ms)。
- [x] 平地・30 cm 穴・連続 15 cm 穴で脚順序が一致する(`FL→BR→FR→BL`、3/3)。
- → Step 11(1 歩の可到達領域と安全足場候補生成、shadow)へ進む。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 10
- `agent_reports/steps/step_09_terrain_grid_and_foothold_measurement.md`
- `agent_reports/steps/step_09b_why_50cm_not_stopping.md`(なぜ Step 10〜14 が要るか)
