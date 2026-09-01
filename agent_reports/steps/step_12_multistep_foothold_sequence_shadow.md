# Step 12: 複数歩足場列の探索(shadow mode)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
指示書 §5 Step 12。**制御は変更しない**(計装のみ)。

## 1. 背景

Step 10(未来脚順序)+ Step 11(1 歩の到達可能・安全候補)が揃った。Step 12 は
その 2 つを鎖にして、**未来の脚順序に沿って複数歩ぶんの足場列が成立するか**を判定する。
これが「危険地点の何歩手前で止まるか」(Step 13/14)の入力になる。

## 2. 目的

- 前方 `terrain_planning_distance`(= 2.5 m)ぶん、クロール順(`FL→BR→FR→BL`)で
  1 歩ずつ足場を置いてみて、判定を出す:
  - `FEASIBLE_TO_RANGE` … 距離末端まで(または step 上限 24 まで)足場を置けた
  - `BLOCKED_AT_STEP_K` … k 歩目で有効候補が 0
  - `UNKNOWN_BEFORE_RANGE` … 先に地図の外へ出た
- `SEARCH_TIMEOUT` は今回の貪欲探索(幅 1)では発生しない(step 上限で必ず終わる)。
  発生し得る実装にしたら FEASIBLE 扱いしないこと(指示書 §5)。
- 計算時間を記録し、local planner 周期(≈33 ms @ 30 Hz)に収まるか確認。

## 3. 変更前のコード経路 / 使うもの

| 使うもの | 出所 |
|---|---|
| 脚順序 `FL→BR→FR→BL` | Step 10(`computeContactSchedule` の実測順) |
| touchdown 間隔 `period·dt/4` ≈ 0.225 s | `setTemporalParams` の `period_` / `dt_` |
| hip の 3D 位置 | `quadKD_->worldToNominalHipFKWorldFrame(leg, body_pos, body_rpy, out)` |
| 候補判定(reach + 安全 + 足裏 + 観測) | Step 11 と同式(`step12PlanSequence` 内に再掲) |
| 前進速度 | `body_plan` の先頭〜末尾 x 差 / 経過時間(0〜1.0 m/s にクランプ) |
| 胴体の前方投影 | `body_x(k) = body_x(0) + v · k·0.225` |

## 4. 事実 / 推測 / 未確認

- 事実:§9 の CSV / 図。
- **簡易版であることの明示**(指示書 §2.3 の 7・9 は近似):
  - 探索は **幅 1 の貪欲**(各 touchdown で min-cost セルを 1 個選ぶ)。beam 幅を
    広げる余地あり。
  - cost = `|x−名目| + |y−hip_y| + 0.5·|y−前足場_y| + 0.3·(reach 比)`(初期式)。
  - 支持脚集合(7)・NMPC 追従(9)は評価しない。
  - 胴体は等速直進で投影(旋回・加減速なし)。
- 未確認:`terrain_planning_distance` = 2.5 m は初期値候補(指示書 §1.3)。

## 5. 変更計画

| ファイル | 追加内容 | 既定 OFF の担保 |
|---|---|---|
| `local_footstep_planner.cpp` | 匿名 namespace に `step12PlanSequence()`。`computeFootPlan` から 5 サイクルに 1 回呼ぶ。`step12_sequence.csv`(毎回)+ `step12_footholds.csv`(1/20 サンプル) | env 未設定なら未実行。戻り値・`foot_positions`・`cmd_vel` 不変。`local_planner.cpp` 無変更 |
| `scripts/trial/step12_measure.sh` | 平地 / 15 cm 連続 / 30 cm / 50 cm / 100 cm | yaml 無変更 |
| `scripts/trial/step12_analyze.py` | 判定の推移 + 予定足場列の図、計算時間集計 | — |

## 6. 変更ファイルと変更理由

- Step 11 の候補判定式を Step 12 でも使う(`step12PlanSequence` 内に再掲。共通化は
  制御接続の Step 15 前にまとめて実施)。

## 7. 入出力・単位・座標系

`step12_sequence.csv`(5 サイクルに 1 行):
`time[s], current_plan_index, verdict, blocked_step_k, blocked_leg, n_placed,
max_feasible_progress_m, plan_distance_m, compute_time_us`

`step12_footholds.csv`(1/20 サンプル、予定足場列):
`time[s], current_plan_index, step_k, leg, x, y, hip_x, n_valid`

## 8. 実験条件

- feature スイッチは出荷既定。0.3 m/s、クロール歩容。
- 平地 `flat_wide`、連続 15 cm `flat_repgap_s15g15n3`、30 cm `flat_gaps_2m`、
  50 cm `flat_trench_s09_50`、100 cm `flat_trench_s09_100`。

## 9. 試行結果

`scripts/trial/step12_analyze.py`。「歩行中の意味ある判定」= FEASIBLE / k>0 の
BLOCKED / UNKNOWN(k=0 の BLOCKED は STAND か転倒後の姿勢由来なので除外)。

| 地形 | 判定の内訳 | BLOCKED の k 範囲 | 計算時間 中央値 / 最大 | 完了条件 |
|---|---|---|---:|---|
| 平地 `flat_wide` | FEASIBLE 100 % | – | 0.61 ms / 3.4 ms | ✅ 視野末端まで成立 |
| 連続 15 cm `flat_repgap_s15g15n3` | FEASIBLE 87 %、UNKNOWN 12 % | – (BLOCKED 0) | 0.57 ms / 3.3 ms | ✅ 既存成功範囲で成立 |
| 30 cm 単独 `flat_trench_s09_30` | FEASIBLE 72 %、BLOCKED 14 %、UNKNOWN 13 % | 4〜31 | 0.57 ms / 2.6 ms | ✅(大半 FEASIBLE) |
| 30 cm `flat_gaps_2m`(MESH_MARGIN 0.10) | **BLOCKED 60 %**、FEASIBLE 39 % | 4〜31 | 0.47 ms / 1.9 ms | ⚠️ **保守的**(下記) |
| 50 cm `flat_trench_s09_50` | FEASIBLE 52 %、**BLOCKED 47 %** | 4〜30 | 0.55 ms / 3.7 ms | ✅ 成立しない |
| 100 cm `flat_trench_s09_100` | FEASIBLE 52 %、**BLOCKED 47 %** | 2〜30 | 0.52 ms / 1.8 ms | ✅ 成立しない |

- **50 / 100 cm**:胴体が穴に近づくにつれ、判定が FEASIBLE → BLOCKED に移り、
  `blocked_step_k` が小さくなっていく(遠いうちは 32 歩先まで届かない → 近づくと
  k=30→…→2 で穴を検知)。g50 の予定足場列(図)は穴の近縁 x=2.0 まで置いて
  そこで止まる。
- **計算時間**:中央値 0.5 ms、最大 3.7 ms。local planner 周期(≈33 ms @ 30 Hz)
  に**十分収まる** → 同一周期内で回してよい。
- `SEARCH_TIMEOUT` は幅 1 貪欲では発生せず(step 上限で必ず終わる)。

図(README 掲載):`artifacts/step12/g50/step12_g50.png`
(上=判定の推移 緑 FEASIBLE / 赤 BLOCKED、下=1 サイクル分の予定足場列)。

## 10. 失敗原因(=既知の保守性)

- **`flat_gaps_2m`(30 cm、MESH_MARGIN 0.10)で 60 % BLOCKED**:この world は
  `traversability` の危険帯が **0.40 m**(Step 09)。実機は向こう岸へ一歩(≈0.40 m)で
  跨いで渡るが、Step 12 のステップ長上限 `max_step_fwd`(0.45 m)+ 足裏マージン
  (進行方向 2 近傍)+ 離散化で、向こう岸縁セルが弾かれ「跨げない」と判定する。
  - 同じ 30 cm でも MESH_MARGIN 0.05 の `flat_trench_s09_30`(危険帯 0.30 m)は
    72 % FEASIBLE。**MESH_MARGIN の不一致(Step 09 の教訓)がここでも効いている。**
  - `max_step_fwd` は 0.30(15 cm・30 cm も落とす)→ 0.45(50 cm を捕まえつつ
    15 cm・単独 30 cm は通す)と 3 回調整した。0.40〜0.55 m の窓が狭く、
    world ごとの危険帯幅のばらつきと同オーダー。
- 対策(Step 13〜15 で):候補判定を「危険帯を生 `z` の NaN 幅で見る」(Step 09 の
  推奨)+ ステップ長を go2 実測の脚可動域に合わせる。今回は**近似のまま先へ**進む
  (50/100 cm の BLOCKED 信号は取れているため、Step 13 の入力には十分)。

## 11. 後方互換性確認

- env 未設定で計装は未実行。`local_planner.cpp` 無変更。gtest **41/41 green**。

## 12. GIF・CSV・ログ

- `artifacts/step12/{flat,r15,g30,g50,g100}/`:`step12_sequence.csv` /
  `step12_footholds.csv` / `state_log.csv` / `run.log` / `step12_*.png`
- README には「判定の推移(緑=FEASIBLE / 赤=BLOCKED)」+「1 サイクル分の予定足場列」。

## 13. 次 Step へ進む条件

- [x] 平地・15 cm 連続・30 cm 単独で `FEASIBLE_TO_RANGE`(flat_gaps_2m は保守的=§10)。
- [x] 50/100 cm で `BLOCKED_AT_STEP_K`(接近につれ k が減少)。
- [x] 計算時間 中央値 0.5 ms / 最大 3.7 ms → **周期内で可**。
- → Step 13(停止余裕 M 歩の推定)へ進む。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 12
- `agent_reports/steps/step_11_reachable_safe_foothold_candidates.md`
- `agent_reports/steps/step_10_future_gait_event_prediction.md`
