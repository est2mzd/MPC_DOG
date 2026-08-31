# 引き継ぎ資料:Quad-SDK 穴対応 Foot Placement 改善(2026-08-31 時点)

読者は制御の大学院初心者を想定。まず **背景・目的・結論** を書き、
そのあと「次に何をするか」「守るべきルール」「ファイルの場所」を続ける。

作成方法の注記:本資料は会話記憶だけでなく、`git log` / `git status` /
対象ファイルの実際の内容を再確認したうえで作成した。

---

## 背景

- MPC_DOG では四足ロボット **Go2** を **Quad-SDK**(C++ の四足制御スタック)で
  走らせ、MuJoCo 上で Step 単位に検証している。
- 以前の作業で、**深さ 1 m・幅 0.3 m の溝を、足を溝に入れずに複数本連続で
  渡る**ことに成功した(`reference:=twist` = `cmd_vel` 駆動 + クロール歩容)。
  証拠 GIF は git 追跡済み:
  - `artifacts/gifs/quadsdk_step03_1m_v0p15_12to35s.gif`(溝間隔 2.0 m、0.15 m/s)
  - `artifacts/gifs/quadsdk_step04_1m_v0p3_15to40s.gif`(溝間隔 1.5 m、0.3 m/s)
- ただしこの成功は「**静的で位置ずれの無い既知の地形** + **手作業の安全
  マージン(±0.05 m)**」に依存しており、実センサ・実機で安全とは言えない。
- そこで指示書
  `chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md`
  に従い、**まずコードを精読して「足の置き場所を決める処理(Foot Placement)と
  MPC の連携」を数式とコードで正確に把握し、そのうえで不足している安全機能を
  1 コミット = 1 目的で段階的に足す**方針で進めている。

## 目的

1. **理解**:センサ → 足場計画 → NMPC → 逆動力学 → トルク の各段が、
   何を入力に何を出力し、穴に対して誰が責任を持つのかを、推測でなく
   コードの関数・行から確定する。→ **完了**。
2. **訂正**:以前の作業メモのコードと食い違う記述を直す。→ **完了(Phase 0)**。
3. **改善**:「有効な足場が無い(`status != FootholdStatus::VALID`)のに
   名目の足場で歩き続ける」等の危険な挙動を、既存の成功挙動を壊さずに、
   段階的に潰す。各フェーズは着手前に変更計画を出し、ユーザー確認を取る。
   → **Phase 1 完了。Phase 2A は変更計画提示済み・未実装(確認待ち)**。

## 結論(現時点)

- **解析は完了。** 最重要の確定事項:
  **Go2 の NMPC は足場を最適化していない。** 足の置き場所は Foot Placement
  (`local_footstep_planner`)が地形マップから決め、NMPC はその足場を
  **固定パラメータ**として胴体軌道と地面反力(GRF)だけを最適化する。
  NMPC の制約は **運動方程式 + 摩擦ピラミッドのみ**で、「脚が届くか(逆運動学)」
  「縁からどれだけ離れているか」の制約は **無い**。
- **Phase 0 完了**(ドキュメント訂正のみ、コード不変)。
- **Phase 1 完了**(コード変更あり、ただしロボットの挙動は不変)。
  足場選択関数を「位置だけ返す」→「**成功/失敗の種類 + 診断値も返す**」型へ
  拡張した。返す位置は従来と 1 バイトも変わらない。
  `local_planner` の **全 29 テストが green**。
- **Phase 2A は変更計画(表)を提示済み・未実装。** 実装前に、ユーザーへの
  3 つの確認事項(下記「Phase 2A をブロックしている確認事項」)に回答が必要。
- **Phase 2B〜6 は未着手。**

---

## 次のチャットが最初にやること(1 ステップ)

**Phase 2A の実装に着手する前に、下記 3 つの確認事項をユーザーに提示して
回答を得る。** 回答が出るまでコードは変更しない(指示書の禁止事項)。
回答が得られたら、`agent_reports/quadsdk_gap_foothold_phase_progress.md`
「次にやること:Phase 2A の実装」の変更計画表(2A-1〜2A-5)に沿って実装し、
`colcon test --packages-select local_planner` が 29/29 green のままであること、
および 0.15/0.3/0.5 m/s の既存成功走行が引き続き溝を渡れることを確認する。

### Phase 2A をブロックしている確認事項

1. `computeFootPlan()` の戻り値を **`bool`** にするか、
   **`struct FootPlanResult { bool ok; FootholdStatus worst_status;
   int failed_leg; int failed_touchdown_index; int failed_count; }`** にするか。
2. Phase 2A の挙動を **常時 ON** にするか、
   **`stop_on_invalid_foothold` パラメータ(既定 ON)** で切れるようにするか。
3. 失敗の記録は **最初の 1 件だけ**でよいか、**全件カウント**するか。

### Phase 2A の変更計画(提示済み・未実装、再掲)

| # | 変更 | 内容 |
|---|---|---|
| 2A-1 | `computeFootPlan()` の戻り値 | `void` → 上記の戻り型。touchdown ループで `status != VALID` を集計、最初の失敗を記録 |
| 2A-2 | `computeFootPlan()` の地図外 `continue`(`local_footstep_planner.cpp:255-261` 付近) | 裸の `continue` の前に `NOMINAL_OUTSIDE_MAP` + leg/index を記録 |
| 2A-3 | `computeFootPlan()` の foothold 書き込み(`:276` ほか) | `status != VALID` のとき穴上の名目/NaN 高さを書かず、直前 touchdown 値を踏襲(非 touchdown 分岐と同じ) |
| 2A-4 | `computeLocalPlan()`(`local_planner.cpp:527-560` 付近) | 戻り値の `ok == false` なら **NMPC を呼ばず `return false`** → `spin()` が `publishLocalPlan()` を呼ばない |
| 2A-5 | 検証 | 単体(穴地形で `ok==false`、失敗 touchdown 行が穴 nominal でない、`computeLocalPlan` が false)+ 無効 plan 非 publish テスト + 回帰(0.15/0.3/0.5 m/s の既存成功走行) |

Phase 2A では STAND 遷移・`cmd_vel`→0・Map 期限切れ・edge clearance・IK 判定は
**入れない**(それぞれ Phase 2B / 3 / 4)。停止自体は既存の
「local plan が 0.1 s 以上古い → `robot_driver` が起立姿勢へ PD ホールド」に委ねる。

---

## git 状態と主要コミット

- ブランチ **`main`**、HEAD **`35f01fc`**、すべて push 済み、working tree クリーン。
- `git config merge.ff false`(このリポジトリのローカル設定)。今後のマージは
  必ずマージコミットを作る。

| コミット | 内容 |
|---|---|
| `c9cf853` | 解析レポートにレビュー指摘 7 点を反映(コードなし) |
| `6e089e1` | **Phase 0**:資料の 3 誤りを訂正(ドキュメントのみ) |
| `484ea13` | **Phase 1**:`FootholdResult` / `FootholdStatus`(コード、挙動不変) |
| `88605aa` | ドキュメント語の統一(`found==false` → `status != FootholdStatus::VALID`) |
| `6282643` | 先行バグ修正:`test_local_planner.cpp` の `N_` 期待値 26 → 40(テストのみ) |
| `45720a5` `ea14d7b` `0fe6b3d` | フェーズ実施ログの作成・書き直し・更新 |
| `f07b5d3` `305b2a5` `a666291` `908583a` | 溝渡り証拠 GIF を追跡 + README 埋め込み + `.gitignore` の `!` 否定修正 |
| `7d63142` | 解析の指示書ファイルを追跡 |
| `35f01fc` | Quad-SDK 元コード変更まとめ(ユーザー執筆)+ README リンク(現 HEAD) |

---

## フェーズ表

| Phase | 目的 | 状態 |
|---|---|---|
| 解析 | 資料 ⇔ コード照合、terrain map / foot placement / NMPC を数式とコードで整理 | ✅ |
| 0 | 解析で判明した資料の 3 誤りを訂正。コード変更なし | ✅ |
| 1 | 足場選択器を「位置だけ」→「成功/失敗 + 診断値」を返す型へ。挙動不変 | ✅ |
| 2A | NMPC へ無効足場(穴上・地図外・高さ非有限)を渡さない | ⬜ 変更計画提示済み・未実装(確認 3 件待ち) |
| 2B | 遊脚を考慮した安全な減速・停止シーケンスを設計(状態遷移表から) | ⬜ 2A 完了後 |
| 3 | 穴縁からの安全距離を地図上で明示判定(`EDGE_TOO_CLOSE` 追加) | ⬜ |
| 4 | 逆運動学の可到達性で候補を絞る(`IK_UNREACHABLE` 追加) | ⬜ |
| 5 | 大きな足場補正時の減速/刻み歩行 | ⬜ |
| 6 | 地図の鮮度・未観測セルの扱い(`MAP_STALE` 追加) | ⬜ |

`FootholdStatus` は Phase 1 時点で **4 値のみ**
(`VALID` / `NOMINAL_OUTSIDE_MAP` / `NO_TRAVERSABLE_CANDIDATE` / `NONFINITE_HEIGHT`)。
`EDGE_TOO_CLOSE` / `IK_UNREACHABLE` / `MAP_STALE` は、それを計算するコードと
**同じ Phase で**追加する(原因と効果を追いやすくするため)。

---

## 守るべきルール(このタスク固有 + プロジェクト共通)

1. **1 コミット = 1 目的。** 制御コードの変更とテスト/ドキュメントの変更は
   別コミットに分ける。
2. **各 Phase は着手前に変更計画(表)を提示し、ユーザー確認を取ってから
   コードを変更する。** Phase 2 をまとめて一括実装しない。
3. **明示的な指示がないファイル変更をしない。**「変えたい」は「修正して」では
   ない(`memory/feedback_no_edit_without_explicit_instruction.md`)。
4. **作業を実行したら必ず .md を作成/更新し、README にリンクを貼る。**
   .md は冒頭に **背景 → 目的 → 結論**、読者は大学院初心者
   (`memory/feedback_write_md_and_readme_link_after_executing.md`)。
   多フェーズ作業は 1 本の running doc
   (`agent_reports/quadsdk_gap_foothold_phase_progress.md`)を更新し続ける。
5. **エージェント作成の .md は `agent_reports/` 配下**(`docs/` 不可、
   `memory/feedback_agent_md_goes_in_agent_reports.md`)。README はリンクのみ。
6. **CoinHSL 導入 / MA27 への変更 / MPC ゲイン調整は、明確な必要性が確認される
   まで行わない**(調査初期からのユーザー制約。現状 MUMPS のまま)。
7. `.gitignore` は **行末インラインコメント非対応**。コメントは必ず行頭 `#`。
8. push・main へのマージはユーザーが都度指示する。勝手に push しない。

---

## 主要ファイルの場所

### 解析・ログ(まず読む)

1. `agent_reports/quadsdk_gap_foothold_phase_progress.md` — **フェーズ実施ログ
   (running doc)。Phase 2A の変更計画表・各コミット詳細はここ。**
2. `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` — 解析本体
   (資料 ⇔ コード照合表・terrain map の式・足場計画 I/O・NMPC 受け渡し・
   §6.1 に Phase 1 と同時に足すべき診断ログの一覧・§8 フェーズ表)。
3. `chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md` —
   指示書(禁止事項・変更計画テンプレート・Phase 分割の根拠)。
4. `agent_reports/quadsdk_original_code_tuning_summary.md` — Quad-SDK 元コードから
   の全変更まとめ(ユーザー執筆)。
5. `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` — twist + クロール
   成功記録(Phase 0 で §3.4/§2 訂正済み)。
6. `agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md` — gbpl 実験 + センシング→
   foot plan→MPC→WBC→トルクの工程別ボトルネック分析。

### Phase 1 で触ったコード(`external/quad-sdk/local_planner/`)

- `include/local_planner/local_footstep_planner.hpp` — `enum class FootholdStatus`
  + `struct FootholdResult` + `getNearestValidFootholdResult()` 宣言。
- `src/local_footstep_planner.cpp` — `getNearestValidFoothold()` は
  `getNearestValidFootholdResult(...).position` を返す薄いラッパ。探索本体は
  新関数へ移動、status を設定。DIAG ログに `found/status/snap` 追記。
  Phase 2A で触るのは `computeFootPlan()`(地図外 `continue` ≈ `:255-261`、
  foothold 書き込み ≈ `:276`)。
- `src/local_planner.cpp` — `computeLocalPlan()`(≈ `:527-560`)が Phase 2A-4 の的。
- `test/test_footstep_planner.cpp` — 合成地形ヘルパ 2 個 + Phase 1 テスト 5 本。
- `test/test_local_planner.cpp:175` — `N_` 期待値 40(`6282643` で 26 から修正)。

### 現在の設定(`main`、`external/quad-sdk/`)

- `quad_utils/config/go2.yaml`:`period: 0.9` / `duty_cycles: [0.75]×4` /
  `phase_offsets: [0.0, 0.75, 0.5, 0.25]`(横列クロール)/
  `foothold_search_radius: 0.7` / `ground_clearance: 0.1`。
- `local_planner/config/local_planner.yaml`:`horizon_length: 40`。
- `nmpc_controller`:`linear_solver: mumps`、実効摩擦係数 μ = 0.6
  (`go2.yaml` が `nmpc_controller.yaml` の 0.3 を launch 順で上書き)。
- 歩容は地形で**自動変更されない**(起動時 1 回読む固定表)。

### 実行ハーネス

- `scripts/trial/run_quadsdk_gap_gbpl.sh` — goal 駆動 GBP-L 版
  (`GOAL_X` ほか env で歩容/ホライズン/計画時間を一時パッチ、`trap` で復元)。
- twist + クロールの溝渡り実行手順は
  `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` 参照。

---

## 未解決の確認事項(コードとは別)

- **実効摩擦係数 μ = 0.6 のライブ確認**:
  `ros2 param get /robot_1/local_planner nmpc_controller.friction_coefficient`
  (または該当ノード)で 0.6 を確認する。解析は launch 順からの推定。
- **`agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` §6.1** に、Phase 1
  と同時に足すべき診断ログ(cost 内訳 / slack / 制約違反 / IPOPT status)の
  一覧がある。まだ未実装。「遠い足場 → NMPC cost 増大 → 非収束」の因果は、
  このログが取れるまで **推測**扱い。
- `NONFINITE_HEIGHT` の診断値の不正確さ:この status のとき `position` は
  best.xy だが `snap_distance` は 0.0(未計算)。`snap_distance` /
  `traversability_selected` が意味を持つのは `VALID` のときのみ。
- `feature/apply_global_planner` ブランチは `--no-ff` で main へマージ済み
  (`ae104fd`、調査記録として)。GBP-L は穴 1 本の leap は約 1/2 で成功、
  連続区間は未達。

## 関連

- `agent_reports/handoff/quadsdk_step01_handoff.md` — Quad-SDK Step 01(前進歩行)
  の旧引き継ぎ。本タスク(穴対応 Foot Placement)とは別系統。
