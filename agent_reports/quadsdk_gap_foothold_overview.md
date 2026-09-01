# 穴対応 Foot Placement 改善:全体の考え方(概観)

読者は制御の大学院初心者を想定。個別の詳細(コード解析・コミット単位のログ・
各 Step の実測)は末尾のリンク先にある。ここは **「何を・なぜ・どういう順で」
やっているかの地図**。

---

## 背景

- MPC_DOG では四足ロボット **Go2** を **Quad-SDK**(C++ の四足制御スタック)で
  走らせ、MuJoCo 上で検証している。`reference:=twist`(= `cmd_vel` で走る)。
- 以前の作業で、**深さ 1 m・幅 0.3 m の溝を、足を溝に入れずに複数本連続で渡る**
  ことに成功した(クロール歩容 + 地形メッシュの作り方の調整。C++ の挙動変更なし。
  → `steps/step_03_04_1m_quadsdk_gap_crossing.md`)。
- ただしそれは「**静的で位置ずれの無い既知の地形** + **手作業の安全マージン**」
  頼りで、**「有効な足場が無いのに名目の足場で歩き続ける」**ような危険な挙動が
  コード上に残っていた。実センサ・実機で安全とは言えない。

## 目的

**既存の「渡れる」挙動を壊さずに、足りない安全機能を 1 コミット = 1 目的で
段階的に足す。** 各フェーズは着手前に変更計画を出してユーザー確認を取る。

---

## 結論(現時点)

- **解析は完了**。最重要事実:**Go2 の NMPC は足場を最適化していない。**
  足の置き場所は Foot Placement(`local_footstep_planner`)が地形マップから
  決め、NMPC はその足場を**固定パラメータ**として胴体軌道と地面反力だけを
  最適化する。NMPC の制約は **運動方程式 + 摩擦錐のみ**(脚が届くか=逆運動学、
  縁からどれだけ離れているか、の制約は**無い**)。
- **Phase 0 / 1 / 2A / 3(A) / 2B 実装済み**(下の対応表)。
- **Step 05(15 cm 連続穴 N=2〜5)は通過できた。** 事前調査の「地図 1 セル・
  幾何学的に成立困難寄り」は実測で覆った。
- **Step 05b(単独の断崖:幅 10 m / 100 cm)は手前で安全停止できた。**
- **Step 06(15 cm 穴 ×2 → 1 m 穴)も Phase 2B で「落ちずに止まる」を達成。**
  Phase 2A の「無効足場で plan を丸ごと凍結」は遊脚中に起きると転倒したので、
  Phase 2B で (a) plan を凍結せず `cmd_vel:=0` して既存の STEP→STAND で
  減速停止、(b) NMPC ホライズンより長い `safe_stop_lookahead`(2.5 m)で
  渡れない穴を早期検知して latch、にした。**Step 06 は 3/3 で 15 cm 穴群の
  手前で直立静止**(転倒なし)、回帰も全 OK。
  → 次は Phase 4(逆運動学の可到達性)。

---

## メンタルモデル:安全のための 4 段

穴に落ちない・転ばないためには、次の 4 段がそれぞれ仕事をする必要がある。
Quad-SDK は 1〜2 は持っているが、3〜4 が弱い。ここを足している。

| 段 | 何をする | Quad-SDK の現状 | 本作業 |
|---|---|---|---|
| 1. 認識(perception) | 地形マップに穴を「欠測(NaN)」として出す | メッシュに実穴があれば NaN になる(静的 PLY のラスタライズ)。実センサ処理は無い | 触らない |
| 2. 足場選択(Foot Placement) | Raibert の仮足場を、地図を見て安全なセルへスナップ。ダメなら「ダメ」と言う | スナップはする。が、**ダメでも名目足場をそのまま返す**(下流に伝わらない)。縁距離・IK は見ない | **Phase 1**:成功/失敗の種類 + 診断値を返す型へ。**Phase 3**:縁からの安全距離 `EDGE_TOO_CLOSE` + 渡河可能性(forward-probe) |
| 3. NMPC へ渡さない(gate) | 無効な足場を NMPC / トルクへ伝播させない | 無い(名目足場がそのまま NMPC のパラメータになる) | **Phase 2A**:無効足場があれば local plan を publish しない。無効 touchdown は直前値を踏襲(穴上/NaN を書かない) |
| 4. 安全に止まる(stop シーケンス) | 減速 → 遊脚着地 → 全脚接地 → STAND 保持。渡れない穴は横断前に検知 | 無かった(「plan を止める → PD ホールド」だけ。遊脚中に起きると転ぶ) | **Phase 2B**:plan 凍結せず `cmd_vel:=0` → 既存 STEP→STAND。+ 胴体前方 `safe_stop_lookahead`(2.5 m)で早期 latch。+ probe の地図端打ち切り |

### 「渡れる穴は渡る / 渡れない穴の手前で止まる」の判定

- Foot Placement 段(2)の中で、**足場から進行方向へ `max_crossable_gap`
  (既定 0.6 m)まで前方スキャン**する(Phase 3(A) の forward-probe)。
- 穴が始まって、その先 `max_crossable_gap` 以内に固い地面が戻れば **渡れる穴**
  → `VALID` のまま(step03/04 の 0.3 m 溝、Step 05 の 15 cm 穴)。
- 戻らなければ **渡れない穴 / 断崖** → `EDGE_TOO_CLOSE` → gate(3)が止める。
- `max_crossable_gap` = **「渡河可能と見なす前方到達距離 = 認識/到達範囲」**。
  ユーザーの言う「認識範囲を規定し、認識範囲内で渡れないと判断する方法」は
  これに相当する。

---

## フェーズ ↔ 何を足したか ↔ どのシナリオで確かめたか

| Phase | 足したもの(段) | 挙動変化 | 確認シナリオ | 状態 |
|---|---|---|---|---|
| 解析 / 0 | 資料⇔コード照合、資料の 3 誤り訂正 | なし | ― | ✅ |
| **1** | 足場選択(2)を「位置だけ」→「status + 診断値」を返す型へ | **不変** | `local_planner` テスト 29→green | ✅ |
| **2A** | gate(3):無効足場を NMPC へ渡さない。無効 touchdown は直前値踏襲 | VALID 経路は不変。無効時は plan 非 publish → PD ホールド | Step 05b(単独断崖で発火するが受動ホールドでは勢いを止めきれず転落 → Phase 3 が必要と判明) | ✅ |
| **3(A)** | 足場選択(2):`EDGE_TOO_CLOSE`(縁の安全距離)+ forward-probe(渡河可能性) | `edge_clearance:=0.15` のときだけ縁の足場を無効化。既定 0.0 で不変 | Step 05b:30 cm 溝は渡る / 100 cm 断崖は手前で直立停止。step03 回帰 OK | ✅ |
| **2B** | 段 4:latch → cmd_vel:=0 → STEP→STAND → 保持(plan 凍結せず)+ 長距離 lookahead で早期 latch | ラッチ時のみ | **Step 06(3/3 で 15 cm 穴群の手前で直立静止)** + step03/04・Step 05・Step 05b 回帰 | ✅ |
| 4 | 段 2:逆運動学の可到達性で候補を絞る | ― | ― | ⬜ **次** |
| 5 | 大きな足場補正時の減速/刻み歩行 | ― | ― | ⬜ |
| 6 | 地図の鮮度・未観測セルの扱い | ― | ― | ⬜ |

---

## シナリオ一覧と現状

| シナリオ | 地形 | 設定 | 結果 | 効いている Phase |
|---|---|---|---|---|
| step03 / 04 | 深 1 m・幅 0.3 m の溝、間隔 2.0 / 1.5 m、複数本 | `edge_clearance:0` | **連続で渡る**(既存成功、維持) | 歩容調整のみ(C++ 不変) |
| Step 05 | 15 cm 平地 / 15 cm 穴 ×2〜5、深 1 m | `edge_clearance:0.15` | **N=2〜5 で渡り切る**(胴体 z≈0.31 保持) | 2A + 3(A) |
| Step 05b | 単独トレンチ 幅 10 m / 100 cm、深 1 m | `edge_clearance:0.15` | **手前で直立停止・保持**(転落なし) | 2A + 3(A) |
| Step 05b | 単独トレンチ 幅 30 cm(= step03/04 地形) | `edge_clearance:0.15` | **渡り切る**(渡河可能と判定) | 2A + 3(A) |
| **Step 06** | 15 cm 穴 ×2 → 15 cm 平地 → **1 m 穴** | `edge_clearance:0.15` | **成立**(Phase 2B、3/3 で 15 cm 穴群の手前 x≈0.93 で直立静止、転倒なし) | 2A + 3(A) + **2B** |

---

## ユーザー判断の履歴

| 日 | 判断 |
|---|---|
| 2026-08-31 | 穴縁マージンはまず 0.05 m。同一シナリオ 5 回だめならそのシナリオに限り緩和可 |
| 2026-08-31 | Stage B(Foot Placement 単体検証)は gtest 追加で(挙動不変・テストのみ) |
| 2026-08-31 | **安全停止を先に**やる。検証は幅 10 m の穴の手前で 3 秒止まれたら OK |
| 2026-08-31 | Step 05 で N=2 破綻時はサイズ緩和して掃引、いろいろな成功/失敗例を見たい |
| 2026-09-01 | Phase 3 は初版(全方位)が狭い穴も止めてしまうので、案 A(渡河可能性判定)へ |
| 2026-09-01 | Phase 3 の検証は 30 cm 穴 と 100 cm 穴 の 2 シナリオで |
| 2026-09-01 | **Phase 2B へ進む。ただし今までのシナリオができることを前提に** |
| 2026-09-01 | 進む前に「今までの考え方」を .md に整理して README リンク(この文書) |

---

## 守っているルール

1. **1 コミット = 1 目的**(制御コードとテスト/ドキュメントは別コミット)。
2. **各 Phase は着手前に変更計画(表)を提示し、確認を取ってからコードを変更**。
3. **明示指示がないファイル変更をしない。**
4. **作業したら .md を作成/更新し README にリンク**(冒頭は 背景→目的→結論、
   読者は大学院初心者)。多フェーズは running doc
   (`quadsdk_gap_foothold_phase_progress.md`)を更新し続ける。
5. **エージェント作成の .md は `agent_reports/` 配下**、README はリンクのみ。
6. **CoinHSL / MA27 / MPC ゲイン調整は明確な必要性が確認されるまでしない**。
7. `.gitignore` は行末インラインコメント非対応。
8. **既存の「渡れる」挙動(step03/04・Step 05)を壊さない**(今回のユーザー念押し)。

---

## リンク(詳細)

- `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` — 解析本体
  (資料⇔コード照合・terrain map の式・足場計画 I/O・NMPC 受け渡し・フェーズ根拠)
- `agent_reports/quadsdk_gap_foothold_phase_progress.md` — フェーズ実施ログ
  (コミット単位。Phase 0/1/2A/3(A) の実装詳細、Phase 2B 変更計画)
- `agent_reports/handoff/quadsdk_gap_foothold_handoff.md` — 引き継ぎ(Phase 2B 設計項目)
- `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` — 溝渡り成功記録
- `agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md` — Step 05(15 cm 連続穴)
- `agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md` — Step 05b(断崖前の安全停止)
- `agent_reports/steps/step_06_quadsdk_last_gap_1m.md` — Step 06(15 cm 穴 ×2 → 1 m 穴)
- `chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md` /
  `chatgpt_instruction/cursor_instruction_quadsdk_step05_repeated_15cm_gaps.md` — 指示書
