# 穴対応 Foot Placement 改善:まとめ(現状・成果・GIF)

読者は制御の大学院初心者。**1 枚で現状がわかる**ことを狙う。考え方は
`quadsdk_gap_foothold_overview.md`、回り道は
`quadsdk_gap_foothold_trial_and_error.md`、コミット単位は
`quadsdk_gap_foothold_phase_progress.md`。

---

## 背景

Go2(Quad-SDK、`reference:=twist`)で、以前 **深さ 1 m・幅 0.3 m の溝を足を
入れずに複数本連続で渡る**ことに成功していた。ただし「有効な足場が無いのに
名目の足場で歩き続ける」等の危険な挙動がコードに残っていた。

## 目的

**既存の「渡れる」挙動を壊さずに**、足りない安全機能を 1 コミット = 1 目的で
段階的に足す。

## 結論(現状)

- **安全のための 4 段**(認識 → 足場選択 → gate → 停止シーケンス)のうち、
  **段 2〜4 を実装完了**。
- 狙った **主要シナリオ**(渡る 3・断崖前の安全停止 2)が **すべて成立**。
  Phase 4(IK 可到達性)も専用地形で ON=停止/OFF=転倒 を実地確認(Step 07)。
- 新機能はすべて **既定 OFF / 従来挙動**(有効化して検証するまで step03/04・
  Step 05 は不変)。`local_planner` テスト **40/40 green**。
- 次は Phase 5(大きな足場補正時の減速)。

---

## フェーズ(すべて実装済み・push 済み)

| Phase | 何を足したか | 既定 | 主なコミット |
|---|---|---|---|
| 解析 / 0 / 1 | コード精読、資料訂正、足場選択を「status + 診断値を返す型」へ(挙動不変) | ― | `3a6c705` `6e089e1` `484ea13` |
| **2A** | 無効足場を NMPC へ渡さない gate。無効 touchdown は直前値を踏襲 | ON | `964bd53` `c8236a0` `007396a` |
| **3(A)** | `EDGE_TOO_CLOSE`(穴縁の安全距離)+ 進行方向 forward-probe で「渡れる穴/渡れない穴」を区別 | OFF(`edge_clearance:0.0`) | `8466ad4` `6814895` |
| **2B** | 無効足場検知で plan を凍結せず `cmd_vel:=0` → 既存 STEP→STAND で減速停止。+ 胴体前方 `safe_stop_lookahead`(2.5 m)で早期停止判断。+ probe の地図端打ち切り | ON(VALID 経路は no-op) | `0063270` `1899558` `b7f6b75` |
| **4** | 選択足場を既存脚 IK で可到達性チェック → `IK_UNREACHABLE`。Step 07 で実地確認(専用地形で ON=停止/OFF=転倒) | OFF(`ik_reach_check:false`) | `f93d1f2` `a7d222f` |
| 5 / 6 | 大補正時の減速 / 地図の鮮度 | ― | 未着手 |

### 「渡れる穴は渡る / 渡れない穴の手前で止まる」の判定

足場から進行方向へ `max_crossable_gap`(0.6 m)前方スキャンし、穴の向こうに
固い地面が **その距離以内に戻れば「渡れる穴」**(`VALID`)、**戻らなければ
「渡れない穴/断崖」**(`EDGE_TOO_CLOSE`)。停止判断は NMPC ホライズンより長い
`safe_stop_lookahead`(2.5 m)でも別に前方を見て、**渡れない穴を横断前に検知**する。
= ユーザーの言う「環境認識部で認識範囲を規定し、認識範囲内で渡れないと判断」。

---

## シナリオ結果と GIF

いずれも `reference:=twist` + クロール歩容、0.3 m/s、固定カメラ。

### 1. step03 / 04:30 cm の溝を複数本、足を入れずに渡る(既存成功・維持)

| step03(間隔 2.0 m、0.15 m/s、12–35 s) | step04(間隔 1.5 m、0.3 m/s、15–40 s) |
|---|---|
| ![step03](../artifacts/gifs/quadsdk_step03_1m_v0p15_12to35s.gif) | ![step04](../artifacts/gifs/quadsdk_step04_1m_v0p3_15to40s.gif) |

詳細:`steps/step_03_04_1m_quadsdk_gap_crossing.md`

### 2. Phase 3(A):渡れる穴は渡る / 渡れない穴の手前で直立停止

`edge_clearance:=0.15`。

| 30 cm 深穴:跨いで渡り切る(15–30 s) | 100 cm 幅の穴:手前で直立停止(15–25 s) |
|---|---|
| ![30cm cross](../artifacts/gifs/quadsdk_phase3_gap30_cross_15to30s.gif) | ![100cm stop](../artifacts/gifs/quadsdk_phase3_gap100_safestop_15to25s.gif) |

詳細:`steps/step_05b_quadsdk_phase2a_safe_stop.md`

### 3. Step 05:15 cm 平地 / 15 cm 穴 ×5 をクロールで渡る(10–30 s)

`edge_clearance:=0.15`。N=2〜5 いずれも再現性を持って通過(胴体 z≈0.31 保持)。
事前調査の「地図 1 セル・幾何学的に成立困難寄り」は実測で覆った。

![step05 cross](../artifacts/gifs/quadsdk_step05_s15g15n5_cross_10to30s.gif)

詳細:`steps/step_05_quadsdk_repeated_15cm_gaps.md`

### 4. Step 06:15 cm 穴 ×2 → 1 m 穴 — Phase 2B で手前に直立停止

`edge_clearance:=0.15`。**Phase 2B 前**は、遠方ホライズンの無効足場で plan を
丸ごと凍結 → 手前の 15 cm 穴を渡る遊脚中に凍結して転倒(左)。
**Phase 2B 後**は、胴体前方 2.5 m の lookahead で 1 m 穴を早期検知 →
`cmd_vel:=0` で減速し、15 cm 穴群の手前で直立静止(右、3/3、転倒なし)。

| Phase 2B 前:手前の穴で断続停止 → 転倒(10–30 s) | Phase 2B 後:手前で直立停止(10–30 s) |
|---|---|
| ![step06 fall](../artifacts/gifs/quadsdk_step06_last1m_fall_10to30s.gif) | ![step06 stop](../artifacts/gifs/quadsdk_step06_last1m_safestop_10to30s.gif) |

詳細:`steps/step_06_quadsdk_last_gap_1m.md`

### 5. Step 07:Phase 4(IK 可到達性)の動作確認 — 届かない足場を検知して停止

地形マップの助走側だけを削って**前方スナップを強制**(物理地面は普通)。
`ik_reach_check:=true` で前方足場が `IK_UNREACHABLE` になり、gate + latch で
手前に直立停止(左)。`false` だと同地形で届かない足場を実行して転倒(右)。

| Phase 4 ON:届かない足場を検知 → 手前で停止(10–30 s) | Phase 4 OFF:同地形で転倒(10–30 s) |
|---|---|
| ![phase4 stop](../artifacts/gifs/quadsdk_phase4_ik_safestop_10to30s.gif) | ![phase4 fall](../artifacts/gifs/quadsdk_phase4_ik_fall_10to30s.gif) |

詳細:`steps/step_07_quadsdk_phase4_ik_reach.md`

---

## シナリオ ↔ 結果 ↔ 効いている Phase

| シナリオ | 地形 | 設定 | 結果 | Phase |
|---|---|---|---|---|
| step03 / 04 | 深 1 m・幅 0.3 m 溝、複数本 | `edge_clearance:0` | 連続で渡る(維持) | 歩容調整のみ |
| Step 05 | 15 cm 平地 / 15 cm 穴 ×2〜5 | `edge_clearance:0.15` | N=2〜5 で渡り切る | 2A + 3(A) |
| Step 05b | 単独トレンチ 30 cm | `edge_clearance:0.15` | 渡り切る | 2A + 3(A) |
| Step 05b | 単独トレンチ 10 m / 100 cm | `edge_clearance:0.15` | 手前で直立停止 | 2A + 3(A)(+ 2B で滑らかに) |
| Step 06 | 15 cm 穴 ×2 → 1 m 穴 | `edge_clearance:0.15` | 15 cm 穴群の手前で直立静止(3/3) | 2A + 3(A) + **2B** |
| Step 07 | 助走側マップを削った地形 | `ik_reach_check:1` | 届かない足場を検知して手前で直立静止(3/3)。OFF は転倒 | 2A + 2B + **4** |

---

## 制御コードの安全性(既存を壊していない根拠)

- 追加機能はすべて **既定で従来挙動**:
  `edge_clearance: 0.0`(Phase 3 OFF)/ `ik_reach_check: false`(Phase 4 OFF)/
  `safe_stop_latch: true` だが **足場が全部 VALID の経路では no-op**。
- `IK_UNREACHABLE` / `EDGE_TOO_CLOSE` はいずれも `!= VALID` なので、
  下流の gate(2A)と停止ラッチ(2B)がそのまま拾う(**追加配線ゼロ**)。
- `local_planner` テスト **40/40 green**。
- sim 回帰:step03/04・Step 05・Step 05b・Step 06 を毎フェーズ後に再走して確認。

## 未着手・保留

- **Phase 4 は掃引地形では踏まない**(足場スナップが手前へ寄るため)。専用地形
  (Step 07)で ON=安全停止/OFF=転倒 を実地確認済み。実運用では探索半径拡大/
  別歩容/実センサ用の安全網。
- forward-probe / lookahead は **+x(進行方向)固定**(全幅横断穴では妥当。
  斜め穴・旋回は将来一般化)。
- 実センサ(LiDAR/深度 → 地図)処理はこの repo に無い。「穴」と「未観測」の
  区別、地図の鮮度は Phase 6。
- Phase 5(大きな足場補正 = 大 snap 時の減速/刻み歩行)。

## 関連ドキュメント

- `quadsdk_gap_foothold_overview.md` — 全体の考え方(4 段モデル、フェーズ対応表)
- `quadsdk_gap_foothold_trial_and_error.md` — 試行錯誤(外した見立て・踏んだバグ)
- `quadsdk_gap_foothold_phase_progress.md` — コミット単位の実施ログ
- `quadsdk_gap_foothold_mpc_code_analysis.md` — コード解析本体
- `steps/step_05_*` / `step_05b_*` / `step_06_*` / `step_07_*` — 各シナリオの実測
