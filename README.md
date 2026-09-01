


# Squad-SDK のためのREADME

 - [README](./agent_reports/step01/quad_sdk_environment_and_step01.md)
 - [PyMPCとSquad-SDKの使い分け](./agent_reports/step01/quad_sdk_pympc_selection_and_distribution.md)

## Quad-SDK

 - [Step 01 検証記録(経緯・実験ログ)](./agent_reports/step01/quad_sdk_step01_investigation.md)
 - [Step 01 引き継ぎ資料(前進歩行の到達点・未完了事項・変更ファイル一覧)](./agent_reports/handoff/quadsdk_step01_handoff.md)
 - [Step 01 変更点と実行方法(要点まとめ)](./agent_reports/step01/quad_sdk_step01_changes_and_usage.md)
 - [記録ハーネス(quadsdk_step01_baseline.py)の構造説明](./agent_reports/step01/quadsdk_step01_baseline_py_structure.md)
 - [Step 01 の制御パイプライン(map → sensing → MPC → WBC のノード構成)](./agent_reports/quadsdk_step01_control_pipeline.md)
 - [Step 01 の地形マップ(map)の作り方とデータ構造(事実と推測を分離)](./agent_reports/quadsdk_step01_terrain_map.md)
 - [Step 01 のセンシング(状態推定)の仕組みとデータ構造(事実と推測を分離)](./agent_reports/quadsdk_step01_sensing.md)
 - [Step 01 の MPC(NMPC)の理論・コスト・制約・最適化とパラメータ(事実と推測を分離)](./agent_reports/quadsdk_step01_mpc.md)
 - [Step 01 の WBC(脚コントローラ/逆動力学)の理論・コード・パラメータ(事実と推測を分離)](./agent_reports/quadsdk_step01_wbc.md)
 - [Step 01 の GAIT(歩容)と MPC の関係 — 理論式・コード(事実と推測を分離)](./agent_reports/quadsdk_step01_gait_and_mpc.md)
 - [NMPC の simple モデルと complex モデルの差分 / MPC でできることの違い(事実と推測を分離)](./agent_reports/quadsdk_step01_mpc_simple_vs_complex.md)
 - [simple モデルで地形対応(高さ考慮の足場選び・穴超え)はどこまでできるか(事実と推測を分離)](./agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md)
 - [go2 の寸法・質量・関節・パラメータ(Quad-SDK モデル + 公称スペック)](./agent_reports/quadsdk_go2_dimensions_and_params.md)
 - [Quad-SDK 元コードからの変更・チューニングまとめ(ビルド/実行修正・歩容/探索/ホライズン/地形表現の調整・診断機能追加。何をなぜ変えたか一覧)](./agent_reports/quadsdk_original_code_tuning_summary.md)
 - [穴対応 Foot Placement 改善:**まとめ(現状・成果・GIF)** — 1 枚で現状がわかる。フェーズ一覧(2A/3(A)/2B/4 実装済み)、5 シナリオの結果と実行 GIF、既存挙動を壊していない根拠](./agent_reports/quadsdk_gap_foothold_summary.md)
 - [穴対応 Foot Placement 改善:**全体の考え方(概観)** — 安全のための 4 段(認識/足場選択/gate/停止シーケンス)、フェーズ ↔ 何を足したか ↔ どのシナリオで確かめたか、シナリオ一覧と現状、ユーザー判断の履歴](./agent_reports/quadsdk_gap_foothold_overview.md)
 - [穴対応 Foot Placement 改善:**試行錯誤の記録** — 外した見立て(15cm連続穴は成立困難 / gate だけで安全停止)、踏んだC++バグ(非voidの return 忘れ→UBで無限ループ / THROTTLE がms巨大値でログ0件)、Phase 3 を2回作り直した経緯、効いた作業のやり方](./agent_reports/quadsdk_gap_foothold_trial_and_error.md)
 - [穴対応:Foot Placement と NMPC 連携のコード解析(資料⇔コード照合表・terrain map の式・足場計画の入出力・NMPC への受け渡し・資料主張の再判定・改善フェーズ計画)](./agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md)
 - [穴対応:**なぜ穴の自動検知は中サイズの穴を見逃すのか(データの流れとロジックの中身)** — 大学院初心者向け。PLY→grid_map→フィルタ連鎖(穴埋め処理 radius 0.4)→「安全度」レイヤ→穴チェック①/②の疑似コード、30/40/50/100 cm で安全度の断面がどう変わるか、危険帯 0.35〜0.9 m、`MESH_MARGIN` の落とし穴、Phase 5 の直し方](./agent_reports/quadsdk_gap_foothold_probe_and_inpaint.md)
 - [穴対応 Foot Placement 改善:フェーズ実施ログ(何をどのコミットで変えたか。Phase 0 資料訂正 / Phase 1 FootholdResult 診断値)](./agent_reports/quadsdk_gap_foothold_phase_progress.md)
 - [穴対応 Foot Placement 改善:次チャットへの引き継ぎ(現状・次の 1 ステップ・Phase 2A をブロックしている確認事項・守るべきルール・ファイルの場所)](./agent_reports/handoff/quadsdk_gap_foothold_handoff.md)
 - [Step 03_1m / 04_1m:1m 深の穴を「足を入れずに」複数本連続で渡る(成功／歩容をクロールに調整・大学院初心者向け解説つき)](./agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md)
 - [Step 03_1m / 04_1m(gbpl 版):global_body_planner で穴を渡る試み + Quad-SDK 公式マニュアルに基づく正しい設定方法 + センシング→foot plan→MPC→WBC→トルクの工程別ボトルネック分析(穴 1 本は跳んで可・連続区間は未達)](./agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md)
 - [Step 05:15 cm 平地・15 cm 穴の連続区間 — **go2 は N=2〜5 で安定して渡り切った**(Phase 3(A) が 15 cm 穴を「渡れる穴」と判定・クロール歩容で 5 cm メッシュ帯を跨ぐ)。事前調査の「成立困難寄り」は実測で覆った](./agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md)
 - [Step 05b:安全停止の検証 — Phase 2A 単独では受動 PD ホールドが勢いを止めきれず転落。**Phase 3(`EDGE_TOO_CLOSE`、進行方向 forward-probe)を足すと、30 cm の穴は跨いで渡り・100 cm の穴の手前で直立停止**(渡れる穴は渡る/渡れない穴の手前で安全に止まる)](./agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md)
 - [Step 06:15 cm 穴 ×2 → 1 m 穴 の複合地形で「落ちずに止まれるか」 — **Phase 2B で達成(3/3 で 15 cm 穴群の手前で直立静止・転倒なし)**。plan を凍結せず `cmd_vel:=0` で減速停止 + NMPC ホライズンより長い前方 lookahead(2.5 m)で 1 m 穴を早期検知。既存シナリオ(step03/04・Step 05・Step 05b)も回帰 OK](./agent_reports/steps/step_06_quadsdk_last_gap_1m.md)
 - [Step 07:Phase 4(`IK_UNREACHABLE`)の動作確認 — **30 cm / 100 cm × `ik_reach_check` OFF/ON の 4 通り**。Phase 4 ON でも 30 cm は渡り切る(機能後退なし)、100 cm は手前で停止。初版は IK の `is_exact` フラグで平地足場まで拾い 30 cm 渡りを止めた → midstance hip からの幾何距離判定に修正](./agent_reports/steps/step_07_quadsdk_phase4_ik_reach.md)
 - [Step 08:穴の数・間隔・幅を振った **全 18 シナリオの回帰スイープ** — 16/18 は期待どおり(≤30 cm は回帰なしで渡り・≥100 cm は手前で直立停止)。**~0.4〜0.9 m の穴で「渡れず・止まらず・転倒」**。※当初「真因は `InpaintFilter` が穴を埋めるから」としたが、Step 09 の計測で **誤りと判明**(下記/Step 09 参照)](./agent_reports/steps/step_08_quadsdk_full_gap_sweep.md)
 - [Step 09:Terrain Map と足場判断の **セル単位の定量計測**(制御変更なし、env ガード計装)— 15/25/30/35/50/100 cm の断面 CSV + 足場 CSV。**50 cm 転倒の因果を数値で確定**:向こう岸へのスナップ(B)は無し、`traversability` は穴の内側を正しく unsafe にしている。落ちるのは(1)既定 `edge_clearance:0` で幅チェックが走らず(2)スナップが **物理 void の縁 1 セル**(ぼかしで `traversability`=1.0 だが生 `z`=NaN)に足を置くため。`max_crossable_gap` を ≤0.44 に下げれば 50 cm は捕まる(Step 08 の「閾値では直らない」を訂正)](./agent_reports/steps/step_09_terrain_grid_and_foothold_measurement.md)
 - [なぜ 50 cm の穴で「数歩手前で止まる」がまだできないのか — **指示書 ↔ 実施の突き合わせ** — 指示書は Step 09〜16 の 8 段階で、「M 歩手前で停止」は Step 14 のゴール。いまは指示書 §9 が限定した **Step 09(計測のみ・制御不変)しか終えていない**ので止める処理が 1 行も入っていない。Step 10(未来脚順序)→11(到達可能な足場候補)→12(複数歩足場列)→13(停止余裕 M 歩)→14(graceful stop 接続)が必要](./agent_reports/steps/step_09b_why_50cm_not_stopping.md)

### 実行例(時系列)

各 Step の試行錯誤ごと残す。上ほど古い。GIF は固定カメラ、`reference:=twist` +
クロール歩容。地形マップは静的メッシュ由来。

---

#### Step 03 / 04:深さ 1 m・幅 0.3 m の溝を、足を入れずに複数本連続で渡る(成功)

歩容をトロット → クロールに落として達成。詳細・CSV 根拠は
[Step 03_1m / 04_1m の検証記録](./agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md)。
別アプローチ(global_body_planner)は連続区間で未達 →
[gbpl 版](./agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md)。

| step03(溝の間隔 2.0 m、0.15 m/s、12–35 s 切り抜き) | step04(溝の間隔 1.5 m、0.3 m/s、15–40 s 切り抜き) |
|---|---|
| ![step03 溝渡り](./artifacts/gifs/quadsdk_step03_1m_v0p15_12to35s.gif) | ![step04 溝渡り](./artifacts/gifs/quadsdk_step04_1m_v0p3_15to40s.gif) |

---

#### Step 05:15 cm 平地 / 15 cm 穴 ×5 をクロールで渡る(成功)

`edge_clearance:=0.15`、0.3 m/s。Phase 3(A) が 15 cm 穴を「渡れる穴」と判定し、
クロール歩容で 5 cm メッシュ帯を跨ぐ。N=2〜5 いずれも再現性を持って通過。
事前調査の「地図 1 セル・幾何学的に成立困難寄り」は実測で覆した。詳細は
[Step 05 の検証記録](./agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md)。

![Step05 15cm連続穴を渡る](./artifacts/gifs/quadsdk_step05_s15g15n5_cross_10to30s.gif)

---

#### Step 05b:Phase 2A 単独では止まれない → Phase 3(`EDGE_TOO_CLOSE`)を足して安全停止

**試行錯誤**:無効足場を NMPC に渡さない gate(Phase 2A)だけでは、受動 PD
ホールドが勢いを止めきれず、穴の縁で前傾して落ちる(左)。→ 進行方向 forward-probe
で穴の縁を早期検知する Phase 3 を足すと、渡れる穴は跨ぎ、渡れない穴の手前で
直立停止する(右)。詳細は
[Step 05b の検証記録](./agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md)。

| Phase 2A 単独:縁で一瞬止まって前傾転落 | Phase 3 追加:穴の 0.7 m 手前で直立停止 |
|---|---|
| ![Phase2A 転落](./artifacts/gifs/quadsdk_phase2a_trench10m_fall.gif) | ![Phase3 安全停止](./artifacts/gifs/quadsdk_phase3_trench10m_safestop.gif) |

Phase 3 有効時(`edge_clearance:=0.15`)の代表:

| 30 cm 深穴：跨いで渡り切る(15–30 s 切り抜き) | 100 cm 幅の穴：手前で直立停止(15–25 s 切り抜き) |
|---|---|
| ![30cm 穴を渡る](./artifacts/gifs/quadsdk_phase3_gap30_cross_15to30s.gif) | ![100cm 穴の手前で安全停止](./artifacts/gifs/quadsdk_phase3_gap100_safestop_15to25s.gif) |

---

#### Step 06:15 cm 穴 ×2 → 1 m 穴 の複合地形 — Phase 2B で 15 cm 穴群の手前に直立停止

**試行錯誤**:NMPC ホライズン(≈0.36 m 先)では 1 m 穴の認識が遅すぎ、plan を丸ごと
凍結すると手前の 15 cm 穴を渡る遊脚中に凍結して転倒(左)。→ Phase 2B:plan を
凍結せず `cmd_vel:=0` で減速 + 胴体前方 2.5 m の lookahead で 1 m 穴を早期検知
(右、3/3 で転倒なし)。既存シナリオ(step03/04・Step 05・Step 05b)も回帰 OK。詳細は
[Step 06 の検証記録](./agent_reports/steps/step_06_quadsdk_last_gap_1m.md)。

| Phase 2B 前:手前の 15 cm 穴で断続停止 → 転倒(10–30 s) | Phase 2B 後:1 m 穴を早期検知 → 手前で直立停止(10–30 s) |
|---|---|
| ![Step06 転倒](./artifacts/gifs/quadsdk_step06_last1m_fall_10to30s.gif) | ![Step06 安全停止](./artifacts/gifs/quadsdk_step06_last1m_safestop_10to30s.gif) |

---

#### Step 07:Phase 4(`IK_UNREACHABLE`)— 脚が届かない足場を検知して手前で停止

**試行錯誤**:初版は IK の `is_exact` フラグで判定 → 平地の足場まで拾って 30 cm
渡りを止めた(機能後退)。→ midstance hip からの幾何距離判定に修正。`ik_reach_check`
既定 OFF。専用地形(助走側の地図を削り、前脚の名目足場を midstance hip から
~0.75 m > `ik_max_reach`(0.45 m)にスナップさせる)で確認。詳細は
[Step 07 の検証記録](./agent_reports/steps/step_07_quadsdk_phase4_ik_reach.md)。

| `ik_reach_check:=false`:届かない足場を実行 → 転倒 | `ik_reach_check:=true`:`IK_UNREACHABLE` 検知 → 直立停止 |
|---|---|
| ![Phase4 OFF 転倒](./artifacts/gifs/quadsdk_phase4_ik_fall_10to30s.gif) | ![Phase4 ON 停止](./artifacts/gifs/quadsdk_phase4_ik_safestop_10to30s.gif) |

Phase 4 ON でも 30 cm の溝は渡り切る(機能後退なし):
![Phase4 ON でも 30cm は渡る](./artifacts/gifs/quadsdk_phase4_g30_ik_cross_12to40s.gif)

**機能 ON/OFF の比較(30 cm の溝 / 100 cm の穴)**:
ON = `edge_clearance:=0.15`。OFF = `stop_on_invalid_foothold:=false` +
`safe_stop_latch:=false` + `edge_clearance:=0`(この穴対応の作業を全部無効化 =
素の Quad-SDK 挙動)。100 cm は助走を見せるため spawn を x=−2.0 に後退
(`SPAWN_X_M=-2.0`、地形は不変、穴の近縁 x=2.0)。

| | 機能 OFF | 機能 ON |
|---|---|---|
| **30 cm の溝** | ![30cm off](./artifacts/gifs/quadsdk_onoff_g30_off.gif)<br>渡り切る(x≈11.7) | ![30cm on](./artifacts/gifs/quadsdk_onoff_g30_on.gif)<br>渡り切る(x≈11.1) |
| **100 cm の穴**<br>(spawn x=−2.0) | ![100cm off](./artifacts/gifs/quadsdk_onoff_g100_off.gif)<br>4.6 m 歩いて**穴に落下** | ![100cm on](./artifacts/gifs/quadsdk_onoff_g100_on.gif)<br>2.2 m 歩いて**穴の手前で直立停止**(standoff ≈ `safe_stop_lookahead 2.5 − max_crossable_gap 0.6`) |

---

#### Step 08:穴の数・平地幅・穴幅を振った全 18 シナリオの回帰スイープ

`edge_clearance:=0.15`。**16/18 は期待どおり**(≤0.30 m は回帰なしで渡り・
≥1.0 m は手前で直立停止)。**残り 2/18 = 幅 0.5 m の穴で「渡れず・止まらず・転倒」**
を発見。詳細は
[Step 08 の検証記録](./agent_reports/steps/step_08_quadsdk_full_gap_sweep.md)。

**タスク結果:穴幅 → 挙動**(単独トレンチ。`scripts/trial/step08_chart.py`)

![Step08 穴幅と挙動](./artifacts/step_charts/step08_gap_width_vs_outcome.png)

参考 GIF:

| ✅ 幅 ≤0.30 m:渡り切る | ✅ 幅 ≥1.0 m:手前で直立停止 | ❌ 幅 0.5 m:落下(未対応) |
|---|---|---|
| ![30cm 渡る](./artifacts/gifs/quadsdk_onoff_g30_on.gif) | ![100cm 止まる](./artifacts/gifs/quadsdk_onoff_g100_on.gif) | ![50cm 落下](./artifacts/gifs/quadsdk_gap50_fall.gif) |

**試行錯誤(この時点の見立ては後に Step 09 で訂正)**:当初は「`max_crossable_gap`
を 0.6→0.54 に下げても直らない → 真因は `InpaintFilter` が穴を `traversability`
まで埋めているから」と結論した。→ **Step 09 で誤りと判明**(下記)。

---

#### Step 09:セル単位の計測で 50 cm 転倒の因果を数値で確定(制御変更なし)

env ガード計装で 15/25/30/35/50/100 cm の地図断面 CSV + 足場 CSV を採取。詳細は
[Step 09 の検証記録](./agent_reports/steps/step_09_terrain_grid_and_foothold_measurement.md)。

| ✅ 30 cm:渡り切る | ❌ 50 cm:縁で転倒 |
|---|---|
| ![step09 30cm](./artifacts/gifs/quadsdk_step09_gap30_cross.gif) | ![step09 50cm](./artifacts/gifs/quadsdk_step09_gap50_fall.gif) |

![step09 50cm 断面](./artifacts/gifs/quadsdk_step09_gap50_cross_section.png)

灰 = 生 `z` が NaN の帯(物理 void、0.60 m)。赤斜線 = `traversability` が unsafe の帯
(0.50 m)。マゼンタ▲ = 足場が置かれたセルで生 `z`=NaN(= void の縁の上)。
赤斜線の内側(穴の中心)には足場は 1 件も無く、B(向こう岸スナップ)も 0 件。

**確定した因果(Step 08 の見立てを訂正)**:

- `traversability` は **どの幅の穴でも内側を正しく unsafe(NaN)** にしている。
  unsafe 帯の幅 ≈ 物理の穴幅(50 cm → 0.50 m、100 cm → 1.00 m)。
  「`InpaintFilter` が穴を埋めるから」は誤りで、埋まるのは高さ(`z_inpainted`)だけ。
- `max_crossable_gap = 0.6 m` は「0.6 m 以内で地面が戻れば渡れる穴」の意味。50 cm の
  unsafe 帯(0.50 m)はこれ未満なので **「渡れる穴」に分類され止めない**。100 cm
  (1.00 m)だけがしきい値を超えて安全停止する。
- 既定 `edge_clearance:0` では幅チェック自体が走らず、足場は `traversability > 0.6`
  の最近傍セル = **物理 void の縁 1 セル**(縁ぼかしで `traversability`=1.0 だが
  生 `z`=NaN = 未支持地面)にスナップし、そこに足を置いて転倒する。
- → `max_crossable_gap` を **≤0.44** に下げれば 50 cm(0.50 m)は `EDGE_TOO_CLOSE`
  になり、30 cm(0.30 m)は crossable のまま。Step 08 の 0.54 は 0.50 より大きく
  外れていただけ。

**教訓**:

1. **「データを疑う」も、実データを取ってから。** Step 08 と解説 doc では「`InpaintFilter`
   が `traversability` まで埋める」と推測で書いたが、Step 09 でセル値を取ると
   **穴の内側は正しく NaN**。誤りは「しきい値 vs データ」ではなく
   「しきい値の値が穴幅に対して大きすぎる」+「既定でチェックが走らない」だった。
2. **計装で 1 段ずつ値を出す。** `z_raw / z_inpainted / hole_mask / traversability` を
   セル単位で並べて初めて「埋めているのは高さだけ」が見えた。
3. **地図の帯幅 ≠ 物理の穴幅、だが規則的。** `生 NaN 帯 = 物理 + 2×MESH_MARGIN`、
   `traversability unsafe 帯 = 生 NaN 帯 − 2×0.05(縁ぼかし)≈ 物理幅`。
4. **1 本の試行を信じない。** go2 の twist 歩容は非決定的で起動失敗フレークが混ざる。
5. **中断した回帰検証は最後まで回す。** 幅を振って回す回帰でしか出ない抜けがあった。

---

#### Step 10:現在歩容から未来の脚順序を再構成(shadow・制御変更なし)

指示書の 8 段(Step 09〜16)の 2 段目。いまの gait 位相から「この先どの脚がどの順で
着地するか」を `computeContactSchedule` の出力そのものから取り出して記録し、実接触
(`state_log.csv` の `contact_*` 立ち上がり)と照合。詳細は
[Step 10 の検証記録](./agent_reports/steps/step_10_future_gait_event_prediction.md)。

**タスク結果:予測 touchdown ↔ 実接触**(色 = 脚。`scripts/trial/step10_analyze.py`)

![Step10 予測 vs 実接触](./artifacts/step10/g30/step10_g30_pred_vs_actual.png)

| 地形 | 予測 脚順 | 実 脚順 | 一致 | touchdown 間隔 誤差 |
|---|---|---|---|---|
| 平地 | `FL→BR→FR→BL` | `FL→BR→FR→BL` | ✅ | 3 ms |
| 30 cm 穴 | `FL→BR→FR→BL` | `FL→BR→FR→BL` | ✅ | 0 ms |
| 連続 15 cm | `FL→BR→FR→BL` | `FL→BR→FR→BL` | ✅ | 0 ms |

3 地形とも脚順一致・間隔誤差 ≤3 ms(sim 1 tick 未満)。→ Step 11(1 歩の可到達領域と
安全足場候補生成)へ。

---

#### Step 11:1 歩の可到達領域と安全足場候補の列挙(shadow・制御変更なし)

各脚の hip 周りの地図セルを走査し、**reach 内**(`‖セル − hip‖ ≤ ik_max_reach`、
Phase 4 と同じ 3D 距離 + 粗い前後左右ボックス)**+ 安全**(`traversability > 0.6`、
足裏 4 近傍も安全)**+ 観測済み**(生 `z` 有限)を満たす数 `n_valid` を記録。詳細は
[Step 11 の検証記録](./agent_reports/steps/step_11_reachable_safe_foothold_candidates.md)。

**タスク結果:前脚の有効足場候補数 `n_valid` vs hip 位置**(`scripts/trial/step11_analyze.py`)

| 30 cm 穴:候補が残る(最小 43) | 100 cm 穴:穴の手前で 0 に落ちる |
|---|---|
| ![Step11 30cm](./artifacts/step11/g30/step11_g30_n_valid.png) | ![Step11 100cm](./artifacts/step11/g100/step11_g100_n_valid.png) |

| 地形 | `n_valid` 最小(穴の近く) | 選択足場が全判定を通る率 | |
|---|---:|---:|---|
| 平地 | 131 | 100 % | ✅ |
| 30 cm 穴 | **43** | 32 % | ✅ 候補は残る(縁足場が選ばれるのは選択ロジックの問題=Step 12〜) |
| 50 cm 穴 | **0** | 17 % | ✅ 向こう岸は reach 外 |
| 100 cm 穴 | **0** | 0 % | ✅ 穴全域で候補なし |

→ 「30 cm は候補が残る / 50・100 cm は候補が消える」という Step 12(複数歩足場列)が
`BLOCKED_AT_STEP_K` を出すための信号が取れた。

---

### 現在の到達点(Step 11 時点):できること / できないこと

**できること**

| 内容 | 根拠 |
|---|---|
| 平地の前進歩行(起立 → 歩行 → 停止)が安定 | Step 01 |
| **幅 ≤0.30 m の穴/溝を、足を入れずに複数本連続で渡る**(回帰なし) | Step 03/04、Step 05 |
| **幅 ≥1.0 m の穴/断崖の手前で直立停止**(`edge_clearance:=0.15` 有効化時)。胴体前方 2.5 m をプローブ → `cmd_vel:=0` で減速 | Step 05b、Step 06 |
| 脚が届かない足場の検知(`ik_reach_check:=true`、専用地形で確認。既定 OFF) | Step 07 |

**できないこと(既知の穴)**

| 内容 | 詳細 |
|---|---|
| **幅およそ 0.44〜0.60 m の穴で「渡れず・止まらず・転倒」** | `max_crossable_gap`(0.6 m)が危険帯(≈ 物理幅 0.5 m)より大きく「渡れる穴」と誤判定。かつ既定 `edge_clearance:0` では幅チェック自体が走らず、足場が穴の縁セルにスナップして落ちる。→ `max_crossable_gap` を ~0.44 に下げるか、穴チェックを生 `z` の NaN で判定(**未着手**) |
| 斜め穴・旋回中の穴 | プローブは +x 方向固定。全幅横断穴のみ対応 |
| 未観測領域と穴の区別 | 地図上どちらも `NaN`。「見えていない所」を安全と誤認しうる |
| 実センサ(LiDAR/深度)からの地図生成 | この repo に無い(MuJoCo メッシュ由来のみ) |
| 複数歩先の足場列計画・停止余裕の歩数換算 | 未実装(Step 10〜16 の対象) |

**安全機能のスイッチ(`local_planner.yaml` 既定)**:
Phase 2A(無効足場を NMPC に渡さない)= **ON** /
Phase 2B(検知時に減速停止)= **ON**(健全地形では no-op)/
Phase 3(A)(渡れる穴/渡れない穴の区別、`edge_clearance`)= **OFF** /
Phase 4(届かない足場の検知、`ik_reach_check`)= **OFF**。
OFF の 2 つは、その実行で明示的に有効化したときだけ動く(既定では素の Quad-SDK 挙動)。

1 枚まとめ:[まとめ doc](./agent_reports/quadsdk_gap_foothold_summary.md)。


## Quadruped-PyMPC

 - [環境構築(acadosビルド・インストール)と実行方法](./agent_reports/step01/pympc_step01_changes_and_usage.md)
 - [Step 01 検証記録(経緯)](./agent_reports/steps/step_01_reference_baseline.md)
 - [記録ハーネス(step_01_baseline.py)の構造説明](./agent_reports/step01/step_01_baseline_py_structure.md)
 - [Step 02 検証記録(平面マップ・歩容周波数と前進速度／成功)](./agent_reports/steps/step_02_frequency.md)
 - [Step 03 検証記録(前進方向に並ぶ穴／轍を落ちずに越える・大学院初心者向け解説つき／成功)](./agent_reports/steps/step_03_gap_crossing.md)
 - [Step 04 検証記録(穴の間隔を 1.5 m に詰めて同様／大学院初心者向け解説つき／成功)](./agent_reports/steps/step_04_gap_crossing_1p5m.md)