# Step 17：Go2 前方ジャンプ — 実装記録

対象読者：この課題に初めて触れる人。
状態：**平地・穴なしでの「その場・垂直ジャンプ」を計測値で確認し、ここでクローズ
（2026-09-03、ユーザー判断）。** 短い前方ジャンプ（後脚 +38 cm）も計測済みだが、
最終的な要件は「その場・垂直・こけずに着地」に絞られた。後脚のみ踏切（`REAR_PUSH`）・
穴シナリオは未実施。堅牢化（Stage C/D）の分析と設計は別紙。事実・計測・推測を分けて書く。

関連：
- 実装前分析 [step_17_forward_jump_code_analysis.md](./step_17_forward_jump_code_analysis.md)
- 垂直ジャンプの gait/WBC 分析・計画（クローズ判断つき）
  [step_17b_vertical_jump_gait_and_wbc_plan.md](./step_17b_vertical_jump_gait_and_wbc_plan.md)

---

## 1. 背景

Go2 に「またぐ」のではなく「前方ジャンプ」をさせたい。分析の結論(前掲)は、
既存の「リープ」が実質「四脚接地スクワット →(運が良ければ)四脚同時飛翔 →
四脚接地」で、後脚だけの支持も前脚だけの着地も**到達不能**、GBP は点質量+単一
合力モデルで姿勢を表現できない、というものだった。

## 2. 方針(途中で修正)

当初は「穴の手前で GBP が自動でジャンプを選ぶ」経路を狙ったが、平地では RRT が
「歩けば届く」と判断してジャンプを選ばない。ユーザー指示で方針を次に絞った。

- **判断ロジック不要**(RRT/global planner に穴判定させない)
- **穴シナリオ不要**(平地のみ)
- **その場ジャンプで可**。前進距離は**後脚の着地 x 位置**で計測して少しずつ伸ばす

## 3. 実装

`jump_mode:=force_leap` のとき、`global_body_planner` ノードが `findPlan`(RRT)を
回さず、ロボットが静止した瞬間に **1 回のジャンプ経路**を決定論的に組み立てて
`body_plan` に流し続ける(`forcedJumpSpinOnce` / `buildForcedJumpPlan`)。

- 現在姿勢を鉛直方向のみのジャンプとして `getRandomLeapAction` で 1 個解く
  (`is_jump=true`)。前進が要るときは、解けた鉛直 GRF の一定割合を前向き
  GRF 成分 `grf_0[0]` として足す(摩擦錐 `0.9·mu` で頭打ち)。
- `interpStateActionPair` が `is_jump` の action を
  `PRELOAD → REAR_PUSH → FLIGHT → FRONT_LAND → SETTLE` に時間分割してスタンプ
  (割合は `jump_preload_fraction` / `jump_front_land_fraction`)。
- `local_footstep_planner::computeContactSchedule` がそれを 4 脚別接触へ変換
  (`PRELOAD/SETTLE={1,1,1,1}` `REAR_PUSH={0,1,0,1}` `FLIGHT={0,0,0,0}`
  `FRONT_LAND={1,0,1,0}`)。
- NMPC + Inverse Dynamics はその body plan を追従するだけ。

計測ハーネス：`scripts/trial/run_step17_jump.sh`(平地 `flat_wide`、穴なし)＋
`src/trial/quadsdk_step17_jump.py`(足先位置・実測接触=足先 z プロキシ・関節
pos/vel/指令トルク・`primitive_id`・`jump_phase`)＋ `scripts/trial/step17_analyze.py`。

### チューニングで効いたこと

| 症状 | 対策 |
|---|---|
| `hop_v0`(dz≈1.6・後脚のみ踏切・姿勢重み既定):離地はするが飛翔中に pitch −80°→ロール 180°で反転着地、NMPC 329 回失敗 | dz を下げる / 支持時間を伸ばす(ピーク GRF を下げる)/ **四脚対称踏切**(`jump_preload_fraction=1.0` で REAR_PUSH を出さない)/ NMPC の roll・pitch 追従重みを 0.5→20 |
| `dz0_min==dz0_max` 固定 → `could not build a valid jump action` | 範囲を残す(`getRandomLeapAction` は範囲内でサンプリングして実行可能解を探す) |
| `s0.vel.x` に前進速度を入れると `refineStance` が収束せず build 失敗 | 前進は速度でなく **`grf_0[0]` 成分**として後付け |
| `x_weights` に整数を書くと ROS が `RCLInvalidROSArgsError` で全ノード abort | 配列要素を全て float 表記に |

---

## 4. 計測結果(平地・穴なし)

### 4.1 その場ジャンプ `step17_hop_sym2`

パラメータ：`vx0=0`、`dz∈[1.1,1.5]`、`t_s∈[0.20,0.28]`、`preload_fraction=1.0`
(四脚対称踏切)、NMPC roll/pitch 重み 20、mu 0.6。

| 項目 | 値 | 判定 |
|---|---|---|
| 胴体上昇(頂点) | 0.318 → **0.544 m(+0.226 m)** | — |
| 四脚離地時間(足先 z > 0.06 m) | **264 ms** | OK(≥30) |
| PRELOAD の沈み込み | 胴体 −35 mm(しゃがみ→伸展) | OK |
| 踏切 GRF ピーク | 全脚対 300 N ずつ(対称・NMPC 上限) | — |
| NMPC 解 | 全区間成功(失敗 0) | OK |
| 前脚離地 / 後脚離地 | 14.51 s / 14.52 s(ほぼ同時) | 対称踏切のため |
| 着地後 2 s | body_z 0.24–0.26 m、\|roll\|,\|pitch\| < 0.003 rad、転倒なし | OK |
| t=41 s | 直立静止、x ≈ 0 | OK |
| 最大関節トルク / 速度 | 39.7 Nm / 16.9 rad/s | OK |

GIF：`artifacts/step17/step17_hop_sym2/hop.gif`

### 4.2 短い前方ジャンプ `step17_fwd_b`

パラメータ：`vx0=0.3`(→ `grf_0[0] = 0.09·grf_0[2] ≈ 0.33` 体重倍)、他は 4.1 と同じ。

| 項目 | 値 | 判定 |
|---|---|---|
| Forced jump plan | `grf_0=[0.33, 0, 3.65]`、計画着地 x=0.43 | — |
| **後脚 BL/BR 平均 x** | **−0.201 → +0.185 m(前進 +0.386 m ≈ 39 cm)** | OK(≥30) |
| 胴体 CoM 前進 | +0.31 m | — |
| 胴体上昇(頂点) | 0.318 → 0.558 m(+0.241 m) | — |
| 四脚離地時間 | **314 ms** | OK(≥30) |
| NMPC 解 | 全区間成功(失敗 0) | OK |
| 着地後 2 s(19–21 s) | body_z 0.238–0.259 m、\|roll\| max 0.001、\|pitch\| max 0.003 rad | OK 転倒なし |
| t=41.9 s | 直立静止、x=0.305、後脚 x ≈ +0.18 | OK |
| 最大関節トルク / 速度 | 50.0 Nm(瞬間上限)/ 17.6 rad/s | 注:トルク上限に瞬間到達 |

GIF：`artifacts/step17/step17_fwd_b/fwd_jump.gif`

### 4.3 失敗例(記録)

| run | 設定 | 結果 |
|---|---|---|
| `step17_hop_v0` | dz≈1.6・後脚のみ踏切・姿勢重み既定 | 離地(胴体+29 cm)するが飛翔中に pitch −1.4 rad → ロール ±3.14 rad(反転)、NMPC 329 回失敗、反転着地 |
| `step17_fwd_a` | vx0=0.8(→ `grf_0[0]≈0.88`) | 前進しすぎ。計画着地 x=1.04、実測後脚前進 +1.02 m だが飛翔/着地で反転(max roll 3.14)、NMPC 318 回失敗 |

---

## 5. 課題 §10 の判定(平地・その場〜短前進の範囲で)

| 成功条件 | `hop_sym2` | `fwd_b` |
|---|---|---|
| 四脚の実接触 < 閾値 が 30 ms 以上 | OK 264 ms | OK 314 ms |
| 全足先が地面から離れる | OK | OK |
| 着地後 2 s 以上転倒しない | OK | OK |
| ロール・ピッチ・関節速度・トルクが上限内 | OK | OK(トルク瞬間上限50到達) |
| CoM 前進 | +0 m(その場) | +0.31 m |
| 前脚が後脚より先/同時に着地 | 同時(対称踏切) | 同時(対称踏切) |
| REAR_PUSH 中に後脚が主荷重 | 未(対称踏切のため REAR_PUSH 無し) | 未 |

**未達**：後脚だけの踏切(`REAR_PUSH`)・前脚だけの着地(`FRONT_LAND`)は、
姿勢発散を避けるため現状 `preload_fraction=1.0` / `front_land_fraction=0.0` で
**四脚対称のホップ**にしている。REAR_PUSH を実際に効かせるには、GBP 側に姿勢
(ピッチ)の基準と後脚接地点まわりのモーメント拘束(問題 C)が要る。

---

## 6. 変更ファイル(コミット)

| commit | 内容 |
|---|---|
| `6fef3f4` | primitive ID 単一定義元、`computeContactSchedule` のデッドコード差し替え、Stage 1 テスト |
| `a300535` | `jump_mode`、`Action.is_jump`、サブフェーズ・スタンプ、前向き踏切(乱数廃止) |
| `9bdf97d` | 計測ハーネス(CSV ロガー + run スクリプト) |
| `63b3cf8` | **決定論的 1 回ジャンプ経路**(`forcedJumpSpinOnce` / `buildForcedJumpPlan`)、`step17_analyze.py`、その場ジャンプ計測 |
| (本コミット) | 前方 GRF 成分での前進、`fwd_b` 計測、本レポート |

---

## 7. 残課題

1. **REAR_PUSH / FRONT_LAND を実際に効かせる**:GBP 側の姿勢基準 + ピッチモーメント拘束(問題 C)。現状は四脚対称ホップ。
2. **前進距離をさらに伸ばす**:`fwd_a`(vx0=0.8)は前進 1 m だが反転。0.3〜0.8 の間で安定上限を詰める。着地を `FRONT_LAND→SETTLE` にすると前進モーメントを受けやすい可能性。
3. **トルク瞬間上限(50 Nm)到達**:`fwd_b` で瞬間的に当たっている。dz/t_s の再調整。
4. **実測接触の本物のソース**:`state/grfs` / `state/ground_truth.feet.contact` がこのビルドで空。現状は足先 z プロキシ。
5. **穴シナリオ**(ユーザー指示で今回は対象外)。

---

## 8. 事実 / 計測 / 推測

**事実(コード・テスト)**：commit の変更内容は本文どおり。`colcon test` 112 pass。
`jump_mode` 既定 `auto` は上流挙動と一致。

**計測(2026-09-03、平地 `flat_wide`)**：
- `step17_hop_sym2`:その場ジャンプ 胴体 +0.226 m、四脚離地 264 ms、着地後 2 s の \|roll\|,\|pitch\| < 0.003 rad、NMPC 失敗 0、転倒なし。
- `step17_fwd_b`:短前方ジャンプ 後脚前進 +0.386 m、四脚離地 314 ms、着地後 2 s の \|roll\| max 0.001 / \|pitch\| max 0.003 rad、NMPC 失敗 0、転倒なし。
- `step17_hop_v0` / `step17_fwd_a`:反転して失敗(本文 4.3)。

**推測(未検証)**：
- REAR_PUSH を効かせても安定させられるか(GBP の姿勢拘束次第)。
- 前進距離の安定上限(0.4〜0.8 m のどこか)。
- 実機トルク/速度余裕。
