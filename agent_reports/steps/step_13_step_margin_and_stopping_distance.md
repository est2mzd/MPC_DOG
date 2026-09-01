# Step 13: 停止余裕 M 歩の推定と shadow 判定

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容)。指示書 §5 Step 13。
**制御は変更しない**(計測 + 後処理のみ)。

## 1. 背景

Step 12 で「k 歩目で足場列が破綻する(`BLOCKED_AT_STEP_K`)」が取れた。Step 13 は
「では **何歩手前で** `STOP_REQUEST` を出せば安全に止まれるか」を、実測した停止距離
から決める。

## 2. 目的

1. 平地で v = 0.15 / 0.30 / 0.50 m/s から Phase 2B の graceful stop を発火させ、
   **latch 発火 → 速度 5 % 以下 → 全脚接地 → STAND 安定** までの時間と距離を測る。
2. 保守的な `a_safe`(減速度)と `t_delay`(地図更新+計画+latch+歩容遷移の遅れ)を
   `d_stop = v²/(2·a_safe) + v·t_delay + distance_margin` から同定。
3. `required_stop_steps = ceil(d_stop / conservative_step_progress)`、
   `final_stop_steps = max(stop_margin_steps, required_stop_steps)`。
4. Step 12 の `BLOCKED_AT_STEP_K` に対し `k ≤ final_stop_steps` なら **shadow で
   `STOP_REQUEST`**。実際に止めていたらどこ(x)で止まったかを表示。

## 3. 変更前のコード経路 / 使うもの

| 使うもの | 出所 |
|---|---|
| latch 発火時刻 | `run.log` の `[safe-stop] latching graceful stop` |
| 速度・位置 | `state_log.csv` の `base_lin_vel_x_mps` / `base_pos_x_m` |
| 全脚接地 | `contact_FL/BL/FR/BR` がすべて True |
| touchdown 間隔 | `period·dt/4` ≈ 0.225 s(Step 10/12) |
| Step 12 の判定 | `artifacts/step12/*/step12_sequence.csv` |

## 4. 事実 / 推測 / 未確認

- 事実:§9 の停止距離実測・後処理。
- 近似:`conservative_step_progress = v · 0.225`(1 touchdown ぶんの胴体前進)。
  `distance_margin` は latch 位置ばらつき + 0.1 m。
- 未確認:`a_safe` は 3 速度の実測から下側(保守)を取る。速度域外は外挿。

## 5. 変更計画

| ファイル | 追加内容 | 制御影響 |
|---|---|---|
| `scripts/trial/step13_measure.sh` | `flat_trench_1m` + `edge_clearance:=0.15` で v=0.15/0.30/0.50 の停止試験(yaml は cp バックアップ→trap 復元) | なし(既存の Phase 2B を発火させて測るだけ) |
| `scripts/trial/step13_analyze.py` | 停止距離の同定、Step 12 verdict と突き合わせた shadow `STOP_REQUEST` 判定、図 | なし(後処理) |

**C++ の計装追加はなし。** Step 12 の CSV + Step 13 の停止試験だけで後処理できる。

## 6. 入出力・単位・座標系

- `artifacts/step13/{v015,v030,v050}/state_log.csv` + `latch.txt`
- 後処理出力:v→`d_stop` 表(実測 + fit)、gap 地形ごとの `STOP_REQUEST` 予定 x と
  物理縁(x=2.0)との差、図。

## 7. 実験条件

- `flat_trench_1m`、`edge_clearance:=0.15`、spawn x=−2.0(速度に乗ってから latch)。
- v = 0.15 / 0.30 / 0.50 m/s(0.15 は前進が続かず不採用)。

## 8. 試行結果

### 8.1 停止距離の同定(平地、`edge_clearance:=0.15`、latch → 停止)

| v_cmd | v_cruise | t_delay | a_safe | d_decel | **d_stop** | t_decel |
|---:|---:|---:|---:|---:|---:|---:|
| 0.30 m/s | 0.264 | 0.01 s | 0.51 m/s² | 0.090 m | **0.092 m** | 0.52 s |
| 0.50 m/s | 0.450 | 0.19 s | 0.44 m/s² | 0.036 m | **0.118 m** | 1.03 s |

- **d_stop は v とともに増える**(0.092 → 0.118 m)。✅
- v=0.15 m/s は**このsim設定で前進歩行が続かない**(`v_cruise` が 0、後方ドリフト)
  ため計測不能 → 2 点で同定。保守値:`a_safe = 0.44 m/s²`、`t_delay = 0.19 s`。
- **latch はもともと穴の ~1.9 m 手前で発火**(`safe_stop_lookahead 2.5 − max_crossable_gap 0.6`)。
  latch 後の**物理減速は 0.09〜0.12 m と非常に短い**(足を全部着いて止まるだけ)。

### 8.2 M 歩換算

`conservative_step_progress(v) = v·0.225`。保守 d_stop モデル(物理式を実測下限で
床止め + 0.10 m 余裕)で:

| v | d_stop(保守) | step_progress | required_stop_steps | final_stop_steps(M=2) |
|---:|---:|---:|---:|---:|
| 0.15 | 0.22 m | 0.034 m | 7 | 7 |
| 0.30 | 0.26 m | 0.068 m | 4 | 4 |
| 0.50 | 0.48 m | 0.113 m | 5 | 5 |

`required_stop_steps ≥ 4` なので `stop_margin_steps`(M=2)は効かず、
**final_stop_steps は 4〜7 歩**(速度依存)。

### 8.3 Step 12 の判定に M 歩マージンを当てた shadow 挙動(v=0.30、final=4)

| 地形 | FEASIBLE | SLOW(k>4) | STOP_REQUEST(k≤4) |
|---|---:|---:|---:|
| 平地 | 100 % | 0 % | **0 %** |
| 連続 15 cm | 87 % | 0 % | **0 %** |
| 30 cm 単独(`t30`) | 72 % | 14 % | **0 %** |
| 30 cm `flat_gaps_2m` | 39 % | 57 % | 2 %(保守・Step 12 §10) |
| **50 cm** | 45 % | 34 % | **5 %** |
| **100 cm** | 45 % | 35 % | **5 %** |

- **50/100 cm**:遠いうちは `SLOW`、穴が final_stop_steps(4 歩)以内に入ると
  `STOP_REQUEST`。図(README)。
- **平地・15 cm・30 cm 単独**:`STOP_REQUEST` **0 %**(不要な停止要求を出さない)。✅
- ただし `STOP_REQUEST` 発火点(block の 4 歩手前)から block までの**マージンは
  +0.01 m と薄い**。物理 d_stop が小さい分、M 歩マージン ≈ ユーザー設定 M で決まる。

## 9. 失敗原因 / 限界

- **v=0.15 m/s の停止試験ができない**(前進歩行が続かない)。3 点目は 0.20〜0.25 m/s で
  取り直す余地あり。
- **STOP_REQUEST 点のマージンが薄い**(+0.01 m)。原因:latch 後の物理 d_stop が
  0.1 m と小さく、`required_stop_steps` も小さめ。対策:
  - `stop_margin_steps` M を 2 → **4〜6** に上げる(0.3 m/s で ~0.3 m の余裕)。
  - 幅の広い穴は既存の `safe_stop_lookahead`(1.9 m 手前で latch)が先に効くので、
    M 歩マージンが要るのは **0.4〜0.9 m の中サイズ穴**(Step 12 で BLOCKED になる帯)。
- `flat_gaps_2m` の SLOW 57 % は Step 12 の保守性(MESH_MARGIN 0.10)がそのまま伝播。

## 10. 後方互換性確認

- yaml は cp バックアップ → trap で必ず復元(`git checkout` 不使用)。C++ 無変更。
  gtest は Step 12 時点の **41/41 green** のまま。

## 11. GIF・CSV・ログ

- 速度別停止距離表、停止位置を表示した図。
- README には「速度 → 必要停止距離・歩数」の図 + 「50/100 cm で物理縁より十分手前に
  STOP_REQUEST / 30 cm では出さない」。

## 12. 次 Step へ進む条件

- [x] 速度が上がるほど必要停止距離・歩数が増える(d_stop 0.092→0.118 m、steps 4→7)。
- [~] 50/100 cm 穴で `STOP_REQUEST` は出る(SLOW → STOP_REQUEST)。ただしマージンは薄く、M を上げるか `safe_stop_lookahead` 併用が要る(§9)。
- [x] 平地・15 cm・30 cm 単独で `STOP_REQUEST` 0 %(不要な停止を出さない)。
- → Step 14(速度制限・graceful stop への接続、opt-in)へ。M の既定は Step 14 で 4〜6 を検討。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 13
- `agent_reports/steps/step_12_multistep_foothold_sequence_shadow.md`
