# Step 14: 速度制限・graceful stop への接続(opt-in)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
指示書 §5 Step 14。**ここで初めて制御パスに触れる**が、全パラメータ既定 OFF。

## 1. 背景

Step 12(複数歩足場列の `BLOCKED_AT_STEP_K`)+ Step 13(停止余裕 M 歩)が揃った。
Step 14 は shadow 判定を **既存の Phase 2B graceful stop** につなぐ。足場列を NMPC へ
渡すのは Step 15。ここでは **速度制限と停止だけ**。

## 2. 目的

- 新パラメータ(すべて既定 OFF)を opt-in したときだけ:
  - `BLOCKED_AT_STEP_K` で `k ≤ final_stop_steps` → **`STOP_REQUEST`** →
    既存 `safe_stop_latched_` を発火(plan は止めず `cmd_vel:=0` → STEP→STAND)。
  - `k > final_stop_steps` → **`SLOW`** → `cmd_vel` を `slow_factor` 倍。
- latch 理由 + first blocked step/leg をログ。
- 完了条件:
  - 50/100 cm 穴で **M 歩以上手前に直立停止、3/3 で転倒なし**。
  - 15/30/35 cm で **不要停止せず通過**。
  - feature OFF で **Step 08 の結果と一致**。

## 3. 変更計画(制御コード)

| ファイル | 変更 | 既定 OFF の担保 |
|---|---|---|
| `local_planner.yaml` | `local_planner.multistep_planner:` ブロック新設(`enabled:false`, `apply_stop_request:false`, `stop_margin_steps:4`, `planning_distance:2.5`, `slow_factor:0.4`) | 既定で shadow も走らない・制御影響ゼロ |
| `local_footstep_planner.{hpp,cpp}` | `setMultistepParams()`。`step12PlanSequence` を `Step12Result` 返しに(CSV は dir 空なら書かない)。`computeFootPlan` で `enabled || dump_env` のとき探索、`apply_stop_request` のとき `final_stop_steps` を計算して `FootPlanResult` に `multistep_stop_request` / `multistep_slow` を立てる | `enabled:false` かつ dump_env 無しで `step12PlanSequence` を呼ばない。`FootPlanResult` の新フィールドは既定値のまま |
| `local_planner.{hpp,cpp}` | 5 パラメータ load、`setMultistepParams` 呼び出し。`computeLocalPlan` で `multistep_stop_request` → `safe_stop_latched_ = true`(Phase 2B と同じ経路)、`multistep_slow` → `multistep_slow_active_`。`getReference` で `multistep_slow_active_ && !latched` のとき `cmd_vel_ *= slow_factor` | すべて `if (multistep_apply_stop_)` の中。false なら 1 行も実行されない |

`final_stop_steps` の式(C++、Step 13 の同定値を埋め込み):
`d_stop = max(v·0.19 + v²/(2·0.44), 0.12) + 0.10`、
`required = ceil(d_stop / (v·period·dt/4))`、
`final_stop_steps = max(stop_margin_steps, required)`。

## 4. 事実 / 推測 / 未確認

- 事実:§8 の実測(11 run)。
- 近似:`d_stop` は Step 13 の 2 点同定(v=0.15 は歩行が続かず不採用)を外挿。
- shadow 探索は幅 1 貪欲・等速直進投影(Step 12 の限界を継承)。

## 5. 実験条件

- ON = `multistep_planner.enabled:=true` + `apply_stop_request:=true`
  (`edge_clearance` は 0 のまま = **多歩プランナ単独の効果**を見る)。
- 50/100 cm(`flat_trench_s09_{50,100}`、spawn x=−2.0)を各 3 回。
- 15 cm 連続 / 30 cm(`flat_gaps_2m`) / 35 cm を 1 回ずつ(不要停止の確認)。
- feature OFF(既定)で 30 cm(`flat_gaps_2m`)・50 cm を 1 回ずつ(Step 08 一致)。

## 6. 変更ファイルと変更理由

- Phase 2B の latch(`safe_stop_latched_`)を再利用 → 新しい停止経路を作らない
  (「plan の publish を突然止めない」「遊脚を着地させ全脚接地後 STAND」を満たす)。
- `SLOW` は `cmd_vel` スケールのみ(latch とは独立、re-plan は毎周期継続)。

## 7. 入出力・単位・座標系

- `run.log` の `[multistep-stop] latching graceful stop: ... blocked at step k=.. (leg=..)`
- `state_log.csv` の `base_pos_x_m` / `base_pos_z_m` / `base_roll_rad` で停止位置と直立を確認。

## 8. 試行結果

`scripts/trial/step14_measure.sh`(spawn x=−2.0、v=0.3 m/s、`edge_clearance` は 0
のまま = 多歩プランナ単独の効果)。全 11 run。集計は `scripts/trial/step14_analyze.py`。

![Step14 停止位置](../../artifacts/step14/step14_stop_position.png)

| シナリオ | mode | 判定 | 停止/最終 x | 空洞縁までの余裕 | roll | min z | mstop | slow |
|---|---|---|---:|---:|---:|---:|---:|---:|
| 50 cm ON #1 | ON | SAFE-STOP | 1.02 | 0.98 m | +0.00 | 0.287 | 1 | 11 |
| 50 cm ON #2 | ON | SAFE-STOP | 1.07 | 0.93 m | +0.00 | 0.305 | 1 | 12 |
| 50 cm ON #3 | ON | SAFE-STOP | 1.05 | 0.95 m | −0.01 | 0.307 | 1 | 11 |
| 100 cm ON #1 | ON | SAFE-STOP | 1.04 | 0.96 m | −0.00 | 0.283 | 1 | 12 |
| 100 cm ON #2 | ON | SAFE-STOP | 1.04 | 0.96 m | +0.00 | 0.305 | 1 | 12 |
| 100 cm ON #3 | ON | SAFE-STOP | 1.04 | 0.96 m | +0.00 | 0.288 | 1 | 12 |
| 35 cm ON | ON | CROSSED | 5.43 | − | +0.00 | 0.305 | 0 | 0 |
| 30 cm ON（flat_gaps_2m） | ON | CROSSED | 8.02 | − | −0.00 | 0.294 | 0 | 0 |
| 15 cm ×3 ON | ON | CROSSED | 6.88 | − | −0.00 | 0.305 | 0 | 0 |
| 30 cm OFF | OFF | CROSSED | 8.35 | − | −0.00 | 0.304 | 0 | 0 |
| 50 cm OFF | OFF | FELL | 2.09 | − | +0.03 | −0.642 | 0 | 0 |

（空洞の手前の縁は全 `flat_trench_s09_*` 世界で x=2.00 m。`min z` は sim_time>12 s
の胴体 z 最小値。`mstop` = `[multistep-stop] latching` 行数、`slow` = `SLOW` 行数。）

**読み取り**

- **50/100 cm ON:6/6 が直立 SAFE-STOP**。停止 x = 1.02〜1.07 m、空洞の縁まで
  **0.93〜0.98 m の余裕**。roll ≤ 0.01 rad、sim 中の最小 z ≥ 0.28 m(起立姿勢を保持)。
  latch は毎回 1 回だけ発火し、その前に `SLOW`(遠い block)が 11〜12 回出ている
  → 「遠くで減速 → 近づいて停止」の 2 段構えが設計どおり動いた。
- **15/30/35 cm ON:3/3 通過、`mstop`=`slow`=0**。渡れる穴で不要な減速・停止を
  一切出さない(Step 12 / Step 13 で `flat_gaps_2m` の 30 cm が保守的に BLOCK して
  いた回帰は、§9 の NaN 帯判定への差し替えで解消)。
- **feature OFF:30 cm 通過 / 50 cm 転倒** = Step 08 と一致。新パラメータ既定 OFF で
  制御パスに変化なし。

**完了条件の判定**

- [x] 50/100 cm で M 歩以上手前に直立停止、3/3 転倒なし(6/6 SAFE-STOP、余裕 ≈0.95 m)。
- [x] 15/30/35 cm で不要停止せず通過(3/3 CROSSED、停止要求ゼロ)。
- [x] feature OFF が Step 08 と一致(30 cm 通過 / 50 cm 転倒)。

## 9. 失敗原因

制御パスに触れる最初の Step なので、既存挙動を壊す方向の失敗を 3 回出して潰した。
いずれも「shadow の判定条件」または「停止指令の出し方」の作り込み不足で、
Phase 2B の latch 経路そのものは最後まで無変更。

### 9.1 NaN 帯ではなく `traversability` 候補数で block していた(30/35 cm を誤停止)

最初の `step12PlanSequence` は「その着地脚のまわりに `traversability > 0.6` の
セルが何個あるか」で block を決めていた。穴の縁では `InpaintFilter` のぼかしで
安全セルが数個しか残らないため、**幅 0.3 m 前後の渡れる穴でも候補数が閾値を割り、
BLOCKED_AT_STEP_K を出して不要停止**していた(Step 12 で `flat_gaps_2m` 30 cm が
60 % BLOCK、Step 14 初回で 30/35 cm が false-stop)。

- 対策:block 判定を **生 `z` 層の NaN 帯の連続幅**に差し替え。Step 09 の計測で
  「生 `z` は物理空洞の上だけ NaN で、`traversability` の縁ぼかしでは削れない」
  「幅 = 物理幅 + 2×MESH_MARGIN」が分かっていた。`uncrossable_nan_width = 0.52 m`
  未満なら block しない(15 cm→0.25、step03/04 の 30 cm→0.50、35 cm→0.45、
  50 cm→0.60、100 cm→1.10 m)。→ ≤35 cm は通過、≥50 cm は block。

### 9.2 `SLOW` を毎周期掛けて実質停止していた(50 cm で x=0.45 に張り付き)

`SLOW` は当初 `cmd_vel_ *= slow_factor(0.4)` を local planner のループ
(333 Hz)で毎回掛けていた。block が遠いあいだ `SLOW` が立ちっぱなしになり、
`0.4ⁿ` で指令速度が指数的にゼロへ潰れ、**latch 前に穴の 1.5 m 手前で止まって
しまった**(直立はしているが「M 歩手前で停止」ではなく事実上の失速)。

- 対策:`SLOW` 中は `cmd_vel` を **0.12 m/s の creep floor でクランプ**
  (`if (sp > 0.12) scale = max(slow_factor, 0.12/sp)`)。減速はするが
  這って前進は続け、block が `final_stop_steps` 以内に入って初めて latch。

### 9.3 前方走査 `scan_ahead` が長すぎて停止が早すぎた(x=0.49 で停止)

NaN 帯走査を各着地脚の hip から前方 **1.5 m** 見ていた。すると空洞が胴体から
1.5 m 以内に入った時点で、まだ足を置く余地が十分あっても k=0〜1 の近い着地脚が
空洞を「見て」しまい、`blocked_k` が常に極小 → `STOP_REQUEST` が
**空洞の約 1.5 m 手前で発火**(100 cm で x=0.49、余裕 1.5 m は過剰)。

- 対策:NaN 帯は「その着地脚の hip から 1 歩の到達距離 `R`(=`ik_max_reach` 0.45 m)
  以内で始まる」ときだけ、その脚の block とみなす。`R` より遠い空洞は後続の
  着地脚が自分の k で判定する。走査距離も `R + 帯幅 + 0.15 ≈ 1.12 m` に短縮
  (帯全幅を測れれば十分)。→ 停止 x が 0.49 → 1.04 m に改善、空洞の縁まで
  約 0.95 m の適正な余裕に収束(6/6 で ±0.03 m)。

### 9.4 NMPC 起動フレーク(結果には未計上)

3 回目の測定で 50 cm ON の 1 本が spawn(x=−2.04)で崩れて FELL になった
(`minz=0.077`、mstop=0)。これは twist gait 既知の NMPC 起動フレークで、
`scan_ahead` 修正後の 4 回目測定では 6/6 が正常起動・SAFE-STOP。
再現性のため本 Step の結果表は 4 回目測定を採用。

## 10. 後方互換性確認

- 全パラメータ既定 OFF。`enabled:false` かつ dump_env 無しで `step12PlanSequence` は
  呼ばれず、`FootPlanResult` の新フィールドは触られない。`getReference` /
  `computeLocalPlan` の追加分は `if (multistep_apply_stop_)` の中。
- feature OFF の 30 cm / 50 cm 実測を Step 08(30 cm 渡る / 50 cm 落下)と突き合わせ:§8。
- gtest **40/40 green**（`colcon test --packages-select local_planner`）。

## 11. GIF・CSV・ログ

- README には「多歩プランナ ON:50 cm / 100 cm 手前で直立停止、30 cm は通過」。

## 12. 次 Step へ進む条件

- [ ] 50/100 cm で M 歩以上手前に直立停止、3/3 転倒なし。
- [ ] 15/30/35 cm で不要停止せず通過。
- [ ] feature OFF が Step 08 と一致。
- 満たせば Step 15(計画足場列を NMPC へ接続)へ。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 14
- `agent_reports/steps/step_13_step_margin_and_stopping_distance.md`
- `agent_reports/steps/step_12_multistep_foothold_sequence_shadow.md`
