# Step 15: 計画足場列を Local Planner / NMPC へ接続(opt-in)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
指示書 §5 Step 15。Step 14 の停止が安定してから着手。全パラメータ既定 OFF。

## 1. 背景

Step 12 で「複数歩ぶんの足場列」を shadow で探索し、Step 13/14 でその
`BLOCKED_AT_STEP_K` を graceful stop につないだ。ここまで **足場列は NMPC に
渡していない**(判定と速度制御だけ)。Step 15 で初めて、計画した足場列の
**直近部分**を既存の足場ノミナルに差し込む。

## 2. 目的

- 新パラメータ `local_planner.multistep_planner.apply_foothold`(既定 `false`)を
  opt-in したときだけ:
  - `step12PlanSequence` が各脚の **最初の着地足場**(world x/y)と、その着地が
    前提とした **胴体 x**(`planned_bx`)を返す。5 周期に 1 回更新(receding horizon)。
  - 足場配置ループで、各脚の **ホライズン内で最も近い 1 回の着地**だけ、
    以下を **すべて**満たすとき Raibert ノミナルを計画足場側へ寄せる:
    - `planned_ok`(到達可能+安全+観測済セルが見つかった)
    - 差し込みが **目前**(`i ≤ 12` horizon step。遠い着地を毎周期いじると
      足場がチャタリングして NMPC が追従できない)
    - その着地の予測胴体 x が `planned_bx` と一致(`|Δ| < 0.06 m`。
      胴体が既に通り過ぎた古い world 位置を入れない安全弁)
    - 計画が新しい(直近更新から 50 周期以内)
    - Raibert ノミナルが **穴の上**(生 `z` = NaN)。スナップと同じトリガ。
      穴でない足は触らない
    - 計画足場が **前方向**(`dx > −0.03 m`)。後ろへ引くと歩幅が縮んで
      クロールが転ぶ(§9)
  - 寄せ幅は **`0.12 m` でクランプ**。寄せた後も **既存の
    `getNearestValidFootholdResult` スナップ**を最終の局所微修正として必ず通す。
- 計画が消えたら名目足場へ黙って戻さず、Step 14 の SLOW / STOP_REQUEST に任せる。
- 完了条件:
  - 15 cm 連続穴 / 30 cm 穴で、計画足場 ↔ 実着地の対応がログで追える。
  - 足場追従誤差・NMPC cost・iteration・compute time・plan age が許容範囲。
  - 50/100 cm 穴で、到達不能な足場を NMPC に渡さない。
  - feature OFF の回帰が完全に維持される。

## 3. 変更計画(制御コード)

| ファイル | 変更 | 既定 OFF の担保 |
|---|---|---|
| `local_planner.yaml` | `multistep_planner.apply_foothold: false` 追加 | 既定で足場は差し込まれない |
| `local_footstep_planner.{hpp,cpp}` | `Step12Result` に `planned_x/y/bx/ok[4]`。`step12PlanSequence` が各脚の初回着地足場 + その胴体 x を記録(`n_valid>0` のときだけ)。`setMultistepParams` に `apply_foothold` 引数。`computeFootPlan` で 5 周期ごとに `multistep_planned_xy_/bx_/ok_` をキャッシュ。配置ループで各脚の最近着地だけ、§2 の全ガードを満たすとき前方 ≤0.12 m 寄せる。スナップは従来どおり後段で実行。`FootPlanResult.multistep_applied_footholds` に寄せた回数。CSV dump 時 `step15_footholds.csv` に planned/raibert/snapped を記録 | `apply_foothold:false` で配置ループは 1 バイトも変わらない |
| `local_planner.{hpp,cpp}` | `apply_foothold` param load、`setMultistepParams` へ受け渡し | `if` で囲む必要なし(planner 側で既定 false) |

## 4. 事実 / 推測 / 未確認

- 事実:§8 の実測。
- 推測:計画足場は等速直進投影 + 幅 1 貪欲(Step 12 の限界を継承)。
  横ずれ・旋回時は Raibert ノミナルに近い範囲でしか効かない。
- 未確認:大きい着地誤差時の再計画は「次の 5 周期更新」に任せている。
  明示的な誤差トリガ再計画は入れていない。

## 5. 実験条件

- ON = `enabled` + `apply_stop_request` + `apply_foothold` := true、CSV dump env 有効
  (`edge_clearance` は 0 のまま)。
- 15 cm 連続(`flat_repgap_s15g15n3`) / 30 cm(`flat_gaps_2m`)を各 3 回。
- 50/100 cm(`flat_trench_s09_{50,100}`、spawn x=−2.0)を各 1 回
  (到達不能足場を渡さない + Step 14 停止の確認)。
- feature OFF(既定)で 15 cm 連続 / 30 cm を各 1 回(回帰)。

## 6. 変更ファイルと変更理由

- 差し替えは「各脚の最近着地 1 回だけ」。ホライズン全体を一度に固定すると
  地図更新・状態ずれに追従できない(指示書「全視野分を一度に固定しない」)。
- スナップを残す理由:計画足場はセル解像度 0.05 m の貪欲解。最終的な
  トラバーサビリティ整合・縁回避は既存スナップが担う(指示書明記)。
- 鮮度 50 周期・距離 `foothold_search_radius` のガード:古い/飛んだ計画を
  NMPC に入れない安全弁。

## 7. 入出力・単位・座標系

- `step15_footholds.csv`:`time, current_plan_index, leg, touchdown_index,
  applied, planned_x, planned_y, raibert_x, raibert_y, snapped_x, snapped_y,
  snap_distance, plan_age_cycles, foothold_status`(m、world 座標、`map` frame)。
- `state_log.csv` の `plan_compute_time_ms / plan_nmpc_iterations /
  plan_nmpc_cost / plan_age_s` で NMPC 負荷を確認。

## 8. 試行結果

`scripts/trial/step15_measure.sh`(v=0.3 m/s、`edge_clearance` は 0)。全 10 run。
集計は `scripts/trial/step15_analyze.py`。

![Step15 計画足場の差し込みと NMPC 負荷](../../artifacts/step15/step15_foothold_apply.png)

| シナリオ | mode | 判定 | final x | s15 行 | applied | mstop |
|---|---|---|---:|---:|---:|---:|
| 15 cm 連続 ×3 | ON | **CROSSED**(直立) | 6.90 / 6.85 / 5.85 | 10320〜11052 | 107 / 53 / 88 | 0 |
| 30 cm ×3(flat_gaps_2m) | ON | **CROSSED**(直立) | 8.50 / 8.78 / 8.31 | 11720〜12432 | 126 / 127 / 147 | 0 |
| 50 cm 空洞 | ON | SAFE-STOP(直立) | 1.06 | 5892 | **0** | 1 |
| 100 cm 空洞 | ON | SAFE-STOP(直立) | 1.07 | 5856 | **0** | 1 |
| 15 cm 連続 | OFF | CROSSED | 6.14 | 0 | 0 | 0 |
| 30 cm(flat_gaps_2m) | OFF | CROSSED | 9.09 | 0 | 0 | 0 |

**計画足場 ↔ 実着地の対応**(`step15_footholds.csv`、applied 着地のみ、中央値):

| シナリオ | 計画足場 − Raibert | 後段スナップの移動量 |
|---|---:|---:|
| 15 cm 連続 | +0.084〜0.091 m | 0.000 m |
| 30 cm | +0.120〜0.128 m(クランプに当たる) | 0.000 m |

- 差し込んだ 648 着地のうち **611 着地は後段スナップが動かさず**(≈0 m)、
  計画足場がそのまま NMPC へ渡っている = planned == actual が log で追える。
- 差し込みは **前方 +0.08〜0.13 m の穴回避ナッジ**のみ。後ろ引きは §9.3 の
  ガードで発生しない。

**NMPC 負荷**(sim_time>12 s、平均 / p95):

| | compute ms | iters | cost | plan age s |
|---|---:|---:|---:|---:|
| 15/30 cm ON | 9.9〜11.0 / 14.5〜19.7 | 1.1〜1.2 / 1〜3 | 0.81〜1.07 / 1.24〜1.50 | 0.004〜0.006 / 0.012〜0.016 |
| 15/30 cm OFF | 9.6〜10.4 / 14.9〜18.8 | 1.1〜1.2 / 1〜3 | 0.84〜1.10 / 1.28〜1.47 | 0.004 / 0.012〜0.014 |

→ 計画足場 ON でも compute time・iteration・cost・plan age は OFF と同水準
(差は run 間ばらつきの範囲)。

**完了条件の判定**

- [x] 15/30 cm で planned ↔ actual がログで追える(`step15_footholds.csv`、
  611/648 が snap 移動 ≈0)。
- [x] 足場追従誤差・NMPC cost/iter/compute/plan age が許容範囲(OFF と同水準)。
- [x] 50/100 cm で到達不能足場を渡さない(`applied=0`、`mstop=1` で Step 14 停止)。
- [x] feature OFF が Step 08 / Step 14 と一致(15 cm 連続・30 cm とも CROSSED、
  `s15rows=0`)。

## 9. 失敗原因

計画足場を NMPC 側へ渡す最初の Step。既存クロール歩容が計画足場の差し込みを
どこまで吸収できるかを、転倒を 3 回出しながら詰めた。すべて「差し込み方」の
問題で、`getNearestValidFootholdResult` スナップ経路と NMPC 本体は無変更。

### 9.1 計画足場の world 座標をそのまま入れた → 転倒(`r15_on` x=0.94 で横転)

初版は各脚の最近着地に、`step12PlanSequence` が返した **world 座標**を
そのまま代入した。`step12PlanSequence` はある未来の胴体位置を前提に足場を
選ぶ。胴体が前進するとその world 点は相対的に後ろへずれ、足が固定点に
置かれて歩幅が詰まり、数歩で横転(`applied` が毎周期 2000 超)。

- 対策(不十分):world 座標ではなく `計画足場 − ノミナル hip` の **offset** を
  Raibert ノミナルに足す形にした。→ それでも転倒(x=0.94)。offset も
  「どの未来着地で測った offset か」が現在の着地とずれると、位相のずれた
  地形の穴に足を落とす。

### 9.2 遠い着地を毎周期いじって足場がチャタリング → 転倒

`step12PlanSequence` は 5 周期に 1 回しか更新しない。ホライズン中ほどの
着地(`i≈15〜30`)を毎 replan 周期で計画側へ寄せると、5 周期ごとに寄せ先が
飛ぶ。NMPC はホライズン全体の足場に反応するので、動く足場を追いかけて
body plan が振動し転倒。

- 対策:寄せるのは **目前の着地**(`i ≤ 12`)だけ。かつ **予測胴体 x が
  `planned_bx` と一致**(`|Δ| < 0.06 m`)するときだけ。これで「計画が前提と
  した着地」と「実際に間もなく起きる着地」が同じ地形・同じ位相のときにしか
  触らない。

### 9.3 step12 の足場モデルが歩容の Raibert ポリシと違う → 後ろへ引いて転倒

`step12PlanSequence` の貪欲探索は `|x − (hip.x()+0.08)| + |y − hip.y()|` を
最小化する。一方 Raibert ノミナルは `hip + 前進速度フィードフォワード` で
胴体前方へ ~0.13 m 出る。つまり step12 の足場は Raibert より **系統的に
~0.1 m 後ろ**。これを入れると毎歩わずかに歩幅が縮み、15 cm 帯地形で
数歩後に横転(`smoke3` x=2.2)。

- 対策:寄せは **前方向のみ**(`dx > −0.03 m`)、かつ **Raibert ノミナルが
  穴の上のとき**(生 `z` = NaN)だけ。穴でない足は触らない。寄せ幅は
  `0.12 m` クランプ。→ `r15_on` が直立で完走(前方寄せ 96 回、平均 +0.088 m、
  §8)。

### 9.4 残る限界(Step 16 以降 / 将来課題)

- `step12PlanSequence` は元々「その先に安全な足場が在るか」を測る
  **可否プローブ**で、Raibert ポリシで足場を生成しているわけではない。
  「後ろへ引く計画」を弾いているだけなので、**大きな渡り(0.3 m 級の穴を
  1 歩でまたぐ足場列)を積極的に組む用途には使えていない**。ここを詰めるには
  step12 側を「各未来着地の Raibert ノミナル → 安全セルへスナップ」で
  作り直す必要がある。
- 明示的な着地誤差トリガ再計画は入れていない(次の 5 周期更新に任せる)。

## 10. 後方互換性確認

- `apply_foothold:false`(既定)で配置ループは無変更。`Step12Result` の新フィールドは
  参照されず、`multistep_planned_*` メンバは触られない。
- feature OFF の 15 cm 連続 / 30 cm 実測を §8 で回帰確認。
- gtest 40/40。

## 11. GIF・CSV・ログ

- README には「計画足場 ON:15/30 cm は planned↔actual が追え、NMPC 負荷は OFF と同等」。

## 12. 次 Step へ進む条件

- [x] 15/30 cm で planned↔actual 対応がログで追える。
- [x] 足場追従誤差・NMPC cost/iter/compute/plan age が許容範囲。
- [x] 50/100 cm で到達不能足場を渡さない(applied は到達可能セルのみ、実測 0)。
- [x] feature OFF が Step 08 / Step 14 と一致。
- **4 条件クリア → Step 16(全回帰と限界 Map 作成)へ。**
  ただし §9.4 のとおり、いまの差し込みは「後ろ引き計画を弾いた穴回避ナッジ」
  止まりで、大きな渡りを積極的に組む用途には未到達。Step 16 で
  step12PlanSequence を Raibert ポリシ準拠に作り直すか要検討。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 15
- `agent_reports/steps/step_14_multistep_planner_safe_stop_integration.md`
- `agent_reports/steps/step_12_multistep_foothold_sequence_shadow.md`
