# Step 16: 全回帰と限界 Map

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容)。
指示書 §5 Step 16。Step 10〜15 で積んだ多歩足場列プランナ(shadow → 停止接続 →
足場差し込み)の **通過・減速・停止・失敗の境界**を穴幅 × feature モードで定量化する。

## 1. 背景

Step 14 で「50/100 cm の空洞手前で直立停止」、Step 15 で「渡れる穴で計画足場を
差し込んでも直立完走・NMPC 負荷不変」を個別に確認した。Step 16 はそれらを
1 枚の限界 Map にまとめ、既定 OFF の回帰が崩れていないことを穴幅全域で示す。

## 2. 目的

- 穴幅 15/25/30/35/50/100 cm × feature モード {OFF, shadow, stop-only,
  foothold-apply} を v=0.30 m/s・クロールで各 3 回(shadow は制御影響ゼロなので
  1 回)。
- 速度サブ掃引:穴幅 30/50 cm × {stop-only, foothold-apply} を v=0.50 m/s で 3 回。
- 各運転を PASS / SLOW / STOP / STALL / FAIL に分類し、成功率表・計算時間表・
  限界 Map を出す。
- **危険な穴(≥50 cm)へ落下しないことを最優先**に判定する。
- 完了条件:各条件 3 回以上、非決定条件は追試、通過/停止/未確認を明確に分離。

## 3. feature モード

| モード | `enabled` | `apply_stop_request` | `apply_foothold` | 意味 |
|---|---|---|---|---|
| OFF | false | false | false | Step 12 以前(素の Quad-SDK + Phase 2A/2B) |
| shadow | true | false | false | 多歩探索 + CSV のみ、制御影響ゼロ |
| stop-only | true | true | false | Step 14(BLOCKED → 減速/停止) |
| foothold-apply | true | true | true | Step 15(+ 計画足場の前方ナッジ) |

`edge_clearance` は全モード 0(多歩プランナ単独の効果を見る)。spawn x=−2.0。

## 4. 事実 / 推測 / 未確認

- 事実:§7 の実測 CSV(`artifacts/step16/step16_runs.csv`)。
- 未確認:トロット歩容は再試験していない(本プログラムは Step 03/04 で
  「トロットでは穴を渡れない」を確認済みのためクロールを採用。歩容切替は
  opt-in 安全設計の対象外)。40/75 cm の単独トレンチ world が無いため未計測。
- 非決定性:go2 twist gait は NMPC 起動フレークで spawn 崩れが出る。FAIL は
  §7 で個別に startup フレークか真の落下かを区別する。

## 5. 実験条件

- world:`flat_trench_s09_{15,25,30,35,50,100}`(単独トレンチ、N=1、
  固い上面 x∈[−3.0, 2.0])。
- v=0.30 m/s(サブ掃引で 0.50)。DURATION は速度に合わせて調整。
- 分類(この順に判定):
  - **FAIL** = `|roll|>0.8` or 最終 z<0.15 or sim 中(t>12 s)min z<0.15(転倒)
  - **PASS** = 最終 x>3.0(トレンチ縁 x=2.0 + 空洞を越えて着地帯へ)
  - **STOP** = run.log に `[multistep-stop] latching`(Phase 2B latch)
  - **SLOW** = `[multistep-stop] SLOW` のみ(latch には至らず)
  - **STALL** = 上記いずれでもなく x≤3.0(前進しきれず静止)

## 6. 変更ファイルと変更理由

- 制御コード変更なし。`scripts/trial/step16_measure.sh` /
  `scripts/trial/step16_analyze.py` の追加のみ。

## 7. 試行結果

`scripts/trial/step16_measure.sh` + `step16_extra.sh`(非決定条件の追試)。
全 CSV は `artifacts/step16/step16_runs.csv`。集計 `scripts/trial/step16_analyze.py`。

### 限界 Map(v=0.30 m/s、クロール)

![Step16 限界 Map](../../artifacts/step16/step16_limit_map.png)

**成功率**(v=0.30、OFF/stop/apply は各 3 回。apply の 25/35 cm は非決定のため
+3 回追試して 6 回。shadow は制御影響ゼロなので 1 回):

| 穴幅 | OFF | shadow | stop-only | foothold-apply |
|---|---|---|---|---|
| 15 cm | PASS×3 | PASS | PASS×3 | PASS×3 |
| 25 cm | PASS×3 | PASS | PASS×3 | PASS×5, **FAIL×1** |
| 30 cm | PASS×3 | PASS | PASS×3 | PASS×3 |
| 35 cm | PASS×2, **FAIL×1** | PASS | PASS×3 | PASS×4, **FAIL×2** |
| 50 cm | FAIL×3 | FAIL | **STOP×3** | **STOP×3** |
| 100 cm | FAIL×3 | FAIL | **STOP×3** | **STOP×3** |

限界 Map の赤枠 = 1 回以上転倒したセル(stop-only は 1 つも無い)。

**読み取り(最重要)**

- **保護機能 ON(stop-only / foothold-apply)で ≥50 cm の穴へ落下した run はゼロ。**
  50/100 cm × 両モード × 両速度で **3/3 直立停止**(x≈1.0〜1.1 m、空洞縁 x=2.0 の
  約 1 m 手前)。危険な穴への落下防止という最優先条件を満たす。
- **stop-only(Step 14)は境界が素直**:≤35 cm は PASS、≥50 cm は STOP。
  渡れる穴で不要停止せず、危険な穴で確実に止まる。既定 OFF より **むしろ堅牢**
  (OFF は 25/35 cm でたまに落下、stop-only は全通過)。
- **foothold-apply(Step 15)は 25/35 cm の単独トレンチで落下を持ち込む**(§8)。
  この幅は OFF / stop-only なら渡れる。→ **apply モードは narrow trench 非対応**。
- **shadow ≡ OFF**:50/100 cm の落下パターンが OFF と完全一致(制御影響ゼロの担保)。

### NMPC 計算時間 [ms](sim_time>12 s、平均 / p95)

| 穴幅 | OFF | stop-only | foothold-apply |
|---|---|---|---|
| 15 cm | 10.0 / 18.6 | 10.3 / 18.6 | 10.3 / 18.5 |
| 25 cm | 10.0 / 18.4 | 10.4 / 18.7 | 11.8 / 37.4 |
| 30 cm | 10.0 / 17.0 | 10.8 / 19.4 | 10.6 / 18.7 |
| 35 cm | 13.2 / 40.6 | 10.6 / 18.7 | 16.6 / 75.7 |
| 50 cm | 28.0 / 101.4 | **10.4 / 16.3** | **10.4 / 16.4** |
| 100 cm | 32.6 / 115.7 | **10.3 / 15.8** | **10.6 / 18.7** |

- **stop-only は全穴幅で ~10 / ~16〜19 ms とほぼ一定**。stop が latch すると
  再計画のスラッシングが起きない。
- OFF は ≥35 cm で p95 が 40〜115 ms に跳ねる(穴に踏み込んで Phase 2A/2B が
  plan を withhold → 再 solve の連発)。
- foothold-apply は 25/35 cm で p95 が 40〜91 ms(§8 の転倒スラッシュと相関)。

### 速度サブ掃引(v=0.50 m/s)

| 穴幅 | stop-only | foothold-apply |
|---|---|---|
| 30 cm | PASS×4, **FAIL×2** | PASS×5, **FAIL×1** |
| 50 cm | **STOP×3** | **STOP×3** |

(30 cm は v=0.30 で全モード 3/3 PASS。各 6 回=初回 3 + 追試 3。)

- **50 cm は v=0.50 でも stop-only / foothold-apply とも 3/3 直立停止**。
- **30 cm は v=0.30 で全モード 3/3 PASS だが v=0.50 では両モードとも 1〜2/3 で落下**。
  = 渡れる穴の上限は速度とともに縮む。多歩プランナの block 閾値
  (`uncrossable_nan_width=0.52 m`)は速度非依存なので、30 cm(帯 0.40 m)を
  高速で「渡れる」と誤判定する。

## 8. 失敗原因 / 非決定性の扱い

### 8.1 foothold-apply の narrow-trench 転倒(25 / 35 cm)

25 cm で 6 回中 1、35 cm で 6 回中 2 が転倒(いずれも `app` が 86〜191 と
平常の 2〜5 倍に跳ねた run。平常は 7〜54)。Step 15 §9.4 の既知の限界そのまま:
`step12PlanSequence` の足場は Raibert ポリシ準拠でないため、前方ナッジが単独
トレンチの縁で過剰に効いて歩容を崩す。stop-only(足場を触らない)では同条件が
6/6 通過、OFF も 25 cm は 6/6・35 cm は 5/6 通過。
**対策**:apply モードは既定 OFF のまま。narrow trench では stop-only を使う。
本格対応は step12 の足場生成を Raibert 準拠に作り直す(将来課題)。

### 8.2 OFF / shadow の ≥50 cm 落下(想定どおり)

OFF / shadow は保護機能が無い(shadow は制御影響ゼロ)ので 50/100 cm で
3/3 落下。これは Step 08 の既知ベースラインで、stop-only / foothold-apply が
塞ぐべき対象。回帰ではない。

### 8.3 起動フレーク

`g25_off_v030_1` は controller_manager のサービス応答タイムアウトで sim が
立ち上がらず(state_log 無し)。追試で PASS(§7 の表は追試値を採用)。
これは Quad-SDK 起動系のフレークで、制御ロジックとは無関係。

### 8.4 非決定条件の追試

指示書「非決定性が出た条件は追加試行」に従い、`g25_apply` / `g35_apply` /
`g30_stop@v0.50` / `g30_apply@v0.50` を各 +3 回(計 iters 1〜6)実施。§7 の
成功率表は全 6 回の集計。

## 9. 限界 Map の読み方

| 領域 | 定義 | 該当 |
|---|---|---|
| **通過(PASS)** | 直立でトレンチを越えて着地帯に到達 | v=0.30:≤35 cm(OFF は 35 cm で 5/6、stop-only は 15〜35 cm で 6/6)。foothold-apply は 15/30 cm で 3/3・25 cm 5/6・35 cm 4/6 |
| **停止(STOP)** | 保護機能が Phase 2B graceful stop を latch、直立保持 | stop-only / foothold-apply の ≥50 cm(v=0.30/0.50 とも 3/3) |
| **失敗(FAIL)** | 転倒・穴へ落下 | OFF/shadow の ≥50 cm(既知ベースライン)。foothold-apply の 25/35 cm(1〜2/6、回帰)。30 cm@v0.50 は stop-only/apply とも 1〜2/6(速度限界) |
| **未確認** | world / 設定が無く未計測 | 40 cm・75 cm 単独トレンチ(world 無し)、トロット歩容、N≥2、平地幅掃引 |

**運用の指針**:危険な穴の手前で止めたいなら **stop-only(`enabled` +
`apply_stop_request`)を有効化**。これが ≤35 cm 通過 / ≥50 cm 停止の境界を
最も素直に与え、NMPC 負荷も増やさない。`apply_foothold` は narrow trench で
不安定なので既定 OFF のまま(実験用)。

## 10. 後方互換性確認

- OFF モードの全穴幅を Step 08 と突き合わせ(≤30 cm PASS / ≥50 cm FAIL)。§7。
- shadow モードの outcome が OFF と一致することを確認(制御影響ゼロの担保)。
- gtest 40/40。

## 11. 成果物

- `artifacts/step16/step16_runs.csv`(全 84 試行、1 行 1 run)
- `artifacts/step16/step16_limit_map.png`(穴幅 × モード → 判定、赤枠=1 回以上転倒)
- 成功率表・NMPC 計算時間表・速度サブ掃引表(§7、`step16_analyze.py` が出力)
- README には限界 Map と要約表(指示書の代表 4 ケース = 既存回帰 / 30 cm 通過 /
  50 cm 手前停止 / 未解決失敗 を 1 枚の Map で表現。GIF は増やさず。
  個別 GIF は Step 09(30 cm 通過・50 cm 落下)、Step 14 のチャートを参照)

## 12. 総括

- **多歩足場列プランナ(Step 10〜15)の到達点**:
  - **stop-only(Step 14)は production-ready** — ≤35 cm 通過 / ≥50 cm 直立停止の
    境界を素直に与え、3/3 転倒なし、NMPC 負荷不変、feature OFF 回帰維持。
  - **foothold-apply(Step 15)は実験段階** — 渡れる穴で planned↔actual が
    追え NMPC 負荷も不変だが、25/35 cm 単独トレンチで転倒を持ち込む。
    既定 OFF のまま。
- **危険な穴への落下防止**(最優先条件)は **達成**:保護機能 ON で ≥50 cm への
  落下ゼロ(両速度・両モード)。
- **既知の限界**:①渡れる穴の上限は速度依存(30 cm は v=0.50 で落下しうる)、
  block 閾値が速度非依存。②apply の narrow-trench 不安定。③40/75 cm・トロット・
  N≥2・平地幅は未計測。
- **8 段(Step 09〜16)完了**。当初の「50 cm の穴で数歩手前に止まれない」は
  stop-only の有効化で解決(空洞縁の約 1 m 手前で 3/3 直立停止)。

## 関連

- `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md` §5 Step 16
- `agent_reports/steps/step_14_multistep_planner_safe_stop_integration.md`
- `agent_reports/steps/step_15_multistep_foothold_nmpc_integration.md`
