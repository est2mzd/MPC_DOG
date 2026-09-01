# Step 08:穴の数・間隔・幅を変えた全シナリオ一括検証(回帰スイープ)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
現行コード = Phase 2A(gate)+ Phase 3(A)(`EDGE_TOO_CLOSE` + 進行方向プローブ)+
Phase 2B(graceful-stop latch + 前方 lookahead)+ Phase 4(`IK_UNREACHABLE`、既定 OFF)。

## 背景

Phase 2A〜4 を段階的に足してきたが、各フェーズは「その回のシナリオ」でしか
確かめていなかった。**穴の数(N)・平地の幅(strip)・穴の幅(gap)を振った
地形すべて**に対して、現行コードが

- **渡れる穴は渡り切る**(既存の跨ぎ挙動を壊していない)
- **渡れない穴は手前で直立停止**(転倒しない)

を満たすか、まとめて回帰確認する。

## 目的

18 本のシナリオを 1 本ずつ実行し、`渡り切る / 手前で安全停止 / 転倒` を
1 行に集計。想定と食い違うものは個別にログを追う。

| グループ | 振ったもの |
|---|---|
| step03/04 ベースライン(`edge_clearance:0`) | 0.3 m 穴、間隔 2.0 / 1.5 m |
| 繰り返し 15 cm strip / 15 cm gap、N 可変(`ec:0.15`) | N = 2, 3, 4, 5, 6 |
| 繰り返し gap15、strip 可変、N=2 | strip = 25 / 35 / 50 cm |
| 繰り返し strip25、gap 可変、N=2 | gap = 25 / 35 / 50 cm |
| 単独トレンチ、幅可変(`ec:0.15`) | 30 / 100 / 1000 cm |
| 混合:15/15 ×2 → 最後だけ拡大 | → 50 cm / → 100 cm |

`edge_clearance` は step03/04 だけ 0、他は 0.15(この機能の opt-in 値)。
他パラメータは既定(`max_crossable_gap:0.6`、`ik_reach_check:false`、latch 系 true)。

## 結論

**16/18 は期待どおり。2/18(幅 50 cm の穴)で「渡れず・止まらず・転倒」**。

| シナリオ | ec | 結果 | 期待 | 備考 |
|---|---|---|---|---|
| step03 0.3 m 間隔2.0 | 0 | CROSSED (x≈11.2) | ✓ | 回帰なし |
| step04 0.3 m 間隔1.5 | 0 | CROSSED (x≈11.3) | ✓ | 回帰なし |
| 繰り返し 15/15 N=2 | .15 | CROSSED (x≈8.0) | ✓ | ※末尾 latch(下記 B) |
| 繰り返し 15/15 N=3 | .15 | CROSSED (x≈8.3) | ✓ | ※B |
| 繰り返し 15/15 N=4 | .15 | CROSSED (x≈8.4) | ✓ | ※B |
| 繰り返し 15/15 N=5 | .15 | CROSSED (x≈8.2) | ✓ | ※B |
| 繰り返し 15/15 N=6 | .15 | CROSSED (x≈8.9) | ✓ | ※B |
| strip25 / gap15 N=2 | .15 | CROSSED (x≈8.2) | ✓ | ※B |
| strip35 / gap15 N=2 | .15 | CROSSED (x≈8.0) | ✓ | ※B |
| strip50 / gap15 N=2 | .15 | CROSSED (x≈8.7) | ✓ | ※B |
| strip25 / gap25 N=2 | .15 | CROSSED (x≈8.4) | ✓ | ※B |
| strip25 / gap35 N=2 | .15 | CROSSED (x≈7.7) | ✓ | go2 が実際に跨げる最大級 ※B |
| **strip25 / gap50 N=2** | .15 | **FELL** (x≈2.4, roll→−π) | **✗** | **A:止まらず落下** |
| 単独 30 cm | .15 | CROSSED (x≈11.3) | ✓ | latch なし(地形が長い) |
| 単独 100 cm | .15 | SAFE-STOP (x≈0.2) | ✓ | 前方プローブ |
| 単独 1000 cm | .15 | SAFE-STOP (x≈0.3) | ✓ | 前方プローブ |
| **15/15 ×2 → 50 cm** | .15 | **FELL** (x≈3.0, roll→π) | **✗** | **A:止まらず落下** |
| 15/15 ×2 → 100 cm(Step 06) | .15 | SAFE-STOP (x≈0.9) | ✓ | 前方プローブ |

### 発見 A(本物の穴):幅 0.35〜0.60 m の穴が「渡れず・止まらず」

`max_crossable_gap = 0.6`(m)は「穴の 0.6 m 以内に地面が戻れば渡れる穴とみなし、
止めない」という設定。ところが go2(Raibert + クロール、0.3 m/s)が実際に
跨げるのは **約 0.35 m** まで:

- 0.35 m の穴 → 渡り切る
- 0.50 m の穴 → **踏み込んで落下**(ログに `EDGE_TOO_CLOSE` も latch も 0 件。
  安全層が一度も働いていない)

→ **幅が 0.35〜0.60 m の穴は、コードは「渡れる」と判定して止めないが、
ロボットは渡れない**。この帯が抜け穴になっている。

**対策候補**(いずれも既定 `edge_clearance:0` は不変 = opt-in 時のみ影響):

1. `max_crossable_gap` を 0.6 → **0.40 前後に下げる**。0.30 m(step03/04)は
   「渡れる」のまま、0.45 m 以上は `EDGE_TOO_CLOSE` → 手前で安全停止。**推奨。**
2. 現状維持。0.6 m は「オペレータ責任」と割り切って文書化。

→ どちらにするかはユーザー判断待ち(この Step では**コード未変更**)。

### 発見 B(テスト地形の副作用、コードのバグではない):末尾 latch

繰り返し穴シナリオの CROSSED 全 11 本で、`impassable gap in the horizon
(nearest_idx=39 status=1)` の latch が **1 回だけ**出る。

- `status=1` = `NOMINAL_OUTSIDE_MAP`、`nearest_idx=39` = NMPC ホライズン末端
  (`safe_stop_horizon=40` のちょうど境界)。
- 発火時刻はいずれも**穴を渡り切った後**(穴群は x≈2.6 で終わるのに、latch は
  x≈8、走行終盤)。
- 繰り返し穴地形は着地側のマップが短く、ロボットが地形マップの端に近づくと
  ホライズン末端の足場がマップ外へ出る → latch。単独トレンチ・step03/04 は
  滑走路が長いのでこの現象は出ない。

→ 「見えている地形の端が近いと止まる」挙動そのものは妥当(むしろ安全)。
今回のスイープでは crossing 完了後なので実害なし。**幅 50 cm 落下(A)とは
別物**(A は latch が **0 件**、B は crossing 後に 1 件)。

本物の安全停止(単独 100/1000 cm、混合 →100 cm)は別メッセージ
`uncrossable gap within 2.50 m ahead of the body`(前方プローブ)で、こちらは
すべて意図どおり手前で直立停止。

## 事実(集計ログ)

`scratchpad/full_sweep.sh` の出力(1 行 = 1 シナリオ):

```
step03 0.3m sp2.0      ec=0 | latch=0 | x= 11.23 minz=0.295 roll= 0.00 | CROSSED
step04 0.3m sp1.5      ec=0 | latch=0 | x= 11.25 minz=0.294 roll=-0.00 | CROSSED
15/15 N=2             ec=1 | latch=1 | x=  7.96 minz=0.291 roll= 0.02 | CROSSED
15/15 N=3             ec=1 | latch=1 | x=  8.25 minz=0.289 roll= 0.01 | CROSSED
15/15 N=4             ec=1 | latch=1 | x=  8.40 minz=0.306 roll=-0.01 | CROSSED
15/15 N=5             ec=1 | latch=1 | x=  8.23 minz=0.306 roll= 0.01 | CROSSED
15/15 N=6             ec=1 | latch=1 | x=  8.92 minz=0.305 roll= 0.01 | CROSSED
strip25/15 N=2        ec=1 | latch=1 | x=  8.21 minz=0.291 roll=-0.19 | CROSSED
strip35/15 N=2        ec=1 | latch=1 | x=  8.00 minz=0.305 roll= 0.01 | CROSSED
strip50/15 N=2        ec=1 | latch=1 | x=  8.69 minz=0.294 roll=-0.07 | CROSSED
25/gap25 N=2          ec=1 | latch=1 | x=  8.38 minz=0.289 roll= 0.07 | CROSSED
25/gap35 N=2          ec=1 | latch=1 | x=  7.67 minz=0.295 roll= 0.01 | CROSSED
25/gap50 N=2          ec=1 | latch=0 | x=  2.36 minz=-0.715 roll=-3.11 | FELL
single 30cm           ec=1 | latch=0 | x= 11.30 minz=0.295 roll=-0.01 | CROSSED
single 100cm          ec=1 | latch=1 | x=  0.21 minz=0.298 roll= 0.00 | SAFE-STOP
single 1000cm         ec=1 | latch=1 | x=  0.30 minz=0.272 roll=-0.00 | SAFE-STOP
15/15 x2 -> 50cm      ec=1 | latch=0 | x=  3.00 minz=-0.950 roll= 3.14 | FELL
15/15 x2 -> 100cm     ec=1 | latch=1 | x=  0.93 minz=0.306 roll=-0.00 | SAFE-STOP
```

判定基準:`|roll|>0.8` or `z<0.15` or `minz<0.15` → FELL / `x>4.0` → CROSSED /
それ以外 → SAFE-STOP。`ec` は 1=`edge_clearance:0.15`、0=`0.0`。
`latch` は `"latching graceful stop"` ログの件数。

## 再現

```bash
bash scratchpad/full_sweep.sh    # scratchpad は tmp。中身は本 md に転記済み
```

`local_planner.yaml` は `cp` バックアップ → trap で復元(`git checkout` 不使用)。
繰り返し・gap 可変・混合の地形は `gen_quadsdk_repeated_gap_world.py` が
実行時に生成。

## 関連

- `agent_reports/quadsdk_gap_foothold_summary.md`(まとめ / 5 シナリオの GIF)
- `agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md`(30/100 cm の原型)
- `agent_reports/steps/step_06_quadsdk_last_gap_1m.md`(前方 lookahead の導入)
- `agent_reports/steps/step_07_quadsdk_phase4_ik_reach.md`(機能 ON/OFF 比較)
