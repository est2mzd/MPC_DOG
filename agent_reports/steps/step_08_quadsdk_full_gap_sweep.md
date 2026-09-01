# Step 08:穴の数・間隔・幅を変えた全シナリオ一括検証(回帰スイープ)

> **訂正(Step 09、2026-09-02)**:本 Step の「真因は `InpaintFilter` が
> `traversability` を埋めるから」は**誤り**。Step 09 のセル単位計測で、
> 50 cm の穴の内側は `traversability = NaN`(正しく危険)だと確認した。
> 本当の原因は「危険帯(≈ 物理幅 0.50 m)が `max_crossable_gap`(0.6 m)より
> 小さいので渡れる穴に分類される」+「既定では幅チェック自体が走らない」。
> `max_crossable_gap` を ~0.44 に下げれば 50 cm は捕まえられる(下で「しきい値
> では直らない」としたのは誤り)。詳細:
> `steps/step_09_terrain_grid_and_foothold_measurement.md`。以下の本文は
> 実行当時の記録として残す。

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

**16/18 は期待どおり(≤30 cm は回帰なしで渡り・≥100 cm は手前で安全停止)。
2/18(幅 50 cm の穴)で「渡れず・止まらず・転倒」** — `max_crossable_gap` を
0.6→0.54 に下げても直らず(2/2 で再現)。真因は閾値ではなく **地形フィルタの
`InpaintFilter`(radius 0.4)が ~0.4〜0.9 m の穴を認識レイヤで埋めている**こと
(下記 発見 A)。コードは 0.6 のまま、修正は Phase 5 へ。

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
| **strip25 / gap50 N=2** | .15 | **FELL** (x≈2.5, roll→±π/2〜π) | **✗** | **A:止まらず落下(2/2)** |
| 単独 30 cm | .15 | CROSSED (x≈11.3) | ✓ | latch なし(地形が長い) |
| 単独 100 cm | .15 | SAFE-STOP (x≈0.2) | ✓ | 前方プローブ |
| 単独 1000 cm | .15 | SAFE-STOP (x≈0.3) | ✓ | 前方プローブ |
| **15/15 ×2 → 50 cm** | .15 | **FELL** (x≈3.0, roll→π) | **✗** | **A:止まらず落下(3/3)** |
| 15/15 ×2 → 100 cm(Step 06) | .15 | SAFE-STOP (x≈0.9) | ✓ | 前方プローブ |

### 発見 A(本物の穴):~0.4〜0.9 m の穴が「渡れず・止まらず」

観測:
- 0.35 m の穴 → 渡り切る(2/2)
- 0.50 m の穴(単独・小穴の後ろ、両方)→ **踏み込んで落下**(2/2。ログに
  `EDGE_TOO_CLOSE` も latch も 0 件 = 安全層が一度も働いていない)
- 1.0 m 以上 → 手前で安全停止

`max_crossable_gap = 0.6` は「穴の先 0.6 m 以内に地面が戻れば渡れる穴とみなす」
設定だが、これを 0.54 に下げても 0.50 m の穴は落下のまま。**閾値ではなかった。**

#### 追検証:`max_crossable_gap` を下げても直らなかった

ユーザー方針(「30 cm 以下が渡れれば OK、それ超えは対象外・ただし落下は困る」)を
受けて `max_crossable_gap` を **0.6 → 0.54** に下げて 11 本を再実行した:

| 検証 | 結果 |
|---|---|
| step03 `ec0` / `ec0.15`、step04 `ec0.15` | CROSSED — **30 cm は回帰なし** |
| 繰り返し 15/15 N=2、strip25 gap25 / **gap35**(2/2) | CROSSED |
| 単独 100 / 1000 cm、15/15 ×2 → 100 cm | SAFE-STOP |
| **strip25 gap50**(2/2)、**15/15 ×2 → 50 cm**(2/2) | **FELL(不変)** |

→ **0.54 でも 0.50 m の穴は一度もフラグされず落下**(0.6 と同じ)。閾値の問題では
なかった。`max_crossable_gap` は **0.6 に戻した**(コミット参照)。

#### 本当の原因:地形フィルタの InpaintFilter

Phase 3 のプローブが読む `traversability` レイヤは、`filter_chain.yaml` の
**`InpaintFilter`(`radius: 0.4`)で穴が埋められた `z_inpainted` から作られる**。

- 幅 0.15 m の穴 → 完全に埋まる(Step 05 で foot が 5 cm 帯に乗れたのはこのため)。
- 幅 0.5〜0.6 m の穴 → **inpaint 半径 0.4 m がほぼ橋渡し**してしまい、
  `traversability` が `foothold_obj_threshold`(0.6)を割るほど下がらない
  → プローブが「連続する危険帯」を検出できない → ロボットが踏み込む
  → 物理的な穴は本物なので落下。
- 幅 1.0 m 以上の穴 → 中心が埋まりきらず `traversability` が谷になる → 検出 →
  手前で安全停止。

つまり **~0.4〜0.9 m の穴は認識(perception)側で埋められていて、
foot placement 層からは「渡れる地面」に見えている**。

**対策候補**:

1. Phase 3 プローブを `traversability`(inpaint 済み)ではなく **生の elevation
   レイヤの NaN** で判定する。生の `z` は物理メッシュの穴の幅ぶん(0.5 + 2×margin
   ≈ 0.6 m)そのまま NaN なので、中くらいの穴も見える。**推奨(将来作業=Phase 5)。**
2. `InpaintFilter` の `radius` を 0.4 → 0.15 程度に下げる。ただし surface normal /
   slope / roughness も同じ `z_inpainted` から作るので影響範囲が広く、スタック
   全体の再検証が必要。リスク高。
3. 現状維持 + 文書化。0.4〜0.9 m の穴は「認識で埋まるため安全停止しない」と明記。

→ この Step では**コード未変更**(`max_crossable_gap` は 0.6 のまま。yaml コメントに
上記の注記を追加した)。プローブの作り直しは Phase 5 に回す。

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

`max_crossable_gap:0.54` での再実行(`scratchpad/retune_verify.sh` +
`scratchpad/rerun_flaky.sh`、曖昧ケースは 2 回ずつ):

```
step03 0.3m ec0        CROSSED (x=11.87)
step03 0.3m ec0.15     CROSSED (x=11.15)   # 30cm 回帰なし(最重要)
step04 0.3m ec0.15     CROSSED (x=10.52)
rep 15/15 N=2          CROSSED (x=8.06,  latch=1 末尾)
rep 25/gap25 N=2       CROSSED (x=8.41)
rep 25/gap35 N=2       CROSSED (x=8.38 / 7.94)          # 2/2
rep 25/gap50 N=2       FELL    (x=2.56 / 2.76, latch=0) # 2/2 本物の落下
15/15 x2 -> 50cm       FELL    (x=3.18 / 3.06 / 2.94)   # 3/3
single 100cm           SAFE-STOP (x=0.05)
single 1000cm          SAFE-STOP (x=0.08)
15/15 x2 -> 100cm      SAFE-STOP (x=0.75)
```

(初回 retune_verify で gap35/gap50 が x=-0.04 のまま出たのは NMPC 起動失敗の
フレーク。再実行で gap35=CROSSED、gap50=FELL に確定。)

## 再現

```bash
bash scripts/trial/quadsdk_full_gap_sweep.sh   # 18 本の一括スイープ(ec:0.6)
# 追検証(scratchpad、内容は本 md に転記済み):
#   scratchpad/retune_verify.sh   max_crossable_gap:0.54 で 11 本
#   scratchpad/rerun_flaky.sh     曖昧 3 ケースを 2 回ずつ
```

`local_planner.yaml` は `cp` バックアップ → trap で復元(`git checkout` 不使用)。
繰り返し・gap 可変・混合の地形は `gen_quadsdk_repeated_gap_world.py` が
実行時に生成。`InpaintFilter` の設定は
`external/quad-sdk/quad_utils/config/filter_chain.yaml`(`filter2`, `radius: 0.4`)。

## 関連

- `agent_reports/quadsdk_gap_foothold_summary.md`(まとめ / 5 シナリオの GIF)
- `agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md`(30/100 cm の原型)
- `agent_reports/steps/step_06_quadsdk_last_gap_1m.md`(前方 lookahead の導入)
- `agent_reports/steps/step_07_quadsdk_phase4_ik_reach.md`(機能 ON/OFF 比較)
