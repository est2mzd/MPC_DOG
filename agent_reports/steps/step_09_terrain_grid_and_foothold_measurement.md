# Step 09: Terrain Map と足場判断の定量計測(制御変更なし)

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
指示書: `chatgpt_instruction/cursor_instruction_quadsdk_multistep_terrain_foothold_planner.md`。

## 1. 背景

Step 08 で「幅 0.4〜0.9 m の穴で渡れず・止まらず・転倒」を見つけたが、原因の説明が
2 転した:

- Step 08 当初:「`max_crossable_gap` を下げても直らない」
- 解説 doc:「`InpaintFilter` が穴を埋めて `traversability` が >0.6 のまま」

どちらも **推測混じり**だった。Step 09 は、15/25/30/35/50/100 cm の単独トレンチ
について、地形マップの各レイヤ値と足場選択結果を **セル単位で数値記録**し、
50 cm の穴が

- **A**: 穴セル自体を `traversable` と誤認しているのか
- **B**: 穴セルは unsafe だが、向こう岸の valid セルへスナップして通過可能と
  判定しているのか
- **C**: A と B の両方

を確定する。制御挙動は一切変更しない(計装のみ)。

## 2. 目的

1. Terrain Map の経路・レイヤ・式・`getNearestValidFootholdResult` の中身を確認して
   記録する(§4)。
2. env `MPCDOG_STEP09_DIR` で有効化する計装(既定 OFF)を `local_planner` に足し、
   `step09_map_cross_section.csv` と `step09_footholds.csv` を吐かせる。
3. 6 種のトレンチで実行し、A/B/C を数値で確定する。
4. feature OFF で既存 18 シナリオ(代表 2 本)の制御結果が変わらないことを確認する。

## 3. 変更前のコード経路(Map 生成 → getNearestValidFootholdResult)

| # | 事実(`rg`・コード・実行ログで確認) |
|---|---|
| 経路 | `mjcf_to_grid_map_node`(生地図・layer `z`・穴 = `NaN`・**解像度 0.05 m**・frame `map`)→ `grid_map_filters_demo`(`filter_chain.yaml`、18 段)→ `/mapping/terrain_map` → `local_planner` が `topics.terrain_map` で購読 → `terrainMapCallback`([local_planner.cpp:235](../../external/quad-sdk/local_planner/src/local_planner.cpp#L235))で `grid_map::GridMap terrain_grid_` へ |
| 公開レイヤ | `z, x, y, z_inpainted, z_smooth, normal_vectors_*, smooth_normal_vectors_*, slope, roughness, traversability, traversability_mask`。**`hole_mask` / `hole_mask_filtered` は filter16 で削除**(公開されない)。生 `z`(NaN 入り)は公開されている |
| 式 | `z_inpainted` = InpaintFilter(`z`, radius **0.4**)。`hole_mask` = `1 − |z_finite − z_inpainted|`。`hole_mask_filtered` = MeanInRadiusFilter(`hole_mask`, radius **0.075**)→ 0 で下限しきい。`traversability` = `((0.5(1−roughness/0.1) + 0.5(1−slope/0.4)) + 0.02) × hole_mask_filtered`(filter12/14)、slope/roughness は `z_inpainted` から |
| `getNearestValidFootholdResult(名目, prev_solve, leg, hip_world)` | `SpiralIterator` で名目セルから半径 `foothold_search_radius_`(**0.7 m**)。各セル `t = atPosition("traversability", cell)`。**`t > foothold_obj_threshold_`(0.6)** かつ `kin_cost = ‖p−名目‖ + 0.5‖p−prev_solve‖` 最小を採用。無ければ `NO_TRAVERSABLE_CANDIDATE`(位置 = 名目)。高さは **`z_inpainted` の線形補間** + `toe_radius_`、非有限なら `NONFINITE_HEIGHT`。[local_footstep_planner.cpp:572](../../external/quad-sdk/local_planner/src/local_footstep_planner.cpp#L572) |
| 範囲外 / 未観測 / 穴 | **範囲外のみ区別**(`isInside`)。**穴と未観測は区別なし** — 生地図はどちらも `z = NaN`、`observed` レイヤ無し。inpaint 後は両方「埋まった地面」 |
| Phase 3/2B の穴チェック | `edge_clearance > 0` のときだけ動く。既定 `edge_clearance:0` では **足場の幅チェックは一切走らない** |

## 4. 事実 / 推測 / 未確認

- **事実(Step 09 で計測)**: 下記 §9 の表・断面図・CSV。
- **推測だったもの(Step 09 で訂正)**: 「InpaintFilter が 50 cm の穴を埋めて
  `traversability` が >0.6 のまま」→ **誤り**。§9 参照。
- **未確認(このStepではCSV列を出すが値は入れない)**: `hole_mask_filtered`
  (公開マップから削除済み)。`hole_mask` は `clamp(1−|z−z_inpainted|,0,1)` で
  再構成した値を記録(列名 `hole_mask_recon`)。

## 5. 変更計画

| ファイル | 追加内容 | 既定 OFF の担保 |
|---|---|---|
| `external/quad-sdk/local_planner/src/local_footstep_planner.cpp` | 匿名 namespace に `step09*` ヘルパ。`computeFootPlan` に env `MPCDOG_STEP09_DIR` ガード付き CSV ダンプ(未設定なら 1 行も実行されない) | 戻り値・`foot_positions`・NMPC 入力・`cmd_vel`・停止挙動すべて不変。`local_planner.cpp` は無変更 |
| `scripts/trial/step09_measure.sh` | 15/25/30/35/50/100 cm 単独トレンチを生成→実行→CSV/GIF 収集 | yaml 無変更。既存 step03〜08 の world を上書きしない |
| `scripts/trial/step09_analyze.py` | CSV → A/B/C 判定 + 断面図 PNG | — |

## 6. 変更ファイルと変更理由

- `local_footstep_planner.cpp`(計装のみ・env ガード): A/B の判定に必要な
  `nominal_x/y`・`snap_distance`・`hip_distance`・`foothold_status` は
  `computeFootPlan` 内部にしか無く publish されない → 外部ノードでは不可。
- 新 world `flat_trench_s09_{15,25,30,35,50,100}`(`gen_quadsdk_wide_trench_world.py`、
  MESH_MARGIN 0.05): 既存 world を汚さない・幅だけを変えた統制条件。

## 7. 入出力・単位・座標系

- frame: `map`(世界固定)。進行方向 +x。トレンチ近縁 x0 = 2.0 m、深さ 1 m、全幅。
- `step09_map_cross_section.csv`(y≈0 の 1 行、全セル): `map_stamp, frame,
  cell_i, cell_j, x[m], y[m], z_raw[m], z_inpainted[m], z_smooth[m], slope[rad],
  roughness[m], hole_mask_recon[0..1], hole_mask_filtered(=nan), traversability[0..1],
  traversability_mask[0..1], observed(=isfinite(z_raw)), inside_map(=1),
  binary_safe(=traversability>0.6)`。
- `step09_footholds.csv`(未来 touchdown ごと・毎周期): `time[s], map_stamp, frame,
  leg(FL/BL/FR/BR), touchdown_index, touchdown_time[s], nominal_x/y[m],
  nominal_traversability, selected_x/y/z[m], selected_z_raw[m](選択セルの生 z),
  selected_z_inpainted[m], selected_hole_mask_recon, selected_observed,
  selected_binary_safe, snap_distance[m], hip_distance[m], foothold_status`。
  - `foothold_status`: 0=VALID 1=NOMINAL_OUTSIDE_MAP 2=NO_TRAVERSABLE_CANDIDATE
    3=NONFINITE_HEIGHT 4=EDGE_TOO_CLOSE 5=IK_UNREACHABLE。
  - `hip_distance` = `‖selected_xyz − hip_position_midstance‖`。
    `hip_position_midstance` は `welzlMinimumCircle` の出力で z 成分は円半径
    (胴体高さではない)。stance 中の hip はほぼ動かないので実質「足〜hip 円中心の
    水平距離」。Phase 4 が `> ik_max_reach` で使うのと同じ量。
  - `map_stamp` は現状 0(フィルタ出力にタイムスタンプが載っていない)。順序は
    `time` で追う。

## 8. 実験条件

- feature スイッチは **出荷既定のまま**(`edge_clearance:0`, `ik_reach_check:false`,
  latch 系 true)。A/B は `getNearestValidFootholdResult` の素のスナップの話で、
  `edge_clearance` に依存しない。`local_planner.yaml` は一切触らない。
- 各トレンチ 1 本(制御主張はしないので反復不要。数値と GIF を突き合わせる)。
  30 s 実行。

## 9. 試行結果

### 9.1 地図断面(全トレンチ・`step09_map_cross_section.csv`)

| トレンチ(物理) | 生 `z` が NaN の帯 | `traversability` unsafe(NaN or ≤0.6)の帯 | `max_crossable_gap`=0.6 との比較 | 実挙動 |
|---|---:|---:|---|---|
| 15 cm | 0.25 m | **0.15 m** | 0.15 < 0.6 → crossable | 渡り切る(x≈6.8) |
| 25 cm | 0.35 m | **0.25 m** | crossable | 渡り切る(x≈7.0) |
| 30 cm | 0.40 m | **0.30 m** | crossable | 渡り切る(x≈7.0) |
| 35 cm | 0.45 m | **0.35 m** | crossable | 渡り切る(x≈6.8) |
| **50 cm** | **0.60 m** | **0.50 m** | **0.50 < 0.6 → crossable** | **転倒(x≈2.3)** |
| 100 cm | 1.10 m | **1.00 m** | 1.00 ≥ 0.6 → uncrossable | 転倒(x≈2.6、幅チェック OFF なので停止せず) |

**規則性**:
- `生 z の NaN 帯 = 物理の穴幅 + 0.10 m`(= 2 × MESH_MARGIN 0.05)。
- `traversability の unsafe 帯 = 生 NaN 帯 − 0.10 m ≈ 物理の穴幅`。
  縁の穴マスクぼかし(MeanInRadiusFilter radius 0.075)が両側 1 セル(0.05 m)ずつ
  「safe」に戻すため。

### 9.2 訂正:「InpaintFilter が穴を埋めて traversability >0.6 のまま」は誤り

50 cm トレンチの断面(抜粋):

```
 x=1.93  z_raw=0.000  z_inp=0.000  trav=1.000  observed=1   ← 近縁 solid
 x=1.98  z_raw=nan    z_inp=0.000  trav=1.000  observed=0   ← 縁1セル: 生NaNだが trav=1.0(ぼかしの穴)
 x=2.02  z_raw=nan    z_inp=0.000  trav=nan    observed=0   ┐
 …(10セル、≈0.50 m)              trav=nan               ├ 穴の内側は正しく unsafe
 x=2.48  z_raw=nan    z_inp=0.000  trav=nan    observed=0   ┘
 x=2.52  z_raw=nan    z_inp=0.000  trav=1.000  observed=0   ← 縁1セル: 同上
 x=2.58  z_raw=0.000  z_inp=0.000  trav=1.000  observed=1   ← 遠縁 solid
```

`InpaintFilter` が埋めるのは `z_inpainted`(高さ)だけ。`hole_mask = 1−|z−z_inpainted|`
は生 `z` が NaN なので **穴の内側は NaN のまま** → `traversability` も NaN。
50 cm の穴の内側 0.50 m はちゃんと unsafe。埋めの影響は **縁 1 セルのぼかしだけ**。

### 9.3 A / B / C の判定

`step09_footholds.csv` の前脚(FL/FR)touchdown を、名目 x がトレンチ近傍のものに
絞って集計(断面図 `s09_50_cross_section.png` に可視化):

| 判定 | 結果 | 根拠(50 cm、前脚 touchdown 2348 件) |
|---|---|---|
| **B(向こう岸へスナップ)** | **NO** | `selected_x > 遠縁` かつ `snap_distance > 0.1` の件数 = **0**。kin_cost(最近傍 + 0.5×prev_solve、prev_solve は後方)は常に **手前寄り**の最近傍セルを選ぶ。遠バンク(x≥2.56)へ跳ぶことはない |
| **A(穴セルを safe と誤認)** | **YES(ただし「縁」限定)** | 名目が穴内(x≈2.0〜2.33)のとき、スナップ先は **x≈1.95〜2.00 または x≈2.50〜2.55**(`selected_z_raw = nan`, `selected_observed = 0`, `selected_binary_safe = 1`, `status = 0 VALID`)= **物理の void 縁の 1 セル**。`traversability` の unsafe 帯(x 2.02〜2.48)の内側に着地した足場は **1 件も無い** |
| **C** | NO | A の「縁」形であって B ではない |

**確定:A(縁形)。** 穴の中心は正しく unsafe。物理 void の縁 ±0.05 m だけが
ぼかしで「safe」に見え、スナップがその縁セルに足を置く。既定では幅チェックが
走らないので `EDGE_TOO_CLOSE` は一度も出ない(全 `status = VALID`)。

### 9.4 なぜ 30 cm は渡れて 50 cm は落ちるか

- 30 cm も同じ「縁セルに着地」が起きる(`selected_z_raw = nan` at x≈1.95〜2.00 /
  x≈2.30〜2.35)。だが物理 void が 0.30 m なので、縁に置いた足から **次の 1 歩で
  遠バンクの実地面(x≥2.36)に届く**。クロール歩容の遅い前進で支持が繋がる。
- 50 cm は縁(x≈2.0)に足を置いた時点で **既に void の上**。遠縁(x≈2.5)は 0.5 m
  先で 1 歩では届かない。支持されない足に体重が乗る → 縁が崩れる / 到達不能 → 転倒。

### 9.5 しきい値で直せるか(Step 08 の訂正)

`traversability` unsafe 帯 ≈ 物理の穴幅。50 cm → 0.50 m。`max_crossable_gap` を
**≤ 0.44 に下げれば** 50 cm(0.50 m)は `EDGE_TOO_CLOSE` に、30 cm(0.30 m)は
crossable のまま。Step 08 で試した 0.54 は 0.50 より大きかったので捕まらなかった
だけ。**「しきい値では直らない」は誤りだった。**
ただし `edge_clearance > 0` が前提(既定 OFF)。また MESH_MARGIN 0.10 の world
(step03/04 用 `flat_gaps_2m`)は 30 cm でも unsafe 帯 ≈ 0.40 m なので、しきい値を
下げる場合はその world の回帰確認が要る。

## 10. 失敗原因(50 cm 転倒の因果、数値で確定)

1. 既定 `edge_clearance:0` → 穴の幅を見るコードが一切走らない。
2. Raibert 名目足場が穴内に落ちる(x≈2.0〜2.3)。
3. `getNearestValidFootholdResult` が `traversability > 0.6` の最近傍セルを探す。
   50 cm の穴では、それが **物理 void 縁のぼかしセル**(生 z = NaN、trav = 1.0)。
4. `status = VALID` で NMPC へ渡る。足は void の縁 = 未支持地面に置かれる。
5. 遠縁は 0.5 m 先で 1 歩では届かず、体重が未支持足に乗って転倒。

## 11. 後方互換性確認

- 計装は env `MPCDOG_STEP09_DIR` 未設定で 1 行も実行されない。`local_planner.cpp`
  無変更。`local_planner` gtest **41/41 green**。
- feature OFF(env なし)sim 回帰(計装ビルドで実行):
  - `flat_gaps_2m`(step03 相当、0.3 m 穴): **渡り切り** x=9.87、z=0.31、roll=0.00、
    minz=0.295(Step 08 baseline は x=11.23。twist 歩容は前進距離が非決定的だが
    どちらも直立で渡り切り = 回帰なし)
  - `flat_trench_1m`(100 cm): **穴に落下** x=2.48、z=−0.94、roll=−π、minz=−0.946
    (Step 08 の feature OFF `onoff_g100_off` = x=2.64、z=−0.94、roll=π と一致。
    `edge_clearance:0` では幅チェックが走らないので落ちるのが既存挙動)

## 12. GIF・CSV・ログ

- `artifacts/step09/s09_{15,25,30,35,50,100}/`:
  `step09_map_cross_section.csv` / `step09_footholds.csv` / `state_log.csv` /
  `run.log` / `s09_*_cross_section.png` / `step09_*.gif`
- README には **30 cm 成功 GIF** と **50 cm 失敗 GIF** + 50 cm の断面図を掲載。

## 13. 次 Step へ進む条件

- [x] 50 cm の A/B を数値で確定(= A の縁形、B は無し)。
- [x] 既存 18 シナリオ(代表)の feature OFF 回帰一致。
- Step 10(現在歩容から未来の脚順序を shadow 再構成)へ進んでよい。
  ただし本 Step で分かった「幅チェックが既定で走らない」「unsafe 帯 ≈ 物理幅」
  「縁 1 セルがぼかしで safe」は Step 11〜13 の候補生成・停止余裕に効く。
