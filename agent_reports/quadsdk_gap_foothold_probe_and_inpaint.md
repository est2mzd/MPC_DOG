# なぜ「前方プローブ」は中サイズの穴を見逃すのか — データの流れとロジックの中身

読者は制御の大学院初心者を想定。数式より「どのデータが、どの処理で、どう変わって、
どのロジックが何を見て判断するか」を追う。

---

## 背景

現行コード(Phase 2A + 3(A) + 2B + 4)で穴対応を有効(`edge_clearance:=0.15`)に
すると、

- 幅 **30 cm** の穴 → 渡り切る ✅
- 幅 **100 cm** の穴 → 手前で直立停止 ✅
- 幅 **50 cm** の穴 → **止まらず踏み込んで転倒** ❌(Step 08 で 2〜3/3 再現)

「前方プローブが『渡れない穴』と判定できず、止まらない」と説明したが、その中身を
ここで丁寧に展開する。

## 目的

- 地形マップがどう作られ、`InpaintFilter`(穴埋め)が何をするかを知る。
- 前方プローブ(2 種類ある)が **どのレイヤの、どの値を、どう見て** 判断するかを、
  実コードの疑似コードで理解する。
- 30 / 40 / 50 / 100 cm の穴で結果が分かれる理由を、レイヤの断面図で説明する。

## 結論(先に)

| 穴幅 | 有効時(`edge_clearance:=0.15`)の挙動 | 直接の理由 |
|---|---|---|
| **30 cm** | 渡り切る | 穴埋めで「ほぼ平地」に見える + クロール歩容が物理的に 0.3 m を跨げる |
| **40 cm** | 未実測。**安全停止は効かない** → 渡れる or 落ちる | 穴埋めで橋渡しされ「渡れる地面」に見える。go2 の実力は 35 cm ○ / 50 cm × の境目 |
| **50 cm** | **落下** | 穴埋め(半径 0.4 m)がほぼ橋渡し → `traversability` が危険閾値を割らない → プローブが「連続する危険帯」を取れない → 止まらない。だが物理の穴は本物なので脚が届かず落ちる |
| **100 cm** | 手前で直立停止 | 半径 0.4 m では埋めきれず穴の中心に「谷」が残る → `traversability` が閾値を割る帯ができる → プローブが検知 |

**キモ**:プローブが読む `traversability` レイヤは、穴を **埋めた後** の高さから作られる。
だから「埋まる大きさの穴」はプローブから見えない。閾値(`max_crossable_gap`)を
いくら調整しても、見えないものは判定できない。

---

## 1. 登場するデータ(grid_map のレイヤ)

地形マップ(`grid_map`)は、同じ XY 格子の上に「レイヤ(層)」を何枚も重ねた
もの。1 セル = **0.05 m 角**。関係するレイヤ:

| レイヤ | 意味 | 穴のセルでの値 |
|---|---|---|
| `z` | **生の高さ**。メッシュ(PLY)にレイキャストして得た標高。面が無い所は `NaN` | **`NaN`**(穴は本当に「無い」) |
| `z_inpainted` | `z` の穴を **画像修復(inpaint)で埋めた** 高さ | 周囲から推定した値(埋まれば ≈ 0、埋まらなければ変な値) |
| `traversability` | **足を置いてよいか** の 0〜1 スコア。1 に近いほど安全。foot placement と前方プローブが読むのはこれ | 埋まった穴 → ≈ 1(安全に見える)/ 埋まらない穴 → ≈ 0 |

`foothold_obj_threshold = 0.6`:`traversability > 0.6` のセルだけ「足を置ける」と
みなす。プローブは `traversability <= 0.6`(または `NaN`)を **危険(unsafe)** と扱う。

---

## 2. データの流れ(処理パイプライン)

```
 物理世界(MuJoCo)              地形マップ生成                 フィルタ連鎖(filter_chain.yaml)
 ┌───────────────┐   PLY    ┌──────────────────┐   /terrain_map_raw   ┌──────────────────────────┐
 │ box strip 群  │ ───────▶ │ mjcf_to_grid_map │ ─────────z レイヤ──▶ │ 18 段のフィルタ           │
 │ (穴は隙間)    │  mesh    │  レイキャスト     │  (穴 = NaN)          │  ・inpaint (穴埋め)      │
 └───────────────┘          └──────────────────┘                     │  ・slope / roughness     │
                                                                     │  ・hole mask             │
                                                                     │  → traversability レイヤ  │
                                                                     └──────────┬───────────────┘
                                                                                │ /terrain_map
                                                                                ▼
                                                       ┌─────────────────────────────────────────┐
                                                       │ local_footstep_planner                   │
                                                       │  ① getNearestValidFoothold…             │
                                                       │     (足場スナップ + 前方プローブ A)     │
                                                       │  ② hasUncrossableGapAhead (前方プローブ B)│
                                                       │     → EDGE_TOO_CLOSE / safe-stop 判定    │
                                                       └─────────────────────────────────────────┘
```

**要点:foot placement が触るのは一番右の `traversability` だけ。生の `z`(NaN)は
見ていない。** 途中の `inpaint` で穴が埋まると、その情報はプローブに届かない。

---

## 3. `InpaintFilter` が何をするか(`filter_chain.yaml` filter2)

```yaml
filter2:
  name: inpaint
  type: gridMapCv/InpaintFilter
  params:
    input_layer: z
    output_layer: z_inpainted
    radius: 0.4          # ← これ
```

- OpenCV の画像修復。`z` の `NaN`(穴)セルを、**穴のフチから内側へ値を伝播** させて
  埋める。`radius` は「フチのどれくらいの範囲を参考にして塗るか」。
- 穴の幅が **`radius` の 2 倍(= 0.8 m)以下** だと、両側のフチの情報が穴の中央で
  出会い、**ほぼ平らに埋まる**。
- 穴の幅がそれより大きいと、中央はフチから遠すぎて情報が届かず、埋めた値が
  荒れる(または埋まりきらない)。

`traversability` レイヤは、この `z_inpainted` から作った slope / roughness と、
「生と修復後の差 `hole_mask = 1 − |z − z_inpainted|`」の掛け算で決まる
(filter12〜filter14)。つまり:

- **修復がうまくいった穴** → `z` と `z_inpainted` の差が小さい → `hole_mask ≈ 1`
  → `traversability ≈ 1` → **プローブから見て「安全な地面」**。
- **修復が破綻した穴(広い)** → 差が大きい → `hole_mask ≈ 0`
  → `traversability ≈ 0` → **プローブから見て「危険帯」**。

穴が「危険」に見えるかどうかは、**穴埋めが失敗するほど広いか** だけで決まる。

---

## 4. 前方プローブのロジック(実コードの中身)

`edge_clearance > 0`(有効化)のときだけ動く。2 種類ある。どちらも
`traversability` レイヤ(`obj_fun_layer_`)を 0.05 m 刻みで前方へ舐める。

### プローブ A:足場ごとの縁チェック
`getNearestValidFootholdResult()`
[local_footstep_planner.cpp:665-706](../external/quad-sdk/local_planner/src/local_footstep_planner.cpp#L665-L706)

```text
選んだ足場から +x 方向へ 0.05 m ずつ、最大 (edge_clearance + max_crossable_gap) まで進む:
  そのセルの traversability を読む
  unsafe = (値が NaN) or (値 <= 0.6)              # foothold_obj_threshold

  まだ穴に入っていない状態で:
    unsafe を見つけた:
       その穴が edge_clearance(0.15 m)より先なら → 「穴は遠い、足場OK」で終了
       0.15 m 以内なら → 「足場は縁の上」フラグを立て、hole_start = 現在距離

  縁の上フラグが立った後で:
    safe(地面)を見つけた → 「向こう岸が届いた = 渡れる穴」→ VALID のまま終了
    unsafe が続き、(現在距離 − hole_start) >= max_crossable_gap(0.6 m) に達した
        → EDGE_TOO_CLOSE(渡れない穴)と判定
```

### プローブ B:胴体からの前方 lookahead
`hasUncrossableGapAhead()`
[local_footstep_planner.cpp:742-768](../external/quad-sdk/local_planner/src/local_footstep_planner.cpp#L742-L768)

```text
胴体の現在位置から +x へ 0.05 m ずつ、最大 safe_stop_lookahead(2.5 m)まで進む:
  unsafe = (traversability が NaN) or (<= 0.6)
  unsafe が連続して max_crossable_gap(0.6 m)分たまったら
      → return true(渡れない穴が前方にある → Phase 2B が cmd_vel:=0 でラッチ停止)
  地図の外に出たら → return false(未知であって崖ではない)
```

**両方に共通する前提:「`traversability <= 0.6` のセルが、進行方向に 0.6 m 以上
続いている」こと。** これが「渡れない穴」の定義。

---

## 5. なぜ 30 / 50 / 100 cm で結果が分かれるか(レイヤの断面図)

進行方向 x に沿って、穴の周りの `traversability`(●=安全>0.6, ・=危険≤0.6)を並べる。
物理の穴幅と、地図上で「危険」に見える帯の幅は **別物** であることに注意。

### 幅 30 cm(物理)

```
 地面     穴(30cm)   地面
 ●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●     ← traversability
                              ↑ 穴埋め(半径0.4m)が完全に埋める。危険セルが 0 個
```
プローブ:`unsafe` を一度も見ない → 何も起きない。
足場スナップ:普通に地面セルへ。クロール歩容が物理的に 0.3 m を跨ぐ → **渡り切る**。

### 幅 50 cm(物理)

```
 地面        穴(50cm)      地面
 ●●●●●●●●●●●●●●●●●・・●●●●●●●●●●●●●●●●●●●●●●
                    ↑ 穴埋め(半径0.4m)がほぼ橋渡し。危険セルは数個(0.6 m 未満)
```
プローブ A/B:`unsafe` の連続が **0.6 m に届かない**(すぐ地面に戻る)→
「渡れる穴」と判定 or そもそも縁と認識しない → **止めない**。
でも物理の穴は 50 cm 本物 → 脚が空を踏む → **落下**。

### 幅 100 cm(物理)

```
 地面              穴(100cm)               地面
 ●●●●●●●●●●●●●●●●・・・・・・・・・・・・・・・・・・・・●●●●●●●●●●●●
                  ↑ 半径0.4m では中央まで埋まらない。危険セルが 0.6 m 以上連続
```
プローブ B:`unsafe` が 0.6 m 以上たまる → `return true` →
Phase 2B が `cmd_vel:=0` → **手前で直立停止**。

---

## 6. 「危険帯」はおよそ 0.35〜0.9 m

- 下限 ≈ **0.35 m**:go2(Raibert + クロール歩容、0.3 m/s)が物理的に跨げる限界。
  これより狭ければ、プローブが見逃しても脚が届いて渡れてしまう(結果オーライ)。
- 上限 ≈ **0.9 m**:`InpaintFilter` の半径 0.4 m が橋渡しできる限界。これより広ければ
  中央に危険帯が残ってプローブが検知する。

この **0.35〜0.9 m の間** だけ、「プローブは見逃す」かつ「脚も届かない」の二重の
谷になり、落下する。Step 08 の 50 cm はこの帯のど真ん中。

---

## 7. おまけの落とし穴:地図の帯幅 ≠ 物理の穴幅

地形生成器は、メッシュを物理の穴より **`MESH_MARGIN` だけ内側に削る**(足場が
崩れ縁に寄りすぎないようにする安全マージン)。値が生成器ごとに違う:

| 生成器 | `MESH_MARGIN`/片側 | 物理 0.30 m の穴 → 生 `z` が NaN の帯 |
|---|---|---|
| `gen_quadsdk_gap_world.py`(step03/04) | 0.10 | **0.50 m** |
| `gen_quadsdk_repeated_gap_world.py` / `wide_trench` | 0.05 | 0.40 m |

つまり「地図座標で穴幅の線を引く」と、物理的な意味とズレる(物理 0.30 m と
0.40 m の穴が、地図上ではどちらも 0.50 m の帯になりうる)。判定は
**生の値で見られる所は生で見る** べき、という教訓。

---

## 8. 直し方(Phase 5 の方針)

前方プローブを、**穴埋め済みの `traversability` ではなく、生の `z` レイヤの `NaN`**
で判定するように変える。生 `z` は物理メッシュの穴の分だけ `NaN` のまま残るので、
中サイズの穴も「連続する NaN」として見える。

- 新パラメータ例 `uncrossable_nan_width`(進行方向に NaN がこの幅続いたら停止)。
- 既定 `edge_clearance:0` は不変なので、有効化しない実行は完全に従来どおり。
- 有効化する Step 05 / 05b / 06 / 07 は閾値を詰めて再スイープが必要
  (詳細と後方互換の議論は `steps/step_08_quadsdk_full_gap_sweep.md` と
  チャット履歴のまとめ)。

---

## 関連

- `quadsdk_gap_foothold_summary.md` — 現状・成功例/失敗例・教訓の 1 枚まとめ
- `steps/step_08_quadsdk_full_gap_sweep.md` — 18 シナリオ回帰と `InpaintFilter` 発見の実測
- `external/quad-sdk/quad_utils/config/filter_chain.yaml` — フィルタ連鎖の実体(filter2 = inpaint)
- `external/quad-sdk/local_planner/src/local_footstep_planner.cpp` — プローブ A(L665)/ B(L742)
