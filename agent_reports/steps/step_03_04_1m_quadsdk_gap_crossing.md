# Step 03_1m / 04_1m:Quad-SDK で「穴に足を入れずに」深い穴を渡る(成功)

対象: `external/quad-sdk`(go2、`reference:=twist` の Step 01 ハーネス系)。
Step 03/04(Quadruped-PyMPC、浅い轍)とは**別実装・別ロボットスタック**。

**状態: 成功。** go2 が **1 m 深・0.3 m 幅・幅 5 m のトレンチを、足を穴に
入れずに複数本連続で渡る**ようになった。C++ の挙動変更は無し(足場計画の
ロジックは素のまま)。効いたのは **歩容(GAIT)の調整**、**地形メッシュの
作り方**、**ホライズン長**の 3 点の設定変更だけ。

- step03_1m(間隔 2.0 m):**0.15 / 0.3 / 0.5 m/s** で連続 5〜6 本を渡り切る。
- step04_1m(間隔 1.5 m):**0.3 m/s** は安定して渡る。0.15 m/s は 2 回中 1 回
  成功(go2 Quad-SDK 特有の非決定性、7 節)。

> **読み方**: 本文は Quad-SDK の内部名を使った作業ログ。制御の初学者は
> 先に **0 節(用語)** と **6 節(大学院初心者向け解説)** を読むと
> 1〜5 節が追える。
>
> **この文書の 2 つの主張の切り分け**:
> - **事実**(ログ・CSV・GIF で確認済み) … 各節で「(確認済み)」と明記。
> - **推測**(そう考えると辻褄が合う、の域) … 各節で「(推測)」と明記。

---

## 結論(先に3行)

1. **GAIT / 足の周波数は Quad-SDK が自動調整しない**(確認済み、コード)。
   `period` / `duty_cycles` / `phase_offsets` は起動時に `go2.yaml` から
   1 回読んで固定テーブル(`nominal_contact_schedule_`)を作るだけ。twist
   モードに歩容のオンライン適応は無く、地形適応は **足場**(`getNearestValidFoothold`)
   と **胴体の高さ/傾き参照**(`getTerrainSlope`/`getTerrainHeight`)の 2 箇所
   だけ。歩容を動的に変える経路(LEAP/FLIGHT プリミティブ)は global body
   planner(`reference:=gbpl` + ゴール)の担当で、twist モードでは発火しない。
   → **穴に合わせて歩容を落としたいなら手で `go2.yaml` を変える**(今回は
   トロット → 静的安定なクロールへ。3 節)。terrain 連動にしたければコード追加。

2. **足場を正しく認識させる要点は「`traversability` レイヤを穴の上で実際に
   下げる」こと。** `getNearestValidFoothold` は
   `traversability ≤ foothold_obj_threshold(0.6)` or NaN のセルを却下し、
   `foothold_search_radius` 内の閾値超え最近セルへスナップするだけ。
   そのために必要な 3 条件:
   - **(a) 地形メッシュに“本物の穴”を空ける**(その帯に三角形を置かない)。
     生 `z` が NaN になり、`filter_chain.yaml` の穴検出フィルタ
     `traversability_hole_mask = 1 − |z_raw − z_inpainted|` が発火する。
     段差・ランプ・ジグザグ・「下げただけの面」では**発火しない**(面が
     在る=傾いてるだけになり、ノイジーな slope/roughness 頼みになる)。
   - **(b) 探索半径が立入禁止帯を跨げる + 足場を崩れかけの縁に載せない。**
     `foothold_search_radius` >(帯の半分)。メッシュ穴を物理穴より少し
     広く(今回 +0.05 m/側)して、スナップ先を縁から手前の solid にする。
   - **(c) solid 部分は完全に平ら・水平に保つ。** 同じマップの `z_smooth` /
     `smooth_normal_vectors` が `getTerrainSlope`/`getTerrainHeight` 経由で
     **胴体の偽の高さ/ピッチ指令**を作る。足場だけ直して胴体参照を壊すと、
     足は穴の外なのに縁で胴体が突っ込む。
   - 配線用の設定(`obj_fun_layer: traversability`, `foothold_obj_threshold: 0.6`)
     は既定のままで正しい。
   - **【未確認】実機で同じ機構が効くか**は本リポジトリのコードからは言えない。
     この repo に LiDAR/深度カメラ → grid_map の `z` レイヤを作る処理は無く、
     `mjcf_to_grid_map_converter` が静的 PLY をラスタライズするのみ。実センサでは
     no-return / occlusion / 未観測セル / 古いセルの扱い、および「穴」と「単なる
     未観測領域」の区別が別問題になる(詳細:
     `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` §6.3)。

3. この 2 つ(静的安定なクロール歩容 + 本物の穴のメッシュ)が揃えば、
   **セントロイダル NMPC + Raibert 足場の twist モードのまま**、go2 は
   1 m 深・0.3 m 幅の穴を連続で渡れる(LEAP プリミティブは不要)。
   詳細な根拠は以下 3〜6 節。

---

## 0. 用語(この文書で使う言葉)

- **四足の脚と歩容**
  - **支持脚(stance leg)** … 今、地面に着いて体重を支えている脚。
  - **遊脚(swing leg)** … 今、空中を次の着地点へ運ばれている脚。
  - **トロット(trot)** … 対角の 2 脚を同時に着け、もう一方の対角 2 脚を
    同時に振る速歩。常に 2 脚しか着かない → 静止では倒れる(6.1 節)。
  - **クロール(crawl)** … 1 脚ずつ順番に振る歩き方。常に 3 脚が接地する
    ので静的に安定。速度は出ないが穴・段差に強い。
  - **`period`(周期)** … 全脚が 1 サイクル(接地→遊脚→接地)を回る時間。
  - **`duty_cycle`(接地時間比)** … 1 周期のうち脚が接地している割合。
    0.75 なら 1 周期の 3/4 は地面に着いている。
  - **`phase_offsets`(位相オフセット)** … 4 脚それぞれがサイクルの
    どこから始まるか。`[0, 0.5, 0.5, 0]` = 対角同時(トロット)、
    `[0, 0.75, 0.5, 0.25]` = 1 脚ずつずらす(クロール)。
  - **`horizon_length`(ホライズン長)** … NMPC が何ステップ先まで
    予測して最適化するか。1 ステップ = `timestep` = 0.03 s。
- **足場と足場計画**
  - **足場(foothold)** … 遊脚が次に着地する目標地点(x, y, z)。
  - **Raibert 則** … 「今の速度と目標速度の差」から着地点を幾何的に決める
    古典式。地形は見ない。
  - **`local_footstep_planner`** … Quad-SDK で足場を決めるモジュール。
    Raibert 則の候補を、地形マップを見て安全な場所へずらす。
  - **`getNearestValidFoothold()`** … その「安全な場所へずらす」関数。
    候補周辺を渦巻き状に探索し、通行可能で `kin_cost` の小さいセルへスナップ。
  - **`kin_cost`** … 候補セルの採点値 = 候補までの距離 + 0.5 × 前回足場から
    の距離。小さいほど良い。
  - **`foothold_search_radius`** … 渦巻き探索の半径。候補からこの距離内でしか
    代わりの足場を探せない。
- **地形マップ(terrain map)** … 周囲の地面を格子で持ったもの。レイヤ:
  - **`z`** … 各セルの生の地面高さ。**メッシュに面が無い所は NaN**。
  - **`z_inpainted`** … NaN を補間で埋めた版。
  - **`z_smooth` / `smooth_normal_vectors_*`** … `z_inpainted` を広めに
    平滑化した高さ・法線。**胴体の高さ・傾き参照**に使われる(3.2 節)。
  - **`slope` / `roughness`** … 高さ変化から出す急峻さ・ざらつき。
  - **`traversability`(通行可能度)** … 0〜1。1 = 平ら・安全、0/NaN = 崖・穴。
    `getNearestValidFoothold` はこれが閾値(`foothold_obj_threshold` = 0.6)を
    超えるセルだけを有効な足場とみなす。
  - **`traversability_hole_mask`** … `filter_chain.yaml` が最初から持つ
    「穴検出器」。`1 − |z_raw − z_inpainted|` で、生 `z` が NaN の所
    (=メッシュの穴)を 0/NaN にして `traversability` を潰す(3.1 節)。
  - Quad-SDK では地形マップは**物理ワールドの XML ではなく、別途用意する
    メッシュ(PLY ファイル)**から作られる。
- **MPC と WBC(足場の下流)**
  - **NMPC** … 接触スケジュールを所与に、未来の胴体軌道と各接地脚の GRF を
    セントロイダルモデルで最適化。`plan_nmpc_cost`(CSV 列)がその目的関数値。
  - **GRF(地面反力)** … 接地脚が地面から受ける力。胴体を支え・進める。
  - **WBC 相当(逆動力学レッグコントローラ)** … NMPC の GRF と足先軌道を
    関節トルクへ変換。
- **動的安定 / 静的安定** … 6.1 節。

---

## 1. 背景・目的

- ご要望: 「Step 03/04 の目的は**穴に足を入れずに歩くこと**、穴の深さは無関係。
  穴に足を入れない foot place control をして、うまく行くまで検討する」。
  実装は **Quad-SDK 側**で行う。
- さらにご指摘(2 点):
  1. **GAIT や足の周波数の調整ができていないことが問題。**
  2. **Quad-SDK ではマップを受領して foot place control ができるはず。**
     そのあたりのコードを分析して検討を継続する。
- この 2 点は**どちらも正しかった**(確認済み)。過去の「twist モードでは
  渡れない」という結論は誤りで、原因は (a) 歩容の調整不足、(b) 地形メッシュの
  作り方がフレームワークの穴検出器と噛み合わず、かつ偽の胴体ピッチ指令を
  生んでいたこと、(c) スナップ先が穴の崩れかけの縁だったこと、の 3 つだった。

## 2. マップ仕様(step03_1m / step04_1m)

`src/trial/assets/gen_quadsdk_gap_world.py <spacing> <phys_depth> <tag> [_] [mesh_margin]`
が **2 つ**を生成する:

- **物理ワールド** `worlds/flat_gaps_<tag>.xml.xacro`(ロボットが実際に踏み、
  落ちうる地面)
  - y ∈ [-2.5, 2.5] の 5 m 幅通路。box 凸条(上面 z=0)を並べ、間に
    **0.30 m 長・幅 5 m・深さ 1.0 m** のトレンチ。単純プリミティブのみ。
  - step03_1m: 間隔 2.0 m(凸条 1.7 m)/ step04_1m: 間隔 1.5 m(凸条 1.2 m)。
- **地形マップ用メッシュ** `models/flat_gaps_<tag>/meshes/flat_gaps_<tag>.ply`
  (プランナが見る地面。`mjcf_to_grid_map_converter` がこの PLY を読む)
  - **各凸条ごとに 1 枚の水平な平面(z = 0)。穴の x 帯には面を置かない
    ── メッシュに本物の穴を空ける。**
  - メッシュの穴は物理トレンチより **`mesh_margin`(既定 0.05 m)ずつ左右に
    広い**。→ プランナが見る「立入禁止帯」= 0.30 + 2×0.05 = 0.40 m、
    実際に落ちる穴 = 0.30 m。スナップした足場が物理的な縁から
    0.05 m 手前に載る(2 節の設計理由は 3.3 節)。
  - `flat_wide.ply` と**同一のバイナリ形式**(binary LE、per-face RGBA +
    uchar-count int32、CRLF ヘッダ)。

実行: `GAP_WORLD=flat_gaps_2m.xml GAP_TAG=step03_1m FORWARD_VEL_MPS=0.3 \
DURATION_S=45 bash scripts/trial/run_quadsdk_gap_1m.sh`

## 3. コード分析で判明したこと(計装ビルドで確定)

`quad_utils` / `local_planner` に `[MPC_DOG DIAG]` ログを仕込んで再ビルドし、
**マップ受領 → フィルタ連鎖 → `getNearestValidFoothold` → 足場計画 → 胴体参照**
を追った。

### 3.1 穴検出はフレームワークが最初から持っている ── ただし「本物の穴」が要る(確認済み)

`filter_chain.yaml` には専用の穴検出フィルタ列がある:

```yaml
filter9:  traversability_hole_mask = 1.0 - abs(z_finite - z_inpainted)
filter10: MeanInRadiusFilter(radius 0.075)   # 穴マスクを少し外へ広げる
filter14: traversability = (traversability + 0.02) .* traversability_hole_mask_filtered
```

- メッシュに**面が無い**セルは、ray-cast 変換器が `z` = NaN のまま残す
  (`addLayerFromPolygonMesh` の DIAG: `finite=57800/67400` ← 差 9600 が穴)。
- `z_inpainted` はそこを埋める → `z` と `z_inpainted` が食い違う
  → `traversability_hole_mask` が NaN/0 → `traversability` が NaN。
- `getNearestValidFoothold` の DIAG(確認済み):
  `nominal x=1.006 trav=nan -> snapped x=1.146`、
  `nominal x=0.940 trav=nan -> snapped x=0.890`。
  **穴帯の nominal を毎回検出し、solid strip へスナップしている。**
  `found=0`(スナップ失敗)は 0 件。

> **過去にハマった点(推測込み)**: 以前は PLY を「連続面 + 穴帯だけ段差
> (dip)」や「ジグザグ」で作っていた。これだと生 `z` が NaN にならないので
> **filter9 の穴検出が発火せず**、`slope`/`roughness` 頼みになって帯が
> 不安定だった。「本物の穴」にした瞬間に穴検出が素直に効いた。

### 3.2 偽の胴体ピッチ指令 ── 平らな凸条なら消える(確認済み)

`local_planner.cpp` の twist モードは、ホライズンの各点の**胴体参照を地形
マップから直接作る**:

```cpp
ref_ground_height_(i) = getTerrainHeight(x, y);          // z_smooth
ref_body_plan_(i, 2)  = z_des_ + ref_ground_height_(i);  // 胴体高さ参照
getTerrainSlope(x, y, yaw, ref_body_plan_(i,3), ref_body_plan_(i,4));
                                                        // 胴体 roll/pitch 参照
```

- `getTerrainSlope` は `smooth_normal_vectors_*` を読む。地形メッシュを
  「dip」や「ジグザグ」にしていた頃は、この平滑法線が縁で傾き、
  **NMPC に「胴体を 30〜50° 鼻下げにせよ」という偽の pitch 参照**が入り、
  縁で鼻から突っ込んでいた(旧 run のピッチスパイク +0.3〜0.5 rad の正体)。
- **凸条を完全に水平・同一高さの平面**にすると、`z_smooth` も平滑法線も
  平ら → 偽の高さ/ピッチ指令が消える。穴の上でも `z_inpainted` は 0 近傍で
  埋まるので `getTerrainHeight` ≈ 0(確認済み: 平面化後は縁での pitch 参照が
  ほぼ 0、CSV のピッチが穴通過中 ±0.02 rad)。
- **足場回避はそれでも維持される。** 足場スナップは `traversability`(=穴
  検出、3.1)を見ており、平滑法線とは別系統だから。

### 3.3 スナップ先が「崩れかけの縁」だった(確認済み)

`mesh_margin = 0` だと、`getNearestValidFoothold` のスナップ先が
x ≈ 0.87〜0.90 ── **物理トレンチ(x∈[0.85, 1.15])の縁の上か、わずかに
中**になっていた(DIAG: `snapped x=0.878`, `0.897`)。1 m 落下の縁に足を
載せる → 接地が不安定 → 胴体が沈む(CSV: `grf_FR_z` が 78→142→150 N に
スパイクし `vz` = −0.3 m/s、確認済み)。

- filter10 の barrier 半径(0.075→0.2)を広げても効かなかった。**穴帯の
  `hole_mask` は 0 ではなく NaN で、`MeanInRadiusFilter` は NaN を広げ
  られない**(確認済み: barrier を広げても snapped x が変わらず)。
- 効いた対策は 2 節の **メッシュの穴を物理穴より 0.05 m ずつ広くする**。
  以後スナップ先は x ≤ 0.80(物理縁 0.85 から 0.05 m 手前)。

### 3.4 「渡る一歩」を早く出しすぎて NMPC が破綻していた(確認済み)

旧 run では、胴体が x ≈ 0.55 m(穴の 0.3 m 手前)で **`plan_nmpc_cost` が
0.05 → 0.3 → 1.0 → 10 と発散**し、その後で胴体が物理的に崩れていた
(確認済み、CSV)。

- 現象(確認済み、CSV): ホライズン内の**前脚接地予定が向こうの
  凸条(x ≈ 1.2)に置かれる**一方、`computeFutureBodyPlan` で延長しても
  胴体は x ≈ 0.9 までしか進まない。この状態で `plan_nmpc_cost` が発散した。
- **【訂正】機序**: 「NMPC 内の脚可到達制約が破れた」は**誤り**。go2 の
  simple NMPC の制約 `g` は **EOM(Backward Euler)+ 摩擦錐のみ**で、関節角・
  足位置・IK 可到達性の制約は**存在しない**(`nmpc_controller/scripts/dynamicsModel.m`。
  詳細:`agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` §4.3・§6.1)。
- **推測(要ログ、未検証)**: 遠い足場が GRF のモーメントアーム
  \((p_f-p_b)\) を変え、`ref_body_plan_` を満たす GRF 配分が
  \(f_z\in[10,150]\) と摩擦錐(実効 μ=0.6)の内側に取りにくくなり、
  スラック(`panic`/`constraint_panic`)が立って `plan_nmpc_cost` が増える、
  という筋は辻褄が合う。ただし cost 内訳(トラッキング項 vs スラック項)・
  制約違反量・IPOPT 終了ステータスを記録するまで**確定ではない**。
- 以前ここに入れていた「近端スナップ禁止(前方バイアス 100×)」は
  **逆効果**だった:向こう側の縁への一歩を強制していた。**素の
  `kin_cost` に戻す**と、前回足場が後ろにあるぶん近端の `kin_cost` が
  小さく、**自然に段階を踏む**:①まず穴の手前の縁に足を置く → ②胴体を
  寄せる → ③次の一歩は「スナップ不要の普通の足場」として向こうの凸条に
  乗る(確認済み: DIAG で near→far の 2 段が見える)。
- **`horizon_length` 26 → 40 で追従が改善したのは実験事実。** ただし
  **「`horizon_length > period_` がコード上の必須条件」ではない**
  (`computeContactSchedule` は `nominal_contact_schedule_[(i+phase) % period_]`
  で剰余ラップし、大小関係に必須条件は無い。詳細:
  `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` §6.2)。
  `period` 0.9 s → `period_` = 30 に対し既定 26 では NMPC が最適化する接触列が
  1 歩容周期を覆わない、という**機序は推測**(要 A/B)。このパラメータ組で
  40 が有利だった、という位置づけで扱う。

## 4. 施した変更(すべて設定のみ。C++ の挙動変更は無し)

| ファイル | 変更 | 理由(節) |
|---|---|---|
| `local_planner/config/local_planner.yaml` | `horizon_length` **26 → 40** | クロール `period_`=30 を覆う(3.4) |
| `quad_utils/config/go2.yaml` | `period` **0.36 → 0.9**、`duty_cycles` **[0.5]→[0.75]**、`phase_offsets` **[0,0.5,0.5,0] → [0,0.75,0.5,0.25]** | トロット → 定石の横回りクロール。常時 3 脚接地、横ドリフトも消える(6.3) |
| 〃 | `foothold_search_radius` **0.25 → 0.7** | 立入禁止帯 0.40 m の中心 nominal からも隣の凸条に届く(3.1, 3.3) |
| 〃 | `ground_clearance` **0.07 → 0.1** | 渡る一歩で足が縁を擦らない |
| `src/trial/assets/gen_quadsdk_gap_world.py` | 地形 PLY を「連続ジグザグ面」→ **凸条ごとの水平平面 + 物理穴より 0.05 m 広いメッシュ穴** | 穴検出を素直に発火させ、偽ピッチを消し、縁から手前にスナップさせる(3.1〜3.3) |
| `local_planner/src/local_footstep_planner.cpp` | `getNearestValidFoothold` の `kin_cost` を**素の式に戻す**(前方バイアス 100× を撤去)。ほかは `[MPC_DOG DIAG]` ログのみ | 段階を踏んだ渡りを妨げない(3.4) |
| `local_planner/src/local_planner.cpp` / `quad_utils/src/mjcf_to_grid_map_converter.cpp` | `[MPC_DOG DIAG]` ログのみ(挙動変更なし) | 再現・継続調査用。撤去可 |
| `quad_utils/config/filter_chain.yaml` | **stock に戻した**(旧 WIP の平滑化半径縮小を撤回) | 穴検出(3.1)が仕事をするので平滑化を素に戻せる |

## 5. 結果(CSV + 固定カメラ GIF で確認)

### 5.1 足場回避(穴に足を入れない) ✅

`getNearestValidFoothold` が穴帯の全 nominal(`traversability` = NaN)を検出し、
毎回 solid strip へスナップ(3.1)。`found=0` は 0 件。

### 5.2 胴体が穴を渡る ✅

- **step03_1m(間隔 2.0 m、深さ 1.0 m)**

  | 指令速度 | 結果 | 到達 x | 横ドリフト |max\|roll\|,\|pitch\| |
  |---|---|---|---|---|
  | 0.15 m/s | ✅ 連続 3〜4 本 | 7.0 m | 0.7 m | < 0.02 rad |
  | 0.3 m/s | ✅ 連続 5 本 | 11.5 m | 0.06 m | < 0.03 rad |
  | 0.5 m/s | ✅ 連続 6 本(指令区間内) | 11.6 m | 0.28 m | < 0.02 rad |

  z(胴体高さ)は全区間 0.31 m を保持。穴通過中も水平(pitch ±0.02 rad)。
  `phase_offsets` を定石の横回り `[0,0.75,0.5,0.25]` にしてから
  **横ドリフトが 0.7 m → 0.06 m に激減**し、0.5 m/s も渡れるようになった。

- **step04_1m(間隔 1.5 m、深さ 1.0 m)**

  | 指令速度 | 結果 | 到達 x |
  |---|---|---|
  | 0.3 m/s | ✅ 連続 5〜6 本、横ドリフト 0.11 m | 10.8 m |
  | 0.15 m/s | △ 2 回中 1 回成功(4 本渡って停止)、1 回は 1 本目で転倒 | 7.0 m / 0.8 m |

  間隔が狭いぶん 1 本渡った直後に次の縁が来て立て直す余裕が少ない。
  0.3 m/s の方がむしろ安定(推測: クロール歩容に対して指令速度が遅すぎると
  1 歩あたりの前進が小さく、穴を跨ぐのに余分な歩数=擾乱回数がかかる)。

GIF: `artifacts/gifs/quadsdk_step03_1m_v0p5.gif`、
`artifacts/gifs/quadsdk_step04_1m_v0p3.gif`(固定カメラ。凸条の目盛りで
前進が目視できる)。

### 5.3 残る弱点

- step04_1m の 0.15 m/s と、step03_1m の 0.5 m/s を超える速度は非決定的に
  転ぶ。go2 Quad-SDK は**平地でも 0.5〜1.1 m/s で非決定的に転ぶ**ことが
  Step 01 で既知(7 節)。穴渡りの限界というより go2 twist 歩容自体の限界。
- 横ドリフト。0.3 m/s では 0.1 m 程度に収まるが、0.15 m/s では 0.7 m まで
  出る(5 m 幅の穴なので落ちはしない)。

---

## 6. 大学院初心者向け解説:なぜ「歩容の調整」で渡れたのか

(用語は 0 節。)

### 6.1 静的安定と動的安定

- **静的安定**: 接地脚が作る多角形(support polygon)の中に重心の鉛直
  投影が入っていれば、**止まっていても倒れない**。
- **動的安定**: トロットは常時 2 脚接地。対角 2 脚を結ぶ線分はほぼ「線」で、
  静的には倒れかけ。それを**次の一歩を正しい場所・時刻に置くこと**で連続的に
  立て直している(倒立振子を手で支え続けるのと同じ)。安定余裕は薄い。

### 6.2 0.3 m 幅の穴が「トロットには広すぎた」

go2 の前後の足の間隔は約 0.35 m、1 歩の歩幅はトロットで約 0.2 m。
0.3 m の穴(+ 縁の余裕で実効 0.4 m)を跨ぐには、**普段より長い一歩**が要る。
トロットでこれをやると:

- 遊脚が短時間(swing ≈ 0.18 s)で 0.4〜0.5 m 動く → 足先速度が 2〜3 m/s。
  着地衝撃と反作用で胴体が煽られる。
- その一瞬、対角の支持脚は穴の**手前だけ**にある → 胴体は穴の上へ
  片持ち → 沈む/ピッチする。

トロットの薄い安定余裕では、この 1 本ぶんの擾乱を吸収しきれない。

### 6.3 効いた歩容:定石の「横回りクロール」

`period` 0.9 s / `duty_cycle` 0.75 / `phase_offsets` `[0, 0.75, 0.5, 0.25]`
= 前左 → 後右 → 前右 → 後左 の順に **1 脚ずつ**振るクロール。

- **常に 3 脚接地**(duty 0.75)。1 脚が穴の上を渡る間、残り 3 脚で
  support polygon を保てる → 静的安定を崩さずに渡れる。
- **ゆっくり**(period 0.9 s)なので、渡る一歩の遊脚に 0.27 s かけられ、
  足先速度が半分以下に。着地衝撃が小さい。
- **対角ではなく横回りの順序**にすると、左右の踏み替えが均等になり、
  横方向の並進(ドリフト)が打ち消される(実測: ドリフト 0.7 m → 0.06 m)。
- ゆっくりで周期が長いぶん、**NMPC のホライズンを 26 → 40 ステップに
  延ばして 1 歩容周期(30 ステップ)を覆う**必要があった。覆えていないと
  NMPC は「1 周期先で何が接地しているか」を知らずに最適化してしまう。

### 6.4 「足を置く場所」と「バランス」は両方要る

- **足を置く場所**: `getNearestValidFoothold` が穴の外へ。
  → 「本物のメッシュ穴」にして初めて素直に効いた(3.1)。
- **バランス**: 縁の擾乱に耐える。
  → **歩容を静的安定なクロールに落とし、ゆっくりにする**ことで、
  縁 1 本ぶんの擾乱を support polygon の余裕内に収めた。

過去に「twist モードでは無理、LEAP プリミティブが要る」と結論していたが、
それは**歩容がトロットのままだった**からで、**クロールに替えれば
セントロイダル NMPC + Raibert 足場のまま渡れる**(0.3 m 幅・1 m 深に対して)。

---

## 7. go2 Quad-SDK twist 歩容の非決定性(既知)

`scripts/trial/run_quadsdk_gap_1m.sh` の冒頭コメントにある通り、go2 の
twist 歩行は**同一条件でも成功/転倒がばらつく**(Step 01 で確認済み)。
本 Step でも step04_1m の 0.15 m/s で観測。判定は **CSV(z, roll, pitch,
到達 x)と GIF の目視の両方**で行い、片方だけでは成功としない。

---

## 8. 再現方法

```bash
# 1) 穴ワールド + 地形 PLY を生成(external/quad-sdk へ書き込む)
python3 src/trial/assets/gen_quadsdk_gap_world.py 2.0 1.0 2m       # step03_1m
python3 src/trial/assets/gen_quadsdk_gap_world.py 1.5 1.0 1p5m     # step04_1m
#   第5引数 = メッシュ穴の片側マージン[m](既定 0.05)

# 2) install/ に反映(--symlink-install なので config はそのまま効く。
#    地形ファイルだけ install ツリーへ symlink)
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
for w in flat_gaps_2m flat_gaps_1p5m; do
  ln -sfn "$PWD/$SRC/worlds/$w.xml.xacro" "$INST/worlds/$w.xml.xacro"
  ln -sfn "$PWD/$SRC/models/$w"           "$INST/models/$w"
done

# 3) DIAG 入りバイナリを反映(挙動変更は無いが DIAG ログが要るなら)
source /opt/ros/jazzy/setup.bash
( cd ros2_ws && colcon build --packages-select quad_utils local_planner \
    --symlink-install --allow-overriding quad_utils local_planner )

# 4) 実行(速度・時間は環境変数で)
GAP_WORLD=flat_gaps_2m.xml  GAP_TAG=step03_1m FORWARD_VEL_MPS=0.3 DURATION_S=45 \
  bash scripts/trial/run_quadsdk_gap_1m.sh
GAP_WORLD=flat_gaps_1p5m.xml GAP_TAG=step04_1m FORWARD_VEL_MPS=0.3 DURATION_S=45 \
  bash scripts/trial/run_quadsdk_gap_1m.sh

# 5) GIF 化(目視確認。数値だけで成功判定しない)
bash scripts/trial/make_gif.sh \
  artifacts/logs/quadsdk_step03_1m/logs/mujoco_go2_*.mp4 \
  artifacts/gifs/quadsdk_step03_1m.gif 8 520
```

出力: `artifacts/logs/quadsdk_step0{3,4}_1m/{state_log.csv, trials_summary.csv}`
+ `.../logs/*.mp4`(いずれも `.gitignore` 対象)。

## 9. 変更・追加ファイル一覧

**MPC_DOG 側:**
- `src/trial/assets/gen_quadsdk_gap_world.py`(地形 PLY を平面 + 本物の穴へ)
- `scripts/trial/run_quadsdk_gap_1m.sh`
- `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`(本ファイル)
- `artifacts/gifs/quadsdk_step03_1m_v0p5.gif` / `quadsdk_step04_1m_v0p3.gif`

**external/quad-sdk(新規):**
- `quad_simulator/quad_sim_scripts/worlds/flat_gaps_2m.xml.xacro` / `flat_gaps_1p5m.xml.xacro`
- `quad_simulator/quad_sim_scripts/models/flat_gaps_2m/meshes/flat_gaps_2m.ply` / `flat_gaps_1p5m/...`

**external/quad-sdk(設定変更):**
- `local_planner/config/local_planner.yaml`(`horizon_length` 26→40)
- `quad_utils/config/go2.yaml`(`period` 0.9 / `duty` 0.75 / `phase_offsets`
  横回りクロール / `foothold_search_radius` 0.7 / `ground_clearance` 0.1)

**external/quad-sdk(C++、挙動変更なし):**
- `local_planner/src/local_footstep_planner.cpp`(前方バイアス撤去 = 素に戻す、+ DIAG)
- `local_planner/src/local_planner.cpp`(DIAG のみ)
- `quad_utils/src/mjcf_to_grid_map_converter.cpp`(DIAG のみ)
- `quad_utils/config/filter_chain.yaml`(stock へ戻す)

## 10. 関連

- `agent_reports/steps/step_03_gap_crossing.md` / `step_04_gap_crossing_1p5m.md`
  (PyMPC、浅い轍、成功)
- `agent_reports/quadsdk_step01_gait_and_mpc.md`(歩容と MPC の役割分担)
- `agent_reports/quadsdk_step01_terrain_map.md`(地形マップ = PLY 由来)
- `agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md`(穴超えの整理)
