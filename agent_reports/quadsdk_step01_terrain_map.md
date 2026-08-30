# Quad-SDK Step 01 の地形マップ(map)の作り方とデータ構造

作成: 2026-08-30。`external/quad-sdk` の launch ファイルと C++ ソースを実際に
読んで確認した内容。

---

## 背景

Step 01(0.3 m/s で 10 m 以上の前進歩行)の調査で、地面まわりが2回問題になった:

1. 既定の `flat.xml` は地面が約9m しかなく、10m 歩くと端から落ちる。
2. 広い地形として `big_flat.xml`(詳細メッシュ)に替えたら、原因不明の
   不安定化が起きた。

最終的に、`flat.xml` と同じ「箱1個」の単純な地面を寸法だけ拡大した
`flat_wide.xml` を新規に作って解決した(handoff 3節)。

このとき「地面(world の XML)を替えると、制御側が見る地形マップはどう作られ、
何が変わるのか」を追う必要が出た。本ドキュメントはその調査結果で、
`world:=flat_wide.xml` を渡したときに **地形マップがどのファイル・どのノードから
生成され、`local_planner` が最終的に受け取る `grid_map_msgs/GridMap` に何が
入っているか** を、コードで確認できた事実と、未確認の推測に分けてまとめる。

パイプライン全体の中での位置づけは
`agent_reports/quadsdk_step01_control_pipeline.md` の4節(MAP 段)を参照。

---

## 概要(先に結論)

- 地形マップの入力は **MJCF(world の XML)ではなく、`.ply` メッシュファイル**。
  ノード名は `mjcf_to_grid_map` だが XML はパースしていない。
- `flat_wide` の実体は **8頂点の箱1個**(`flat_wide.ply`)。world 座標系で
  x ∈ [-3, 15] m、y ∈ [-5, 5] m、上面 z = 0。`flat.ply` と構造は同じで寸法だけ違う。
- 生成は **起動時に1回だけ**。歩行中に地形マップが更新されることはない
  (latched publish で保持される)。
- 流れは3ノード:
  1. `mjcf_to_grid_map`(`.ply` を読み、生の高さマップ `terrain_map_raw` を作る)
  2. `grid_map_filters`(穴埋め・平滑化・法線・傾斜・粗さ・traversability を追加 → `terrain_map`)
  3. `topic_tools/relay`(`/mapping/terrain_map` → `/robot_1/terrain_map` に中継)
- `local_planner` はこれを受けて内部構造 `FastTerrainMap` に詰め替え、
  主に `z_inpainted`(高さ)・`normal_vectors_*`(法線)・`traversability`
  (着地点の良さ)の3種類を使う。
- **flat_wide は完全な平面**なので、実際に効いてくるのは「地面高さ ≒ 0」
  「傾き 0」だけ。traversability 等の層は計算はされるがほぼ一様。

---

## 流れ(順を追って)

### ステップ1: world XML から `.ply` パスを決める

`scripts/trial/run_quadsdk_step01_baseline.sh` が
`ros2 launch quad_utils quad_mujoco.py world:=flat_wide.xml ...` を実行する。
その中の `launch_robot_mapping` が `/mapping` 名前空間で `mujoco_mapping.py` を
起動し、`mjcf_to_grid_map_node` に `world:=flat_wide.xml` を渡す。

`mjcf_to_grid_map_converter.cpp` のコンストラクタは、この `world` から `.xml` を
外して、

```
quad_sim_scripts/models/flat_wide/meshes/flat_wide.ply
```

というパスを組み立て、`loadMeshFromFile()` を1回呼ぶ。
**MJCF(world の XML)は読んでいない**。同名の `.ply` メッシュを探すだけ。

### ステップ2: `.ply`(箱)を高さグリッドに変換 → `terrain_map_raw`

`meshToGridMap()` が PCL でメッシュを読み、`grid_map::GridMapPclConverter` で
`grid_map::GridMap` にラスタライズする。ここでできるのは:

- `z` 層(または `grid_map_layer_name` 指定名)… 各セルの地面高さ
- `x` 層 / `y` 層 … 各セルの world 座標(後段が座標を引きやすいように手動追加)

これを `/mapping/terrain_map_raw` に publish する。**publish は1回だけ**で、
以後は latched(`transient_local`)で保持される。

### ステップ3: フィルタチェーンで層を足す → `terrain_map`

`grid_map_filters_demo`(name `grid_map_filters`)が
`quad_utils/config/filter_chain.yaml` の filter1〜18 を順に適用し、
`/mapping/terrain_map_raw` → `/mapping/terrain_map` を出力する。追加される層:

- `z_inpainted` … 穴埋め済み高さ(inpaint、radius 0.4)
- `z_smooth` … 平滑化高さ(mean-in-radius、radius 0.2)
- `normal_vectors_{x,y,z}` … 法線(z_inpainted から)
- `smooth_normal_vectors_{x,y,z}` … 平滑法線(z_smooth から)
- `slope` … `acos(normal_vectors_z)`(傾斜角)
- `roughness` … `abs(z_inpainted - z_smooth)`(粗さ)
- `traversability` … `0.5*(1 - roughness/0.1) + 0.5*(1 - slope/0.4)` を 0〜1 にクランプ
- `traversability_mask` … traversability を 0.5 でしきい値処理したもの
  (この 0.5 は footstep planner の `foothold_obj_threshold` と一致させる約束)

### ステップ4: ロボット名前空間へ中継

`quad_mujoco_bringup.py: access_terrain_map()` が `topic_tools/relay` で
`/mapping/terrain_map` → `/robot_1/terrain_map` にそのまま転送する。
(同時に起動される `grid_map_visualization` と `static_transform_publisher` は
RViz 表示と `world`→`map` TF 用で、マップ生成には関与しない。)

### ステップ5: `local_planner` が受け取り、内部構造に詰め替える

`local_planner/src/local_planner.cpp` の `terrainMapCallback()`(`:208`):

1. `GridMapRosConverter::fromMessage()` で `grid_map::GridMap`(`terrain_grid_`)に復元
2. `terrain_.loadDataFromGridMap(terrain_grid_)` で独自の `FastTerrainMap`
   (高速クエリ用の `std::vector` 集合)に詰め替え
3. `local_footstep_planner` に両方を渡す

以降、プランナは:

- `z_inpainted` … 胴体参照軌道の地面高さ、着地点の高さ
- `normal_vectors_*` … 地面法線
- `traversability` … 着地点探索の目的関数(`traversability > 閾値` で採用可)

を引いて歩容を組み立てる。flat_wide では高さ ≒ 0・法線 ≒ (0,0,1)・
traversability ≒ 一様なので、実質「平らな床」以上の情報は使われない。

---

## 詳細:データ構造

### `grid_map_msgs/msg/GridMap`(ROS メッセージ)

grid_map ライブラリの標準型:

- `header` … `stamp` / `frame_id`(= `map`)
- `info`
  - `resolution` … セル1辺の長さ [m]
  - `length_x`, `length_y` … マップ全体の実寸 [m]
  - `pose` … マップ中心の world 姿勢
- `layers` … 層名の配列(上記「流れ」ステップ2〜3の層)
- `basic_layers` … 有限値必須とみなす層の部分集合
- `data` … `std_msgs/Float32MultiArray` の配列。層ごとに1枚、
  `Size(0) x Size(1)` セルを行優先で格納
- `outer_start_index` / `inner_start_index` … 円環バッファの回転量
  (マップを動かしたときのズレ管理。静的マップでは実質 0)

### grid_map の座標規約

- マップ中心が原点。`+x` は行インデックス減少方向、`+y` は列インデックス減少方向
- セル `(0,0)` はマップの「左上」= world 座標が最大の角
- `mjcf_to_grid_map_converter` が `x` / `y` 層を明示的に埋めているのは、
  この規約を意識せずセルの world 座標を直接引けるようにするため

### `FastTerrainMap`(`local_planner` 内部)

`quad_utils/src/fast_terrain_map.cpp`。grid_map から詰め替えたプレーンな配列:

- `x_size_`, `y_size_` … セル数
- `x_data_`(長さ `x_size_`), `y_data_`(長さ `y_size_`)… 各行/列の world 座標
- `z_data_`[i][j] … 高さ(`z_inpainted` 由来)
- `nx_data_` / `ny_data_` / `nz_data_`[i][j] … 法線(`normal_vectors_*` 由来。
  無ければ (0,0,1))
- `z_data_filt_`, `nx_data_filt_` ほか … 平滑版(`z_smooth` /
  `smooth_normal_vectors_*` 由来。無ければ生値で代用)
- 双線形補間で任意 `(x,y)` の高さ・法線・傾斜を返す

### `flat_wide` の確定値

- `flat_wide.ply`(バイナリを実際にデコードして確認):
  - PLY ヘッダ: `binary_little_endian` / `SOLIDWORKS generated, length unit = meters`
  - `element vertex 8` / `element face 12` = **直方体(箱)1個**
  - 頂点座標(world 座標系そのまま、m): x ∈ [-3, 15]、y ∈ [-5, 5]、z ∈ [-0.1, 0.0]
- `flat_wide.xml.xacro` の floor: `<geom type="box" size="9 5 0.05" pos="6 0 -0.05">`
  → x ∈ [-3, 15]、y ∈ [-5, 5]、上面 z = 0(メッシュと一致)
- 高さ層はどのセルも z ≒ 0.0(完全な平面。傾き 0、粗さ 0)
- マップは**起動時に1回だけ生成 → latched で保持**。歩行中は不変
  (`mesh` トピックに publisher が無いため再変換されない)

### `flat` / `flat_wide` / `big_flat` の違い

- `flat.ply` … 8頂点の箱、x∈[-2,7]・y∈[-2,2]
- `flat_wide.ply` … 8頂点の箱、x∈[-3,15]・y∈[-5,5]。
  **`flat.ply` と同じ「箱1個」で寸法だけ拡大**(バイト比較で頂点データのみ差分)
- `big_flat` … `big_flat.stl`(handoff の記述で 98,592 三角形の詳細メッシュ)。
  形式が `.stl`、三角形数が3桁以上多い
- Step 01 の成功構成は `flat_wide`(handoff 4節)

---

## 【推測】未確認事項

- **解像度**
  - `mujoco_mapping.py` は `grid_map_resolution:=0.05`(m)を渡している。
    一方 `mjcf_to_grid_map_converter.hpp` の既定値は `0.2`
  - launch 引数が優先されるはずなので実効 0.05 m と推測しているが、
    実行時パラメータのダンプ確認はしていない
  - 0.05 m なら flat_wide(18 m × 10 m)で概ね 360 × 200 ≒ 72,000 セル。
    平地なのでフィルタ計算は軽いはずだが未計測
- **平地での `traversability`**
  - slope=0・roughness=0 なら式上ほぼ一様 1.0、`traversability_mask` も
    一様 1.0 になるはず。実際の publish 内容(端の穴埋め領域や境界セル)は未確認
- **`big_flat.xml` 不安定化のメカニズム**
  - handoff 9節でも「メッシュ衝突判定の数値的脆さ」と推測止まり
  - 地形マップ観点の追加仮説: `.stl` 詳細メッシュを 0.05 m グリッドへ
    ラスタライズする際、三角形境界で微小な高さノイズが乗って
    `slope` / `roughness` が非ゼロになり、着地点の目的関数(`traversability`)が
    場所ごとにブレて着地点が揺れる可能性。未検証(`flat_wide` で達成済みのため優先度低)
- **マップ更新頻度**
  - `mesh` トピックに publisher が無いことはコードから読めるが、
    別プロセスが publish しない保証まではとっていない。実行中に `terrain_map` が
    再 publish されないことは `ros2 topic hz` 等で未確認
- **`normal_vectors_z` の符号**
  - `NormalVectorsFilter` の `normal_vector_positive_axis: z` で上向きに
    正規化されるはずだが、境界セルの挙動は未確認

---

## その後(このマップの使われ方と、次に見るべき点)

- **このマップの下流**: `local_planner` → `local_footstep_planner` が
  `z_inpainted` と `traversability` で着地点を決め、`local_planner.cpp` が
  地面高さ入りの参照軌道を作り、`nmpc_controller`(NMPC)に渡す。
  詳細は `agent_reports/quadsdk_step01_control_pipeline.md` の6〜7節。
- **flat_wide での結論**: 地形マップは「平らな床、高さ 0」以上の情報を
  与えていない。Step 01 の歩行の成否は地形マップ側ではなく、
  起動シーケンス(`joint_controller` 起動待ち)と地面の物理サイズで決まっていた
  (handoff 8節)。
- **段差・傾斜のある Step に進むとき最初に確認すべきこと**:
  - 実効解像度(上記【推測】)を実測で確定する
  - 段差地形用の world は `.stl` + `<world>.bin/.png` hfield 経路になる
    (`quad_mujoco.py: prepare_world` が `terrain_heightmap` を解決する分岐)。
    その場合 `.ply` 経路ではなく hfield 経路を読む必要がある
  - `traversability_mask` のしきい値 0.5 と footstep planner の
    `foothold_obj_threshold` の対応(`filter_chain.yaml` のコメント)

---

## ソース早見表(`external/quad-sdk/`)

- メッシュ実体
  - `quad_simulator/quad_sim_scripts/models/flat_wide/meshes/flat_wide.ply`(MPC_DOG 追加)
  - `quad_simulator/quad_sim_scripts/worlds/flat_wide.xml.xacro`(MPC_DOG 追加)
- メッシュ → grid_map
  - `quad_utils/src/mjcf_to_grid_map_node.cpp`(エントリ)
  - `quad_utils/src/mjcf_to_grid_map_converter.cpp`(本体、`meshToGridMap`)
  - `quad_utils/include/quad_utils/mjcf_to_grid_map_converter.hpp`(既定値・宣言)
- フィルタチェーン
  - `quad_utils/launch/mujoco_mapping.py`(ノード起動)
  - `quad_utils/config/filter_chain.yaml`(フィルタ定義 filter1〜18)
- 名前空間中継・可視化・TF
  - `quad_utils/launch/quad_mujoco_bringup.py`(`access_terrain_map`)
  - `quad_utils/launch/quad_mujoco.py`(`launch_robot_mapping`)
- 消費側
  - `local_planner/src/local_planner.cpp`(`terrainMapCallback`, `:208`)
  - `quad_utils/src/fast_terrain_map.cpp`(`loadDataFromGridMap`, `:158`)
  - `local_planner/src/local_footstep_planner.cpp`(`z_inpainted` / `traversability` 参照)
