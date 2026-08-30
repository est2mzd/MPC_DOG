# Quad-SDK の simple モデルで地形対応(高さ考慮の足場選び・穴超え)はどこまでできるか

作成: 2026-08-31。`external/quad-sdk` の C++ ソース・YAML・world 定義を実際に
読んで確認した内容。**【事実】=コードで確認済み**、**【推測】=未確認・仮説**
として節を分ける。関連: `agent_reports/quadsdk_step01_mpc.md`、
`agent_reports/quadsdk_step01_mpc_simple_vs_complex.md`、
`agent_reports/quadsdk_step01_gait_and_mpc.md`、
`agent_reports/quadsdk_step01_terrain_map.md`。

---

## 背景・目的

go2(および spirit 以外の全ロボット)は NMPC を常に **simple モデル**
(胴体 12 状態のみ)で回している
(`agent_reports/quadsdk_step01_mpc_simple_vs_complex.md`)。
Step を平地(01/02)から先へ進めるにあたり、
「simple のままで地形の高さを見た足場選びができるのか」
「穴(gap)を超えられるのか」がよく問われる。本ドキュメントはその答えを
コード根拠つきで整理する。

---

## 概要(結論を先に)

- **足場選び(x,y,z 選定 + 通行可否判定)は simple / complex に関係なくできる。**
  足場を決めるのは NMPC ではなく `local_footstep_planner` で、モデル複雑度に
  依存しない。地形マップ(`z_inpainted` / `traversability`)を見て着地点を
  決め、穴・段差を避ける。
- **地形高さは simple NMPC にも部分的に入る**: `ref_ground_height_` が
  胴体高さのソフト下限になる。ただし**足先が地形に当たらないことを MPC 制約で
  保証する経路(`foot_height_leg_*`)は complex 専用、しかも既定オフ**
  (`use_terrain_constraint_ = false` ハードコード)。
- **狭い穴の「踏み越え」(飛ばない)は simple で原理的に可能**。ただし穴幅は
  足場探索半径 `foothold_search_radius = 0.25 m` + 歩幅で頭打ち。
- **「飛び越え」(flight phase が必要な穴)は simple では非推奨・未検証**:
  - leap の primitive は global body planner(`reference:=gbpl`)経由でしか
    生成されず、Step 01/02 の `reference:=twist` では**そもそも発生しない**。
  - simple でも弾道飛行の**表現**はできるが、打ち上げ力が GRF 上限 150 N/脚に
    制限され、関節限界・膝高さ・FK 整合の制約が無い。
  - Quad-SDK の leap は **adaptive / mixed complexity 前提**で、それは
    **spirit 専用**(`if (robot_ns_ != "spirit") enable_mixed_complexity_ = false`)。
- **追加の注意**: `gap_*` world は詳細メッシュ(`.stl` + `<geom type="mesh">`)で、
  `big_flat.xml` で原因不明の不安定化を起こしたのと同じ経路(handoff 9節)。

---

## 詳細 1: 高さ考慮の足場選びは simple でできる(が MPC の外)

### 足場を決めるのは `local_footstep_planner`(モデル非依存)

出所: `local_planner/src/local_footstep_planner.cpp: computeFootPlan()`。

- 着地点 (x,y) を Raibert 則(hip 中点 + 遠心力補償 + キャプチャポイント)で決定
- **z を地形にスナップ**:
  ```
  foot_position_nominal.z() =
      terrain_grid_.atPosition("z_inpainted",
          getClosestPositionInMap(foot_xy), INTER_NEAREST) + toe_radius_;
  ```
- **通行可否判定**: `getNearestValidFoothold()` が
  `traversability` レイヤを見て `> foothold_obj_threshold`(go2 は 0.6)の
  最寄り点へ着地点をずらす。穴・急斜面はここで除外される。
- これらは NMPC の simple / complex と無関係。着地点は NMPC には
  **固定パラメータ**(`foot_pos_body_`)として渡るだけ。

### 地形高さが simple NMPC に入る唯一の経路

出所: `local_planner.cpp:308/393/427/430`(`getTerrainHeight` 呼び出し)、
`nmpc_controller.cpp: computeLegPlan(..., ref_ground_height, ...)`、
`quad_nlp.cpp:293`。

- `getTerrainHeight(x, y)` は `z_smooth` レイヤを線形補間して返す
  (`local_footstep_planner.hpp:188`)。
- これが `ref_ground_height_` になり、`computeLegPlan` → `update_solver` で
  `ground_height_` に入る。
- `get_bounds_info()`:
  ```cpp
  get_slack_constraint_vals(g_l_matrix, i)(2, 0) = ground_height_(0, i);
  ```
  → **胴体高さ(状態の 3 番目)のソフト下限**が地形高さになる。
  「地面よりも下に胴体が沈む」解にペナルティがかかる。

### simple ではできないこと(complex 専用)

- **遊脚が地形に当たらないことの MPC 制約** = `foot_height_leg_*`。
  complex モデルのみ、かつ `use_terrain_constraint_ = false`
  (`quad_nlp.hpp:173` ハードコード)なので complex でも既定オフ。
- **MPC が着地点そのものを地形に合わせて微修正すること** = 足先が決定変数
  なのは complex のみ。
- **不整地での関節 ↔ 足先の順運動学整合**(`fk_pos_*` / `fk_vel_*`) = complex 専用。

> 要するに「地形高さを見て足場を選び、その高さで胴体を支える」までは simple で
> 回る。実際 Step 01(`flat_wide.xml`)はこのパスで動いている(平地なので
> `z ≈ 0` でほぼ縮退)。「MPC の中で足場高さを拘束・最適化する」までは simple 不可。

---

## 詳細 2: 穴超え —「踏み越え」と「飛び越え」を分ける

### ケース A: 狭い穴を歩容のまま踏み越える(飛ばない)→ simple で原理的に可能

- 穴の検出: フィルタ連鎖の `traversability_hole_mask`
  (`1 − |z_finite − z_inpainted|`。`filter_chain.yaml` filter9)が
  穴を検出して `traversability` を下げる
  (`agent_reports/quadsdk_step01_terrain_map.md`)。
- 足場の回避: `getNearestValidFoothold()` が半径
  `foothold_search_radius = 0.25 m`(go2.yaml)内の有効地点へ着地点をずらす。
- simple NMPC は「穴を跨ぐ着地点」を固定入力として受け取り、通常どおり
  GRF を最適化するだけ。接触スケジュールは通常トロットのまま。
- go2 の前後スタンス長 ≈ 0.36 m、`duty_cycles = [0.5]×4`、対角トロットで
  常時 2 脚支持 → `gap_20cm`(20 cm)程度なら支持形が穴を跨げる見込み。
- **限界**: 穴幅が「足場探索半径 0.25 m + 歩幅の許容」を超えると有効着地点が
  見つからず、足が前位置に留まってつまずく(`local_footstep_planner.cpp`
  の "Foot position is outside the map / Steer the robot in another direction"
  警告経路)。`gap_40cm` 以上は踏み越えでは厳しい。

### ケース B: 飛び越え(flight phase が必要)→ simple では非推奨・未検証

- **leap の primitive は twist モードでは発生しない**:
  - `LEAP_STANCE(1) / FLIGHT(2) / LAND_STANCE(3)` は
    `ref_primitive_plan_` 経由で `computeContactSchedule()` の上書きに使われる
    (`local_footstep_planner.cpp:104-117`)。
  - `ref_primitive_plan_` は `body_plan_msg_->primitive_ids` から埋まる
    (`local_planner.cpp:417`)。`body_plan_msg_` は **global body planner**
    (`reference:=gbpl` + ゴール指定)の出力。
  - Step 01/02 は `reference:=twist` なので `ref_primitive_plan_` は
    `setZero()` = 全 `CONNECT_STANCE(0)` = 通常トロット。
    → **現行設定では飛び越えは起きない**。
- **FLIGHT 中の表現**: simple でも可能。接地脚ゼロのとき
  `eval_f()` の `if (num_contacts > 0)` が false → `u_nom` 全ゼロ、
  `get_bounds_info()` が全脚 GRF 上下限を 0 に潰す
  → 単剛体 EOM が重力のみの弾道運動を積分する。
- **打ち上げ(LEAP_STANCE)の壁**:
  - simple の GRF 鉛直上限は **150 N/脚**(go2.yaml `body.u_ub` の z 成分)。
  - go2 の質量 ≈ 16.1 kg(`global_body_planner.mass`、NMPC が `u_nom` に使用)
    → 静止支持に ≈ 158 N。飛び上がりに要る数×体重の力積が
    150 N cap + 摩擦ピラミッドで頭打ち。
  - 関節限界・膝高さ・FK 整合の制約が simple に無いので、着地衝撃で関節が
    飽和しても MPC は気づけない。
- **Quad-SDK の設計意図**: adaptive / mixed complexity は
  **leap のために作られた**機能
  (`global_body_planner` の `enable_leaping`、`Phase{CONNECT, LEAP_STANCE,
  FLIGHT, LAND_STANCE}`、leap/land 区間の complex 化)。
  しかし `nmpc_controller.cpp`:
  `if (robot_ns_ != "spirit") enable_mixed_complexity_ = false;`
  → **go2 では adaptive complexity が使えない**。
  go2 で安定して飛び越える公式経路は用意されていない。

### 共通の落とし穴: gap world は詳細メッシュ

- `gap_20cm/40cm/80cm` の world xacro は
  `<geom name="floor" type="mesh" mesh="terrain" ...>`(`.stl`)。
  `big_flat.xml` で原因不明の不安定化を起こしたのと同じメッシュ地形経路
  (handoff 9節、`flat_wide.xml` は単純プリミティブで回避した)。
- 地形マップ生成(`GridMapPclConverter` のラスタライズ)と
  MuJoCo のメッシュ衝突判定の両方で脆さの実績がある。

---

## まとめ表

- **`gap_20cm` 程度の踏み越え**
  - simple で可能か: 原理的に可能(未実測)
  - 前提: `reference:=twist` のまま、terrain map が穴を flag、
    着地点が探索半径 0.25 m 内に見つかる
- **`gap_40cm`+ の飛び越え**
  - simple で可能か: 非推奨・未検証
  - 前提: `reference:=gbpl` 必須、GRF 150 N/脚 cap がボトルネック、
    関節/膝/FK 制約なし、adaptive complexity は spirit 専用、
    詳細メッシュ地形の既知の不安定要因

---

## 【事実】と【推測】

### 【事実】(コードで確認済み)

- 足場選定(x,y,z スナップ + `traversability` 判定)は
  `local_footstep_planner` が行い、NMPC の simple / complex に依存しない。
- 地形高さ(`z_smooth` 補間 = `getTerrainHeight`)は `ref_ground_height_`
  → simple NMPC の胴体高さソフト下限(`quad_nlp.cpp:293`)に入る。
- `foot_height_leg_*` / `knee_height_leg_*` は complex 専用制約。
  `use_terrain_constraint_ = false` はハードコード(`quad_nlp.hpp:173`)。
- `LEAP_STANCE/FLIGHT/LAND_STANCE` は `body_plan_msg_->primitive_ids`
  (global body planner)由来。`reference:=twist` では `ref_primitive_plan_`
  は全ゼロ。
- go2 の GRF 鉛直上限 = 150 N/脚(go2.yaml `nmpc_controller.body.u_ub`)。
- `if (robot_ns_ != "spirit") enable_mixed_complexity_ = false;`
  (`nmpc_controller.cpp`)。
- `gap_*` world は `type="mesh"` の `.stl` 地形。

### 【推測】(未検証)

- **`gap_20cm` を twist + simple で実際に踏み越えられるか**は未実行。
  探索半径 0.25 m と 20 cm 穴・go2 の歩幅の関係から「行けそう」と
  推測しているだけ。
- **踏み越え可能な最大穴幅**の実測値は不明。
- **go2 で `reference:=gbpl` + leap を試したときの挙動**(そもそも go2 用の
  leap 軌道が global body planner から出るのか、simple NMPC が
  LEAP_STANCE の GRF 要求を 150 N cap 内で満たせるのか)は未検証。
- **`use_terrain_constraint_` を true にビルドし直したときの complex の挙動**
  も未検証(go2 用 complex 生成コードの存在自体が不明。
  `agent_reports/quadsdk_step01_mpc_simple_vs_complex.md` の【推測】参照)。

---

## その後(穴・段差の Step に進むとき)

1. **まず `gap_20cm` を現行設定(twist + simple + 詳細メッシュ)で実行**して、
   踏み越えの可否と `local_footstep_planner` の警告有無を確認する。
   不安定化するなら world を単純プリミティブで作り直す
   (`flat_wide.xml` と同じ回避策)。
2. 踏み越えの限界を超える穴に進むなら:
   - `reference:=gbpl` + ゴール指定で global body planner を有効化し、
     leap primitive が go2 で生成されるか確認する。
   - GRF 上限 150 N/脚、`friction_coefficient`、`max_wall_time` を
     leap 用に見直す必要が出る(Step の制約次第)。
   - go2 で complex/adaptive を使えるようにするには生成コードの追加が要る
     可能性が高い。
3. 監視: `plan_nmpc_iterations` / `plan_compute_time_ms` /
   `RobotPlanDiagnostics.complexity_schedule`。

---

## ソース早見表(`external/quad-sdk/`)

- 足場選定(モデル非依存)
  - `local_planner/src/local_footstep_planner.cpp: computeFootPlan()`
    (Raibert + z スナップ + `getNearestValidFoothold`)
  - `local_planner/include/local_planner/local_footstep_planner.hpp:188`
    (`getTerrainHeight` = `z_smooth` 補間)
- 穴・接触スケジュール
  - `local_planner/src/local_footstep_planner.cpp:84-117`
    (`computeContactSchedule` と `LEAP_STANCE/FLIGHT/LAND_STANCE` 上書き)
  - `local_planner/include/local_planner/local_footstep_planner.hpp:422-431`
    (`CONNECT_STANCE=0 / LEAP_STANCE=1 / FLIGHT=2 / LAND_STANCE=3`)
  - `local_planner/src/local_planner.cpp:291,417`(`ref_primitive_plan_` の埋め方)
  - `quad_utils/config/filter_chain.yaml`(filter9 `traversability_hole_mask`)
- simple NMPC への地形の入り方
  - `nmpc_controller/src/quad_nlp.cpp:293`(`ground_height_` → 胴体高さ下限)
  - `nmpc_controller/include/nmpc_controller/quad_nlp.hpp:170,173`
    (`always_constrain_feet_ = false` / `use_terrain_constraint_ = false`)
- leap / complexity
  - `global_body_planner/include/global_body_planner/planning_utils.hpp:97,179`
    (`enable_leaping` / `enum Phase`)
  - `nmpc_controller/src/nmpc_controller.cpp`
    (`if (robot_ns_ != "spirit") enable_mixed_complexity_ = false`)
- world
  - `quad_simulator/quad_sim_scripts/worlds/gap_{20,40,80}cm.xml.xacro`
    (`<geom type="mesh">`)
  - `quad_simulator/quad_sim_scripts/models/gap_*/meshes/*.stl`
- パラメータ
  - `quad_utils/config/go2.yaml`
    (`foothold_search_radius = 0.25`、`foothold_obj_threshold = 0.6`、
    `nmpc_controller.body.u_ub` の GRF 上限 150、`global_body_planner.mass = 16.1`)
