# Quad-SDK 穴対応:Foot Placement と NMPC 連携のコード解析

対象:`external/quad-sdk`(Go2、MuJoCo、`reference:=twist`)。
指示書:`chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md`。

**本レポートはコード解析のみ。コードは変更していない。** 実装(Phase 1〜6)は
本レポートのユーザー確認後に、1変更=1目的で順に行う。

分類の凡例:
- **実験事実** … 資料(`agent_reports/steps/step_03_04_1m_*.md`)に記録された実測。
- **コード事実** … 下記ファイルの関数・行から確認できること。
- **推測** … 上記から辻褄は合うが未検証。

---

## 0. 読んだファイルと基準

**基準**:ブランチ `main`、HEAD `e3a0805`、未コミット差分は指示書ファイルのみ。
指示書の upstream 基準 `a3591a9f…` は、この repo では `external/quad-sdk` が
**ベンダリング**されており独立 git 履歴が無い(コミット照合不可、コード実体で確認)。

**資料**(指示書 3 節。step docs は今回の整理で `docs/steps/` → `agent_reports/steps/` へ移動済み):
`agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`、
`agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md`、
`agent_reports/quadsdk_step01_control_pipeline.md`、
`agent_reports/quadsdk_step01_terrain_map.md`、
`agent_reports/quadsdk_step01_mpc.md`。

**コード**:

| 分類 | ファイル | 読んだ関数 |
|---|---|---|
| Terrain Map | `quad_utils/config/filter_chain.yaml` | 全 18 フィルタ |
| | `quad_utils/src/mjcf_to_grid_map_converter.cpp` | `meshToGridMap`(PLY→grid_map、`GridMapPclConverter`) |
| | `quad_utils/src/fast_terrain_map.cpp` | `loadDataFromGridMap`(158-)、`getGroundHeight`(255-)、`getSurfaceNormalFiltered`(345-) |
| Foot Placement | `local_planner/src/local_planner.cpp` | `initLocalFootstepPlanner`(157-)、`terrainMapCallback`(208-)、`getReference`(275-)、`computeLocalPlan`(514-)、`spin`(658-) |
| | `local_planner/src/local_footstep_planner.cpp` | `setTemporalParams`(8-)、`computeContactSchedule`(84-)、`cubicHermiteSpline`(121-)、`computeFootPlan`(160-)、`getNearestValidFoothold`(523-)、`computeSwingApex`(685-)、`getFootPositionsBodyFrame`(61-) |
| | `local_planner/include/local_planner/local_footstep_planner.hpp` | `getTerrainHeight`(188-)、`getTerrainSlope`(196-, 212-) |
| | `local_planner/config/local_planner.yaml`、`quad_utils/config/go2.yaml` | gait / NMPC パラメータ |
| NMPC | `nmpc_controller/src/nmpc_controller.cpp` | `NMPCController`(ctor、`friction_coefficient` の読み込み 57、`enable_mixed_complexity_` の go2 無効化 206)、`computeLegPlan`(278-) |
| | `nmpc_controller/src/quad_nlp.cpp` | `get_bounds_info`(236-)、`eval_f`(478-)、`eval_grad_f`、`eval_g`(≈545-)、`update_solver`(全体) |
| | `nmpc_controller/scripts/dynamicsModel.m` | 全 139 行(離散 EOM + 摩擦錐の CasADi 生成) |
| | `quad_utils/launch/planning.py` | `launch_local_planner`(196-216)のパラメータファイル読み込み順 |
| 下流 | `robot_driver/src/controllers/inverse_dynamics_controller.cpp` | `computeLegCommandArray`(8-72)= plan age 判定 + stale bail |
| | `robot_driver/src/robot_driver.cpp` | `computeLegCommandArray()==false` 時の stand fallback(≈ 795-830)、`leg_controller_` 初期化(273-328) |
| | `robot_driver/src/estimators/comp_filter_estimator.cpp` | `body.twist.angular = imu.angular_velocity`(45、"IMU is in body frame" 61) |
| | `quad_utils/include/quad_utils/quad_kd2.hpp` | FK/IK 在庫(`worldToFootIKWorldFrame` 317、`legbaseToFootIKLegbaseFrame` 330) |

**未読(必要時に追加で読む)**:`quad_nlp.cpp` の adaptive-complexity 分岐、
`quad_kd2.cpp` の IK 実装本体(FK/IK の宣言と bool 戻り値は確認済み、数値解法の中身は未読)、
`robot_driver` の joint_controller、MuJoCo ハーネスで使う推定器
(`state/ground_truth` か `mujoco_estimator` か)の並進速度フレーム。

---

## 1. 資料 ⇔ コード 照合表(指示書 6.1 の 12 判定 + 資料 4 節)

| # | 主張 | 分類 | コード上の根拠 | 判定 | 補足 |
|---|---|---|---|---|---|
| 1 | `reference="twist"` では Global Body Planner を使わない | コード事実 | `quad_utils/launch/planning.py:90` `launch_global_planner` が `reference=='gbpl'` のときだけ `global_body_planner_node` を起動 | 一致 | twist では `body_plan_msg_` は来ず、`getReference` の `use_twist_input_` 分岐(`local_planner.cpp:320-433`)で cmd_vel を前進積分 |
| 2 | `twist` でも terrain map による足場補正を使う | コード事実 | `local_planner.cpp:241-242` `updateMap(terrain_)` / `updateMap(terrain_grid_)` は `reference` 非依存。`computeFootPlan`→`getNearestValidFoothold`(`local_footstep_planner.cpp:273, 523`)も常時実行 | 一致 | 「twist=地図を使わない」ではない(指示書 8 番) |
| 3 | gait は地形に応じて自動変更されない | コード事実 | `setTemporalParams`(`local_footstep_planner.cpp:8-34`)は `initLocalFootstepPlanner`(`local_planner.cpp:157-207`)から起動時 1 回。`nominal_contact_schedule_` を作り、以後 `period_`/`duty_cycles_`/`phase_offsets_` は read のみ。`computeContactSchedule`(84-101)は tiling のみ | 一致 | 例外は `LEAP_STANCE`/`FLIGHT`/`LAND_STANCE`(102-118)。これは `ref_primitive_plan_`(=global planner 由来)でのみ発火。twist では全ゼロ |
| 4 | 穴上の NaN は候補判定で無効になる | コード事実 + 実行ログ | `getNearestValidFoothold`(`local_footstep_planner.cpp:548-565`):`traversability = terrain_grid_.atPosition("traversability", pos)`、`if (traversability > foothold_obj_threshold_ && kin_cost < best)` のみ採用。`NaN > 0.6` は false → 却下。gbpl 実行ログの `[DIAG] gnvf` に `nominal x=1.006 trav=nan -> snapped x=1.146` | 一致 | NaN が「その各フィルタでどう伝播したか」は §3 で別途扱う。ここは「最終 `traversability` レイヤ上で NaN のセルは却下される」という事実 |
| 5 | `foothold_search_radius` 内で有効セルを探索する | コード事実 | `getNearestValidFoothold`(538-540):`grid_map::SpiralIterator iterator(terrain_grid_, pos_center_aligned, foothold_search_radius_)` を渦巻き走査 | 一致 | `foothold_search_radius: 0.7`(`go2.yaml`。素は 0.25) |
| 6 | 足場の評価関数は距離だけで、IK 可到達性を含まない | コード事実 | `getNearestValidFoothold:549-551` `kin_cost = ‖p−p_nom‖ + 0.5‖p−p_prev_solve‖`。他項なし | 一致 | 足裏面積・縁距離・map 誤差・関節限界・支持多角形・map 鮮度も無し(指示書 8.3) |
| 7 | toe 半径は水平安全距離の判定に使われない | コード事実 | `toe_radius_` の使用箇所:`getNearestValidFoothold:594` `foot_position_best.z() = z_inpainted + toe_radius_`(**z のみ**)、`local_planner.cpp:544-549` `grf_positions_*.col(3i+2) -= toe_radius_`(GRF 点の z 補正)。水平判定に登場しない | 一致 | `toe_radius: 0.022`(`go2.yaml`) |
| 8 | 有効足場がない場合、名目足場を返す | コード事実 | `getNearestValidFoothold:526` `foot_position_best = foot_position`(初期化=nominal)。`568-573` `best_kin_cost==max` のとき `RCLCPP_WARN_THROTTLE("No valid foothold found …, returning nominal")` して `return foot_position_best`(=nominal) | 一致 | この WARN_THROTTLE は第1引数 `1e9` ns → ミリ秒扱いで sim 時刻が小さいと**実質出ない**(§4.3 補足)。成功/失敗が下流へ伝わらない ← Phase 1/2 の対象 |
| 9 | Go2 の NMPC では足場位置を最適化しない | コード事実 | `dynamicsModel.m:40-41` `p=[dt; mu; feet_location]`(**パラメータ**)、`w=[x0; u; x1]`(決定変数)。`quad_nlp.cpp:eval_g` は `pk.segment(14,12)=foot_pos_world_.row(i+1)` を CasADi 関数へ**パラメータ渡し**。`nmpc_controller.cpp:206` `if (robot_ns_ != "spirit") enable_mixed_complexity_ = false` → go2 は 12 状態 simple model のみ | 一致 | complex/feet モデル(spirit)は足位置を状態に持つ(`go2.yaml` `feet:` ブロック)が go2 では未使用 |
| 10 | Go2 の NMPC には脚の IK 可到達制約がない | コード事実 | `dynamicsModel.m:43` `g = [EOM; friction]` のみ。`quad_nlp.cpp` に関節角・足到達性の制約なし。`get_bounds_info`(236-284)は入力境界(接触脚 `f_z∈[10,150]`、遊脚 `f=0`)+ EOM 等式 + 摩擦錐のみ | 一致 | §6.1(資料の「脚可到達制約が破れた」記述の訂正)へ |
| 11 | `horizon_length > period_` はコード上の必須条件か | コード事実 | `computeContactSchedule:99` `nominal_contact_schedule_[(i + phase) % period_]`(i∈[0,horizon_length_))。剰余ラップするので **horizon と period_ の大小に必須条件はない**。`computeFutureBodyPlan`(`local_footstep_planner.cpp:197-200, 349-352`)は stance 窓がホライズンを超える場合に胴体プランを外挿するもので、period_ とは独立 | **不一致(要分離)** | 「26→40 で改善」は実験事実。「horizon>period_ が必須」はコード上の事実ではない(§6.2) |
| 12 | 実センサの穴が必ず NaN になる保証があるか | コード事実 | この repo に LiDAR/深度カメラ → `z` レイヤの処理は**無い**。`mjcf_to_grid_map_converter.cpp` が静的 PLY を `GridMapPclConverter::addLayerFromPolygonMesh` でラスタライズするのみ。面が無いセルが NaN になるのは PLY 由来 | **未確認** | no-return / occlusion / 未観測 / 期限切れの扱いは未検証(§6.3) |
| A | `twist` + trot → crawl で深さ1m・幅0.3m の穴を複数連続で通過(0.15/0.3/0.5 m/s、5〜6本) | 実験事実 | ― | ― | `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` §5。CSV+GIF 確認済み |
| B | `period 0.9` / `duty [0.75]×4` / `phase [0,0.75,0.5,0.25]` / `horizon_length 40` / `foothold_search_radius 0.7` / `ground_clearance 0.1` | 実験事実(設定値) | `go2.yaml` `local_footstep_planner:`、`local_planner.yaml` `horizon_length` | 一致 | 現在の main のコミット値と一致 |
| C | 地形 PLY の穴を物理穴より左右 0.05 m 広く。穴部の生 `z`=NaN、穴上 `traversability`=NaN | 実験事実 | PLY 生成:`src/trial/assets/gen_quadsdk_gap_world.py`。NaN は実行ログの `[DIAG] gnvf … trav=nan` で確認 | 一致 | §3 で伝播を詳述 |
| D | nominal `x=1.006, trav=NaN` → selected `x=1.146` / nominal `x=0.940` → selected `x=0.890` | 実験事実 | `getNearestValidFoothold` の DIAG(`[MPC_DOG DIAG] gnvf`) | 一致 | ― |
| E | 記録中 `found=0`(有効候補なし)は 0 件 | 実験事実 | 同上 DIAG(`found=` フィールド) | 一致 | 「静的既知 PLY・正確な位置合わせ・手作業マージン」下での 0 件。実センサでの保証ではない |

---

## 2. Terrain Map の処理(数式とコード)

### 2.1 生成経路

物理ワールド(`worlds/<name>.xml.xacro`、box 凸条 + トレンチ)とは**別**に、
`models/<name>/meshes/<name>.ply` から grid_map を作る
(`mjcf_to_grid_map_converter.cpp:meshToGridMap`、`GridMapPclConverter::
initializeFromPolygonMesh` + `addLayerFromPolygonMesh`、`grid_map_resolution: 0.05`)。
**メッシュに面が無いセルの `z` は NaN のまま残る**(実行ログ
`addLayerFromPolygonMesh -> true, finite=57800/67400`、差 9600 が穴)。
raw マップ `/mapping/terrain_map_raw` が `filter_chain.yaml` を通って
`/mapping/terrain_map` になる。

### 2.2 フィルタ連鎖(`filter_chain.yaml`、実行順)

| # | 名前 | type | 入力 → 出力 | 目的 |
|---|---|---|---|---|
| 1 | `duplicate_z` | Duplication | `z` → `z_finite` | 生高さの複製 |
| 2 | `inpaint` | gridMapCv/Inpaint | `z` → `z_inpainted`(radius 0.4) | 穴を補間で埋める |
| 3 | `mean_in_radius` | MeanInRadius | `z_inpainted` → `z_smooth`(radius 0.2) | 平滑高さ |
| 4 | `surface_normals` | NormalVectors | `z_inpainted` → `normal_vectors_{x,y,z}`(radius 0.15) | 法線 |
| 5 | `smooth_surface_normals` | NormalVectors | `z_smooth` → `smooth_normal_vectors_{x,y,z}`(radius 0.4) | 平滑法線 |
| 6 | `slope` | MathExpression | `normal_vectors_z` → `slope` | \(\mathrm{slope}=\arccos(n_z)\) |
| 7 | `roughness` | MathExpression | `z_inpainted`,`z_smooth` → `roughness` | \(\mathrm{roughness}=|z_{\mathrm{inpainted}}-z_{\mathrm{smooth}}|\) |
| 8 | `z_finite_threshold` | Threshold | `z`(下限 −1000、`set_to −1000`)→ `z_finite` | −1000 未満を −1000 に。**NaN は対象外**(閾値比較が false) |
| 9 | `traversability_hole_mask` | MathExpression | `z_finite`,`z_inpainted` → `traversability_hole_mask` | \(H=1-|z_{\mathrm{finite}}-z_{\mathrm{inpainted}}|\) |
| 10 | `traversability_filter` | MeanInRadius | `H` → `H_filtered`(radius 0.075) | 穴マスクを外へ広げる |
| 11 | `..._lower_threshold` | Threshold | `H_filtered`(下限 0、`set_to 0`) | 負を 0 に |
| 12 | `traversability` | MathExpression | `roughness`,`slope` → `traversability` | \(T_{\mathrm{shape}}=0.5(1-\mathrm{roughness}/0.1)+0.5(1-\mathrm{slope}/0.4)\) |
| 13 | `..._lower_threshold` | Threshold | `traversability`(下限 0) | 負を 0 に |
| 14 | `traversability_apply_hole_mask` | MathExpression | → `traversability` | \(T=(T_{\mathrm{shape}}+0.02)\cdot H_{\mathrm{filtered}}\) |
| 15 | `..._upper_threshold` | Threshold | `traversability`(上限 1、`set_to 1`) | 1 で頭打ち |
| 16 | `delete` | Deletion | `z_finite`,`H`,`H_filtered` を削除 | 中間層の掃除 |
| 17 | `duplicate` | Duplication | `traversability` → `traversability_mask` | 複製 |
| 18 | `..._mask_lower_threshold` | Threshold | `traversability_mask`(下限 **0.5**、`set_to 0`) | 0.5 未満を 0 に。コメントに「`/local_footstep_planner/foothold_obj_threshold` に一致させること」とあるが、実際の値は **0.5 ≠ 0.6**(§2.5) |

### 2.3 NaN 伝播(式だけで断定せず、実行ログで確認したこと)

- **フィルタ 8** は `ThresholdFilter`。生 `z` が NaN のセルは「−1000 未満」を満たさず
  素通り → `z_finite` は NaN のまま(**コード事実:実行ログの下流結果から逆算**。
  指示書の想定「−1000 sentinel 化」は起きていない)。
- **フィルタ 9**:\(H=1-|{\rm NaN}-z_{\rm inpainted}|\) = NaN。
- **フィルタ 10**(`MeanInRadius`):近傍平均。NaN セルの近傍が全 NaN なら NaN、
  一部有限なら有限値。→ **穴の中心は NaN、縁は有限**(実測:barrier 半径を
  0.075→0.2 に広げても snapped x が変わらなかった。`MeanInRadius` は NaN を
  広げられない。`agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` §3.3)。
- **フィルタ 14**:\((T_{\rm shape}+0.02)\cdot{\rm NaN}\) = NaN。→ 最終 `traversability` は
  **穴帯で NaN**(実測:`[DIAG] gnvf … trav=nan`)。
- 結論:穴対応の実効メカニズムは
  「**メッシュに実穴 → 生 `z`=NaN → `z_inpainted` と食い違う → `H`=NaN →
  `traversability`=NaN → `getNearestValidFoothold` が却下**」。
  段差/ランプ/ジグザグでは生 `z` が NaN にならず発火しない(実験事実、同 §3.1)。

### 2.4 2 つの地形表現(重要)

| 表現 | 生成 | 使う関数 | 穴の扱い |
|---|---|---|---|
| `terrain_grid_`(`grid_map::GridMap`) | `terrainMapCallback` で `fromMessage`(`local_planner.cpp:210`) | `getNearestValidFoothold`(`updateMap(terrain_grid_)`)、NMPC へ `computeLegPlan(… terrain_grid_ …)` | `traversability` レイヤに **NaN が残る** |
| `terrain_`(`FastTerrainMap`) | `terrain_.loadDataFromGridMap(terrain_grid_)`(`local_planner.cpp:237`) | `getTerrainHeight`(`z_smooth`)、`getTerrainSlope`(`smooth_normal_vectors_*`) → **胴体の高さ・傾き参照** | `loadDataFromGridMap`(`fast_terrain_map.cpp:186`)が **`z_inpainted` を読む**。穴は埋まった状態。NaN 無し |

→ 足場計画は穴を見る。**胴体参照は穴を見ない(埋めた地形を見る)。**
凸条を完全に水平・同一高さの平面にすると、`getTerrainSlope` が偽のピッチ指令を
出さない(実験事実、gbpl doc §3.2 / gap_crossing doc §3.2)。

### 2.5 `traversability_mask` の閾値 0.5 と `foothold_obj_threshold` 0.6 の不一致

- **コード事実**:`filter_chain.yaml` フィルタ 18 は `traversability_mask` の下限を
  **0.5** にし、コメントで「footstep planner の `foothold_obj_threshold` に
  一致させること」と書いている。だが `local_planner.yaml` の
  `foothold_obj_threshold` は **0.6**(§3.3)。→ **不一致**。
- **現挙動への直接影響はない**:`getNearestValidFoothold` が読むのは
  `obj_fun_layer_ = traversability`(`local_planner.yaml` `obj_fun_layer: traversability`)
  であり、`traversability_mask` **ではない**。しきい値比較(`> 0.6`)も
  この関数の中で行う。`traversability_mask` レイヤは足場選択の経路では使われない。
- したがってこの不一致は「潜在的な設定齟齬(将来 `traversability_mask` を使う
  コードを足すと 0.5 で切られてしまう)」であり、今の穴対応の挙動は変えない。

---

## 3. Foot Placement Control の入出力

### 3.1 接触スケジュール(`computeContactSchedule`)

- `setTemporalParams(dt, period, horizon_length, duty_cycles, phase_offsets)`
  (`local_footstep_planner.cpp:8-34`)が起動時に
  `nominal_contact_schedule_`(長さ `period_ = period/dt` の固定表)を作る:

  各位相 `i∈[0,period_)`、脚 `leg_idx` について
  \[
  c_{i,\,leg}=
  \begin{cases}
  1 & i\ge period_\cdot\phi_{leg}\ \text{かつ}\ i< period_\cdot(\phi_{leg}+d_{leg})\\
  1 & i< period_\cdot(\phi_{leg}+d_{leg}-1)\quad(\text{ラップ})\\
  0 & \text{otherwise}
  \end{cases}
  \]
  \(\phi\)=`phase_offsets`、\(d\)=`duty_cycles`。

- `computeContactSchedule`(84-101):`phase = current_plan_index % period_`、
  各ホライズン点 `i∈[0,horizon_length_)` に
  `contact_schedule[i] = nominal_contact_schedule_[(i+phase) % period_]` を貼るだけ。
  **`STAND` モードなら全脚 `true`。**
- **twist で地形に応じて接触スケジュールを変更する処理は無い**(検索結果:
  `computeContactSchedule` 内の地形依存分岐は `LEAP_STANCE`/`FLIGHT`/`LAND_STANCE`
  のみ、いずれも `ref_primitive_plan` 由来 = global planner)。
- Go2 現行値:`period 0.9`(→`period_ = 30`)、`duty [0.75]×4`、
  `phase [0.0, 0.75, 0.5, 0.25]`(FL→BR→FR→BL の横回りクロール、常時 3 脚接地)。

### 3.2 名目足場(`computeFootPlan`、`local_footstep_planner.cpp:160-284`)

各脚 `j`、ホライズン内の新規接地イベント `i`(`isNewContact`)で:

1. **中間姿勢の hip 位置**:接地窓 `k∈[i, end_of_stance)` にわたって
   `quadKD_->worldToNominalHipFKWorldFrame(j, body_pos(k), body_rpy(k), hip_k)` を計算し、
   その 2D 点群 `P` の **最小包含円の中心** `welzlMinimumCircle(P)` を
   `hip_position_midstance` とする(198-227)。stance 窓がホライズンを超える分は
   `computeFutureBodyPlan` で外挿(197-200)。
2. **動的オフセット**(234-243):
   \[
   h=\max\!\big(p_{b,z}(i)-z_{\mathrm{inpainted}}(p_{b,xy}(i)),\,0\big)
   \]
   \[
   c_{\mathrm{centrifugal}}=\frac{h}{g}\,\big(v_{b}(i)\times\omega_{\mathrm{ref}}(i)\big),\qquad
   c_{\mathrm{vel}}=\sqrt{\tfrac{h}{g}}\,\big(v_{b}(i)-v_{\mathrm{ref}}(i)\big)
   \]
   `v_b = body_plan.block<1,3>(i,6)`(**現在**の胴体並進速度、cols 6-8)、
   `v_ref = ref_body_plan.block<1,3>(i,6)`(**参照**並進速度)、
   `ω_ref = ref_body_plan.block<1,3>(i,9)`(参照角速度、cols 9-11)。
   **座標系(要注意)**:`dynamicsModel.m:66` は NMPC 状態を
   「並進速度 `p_dot` = **world 系**、角速度 `omega` = **body 系**」と定義している
   (`feet_location` は「foot-to-body ベクトルの **world 系**」)。`body_plan_` /
   `ref_body_plan_` は NMPC の状態規約に従うので、cols 6-8 = 並進速度 world、
   cols 9-11 = 角速度 **body**。よって `centrifugal = h/g·(v_b × ω_ref)` は
   world 並進 × body 角速度 の混在積になる。以前の「すべて world 系」は誤り。
   なお **RobotState → `current_state_`(`bodyStateMsgToEigen`)の並進速度が
   どのフレームか**は本レポートでは未確定。角速度は
   `comp_filter_estimator.cpp:45,61` で IMU=body 系と分かるが、MuJoCo ハーネスの
   推定器(`state/ground_truth` か `mujoco_estimator` か)と並進速度フレームは
   未追跡 ← **未確認**。
3. **名目足場**(249-251):
   \[
   p_{\mathrm{nom}} = p_{\mathrm{hip,midstance}} + c_{\mathrm{centrifugal}} + c_{\mathrm{vel}}
   \]
   これは Raibert 型。地形は 2. の `h`(埋めた地形の高さ)にしか入らない。
4. z の暫定値(264-269):`p_nom.z = z_inpainted(closest_in_map(p_nom.xy)) + toe_radius`。
5. **map 外判定は `getNearestValidFoothold` の手前で行われる**(`computeFootPlan:255-261`):
   `if (!terrain_grid_.isInside(foot_position_grid_map)) { RCLCPP_WARN("Foot
   position is outside the map …"); continue; }`。この `continue` で
   **その接地イベントの処理自体を打ち切り**、`getNearestValidFoothold` は
   呼ばれない。→ **`getNearestValidFoothold` の戻り値(将来の `FootholdResult`)
   だけでは map 外を `found=false` として表現できない。** map 外の失敗伝播は
   この `continue` 地点に別途足す必要がある(§8 Phase 2 の注記)。

### 3.3 地図による足場補正(`getNearestValidFoothold`、523-596)

候補集合(概念):
\[
\mathcal{P}=\{\,p \mid \|p-p_{\mathrm{nom}}\|\le R,\ T(p)>T_{\min}\,\},\qquad
R=\texttt{foothold\_search\_radius}=0.7,\quad T_{\min}=\texttt{foothold\_obj\_threshold}=0.6
\]
選択(549-565):
\[
p^{*}=\arg\min_{p\in\mathcal{P}}\Big[\ \|p-p_{\mathrm{nom}}\| \;+\; 0.5\,\|p-p_{\mathrm{prev\_solve}}\|\ \Big]
\]
- 走査は `grid_map::SpiralIterator(terrain_grid_, pos_center_aligned, R)`。
  サブセル offset を保持したまま各セルを見る(529-542)。
- `T(p) = terrain_grid_.atPosition("traversability", p)`。`NaN > 0.6` は false → 却下。
- 高さ(591-594):\(p^{*}_z = z_{\mathrm{inpainted}}(p^{*}_{xy})\,(\text{INTER\_LINEAR}) + r_{\mathrm{toe}}\)。
- **有効候補が無い場合**(568-573):`best_kin_cost == max` のまま。
  `RCLCPP_WARN_THROTTLE(…, 1e9, "No valid foothold found …, returning nominal")` の後
  `foot_position_best`(=nominal)を返す。
  - 補足:この `WARN_THROTTLE` は第1引数に `1e9`(ns のつもり)を渡すが
    ROS2 の当該マクロはこれを **ミリ秒**として扱う。sim 時刻が小さいうちは
    「1e9 ms 経過」の条件を満たさず**実質ログが出ない**。
    既存コードには `[MPC_DOG DIAG]` の別ログ(`% 40` ゲート)が入っている。

**この関数が評価していないもの(コードに存在しない)**:足裏面積、穴の縁までの
距離、map 位置誤差、IK 可到達性、関節角度/速度限界、支持多角形、地図の時刻・鮮度。

### 3.4 出力

`computeFootPlan` は `foot_positions_world_`(N×12)、`foot_velocities_world_`、
`foot_accelerations_world_` を埋める。`loadFootPlanMsgs`(473-521)がメッセージ化。

| 変数 / メッセージ | 型・形状 | 座標系・単位 | 生成/publish |
|---|---|---|---|
| `contact_schedule_` | `vector<vector<bool>>` [horizon_length_][4] | ― | `computeContactSchedule` |
| `foot_positions_world_` | `Eigen::MatrixXd` N×12(脚順 FL,BL,FR,BR、各 xyz) | world [m] | `computeFootPlan` |
| `foot_positions_body_` | N×12 | **胴体原点からの相対ベクトルを world 軸で表現** [m] | `getFootPositionsBodyFrame`(`local_footstep_planner.cpp:61-69`)= `foot_positions_world − body_plan.segment<3>(0)`。**姿勢回転(`R_wb^⊤`)を掛けていない**ので真の body フレームではない。名前が紛らわしい |
| `foot_velocities_world_` | N×12 | world [m/s] | `computeFootPlan`(swing 補間の微分) |
| `foot_accelerations_world_` | N×12 | world [m/s²] | `computeFootPlan` |
| `grf_positions_body_/world_` | N×12 | body/world [m] | `computeLocalPlan`:`foot_positions_* − (0,0,toe_radius)` を NMPC へ |
| discrete foot plan | `quad_msgs/MultiFootPlanDiscrete` | world | `loadFootPlanMsgs`。`isNewContact` の点のみ push |
| continuous foot plan | `quad_msgs/MultiFootPlanContinuous` | world | `loadFootPlanMsgs`。各ホライズン点、`traj_index` と `stamp` 付き |

**遊脚軌道**(`computeFootPlan:394-433`):
- x, y:端点速度を 0 に固定した**三次 Hermite 補間**
  `cubicHermiteSpline(prev, 0, next, 0, phase, duration, pos, vel, acc)`(408-415)。
- z:**上昇/下降の 2 区間**に分け、頂点 `swing_apex` を経由(417-433)。
  `swing_apex = min( ground_clearance − toe_radius + max(prev_z, next_z),  hip_height − hip_clearance )`、
  さらに `max(swing_apex, hip_height − 0.35)`(`computeSwingApex:685-700`)。
  前半 `cubicHermiteSpline(prev_z, 0, apex, 0, …)`、後半 `cubicHermiteSpline(apex, 0, next_z, vel_next_z, …)`。
- 端点が両方 solid(z≈0)にスナップされていれば、遊脚 z は区間中 `[0, apex]` で
  **穴の下へは行かない**(実験事実:成功時 z は穴の上を通過)。

---

## 4. NMPC への受け渡し(`reference:=twist`、Go2 simple model)

### 4.1 関数チェーン

`LocalPlanner::computeLocalPlan`(`local_planner.cpp:514`)

| 関数 | 入力 | 出力 | 足場の扱い | 地図の扱い |
|---|---|---|---|---|
| `computeContactSchedule` | `current_plan_index_`, `body_plan_`, `ref_primitive_plan_`, `control_mode_` | `contact_schedule_` | ― | ― |
| `computeFootPlan` | `contact_schedule_`, `body_plan_`, `grf_plan_`, `ref_body_plan_`, 現在足位置/速度, `first_element_duration_`, `past_footholds_msg_` | `foot_positions_world_` ほか | **決定**(§3.2-3.3) | `terrain_grid_`(traversability)+ `z_inpainted` |
| `getFootPositionsBodyFrame` | `body_plan_`, `foot_positions_world_` | `foot_positions_body_` | 座標変換のみ | ― |
| `computeLegPlan`(NMPC) | `current_full_state`(36), `ref_body_plan_`, `grf_positions_body/world`, `foot_velocities_world_`, `contact_schedule_`, `ref_ground_height_`, `first_element_duration_`, `terrain_grid_` | `body_plan_`(状態列)、`grf_plan_`(GRF 列) | **固定パラメータ**(下記) | `terrain_grid_` は保持するが simple model の制約には未使用 |

`computeLegPlan`(`nmpc_controller.cpp:278-`):
`mynlp_->foot_pos_body_ = -foot_positions_body`、`mynlp_->foot_pos_world_ = foot_positions_world`
を**メンバ行列にコピー**(288-289)してから `update_solver` → IPOPT 求解。
`foot_positions_body`(= `foot_world − body_pos`、world 軸)を **符号反転**して
`foot_pos_body_ = body_pos − foot_world` としている。`dynamicsModel.m:24,67` の
`feet_location` コメント「Foot to body vectors in **world frame**」と一致
(= 足先から胴体原点へ向かう world 軸ベクトル)。`eval_g` では
`pk.segment(2,12) = foot_pos_body_.row(i+1)`、`pk.segment(14,12) = foot_pos_world_.row(i+1)`
をパラメータ渡し。
Go2 では `enable_mixed_complexity_ = false`(206)→ 全ホライズンで simple model。

### 4.2 Simple model の次元(`dynamicsModel.m`、`go2.yaml` `nmpc_controller.body:`)

\[
x_k=\begin{bmatrix}p_b & \theta_b & \dot p_b & \omega_b\end{bmatrix}^{\!\top}\in\mathbb{R}^{12},\qquad
u_k=\begin{bmatrix}f_{FL} & f_{BL} & f_{FR} & f_{BR}\end{bmatrix}^{\!\top}\in\mathbb{R}^{12}
\]
`x_dim: 12`, `u_dim: 12`, `g_dim: 28`(= 12 EOM 等式 + 16 摩擦錐 = 4脚×4面)。

### 4.3 足場位置の役割(パラメータであることの確認)

- `dynamicsModel.m:40` `p = [dt; mu; feet_location]`、`w = [x0; u; x1]`。
  `feet_location` は最適化変数ではない。
- `quad_nlp.cpp:eval_g`:`pk.segment(2,12)=foot_pos_body_.row(i+1)`、
  `pk.segment(14,12)=foot_pos_world_.row(i+1)` を CasADi 生成関数へ渡す(パラメータ)。
- 離散運動方程式(`dynamicsModel.m:29-31`、Backward Euler):
  \[
  \big(q_{k+1}-q_k\big)-\Delta t\,\dot q_{k+1}=0
  \]
  \[
  M(x_{k+1})\big(v_{k+1}-v_k\big)+\Delta t\big[h(x_{k+1})-J_u(p_f)\,u_k\big]=0
  \]
  物理的意味(セントロイダル):
  \[
  m\ddot p_b=\sum_j c_j f_j + m g,\qquad
  I\dot\omega+\omega\times I\omega=\sum_j c_j\,(p_{f,j}-p_b)\times f_j
  \]
  → **足場 \(p_f\) は GRF のモーメントアーム**として EOM に入るのみ。
  Go2 simple NMPC 自体は足場を動かさない。

### 4.4 接触・摩擦・境界(`quad_nlp.cpp:get_bounds_info` 236-284、`go2.yaml`)

- 接触脚 `c_{j,i}=1`:入力境界は `u_lb/u_ub` の該当行。`go2.yaml` `body:`
  `u_lb[2,5,8,11]=10`、`u_ub[2,5,8,11]=150` → **\(10\le f_z\le 150\) N**。
  \(f_x,f_y\) は `±2e19`(自由)、実効的には摩擦錐で拘束。
- 遊脚 `c_{j,i}=0`(258-262):`u_l/u_u` の該当 3 成分に `× contact_sequence_(j,i)` を
  掛ける → `u_lb=u_ub=0` → **\(f_j=0\)**。
- 摩擦(`dynamicsModel.m:33-38`):各脚
  \[
  \begin{bmatrix}1&0&-\mu\\-1&0&-\mu\\0&1&-\mu\\0&-1&-\mu\end{bmatrix}f_j \le 0
  \quad\Longleftrightarrow\quad |f_{j,x}|\le\mu f_{j,z},\ \ |f_{j,y}|\le\mu f_{j,z}
  \]
  **実効 \(\mu\) = 0.6**(下記)。**摩擦ピラミッド**。

  - `nmpc_controller.cpp:57` は `loadROSParam(node_, "nmpc_controller.friction_coefficient", mu)`
    で読む。この値は 2 か所で定義:`nmpc_controller.yaml` に **0.3**、
    `go2.yaml`(`nmpc_controller:` ブロック直下、6 スペース字下げ)に **0.6**。
  - `planning.py:201-204` の `local_planner` ノードの `parameters=[…]` は
    `[local_planner.yaml, nmpc_controller.yaml, local_planner_topics.yaml,
    go2.yaml(robot_specific), {dict}]` の順。**ROS 2 launch は後勝ち**なので
    `go2.yaml`(4 番目)が `nmpc_controller.yaml`(2 番目)を上書きする。
  - → **実効値は 0.6**(nmpc_controller.yaml の 0.3 ではない)。
    `pk[1] = mu_`(`quad_nlp.cpp:eval_g`)として CasADi の摩擦錐へ渡る。
  - **要ライブ確認**:`ros2 param get /robot_1/local_planner nmpc_controller.friction_coefficient`
    で実行時の値を突き合わせること(本レポートは launch 順からの推定)。
- 状態境界(`go2.yaml` `body:` `x_lb/x_ub`):\(p_{b,z}\ge 0\)、
  roll・pitch \(\in[-\pi,\pi]\)(hard)、yaw \(\in[-10,10]\)、他は自由。
  `x_lb_soft/x_ub_soft` も同値でスラック緩和対象。
- 「motor model(関節トルク上限を GRF に射影する制約)」は complex/joints モデル用
  (`quad_nlp.cpp:220-225` `remove_motor_model_in_swing`)。**Go2 simple の
  `g_dim: 28` には含まれない**(28 = EOM12 + friction16)。

### 4.5 目的関数(`quad_nlp.cpp:eval_f` 478-517、`eval_grad_f`)

\[
J=\sum_{k=0}^{N-2}\Big[\tfrac12 (x_{k+1}-x_{k+1}^{\mathrm{ref}})^{\!\top}Q_k(x_{k+1}-x_{k+1}^{\mathrm{ref}})
+\tfrac12 (u_k-u_k^{\mathrm{nom}})^{\!\top}R_k(u_k-u_k^{\mathrm{nom}})\Big]
+ w_{p}\!\!\sum \sigma_{x} + w_{c}\!\!\sum \sigma_{g}
\]
- \(x^{\mathrm{ref}}\):`x_reference_.col(i+1)`(= `ref_body_plan_`。twist では cmd_vel 積分
  + `getTerrainHeight`/`getTerrainSlope` による高さ・傾き。§2.4)。
- \(u^{\mathrm{nom}}\):接地脚に \(mg/n_{\mathrm{contacts}}\) を z 成分へ配分(重力補償)。
- \(Q_k = Q_{\mathrm{complex}}\cdot \kappa_Q^{\,k}\)、\(\kappa_Q=\)`Q_temporal_factor`\(^{1/(N-2)}\)
  (`nmpc_controller.cpp:66`。yaml 値 100 は「ホライズン終端/始端の比」)。
  `go2.yaml` `x_weights = [5,5,5, 0.5,0.5,0.5, 0.1,0.1,0.2, 0.05,0.05,0.01]`、
  `u_weights = 5e-5 ×12`。
- \(w_p\)=`panic_weights: 200`(状態境界スラック)、
  \(w_c\)=`constraint_panic_weights: 20`(制約スラック)。
- **`traversability` は `quad_nlp.cpp` に一切現れない**(検索結果 0 件)。
  Go2 simple model の目的関数・制約は地形通行性に依存しない。

### 4.6 下流(`inverse_dynamics_controller.cpp` + `robot_driver.cpp`)

**plan の鮮度ガード**(`inverse_dynamics_controller.cpp:12-15`):
`if (last_local_plan_msg_ == NULL || (now − header.stamp).seconds() >= 0.1) return false;`。
さらに `t_now` が plan の `[states.front, states.back]` 時刻窓の外なら
`RCLCPP_ERROR("ID node couldn't find the correct ref state!")`(44-68)して `return false`。

**`computeLegCommandArray()==false` 時のフォールバック**(`robot_driver.cpp` ≈ 795-830):
全脚の `motor_commands` を **`stand_joint_angles_` への PD 制御**へ差し替える
(`loadMotorCommandMsg(stand_joint_angles_[j], 0, 0, stand_kp_[j], stand_kd_[j], …)`)。
go2:`stand_joint_angles [0,0.8,-1.5]`、`stand_kp [60,60,60]`、`stand_kd [2,2,2]`。

**plan が新しく有効な場合のみ**(`computeLegCommandArray` 本体、16-181):
- 入力:`last_local_plan_msg_`(NMPC の `body_plan_` + `grf_plan_`)を時刻補間 →
  `ref_body_state`、`grf_array`、`ref_foot_acceleration`、`contact_mode`。
- `grf_array` は指数フィルタ(`grf_exp_filter_const_`)。
- **接地脚**:`quadKD_->computeInverseDynamics(ref_foot_acceleration, grf_array,
  contact_mode, tau_array)` → フィードフォワード関節トルク。`kp/kd = stance_kp_/stance_kd_`。
- **遊脚**:Cartesian PD `swing_cart_fb = swing_kp∘Δp_foot + swing_kd∘Δṗ_foot`、
  `J^{\top}` で関節トルクへ(143-146)。`kp/kd = swing_kp_/swing_kd_`。
- 出力:`leg_command.motor_commands[j]` = `torque_ff` + `kp,kd`。

**したがって「NMPC 失敗後に壊れた GRF がそのままトルク化される」は誤り。** 分けて扱う:

| 事象 | plan の状態 | 下流の挙動 |
|---|---|---|
| **(a) NMPC 求解失敗が継続**(`computeLocalPlan()` が false) | 新しい `local_plan` を publish しない → plan が古くなる | 0.1 s 以内に ID コントローラが `false` → **robot_driver が `stand_joint_angles` へ PD ホールド**(安全側)。`local_planner::spin` は `consecutive_failures_ >= failure_threshold_` で `planner_failed` を publish |
| **(b) NMPC は求解成功だが解の品質が悪い**(cost 大、スラック大、GRF が `f_z=150` に飽和) | plan は新しく publish され、時刻窓内 | ID コントローラが**その GRF をそのままトルク化**(飽和 GRF → 飽和トルク、effort 超過警告)。run7/run8 の**転倒はこの (b) の区間**で起きる。(a) の stand ホールドは既に反転した後に始まる(CSV:`plan_age_s` が 1〜2 s に伸びるのは転倒後) |

- **この段は足場を選ばない・IK 可到達性を見ない。**

---

## 5. 役割分担(初心者向け)

| モジュール | 主入力 | 主出力 | 穴への責任 |
|---|---|---|---|
| Terrain Map | 高さ(PLY 由来)、未観測=NaN | `z_inpainted`, `slope`, `roughness`, `traversability` ほか | 穴/危険領域を `traversability` の低値・NaN として表現する |
| Contact Schedule | gait 設定(`period`/`duty`/`phase`) | 各脚の接地/遊脚時刻(固定表の tiling) | いつ足を上げるか決める。**twist では地形適応しない** |
| Footstep Planner | 胴体予測、`terrain_grid_`、前回足場 | 着地点 `foot_positions_world_`、足先軌道(x/y 三次 Hermite、z 上昇/下降) | Raibert 名目 → `getNearestValidFoothold` で穴上(NaN)を避けて平面へスナップ |
| NMPC(Go2 simple) | 胴体参照 `ref_body_plan_`、**固定足場** `foot_pos_*`、接触時刻、`ref_ground_height_` | 胴体状態列 `body_plan_`、GRF 列 `grf_plan_` | 与えられた足場で EOM + 摩擦 + 状態境界を満たす胴体軌道と GRF を最適化。足場は最適化しない |
| Robot Driver(inverse_dynamics) | NMPC の胴体・GRF 計画(0.1 s 以内に限る)、足先軌道 | 関節 `torque_ff` + `kp/kd` | 計画を MuJoCo/実機で実行する。**plan が 0.1 s 以上古い/時刻窓外なら `stand_joint_angles` へ PD ホールド**。足場選択・IK 可到達性判定はしない |

> **断定(指示書 16 節)**:現在の Go2 構成では、MPC が穴を避けて足場を最適化して
> いるのではない。**Local Footstep Planner が地図から足場を決定し、NMPC はその
> 足場を固定パラメータとして胴体軌道と GRF を最適化する。**

---

## 6. 資料中の主張の再判定(指示書 10 節)

### 6.1 「NMPC の脚可到達制約が破れた」→ **訂正**

- **誤**(`agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md` §3.4 の旧記述
  「接地脚が脚の可到達域の外 → セントロイダル NMPC の運動学・GRF 制約が破れ」)。
- **コード事実**(確定):Go2 simple model の制約 `g` は
  **EOM(Backward Euler)+ 摩擦錐のみ**(`dynamicsModel.m:43`、`g_dim: 28`)。
  関節角・足位置・IK 可到達性の制約は**存在しない**。
  → 「**NMPC 内の脚可到達制約違反**」という表現は**誤り**。この点だけは断定できる。
- **推測**(未確定。因果の裏取りには下記のログ収集が必要):遠い足場が
  1. GRF のモーメントアーム \((p_{f,j}-p_b)\) を変える、
  2. その足場で `ref_body_plan_` を満たす GRF 配分が \(f_z\in[10,150]\) と
     摩擦錐(実効 μ=0.6)の内側に取りにくくなり、スラック
     (`panic`/`constraint_panic`)が立って `plan_nmpc_cost` が増える、
  3. 反復・計算時間が増えて `compute_time` が replan 予算を超え、非収束・停滞、
  という機序は**辻褄は合うが未検証**。CSV には `plan_nmpc_cost` の総和しか
  無く、**どの項が増えたか(トラッキング項 vs スラック項)**、
  **制約違反量**、**IPOPT の終了ステータス**を記録していない。
- **確定に必要なログ**(Phase 1 と同時に足す。コード変更は診断のみ):
  `eval_f` のトラッキング項/入力項/`panic`/`constraint_panic` の内訳、
  `get_slack_state_var` / `get_slack_constraint_var` の値、
  最大制約違反、IPOPT の `status` / `iter_count` / `obj_value`。
- gbpl doc の該当記述は「脚可到達制約が破れた」を削除し、上を
  「コード事実」と「推測(要ログ)」に分けて書き直す(Phase 0、コード変更なし)。

### 6.2 「`horizon_length` は `period_` より大きい必要がある」→ **要分離**

- **実験事実**:`horizon_length` 26→40 で gbpl 実験の追従が改善
  (`agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`)。
- **コード事実**:`computeContactSchedule:99` の `(i+phase) % period_` は剰余ラップ。
  `horizon_length_` と `period_` の大小に**必須条件はない**。`computeFutureBodyPlan`
  はホライズン外の**胴体**外挿で、period_ とは無関係。
- **推測**(未検証、要 A/B):`period_=30` に対し `horizon_length=26` だと、
  NMPC が最適化する接触列が 1 歩容周期(30)を覆わず、遅いクロールの
  stance/swing パターンの終盤を「見ない」まま解くため追従が悪い、という機序は
  あり得る。だが「必須」ではなく「このパラメータ組で有利」。
- doc は「実験事実」と「コード必須条件ではない」を分けて記述し直す(Phase 0)。

### 6.3 「実機の穴も自然に NaN になる」→ **未確認へ格下げ**

- **コード事実**:この repo に LiDAR/深度カメラ → grid_map の `z` を作る処理は無い。
  `mjcf_to_grid_map_converter.cpp` が静的 PLY をラスタライズするのみ。
- **未確認事項**(実センサ経路が入ったときに要検討):
  - no-return 点(反射なし)の値
  - occlusion(遮蔽)セルの値
  - 未観測セルの表現(NaN か、古い値か、既定値か)
  - 古いセル(更新停止)の表現
  - 「穴」と「単なる未観測領域」の区別
- 現状の成功範囲は「**静的既知 PLY・正確な位置合わせ・手作業マージン**」下の
  シミュレーション。実センサでの安全性は主張できない(指示書 4 節末)。

---

## 7. 現時点の技術的結論(指示書 17 節)

### コードと資料から確認できる事実

- 静的 PLY の穴を NaN / 低 `traversability` として表現できれば、Local Footstep
  Planner は穴上の名目足場を近傍の有効セルへスナップできる(`getNearestValidFoothold`)。
- `reference="twist"` でも terrain map による足場補正は動作する(§1-2、§2.4)。
- gait は地形から自動変更されない(固定表、§3.1)。
- Go2 の simple NMPC は足場を最適化しない。足場は GRF のモーメントアーム
  パラメータ(`dynamicsModel.m` の `feet_location`、§4.3)。
- 有効足場がない場合、現コードは(実質出ない)警告後に**名目足場を返す**(§3.3)。
  ただし **map 外は `getNearestValidFoothold` 手前の `continue` で別処理**(§3.2-5)。
- 足場選択には足裏面積・縁距離・IK 可到達性・map 鮮度が含まれない(§3.3)。
- NMPC(Go2 simple)の目的関数・制約に `traversability` は現れない(§4.5)。
- NMPC 求解失敗が続くと下流は `stand_joint_angles` へ PD ホールドする
  (壊れた GRF を出し続けるのではない、§4.6)。転倒は「求解成功だが低品質な解」
  を実行する区間で起きる(§4.6 (b)、§6.1)。
- 実効摩擦係数 μ は `go2.yaml` の 0.6 が `nmpc_controller.yaml` の 0.3 を
  上書きしている(launch 順、§4.4。ライブ `ros2 param get` で最終確認が要)。

### まだ確認できていないこと

- 実 LiDAR / 深度カメラ Map で穴が確実に NaN または低 `traversability` になること。
- 地図誤差を含めても足が穴の縁から十分離れること(現状はメッシュを手作業で
  0.05 m 広げているだけ)。
- `foothold_search_radius = 0.7` の候補が Go2 の脚で到達可能であること。
- 有効足場がない状況で安全に停止できること。
- **遠い足場 → NMPC cost 増大 → 非収束 の因果**(§6.1。cost 内訳・slack・
  制約違反・IPOPT status を記録するまで推測)。
- **RobotState → NMPC 状態の並進速度フレーム**(§3.2。角速度は body 系と確認済み)。
- 実効 μ のライブ値(`ros2 param get`)。

### 次の最優先対策(実装は本レポート確認後)

`getNearestValidFoothold()` を、**位置だけでなく成功/失敗と診断値を返す**足場
選択器へ変更し、有効足場がない場合に名目足場を返さず、安全に減速・停止できる
経路を作る。その後、穴縁からの安全距離 → IK 可到達性 → Map 鮮度 の順で追加する。

---

## 8. 実装フェーズの見取り図(指示書 11・14 節。詳細な変更計画は各 Phase 着手時に別途提示)

| 順番 | 主対象 | 目的 | 既存挙動への影響(想定) | 検証 |
|---|---|---|---|---|
| 0 | `agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md` ほか | §6 の 3 点(可到達制約 / horizon>period_ / 実センサ NaN)の記述訂正。**コード変更なし** | なし(doc のみ) | 差分レビュー |
| 1 | `local_footstep_planner.{hpp,cpp}` `getNearestValidFoothold` | 戻り値を `FootholdResult{position, found, traversability, snap_distance, edge_clearance, reachable}` 相当へ。まず**診断値の算出と DIAG 出力のみ**、呼び出し側は `position` のみ使用。**§6.1 の NMPC 内訳ログ(cost 項別・slack・制約違反・IPOPT status)も同時に追加**(診断のみ) | なし(挙動不変、ログ追加) | §9 の単体試験(平面/穴中央/縁/広い穴/map 外) |
| 2 | `getNearestValidFoothold` + **`computeFootPlan` の map 外 `continue` 地点**(`local_footstep_planner.cpp:255-261`)+ `local_planner.cpp` | `found==false` を下流へ伝播。**map 外は `getNearestValidFoothold` に到達しない**(§3.2-5)ので、`continue` 地点でも「その脚の接地が確定できなかった」を記録・伝播する必要がある。新しい一歩を確定しない / `cmd_vel`→0 / 全脚接地可なら STAND / `planner_failed` へ理由(map 外 / 未観測 / 広すぎ / map 期限切れ)通知 | 有効足場がある通常時は不変。無い時のみ挙動変化(現在は名目足場で継続 or `continue` で黙って前回踏襲 → 危険) | 「広い穴」「map 外」で停止すること + 回帰(§13) |
| 3 | terrain map(`filter_chain.yaml` or 新レイヤ)+ `getNearestValidFoothold` | 穴縁からの安全距離 \(d_{\rm edge}(p) > r_{\rm toe}+e_{\rm map}+m_{\rm safety}\)。PLY 手作業マージンを地図上判定へ置換。候補:距離変換レイヤ / 円内無効セル判定 / マスクのモルフォロジー膨張 | スナップ先が現在より手前(安全側)に寄る。到達距離が落ちる可能性 | 最小穴縁距離、最大 snap distance、到達距離の変更前後比較 |
| 4 | `getNearestValidFoothold` + `QuadKD2::worldToFootIKWorldFrame`(既存) | 予測接地時の胴体姿勢から `p_f^{leg}=R_{wb}^{\top}(p_f-p_{hip})` を作り、IK 解の存在・関節角限界・左右跨ぎ排除で候補を絞る。**新規 IK は書かない** | 到達不能候補が除外される。候補が減り `found=false` が増える可能性 → Phase 2 の停止と連動 | 「到達不能(traversability 高いが IK 範囲外)」試験 |
| 5 | `local_planner` / `local_footstep_planner` | `d_snap = ‖p*−p_nom‖` を記録し、大補正時に探索半径拡大だけでなく減速/刻み歩行/停止を選べる設計。閾値は**提案値**としてパラメータ化 | 通常時は不変。大補正時のみ | snap distance と速度/転倒の関係を実測 |
| 6 | terrain map 受信部 | header stamp 保存、現在時刻との差、最大許容 age、frame 整合、未観測/危険セルの区別、更新停止時の減速・停止 | 通常時は不変。map 停止時のみ | 「古い Map(stamp 超過)」試験 |

**禁止事項(指示書 15 節)を厳守**:平坦路 Step 01 再実装なし、Global Body Planner
改造なし、足場を NMPC 決定変数へ入れる大改造なし、`foothold_search_radius` の
無根拠拡大なし、有効足場なし時の名目足場継続なし、動画だけの成功判定なし、
PLY マージンだけで一般的安全性を主張しない、無関係ファイルの整形なし、
1 コミット 1 目的。

---

## 9. 単体試験の骨子(指示書 12 節。Phase 1 と同時に作る)

MuJoCo 全体試験の前に、`getNearestValidFoothold`(+ 診断値算出)だけのテスト。
入力は合成 grid_map(`traversability` / `z_inpainted` を明示設定)。

| 試験 | 入力 | 期待 |
|---|---|---|
| 平面 | 名目=平面中央 | ほぼ移動なし、`found=true` |
| 穴中央 | 名目=穴中央(NaN) | 安全領域へ移動、`found=true` |
| 穴の縁 | 名目=縁付近 | 必要安全距離を確保(Phase 3 後) |
| 広い穴 | 安全セルが R 外 | `found=false` |
| Map 外 | 名目が map 外 | `found=false` |
| 未観測 | 候補が未観測セル | `found=false`(Phase 6 後) |
| 到達不能 | traversability 高いが IK 範囲外 | `reachable=false`(Phase 4 後) |
| 古い Map | stamp 超過 | 計画停止(Phase 6 後) |

各試験で記録:脚番号、touchdown index と時刻、nominal foothold[m]、
selected foothold[m]、nominal/selected traversability、snap distance[m]、
edge clearance[m]、IK 可否、found、失敗理由。

---

## 10. 関連

- `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`(twist+クロール成功記録)
- `agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md`(gbpl 実験 + 工程別分析。§6 で一部訂正予定)
- `agent_reports/quadsdk_step01_control_pipeline.md` / `quadsdk_step01_terrain_map.md` / `quadsdk_step01_mpc.md`
- Quad-SDK Wiki(`agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md` §6 にリンク集)
