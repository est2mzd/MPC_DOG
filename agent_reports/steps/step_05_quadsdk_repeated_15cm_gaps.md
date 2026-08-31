# Step 05:15 cm 平地・15 cm 穴の連続区間(N=2〜5)— 事前調査結果

対象: `external/quad-sdk`(go2、`reference:=twist` の Step 01 ハーネス系)。
Step 03/04(Quadruped-PyMPC、浅い轍)とは**別実装・別ロボットスタック**。

> **この文書の段階**: 指示書
> `chatgpt_instruction/cursor_instruction_quadsdk_step05_repeated_15cm_gaps.md`
> の「§17 最初の回答で行うこと」に対応する **事前調査のみ**。
> **コードも地形も、まだ一切変更していない。** 末尾の変更計画表を提示した
> 時点で停止し、ユーザーの承認を待つ(指示書 §15・§18)。
>
> **読み方の約束**:
> - **事実** … コード・設定ファイル・`git` で確認済み。各項に「(確認済み)」。
> - **未確認** … まだ実行/計測していない。
> - **推測** … 辻褄が合う、の域。

---

## 背景

- MPC_DOG では四足ロボット **Go2** を **Quad-SDK**(C++ の四足制御スタック)で
  走らせ、MuJoCo 上で Step 単位に検証している。
- Step 03_1m / 04_1m で、**深さ 1 m・幅 0.30 m のトレンチを、間隔 2.0 m /
  1.5 m で複数本連続で(足を穴に入れずに)渡る**ことに成功済み
  (`reference:=twist` + クロール歩容、C++ の挙動変更なし・設定変更のみ)。
- ただしその成功は「**穴 1 本ぶんの擾乱をクロール歩容の support polygon 余裕で
  吸収できる**」ことに依存していた。**穴と穴の間の平地が広い**(凸条 1.2〜1.7 m)
  ため、1 本渡り切ってから次の縁が来るまでに立て直す余地があった。
- Step 05 は逆に、**平地を 15 cm まで詰めて穴を連続させたとき、Terrain Map /
  Foot Placement / NMPC がどこまで機能するか**を測る。狙いは「N=5 を無理に
  成功させる」ことではなく、**連続穴に対する現行方式の成立範囲と、
  安全に失敗できる境界**をコードとログで明らかにすること。

## 目的(指示書 §0 より)

1. 現在の Foot Placement が連続した狭い支持面を正しく選べるか。
2. 穴縁の危険帯を除くと、実際に何 cm の接地可能領域が残るか。
3. どの `N`(=2,3,4,5)まで再現性を持って連続通過できるか。
4. 通過できない場合、転倒ではなく安全停止へ移行できるか。
5. 失敗原因を Terrain Map / 足場選択 / IK 可到達性 / NMPC / 下位制御に分離する。

---

## 結論(事前調査時点)

1. **既存の地形生成器は、そのままでは 15 cm/15 cm パターンを作れない(確認済み)。**
   `src/trial/assets/gen_quadsdk_gap_world.py` は穴長を **`HOLE_LEN = 0.30` で
   ハードコード**しており、15 cm 穴を指定する引数が無い。さらにメッシュ穴を
   物理穴より片側 `MESH_MARGIN` だけ広げる設計で、**コード上の既定は 0.10 m**
   (指示書・step03/04 資料が想定する 0.05 m と食い違う。→「未確認事項」)。
   仮に穴長を 0.15 m にしても、`MESH_MARGIN = 0.10` ならメッシュ穴幅 =
   0.15 + 0.20 = **0.35 m** が平地ピッチ 0.30 m を超え、**平地のメッシュ面が
   1 枚も生成されず、テスト区間全体が地図の穴になる**。
   → **Step 05 には、穴長・マージンを引数化した専用の地形生成器が必要**
   (MPC_DOG 側のスクリプト追加。C++ 変更ではない)。

2. **15 cm 平地の「安全支持領域」は、幾何・地図解像度・足先寸法のどれで見ても
   境界条件、実質「幾何学的に成立困難」寄り(確認済みの数値からの計算)。**
   - Terrain Map 解像度 = **0.05 m**(確認済み。`mujoco_mapping.py` の
     `grid_map_resolution` 既定 `0.05`。`terrain_map_publisher.yaml` の
     `resolution: 0.1` は無効化された別経路)。
   - 片側マージン 0.05 m を仮定すると
     `L_safe = 0.15 − 0.05 − 0.05 = 0.05 m` = **地図 1 セルぶん**。
   - Go2 の足先接触幅 ≈ `2 × toe_radius = 2 × 0.022 = 0.044 m`(確認済み。
     `go2.yaml: toe_radius`、mjcf foot geom `size 0.022` と一致)。
     → 0.05 m の領域に対し左右 **各 3 mm** しか余裕がない。
   - しかも **`toe_radius` は足場選択の水平判定に使われていない**(確認済み。
     選択コストは `kin_cost = ‖p−p_nom‖ + 0.5‖p−p_prev‖` のみ。`toe_radius` は
     遊脚アペックスの Z 補正だけ)。**縁からの距離制約も無い。**
   - IK 可到達性の判定も**無い**(確認済み。Foot Placement にも NMPC にも
     脚可到達制約は存在しない)。
   → 判定(指示書 §6):**`境界条件`(実質 `幾何学的に成立困難` 寄り)**。
     詳細は「15 cm 平地+危険帯の幾何学的成立性」節。

3. **安全停止(Phase 2)は未実装(確認済み)。**
   `computeFootPlan()` の戻り値は `void` のまま、`computeLocalPlan()` は足場の
   有効性で分岐しない、`stop_on_invalid_foothold` パラメータも無い。
   → 指示書 §9 Stage D / §18 に従い、**通過不能な `N` では「安全停止できない」を
   Step 05 の結果として記録し、名目足場へフォールバックして歩き続けさせない**。
   Phase 2A を先に実装すべき、が現時点の結論。

4. **したがって Step 05 の現実的な価値は、Stage A(地形検証)+ Stage B
   (Foot Placement 単体)+ Stage C の N=2 一撃 + Stage D(安全停止の不在の記録)**。
   N=2 すら「通過成功」の全条件(指示書 §10.1)を満たす見込みは低い(推測)。

---

## コードで確認した事実

| # | 事実 | 根拠(ファイル:行 / コミット) |
|---|---|---|
| F1 | ブランチ `main`、HEAD `cb9d762`、working tree はクリーン(未追跡は本 Step の指示書 md のみ)。submodule `external/quad-sdk` も同 HEAD・クリーン | `git status` / `git rev-parse HEAD` |
| F2 | 既存地形生成器は穴長 `HOLE_LEN = 0.30` 固定。引数は `<spacing> [depth] [tag] [_] [mesh_margin]` | `src/trial/assets/gen_quadsdk_gap_world.py:35` |
| F3 | メッシュ穴 = 物理穴 + 片側 `MESH_MARGIN`。コード既定 `MESH_MARGIN = 0.10 m`(`sys.argv[5]` 省略時) | 同上 `:119` |
| F4 | step03/04 資料 §2・§8 は「mesh_margin 既定 0.05 m」と記述 → **コードと不一致** | `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` §2,§8 |
| F5 | Terrain Map 解像度は **0.05 m**(MuJoCo 経路) | `quad_utils/launch/mujoco_mapping.py:23` `DeclareLaunchArgument('grid_map_resolution', default_value='0.05')`。`quad_mujoco.py:155-166` が `mujoco_mapping.py` を include。実行スクリプトは上書きしない |
| F6 | `terrain_map_publisher.yaml: resolution: 0.1` は grid 入力用で、`mapping.py` 内でノードごとコメントアウト済み(無効) | `quad_utils/launch/mapping.py:23-31` |
| F7 | 穴検出は「メッシュに面が無い→生 `z`=NaN→`traversability`=NaN」。`filter9` `1−|z_finite−z_inpainted|`、`filter14` で `traversability` に乗算。`filter2` inpaint 半径 0.4、`filter10` バリア半径 0.075 | `quad_utils/config/filter_chain.yaml` filter2/9/10/14 |
| F8 | 足場選択は `traversability > foothold_obj_threshold(0.6)` のセルへ `SpiralIterator`(半径 `foothold_search_radius`)でスナップ。コスト `kin_cost = ‖p−p_nom‖ + 0.5‖p−p_prev‖`。**縁距離・足裏面積・IK・支持多角形・map 鮮度は無し** | `local_footstep_planner.cpp` `getNearestValidFootholdResult`(`:534-620`)、解析doc §表 行5,6 |
| F9 | Phase 1 の `FootholdStatus`(`VALID` / `NOMINAL_OUTSIDE_MAP` / `NO_TRAVERSABLE_CANDIDATE` / `NONFINITE_HEIGHT`)+ `FootholdResult{position, status, traversability_nominal, traversability_selected, snap_distance}` は実装済み | `local_footstep_planner.hpp:36-55`、コミット `484ea13` |
| F10 | **Phase 2A は未実装**。`computeFootPlan()` は `void`(`:162`)。`computeLocalPlan()`(`:514`)は戻り値 `bool` だが足場有効性で分岐しない。`stop_on_invalid` 系の識別子はコードに存在しない | `local_footstep_planner.cpp:162` / `local_planner.cpp:514,533` / `rg` 全滅 |
| F11 | NMPC の制約は EOM(Backward Euler)+ 摩擦錐のみ。関節角・足位置・IK 可到達性の制約なし | 解析doc §表 行10、§4.3/§6.1 |
| F12 | 実効摩擦係数 μ:`go2.yaml` の `nmpc_controller.friction_coefficient: 0.6` が launch 順で `nmpc_controller.yaml` の `0.3` を上書き(**ライブ未確認**、推定) | `go2.yaml` / `nmpc_controller.yaml` |
| F13 | この repo に LiDAR/深度→`z` レイヤ生成は無い。静的 PLY のラスタライズのみ | `mjcf_to_grid_map_converter.cpp`、解析doc §表 行12 |

## 現在の実験パラメータ(step03/04 の再現性最良条件 = Step 05 のベースライン)

| 項目 | 値 | 出典(確認済み) |
|---|---|---|
| `cmd_vel`(前進) | 0.3 m/s(最安定)。0.15 / 0.5 も既存成功域 | step03/04 doc §5.2 |
| gait | 横列クロール(FL→BR→FR→BL) | `go2.yaml` `phase_offsets: [0.0, 0.75, 0.5, 0.25]` |
| `period` | 0.9 s | `go2.yaml` |
| `duty_cycles` | `[0.75, 0.75, 0.75, 0.75]`(常時 ≥3 脚接地) | `go2.yaml` |
| `phase_offsets` | `[0.0, 0.75, 0.5, 0.25]` | `go2.yaml` |
| `ground_clearance` | 0.1 m | `go2.yaml` |
| `hip_clearance` | 0.1 m | `go2.yaml` |
| `horizon_length` | 40 ステップ | `local_planner.yaml` |
| NMPC 時間刻み | 0.03 s(`local_planner.timestep`) | `local_planner.yaml` |
| `foothold_search_radius` | 0.7 m | `go2.yaml`(素は 0.25) |
| `foothold_obj_threshold` | 0.6 | `local_planner.yaml` |
| `obj_fun_layer` | `traversability` | `local_planner.yaml` |
| `grf_weight` | 0.45 | `local_planner.yaml` |
| 摩擦係数(実行時) | μ = 0.6(推定、F12) | `go2.yaml` |
| IPOPT linear solver | `mumps` | `nmpc_controller` 既定(CoinHSL/MA27 未導入・ユーザー制約) |
| 穴縁マージン(片側) | **想定 0.05 m / コード既定 0.10 m(要確定)** | F3,F4 |
| Terrain Map 解像度 | 0.05 m | F5 |
| `toe_radius` | 0.022 m | `go2.yaml` |
| swing apex | `min(ground_clearance − toe_radius + max(prev_z,next_z), hip_height − hip_clearance)` | 解析doc §280 |

助走・着地区間(指示書 §5.1):既存 `run_quadsdk_gap_1m.sh` は
`init_pose -x 0.0`、`STAND_SETTLE_S=8` + `PLAN_STARTUP_S=3` 後に WALK。
step03/04 の凸条は x ≈ 0 に 1 本目。**採用予定**:テスト区間開始 `x0` は
助走に凸条数本ぶん(≥ 1.5 m)を確保して `x0 ≈ 2.0 m`、最後の穴の後に
着地・停止用の連続平地を ≥ 2.0 m。根拠と最終値は実装時にスクリプトへ明記する。

## 15 cm 平地+危険帯の幾何学的成立性

指示書 §6 の式(片側マージン `m`):

```
1 つの穴の接地禁止帯   L_forbidden = m + 0.15 + m
連続穴間の平地の安全幅 L_safe      = 0.15 − m − m
```

| 片側マージン `m` | `L_safe` | 地図セル数(0.05 m/セル) | 足先 0.044 m との関係 |
|---|---|---|---|
| 0.05 m(指示書の想定) | **0.05 m** | 1 セル | 左右各 3 mm 余裕(ほぼゼロ) |
| 0.10 m(コード既定) | **−0.05 m** | 0(平地が地図から消える) | 成立不能 |
| 0.00 m(マージン無し) | 0.15 m | 3 セル | 収まる。ただし縁ぎりぎり(1 m 落下の物理縁に直載り) |

- **指示書 §6 の必須判断**:`境界条件`(実質 `幾何学的に成立困難` 寄り)。
  理由:(a) 安全幅がベストでも地図 1 セル、(b) 足場選択に縁距離制約が無い
  (F8)ので、その 1 セルを外して物理縁へスナップし得る、(c) `foothold_search_radius
  = 0.7 m` は連続穴を何本も跨ぐので、スナップ先が隣接しない凸条へ 0.3〜0.6 m
  飛び、`snap_distance` が大きくなり支持多角形が歪む(推測)、(d) IK 可到達性
  未判定(F11)。
- **指示書の禁止事項に従い、穴やマージンを勝手に狭めない。** 成立困難でも
  Step 05 の主目的は「正しく失敗を検出して安全停止できるか」に移る(§6 末尾)。

## Terrain Map 解像度との整合

- 解像度 0.05 m(F5)。15 cm 物理穴 = 3 セル、15 cm 平地 = 3 セル、
  片側 0.05 m マージン = 1 セル、安全幅 0.05 m = **1 セル**。
- `filter10`(`MeanInRadius` 半径 0.075 = 1.5 セル)が穴マスクをにじませる。
  step03/04 §3.3 で「穴帯の `hole_mask` は 0 でなく NaN で、`MeanInRadiusFilter`
  は NaN を広げない」と確認済み → バリアで安全 1 セルが潰れるかは**未確認**
  (Stage A で `traversability` 実値を見る)。
- `filter2` inpaint 半径 0.4 m は 15 cm 穴を確実に埋める → `z_inpainted` は
  穴上でも 0 近傍 → 偽の胴体高さ/ピッチは出にくい(推測、step03/04 と同機序)。

## Go2 足先寸法との整合

- 足先接触幅 ≈ 0.044 m(F8 根拠と同じ `toe_radius = 0.022`)。
- 安全幅 0.05 m(m=0.05)に対し**収まるが余裕 3 mm/側**。step03/04 の
  横ドリフト実測は 0.3 m/s で 0.06 m、0.15 m/s で最大 0.7 m。
  → 3 mm の余裕はドリフトにも足場追従誤差にも耐えない(推測)。
- `toe_radius` は水平安全判定に**不使用**(F8)。Z 補正/ swing apex のみ。

## IK・支持多角形との整合

- **IK 可到達性の判定は Foot Placement にも NMPC にも無い**(F11)。→「未判定」。
- 脚全伸長 ≈ 0.426 m(股→足先)。`foothold_search_radius = 0.7 m` は
  可到達域より広いので、スナップ先が IK 範囲外になり得る(推測、Stage B で計測)。
- crawl 1 歩の前進 ≈ `v × period = 0.3 × 0.9 = 0.27 m`(1 脚は 1 周期に 1 回踏む)。
  名目足場ピッチ ≈ 0.27 m はパターン周期 0.30 m と近いが、Raibert 名目は
  地形位相にロックされない → 名目が穴に落ちる頻度が高くスナップ多発(推測)。
- 支持多角形:duty 0.75 で常時 3 脚接地だが、安全幅 1 セルへ 3 脚を同時に
  正確に載せ続ける必要があり、1 脚でも縁へ滑ると崩れる(推測)。

## Phase 2 安全停止の実装状態

- **未実装**(F10)。`agent_reports/quadsdk_gap_foothold_phase_progress.md`
  §「次にやること:Phase 2A」に変更計画表(2A-1〜2A-5)があるが、
  ブロック中の確認 3 件(戻り値型 / 常時 ON か param か / 失敗記録の粒度)が
  未回答でコード未着手。
- 現状の停止手段は「local plan が 0.1 s 以上古い → `robot_driver` が
  `stand_joint_angles` へ PD ホールド」のみ(受動的)。無効足場を能動的に
  検出して離脚を止める仕組みは無い。

## 未確認事項

1. **穴縁マージンの実値**:step03/04 の**コミット済み PLY** が片側 0.05 m か
   0.10 m か。`gen_quadsdk_gap_world.py` のコード既定は 0.10、資料は 0.05。
   → 実際の `.ply` を読むか、決めた値で再生成して確定する(Stage A 前提)。
2. **安全 1 セルが `traversability > 0.6` として生き残るか**:Stage A で
   N=2,3,4,5 の `terrain_map` 実値(z / z_inpainted / z_smooth / traversability /
   in-out / finite-NaN)を strip 中央・境界で計測して確認。
3. **μ = 0.6 のライブ確認**(F12):
   `ros2 param get /robot_1/local_planner nmpc_controller.friction_coefficient`。
4. **Stage B(Foot Placement 単体)の実行方法**:合成した名目足場列を
   `getNearestValidFootholdResult()` へ入れて `FootholdResult` を記録する
   ハーネスが未整備。gtest 拡張か、DIAG ログ増設か、専用ノードか(下記変更計画 #3)。
5. **NMPC 内訳ログ**(cost 項別 / slack / constraint violation / IPOPT status)は
   未実装(解析doc §6.1)。「遠い足場→NMPC 非収束」の因果は取得まで推測。
6. `quad_mujoco.py` が `grid_map_resolution` を上書きしないことの最終確認
   (launch 引数のデフォルト伝播。Stage A の DIAG ログ `res=%.4f` で実測)。

## 推測・仮説

- N=2 でも「通過成功」(指示書 §10.1 全条件)は達成困難。最も早い破綻は
  「安全 1 セルを外して物理縁へスナップ → 接地不安定 → 胴体沈み/ピッチ」
  (step03/04 §3.3 と同型、ただし逃げ場の平地が無いぶん深刻)。
- 破綻の主原因は **Map 解像度 × 縁距離制約の不在**(Foot Placement 層)。
  NMPC や下位制御はその結果を実行するだけ(F8,F11)。
- Stage D は「安全停止できず、受動タイムアウト PD ホールドに落ちるか、
  その前に転倒/落下」になる公算が高い → Phase 2A の先行実装が必要という
  指示書 §18 の想定どおりになる。

---

## 変更計画(指示書 §15 形式)— 提示のみ・未実装

> **重要**:表を提示した時点で停止し、ユーザーの承認を待つ(指示書 §15・§18)。
> 承認前にコード・地形・スクリプトを変更しない。

| # | 変更ファイル(新規/既存) | 現状 | 変更内容 | 必要な理由 | 制御挙動への影響 | 検証方法 |
|---:|---|---|---|---|---|---|
| 1 | **新規** `src/trial/assets/gen_quadsdk_repeated_gap_world.py` | 既存 `gen_quadsdk_gap_world.py` は穴長 0.30 固定・マージン既定 0.10 で 15 cm/15 cm を作れない(F2,F3) | 穴長 `--hole`、平地長 `--strip`、穴数 `--n`、深さ `--depth`、片側マージン `--margin`、`x0` を引数化した world XML + 地形 PLY 生成器。既存 step03/04 の生成器・地形ファイルには一切触れない | Step 05 の地形が既存方式では表現不能。指示書 §14「既存 step03/04 を変更しない・N を引数化」 | **なし**(MPC_DOG 側の地形生成スクリプト。C++ 不変、ROS 再ビルド不要) | 生成物を PLY パーサで検証:strip/gap の x 範囲、面の有無、深さ。N=2..5 で目視 + 数値 |
| 2 | **新規** `scripts/trial/run_quadsdk_step05_map_probe.py`(+ `src/trial/quadsdk_step05_map_probe.py`) | Terrain Map の実値を N 別に確認する手段が無い | `terrain_map` を購読し、各 strip 中央・境界で z / z_inpainted / z_smooth / traversability / in-out / finite-NaN を CSV 化。x 断面プロット + 上面図 PNG | 指示書 §7「歩かせる前に Terrain Map を検証」。安全 1 セルの生存確認(未確認事項 2) | **なし**(購読のみ) | N=2..5 で CSV + PNG を生成、物理穴/マージン/安全幅/通常平地の 4 区別が数値で出るか |
| 3 | **要ユーザー判断**:Stage B の Foot Placement 単体ハーネス | 合成名目足場 → `FootholdResult` を記録する手段が無い(未確認事項 4) | 案A: `local_planner/test/test_footstep_planner.cpp` に Step 05 地形の gtest を追加(C++、テストのみ・挙動不変)。案B: 既存 DIAG ログ(`[DIAG] gnvf`)を CSV へ流す小改造。案C: 専用診断ノード | Stage B(指示書 §9)を「捏造せず」実施(§Stage B 末尾) | 案A/B: なし(テスト or ログのみ)。案C: 新規ノード追加 | `colcon test --packages-select local_planner` が 29/29 green のまま + 新規ケース pass |
| 4 | **新規** `scripts/trial/run_quadsdk_step05_repeated_gaps.sh`(+ 記録 `src/trial/quadsdk_step05_repeated_gaps.py`) | Step 05 の実走ハーネスが無い | `run_quadsdk_gap_1m.sh` のパターン(joint_controller 待ち・固定カメラ録画・CSV ロガー・trap 後片付け)を流用し、`N` を引数化。指示書 §11 の試行単位/時系列項目を記録 | Stage C(指示書 §9)。既存ハーネスの再利用(§14) | **なし**(既存の設定値で走らせるだけ。N 以外のパラメータは固定) | N=2 を 1 回 → §10 判定。安全に通れた N のみ 3 回反復 |
| 5 | **新規** `agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md` を結果で更新 + README リンク | 本ファイル(事前調査のみ) | Stage A〜D の結果、指示書 §12 の N 別比較表、§13 失敗分類、§19 の 8 問への回答を追記 | プロジェクト共通ルール(実行したら .md + README リンク) | なし(ドキュメント) | 差分レビュー |

### 明記事項(指示書 §15)

- **変更しないファイル**:`src/trial/assets/gen_quadsdk_gap_world.py`、
  `scripts/trial/run_step_03.sh` / `run_step_04.sh` / `run_quadsdk_gap_1m.sh`、
  既存の `flat_gaps_2m` / `flat_gaps_1p5m` world・PLY、`external/quad-sdk` の
  **制御コード全般**(`local_footstep_planner.cpp` の探索本体、`local_planner.cpp`、
  `nmpc_controller`、`inverse_dynamics_controller.cpp`)、`go2.yaml` /
  `local_planner.yaml` / `filter_chain.yaml` の**値**。
- **再ビルドが必要な ROS 2 package**:上記 #1・#2・#4 のみなら **不要**
  (地形生成 + 購読 + 既存バイナリでの実走)。#3 で案A/案C を採るときだけ
  `colcon build --packages-select local_planner`。
- **地形生成だけで済むか / C++ が要るか**:Stage A・C・D は地形生成
  + スクリプトのみで可能。**Stage B の「正しい」実施と、Stage D の
  「能動的な安全停止」には C++ が要る**(後者は Phase 2A そのもの)。
- **Phase 2 の安全停止が Step 05 より先に必要か**:**Stage D を「通過」で
  評価するなら Yes**。指示書 §18 は「Phase 2 未完成なら『安全停止できなかった』を
  Step 05 の結果として記録し、先に Phase 2 を実装すべきと結論」と明示。
  → 本計画は **Phase 2A を Step 05 に混ぜない**。Step 05 は現状のまま走らせて
  失敗と安全停止不在を記録し、その結果をもって Phase 2A 実装の是非をユーザーに諮る。
- **1 コミットごとの目的**:#1 地形生成器 / #2 Map 検証 / #3 Foot Placement 診断 /
  #4 N=2..5 ランナー / #5 結果ドキュメント。混ぜない。

## ユーザー判断が必要な項目

1. **穴縁マージン**:片側 **0.05 m**(指示書想定)で確定してよいか。それとも
   既存 step03/04 の実 PLY を先に計測して合わせるか。
2. **Stage B ハーネス**(変更計画 #3):案A(gtest 追加・テストのみ)/
   案B(DIAG→CSV の小改造)/ 案C(専用ノード)のどれで進めるか。
   → 推奨は **案A**(挙動不変、29/29 green を維持したまま検証できる)。
3. **Phase 2A(能動的な安全停止)を Step 05 の前に実装するか**、それとも
   **Step 05 を先に走らせて「安全停止不在」を実測・記録**してから Phase 2A に
   入るか。→ 指示書 §18 は後者(先に測って結論)を想定。あわせて Phase 2A の
   ブロック中確認 3 件(戻り値を `bool` か `FootPlanResult` 構造体か /
   常時 ON か `stop_on_invalid_foothold` パラメータか / 失敗記録は最初の 1 件か
   全件カウントか)への回答も必要。
4. **N の上限で止めるか**:指示書は N=2→3→4→5 の順で、転倒/危険足場が出たら
   自動で次に進まない、としている。N=2 で破綻した場合、N=3..5 は
   「地形・Map 検証(Stage A)+ Foot Placement 単体(Stage B)」のみ実施でよいか。
