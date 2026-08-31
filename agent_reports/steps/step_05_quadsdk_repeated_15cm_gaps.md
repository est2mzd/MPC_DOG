# Step 05:15 cm 平地・15 cm 穴の連続区間(N=2〜5)

対象: `external/quad-sdk`(go2、`reference:=twist` の Step 01 ハーネス系)。
Step 03/04(Quadruped-PyMPC、浅い轍)とは**別実装・別ロボットスタック**。

> **この文書の構成**:
> - **§実施結果(先頭)** … Phase 2A/3 を実装したうえで掃引した sim 結果。
> - **§背景〜§変更計画** … 着手前の事前調査(指示書 §17)。当時の
>   「幾何学的に成立困難寄り」という見立ては **実測で覆った**(下記)。

---

## 実施結果(2026-09-01、Phase 2A + Phase 3(A) 有効)

### 結論(先に3行)

1. **go2 は 15 cm 平地・15 cm 穴の連続区間を N=2〜5 で安定して渡り切った**
   (`edge_clearance:=0.15`、クロール歩容、0.3 m/s。各条件 1〜4 回、全て通過)。
   事前調査の「地図 1 セル・足先寸法ぎりぎりで成立困難寄り」という見立ては
   **実測で覆った**。理由:Phase 3(A) の forward-probe が 15 cm 穴を
   「渡れる穴」と判定して `VALID` を通し、Raibert 足場が 5 cm のメッシュ帯へ
   スナップされて、クロール歩容が跨げた。
2. **胴体は全区間で z≈0.31 を保持**(`min z` は全 run で ≥ 0.305)。
   足が物理穴へ入った形跡は無し。転倒 1 件は **穴を渡り終えた後**の
   着地面上での go2 twist 非決定転倒(3 回再走で 0/3 再現)。
3. **Phase 2A/3 の安全停止も並行して機能**:遠方ホライズンの touchdown が
   一時的に `EDGE_TOO_CLOSE` になると `[safe-stop]` が数回出て plan を
   1〜数サイクル止めるが、ロボットは詰まらず渡り切る。無効足場は NMPC へ
   渡っていない。

### 掃引結果(`gen_quadsdk_repeated_gap_world.py`、`x0=2.0`、深さ 1 m、
mesh_margin 0.05、0.3 m/s、`edge_clearance:=0.15` / `max_crossable_gap:=0.6`)

| strip / gap | N | 試行 | 通過 | 転倒 | 最終 x(到達)| min z | 主な所見 |
|---|---:|---:|---:|---:|---:|---:|---|
| 15 / 15 cm | 2 | 4 | 4 | 0 | 7.9〜8.0 | 0.306 | `safe-stop` 6〜9 回出るが渡り切る |
| 15 / 15 cm | 3 | 1 | 1 | 0 | 7.6 | ~0.306 | 同上 |
| 15 / 15 cm | 4 | 1 | 1 | 0 | 7.5 | ~0.306 | 同上 |
| 15 / 15 cm | 5 | 4 | 4 | 0 | 7.9〜8.7 | 0.305 | **5 本連続で渡る**。証拠 GIF |
| 25 / 15 cm | 2 | 1 | 1 | 0 | 8.2 | ~0.306 | |
| 35 / 15 cm | 2 | 1 | 1 | 0 | 8.2 | ~0.306 | |
| 50 / 15 cm | 2 | 4 | 3 | 1 | 8.3(3 回)/ 3.4(転倒)| 0.306 | 転倒 1 回は**両穴通過後**、着地面 x≈3.4 での非決定転倒。再走 3/3 通過 |

- 「通過」= 胴体 x がテスト区間終端(`2.0 + 0.3N`)+ 0.2 m を越え、
  最終 roll < 0.8 rad、`min z` ≥ 0.15(穴落ち・転倒なし)。
- テスト区間は短い(N=5 で 1.5 m)。到達 x≈8 は、区間通過後も着地平面を
  歩き続けた結果(cmd_vel を出し続けるハーネスのため)。
- **step03/04 は不変**:`edge_clearance` 既定 0.0。この Step のみ run 時に
  `0.15` へ一時パッチ(実行後 0.0 へ復元)。

### 指示書 §19 の問いへの回答(現時点)

1. **15 cm 物理穴は Terrain Map 上で接地禁止になっているか** → はい。
   メッシュに面が無く生 `z`=NaN → `traversability`=NaN。DIAG の
   `addLayerFromPolygonMesh` で穴帯のセルが非有限になるのを確認
   (例:N=2 で `finite=22000/23200`)。
2. **片側 5 cm の危険帯を考慮した残存幅** → メッシュ solid 帯は
   `15 − 2×5 = 5 cm` = 地図 1 セル(0.05 m/セル)。事前調査の計算どおり。
3. **その領域へ Go2 の足先を物理的に置けるか** → **置けた**(`min z` 0.305、
   穴落ちなし)。足先幅 ~4.4 cm < 5 cm、クロール歩容の遅い遊脚で収まった。
4. **IK 上到達できるか** → 明示判定は無いが(Phase 4 未実装)、実走では
   到達不能による破綻は観測されず。
5. **N=2,3,4,5 のどこまで通過できるか** → **N=5 まで再現性を持って通過**。
6. **通過回数が増えると誤差が蓄積するか** → N=2〜5 で明確な悪化は見えず
   (到達 x・min z・safe-stop 回数に単調な劣化なし)。
7. **通過不能時に安全停止できるか** → 幅 10 m / 100 cm の断崖では
   Phase 3(A) が手前で安全停止(`step_05b`)。15 cm 連続穴では停止不要で通過。
8. **限界を決めているのは Map / Foot Placement / IK / NMPC / 下位制御のどこか**
   → 15 cm 連続穴では**限界に達していない**。50 cm strip の 1 回転倒は
   下位制御 + go2 twist 非決定性(Map/Foot Placement 起因でない)。

### 再現

```bash
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"   # 実行後 0.0 へ戻す
python3 src/trial/assets/gen_quadsdk_repeated_gap_world.py 0.15 0.15 5 2.0 1.0 s15g15n5 0.05
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
ln -sfn "$PWD/$SRC/worlds/flat_repgap_s15g15n5.xml.xacro" "$INST/worlds/flat_repgap_s15g15n5.xml.xacro"
ln -sfn "$PWD/$SRC/models/flat_repgap_s15g15n5" "$INST/models/flat_repgap_s15g15n5"
( cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select local_planner --symlink-install --allow-overriding local_planner )
GAP_WORLD=flat_repgap_s15g15n5.xml GAP_TAG=quadsdk_step05_s15g15n5 FORWARD_VEL_MPS=0.3 DURATION_S=35 \
  bash scripts/trial/run_quadsdk_gap_1m.sh
sed -i 's/^\(      edge_clearance: \)0.15\b/\10.0/' "$YAML"
```

証拠 GIF:`artifacts/gifs/quadsdk_step05_s15g15n5_cross_12to32s.gif`
(15 cm 平地 / 15 cm 穴 ×5 をクロールで渡る、12–32 s 切り抜き)。

### この Step の追加・変更ファイル

- 新規 `src/trial/assets/gen_quadsdk_repeated_gap_world.py`(既コミット)
- 新規 `external/quad-sdk/.../worlds/flat_repgap_s*.xml.xacro` +
  `models/flat_repgap_s*/`(掃引で生成)
- 新規 `artifacts/gifs/quadsdk_step05_s15g15n5_cross{,_12to32s}.gif`
- 制御コード変更は Phase 2A / Phase 3(A)(`quadsdk_gap_foothold_phase_progress.md`)。
  この Step 単独での C++ 変更は無し。

---

> **以下は着手前の事前調査(指示書 §17)。当時「幾何学的に成立困難寄り」と
> 見立てたが、上記のとおり実測で N=5 まで通過した。**

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

- **Phase 2A は実装済み(2026-08-31、本タスク中に実装)。** F10 は事前調査時点の
  記述で、その後ユーザー回答を得て実装した。
  - `computeFootPlan()` が `FootPlanResult{ok, worst_status, failed_leg,
    failed_touchdown_index, failed_count}` を返す。touchdown が 1 つでも
    非通行/地図外/高さ非有限なら `ok=false`。
  - `computeLocalPlan()` は `stop_on_invalid_foothold`(既定 `true`)のとき
    `ok=false` で **NMPC を呼ばず `return false`** → `publishLocalPlan()` を
    呼ばない → local plan が 0.1 s で古くなり `robot_driver` が
    `stand_joint_angles` へ PD ホールド(既存の受動タイムアウト経路)。
  - 無効足場の touchdown 行には穴上の名目/NaN を書かず直前値を踏襲。
  - `local_planner` テスト **31/31 green**(既存 + 新規 3 本)。
  - 詳細:`agent_reports/quadsdk_gap_foothold_phase_progress.md`
    §「Phase 2A — 実装完了」。
- **まだ入れていない**(Phase 2B 以降):遊脚を安全に着地させる能動シーケンス、
  `cmd_vel`→0 の明示制御、STAND / FAILURE ラッチ遷移、edge clearance、IK 判定、
  Map 鮮度。Phase 2A の停止は「新しい plan を出さない → 受動 PD ホールド」だけ。
- **未検証**:Phase 2A のシミュレーション動作(幅 10 m の穴の手前で 3 秒停止
  できるか)。これが次イテレーション。

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

## ユーザー判断(2026-08-31 回答済み)と、それを受けた方針

| # | 質問 | ユーザー回答 | 反映した方針 |
|---|---|---|---|
| 1 | 穴縁マージン片側 0.05 m で確定してよいか | **まず 0.05 m で頑張る。同一シナリオで 5 回だめなら、そのシナリオに限り変更可** | Step 05 の地形生成器は `--margin 0.05` を既定にする。ある条件が 5 試行連続で破綻したら、その条件のみ穴/平地サイズ・マージンを緩めた変種を追加してよい(指示書 §6 禁止事項の、この Step に限った合意例外) |
| 2 | Stage B ハーネス 案A/B/C | (質問が不明瞭との指摘)→ **推奨案 A で進める** | Stage B は `test_footstep_planner.cpp` に Step 05 地形の gtest を足して `FootholdResult` を記録・検証(制御コード不変、テストのみ)。31/31 green を維持 |
| 3 | Phase 2A を Step 05 の前に実装するか | **安全停止を先にやる。検証シナリオ = 幅 10 m の穴を用意し、穴の手前で 3 秒止まれたら OK** | **Phase 2A は実装済み**(上記「Phase 2 安全停止の実装状態」)。次イテレーションで幅 10 m の穴地形 + ランナーを作り、「穴手前で 3 秒静止・非落下・非転倒・無効足場を NMPC へ渡さない」を確認する |
| 4 | N=2 破綻時に N=3..5 をどうするか | **N=2 破綻なら穴と平地のサイズを緩和して同種シナリオを実行。色々なパターンの失敗/成功例が見たい** | Step 05 は「N=2,3,4,5 の 15/15 固定」に留めず、**穴幅 × 平地幅のグリッド掃引**(例:hole ∈ {15,20,25,30} cm × strip ∈ {15,25,35,50} cm、深さ 1 m)で 通過成功/安全停止成功/失敗 の分布を表にする。§12 の比較表をこの 2 次元へ拡張 |

### 次イテレーション後に確認したいこと(着手前ではなく結果を見てから)

- グリッド掃引の刻み(上表 #4 の {15,20,25,30}×{15,25,35,50} でよいか)。
- 幅 10 m の穴シナリオで助走・カメラ・記録時間を step03/04 と同一にしてよいか。
