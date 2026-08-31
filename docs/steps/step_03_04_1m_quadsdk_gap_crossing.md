# Step 03_1m / 04_1m:Quad-SDK で「穴に足を入れずに」深い穴を渡る(WIP チェックポイント)

対象: `external/quad-sdk`(go2、`reference:=twist` の Step 01 ハーネス系)。
Step 03/04(Quadruped-PyMPC、浅い轍)とは**別実装・別ロボットスタック**。

**状態: 未完(WIP)。** 足場を穴の外へ置く制御(foot placement control)は
**動くようになった**が、go2 のトロットが最初の穴の縁で転倒し、**穴を渡り切れて
いない**。この文書はそこまでの調査・修正・残課題の記録。

---

## 1. 背景・目的

- ご要望: 「Step 03/04 の目的は**穴に足を入れずに歩くこと**、穴の深さは無関係。
  穴に足を入れない foot place control をして、うまく行くまで検討する」。
  実装は **Quad-SDK 側**で行う。
- Quadruped-PyMPC(Step 03/04)は素の状態で**地形を見た足場回避ができない**
  (`blind` 既定、`height` VFA は z のみ、`vfa` は非公開)。深い穴は最初の穴で
  転落する。
- 対して **Quad-SDK の `local_footstep_planner` は地形マップの `traversability`
  レイヤと `getNearestValidFoothold()` で「足場を通行不可セルの外へずらす」
  機構を最初から持っている**。これが「穴に足を入れない」の正しい道具。
  深さは `traversability` の判定には無関係(斜面・粗さで穴と分かればよい)。

## 2. マップ仕様(step03_1m / step04_1m)

`src/trial/assets/gen_quadsdk_gap_world.py <spacing> <phys_depth> <tag> [map_dip]`
が **2 つ**を生成する:

- **物理ワールド** `worlds/flat_gaps_<tag>.xml.xacro`
  - y ∈ [-2.5, 2.5] の 5 m 幅通路。box 凸条(上面 z=0)を並べ、間に
    **0.30 m 長・幅 5 m・深さ 1.0 m** のトレンチ。単純プリミティブのみ
    (`big_flat.xml` の不安定化を回避)。
  - step03_1m: 間隔 2.0 m(凸条 1.7 m)/ step04_1m: 間隔 1.5 m(凸条 1.2 m)
- **地形マップ用メッシュ** `models/flat_gaps_<tag>/meshes/flat_gaps_<tag>.ply`
  - Quad-SDK の地形マップは物理ワールドではなく **この PLY** から作られる
    (`mjcf_to_grid_map_converter` が `models/<world>/meshes/<world>.ply` を読む)。
  - 連続面。穴の x 帯だけ `map_dip`(既定 0.04 m)落として急斜面(ランプ)を作り、
    `slope`/`roughness` を跳ね上げて `traversability` を下げる。
    `map_dip` を小さくしているのは、`z_inpainted` が 0 近傍に保たれ、
    足先 z 参照が地面下に潜らないようにするため。
  - `flat_wide.ply` と**同一のバイナリ形式**(binary LE、per-face RGBA +
    uchar-count int32、CRLF ヘッダ)。PCL/VTK の `loadPolygonFilePLY` が
    そのまま読む。

実行: `GAP_WORLD=flat_gaps_2m.xml GAP_TAG=step03_1m FORWARD_VEL_MPS=0.3 \
DURATION_S=25 bash scripts/trial/run_quadsdk_gap_1m.sh`

## 3. 調査で判明したこと(計装ビルドで確定)

`quad_utils` / `local_planner` に `[MPC_DOG DIAG]` ログを仕込んで再ビルドし、
チェーンを追った。

1. **地形マップ生成は最初から正常だった(赤ニシン)。**
   `initializeFromPolygonMesh → true`、grid 674×100、`z` レイヤ全 67400 セル有限。
   `Created map ...` ログが出なかったのは `verbose` パラメータのゲートのせいで、
   マップ自体は毎回正しく publish されていた。
2. フィルタ連鎖も正常。`terrain_map` に `traversability` / `traversability_mask`
   レイヤが生成され、穴帯で `traversability ≈ 0.02`(浅いランプ版で ≈ 0.46)。
3. `getNearestValidFoothold()` は呼ばれていて、穴の nominal を検出はするが、
   **`foothold_search_radius = 0.25 m` が狭すぎて**、フィルタ平滑化で ~0.7 m 幅に
   なった非通行帯の中心付近では 0.25 m 内に安全セルが無く、`found=0`(スナップ
   失敗)→ nominal のまま穴へ。

## 4. 施した修正(external/quad-sdk、WIP)

| ファイル | 変更 | 効果 |
|---|---|---|
| `quad_utils/config/go2.yaml` | `local_footstep_planner.foothold_search_radius` **0.25 → 0.55** | 穴中心の nominal からも隣の凸条に届く |
| `quad_utils/config/filter_chain.yaml` | `z_smooth` 半径 0.2→0.06、`normal_vectors_` 0.15→0.06、`smooth_normal_vectors_` 0.4→0.10 | 非通行帯 ~0.7 m → ~0.4 m に |
| `local_planner/src/local_footstep_planner.cpp` `getNearestValidFoothold` | スナップ先の kin_cost に **前方バイアス**(`5.0·max(0, nominal.x − cand.x)`)+ **履歴ヒステリシス**(前回解への距離重み 0.5 → 2.0) | 近端で足踏みせず渡る側へ commit、planning cycle 間で目標がチャタらない |
| `quad_utils/src/mjcf_to_grid_map_converter.cpp` / `local_planner/src/local_planner.cpp` | `[MPC_DOG DIAG]` ログ(軽量・カウンタ間引き) | 再現・継続調査用。撤去可 |

## 5. 結果

### 動くようになった: foot placement

`getNearestValidFoothold()` が**穴の全 nominal(`traversability < 0.6`)を検出し、
毎回 solid strip へスナップする**。`found=0`(スナップ失敗)は **0 件**。
DIAG 例:`nominal x=1.006 trav=0.020 → snapped x=1.256`、
`nominal x=0.838 → snapped x=0.988`。

### 塞がっている: 直立して渡る

go2 のトロットが **最初の穴の縁**(body x ≈ 0.68〜0.89 m、前脚が穴の縁 x≈0.85)で
**前方ピッチが 0.5〜0.7 rad スパイク → roll → 横倒れ → 停止**。

15 回の試行(速度 0.25/0.3 m/s、`ground_clearance` 0.07/0.16、前方バイアス
1.0/5.0、ヒステリシス 0.5/2.0、`map_dip` 0.30/0.04、非通行帯幅)で**縁での転倒は
不変**。足場は穴の外に置けているのに、縁を通過する擾乱でトロットが balance を
失う。

これは Quad-SDK Step 01 の handoff が**未解決事項として明記**している
「go2 トロットは 0.5〜1.1 m/s で安定性が心許ない、平地でも非決定的に転倒」と
同じ壁。

---

## 6. 大学院初心者向け解説:なぜ足場を避けても転ぶのか

### 6.1 「足を置く場所」と「バランス」は別問題

トロット(対角の 2 脚ずつ接地して進む歩き方)は**動的歩行**。止まっていれば
倒れる姿勢を、**次の一歩を正しい場所・正しいタイミングで置くこと**で連続的に
立て直している(倒立振子を手のひらで支え続けるのと同じ)。

- **足を置く場所(foothold placement)**: 穴を避ける、平らな所を選ぶ。
  → Quad-SDK の `getNearestValidFoothold` が担当。**今回これは解決した。**
- **バランス(balance / dynamic stability)**: 胴体の姿勢・速度を、限られた
  接地脚と GRF(地面反力)で目標へ引き戻す。
  → NMPC + WBC(逆動力学レッグコントローラ)が担当。**ここが縁で破綻する。**

「足場を穴の外に置く」だけでは足りない。**置くまでの遊脚軌道、置いた瞬間の
接地衝撃、支持脚の切り替え**が、穴の縁という地形の変化点で乱れると、
トロットの薄い安定余裕を食い潰して倒れる。

### 6.2 縁で何が起きているか

body x ≈ 0.68 m のとき、前脚の hip は x ≈ 0.87 m ── ちょうど穴の物理的な縁。
このタイミングで:

- 前脚の遊脚軌道は、Raibert 則の目標(穴の中)が計算され、それが
  `getNearestValidFoothold` で凸条へスナップされる。目標が cycle ごとに
  「近端に短く」「向こう側に長く」揺れると、遊脚は行き先を見失って脚が
  ジャークする(→ ヒステリシス 2.0 で軽減を試みた)。
- スナップで**歩幅が急に変わる**(例: 0.9 m → 1.28 m の 0.38 m ジャンプ)と、
  NMPC が想定する GRF 配分と、WBC が実際に出すトルクの間にズレが出て、
  胴体に余計な力/モーメントがかかる。
- go2 のトロットは平地でも安定余裕が小さい(handoff)。上記の擾乱が
  その余裕を超えると、まず前方ピッチ、続いて対角接地の非対称から roll が
  立ち上がり、π まで回って横倒れになる。

### 6.3 何が必要か

穴を「渡る」には、**穴を避ける足場計画**の上に、**縁の擾乱に耐える
バランス制御**が要る。後者は:

- トロットの `period` / `duty_cycle` を支持重複が増える方向に調整
- 遊脚軌道の生成・接地検出(early/late contact)を縁で頑健化
- 支持脚切り替え時の GRF / トルクの不連続を抑える
- そもそも go2 Quad-SDK トロットの平地不安定性(未解決)を先に潰す

これらはパラメータ数点のチューニングでは届かない、WBC/歩容側の別規模の課題。

---

## 7. 再現方法

```bash
# 1) 穴ワールド + 地形 PLY を生成(external/quad-sdk へ書き込む)
python3 src/trial/assets/gen_quadsdk_gap_world.py 2.0 1.0 2m 0.04     # step03_1m
python3 src/trial/assets/gen_quadsdk_gap_world.py 1.5 1.0 1p5m 0.04   # step04_1m

# 2) install/ に symlink(または colcon build --packages-select quad_sim_scripts)
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
for w in flat_gaps_2m flat_gaps_1p5m; do
  ln -sfn "$PWD/$SRC/worlds/$w.xml.xacro" "$INST/worlds/$w.xml.xacro"
  ln -sfn "$PWD/$SRC/models/$w"           "$INST/models/$w"
done

# 3) 修正を反映(DIAG 入り)
source /opt/ros/jazzy/setup.bash
( cd ros2_ws && colcon build --packages-select quad_utils local_planner \
    --symlink-install --allow-overriding quad_utils local_planner )

# 4) 実行
GAP_WORLD=flat_gaps_2m.xml GAP_TAG=step03_1m FORWARD_VEL_MPS=0.3 DURATION_S=25 \
  bash scripts/trial/run_quadsdk_gap_1m.sh
```

出力: `artifacts/logs/quadsdk_step03_1m/{state_log.csv, trials_summary.csv}` +
`.../logs/*.mp4`(いずれも `.gitignore` 対象)。

## 8. 次の手(候補)

1. go2 Quad-SDK トロットの堅牢化を深掘り(平地の非決定的転倒から)
2. PyMPC 路線に戻り、Step 03/04(浅い轍は成功済み)に **reference foothold を
   既知マップで穴外へずらす小モジュール**を足す(深さ非依存の回避)
3. `map_dip` / `foothold_search_radius` / 前方バイアス係数の探索を継続
   (縁の擾乱そのものは残るので効果は限定的と予想)

## 9. 変更・追加ファイル一覧

**MPC_DOG 側(新規):**
- `src/trial/assets/gen_quadsdk_gap_world.py`
- `scripts/trial/run_quadsdk_gap_1m.sh`
- `docs/steps/step_03_04_1m_quadsdk_gap_crossing.md`(本ファイル)

**external/quad-sdk(新規):**
- `quad_simulator/quad_sim_scripts/worlds/flat_gaps_2m.xml.xacro` / `flat_gaps_1p5m.xml.xacro`
- `quad_simulator/quad_sim_scripts/models/flat_gaps_2m/meshes/flat_gaps_2m.ply` / `flat_gaps_1p5m/...`

**external/quad-sdk(変更、WIP):**
- `quad_utils/config/go2.yaml`(`foothold_search_radius` 0.25→0.55)
- `quad_utils/config/filter_chain.yaml`(平滑化半径を縮小)
- `local_planner/src/local_footstep_planner.cpp`(前方バイアス + ヒステリシス、DIAG)
- `local_planner/src/local_planner.cpp`(DIAG のみ)
- `quad_utils/src/mjcf_to_grid_map_converter.cpp`(DIAG のみ)

## 10. 関連

- `docs/steps/step_03_gap_crossing.md` / `step_04_gap_crossing_1p5m.md`(PyMPC、浅い轍、成功)
- `agent_reports/quadsdk_step01_gait_and_mpc.md`(歩容と MPC の役割分担)
- `agent_reports/quadsdk_step01_terrain_map.md`(地形マップ = PLY 由来)
- `agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md`(穴超えの整理)
