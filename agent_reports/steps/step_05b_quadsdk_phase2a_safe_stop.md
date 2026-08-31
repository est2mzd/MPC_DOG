# Step 05b:安全停止の検証(幅 10 m の穴・穴の手前で止まれるか)— Phase 2A → Phase 3 で成立

> **更新(Phase 3 実施後)**:Phase 2A だけでは転落したが、**Phase 3
> (`EDGE_TOO_CLOSE` = 穴縁からの安全距離)を有効(`edge_clearance:=0.15`)に
> すると、go2 は穴の約 0.7 m 手前で停止し、そのまま直立を保った(≈10 s、
> 試行終了まで転落せず)。ユーザー基準「穴の手前で 3 秒止まれたら OK」を満たす。**
> 証拠 GIF: `artifacts/gifs/quadsdk_phase3_trench10m_safestop.gif`。
> 以下、Phase 2A 単独の記録(§1〜)と Phase 3 の結果(§末尾「Phase 3 で解決」)。


対象: `external/quad-sdk`(go2、`reference:=twist`)。Step 05(15 cm 連続穴)の
前に、ユーザー指定で **安全停止を先に検証** した記録。

## 背景

- Step 05(15 cm 平地・15 cm 穴の連続区間)の事前調査で、
  「安全支持幅が地図 1 セル・足先寸法ぎりぎり」で幾何学的に成立困難、
  かつ **無効足場を NMPC に渡さない安全停止(Phase 2A)が当時未実装** と判明した
  (`step_05_quadsdk_repeated_15cm_gaps.md`)。
- ユーザー回答:**「安全停止を先にやる。検証シナリオ = 進行方向に幅 10 m の穴を
  用意し、穴の手前で 3 秒止まれたら OK」。**
- そこで Phase 2A を実装(`quadsdk_gap_foothold_phase_progress.md` §Phase 2A)し、
  幅 10 m・深さ 1 m のトレンチで検証した。

## 目的

1. Phase 2A の「無効足場を検出したら local plan を publish しない」が
   シミュレーションで実際に発火するか。
2. その結果、go2 が **穴に落ちず・転ばず・穴の手前で 3 秒以上停止** できるか。
3. できない場合、何が足りないか(どの Phase が必要か)を切り分ける。

## 結論

- **Phase 2A の足場ガードは発火した(確認済み)。** 前方 touchdown の名目足場が
  穴帯へ入り `foothold_search_radius`(0.7 m)内に有効セルが無くなると
  `FootholdStatus::NO_TRAVERSABLE_CANDIDATE` になり、`computeLocalPlan()` が
  `stop_on_invalid_foothold`(既定 ON)で `false` を返して plan を publish しない。
  ログ `[safe-stop] withholding local plan: N touchdown(s) ...` を確認
  (17 回/試行、1 s throttle)。
- **しかし「穴の手前で安全に停止」には至らなかった(確認済み、3 試行)。**
  検出タイミングが遅く、受動的な停止では前進の勢いを止めきれない:
  - 足場は穴帯の名目でも **穴の縁(x≈1.95〜2.0、物理縁 x=2.0)へスナップして
    `VALID` を返し続ける**。無効判定が出るのは名目が縁から 0.7 m 以上先に
    入ってから = 胴体が縁の約 0.5 m 手前まで来たとき。
  - そこで plan を止めても、`robot_driver` は 0.1 s 後に
    `stand_joint_angles` へ PD ホールドするだけ(**能動的な減速・着地シーケンス
    は無い**)。前進速度が乗った状態で縁で姿勢を固定 → 前のめりに転落。
  - 3 試行の結果:
    | 試行 | 指令速度 | 停止できた時間 | 最終 |
    |---|---|---|---|
    | run1 | 0.3 m/s | 縁(x≈1.8)で約 11 s 保持 | その後ゆっくり前傾して転落 |
    | run2 | 0.15 m/s | 縁(x≈1.9)で約 5 s 保持 | その後転落(横倒れ) |
    | run3 | 0.3 m/s | ほぼ 0 s(勢いのまま)| 即転落(x=2.6, z=−0.94, 上下反転) |
  - go2 twist 歩容は既知の非決定性があり(Step 01)、run ごとに保持時間が
    ばらつく。いずれも**最終的に穴へ落ちた**。
- **判定**:ユーザー基準「穴の手前で 3 秒止まれたら OK」は run1 のみ字義上満たすが、
  **再現性が無く、最終的に転落するので安全停止としては不成立**。
- **不足しているもの(次にやるべき Phase)**:
  1. **Phase 2B(能動的な停止シーケンス)**:plan を止めるだけでなく、
     遊脚を安全な位置へ着地 → `cmd_vel`→0 → 全脚接地 → STAND 遷移。
     前進の勢いを能動的に殺す。
  2. **Phase 3(穴縁からの安全距離 `EDGE_TOO_CLOSE`)**:足場が縁ぎりぎりに
     スナップされる前に無効判定を出す(例:スナップ距離が大きい/選択セルが
     NaN セルに近い)。これで停止判断が **もっと手前**で出る。
  - 本シナリオを安定して通すには、少なくとも Phase 2B が要る(Phase 3 が
    あると停止位置に余裕ができてなお良い)。指示書 §18 の想定どおり。

---

## 事実(コード・ログで確認)

| # | 事実 | 根拠 |
|---|---|---|
| S1 | Phase 2A ガートが sim で発火 | `[safe-stop] withholding local plan` ログ(run3 で 17 回)。直前の `[DIAG] gnvf ... status=2`(`NO_TRAVERSABLE_CANDIDATE`)と対応 |
| S2 | `stop_on_invalid_foothold_` は実行時 `true` | 一時計装ログ `[MPC_DOG TMP] stop_on_invalid_foothold_=1`(検証後に撤去) |
| S3 | `computeFootPlan()` が `ok=0`(無効あり)を返し、`computeLocalPlan()` が `false` を返して非 publish | 一時計装ログ `foot_plan_result.ok=0 failed_count=5 stop_on_invalid=1` の直後に plan_age 上昇開始 |
| S4 | 足場は穴の名目でも縁へスナップし `VALID` を返し続ける | `[DIAG] gnvf: nominal x=2.20 trav=nan -> snapped x=1.95 found=1 status=0 snap=0.255`。`status=2` は名目 x≥2.65 付近から |
| S5 | 停止後、robot は縁で PD ホールド → 前傾 → 転落 | CSV:run1 は x≈1.8 で z=0.31 を約 11 s 保持後、pitch 0.07→0.74→1.10、z 0.30→−0.94、roll→±π |
| S6 | `[safe-stop]` ログが最初 0 件だったのは throttle 実装バグ | `RCLCPP_WARN_THROTTLE` の間隔を `5e8`(ms=約 5.8 日)にしていた。sim クロックが 0 付近始まりだと `経過 >= 間隔` が一度も成立せず**一度も出ない**。`1000`(ms)へ修正。既存コードの `1e9` throttle 数箇所にも同じ潜在バグがあるが本タスク範囲外 |

## 未確認・保留

- Phase 2A ガードの発火は確認したが、`stop_on_invalid_foothold:=false` との
  A/B(ガード無しだともっと早く落ちるか)は未取得。
- 「足場が縁へスナップし続ける」→「胴体が縁へ 0.5 m」までガードが出ない、の
  距離関係は DIAG から推定。厳密なマージン計測は未実施。
- NMPC 内訳ログ(cost 項別 / slack / IPOPT status)は未実装のまま
  (`quadsdk_gap_foothold_mpc_code_analysis.md` §6.1)。

## 再現

```bash
# 1) 幅10m トレンチ地形を生成(external/quad-sdk へ書き込み)
python3 src/trial/assets/gen_quadsdk_wide_trench_world.py 10.0 2.0 1.0 10m 0.05

# 2) install ツリーへ symlink
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
ln -sfn "$PWD/$SRC/worlds/flat_trench_10m.xml.xacro" "$INST/worlds/flat_trench_10m.xml.xacro"
ln -sfn "$PWD/$SRC/models/flat_trench_10m"           "$INST/models/flat_trench_10m"

# 3) Phase 2A 入りの local_planner をビルド
source /opt/ros/jazzy/setup.bash
( cd ros2_ws && colcon build --packages-select local_planner --symlink-install --allow-overriding local_planner )

# 4) 実行(既存ハーネスを流用。GAP_WORLD/GAP_TAG で地形を差し替え)
GAP_WORLD=flat_trench_10m.xml GAP_TAG=quadsdk_phase2a_trench10m \
  FORWARD_VEL_MPS=0.3 DURATION_S=25 bash scripts/trial/run_quadsdk_gap_1m.sh

# 5) GIF(目視確認)
bash scripts/trial/make_gif.sh \
  artifacts/logs/quadsdk_quadsdk_phase2a_trench10m/logs/mujoco_go2_*.mp4 \
  artifacts/gifs/quadsdk_phase2a_trench10m_fall.gif 8 520
```

出力 CSV/mp4 は `.gitignore` 対象。証拠 GIF `artifacts/gifs/quadsdk_phase2a_trench10m_fall.gif`
(縁で一瞬止まってから前傾転落する run3)のみ追跡。

## 追加・変更ファイル

- 新規 `src/trial/assets/gen_quadsdk_wide_trench_world.py`(幅可変トレンチ生成器)
- 新規 `external/quad-sdk/quad_simulator/quad_sim_scripts/worlds/flat_trench_10m.xml.xacro`
- 新規 `external/quad-sdk/quad_simulator/quad_sim_scripts/models/flat_trench_10m/meshes/flat_trench_10m.ply`
- 変更 `external/quad-sdk/local_planner/src/local_planner.cpp`(`[safe-stop]` throttle 間隔 `5e8`→`1000` ms のバグ修正のみ。挙動ロジックは不変)
- 新規 `artifacts/gifs/quadsdk_phase2a_trench10m_fall.gif`

---

## Phase 3 で解決(`EDGE_TOO_CLOSE` = 穴縁からの安全距離)

Phase 2A 単独の敗因は「足場が縁ぎりぎり(x≈1.95)にスナップして `VALID` を
返し続け、無効判定が出るのが遅い」だった。そこで **Phase 3** を実装:

- 足場選択後、`edge_clearance`(m)以内に **地図外 / 非有限 /
  `traversability ≤ 0.6` のセル** があれば `FootholdStatus::EDGE_TOO_CLOSE`
  にする(`getNearestValidFootholdResult`)。
- `EDGE_TOO_CLOSE` は `VALID` でないので **Phase 2A がそのまま拾い**、
  `computeLocalPlan()` が plan を withhold する。追加配線は不要。
- 既定 `edge_clearance: 0.0`(無効=Phase 3 前の挙動)。**step03/04 の溝渡りは
  縁へのスナップを前提にしているので、既定 0 で不変。run ごとに opt-in。**
- 詳細:`agent_reports/quadsdk_gap_foothold_phase_progress.md` §Phase 3。

### 結果(`edge_clearance:=0.15`、幅 10 m トレンチ、0.3 m/s)

| 項目 | Phase 2A 単独 | Phase 3 有効 |
|---|---|---|
| 停止時の胴体 x | x≈1.8〜2.0(縁の直上) | **x≈1.30(縁の約 0.7 m 手前)** |
| 停止時の前進速度 | 0.5〜0.7 m/s(勢いあり) | 0.16 m/s → 0 |
| 停止後 | 前傾 →(5〜11 s 後)穴へ転落 | **直立を保持(≈10 s、試行終了まで転落なし)** |
| 最終姿勢 | 穴底で上下反転 / 横倒れ | **x=1.30, z=0.32, roll/pitch < 0.02 rad、直立** |
| ガードの status | `NO_TRAVERSABLE_CANDIDATE`(2) | `EDGE_TOO_CLOSE`(4)。DIAG:`nominal x=1.938 status=4 edge_clr=0.100`(縁まで 0.10 m < 0.15 でトリップ)|

- `[safe-stop]` ログ 20 回、`status=4`(EDGE_TOO_CLOSE)200 回。
- **判定:成立**(穴に落ちない・転ばない・無効足場を NMPC へ渡さない・
  穴の手前で 3 秒以上停止 → すべて満たす)。
- go2 twist の非決定性はあるため、複数回・複数速度での再現確認は次イテレーションで
  行う(現時点は 1 回)。

### 追加検証:step03/04 の回帰

`edge_clearance` 既定 0.0 では `getNearestValidFootholdResult` の Phase 3 ブロックは
`if (... && edge_clearance_ > 0.0)` で丸ごとスキップされる → 挙動はバイト単位で不変。
単体 `EdgeClearanceLeavesInteriorAndDisabledCaseValid` で opt-out を確認済み。
`flat_gaps_2m`(step03_1m、0.3 m/s、既定設定)の実走回帰も実施(結果は
`quadsdk_gap_foothold_phase_progress.md` に記録)。

### Phase 3 初版の欠陥と修正(forward-probe 化)

**初版**は「足場の全方位 `edge_clearance` 以内に穴セルがあれば `EDGE_TOO_CLOSE`」
だった。これだと **15 cm など渡れる穴の手前でも一律に停止**してしまう
(ユーザー指摘)。掃引でも strip/gap を 15/15〜50/15 と変えても全部
「穴に届く前に安全停止」だった。step03/04 は **30 cm の穴**を「縁に足を置いて
跨ぐ」で渡れているので、これは過剰。

**修正(A、forward-probe)**:足場から**進行方向(+x)へ 1 本スキャン**する。
「`edge_clearance` 以内で穴が始まり、その先 `max_crossable_gap`(既定 0.6 m)
以内に固い地面が戻らない」ときだけ `EDGE_TOO_CLOSE`。

- 足場の**手前(後ろ)にある穴**は無視(渡り終えた穴の遠い縁で止まらない)。
- `edge_clearance` より遠い穴も無視。
- 渡れる穴(向こう岸が届く)は `VALID` のまま。

**再検証(ユーザー指定の 2 シナリオ、`edge_clearance:=0.15`、0.3 m/s、各 1 回)**:

| シナリオ | 地形 | 結果 | 最終 |
|---|---|---|---|
| **30 cm 穴** | `flat_gaps_2m`(step03/04、深さ 1 m、間隔 2 m)| **渡り切った**(`safe-stop` 0 回)| x=9.74、z=0.31、roll/pitch < 0.03 rad、歩行継続 |
| **100 cm 穴** | `flat_trench_1m`(新規、深さ 1 m)| **穴の約 0.63 m 手前で安全停止・直立保持**(`safe-stop` 26 回)| x=1.37、z=0.32、roll/pitch < 0.01 rad |

→ **Phase 3(A)は「渡れる穴は渡る・渡れない穴/断崖の手前で安全に止まる」を
両立できた。** 証拠 GIF:`artifacts/gifs/quadsdk_phase3_gap30_cross.gif` /
`artifacts/gifs/quadsdk_phase3_gap100_safestop.gif`。
単体テスト **34/34 green**(`makeTerrainWithGapBand` ベースに再構成:
1.45 m 穴の手前→`EDGE_TOO_CLOSE`、0.30 m 穴の手前→`VALID`、
穴が後ろ/遠い/`edge_clearance==0`→`VALID`)。

**注意**:forward-probe は **+x(進行方向)固定**。本 Step の全幅横断穴・
Step 05 では妥当だが、斜めの穴や旋回時は body 速度方向でのスキャンが要る
(将来の一般化課題)。

## Phase 3 を含む追加・変更ファイル

- 変更 `external/quad-sdk/local_planner/include/local_planner/local_footstep_planner.hpp`
  (`FootholdStatus::EDGE_TOO_CLOSE`、`FootholdResult.edge_clearance`、
  `setSpatialParams` に `edge_clearance` 引数、`edge_clearance_` メンバ)
- 変更 `external/quad-sdk/local_planner/src/local_footstep_planner.cpp`
  (edge-clearance スパイラル走査、DIAG に `edge_clr` 追記)
- 変更 `external/quad-sdk/local_planner/src/local_planner.cpp`
  (`local_footstep_planner.edge_clearance` を `loadROSParamDefault` で読む、既定 0.0)
- 変更 `external/quad-sdk/local_planner/config/local_planner.yaml`(`edge_clearance: 0.0` キー追加)
- 変更 `external/quad-sdk/local_planner/test/test_footstep_planner.cpp`(Phase 3 テスト 2 本)
- 新規 `artifacts/gifs/quadsdk_phase3_trench10m_safestop.gif`

### Phase 3 の再現

```bash
# local_planner.yaml の edge_clearance を一時的に 0.15 にして実行(実行後 0.0 へ戻す)
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"
( cd ros2_ws && source /opt/ros/jazzy/setup.bash && \
  colcon build --packages-select local_planner --symlink-install --allow-overriding local_planner )
GAP_WORLD=flat_trench_10m.xml GAP_TAG=quadsdk_phase3_trench10m_ec0p15 \
  FORWARD_VEL_MPS=0.3 DURATION_S=25 bash scripts/trial/run_quadsdk_gap_1m.sh
sed -i 's/^\(      edge_clearance: \)0.15\b/\10.0/' "$YAML"
```

## 関連

- `agent_reports/quadsdk_gap_foothold_phase_progress.md` §Phase 2A / §Phase 3(実装詳細)
- `agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md`(Step 05 事前調査)
- `agent_reports/handoff/quadsdk_gap_foothold_handoff.md`(Phase 2B 設計項目)
