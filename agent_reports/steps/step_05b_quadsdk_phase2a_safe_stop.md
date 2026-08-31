# Step 05b:Phase 2A 安全停止の検証(幅 10 m の穴・穴の手前で止まれるか)

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

## 関連

- `agent_reports/quadsdk_gap_foothold_phase_progress.md` §Phase 2A(実装詳細)
- `agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md`(Step 05 事前調査)
- `agent_reports/handoff/quadsdk_gap_foothold_handoff.md`(Phase 2B 設計項目)
