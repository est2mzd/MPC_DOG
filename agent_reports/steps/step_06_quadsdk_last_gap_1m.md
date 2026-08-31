# Step 06:Step 05 の最後の穴だけ 1 m にした場合、落ちずに止まれるか

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。
Step 05(15 cm 平地 / 15 cm 穴の連続、go2 は N=2〜5 で渡れた)の派生。

## 背景

- Step 05 で go2 は 15 cm 平地 / 15 cm 穴の連続区間を渡り切れた。
- Step 05b で、幅 10 m / 100 cm の**単独の断崖**の手前では Phase 3(A) が
  安全停止できた(手前で直立保持)。
- **今回の問い(ユーザー)**:Step 05 の地形の**最後の穴だけ 1 m** にしたら、
  「渡れる 15 cm 穴は渡って、最後の 1 m 穴の手前で**落ちずに止まれる**か」。
  途中まで渡って止まっても可。
  環境認識部で**認識範囲を規定し、認識範囲内で渡れないと判断して止める**
  方法でも可(= Phase 3(A) の forward-probe がまさにそれ:
  `max_crossable_gap` が「渡河可能と見なす前方距離 = 認識/到達範囲」)。

## 目的

`gen_quadsdk_repeated_gap_world.py` に `LAST_GAP` を足し、
**15 cm 穴 ×2 → 15 cm 平地 → 1 m 穴** の地形を作って走らせ、
- 15 cm 穴を渡れるか
- 1 m 穴の手前で落ちずに止まれるか(途中まででも可)
- 止まれない場合、原因はどこか
を確認する。

## 結論

- **現状の Phase 2A + Phase 3(A) では「落ちずに止まる」を再現できない。**
  7 回中:
  - **5 回 転倒**(roll → ±π、x≈2.2〜2.3 = **手前の 15 cm 穴の位置**で転ぶ)
  - **1 回 出発点で凍結**(x≈0、直立のまま一歩も踏み出さず)
  - **1 回 15 cm 穴 ×2 を渡ったあと 1 m 穴の手前で傾いて停止**
    (x≈2.20、roll 0.75 / pitch 0.39、z 0.20 — 転倒はしていないが直立でもない)
- **Phase 3(A) の「認識範囲内で渡れないと判断」自体は正しく働いている**:
  strip_2(1 m 穴の直前の 15 cm 平地、x∈[2.60, 2.75])上の足場は
  `EDGE_TOO_CLOSE`(DIAG:`nominal x≈2.70 status=4 edge_clr=0.05〜0.10`)。
  forward-probe が 1 m 穴を「向こう岸が `max_crossable_gap`=0.6 m 以内に無い穴」
  と正しく判定している。
- **問題は「止め方」**:Phase 2A は **ホライズン内(idx 39、約 1.2 s 先)の
  touchdown が 1 つでも無効なら local plan を丸ごと publish しない**。
  1 m 穴は、胴体がまだ助走面 / 手前の 15 cm 穴を渡っている段階で、
  **遠方ホライズンの touchdown が strip_2 に射影された時点**で無効判定になる
  (最初の `[safe-stop]` は `first: leg=2 horizon_idx=39`)。
  → plan が数十回/run 断続的に止まり(`safe-stop` 28〜35 回/run)、
  go2 が **手前の 15 cm 穴を渡っている最中に遊脚を宙に浮かせたまま plan が
  凍結** → バランスを崩して転倒(2/3〜5/7)。
  凍結が「最初の離脚より前」に間に合った 1 回だけ、出発点で静止できた(r3)。
- **必要なもの:Phase 2B(能動的な停止シーケンス)。**
  「渡れない穴をホライズンに検知 → **stop をラッチ** → 遊脚を着地させる →
  新しい離脚を穴方向へ始めない → 減速して全脚接地 → STAND 保持」。
  現状の「plan を止める → 0.1 s 後に PD ホールド」は、遊脚中に起きると転ぶ。
  あるいはユーザー案どおり **「認識範囲内に渡れない穴があるなら、
  15 cm 穴の連続区間に踏み込む前に止まる」を最初の離脚前にラッチ**する
  (r3 の挙動を 1/7 → 7/7 にする)。どちらも stop シーケンスの設計で、
  `agent_reports/handoff/quadsdk_gap_foothold_handoff.md` の Phase 2B の範囲。

---

## 事実(コード・ログで確認)

| # | 事実 | 根拠 |
|---|---|---|
| 6-1 | 地形 = 助走 [-3, 2.0] / strip_0 [2.0,2.15] / 15cm穴 / strip_1 [2.30,2.45] / 15cm穴 / strip_2 [2.60,2.75] / **1 m 穴 [2.75,3.75]** / 着地 [3.75, 9.75] | `gen_quadsdk_repeated_gap_world.py 0.15 0.15 3 2.0 1.0 s15g15n3_last100 0.05 1.0` の出力 |
| 6-2 | Terrain Map は穴を認識(生 z=NaN)。N=3 相当で `finite=22100/25500` | `[DIAG] addLayerFromPolygonMesh` |
| 6-3 | strip_2 上の足場が `EDGE_TOO_CLOSE`(status=4)。forward-probe が 1 m 穴を渡河不可と判定 | `[DIAG] gnvf nominal x≈2.70 ... status=4 edge_clr=0.05〜0.10` |
| 6-4 | 最初の `[safe-stop]` は遠方 touchdown(`horizon_idx=39`)起因。胴体がまだ x≈0〜0.5 の段階で発火 | `[safe-stop] withholding local plan: 1 touchdown(s) ... first: leg=2 horizon_idx=39 status=4` |
| 6-5 | plan の断続停止(`safe-stop` 28〜35 回/run)中に、15 cm 穴を渡る遊脚が宙に浮いたまま凍結 → 転倒 | CSV:5/7 で roll→±π、z→0.05〜0.06(strip 上に崩れる)、転倒 x≈2.2〜2.3 |

## 7 回の結果

| run | safe-stop | 最終 x | 最終 z | 最終 roll | min z | 判定 |
|---:|---:|---:|---:|---:|---:|---|
| r1 | 27 | 2.19 | 0.06 | −π | 0.054 | 転倒(手前の 15 cm 穴で) |
| r2 | 30 | 2.26 | 0.06 | −π | 0.054 | 転倒 |
| r3 | 27 | −0.01 | 0.32 | 0.00 | 0.306 | **出発点で静止(一歩も出ず)** |
| r4 | 35 | 2.33 | 0.06 | −1.89 | 0.055 | 転倒 |
| r5 | 28 | 2.20 | 0.06 | −π | 0.054 | 転倒 |
| r6 | 35 | 2.20 | 0.20 | 0.76 | 0.198 | 15 cm 穴 ×2 を渡り、1 m 穴手前で傾いて停止(直立ではない) |
| r7 | 34 | 2.27 | 0.06 | π | 0.055 | 転倒 |

- 「落ちずに止まる」(直立で静止・穴に落ちない)を満たしたのは **0/7**。
  r3 は静止だが一歩も進んでおらず、r6 は傾き停止で直立ではない。
- 転倒はいずれも**手前の 15 cm 穴の位置**(x≈2.2)で、1 m 穴(x≥2.75)には
  到達していない。

## 未確認・保留

- `edge_clearance` / `max_crossable_gap` の値を変えたときの挙動(未掃引)。
- 遅い指令速度(0.15 m/s)での再現(Step 05b では 0.15 でも同傾向だった)。
- 「WALK を送る前に、認識済みマップに渡れない穴があれば WALK 自体を保留」
  というハーネス側の対処(未試行)。

## 再現

```bash
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
sed -i 's/^\(      edge_clearance: \)0.0\b/\10.15/' "$YAML"   # 実行後 0.0 へ戻す
python3 src/trial/assets/gen_quadsdk_repeated_gap_world.py 0.15 0.15 3 2.0 1.0 s15g15n3_last100 0.05 1.0
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
ln -sfn "$PWD/$SRC/worlds/flat_repgap_s15g15n3_last100.xml.xacro" "$INST/worlds/flat_repgap_s15g15n3_last100.xml.xacro"
ln -sfn "$PWD/$SRC/models/flat_repgap_s15g15n3_last100" "$INST/models/flat_repgap_s15g15n3_last100"
( cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select local_planner --symlink-install --allow-overriding local_planner )
GAP_WORLD=flat_repgap_s15g15n3_last100.xml GAP_TAG=quadsdk_step06_last100 FORWARD_VEL_MPS=0.3 DURATION_S=35 \
  bash scripts/trial/run_quadsdk_gap_1m.sh
sed -i 's/^\(      edge_clearance: \)0.15\b/\10.0/' "$YAML"
```

証拠 GIF:`artifacts/gifs/quadsdk_step06_last1m_fall_10to30s.gif`
(手前の 15 cm 穴で断続停止 → 転倒する典型例、10–30 s 切り抜き)。

## 追加・変更ファイル

- 変更 `src/trial/assets/gen_quadsdk_repeated_gap_world.py`(`LAST_GAP` 引数)
- 新規 `external/quad-sdk/.../worlds/flat_repgap_s15g15n3_last100.xml.xacro` +
  `models/flat_repgap_s15g15n3_last100/`
- 新規 `artifacts/gifs/quadsdk_step06_last1m_fall{,_10to30s}.gif`
- 制御コード変更なし(Phase 2A / Phase 3(A) のまま)。

## 関連

- `agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md`(15 cm 連続穴、通過)
- `agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md`(単独断崖の安全停止)
- `agent_reports/quadsdk_gap_foothold_phase_progress.md` §Phase 2A / §Phase 3
- `agent_reports/handoff/quadsdk_gap_foothold_handoff.md`(Phase 2B 設計項目)
