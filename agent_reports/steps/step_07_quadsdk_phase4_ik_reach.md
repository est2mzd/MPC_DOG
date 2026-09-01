# Step 07:Phase 4(`IK_UNREACHABLE`)の動作確認 — 脚が届かない足場を検知して安全停止

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。

## 背景

- 足場スナップ(`getNearestValidFoothold`)は `foothold_search_radius`(0.7 m)
  まで候補をずらせる。一方 Go2 の脚の到達域は約 **0.4 m**。NMPC も可到達性を
  見ない(制約は運動方程式 + 摩擦のみ)。
- → **届かない足場が NMPC の固定パラメータになる**と、支持/遊脚が破綻して転倒。
- Phase 4:選択足場を**既存の脚 IK**(`worldToFootIKWorldFrame`)で touchdown
  時の予測胴体姿勢からチェックし、IK がクランプ(遠すぎ/関節限界/特異)=
  `false` を返したら `FootholdStatus::IK_UNREACHABLE`(既定 OFF、
  `ik_reach_check` パラメータで opt-in)。
- Step 05/05b/06 の掃引地形では **`IK_UNREACHABLE` を踏むケースが無かった**
  (足場スナップは常に手前=胴体側へ寄るので届く)。そこで Phase 4 が**実際に
  効く地形**を専用に作って動作確認する。

## 目的

「脚が届かない足場」を強制的に作り、
1. `ik_reach_check:=true` で `IK_UNREACHABLE` が発火するか
2. 発火した結果、Phase 2A gate + Phase 2B latch で**転ばずに安全停止**するか
3. `ik_reach_check:=false`(既定)だと同じ地形で転ぶか(対比)
を確認する。

## 結論

- **`ik_reach_check:=false`(既定)**:go2 は届かない前方足場を実行して
  **x≈1.2 で前のめりに転倒**(roll → −π)。
- **`ik_reach_check:=true`**:前方足場が `IK_UNREACHABLE`(`status=5`)になり、
  `[safe-stop] latching graceful stop: ... status=5` → 減速して
  **x≈0.80〜0.82 で直立静止**(z=0.31、roll/pitch≈0、`min z` 0.28、
  以後 試験終了まで安定)。**3/3 で転倒なし。**
- Phase 4 は `!= VALID` を返すだけで、下流(gate + latch)に**追加配線ゼロ**で
  つながることを実地で確認。

証拠 GIF:
- `artifacts/gifs/quadsdk_phase4_ik_safestop_10to30s.gif`(ON:届かない足場を
  検知して手前で直立停止)
- `artifacts/gifs/quadsdk_phase4_ik_fall_10to30s.gif`(OFF:同地形で転倒)

---

## 地形の作り方(なぜ IK_UNREACHABLE を強制できるか)

`gen_quadsdk_wide_trench_world.py` に `approach_margin` 引数を追加した。
**物理地面は普通のまま**、地形マップ(足場計画が見る面)だけを手前側に
大きく削る:

```bash
python3 src/trial/assets/gen_quadsdk_wide_trench_world.py 0.15 2.0 1.0 ikdemo 0.05 1.0
#  幅0.15  x0=2.0  深さ1.0  tag  mesh_margin0.05  approach_margin1.0
```

- **物理**:助走面 x∈[-3, 2.0] は solid、穴 x∈[2.0, 2.15](深さ 1 m、幅 15 cm
  なので落ちにくい)、着地面 x∈[2.15, …] solid。
- **地形マップ**:助走側メッシュを `approach_margin=1.0` 手前で切る →
  通行可能セルは **x ≤ 1.00**。着地側は x ≥ 2.20。
  → **マップ上の立入禁止帯 [1.00, 2.20](1.2 m)。物理的には歩ける。**
- ロボットが物理地面 x≈1.5 まで歩くと、前脚の名目足場(≈1.7、立入禁止帯の中)
  の最寄り有効セルは **助走端 1.00(0.7 m 後ろ)より着地面 2.20(0.5 m 前)が
  近い** → **前方 2.20 へスナップ**。胴体/股は x≈1.5 → 足場は 0.7 m 前方 =
  脚の到達域(≈0.4 m)を超える → **IK がクランプ = `IK_UNREACHABLE`**。
- `edge_clearance:0`(Phase 3 OFF)なので `EDGE_TOO_CLOSE` も前方 lookahead も
  効かない → **Phase 4 だけがこの足場を止める**(切り分け)。

## 事実(ログ・CSV)

| 設定 | `status=5` 回数 | latch | 最終 x | 最終 z | 最終 roll | min z | 判定 |
|---|---:|---:|---:|---:|---:|---:|---|
| `ik_reach_check:false` | 0 | 0 | 1.23 | 0.06 | −3.14 | 0.054 | 転倒 |
| `ik_reach_check:true` (r1) | ≥1 | 1 | 0.82 | 0.31 | 0.00 | 0.286 | 直立静止 |
| `ik_reach_check:true` (r2) | ≥1 | 1 | 0.81 | 0.31 | 0.00 | 0.282 | 直立静止 |
| `ik_reach_check:true` (r3) | ≥1 | 1 | 0.80 | 0.31 | 0.00 | 0.286 | 直立静止 |

- latch ログ:`[safe-stop] latching graceful stop: impassable gap in the
  horizon (leg=0/2 nearest_idx=39 status=5)`(status=5 = `IK_UNREACHABLE`)。
- **既存シナリオへの影響**:`ik_reach_check` 既定 false。この地形以外は
  Phase 4 前と不変(Step 05/05b/06 の回帰は Phase 4 コミット時に確認済み。
  `quadsdk_gap_foothold_phase_progress.md` §Phase 4)。

## 未確認・保留

- `IK_UNREACHABLE` が「クランプした = 厳密解でない」を全部拾うので、
  わずかなクランプ(実用上は許容範囲)でも止まりうる。閾値(クランプ量)を
  設けるかは Phase 4b(候補の再探索)と合わせて検討。
- forward-probe / lookahead 同様、この確認は **+x 進行**前提。

## 再現

```bash
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
python3 src/trial/assets/gen_quadsdk_wide_trench_world.py 0.15 2.0 1.0 ikdemo 0.05 1.0
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
ln -sfn "$PWD/$SRC/worlds/flat_trench_ikdemo.xml.xacro" "$INST/worlds/flat_trench_ikdemo.xml.xacro"
ln -sfn "$PWD/$SRC/models/flat_trench_ikdemo" "$INST/models/flat_trench_ikdemo"
( cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select local_planner --symlink-install --allow-overriding local_planner )
# ON(Phase 4 有効)
sed -i 's/^\(      ik_reach_check: \)false\b/\1true/' "$YAML"
GAP_WORLD=flat_trench_ikdemo.xml GAP_TAG=quadsdk_p4demo_on FORWARD_VEL_MPS=0.3 DURATION_S=30 \
  bash scripts/trial/run_quadsdk_gap_1m.sh
sed -i 's/^\(      ik_reach_check: \)true\b/\1false/' "$YAML"
```

## 追加・変更ファイル

- 変更 `src/trial/assets/gen_quadsdk_wide_trench_world.py`(`approach_margin` 引数)
- 新規 `external/quad-sdk/.../worlds/flat_trench_ikdemo.xml.xacro` +
  `models/flat_trench_ikdemo/`
- 新規 `artifacts/gifs/quadsdk_phase4_ik_safestop{,_10to30s}.gif` /
  `quadsdk_phase4_ik_fall{,_10to30s}.gif`
- 制御コード変更なし(Phase 4 は `f93d1f2` / `a7d222f`)。

## 関連

- `agent_reports/quadsdk_gap_foothold_phase_progress.md` §Phase 4(実装詳細)
- `agent_reports/quadsdk_gap_foothold_summary.md`(まとめ)
- `agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md`(Phase 2A/3 の安全停止)
- `agent_reports/steps/step_06_quadsdk_last_gap_1m.md`(Phase 2B の安全停止)
