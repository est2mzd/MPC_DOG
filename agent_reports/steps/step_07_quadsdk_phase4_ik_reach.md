# Step 07:Phase 4(`IK_UNREACHABLE`)の動作確認 — 30 cm / 100 cm × ON/OFF の 4 通り

対象: `external/quad-sdk`(go2、`reference:=twist`、クロール歩容、0.3 m/s)。

## 背景

- 足場スナップ(`getNearestValidFoothold`)は `foothold_search_radius`(0.7 m)
  まで候補をずらせる。一方 Go2 の脚の全伸長は約 **0.42 m**。NMPC も可到達性を
  見ない → **届かない足場が NMPC の固定パラメータになる**と支持姿勢が破綻して転倒。
- Phase 4:選択足場と、その足が支える **midstance hip**(`computeFootPlan` が
  計算済み)との距離が `ik_max_reach`(既定 **0.45 m**)を超えたら
  `FootholdStatus::IK_UNREACHABLE`。`ik_reach_check`(既定 false)で opt-in。

### 初版のミスと修正(2026-09-01、ユーザー指摘)

**初版**は `worldToFootIKWorldFrame` の戻り値(`is_exact`)で判定していた。
これは**わずかなクランプ(実用上は許容)でも false** を返すため、平地歩行中の
足場まで `IK_UNREACHABLE` になり、**30 cm の溝渡り(渡れるはず)を x≈0.35 で
止めてしまった = 機能後退**。

**修正**:`is_exact` をやめ、**midstance hip からの幾何距離 > `ik_max_reach`**
のみで判定。参照 hip を `worldToNominalHipFKWorldFrame(row(i))`(touchdown
開始時の hip)ではなく、足場が実際に支える **midstance hip**(足場 nominal の
基準そのもの)にしたことで、遠方ホライズンの加速中足場の誤検知も消えた。

## 目的

ユーザー指定:**穴幅 30 cm / 100 cm** × **`ik_reach_check` OFF / ON** の
**4 通り**で、Phase 4 ON が
1. **30 cm(渡れる穴)を止めない**(= 機能後退しない)
2. **100 cm(渡れない穴)は従来どおり手前で停止**
ことを確認する。あわせて、Phase 4 が**実際に効く**専用地形(`ikdemo`)でも確認。

## 結論

| 地形 | `edge_clearance` | `ik_reach_check` | 結果 | 期待どおりか |
|---|---|---|---|---|
| **30 cm 溝**(`flat_gaps_2m`) | 0.15 | **OFF** | 渡り切る(x≈11.3) | ✓ |
| **30 cm 溝** | 0.15 | **ON** | **渡り切る**(x≈11.9、`IK_UNR`=0、latch なし) | **✓ 機能後退なし** |
| **100 cm 穴**(`flat_trench_1m`) | 0.15 | **OFF** | 手前で直立停止(x≈0.16) | ✓ |
| **100 cm 穴** | 0.15 | **ON** | 手前で直立停止(x≈0.17) | ✓ 変化なし |
| ikdemo(助走側マップを削った地形) | 0 | **OFF** | 届かない足場を実行して**転倒**(x≈1.3、roll→−π) | ―(Phase 4 なし)|
| ikdemo | 0 | **ON** | **`IK_UNREACHABLE`(status=5)発火 → 手前で直立停止**(x≈0.8、2/2) | ✓ Phase 4 が効く |

- **`ik_max_reach=0.45`** で、平地・溝渡り・近縁スナップの足場(hip から ~0.30〜
  0.40 m)は通し、`ikdemo` の強制前方スナップ(hip から ~0.75 m)だけ止める。
- Phase 4 は `!= VALID` を返すだけで、下流(gate 2A + graceful-stop latch 2B)に
  **追加配線ゼロ**でつながる。
- 既定 `ik_reach_check: false` なので、**有効化して検証するまで全シナリオ不変**。
  `local_planner` テスト **40/40 green**。

証拠 GIF(README「機能 ON/OFF の比較」に埋め込み):
- `quadsdk_onoff_g30_off.gif` / `quadsdk_onoff_g30_on.gif`
  (30 cm の溝:機能 OFF/ON いずれも渡り切る = 機能後退なし)
- `quadsdk_onoff_g100_off.gif`(100 cm の穴:機能 OFF → 4.6 m 歩いて穴に落下)
- `quadsdk_onoff_g100_on.gif`(100 cm の穴:機能 ON → 2.2 m 歩いて手前で直立停止)

> **100 cm の GIF は spawn を x=−2.0 に後退**(`SPAWN_X_M=-2.0`、`run_quadsdk_gap_1m.sh`
> に追加した env)して撮影。安全停止は穴の近縁(x=2.0)の約 1.9 m 手前
> (`safe_stop_lookahead 2.5 − max_crossable_gap 0.6`)でラッチするため、
> spawn x=0 だと助走がほぼ無く「歩いてから止まる」様子が見えないため。
> 地形マップは world 固定なので spawn 位置に依存しない。
> ON: x=−2.04→+0.20(2.24 m 歩行、latch 1 回、直立)。
> OFF: x=−2.04→+2.55(4.59 m 歩行、穴に落下、roll→π)。

---

## Phase 4 が「効く」地形の作り方(ikdemo)

通常の穴掃引地形では足場スナップが**手前(胴体側)へ寄る**ので `IK_UNREACHABLE`
を踏まない。そこで `gen_quadsdk_wide_trench_world.py` に `approach_margin`
引数を足し、**物理地面は普通のまま、地形マップの助走側だけを大きく削る**:

```bash
python3 src/trial/assets/gen_quadsdk_wide_trench_world.py 0.15 2.0 1.0 ikdemo 0.05 1.0
#  幅0.15  x0=2.0  深さ1.0  tag  mesh_margin0.05  approach_margin1.0
```

- 物理:助走 x∈[-3, 2.0] solid、穴 x∈[2.0, 2.15](幅 15 cm なので落ちにくい)、
  着地 x∈[2.15, …] solid。
- 地形マップ:助走側を `approach_margin=1.0` 削る → 通行可能セルは x ≤ 1.00。
  着地側は x ≥ 2.20。**マップ上の立入禁止帯 [1.00, 2.20](1.2 m)。**
- ロボットが x≈1.5 まで歩くと、前脚の名目足場(立入禁止帯の中)の最寄り有効セルは
  **前方 2.20**(助走端 1.00 は 0.7 m 後ろで遠い)→ **前方 0.7 m へスナップ** →
  midstance hip から ~0.75 m → `> ik_max_reach(0.45)` → `IK_UNREACHABLE`。
- `edge_clearance:0`(Phase 3 OFF)なので `EDGE_TOO_CLOSE` も前方 lookahead も
  効かない → **Phase 4 だけがこの足場を止める**(切り分け)。

## 事実(ログ・CSV)

| tag | ec | ik | `status=5` | latch | 最終 x | 最終 z | 最終 roll | min z | 判定 |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| g30_ik0 | 0.15 | off | 0 | 0 | 11.25 | 0.32 | −0.00 | 0.304 | 渡り切る |
| g30_ik1 | 0.15 | on | 0 | 0 | 11.89 | 0.31 | −0.00 | 0.301 | 渡り切る |
| g100_ik0 | 0.15 | off | 0 | 1 | 0.16 | 0.31 | −0.00 | 0.307 | 手前で停止 |
| g100_ik1 | 0.15 | on | 0 | 1 | 0.17 | 0.31 | −0.00 | 0.305 | 手前で停止 |
| ikd_ik0 | 0 | off | 0 | 0 | 1.32 | 0.16 | −1.59 | 0.151 | 転倒 |
| ikd_ik1 | 0 | on | 1 | 1 | 0.82 | 0.31 | 0.00 | 0.274 | 直立停止 |
| ikd_ik1(r2) | 0 | on | 1 | 1 | 0.86 | 0.31 | −0.00 | 0.291 | 直立停止 |

latch ログ(ikdemo ON):`[safe-stop] latching graceful stop: impassable gap
in the horizon (leg=0/2 ... status=5)`(status=5 = `IK_UNREACHABLE`)。

## 再現

```bash
YAML=external/quad-sdk/local_planner/config/local_planner.yaml
SRC=external/quad-sdk/quad_simulator/quad_sim_scripts
INST=ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts
python3 src/trial/assets/gen_quadsdk_wide_trench_world.py 0.15 2.0 1.0 ikdemo 0.05 1.0
ln -sfn "$PWD/$SRC/worlds/flat_trench_ikdemo.xml.xacro" "$INST/worlds/flat_trench_ikdemo.xml.xacro"
ln -sfn "$PWD/$SRC/models/flat_trench_ikdemo" "$INST/models/flat_trench_ikdemo"
( cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select local_planner --symlink-install --allow-overriding local_planner )
# 4 通り(ec=0.15, ik=off/on for 30cm & 100cm)は edge_clearance と ik_reach_check を sed で切替
sed -i 's/^\(      ik_reach_check: \)false\b/\1true/' "$YAML"
GAP_WORLD=flat_trench_ikdemo.xml GAP_TAG=quadsdk_p4_ikdemo FORWARD_VEL_MPS=0.3 DURATION_S=30 \
  bash scripts/trial/run_quadsdk_gap_1m.sh
sed -i 's/^\(      ik_reach_check: \)true\b/\1false/' "$YAML"
```

## 追加・変更ファイル

- 変更 `src/trial/assets/gen_quadsdk_wide_trench_world.py`(`approach_margin` 引数)
- 新規 `external/quad-sdk/.../worlds/flat_trench_ikdemo.xml.xacro` +
  `models/flat_trench_ikdemo/`
- 変更 `local_planner`:Phase 4 の判定を `is_exact` → **midstance hip からの
  幾何距離 > `ik_max_reach`(0.45)** に。`getNearestValidFootholdResult` の
  第 4 引数を `hip_world` に。`ik_max_reach` パラメータ追加。
- 新規 `artifacts/gifs/quadsdk_phase4_{g30_ik_cross,g100_ik_stop,ik_safestop,ik_fall}*.gif`

## 関連

- `agent_reports/quadsdk_gap_foothold_phase_progress.md` §Phase 4(実装詳細)
- `agent_reports/quadsdk_gap_foothold_summary.md`(まとめ)
- `agent_reports/steps/step_05b_quadsdk_phase2a_safe_stop.md`(30 cm 渡る / 100 cm 停止)
