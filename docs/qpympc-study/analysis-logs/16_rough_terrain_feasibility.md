# Log 16: 不整地での Foothold・Timing・速度の実現可能性

対応プロンプト: フェーズ13。地形安全集合、運動学可到達、Timing可到達、到達不能時、Planner出力。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

標準: `visual_foothold_adaptation='blind'`, `use_foothold_constraints=False`, `optimize_step_freq=False`, `reflex_trigger_mode=False`, `velocity_modulator=True`, `start_and_stop_activated=False`。

判定の前提: 地形上安全な位置と、脚が届く位置と、残りSwing時間で届く位置は別集合である。現行標準経路はこの交差を保証しない。

---

## 1. 地形安全集合 \(\mathcal S_{terrain}\)

コードに「穴でない・端部余裕がある」という明示集合は無い。部品ごとの事実:

| 判定対象 | 標準 `blind` | `height` | `vfa` | 対応コード | 制限 |
|---|---|---|---|---|---|
| 穴 | 未実装 | 未実装（xyは動かさない） | 外部`virall`に委譲。本リポジトリにソースなし | `VisualFootholdAdaptation.compute_adaptation` | `virall` 未インストールなら print のみ |
| 段差 | 未実装 | zだけ `HeightMap.get_height` で置換 | 同上 + セル選択 | `heightmap.py` `get_height` | 最近傍セルの z+0.02。穴判定なし |
| 傾斜 | 平面近似のみ | 同上 | VFA入力に Euler / 角速度 | `TerrainEstimator` | `roll_activated=False` で roll=0。pitchは4足lift-offの幾何 |
| 法線 | 未実装 | 未実装 | コードから確認不能（`virall`内部） | — | HeightMapは鉛直rayだけ |
| 足を置ける面積 | 未実装 | 未実装 | `convex_data` / `safe_map` を受ける | VFA 81–110 | `safe_map` は print 用。選択は `best_foothold_id` |
| Edge Margin | 未実装 | 未実装 | 頂点2点を制約にする | `footholds_constraints = [vertex1, vertex2]` | 標準は制約オフ。virallのmargin定義は未公開 |
| Heightmap | **作られない** | 7×7、間隔 0.04 m、中心は nominal foothold | 同じグリッド | `simulation.py` 96–115, `HeightMap.update_height_map` | 約 0.24 m 四方。apex時1回 |
| Collision | 未実装 | rayは静的geom。脚との干渉検査なし | なし | `mj_ray` `flg_static=1` | 自己衝突は見ない |
| Foothold cost map | 未実装 | 未実装 | `virall`内部。公開コードにコスト場なし | — | — |

`TerrainEstimator` は \(\mathcal S_{terrain}\) ではない。lift-off 4点から平面の roll/pitch/height を推定し、`ref_orientation` と `ref_z` と速度回転に使う。穴・縁・面積は見ない。`current_contact` で高さ平均を重み付けするコードはコメントアウト。

`height` 戦略は \(\mathcal S_{terrain}\) の近似ですらない。nominal の xy を固定し、その直下の標高に z を合わせるだけである。穴の上なら穴の底の z になる。

MPC foothold 箱制約（`use_foothold_constraints=True` のときだけ）は、参照または VFA 頂点のまわりの箱であり、地形分類器ではない。標準は `False`。

---

## 2. 運動学可到達集合 \(\mathcal R_{kinematic}\)

\[
\mathcal R_{kinematic}
=
\{p\mid q_{\min}\le IK(p)\le q_{\max}\}
\]

この集合を計算して候補を落とす処理は無い。

| 確認項目 | 結果 | 対応コード | 制限 |
|---|---|---|---|
| IK可動範囲検査 | **なし**。`compute_solution` は5回の減衰最小二乗。成否を返さない | `inverse_kinematics_numeric_mujoco.py` | 到達不能でも関節角を返す |
| Joint limit検査 | **なし**。XML `qmin/qmax` は見ない | 同上 | 下流で現在角±3 rad に差をclip。これは可到達保証ではない |
| Hipからの距離Clip | **近似のみ**。FRGは速度先送りを \(\pm hip\_height\cdot 1.5=\pm0.42\) m。誤差項 \(\pm0.05\) m | `compute_footholds_reference` | hip基準の全可到達域ではない。脚リンク長検査なし |
| 厳密なReachability constraint | **未実装** | — | — |
| MPC内部のFoothold bound | 標準オフ。オン時は Heading 箱（stance ±0.1 m、次TD ±0.15 m、またはVFA頂点±0.005） | `create_foothold_constraints`, `set_stage_constraint` | 関節空間ではない。slackあり |
| 脚別可到達範囲 | **なし**。左右は `hip_offset=±0.1` だけ違う | FRG | 脚長差・特異姿勢なし |

VelocityModulator（標準 `activated=True`）は可到達集合ではない。いずれかの足–hip 水平距離が 0.2 m を超えたとき、**既に伸びたあと**で \(v^{ref}\leftarrow0\) にする。候補 \(p_{td}\) を検査しない。指令がほぼゼロなら何もしない。

---

## 3. Timing可到達集合 \(\mathcal R_{timing}\)

\[
\mathcal R_{timing}
=
\{p\mid \|p-p_{lo}\|\le v_{foot,max}T_{swing,remaining}\}
\]

この不等式はコードに無い。

| 確認項目 | 結果 | 対応コード | 制限 |
|---|---|---|---|
| 残りSwing時間 | `swing_time[i]` は進める。残時間 \(T_{sw}-t_{sw}\) は制約に使わない | `STC.update_swing_time` | `swing_time < swing_period` のときだけ加算。超過は止めない判定に使わない |
| 足先速度上限 | **なし**。Splineが距離/時間で決まる速度を出す | `SwingTrajectoryGenerator.createCurve` | 遠いTDほど速い。棄却なし |
| 足先加速度上限 | **なし**。clamped CubicSpline の2階微分をそのまま使う | 同上 | — |
| Joint velocity上限 | IK後の目標差を \(\pm10\) rad/s。標準simは関節PD未使用 | `WBInterface` 450–468 | 実行トルク経路には乗らない |
| Touchdown時刻 | PGG位相 \(\phi<d\) で決まる。Foothold遠近でずらさない | `PeriodicGaitGenerator.run` | — |
| 軌道再生成 | 毎Swing周期 `createCurve(lo, td)`。reflex時は hit から残り時間で作り直す | `compute_trajectory_references` | 遠すぎても新しいSplineを作るだけ |
| MPC Foot velocity constraint | 硬制約なし。`R=[1e-4,1e-4,1e-5]`。`u` の足速度はSwingへ渡さない | `set_weight`, log 12 | Timing可到達の代替にならない |

`swing_period=(1-d)/f` は初期化時（と周波数選択時）に固定される。標準 Trot で約 0.193 s。VFAがTDを動かしてもこの時間は伸びない。

---

## 4. 3集合の統合

現行コードは

\[
p_{td}\in\mathcal S_{terrain}\cap\mathcal R_{kinematic}\cap\mathcal R_{timing}
\]

を**保証しない**。標準 `blind` では \(\mathcal S_{terrain}\) 自体を計算しない。

| 集合 | 実装済み | 近似のみ | 未実装 | 対応コード | 制限 |
|---|---|---|---|---|---|
| \(\mathcal S_{terrain}\) | — | `height` のz置換、`vfa`+HeightMap（非標準） | 標準全体。穴/縁/法線/面積 | TerrainEstimator, VFA, HeightMap | 標準は HeightMap 非生成 |
| \(\mathcal R_{kinematic}\) | — | FRG clip、VM 0.2 m、MPC箱（オフ） | IK/limit検査 | FRG, VM, IK, NMPC | clipは先送りだけ |
| \(\mathcal R_{timing}\) | — | Splineが時間内に幾何補間する | 残時間制約、速度上限 | STC, PGG | 届かない目標も補間する |
| 交差 | — | — | **未実装** | — | 3集合を積集合にする関数はない |

MPC `use_foothold_optimization=True` は胴体Costに合わせて遊脚位置を動かす。地形安全でもTimingでもない。

---

## 5. 到達不能時の現行処理

安全Footholdが残りSwingで届かない場合、コードが行うこと:

| 動作 | 行うか | 条件 |
|---|---|---|
| Footholdを再選択 | しない（標準）。非blindはapexで**1回**だけ適応。可到達検査の再選択はない | `initialized==False` かつ apex |
| 目標速度を下げる | 条件付き別理由のみ。到達不能TDでは下げない | VM: 足が既に hip から 0.2 m。地形 `|pitch|>0.2` で \(v_x/2\) |
| Step frequencyを変更 | しない（標準）。オンでも接触apex条件であり、TD到達不能では発火しない | `optimize_step_freq` |
| Duty factorを変更 | **しない** | — |
| Touchdown時刻を変更 | **しない** | PGG固定 |
| Gait phaseを変更 | **しない**（sim） | `start_and_stop` は ROS2 のみ |
| Gait typeを変更 | **しない**（sim） | 停止時 `FULL_STANCE` のみ、かつフラグオフ |
| 停止 | しない（sim）。VMは脚が伸び切ったあと指令をゼロ | `start_and_stop_activated=False` |
| Solver failure | Foothold到達不能では起きない。infeasibleはGRF/摩擦側 | 前回GRF + reset |
| 到達不能な目標をそのまま使用 | **これが標準動作** | FRG/VFA/MPCのTDをSTCがSplineで追う |

Reflex（標準オフ）: 追従誤差が大きいと `early_stance` になり、軌道を高く作り直す。TD時刻・速度・周波数は変えない。次の数周期のステップ高さを上げるオプションがある。

したがって到達不能時の現行挙動は、「届かない目標へ速いSplineを出し、トルク飽和と実接触に任せる」である。

---

## 6. Planner出力 Interface

\[
\{p_{td,i},\, t_{td,i},\, c_i(t),\, v_{base}^{feasible}\}
\]

| 出力 | 物理的意味 | 現行生成元 | 現行使用先 | 追加が必要か |
|---|---|---|---|---|
| \(p_{td,i}\) | 着地点 | FRG。非blindでVFA。MPCが遊脚を微修正して `nmpc_footholds` | STC `touch_down`、`ref_state['ref_foot_*']` | 地形∩可到達∩残時間の交差で選び直す処理が**追加必要**（現行に無い） |
| \(t_{td,i}\) | 着地時刻 | 明示変数なし。PGGの \(\phi\) が \(d\) に戻る時刻 | `current_contact`、接触列 | **追加必要**。位置変更に合わせて時刻を動かす関数は無い |
| \(c_i(t)\) | 接地予定 | `PeriodicGaitGenerator.compute_contact_sequence` | MPC `p[0:4]`、stance/swing切替 | 地形に応じた再スケジュールは**未実装**。固定周期の列だけ |
| \(v_{base}^{feasible}\) | 実現可能な胴体速度 | ユーザー指令。VMと急傾斜で後から縮小 | FRG先送り、MPC速度参照 | **追加必要**。安全TDから逆算した速度上限は無い |

現行と推奨の分離:

- **現行**: 速度とGait timingを先に決め、位置を後から付ける。位置が timing/速度と矛盾しても上流へ戻さない。
- **推奨（未実装）**: 資料 `13` §4 の順（位置→速度→周波数→Duty→時刻→Gait→停止）。これは改善案であり、コード経路ではない。

Planner相当のモジュールは無い。`TerrainEstimator`、VFA、PGG、FRG、VM、batched周波数はそれぞれ局所処理で、4出力を同時に整合させない。

---

## 7. シナリオ追跡

```text
Nominal footholdが穴に入る
→ VFAが安全位置へ移動
→ 安全位置が現在の残りSwing時間では遠すぎる
```

### 標準 `blind`（ディスク設定）

このシナリオの「VFAが安全位置へ移動」は**起きない**。

| 段階 | 関数 | 変数 | 実際の動き |
|---|---|---|---|
| 1 | `FRG.compute_footholds_reference` | `ref_feet_pos` | hip + \(\frac{T_{st}}{2}v^{ref}\) + 誤差。z=lift-off z。穴を見ない |
| 2 | VFA分岐 | `heightmaps=None` | `visual_foothold_adaptation=='blind'` でスキップ。`ref_feet_constraints=None` |
| 3 | `Acados_NMPC_Nominal.compute_control` | `ref_state['ref_foot_*']` | 穴上の参照を追う。`use_foothold_constraints=False` |
| 4 | foothold抽出 | `nmpc_footholds` | 遊脚は予測xまたは参照。±0.15 clip（制約オフ時の出力clip） |
| 5 | `STC.compute_swing_control_*` | `lift_off`, `touch_down=nmpc_footholds`, `swing_time` | 全 `swing_period` でSpline。残時間検査なし |
| 6 | `env.step` | `mjData.contact` | 実接地はMuJoCo。指令側は計画接触のまま |

穴に落ちるか縁に引っかかるかはPlant側。コントローラは目標を捨てない。

### 非標準 `vfa`（フラグを立て、`virall` がある場合）

| 段階 | 関数 | 変数 | 実際の動き |
|---|---|---|---|
| 1 | 同上 FRG | `ref_feet_pos` | 穴の上のnominal |
| 2 | `STC.check_apex_condition(..., interval=0.01)` | `swing_time ≈ T_sw/2` | apexかつ `vfa.initialized==False` のときだけ適応 |
| 3 | `HeightMap.update_height_map(ref_feet_pos, yaw)` | `heightmaps[leg].data` (7×7×1×3) | nominalまわり 0.24 m の鉛直ray |
| 4 | `VFA.compute_adaptation` | `convex_data`, `best_foothold_id`, `safe_map` | xyをセルへ移動、zを `get_height`。制約に凸包2頂点 |
| 5 | `get_footholds_adapted` | `footholds_adaptation` | 以後full stanceまでその位置を保持 |
| 6 | 主MPC | 同じ `ref_state` | 新しい位置を参照。残時間は見ない |
| 7 | STC Spline | `\|p_{td}^{safe}-p_{lo}\|` が大きい | 同じ \(T_{sw}\) で補間 → 足先速度・加速度が増えるだけ |
| 8 | 到達不能の後処理 | なし | 周波数・Duty・時刻・速度（VM以外）は変わらない |
| 9 | 任意 reflex | `EarlyStanceDetector` | 追従誤差で高さを変える。TDは変えない |

`height` だけの場合、段階4は xy を動かさない。穴なら z が下がる。

フルスタンスで `vfa.reset()` され、次Swingのapexで再適応する。その間に遠すぎると分かっても再計画しない。

---

## 8. 資料照合

### `05_Foothold_Reference_and_Terrain_Adaptation.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| TerrainEstimatorが先頭、入力はlift-off | 一致 | 正しい | `wb_interface` |
| Nominal式とclip | 一致 | 正しい | FRG |
| Footholdは地形回転前速度 | 一致 | 正しい | 回転はFRGの後 |
| z=lift-off z | 一致 | 正しい | 標準 |
| blind既定、HeightMap非生成 | 一致 | 正しい | `simulation.py` |
| 3集合は理論上必要でVFAだけでは保証しない | 一致 | 正しい | 本ログで再確認 |
| 「MPCがさらに最適化」 | 位置Cost。地形安全ではない | 不完全 | 読者が \(\mathcal S\) 保証と読む余地 |
| Cursor課題3（clipがIKを保証するか） | 課題のまま | 正しい（課題） | 保証しない。本ログ §2 |

### `13_Feasibility_on_Rough_Terrain.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §1–2 3集合が必要 | 理論 | 正しい | 実装保証ではない |
| §3 安全位置が遠すぎる例 | 起こり得る | 正しい | 標準ではVFA自体が動かない点は未記 |
| §4 対応の順序 | 推奨 | 正しい（推奨） | 現行にこの連鎖は無い |
| §5 Planner 4出力 | 必要Interface | 正しい（推奨） | \(t_{td}\) と \(v^{feasible}\) は未生成 |
| §6 同時最適化しない | 範囲記述 | 正しい | 標準は固定timing + 幾何Foothold |
| §8 VFA制約にIKと残時間があるか | 確認課題 | 正しい（課題） | **どちらも無い** |

### `appendices/F_Open_Questions.md`

| 項目 | 判定 | 本ログでの確定 |
|---|---|---|
| VFAと残りSwing時間の厳密な整合 | 未実装と確定してよい | 残時間制約なし。整合検査なし |
| 安全Footholdがない場合の減速・停止 | 未実装と確定してよい（sim） | VMは脚伸び後。`start_and_stop` はsimオフ |
| Contact timingを変える上位Planner | 未実装と確定してよい | PGG固定。Plannerモジュールなし |
| Frequency候補の実機範囲 | 未確認のまま | 本フェーズ対象外 |
| 速度ごとのGait切替Envelope | 未実装のまま | 自動切替なし |

Fの「未確定」のうち、上3つは公開コードだけで「無い」と閉じられる。実機で別バイナリがあるかはコードから確認不能。

---

## 9. 事実 / 解釈 / 未確認

**事実**

- 標準経路に地形安全集合、IK可到達検査、残Swing時間制約は無い。
- 到達不能でも目標TDを捨てず、Splineで追う。
- VFAは非標準。`height` はzのみ。`vfa` は非公開 `virall`。
- HeightMapは 7×7・0.04 m の鉛直ray。標準では生成しない。
- VMと急傾斜の速度縮小は、安全TDの到達不能判定ではない。

**解釈**

- 大きな穴・飛び石では、資料 `13` が言う上位再計画が必要。現行スタックはその層を持たない。

**未確認**

- `virall` 内部の穴・縁・面積・法線の定義。
- `vfa` を実際にONにしたときのセル選択の数値例（本環境にパッケージなし）。
- 実機ROS2で `start_and_stop` を使った停止が、安全Foothold欠如に連動するか。
