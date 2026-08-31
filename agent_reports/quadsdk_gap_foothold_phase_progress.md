# Quad-SDK 穴対応 Foot Placement 改善:フェーズ実施ログ

指示書 `chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md` に
沿った、コード解析 → 段階的改善の作業記録。**1 コミット = 1 目的**で進める。

- 解析レポート本体:`agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md`
- 対象:`external/quad-sdk`(Go2、MuJoCo、`reference:=twist`)

## 全体像(フェーズ計画)

| Phase | 目的 | 状態 |
|---|---|---|
| 解析 | 資料 ⇔ コード照合、terrain map / foot placement / NMPC の入出力を数式とコードで整理 | ✅ 完了(`c9cf853`, `3a6c705`) |
| 0 | 解析で判明した資料の 3 誤り(可到達制約 / horizon>period_ / 実センサ NaN)を資料本体で訂正。**コード変更なし** | ✅ 完了(`6e089e1`) |
| 1 | 足場選択器を「位置だけ」→「成功/失敗 + 診断値」を返す型へ。**挙動は不変** | ✅ 完了(`484ea13`) |
| 2 | `found==false` / map 外を下流へ伝播し、名目足場で歩き続けず安全に減速・停止 | ⬜ 未着手(変更計画を提示してから) |
| 3 | 穴縁からの安全距離を地図上で明示判定(PLY 手作業マージンを置換) | ⬜ 未着手 |
| 4 | IK 可到達性で候補を絞る(既存 `QuadKD2::worldToFootIKWorldFrame` を使用) | ⬜ 未着手 |
| 5 | 大きな足場補正時の減速/刻み歩行 | ⬜ 未着手 |
| 6 | 地図の鮮度・未観測セルの扱い | ⬜ 未着手 |

---

## 解析(先行作業)

### `3a6c705` — 解析レポート新規作成(コード変更なし)

`agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` を作成。読んだコード:
`filter_chain.yaml` / `mjcf_to_grid_map_converter.cpp` / `fast_terrain_map.cpp` /
`local_planner.cpp` / `local_footstep_planner.cpp` / `nmpc_controller.cpp` /
`quad_nlp.cpp`(`eval_f` / `eval_g` / `get_bounds_info` / `update_solver`)/
`dynamicsModel.m` / `inverse_dynamics_controller.cpp` / `robot_driver.cpp` /
`planning.py` / `quad_kd2.hpp`。

主な確定事項:

- **`reference:=twist` でも terrain map による足場補正は動く。** Global Body
  Planner だけが `reference:=gbpl` 限定。
- **gait は地形で自動変更されない。** `period`/`duty_cycles`/`phase_offsets` は
  起動時 1 回読んで `nominal_contact_schedule_`(固定表)を作るだけ。
- **穴対応の実効メカニズム**:メッシュに実穴 → 生 `z`=NaN → `z_inpainted` と
  食い違う → `traversability_hole_mask`(`1 − |z_raw − z_inpainted|`)が NaN →
  最終 `traversability` が穴帯で NaN → `getNearestValidFoothold` が却下。
  段差/ランプ/ジグザグでは発火しない。
- **地形表現は 2 系統**:`terrain_grid_`(GridMap、穴=NaN、足場選択用)と
  `terrain_`(FastTerrainMap、`z_inpainted` 由来で穴なし、胴体の高さ/傾き参照用)。
- **Go2 の NMPC は 12 状態 simple model のみ**(`enable_mixed_complexity_` は
  非 spirit で false)。**足場は決定変数ではなくパラメータ**
  (`dynamicsModel.m` の `feet_location`)、GRF のモーメントアームとして EOM に入る。
- **NMPC の制約は EOM(Backward Euler)+ 摩擦ピラミッドのみ。** 関節角・IK
  可到達性の制約は存在しない。`f_z ∈ [10, 150] N`、実効 μ = 0.6
  (`go2.yaml` が `nmpc_controller.yaml` の 0.3 を launch 順で上書き)。
- **有効足場がない場合、現コードは名目足場をそのまま返す**(警告は
  `WARN_THROTTLE` の第1引数バグで実質出ない)。成功/失敗が下流へ伝わらない。
- **下流の安全側フォールバック**:local plan が 0.1 s 以上古い/時刻窓外なら
  `inverse_dynamics_controller` が false を返し、`robot_driver` が
  `stand_joint_angles` へ PD ホールドする。

### `c9cf853` — 解析レポートにレビュー指摘 7 点を反映(コード変更なし)

| # | 訂正 |
|---|---|
| 1 | `foot_positions_body_` は body フレームではない(`foot_world − body_pos`、姿勢回転なし) |
| 2 | Raibert 名目の速度は「並進=world / 角速度=body」(`dynamicsModel.m`)。RobotState 側の並進速度フレームは未確認 |
| 3 | 実効摩擦係数 μ は **0.6**(`go2.yaml` が `nmpc_controller.yaml` の 0.3 を上書き)。要ライブ `ros2 param get` |
| 4 | 「NMPC 失敗後に壊れた GRF がそのままトルク化」は誤り。(a) 失敗継続→stand ホールド と (b) 求解成功だが低品質解→飽和 GRF 実行 を分離(転倒は (b)) |
| 5 | map 外は `getNearestValidFoothold` 手前の `continue` で処理される。`FootholdResult` だけでは `found=false` にできない |
| 6 | 「遠い足場 → NMPC cost 増大 → 非収束」の因果は**推測**へ格下げ(cost 内訳・slack・制約違反・IPOPT status を記録するまで) |
| 7 | `filter_chain.yaml` の `traversability_mask` 閾値 0.5 と `foothold_obj_threshold` 0.6 の不一致。足場選択は `traversability` レイヤを読むため現挙動への直接影響なし |

---

## Phase 0 — 資料の 3 誤りを訂正(`6e089e1`、doc のみ)

`agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` の以下を書き換え:

- **§3.4「NMPC 内の脚可到達制約が破れた」** → 削除。
  「go2 simple NMPC の制約は EOM + 摩擦錐のみで可到達制約は不在」を**コード事実**、
  モーメントアーム→GRF 実現困難→スラック増大の筋を**推測(要ログ)**に分離。
- **§3.4「`horizon_length > period_` が必須」** → 「26→40 で改善は実験事実」と
  「剰余ラップなのでコード上の必須条件ではない」を分離。機序は推測。
- **§2「実センサでも穴は自然に NaN」** → **未確認**へ格下げ
  (この repo に LiDAR/深度 → grid_map `z` の処理はない)。

---

## Phase 1 — `FootholdResult` / `FootholdStatus`(`484ea13`、挙動不変)

### 変更ファイル

| ファイル | 変更 |
|---|---|
| `local_planner/include/local_planner/local_footstep_planner.hpp` | `enum class FootholdStatus` + `struct FootholdResult` + `getNearestValidFootholdResult()` 宣言 + `#include <limits>` |
| `local_planner/src/local_footstep_planner.cpp` | `getNearestValidFoothold()` を薄いラッパ化。探索本体を `getNearestValidFootholdResult()` へ移し、status を設定。DIAG に status/snap を追加。`#include <cmath>, <limits>` |
| `local_planner/test/test_footstep_planner.cpp` | 合成地形ヘルパ 2 個 + テスト 5 本 |

### 型

```cpp
// Phase 1 はこの 4 値だけ。EDGE_TOO_CLOSE / IK_UNREACHABLE / MAP_STALE は
// それを計算するコードと一緒に後の Phase で追加する。
enum class FootholdStatus {
  VALID,
  NOMINAL_OUTSIDE_MAP,
  NO_TRAVERSABLE_CANDIDATE,
  NONFINITE_HEIGHT,
};

struct FootholdResult {
  Eigen::Vector3d position;               // 選ばれた足場(world)。従来の戻り値と同一
  FootholdStatus status;
  double traversability_nominal;          // 名目位置の obj レイヤ値
  double traversability_selected;         // 採用セルの obj レイヤ値
  double snap_distance;                   // ||selected.xy − nominal.xy||
};
```

**`edge_clearance` / `reachable` は入れていない**(ユーザー指示)。

### 挙動が不変であること

- `getNearestValidFoothold()` は `getNearestValidFootholdResult(...).position` を返すだけ。
  唯一の呼び出し(`computeFootPlan:273`)は
  `foot_position = getNearestValidFoothold(...)` のまま。
- `NO_TRAVERSABLE_CANDIDATE`:従来どおり `(nominal.xy, z_inpainted(nominal)+toe)` を返す。
- `NONFINITE_HEIGHT`:従来どおり NaN z を含む位置をそのまま返す(ラベルを付けるだけ)。
- `NOMINAL_OUTSIDE_MAP`:防御的 early-return。現フローでは `computeFootPlan` が
  手前で `continue` するので**到達しない**。Phase 2 で map 外処理を移すときに生きる。
- DIAG(`[MPC_DOG DIAG] gnvf`、`% 40` 間引き)に `status` と `snap` を追記(ログのみ)。

### 検証(`colcon test --packages-select local_planner`)

- 新規 5 本すべて pass:
  - `FootholdResultReportsValidOnFlatTerrain`
  - `FootholdResultReportsNoTraversableCandidate`
  - `FootholdResultReportsNominalOutsideMap`
  - `FootholdResultReportsNonfiniteHeight`
  - `FootholdResultSnapDistanceMatchesSelection`
- **既存の挙動不変テスト 2 本**(`FootholdSearchUsesValidTerrainAndToeRadius`、
  `FootholdSearchFallsBackWhenTerrainInvalid`)も pass → 選択挙動は変わっていない。
- `LocalFootstepPlannerTest` **17/17 pass**。

### 既知の無関係な失敗

`LocalPlannerTest.ConstructorLoadsYamlConfigurationAndInterfaces` が fail。
`test_local_planner.cpp:175` の `EXPECT_EQ(planner.N_, 26)` と
`local_planner.yaml` の `horizon_length: 40`(コミット `9ccd639` 以来)の齟齬。
**Phase 1 とは無関係の先行バグ。** 別コミットで直す(1 コミット 1 目的のため保留)。

---

## 次(Phase 2、未着手)

`found==false` / map 外を下流へ伝播:
- 新しい一歩を確定しない / `cmd_vel` → 0 / 全脚接地可なら STAND / `planner_failed`
  へ理由(map 外 / 未観測 / 広すぎ / map 期限切れ)通知。
- **map 外は `getNearestValidFoothold` に到達しない**ので、`computeFootPlan` の
  `continue` 地点(`local_footstep_planner.cpp:255-261`)にもフックが要る。
- 急停止で転倒しうるので即トルク 0 にはしない。

着手前に変更計画(指示書 14 節の表形式)を提示する。

## 関連

- `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md`(解析本体)
- `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`(twist+クロール成功記録、Phase 0 で訂正)
- `agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md`(gbpl 実験 + 工程別分析)
