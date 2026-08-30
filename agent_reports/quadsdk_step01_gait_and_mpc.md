# Quad-SDK Step 01 の GAIT(歩容)と MPC の関係 — 理論式・コード

作成: 2026-08-30。`external/quad-sdk` の C++ ソース・YAML を実際に読んで
確認した内容。**【事実】=コードで確認済み**、**【推測】=未確認・仮説** として
節を分ける。関連: `agent_reports/quadsdk_step01_mpc.md`(MPC 本体)、
`agent_reports/quadsdk_step01_control_pipeline.md`(6節)、
`agent_reports/quadsdk_step01_wbc.md`。

---

## 背景

`agent_reports/quadsdk_step01_mpc.md` は NMPC 単体(コスト・制約・IPOPT)を
説明したが、その入力である **「接触スケジュール(どの脚がいつ地面につくか)」
「足先の着地位置(どこにつくか)」がどこで決まり、どう MPC に入るのか** は
「local_footstep_planner が決める」以上に踏み込んでいなかった。

歩容と MPC は「別々に動いて結果だけ受け渡す」のか「相互に最適化する」のかで
デバッグの見方が変わる。Step 01 の「WALK 移行直後の姿勢跳躍」(handoff 4節)や
速度を上げたときの不安定化を切り分けるには、この境界を明文化する必要がある。

## 目的

- 歩容生成(接触スケジュール + 着地位置)の**理論式**とコード対応を示す
- 歩容の出力が **どの変数として MPC に入り、MPC のどの項を変えるか** を
  1対1で対応づける
- 歩容 ↔ MPC が一方向か双方向か、フィードバックの経路を明確にする

---

## 概要(結論を先に)

- **役割分担は明確に分かれている**:
  - **歩容(`local_footstep_planner`)** = 「**いつ・どの脚**が接地するか」
    (接触スケジュール)と「**どこ**に足を置くか」(着地位置)を決める
  - **MPC(`nmpc_controller`)** = 接触スケジュールと着地位置を**固定の入力**として
    受け取り、「各接地脚が**どれだけの力**を出すか(GRF)」と
    「その結果の胴体軌道」を最適化する
- **MPC は接地タイミングも着地位置も変えない**。歩容 → MPC の**一方向**。
- 歩容 → MPC の結合は **2チャンネル**:
  1. **接触スケジュール**(N×4 の bool)→ MPC の `contact_sequence_` →
     ノミナル入力 `u_nom`・遊脚 GRF の強制ゼロ・摩擦制約の有効脚を決める
  2. **足先位置**(body 相対)→ MPC の `foot_pos_body_` →
     運動方程式の `Σ J_i(θ, r_i)^T f_i` の**モーメントアーム `r_i`** を決める
- フィードバックはサイクルをまたいで間接的にのみ存在:
  MPC が出した胴体軌道 `body_plan_` を、次サイクルの歩容が
  Raibert 則の基準(hip 軌道)として使う。
- go2 の歩容は **トロット**(対角の 2 脚ずつ)。
  `period = 0.36 s`、`duty_cycles = [0.5]×4`、`phase_offsets = [0, 0.5, 0.5, 0]`。

---

## 流れ(1 計画サイクル、`local_planner.cpp: computeLocalPlan()`)

順序が結合を理解する鍵。`body_plan_` は**前サイクルの MPC 出力**を保持している。

1. **接触スケジュール生成**
   `local_footstep_planner_->computeContactSchedule(current_plan_index_,
   body_plan_, ref_primitive_plan_, control_mode_, contact_schedule_)`
   → `contact_schedule_`(N×4 bool)
2. **着地位置生成**
   `local_footstep_planner_->computeFootPlan(current_plan_index_,
   contact_schedule_, body_plan_, grf_plan_, ref_body_plan_, ...,
   foot_positions_world_, foot_velocities_world_, foot_accelerations_world_)`
   → Raibert 則で新しい着地位置(前サイクルの `body_plan_` / `grf_plan_` を使う)
3. **body 座標への変換**
   `getFootPositionsBodyFrame(body_plan_, foot_positions_world_,
   foot_positions_body_)`、toe 半径ぶんの補正
4. **MPC 求解**
   `local_body_planner_nonlinear_->computeLegPlan(current_full_state,
   ref_body_plan_, grf_positions_body, grf_positions_world,
   foot_velocities_world_, contact_schedule_, ref_ground_height_, ...,
   body_plan_, grf_plan_)`
   → `body_plan_`(N×12)と `grf_plan_` を**上書き**
5. 次サイクルへ。手順 1・2 が再び上書き後の `body_plan_` を参照する

---

## 詳細: 歩容の理論(1)接触スケジュール

出所: `local_planner/src/local_footstep_planner.cpp: setTemporalParams()`,
`computeContactSchedule()`。

### ノミナル歩容テーブルの構築

`setTemporalParams(dt, period, horizon_length, duty_cycles, phase_offsets)`:

- `period_` = `period[s] / dt_` = `0.36 / 0.03` = **12 位相**(離散)
- 各位相 `i ∈ [0, period_)`、各脚 `leg` について接地 bool:

```
contact(i, leg) = true  if  (period_·φ_leg ≤ i < period_·(φ_leg + d_leg))
                          or (i < period_·(φ_leg + d_leg − 1))      ← 位相の巻き戻し
                = false otherwise
```

- `d_leg` = `duty_cycles[leg]`(接地時間比。go2 は全脚 0.5)
- `φ_leg` = `phase_offsets[leg]`(歩容内の開始位相。go2 は `[0, 0.5, 0.5, 0]`)
- 脚順は quad-sdk 規約 `FL(0), BL(1), FR(2), BR(3)`
  → `φ = [0, 0.5, 0.5, 0]` は **FL と BR が同位相**、**BL と FR が半周期ずれ**
  = **対角トロット**

### ホライズンへの展開

`computeContactSchedule(current_plan_index, body_plan, ref_primitive_plan,
control_mode, contact_schedule)`:

- `phase = current_plan_index % period_`(今どの位相か)
- `control_mode == STAND` → 全ホライズン・全脚 `true`(4脚接地で立つ)
- それ以外(STEP)→
  `contact_schedule[i] = nominal_contact_schedule_[(i + phase) % period_]`
  (ノミナル歩容を現在位相から N 点ぶんタイル状に貼る)
- `LEAP_STANCE` / `FLIGHT` / `LAND_STANCE` プリミティブは上書き
  (跳躍・遊泳・着地。**Step 01 の平地トロットでは使われない**)

> 出力 `contact_schedule`(N×4 bool)は `body_plan_` を参照するが、
> 実際に見ているのは `current_plan_index`(位相)だけで、
> `body_plan` の中身(前 MPC の胴体軌道)は STEP 分岐では未使用。
> つまり接地タイミングは**歩容パラメータと時刻だけ**で決まる決定論的なもの。

---

## 詳細: 歩容の理論(2)着地位置(Raibert 則)

出所: `local_footstep_planner.cpp: computeFootPlan()`(`:160`〜)。

各脚 `j`、接地開始(touchdown)イベントの位相 `i` について:

### hip 中点

- スタンス窓 `[i, end_of_stance)` の各時刻で「脚 `j` のノミナル hip 位置」
  (`worldToNominalHipFKWorldFrame`)を集め、その**最小包含円**の中心
  (Welzl アルゴリズム `welzlMinimumCircle`)を `hip_position_midstance` とする
  → 「スタンス中、足を hip 直下付近に置く」ための基準点

### 動的オフセット(遠心力補償 + キャプチャポイント)

```
h        = max(body_z − terrain_z(body_xy), 0)          胴体高さ
centrifugal  = (h / g) · (v_td × ω_ref,td)              遠心力補償
vel_tracking = sqrt(h / g) · (v_td − v_ref,td)          速度追従(キャプチャポイント)
foot_raibert = hip_position_midstance + centrifugal + vel_tracking
```

- `v_td`, `ω_ref,td` … touchdown 時刻での胴体速度・参照角速度
  (`body_plan`(前 MPC)と `ref_body_plan` から)
- `sqrt(h/g)·(v − v_d)` は Raibert / 倒立振子のキャプチャポイント項
- `g = 9.81`

### 地形スナップ

- `foot_raibert` の (x,y) を地形マップの `z_inpainted` に投影 + `toe_radius`
  ぶん持ち上げ(`agent_reports/quadsdk_step01_terrain_map.md`)
- `getNearestValidFoothold()` で `traversability` を満たす最寄り点に補正
  (平地では実質そのまま)
- 非 touchdown サンプルは直前の着地位置を保持

### 遊脚軌道

- liftoff の足位置 → touchdown の足位置を **3次エルミートスプライン**
  (`cubicHermiteSpline`)で補間
- 中間点で頂点高さ `swing_apex`(`computeSwingApex`、`ground_clearance = 0.07 m`
  ベース)を与える
- これが `foot_positions_world_` / `_velocities_` / `_accelerations_` として
  次段(MPC のパラメータ、WBC の遊脚追従)に渡る

---

## 詳細: 歩容の出力が MPC のどの項に入るか

出所: `nmpc_controller/src/nmpc_controller.cpp: computeLegPlan()`、
`quad_nlp.cpp: update_solver() / eval_f() / eval_g() / get_bounds_info()`。

### チャンネル 1: 接触スケジュール → `contact_sequence_`

`computeLegPlan(..., contact_schedule, ...)` →
`quadNLP::update_solver()` が `contact_schedule`(N×4 bool)を
`contact_sequence_`(`Eigen::MatrixXi`、4×(N-1))に格納。以下3か所で使う:

1. **ノミナル入力 `u_nom`**(`eval_f()` / `eval_grad_f()`):
   ```
   num_contacts = contact_sequence_.col(i).sum()          その時刻の接地脚数
   u_nom[3j+2]  = mass · g / num_contacts   (接地脚 j のみ、鉛直成分)
   ```
   → コストは「体重を接地脚で等分した鉛直 GRF」からの差分に対してかかる。
   接地脚数が変われば各脚の基準荷重が変わる。
2. **遊脚 GRF の強制ゼロ**(`get_bounds_info()`):
   ```
   u_lb[3j:3j+3] ← u_lb[3j:3j+3] · contact_sequence_(j, i)
   u_ub[3j:3j+3] ← u_ub[3j:3j+3] · contact_sequence_(j, i)
   ```
   → 遊脚(0)は GRF の上下限が 0 に潰れ、力を出せない。
3. **摩擦ピラミッド制約**(`eval_g()` の生成コード):
   遊脚は GRF=0 なので摩擦制約 `|f_x| ≤ μ f_z` 等は自明に満たされる。
   実質、摩擦制約は接地脚だけで有効。

### チャンネル 2: 着地位置 → `foot_pos_body_`(運動方程式のモーメントアーム)

`computeLegPlan()`:
```
mynlp_->foot_pos_body_  = -foot_positions_body;   (body 相対の足先位置)
mynlp_->foot_pos_world_ = foot_positions_world;
mynlp_->foot_vel_world_ = foot_velocities_world;
```

`eval_g()` が各 finite element のパラメータ `pk` に詰める:
```
pk[0]              = dt (i==0 は first_element_duration)
pk[1]              = mu
pk.segment(2, 12)  = foot_pos_body_.row(i+1)     ← 各脚の接触点位置 r_i
pk.segment(14, 12) = foot_pos_world_.row(i+1)
pk.segment(26, 12) = foot_vel_world_.row(i+1)
```

`dynamicsModel.m` の運動方程式

```
M(θ) v̇ + h(θ, v) = Σ_i J_i(θ, r_i)^T f_i
```

で、`J_i` は body から接触点までのベクトル `r_i`(= `feet_location`)に依存する。
つまり **足をどこに置くか(Raibert 則の出力)が、GRF が胴体に及ぼす
モーメントの腕を決め、MPC の等式制約(`eom_state_*`)に直接入る**。

### go2(simple モデル)で入らないもの

- `nmpc_controller.cpp`: `components_in_cost = {body:true, feet:false, joints:false}`、
  かつ simple モデルのコスト次元は body のみ。
  → **足先位置・速度は MPC の「コスト(追従目標)」には入らない**。
  あくまで運動方程式のパラメータとして入るだけ。
  (spirit の complex / adaptive complexity モデルでは足先もコストに入るが
  go2 では無効。)

---

## 詳細: フィードバックの経路(双方向か)

- **1サイクル内**: 歩容 → MPC の**一方向**。
  MPC は接地タイミングも着地位置も変更しない。
- **サイクルをまたいで**: `body_plan_`(MPC が上書きする胴体軌道)が
  次サイクルの
  - `computeFootPlan()` の Raibert 基準(hip 軌道・`v_td`)
  - `getFootPositionsBodyFrame()` の body 位置
  に使われる。→ MPC の結果が次の歩容の**着地位置**に間接的に効く。
- 接地**タイミング**へのフィードバックは無い(位相 = `current_plan_index %
  period_` は時刻だけの関数)。速度が変わっても足の運び頻度は
  `period = 0.36 s` 固定。

---

## 初期パラメータ一覧(go2、Step 01 時点)

`go2.yaml`(`local_planner.local_footstep_planner`):

- `period = 0.36` s → `period_ = 12` 位相(`/ dt 0.03`)
- `duty_cycles = [0.5, 0.5, 0.5, 0.5]`(全脚スタンス 50%)
- `phase_offsets = [0.0, 0.5, 0.5, 0.0]`(FL/BR 同位相、BL/FR 半周期ずれ = トロット)
- `ground_clearance = 0.07` m(遊脚頂点高さの基準)
- `hip_clearance = 0.1` m
- `foothold_search_radius = 0.25` m
- `grf_weight = 0.45`(GRF ベース着地点と地形ベース着地点のブレンド重み。
  local_planner.yaml)
- `foothold_obj_threshold = 0.6`、`obj_fun_layer = traversability`
  (着地点探索の地形スコア閾値。local_planner.yaml)

`local_planner.yaml`(`local_planner`):

- `timestep = 0.03` s、`horizon_length = 26`(歩容も MPC と同じ N・dt を共有)
- `stand_cmd_vel_threshold = 0.05`(**元 0.1**。これ未満の指令だと STAND のまま。
  handoff 4節)
- `stand_vel_threshold = 0.1`、`stand_pos_error_threshold = 0.05`

歩容モード切替(`local_planner.cpp:331`):
```
STEP に入る条件 = (cmd_vel.norm() > stand_cmd_vel_threshold)
              or (現在の水平速度 > stand_vel_threshold)
              or (支持中心からのズレ > stand_pos_error_threshold)
```

---

## 【推測】未確認事項

- **`period = 0.36 s` 固定がどの速度域まで妥当か**: 速度が上がると
  1歩の距離(`≈ v · period · duty`)が伸びるだけで足の運び**頻度**は
  変わらない。0.5〜1.1 m/s の不安定化(handoff 4節)に歩容周期の固定が
  効いている可能性はあるが未検証。歩容パラメータ調整は Step 01 の制約により
  未実施。
- **`computeContactSchedule` が `body_plan` を引数に取るが STEP 分岐で
  使っていない**ように読める。LEAP/FLIGHT 系プリミティブ経路では
  使う可能性があるが、平地トロットでは無関係。完全な確認はしていない。
- **`grf_midstance`(GRF を使った着地点)経路はコメントアウトされている**
  (`local_footstep_planner.cpp:270` 付近)。現状の着地点は純粋に
  hip 中点 + Raibert オフセット + 地形スナップ。`grf_weight = 0.45` が
  どこで効いているかは要再確認。
- **`computeSwingApex` の詳細**(`ground_clearance` からの頂点高さ算出式)は
  未精査。

---

## その後(歩容と MPC の切り分けに使える視点)

- **「姿勢跳躍」の切り分け**: WALK 移行の瞬間は
  `control_mode` が STAND(全脚接地)→ STEP(トロット)に切り替わり、
  `contact_schedule` が一斉に変わる。同時に `u_nom`(接地脚数 4 → 2)、
  遊脚 GRF 境界(自由 → 0)、モーメントアーム基準が不連続に変わる。
  跳躍がこの切替フレームに集中しているかを `plan_nmpc_cost` /
  `plan_nmpc_iterations` の時系列で確認できる。
- **速度を上げる Step で最初に見るべき歩容パラメータ**:
  `period`(足の運び頻度)、`duty_cycles`(接地時間比 = 支持の余裕)、
  `ground_clearance`(つまずき耐性)。MPC 側の `Q`/`R`/`N` と対で調整対象。
- **ロガーとの対応**: `contact_*` 列は WBC が出す接地判断(`local_plan` の
  接地スケジュール由来)であり、ここで説明した `contact_schedule` が
  MPC → `local_plan` → WBC → ロガー と流れた末端。

---

## ソース早見表(`external/quad-sdk/`)

- 歩容生成(接触スケジュール + 着地位置)
  - `local_planner/src/local_footstep_planner.cpp`
    - `setTemporalParams`(`:8`。ノミナル歩容テーブル)
    - `computeContactSchedule`(`:84`。ホライズン展開)
    - `computeFootPlan`(`:160`。Raibert 則の着地点)
    - `cubicHermiteSpline`(`:121`。遊脚軌道)
  - `local_planner/include/local_planner/local_planner_modes.hpp`
    (`enum LocalPlannerMode { STAND, STEP }`)
- 歩容 → MPC の受け渡し
  - `local_planner/src/local_planner.cpp: computeLocalPlan()`(`:487`。
    contactSchedule → footPlan → computeLegPlan の順序)
- MPC 側での消費
  - `nmpc_controller/src/nmpc_controller.cpp: computeLegPlan()`
    (`foot_pos_body_` などの受け渡し)
  - `nmpc_controller/src/quad_nlp.cpp`
    - `update_solver`(`contact_sequence_` 格納)
    - `eval_f`(`:478`。`num_contacts` → `u_nom`)
    - `get_bounds_info`(`:232`。遊脚 GRF 境界を 0 に)
    - `eval_g`(`:678`。`pk.segment(2,12) = foot_pos_body_`)
- パラメータ
  - `quad_utils/config/go2.yaml`(`local_footstep_planner` の period/duty/phase)
  - `local_planner/config/local_planner.yaml`(timestep/horizon、閾値、grf_weight)
- ダイナミクスの `feet_location` 依存
  - `nmpc_controller/scripts/dynamicsModel.m`
    (`generateLegDynamics`。`J_feet` が `feet_location` に依存)
