# Quad-SDK Step 01 の MPC(NMPC)の理論・コスト・制約・最適化とパラメータ

作成: 2026-08-30。`external/quad-sdk` の C++ ソース・CasADi 生成コード・
MATLAB 記号スクリプト・YAML を実際に読んで確認した内容。
**【事実】=コードで確認済み**、**【推測】=未確認・仮説** として節を分ける。
関連: `agent_reports/quadsdk_step01_control_pipeline.md`(6〜7節)、
`agent_reports/quadsdk_step01_wbc.md`。

---

## 背景

`agent_reports/quadsdk_step01_control_pipeline.md` の7節は「NMPC は独立ノードでなく
`local_planner` にリンクされるライブラリ」「IPOPT + MUMPS」という要点だけで、
**何を最適化しているのか(モデル・コスト・制約)** には踏み込んでいなかった。

Step 01 の調査で NMPC 関連に加えられた変更は3つある(handoff 5節、go2.yaml):

1. `linear_solver` を `"ma27"` → `"mumps"`(HSL 未導入のため)
2. `print_level` を `0` → `5`(診断用、そのまま残存)
3. go2.yaml 側で yaw 境界の拡大(PR #400 相当)、関節速度境界を
   `motor_limits.speed` に合わせて狭める、`friction_coefficient` 0.6

「MUMPS の数値精度不足が転倒の原因」という仮説を検証するには、NMPC が実際に
どういう問題を解いているのかを押さえる必要があった。本ドキュメントはその整理。
結論として、転倒原因は NMPC ではなかった(handoff 8節)。

## 目的

- NMPC の**最適化問題**(決定変数・目的関数・制約・ダイナミクスモデル)を
  理論式とコード対応で明文化する
- go2 で実際に使われる**数値パラメータ**(ホライズン・重み・境界・ソルバ設定)を
  1か所にまとめる
- どこが調査で変更済みで、どこが未検証かを分ける

---

## 概要(何を・どう解いているか)

- **問題種別**: 直接コロケーション(backward Euler)による軌道最適化 (NLP)。
  CasADi で目的関数・制約・ヤコビアン・ヘシアンを記号生成 → **IPOPT** で解く。
  `nmpc_controller` は独立ノードではなく `local_planner_node` にリンクされる
  **ライブラリ**。
- **ダイナミクスモデル(go2 = "simple")**: 単剛体(ラグランジュ形式)。
  - 状態 `x ∈ R^12` = `[p(world 位置 3), θ(ZYX オイラー角 3), ṗ(world 並進速度 3),
    ω(body 角速度 3)]`
  - 入力 `u ∈ R^12` = 4脚の接地力 `[f_0; f_1; f_2; f_3]`(各 world 系 R^3)
- **ホライズン**: `N = 26` 点、`dt = 0.03 s`(`local_planner.yaml`)。
  約 0.75 s 先まで予測。
- **目的関数**: 参照追従の二次形式 + slack ペナルティ(下記「詳細: コスト」)。
- **制約**: (a) 運動方程式(backward Euler、等式)(b) 摩擦ピラミッド(不等式)
  (c) 遊脚の接地力 = 0(境界を潰す)(d) 接地力の鉛直成分 10〜150 N
  (e) 状態のソフト境界(slack + panic weight)。
- **出力**: 最適化された `state_traj`(N×12、body 状態の時系列)と
  `control_traj`(N-1×12、GRF 系列)。`local_planner` がこれを `RobotPlan`
  (`local_plan`)に詰めて publish。
- **go2 固有の重要点**: `friction_coefficient = 0.6`(go2.yaml が
  nmpc_controller.yaml の 0.3 を上書き)、yaw 境界 ±10 rad、
  adaptive/mixed complexity は **spirit 専用で go2 では無効**。

---

## 流れ(順を追って)

1. `local_planner_node` が `update_rate = 333 Hz`(local_planner.yaml)で計画ループ。
   `cmd_vel`(`use_twist_input=true`)を積分して body 参照軌道
   `ref_traj`(N×12)を作る。
2. `local_footstep_planner` が接触スケジュール `contact_schedule`(N×4 bool)、
   足先位置・速度、地面高さ `ref_ground_height` を決める。
3. `NMPCController::computeLegPlan(initial_state, ref_traj, foot_positions_body,
   foot_positions_world, foot_velocities, contact_schedule, ref_ground_height,
   first_element_duration, plan_index_diff, terrain, state_traj, control_traj)`
   を呼ぶ(`nmpc_controller.cpp`)。
4. `quadNLP::update_solver(...)` が参照・接触・地形・warm start 情報を
   NLP に差し替える。
5. `app_->OptimizeTNLP(mynlp_)` で IPOPT 実行(`computePlan`)。
   - `mu_init` を `mynlp_->mu0_` に、`warm_start_init_point` を
     前回成否に応じて `"yes"/"no"` に設定
6. 成功時: `mynlp_->warm_start_ = true`。`w0_`(primal 解)から
   `state_traj`(N×12)と `control_traj`(N-1×12)を取り出す。
7. 失敗時: `mu0_ = 1e-1`、`warm_start_ = false`、`require_init_ = true`、
   `"NMPC solving fail"` を WARN。この tick は `local_plan` を publish しない
   (`quadsdk_step01_baseline.py` の `plan_age_s` が伸びる)。

---

## 詳細: ダイナミクスモデル(理論式)

出所: `nmpc_controller/scripts/dynamicsModel.m`(CasADi 記号生成スクリプト)。

### 状態と一般化座標

- `q = [p; θ]` … world 位置 `p` と ZYX 系オイラー角 `θ`
- `v = [ṗ; ω]` … world 並進速度 `ṗ` と **body 系**角速度 `ω`
- 状態 `x = [q; v] ∈ R^12`
- オイラー角速度は `θ̇ = J_wb__b(θ)^{-1} ω` で `ω` と関係づく
  (`J_wb__b` は姿勢ヤコビアン)

### 運動方程式(ラグランジュ形式)

ラグランジアン `L = T - V`:

```
T = (1/2) V_wb__b^T M_b V_wb__b        (運動エネルギー)
V = m g p_z                            (位置エネルギー)
M_b = blkdiag(m·I_3, I_body)          (body 質量 + 慣性テンソル)
```

から導かれる運動方程式(記号 `M`, `h` はスクリプトが自動生成):

```
M(θ) v̇ + h(θ, v) = Σ_{i=0}^{3} J_i(θ, r_i)^T f_i
```

- `M(θ)` … 一般化慣性行列(12×12 のうち有効部は 6×6)
- `h(θ, v)` … コリオリ項 + 重力項
- `J_i` … 脚 `i` の接触点(body から `r_i` の位置)での力ヤコビアン。
  GRF `f_i` を一般化力へ写す
- `f_i ∈ R^3` … 脚 `i` の接地力(world 系)

**関節レベルの全身動力学は解かない**。脚は「body に力を及ぼす接触点」としてのみ
モデル化される(足質量 `foot_mass_ = 0.01` は簡易考慮)。関節トルクは次段の
WBC(`agent_reports/quadsdk_step01_wbc.md`)が担当。

### backward Euler コロケーション(有限要素 k → k+1)

```
(q_{k+1} - q_k) - dt · q̇(x_{k+1})                       = 0
M(θ_{k+1})(v_{k+1} - v_k) + dt · (h_{k+1} - Σ J_i f_i)   = 0
```

- `x_{k+1}` 側で評価する完全陰的(backward Euler)。
- これが各 finite element の **等式制約 12 本**(`eom_state_0..11`)。
- `dt` は `i == 0` のみ `first_element_duration_`(次の計画インデックスまでの
  実時間)、以降は `dt_ = 0.03`。

### 摩擦ピラミッド(不等式制約)

各脚について(`tmp` を 4 脚に `kron`):

```
[ 1  0 -μ ]           [ f_x - μ f_z ]
[-1  0 -μ ] · f_i  =  [-f_x - μ f_z ]  ≤  0
[ 0  1 -μ ]           [ f_y - μ f_z ]
[ 0 -1 -μ ]           [-f_y - μ f_z ]
```

→ `|f_x| ≤ μ f_z` かつ `|f_y| ≤ μ f_z`(円錐ではなく**ピラミッド近似**)。
go2 は `μ = 0.6`。1 finite element あたり **4脚 × 4 = 16 本の不等式**
(`friction_x_pos_foot_i` … `friction_y_neg_foot_i`)。

### 制約ベクトル `g` の構成(go2 = simple モデル)

`nmpc_controller/src/quad_nlp_utils.cpp: loadConstraintNames()` と
`eval_g_go2.h`(`SZ_W 52`)より、simple モデルの 1 finite element の
`g_dim = 28`:

- `eom_state_0..11` … 12 本(運動方程式、等式 `g = 0`)
- `friction_{x,y}_{pos,neg}_foot_0..3` … 16 本(摩擦、`-inf ≤ g ≤ 0`)

(complex モデルの `foot_height` / `knee_height` / `fk_pos` / `fk_vel` /
`motor_model` 制約は spirit の adaptive complexity 用で、go2 では使われない。)

---

## 詳細: コスト(目的関数)

出所: `nmpc_controller/src/quad_nlp.cpp: eval_f()`(勾配は `eval_grad_f()`)。

```
obj = Σ_{i=0}^{N-2} [ (1/2) x̃_i^T Q_i x̃_i + (1/2) ũ_i^T R_i ũ_i ]
      + panic_weights · Σ (状態 slack 変数)
      + constraint_panic_weights · Σ (制約 slack 変数)
```

- `x̃_i = x_{i+1} - x_nom_i`
  - `x_nom_i` = 参照 body 状態 `x_reference_.col(i+1)`(local_planner が
    cmd_vel から作った参照軌道)
- `ũ_i = u_i - u_nom_i`
  - `u_nom_i[3j+2] = mass · g / (接地脚数)` を接地脚 `j` に設定、他は 0。
    つまり「体重を接地脚に等分した鉛直 GRF」をノミナル入力とする
- `Q_i = Q · Q_temporal_factor^i`、`R_i = R · R_temporal_factor^i`
  - `nmpc_controller.cpp` で
    `Q_temporal_factor = pow(100.0, 1/(N-2))`、
    `R_temporal_factor = pow(1.0, 1/(N-2)) = 1`
  - i が進むほど状態追従の重みが指数的に増える(ホライズン終端を重視)
- `i == 0` のみ `Q_0, R_0` を `first_element_duration_ / dt_` 倍(時間長で正規化)

### go2 の重み(go2.yaml `nmpc_controller.body`)

- `Q`(`x_weights`)= `[5, 5, 5,  0.5, 0.5, 0.5,  0.1, 0.1, 0.2,  0.05, 0.05, 0.01]`
  - 位置 `p_x,p_y,p_z` = 5
  - 姿勢 `roll,pitch,yaw` = 0.5
  - 並進速度 `v_x,v_y,v_z` = 0.1, 0.1, 0.2
  - 角速度 `ω_x,ω_y,ω_z` = 0.05, 0.05, 0.01
- `R`(`u_weights`)= 全 12 成分 `5e-5`(GRF をほぼ自由に使ってよい)
- `panic_weights = 200.0`、`constraint_panic_weights = 20.0`
  (nmpc_controller.yaml)

---

## 詳細: 変数境界と slack(get_bounds_info)

出所: `nmpc_controller/src/quad_nlp.cpp: get_bounds_info()`、go2.yaml。

### 制御(GRF)境界 — go2.yaml `body.u_lb / u_ub`

```
u_lb = [ -2e19, -2e19, 10.0,  ... ]   (各脚 x,y は自由、z ≥ 10 N)
u_ub = [  2e19,  2e19, 150.0, ... ]   (z ≤ 150 N)
```

**遊脚の GRF は境界ごと 0 に潰される**:
`get_bounds_info()` が各脚の control 下限・上限に `contact_sequence_(j,i)`
(0 or 1)を乗じる。遊脚(=0)なら下限・上限とも 0 → 力ゼロを強制。

### 状態境界 — go2.yaml `body.x_lb / x_ub`

- `p_z ≥ 0`(地面より下に行かない)
- `roll, pitch ∈ [-π, π]`
- `yaw ∈ [-10, 10] rad`(PR #400。参照 yaw は `unwrapVector` で連続化されるため、
  ±π のままだと 180° 超の旋回で参照がクリップされる。roll/pitch は据え置き)
- 他(位置 x,y、全速度)は自由(`±2e19`)
- `get_bounds_info()` 本体では状態を一旦すべて `±2e19` にし、
  ソフト境界を slack 制約経由で課す

### ソフト境界と slack 変数

- `x_lb_soft / x_ub_soft`(go2 では hard と同値)を、状態ごとの slack 変数
  `panic` 経由で拘束: `eval_g()` が
  `x_{i+1} + panic_lower` と `x_{i+1} - panic_upper` を制約値に入れ、
  下限は `x_min_complex_soft`(高さ成分だけ `ground_height_(0,i)` に差し替え)、
  上限は `x_max_complex_soft`
- slack 変数は目的関数で `panic_weights = 200` の線形ペナルティ
- `constraint_panic_weights = 20` は緩和制約用(go2 の simple モデルでは
  緩和対象が無いので実質不使用)

---

## 詳細: 最適化(IPOPT)設定

出所: `nmpc_controller/src/nmpc_controller.cpp` コンストラクタ。

- `linear_solver = "mumps"`(**元は `"ma27"`**。HSL 未導入のため変更。handoff 5節)
- `print_level = 5`(**元は `0`**。診断用、そのまま)
- `print_timing_statistics = "no"`
- `tol = 1e-3`
- `dual_inf_tol = 1e10`(双対実行可能性はほぼ無視)
- `constr_viol_tol = 1e-2`
- `compl_inf_tol = 1e-2`
- `fixed_variable_treatment = "make_parameter_nodual"`
  (遊脚 GRF のように上下限が一致する変数をパラメータ扱いにする)
- `ma57_pre_alloc = 1.5`(MA57 用。MUMPS では無効)
- warm start: `warm_start_bound_push / slack_bound_push / mult_bound_push = 1e-6`、
  `mu_init = mynlp_->mu0_`、`warm_start_init_point` は前回成否で `"yes"/"no"`
- **時間打ち切り**: `max_wall_time = max_cpu_time = 4.0 · dt_ = 0.12 s`

### warm start / 初期推定

- `require_init_` は初回 `true` → `update_solver(..., init=true)` で
  ソルバ状態をリセット
- 解が成功するたび `warm_start_ = true` → 次回は前回の primal/dual を
  `plan_index_diff` 分シフトして初期推定に使う(`update_initial_guess()`)
- 失敗すると `mu0_ = 1e-1` に戻し cold start へ

---

## 初期パラメータ一覧(go2、Step 01 時点)

`local_planner.yaml`:

- `local_planner.update_rate = 333.0` Hz
- `local_planner.timestep = 0.03` s(= NMPC の `dt_`)
- `local_planner.horizon_length = 26`(= NMPC の `N_`)
- `stand_cmd_vel_threshold = 0.05`(**元 0.1**。0.1 m/s 指令で歩行移行できない
  問題のため変更。handoff 4節)

`nmpc_controller.yaml`:

- `panic_weights = 200.0`
- `constraint_panic_weights = 20.0`
- `Q_temporal_factor = 100.0`(内部で `^(1/(N-2))`)
- `R_temporal_factor = 1.0`
- `friction_coefficient = 0.3`(**go2.yaml が 0.6 で上書き** → 実効 0.6)
- `enable_variable_horizon = false`
- `enable_mixed_complexity = false` / `enable_adaptive_complexity = false`
  (加えて spirit 以外は強制 false)

`go2.yaml`(`nmpc_controller.body`):

- `x_dim = 12`, `u_dim = 12`, `g_dim = 28`
- `x_weights = [5,5,5, 0.5,0.5,0.5, 0.1,0.1,0.2, 0.05,0.05,0.01]`
- `u_weights = [5e-5] × 12`
- `x_lb`(要点): `p_z ≥ 0`, `roll,pitch ∈ [-π,π]`, `yaw ∈ [-10,10]`
- `u_lb / u_ub`(要点): GRF `f_z ∈ [10, 150] N`, `f_x, f_y` 自由
- `friction_coefficient = 0.6`

`global_body_planner.mass = 16.1` kg(NMPC が `u_nom` の体重項に使用)

---

## 【推測】未確認事項

- **モデルの厳密な呼称**: README は "centroidal-dynamics model" と書くが、
  `dynamicsModel.m` を読む限り「world 位置 + ZYX オイラー角の単剛体を
  ラグランジュ形式で立てたもの」。重心まわりの角運動量表現(狭義の centroidal)
  とは形が違う。呼び方の問題で式自体はコードで確認済み
- **慣性テンソル `I_body` の数値**: `dynamicsModel.m` の
  `parameter.physics.inertia_body` は別スクリプト(`main.m` など)で設定される。
  go2 用の具体値は未確認(CasADi 生成コードにハードコード済みのはず)
- **`ref_foot_acceleration` 等の参照の中身**: NMPC は body だけを解くが、
  `foot_pos_world_` / `foot_vel_world_` を参照として受け取っている。
  これらを local_footstep_planner がどう生成しているかは未精査
- **`print_level = 5` のログ内容**: 精査していない(NMPC 失敗がほぼ 0 件に
  なったため必要性低。handoff 4節)
- **`N = 26` / `dt = 0.03` / `μ = 0.6` / GRF 上限 150 N が go2 の
  0.5〜1.1 m/s 域に最適か**: 未検証。ゲイン・パラメータ調整は調査初期の
  ユーザー制約により未実施(handoff 2節)
- **MUMPS vs MA27 の数値差**: 「MUMPS の精度不足が転倒原因」という証拠は
  調査全体で一度も出なかった(handoff 8節)が、両者の解を直接比較する試験は
  していない

---

## その後(この MPC の位置づけと次に見るべき点)

- **Step 01 の結論**: NMPC はほぼ毎周期成功しており、転倒原因は MPC ではなかった
  (根本原因は起動シーケンスと地面サイズ。handoff 8節)。
- **監視に使える量**: `local_plan`(`RobotPlan`)の `compute_time`・
  `diagnostics.iterations`・`diagnostics.cost` をロガーが
  `plan_compute_time_ms` / `plan_nmpc_iterations` / `plan_nmpc_cost` として
  記録している。`plan_age_s` が伸び続ける = NMPC 失敗が続いている間接指標。
- **WALK 移行直後の姿勢跳躍(handoff)**: MPC 側か WBC 側かの切り分けは未了。
  `agent_reports/quadsdk_step01_wbc.md` と合わせて追う。
- **速度を上げる Step で最初に検討すべきパラメータ**:
  `Q`/`R` の配分、`N`・`dt`(予測長 vs 計算時間 0.12 s の打ち切り)、
  `friction_coefficient`、GRF 鉛直上限 150 N、`max_wall_time`。
  いずれも現状は Step 01 の制約により未変更。

---

## ソース早見表(`external/quad-sdk/`)

- ソルバ設定・エントリ
  - `nmpc_controller/src/nmpc_controller.cpp`(IPOPT オプション、`computeLegPlan` /
    `computePlan`、robot_id → SystemID)
  - `nmpc_controller/include/nmpc_controller/nmpc_controller.hpp`
- NLP 本体(IPOPT `TNLP` 実装)
  - `nmpc_controller/src/quad_nlp.cpp`
    - `eval_f`(コスト、`:478`)/ `eval_grad_f`(`:537`)
    - `eval_g`(制約、`:678`)/ `eval_jac_g`(`:785`)/ `eval_h`(`:1122`)
    - `get_bounds_info`(`:232`)/ `get_starting_point`(`:454`)
    - `update_solver`(`:1739`)/ `update_initial_guess`(`:1400`)
    - `finalize_solution`(`:1316`。`ip_data->iter_count()` が `iterations` の出所)
  - `nmpc_controller/src/quad_nlp_utils.cpp`(`loadConstraintNames`, `:414`)
  - `nmpc_controller/include/nmpc_controller/quad_nlp.hpp`(`NLPConfig` /
    `NLPDiagnostics` / 次元定数 `n_body_=12` ほか)
- ダイナミクスの記号生成
  - `nmpc_controller/scripts/dynamicsModel.m`(EOM・摩擦・CasADi 生成)
  - `nmpc_controller/scripts/generated_codes/matlab/legDynamics.m`
  - `nmpc_controller/src/gen/eval_g_go2.cpp` ほか(生成された C コード)
- パラメータ
  - `local_planner/config/local_planner.yaml`(horizon, timestep, update_rate)
  - `nmpc_controller/config/nmpc_controller.yaml`(重み、複雑度スケジュール)
  - `quad_utils/config/go2.yaml`(`nmpc_controller.body` の次元・重み・境界、`μ`)
- 呼び出し側
  - `local_planner/src/local_planner.cpp`(`computeLegPlan` を毎周期呼ぶ)
