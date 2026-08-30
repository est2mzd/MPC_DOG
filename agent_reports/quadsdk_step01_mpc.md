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

## 詳細: 足先(あしば)はコスト・制約にどう入るか

**結論(go2 = simple モデル)**: 足先の位置・速度・接地位置に対する
**専用のコスト項も専用の制約も存在しない**。足先は
**運動方程式のパラメータ(GRF のモーメントアーム)としてだけ** NLP に入る。
「あしばのコスト/制約が見当たらない」のは正しい観察で、go2 では実際に無い。

### 1. なぜコストに無いか(コード)

`nmpc_controller/src/nmpc_controller.cpp` コンストラクタ:

```
components            = {"body", "feet", "joints"}
components_in_simple  = {true,   false,  false}     ← simple モデル = body のみ
components_in_complex = {true,   true,   true}
components_in_cost    = {true,   true,   false}
```

- go2 は `default_system = GO2`、`enable_mixed_complexity = false`、
  `enable_adaptive_complexity = false`(かつ spirit 以外は強制 false)なので
  **全 finite element が simple モデル**。
- simple では `config_.x_dim_cost_simple` に body(12)しか足されない
  → コスト次元 `n_cost_vec_[i] = 12`。
- `quad_nlp.cpp: eval_f()` の該当箇所:
  ```cpp
  Eigen::VectorXd x_nom(n_cost_vec_[i]);
  x_nom.head(n_body_) = x_reference_.col(i + 1);
  if (n_cost_vec_[i] > n_body_) {                      // ← simple では常に false
    x_nom.segment(n_body_, n_foot_ / 2)     = foot_pos_world_.row(i + 1);
    x_nom.segment(n_body_ + n_foot_/2, ...) = foot_vel_world_.row(i + 1);
  }
  ```
  `n_cost_vec_[i] == n_body_` なので **足先位置・速度を追従目標にする分岐に
  入らない**。二次コスト `(1/2)(x - x_nom)^T Q (x - x_nom)` は body 12 成分のみ。
- `Q`(go2.yaml `body.x_weights`)も 12 要素だけで、足先の重みは持っていない。

### 2. なぜ専用制約が無いか(コード)

- go2 の制約は `eval_g_go2`(CasADi 生成)。入力は 2 つだけ:
  `eval_g_go2_name_in` → `0:"w"`(その finite element の `x0, u, x1`)、
  `1:"p"`(パラメータ)。
- 1 finite element の制約次元 `g_dim = 28`(go2.yaml `body.g_dim`)=
  **12(運動方程式 `eom_state_*`)+ 16(摩擦 `friction_*_foot_*`)** だけ
  (`quad_nlp_utils.cpp: loadConstraintNames()`)。
  足先高さ・FK 整合・膝高さ・モーターモデルといった制約名は
  `COMPLEX_TO_COMPLEX`(spirit の adaptive complexity)用で、
  go2 の 28 本には含まれない。

### 3. 足先が「入る」唯一の経路: モーメントアーム

`eval_g()` がパラメータ `pk` を組み立てる(`quad_nlp.cpp:684`):

```
pk[0]              = dt
pk[1]              = mu
pk.segment(2, 12)  = foot_pos_body_.row(i+1)     ← 各脚の接触点位置 r_i(body 相対)
pk.segment(14, 12) = foot_pos_world_.row(i+1)
pk.segment(26, 12) = foot_vel_world_.row(i+1)
pk.segment(38, 16) = 地形(go2 は使わず -10 / 1 で埋める)
```

`foot_pos_body_` は `computeLegPlan()` で
`mynlp_->foot_pos_body_ = -foot_positions_body`(歩容が決めた着地位置)。
これが `dynamicsModel.m` の `feet_location`(記号 `feet_location [12,1]`)に相当し、
運動方程式

```
M(θ) v̇ + h(θ, v) = Σ_i J_i(θ, r_i)^T f_i        (= eom_state_* 制約)
```

の `J_i`(力ヤコビアン = GRF が胴体に与えるモーメントの腕)を決める。
`J_feet` の各列は `dynamicsModel.m: generateLegDynamics()` で
`g_lb = [R_wb, feet_location(3i-2:3i); 0 0 0 1]` の随伴変換から作られ、
`feet_location` に依存する。

→ **足を前に置けば GRF は胴体を後ろに回す方向のモーメントを持ち、
横に置けばロール方向のモーメントを持つ。この効果だけが MPC に反映される。**
足先そのものは決定変数ではなく、毎周期の固定入力。

### 4. contact schedule 経由の間接的な「制約」

厳密には足先そのものの制約ではないが、歩容の接地スケジュールが
`get_bounds_info()` で **遊脚の GRF 上下限を 0 に潰す**(下記「変数境界」節)。
これが「その足は今こう振る舞え」という MPC への実質的な拘束になっている。

### 5. complex モデル(go2 では未使用)なら足先はどう入るか

参考として、`enable_adaptive_complexity` を有効にした spirit などでは:

- 足先が状態に昇格(`feet.x_dim = 24` = 位置 12 + 速度 12)
- **コスト**: go2.yaml `feet.x_weights = [7.51 ×12, 0.111 ×12]`
  (位置追従重み 7.51、速度追従重み 0.111)で
  歩容の計画足先軌道 `foot_pos_world_` / `foot_vel_world_` を追従
- **制約**: `foot_height_leg_*`(地形高さ以上)、`knee_height_leg_*`、
  `fk_pos_*` / `fk_vel_*`(順運動学と body 状態の整合)、
  `motor_model_*_joint_*`(関節速度-トルク包絡線)
- go2 はこれを使わないので、上記はいずれも**効いていない**。

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

## 付録: MPC の基礎(大学院初心者向け)

この付録は、線形代数・微分積分・古典制御(状態方程式、PID、できれば LQR)は
知っているが、**最適化ベースの制御(MPC)や非線形最適化ソルバは初めて**、
という読者向けに、上の節を読めるところまで橋渡しする。既知の人は
J 節(要約)だけ見て飛ばしてよい。

説明用に、ずっと使う**おもちゃの例**を1つ置く:

> **例(1次元の台車)**: 質量 `m` の台車が直線上にあり、
> 位置 `p`、速度 `v`、押す力 `u` を入力とする。状態 `x = [p, v]`。
> 連続時間の運動方程式は `ṗ = v`, `v̇ = u/m`。
> 目標は「目標位置 `p_ref` に、力を使いすぎず、レール端 `|p| ≤ p_max` を
> 守って到達する」。四足の NMPC はこれの多次元・非線形版にすぎない。

### A. なぜ PID ではなく MPC なのか

- **PID / 状態フィードバック**: 「今の誤差」だけを見て入力を決める。
  未来の制約(この先レール端にぶつかる、この先足を出せない)を
  先読みできない。
- **MPC の発想**: モデルで**未来 `N` ステップ**の挙動を予測し、
  「予測した未来全体」を最適化して入力系列を決める。
  制約(位置上限、力上限、摩擦)を**未来にわたって**明示的に課せる。
- **receding horizon(後退ホライズン)**: そうやって求めた入力系列のうち
  **最初の1ステップだけ**を実際に加える。次の制御周期になったら、
  観測した新しい状態から**また最初から解き直す**。
  これでモデル誤差・外乱が入ってもフィードバックとして働く。
  「毎周期、少し先まで見て、1歩だけ踏み出す」を繰り返す。
- **本実装**: ホライズン `N = 26`、刻み `dt = 0.03 s`(→ 約 0.75 s 先まで予測)、
  `local_planner` が `333 Hz` で解き直す。出力の GRF 系列の先頭付近が
  WBC に渡って実トルクになる。

### B. 連続時間モデルを「最適化に載る形」にする(離散化)

最適化ソルバは連続時間の微分方程式 `ẋ = f(x,u)` をそのまま扱えない。
時間を `N` 個の格子点(**finite element / 有限要素**)
`t_0, t_1, …, t_{N-1}`(間隔 `dt`)に区切って、各点の状態
`x_0, x_1, …, x_{N-1}` と各区間の入力 `u_0, …, u_{N-2}` を
**すべて最適化の変数(未知数)にする**。

区間 `k → k+1` で「モデルとつじつまが合う」ことを**等式制約**として課す。
これを**コロケーション(collocation)**と呼ぶ。積分器を回して `x_{k+1}` を
計算するのではなく、`x_{k+1}` も変数にしておいて「制約を満たす組み合わせ」を
ソルバに探させる、という考え方。

- **前進オイラー**: `x_{k+1} - x_k = dt · f(x_k, u_k)`(始点で微分を評価)
- **後退オイラー(backward Euler)**: `x_{k+1} - x_k = dt · f(x_{k+1}, u_k)`
  (**終点** `x_{k+1}` で微分を評価。`x_{k+1}` が両辺に出る「陰的」な式)
  - 陰的なので1ステップの計算は重いが、**数値的に安定**(剛い系でも発散しにくい)。
  - 本実装の運動方程式制約 `eom_state_0..11` がこれ。

台車の例なら区間ごとに
`p_{k+1} - p_k = dt·v_{k+1}`、`v_{k+1} - v_k = dt·u_k/m`。
四足では `x` が 12 次元(胴体の位置・姿勢・速度・角速度)、
`f` が単剛体の運動方程式(C 節)。

### C. 最適化問題(NLP)の3点セット

最適化問題は必ず **(1) 変数 (2) 最小化したい量 (3) 満たすべき条件**
の3つで決まる。本実装は非線形なので **NLP(nonlinear program、非線形計画)**。

1. **決定変数 `w`**(ソルバが動かす未知数)
   - 全時刻の状態 `x_0..x_{N-1}`(各 12 次元)
   - 全区間の入力 `u_0..u_{N-2}`(各 12 次元 = 4脚 × GRF 3 成分)
   - **slack 変数**(E 節。制約を「柔らかく」するための追加変数)
2. **目的関数 `J(w)`**(小さいほど良い)
   - 二次形式の**参照追従コスト** + slack ペナルティ(D 節)
3. **制約**
   - **等式** `g_eq(w) = 0` … 運動方程式(コロケーション)。必ず満たす
   - **不等式** `g_ineq(w) ≤ 0` … 摩擦ピラミッド、変数の上下限
   - **解 = 全制約を満たす `w` の中で `J(w)` が最小のもの**

> ポイント: MPC の「解」は**1点の入力ではなく、未来 `N` ステップ分の
> 状態と入力の系列まるごと**。そのうち先頭だけ使う(A 節)。

### D. 参照追従コストと重み `Q`, `R`(LQR の一般化)

MPC の目的関数の定番は、LQR と同じ**二次形式**:

```
J = Σ_k  (x_k - x_ref,k)^T Q (x_k - x_ref,k)   ← 状態を目標にどれだけ寄せたいか
       +  (u_k - u_ref,k)^T R (u_k - u_ref,k)   ← 入力をどれだけ基準値付近に保ちたいか
```

- `Q`, `R` は**対角の重み行列**。成分ごとに「その量の誤差をどれだけ嫌うか」。
  - `Q` を大きく → その状態成分の追従を優先(ただし入力を余分に使う)
  - `R` を大きく → 入力を節約・なめらかに(ただし追従は甘くなる)
  - `Q` と `R` の**比**がトレードオフを決める(絶対値ではなく比が本質)。
- 台車の例: `Q = diag(q_p, q_v)`、`R = r`。
  `q_p` を上げれば速く目標位置へ、`r` を上げれば弱い力でゆっくり。
- **本実装の値**:
  - `Q` = go2.yaml `body.x_weights` = `[5,5,5, 0.5,0.5,0.5, 0.1,0.1,0.2, 0.05,0.05,0.01]`
    (位置 > 姿勢 > 速度 > 角速度 の順で重視)
  - `R` = 全 `5e-5`(**とても小さい** → GRF はほぼ自由に使ってよい設定)
  - `u_ref`(コード上 `u_nom`)= 「体重を接地脚に等分した鉛直 GRF」。
    つまり MPC は「静止立位に必要な力**からの差分**」を最適化している。
- **終端重み(terminal weight)**: ホライズンの**最後**の状態を特に重く
  罰すると閉ループの安定余裕が増える(理論的にも重要)。
  本実装は行列を1個足す代わりに `Q_temporal_factor = 100^{1/(N-2)}` を
  時刻 `i` 乗して「後ろの時刻ほど `Q` を指数的に大きくする」近似をしている。

### E. ハード制約・ソフト制約・slack 変数(ここが忘れやすい)

**ハード制約** = 「1 mm でも破ったら、その `w` は解として認めない」制約。
破る `w` しか存在しない状況を **infeasible(実行不可能)** と言い、
ソルバは「解なし」を返す。

- 運動方程式(等式)や摩擦 piramid は物理的に必須なのでハードでよい。
- しかし**状態の上下限**(胴体の姿勢角 `±π`、高さ `≥ 0` など)まで
  ハードにすると困る:
  外乱で胴体がほんの少し傾いて制限をかすった瞬間、
  「その状態から先、制約を満たす軌道が存在しない」= infeasible となり、
  **MPC が何も出力を返せなくなる**(制御が止まる = 転倒)。
- リアルタイム制御では「解が返らない」が一番怖い。**必ず何か返させたい**。

**ソフト制約 + slack 変数**でこれを回避する。アイデアは
「基本は守る。どうしても無理なら**少しだけ**はみ出してよい。
ただしはみ出した量にペナルティを払う」:

1. 新しい変数 **slack `s ≥ 0`**(このコードでは `panic` 変数)を追加する。
2. 制約 `x ≤ x_max` を `x ≤ x_max + s` に緩める(`s` の分だけ越境を許す)。
   下限側は `x ≥ x_min − s'` のように別の slack で。
3. 目的関数に **`ρ · s` を足す**(`ρ` = ペナルティ重み、大きい正の数)。
4. すると:
   - 制約に余裕があるとき → `s = 0` が最適(ペナルティを避ける)。元の制約と同じ挙動。
   - 本当に守れないとき → `s > 0` になって**必ず実行可能な解が存在する**。
     はみ出しは `ρ` のペナルティで最小限に抑えられる。

- 台車の例: `|p| ≤ p_max` をソフト化 → `p ≤ p_max + s`, `s ≥ 0`,
  コストに `ρ·s`。普段はレール内、外乱で一瞬超えても「解なし」にならず、
  `ρ` に押されてすぐ戻る。
- **本実装の対応**:
  - `panic_weights = 200.0` … 状態のソフト境界(`x_lb_soft` / `x_ub_soft`)を
    超えた分への線形ペナルティ `ρ`。
  - `constraint_panic_weights = 20.0` … 緩和した一般制約用の `ρ`
    (go2 の simple モデルでは緩和対象がほぼ無く、実質不使用)。
  - コード: `eval_g()` が「`x_{i+1} + s_lower`」「`x_{i+1} − s_upper`」を
    制約値ベクトルに入れて緩和し、`eval_f()` が `panic_weights · Σ s` を
    目的関数に足す。

### F. なぜ slack のペナルティは「1次(線形)」なのか

- ペナルティを **`ρ·s`(1次)** にすると、`ρ` が
  「制約を破ってでもコストを下げたい強さ(コスト勾配)」より大きい限り、
  最適解では `s` が**ちょうど 0 に張り付く**。
  これを **exact penalty(厳密ペナルティ)** と呼ぶ。
  「守れるときは完全に守る」が保証される。
- **`ρ·s^2`(2次)** だと、どんなに `ρ` を大きくしても最適解で `s` が
  わずかに正になり、制約が常時少しにじむ。
- 本実装は `eval_grad_f()` で slack の勾配を**定数** `panic_weights` に
  しており(2次なら `s` に比例するはず)、1次ペナルティだと確認できる。

### G. 非線形だと何が難しいか — KKT と内点法(IPOPT)

四足の運動方程式は回転行列や慣性項が入って**非線形**なので、
問題は QP(目的が2次・制約が線形)ではなく **NLP**。IPOPT で解く。

**(a) 制約付き最適化の最適性条件(KKT)**

制約なしなら「勾配 = 0」で最適。制約付きだと **ラグランジュ乗数**
(dual 変数 `λ`)を導入して、

```
∇J(w) + Σ λ_i ∇g_i(w) = 0        (定常性: コスト勾配と制約勾配が釣り合う)
g_eq(w) = 0,  g_ineq(w) ≤ 0        (元の制約が成り立つ = primal feasibility)
λ_ineq ≥ 0                          (不等式乗数の符号)
λ_i · g_i(w) = 0                    (相補性: 効いていない制約の乗数は 0)
```

この連立が **KKT 条件**(制約付き最適化の1次の必要条件。
学部の「ラグランジュ未定乗数法」に不等式を加えたもの)。
NLP を解く = この非線形連立を解く、とほぼ同義。

- 用語:
  - **primal 変数** = 元の変数 `w`(状態・入力・slack)
  - **dual 変数** = 乗数 `λ`(各制約に1つ。「その制約の効き具合」)
  - **primal feasibility** = 制約が満たされている度合い
  - **dual feasibility** = 定常性の式の残差(勾配の釣り合いの崩れ)
  - **complementarity(相補性)** = `λ_i · g_i = 0` の崩れ

**(b) 内点法(interior point method)**

不等式制約 `c(w) ≥ 0` を直接扱う代わりに、目的関数に
**対数バリア** `−μ Σ log(c_i(w))` を足す。`c_i` が 0 に近づくと
`−log` が `+∞` に発散するので、反復中は**常に制約の内側(interior)に
留まる**。これで「不等式付き問題」を「等式付きのなめらかな問題」に変換できる。

- **バリアパラメータ `μ`** を大きい値から始めて徐々に 0 へ縮める。
  `μ → 0` の極限で、バリア問題の解が元の問題の解に収束する。
- 各 `μ` について、KKT 条件を **Newton 法**で解く。
  Newton の1反復ごとに**大きくて疎な線形方程式 `A Δ = b`** を解く必要がある。
  - この線形ソルバが **MUMPS**(または HSL の `MA27` / `MA57`)。
    本実装は HSL 未導入なので **MUMPS**。
    handoff の「MA27 → MUMPS」変更はここ。
- `mu_init`(初期 `μ`)、`warm_start_*`(初期点を前回解の近くに置く)は
  この反復の**初期化**パラメータ。

**(c) 収束判定(本実装の値)**

- `tol = 1e-3` … 総合的な最適性の許容誤差
- `constr_viol_tol = 1e-2` … 制約違反(primal feasibility)の許容量
- `compl_inf_tol = 1e-2` … 相補性の許容量
- `dual_inf_tol = 1e10` … 定常性(dual feasibility)の残差は**ほぼ見ない**
  (勾配が多少釣り合っていなくても、制約さえ満たせば OK という割り切り)
- `max_wall_time = 4·dt = 0.12 s` … これを超えたら反復を打ち切って
  **途中解を返す**(リアルタイム制約。0.12 s 以内に何か返す方を優先)

### H. warm start(前回の答えから始める)

MPC は毎周期「1ステップ分だけずれた、ほぼ同じ問題」を解く。
前回の最適解を今回の初期推定に使えば、Newton 反復が数回で収束する
(cold start だと数十回)。

- **shift(シフト)**: 前回解を時間方向に1格子ずらして流用する
  (`update_initial_guess(nlp_prev, shift_idx)`)。
  1周期進んだので「前回の `x_1` が今回の `x_0` に近い」はず、という発想。
- 前回が**失敗**した周期は「前回解が良い初期点」の前提が崩れるので、
  `mu0_` を大きめ(`1e-1`)に戻し、バリアを緩くして cold start 相当で
  やり直す。

### I. 接触スケジュールと役割分担(MPC は何を決め、何を決めないか)

- **どの脚がいつ地面につくか(接触スケジュール)は MPC の外**で決まる。
  歩容生成(`local_footstep_planner`)が周期パラメータから作り、
  MPC には**固定入力**として渡る。
- MPC が決めるのは「その接触スケジュールを所与として、
  各接地脚が**どれだけの力(GRF)**を出すか」と「その結果の胴体軌道」。
- **遊脚(浮いている脚)の GRF** は、変数の上下限を 0 に潰して
  **強制的にゼロ**にする(E 節の `get_bounds_info`。変数境界の節も参照)。
- 足を**どこに**置くか(着地位置)も MPC は決めない。歩容が Raibert 則で
  決めて、MPC には「GRF のモーメントアーム(`foot_pos_body_`)」として
  入るだけ(「足先(あしば)はコスト・制約にどう入るか」節)。
- → **GAIT(歩容)→ MPC の一方向**。詳細は
  `agent_reports/quadsdk_step01_gait_and_mpc.md`。

### J. この実装を一言で

> 「接触スケジュールを所与として、単剛体ダイナミクス(backward Euler
> コロケーション)と摩擦ピラミッドを満たしつつ、参照軌道への二次追従コストを
> 最小化する GRF 系列を、状態境界はソフト化(slack + 線形ペナルティ)して
> IPOPT(内点法)で毎周期 warm start しながら解く NLP。
> 出力系列の先頭付近を WBC に渡す。」

### K. さらに学ぶための定番文献

- MPC 全般: Rawlings, Mayne, Diehl, *Model Predictive Control: Theory,
  Computation, and Design*(2nd ed., 無料 PDF あり)
- 数値最適化 / 内点法 / KKT: Nocedal & Wright, *Numerical Optimization*
- 直接コロケーションによる軌道最適化: Matthew Kelly,
  "An Introduction to Trajectory Optimization"(SIAM Review, 2017)
- 四足の MPC/WBC: Di Carlo et al. 2018(MIT Cheetah 3 の凸 MPC)、
  Quad-SDK 論文(`nmpc_controller/README.md` の引用)

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
