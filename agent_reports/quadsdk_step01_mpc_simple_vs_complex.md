# Quad-SDK の NMPC: simple モデルと complex モデルの差分 / MPC でできることの違い

作成: 2026-08-31。`external/quad-sdk` の C++ ソース・CasADi 生成コード・
MATLAB 記号スクリプト・YAML を実際に読んで確認した内容。
**【事実】=コードで確認済み**、**【推測】=未確認・仮説** として節を分ける。
関連: `agent_reports/quadsdk_step01_mpc.md`(MPC 本体)、
`agent_reports/quadsdk_step01_gait_and_mpc.md`。

---

## 背景

`agent_reports/quadsdk_step01_mpc.md` では「go2 は simple モデルを使う」
「adaptive/mixed complexity は spirit 専用で go2 では無効」とだけ書き、
**simple と complex で最適化問題そのものがどう変わるのか**、
**complex にすると MPC は何を追加でできるのか** には踏み込んでいなかった。

Quad-SDK の NMPC は「単剛体(simple)」と「単剛体 + 足先 + 関節(complex)」の
2つのダイナミクスモデルを持ち、**ホライズンの区間ごとに** どちらを使うか
切り替えられる(mixed / adaptive complexity)。段差・跳躍・関節限界に
関わる Step へ進むと complex 側の理解が必要になるため、ここで整理する。

## 目的

- simple / complex の **状態・入力・コスト・制約の次元と中身**を1対1で対比する
- complex にすると **MPC が追加で表現・拘束できること** を明文化する
- 区間ごとの切り替え(`fixed` / `adaptive` complexity schedule)の仕組みと、
  go2 でそれが無効な理由をコードで示す

---

## 概要(結論を先に)

- **go2(および spirit 以外の全ロボット)は常に simple モデル**。
  `nmpc_controller.cpp`:
  `if (robot_ns_ != "spirit") enable_mixed_complexity_ = false;`
  → complex 区間が1つも作られない。Step 01/02 の実機挙動は 100% simple。
- **simple モデル**: 状態 = 胴体 12 次元だけ。
  「接触スケジュールを所与に、各接地脚の GRF を最適配分して胴体軌道を作る」。
  足先・関節は決定変数ではなく**外から与えるパラメータ**。
- **complex モデル**: 状態 = 胴体 12 + 足先 24 + 関節 24 = **60 次元**。
  足先の位置・速度、関節角・関節速度まで**決定変数**にして、
  順運動学の整合・足先高さ・膝高さ・関節のトルク-速度包絡線(モーターモデル)
  まで**制約として陽に**扱える。
- **切り替えは区間単位**。`fixed_complexity_schedule`(設定で固定)と
  `adaptive_complexity_schedule_`(実行時に自動判定)の OR を取り、
  各有限要素(finite element, FE)に
  `SIMPLE` / `SIMPLE_TO_COMPLEX` / `COMPLEX_TO_COMPLEX` / `COMPLEX_TO_SIMPLE`
  のいずれかを割り当てる。
- **代償**: complex 区間は 1 FE あたり状態 5 倍・制約 約 3.9 倍。NLP が大きくなり
  1 周期の求解が重くなる(`updateHorizonLength()` が計算超過時に N を削る)。

---

## 詳細: 次元の対比(1 有限要素あたり)

出所: `nmpc_controller/src/nmpc_controller.cpp` コンストラクタ、
`quad_utils/config/go2.yaml`、`quad_nlp.hpp` の次元定数。

コンストラクタが 3 コンポーネントを読む:

- `components = {"body", "feet", "joints"}`
- `components_in_simple  = {true,  false, false}` … simple = body のみ
- `components_in_complex = {true,  true,  true }` … complex = body + feet + joints
- `components_in_cost    = {true,  true,  false}` … コストは body + feet(joints は不参加)

各コンポーネントの次元(go2.yaml):

- **body**: `x_dim 12`, `u_dim 12`(4脚 GRF), `g_dim 28`
- **feet**: `x_dim 24`(位置 12 + 速度 12), `u_dim 24`, `g_dim 28`
- **joints**: `x_dim 24`(関節角 12 + 関節速度 12), `u_dim 0`, `g_dim 52`

積み上げ結果(`NLPConfig`):

```
                     simple            complex
状態  x_dim           12                60   (= 12 + 24 + 24)
入力  u_dim           12                36   (= 12 + 24 + 0)
コスト状態 x_dim_cost 12                36   (= body 12 + feet 24)
コスト入力 u_dim_cost 12                36
制約  g_dim           28               108   (= 28 + 28 + 52)
```

- `x_dim_null = x_dim_complex - x_dim_simple = 48`
  (complex 化で「増える」状態 = 足先 24 + 関節 24)。
  `x_null_nom_`(24 = 関節のノミナル姿勢)は complex 昇格時の初期値に使う
  (`update_initial_guess` で `bottomRows(n_joints_) = x_null_nom_`)。

---

## 詳細: 制約の中身の差

出所: `nmpc_controller/src/quad_nlp_utils.cpp: loadConstraintNames()`。
`COMPLEX_TO_COMPLEX` の 1 FE = **108 本**の内訳:

- **simple と共通(28 本)**
  - `eom_state_0..11` … 単剛体の運動方程式(backward Euler コロケーション、等式)
  - `friction_{x,y}_{pos,neg}_foot_0..3` … 摩擦ピラミッド(16 本、不等式)
- **complex で追加される足先の運動学(24 本、等式)**
  - `eom_{x,y,z}_pos_foot_0..3` … 足先「位置」の時間発展の整合(12 本)
  - `eom_{dx,dy,dz}_pos_foot_0..3` … 足先「速度」の時間発展の整合(12 本)
- **complex で追加される幾何制約(8 本、不等式)**
  - `foot_height_leg_0..3` … 足先が地形高さ以上(`use_terrain_constraint_` 有効時、
    `z_inpainted` と法線を参照)
  - `knee_height_leg_0..3` … 膝が地面にめり込まない
- **complex で追加される順運動学整合(24 本、等式)**
  - `fk_pos_{x,y,z}_foot_0..3` … 関節角から FK した足先位置 = 足先状態(12 本)
  - `fk_vel_{x,y,z}_foot_0..3` … 関節速度から FK した足先速度 = 足先速度状態(12 本)
- **complex で追加されるモーターモデル(24 本、不等式)**
  - `motor_model_{pos,neg}_joint_0..11` … 関節の**トルク-速度包絡線**
    (BEMF 由来。`|τ| ≤ τ_max·(1 - |q̇|/q̇_max)` 形)。
    `get_bounds_info_single_complex_fe()` は遊脚では
    `remove_motor_model_in_swing` でこの上限を `2e19` に開放する。

> つまり complex は「胴体の力学」に加えて
> **「足先の運動学」「地面との幾何」「関節から足先への順運動学」「関節の実行可能性」**
> を同じ最適化の中で一貫させる。simple はこれらを外部(footstep planner、WBC)に
> 委ねている。

---

## 詳細: コストの差

出所: `nmpc_controller.cpp`(重み読み込み)、`quad_nlp.cpp: eval_f()`。

- **simple**: コストは胴体 12 成分の二次追従のみ(`Q` = go2.yaml `body.x_weights`)。
  足先・関節はコストに入らない。
- **complex**: 胴体に加えて**足先の位置・速度をコストで追従**する。
  - go2.yaml `feet.x_weights = [7.51 ×12, 0.111 ×12]`
    (足先位置追従の重み 7.51、足先速度追従の重み 0.111)
  - `eval_f()` の
    `if (n_cost_vec_[i] > n_body_)` 分岐が **complex 区間でのみ true** になり、
    `x_nom` に `foot_pos_world_` / `foot_vel_world_`(歩容の計画足先軌道)を入れる。
  - 関節(joints)は `components_in_cost = false` なので complex でもコスト非参加
    (`x_weights` は空にクリアされる)。

---

## 詳細: 区間ごとの切り替え(mixed / adaptive complexity)

出所: `nmpc_controller.cpp`(コンストラクタ)、
`quad_nlp.cpp: update_structure()`、`adaptive_complexity_utils.cpp`。

### complexity schedule → sys_id_schedule

- 2 本のスケジュール(長さ N の 0/1 ベクトル)
  - `fixed_complexity_schedule_` … 設定で固定
    (`fixed_complex_idxs` / `fixed_complex_head` / `fixed_complex_tail` から生成)
  - `adaptive_complexity_schedule_` … 実行時に自動更新(下記)
- `update_structure()` が両者の要素ごと最大(OR)を `complexity_schedule` とし、
  各 FE `i` に system ID を割り当てる:
  - `complexity_schedule[i]==0 && [i+1]==0` → `default_system_`(go2 なら `GO2` = simple)
  - `complexity_schedule[i]==0 && [i+1]==1` → `SIMPLE_TO_COMPLEX`(昇格の遷移要素)
  - `complexity_schedule[i]==1 && [i+1]==1` → `COMPLEX_TO_COMPLEX`
  - `complexity_schedule[i]==1 && [i+1]==0` → `COMPLEX_TO_SIMPLE`(降格の遷移要素)
- 遷移要素用に別々の CasADi 生成関数がある
  (`eval_g_leg_simple` / `eval_g_leg_complex` /
  `eval_g_leg_simple_to_complex` / `eval_g_leg_complex_to_simple`)。
- FE ごとに `n_vec_[i]` / `m_vec_[i]` / `g_vec_[i]` が simple 値か complex 値に
  変わり、決定変数ベクトルのインデックス(`x_idxs_` / `u_idxs_` など)も
  `update_structure()` で組み直される。

### adaptive complexity の判定ロジック

`NMPCController::updateAdaptiveComplexitySchedule()`
(`adaptive_complexity_utils.cpp`):

1. まず simple(heuristic)で解いた軌道を用意する。
2. その軌道を **complex モデルの制約 `eval_g_single_complex_fe()` で評価**し、
   complex の bound(`get_bounds_info_single_complex_fe()`)と比べて
   違反量を計算する。
3. 状態 bound・制御 bound・一般制約のいずれかが許容誤差
   (`tol` / `constr_viol_tol`)を超えた FE を
   `adaptive_complexity_schedule[i] = adaptive_complexity_schedule[i+1] = 1`
   にする(= 次周期その区間を complex 化)。
4. `is_adaptive_complexity_sparse_` が真なら、**最大違反の1点だけ**を
   complex 化する(疎モード)。
5. 「simple 解を complex 制約で測ったら破っていた」= simple モデルでは
   捉えられない現象(関節が限界、足が地面にめり込む等)が起きている、という
   シグナル。そこだけ高忠実度モデルに切り替える。

### go2 で無効な理由(コード)

- `nmpc_controller.cpp`:
  `if (robot_ns_ != "spirit") enable_mixed_complexity_ = false;`
- `enable_mixed_complexity_ == false` のとき
  `fixed_complexity_schedule` はゼロのまま、`enable_adaptive_complexity_` も
  読まれない → `adaptive_complexity_schedule_` もゼロ。
- 結果 `complexity_schedule` が全ゼロ → 全 FE が `default_system_`(simple)。
- go2.yaml の `nmpc_controller.yaml` 既定も
  `enable_mixed_complexity: false` / `enable_adaptive_complexity: false` /
  `fixed_complex_idxs: [0]`(実質無効化のセンチネル)。

---

## MPC でできることの違い(まとめ)

- **simple(go2 / Step 01・02)ができること**
  - 接触スケジュールを所与に、各接地脚の GRF を最適配分
  - 胴体の位置・姿勢・速度・角速度を参照軌道へ追従
  - 摩擦ピラミッド(接地脚が滑らない範囲で力を出す)
  - GRF の鉛直成分の上下限(10〜150 N)
  - 胴体の姿勢角・高さのソフト境界(slack + panic weight)
  - **できないこと**: 足先をどこに置くか(= footstep planner 任せ)、
    関節が限界かどうか、足先が地面にめり込まないか、
    関節速度-トルクの実行可能性 — これらは NMPC の外(歩容 / WBC / ロボットの
    ファーム)で担保。
- **complex が追加でできること**
  - 足先の位置・速度を**決定変数として最適化**(歩容の計画足先軌道をコストで追従
    しつつ、力学と整合する範囲で微修正)
  - 関節角・関節速度も決定変数にし、**FK 整合**(関節 ↔ 足先)を制約で保証
  - **足先高さ ≥ 地形**、**膝高さ**の幾何制約(段差・凹凸での踏み外し防止)
  - **モーターモデル**(トルク-速度包絡線)で、ファームが実行できない
    参照を最初から出さない
  - → 段差・跳躍(LEAP/FLIGHT/LAND プリミティブ)・関節限界ぎりぎりの
    アジャイル動作で simple より破綻しにくい
- **トレードオフ**: complex 区間は状態 12→60、制約 28→108。
  NLP の変数・非ゼロが増え、IPOPT の 1 反復(線形ソルバ MUMPS)の
  コストが上がる。`updateHorizonLength()` は
  `compute_time > dt_` のとき `N_` を減らして間に合わせる(`enable_variable_horizon`
  有効時)。

---

## 【推測】未確認事項

- **complex モデルの CasADi 生成スクリプト**は本リポジトリに無い
  (`nmpc_controller/scripts/main.m` は simple = `parameter.n = 12` のみ生成)。
  `eval_g_leg_complex*` は生成済みコードとして vendored されており、
  上記「制約の中身」は `loadConstraintNames()` の命名から復元したもの。
  各制約の**厳密な数式**(特に `eom_*_pos_foot` の離散化形、
  モーターモデルの係数)は生成コードを解析していない。
- **`x_null_nom_` に入る関節角の値**は
  `quad_nlp.cpp:57-64` で読み込んでいるが、どのパラメータ(URDF? yaml?)由来かは
  未精査。
- **`is_adaptive_complexity_sparse_` の設定元**(疎モードのオン/オフ)は
  未確認。
- **complex を go2 で強制有効化したときの実挙動**(そもそも go2 用の
  complex 生成コードが存在するのか、`GO2` 系 system ID と leg_complex の
  次元が噛み合うのか)は未検証。コードは spirit 前提で書かれている。
- Step 01/02 は完全に simple なので、complex 側の記述は
  **将来 spirit / 段差 Step へ進むための予備知識**であり、現行の go2 挙動には
  影響しない。

---

## その後(この差分を踏まえて次に見るべき点)

- **平地の Step(01/02)では simple で十分**。complex を気にする必要はない。
- **段差・傾斜・跳躍の Step** に進むとき:
  - go2 で complex を使うには、go2 用の complex ダイナミクス生成コードが
    必要かどうかをまず確認する(spirit しか対応していない可能性が高い)
  - 代替として、simple のまま `use_terrain_constraint_` / footstep planner 側の
    地形対応(`agent_reports/quadsdk_step01_terrain_map.md` /
    `agent_reports/quadsdk_step01_gait_and_mpc.md`)を強化する路線もある
  - `enable_variable_horizon` + `updateHorizonLength()` の挙動
    (計算時間で N が動的に変わる)は complex を混ぜたときに効いてくる
- **監視**: `plan_nmpc_iterations` / `plan_compute_time_ms` が complex 区間の
  混入で跳ねるので、`diagnostics_.complexity_schedule`
  (`RobotPlanDiagnostics.complexity_schedule`)と併せて見る。

---

## ソース早見表(`external/quad-sdk/`)

- モデル構成・次元の組み立て
  - `nmpc_controller/src/nmpc_controller.cpp`
    (`components` / `components_in_simple` / `components_in_complex` /
    `components_in_cost`、`config_` への積み上げ、
    `if (robot_ns_ != "spirit") enable_mixed_complexity_ = false`)
  - `nmpc_controller/include/nmpc_controller/quad_nlp.hpp`
    (`NLPConfig` の `x_dim_simple/complex` ほか、`n_body_=12 / n_foot_=24 /
    n_joints_=24`、`n_null_ / m_null_ / x_null_nom_`、`SystemID` enum)
- 区間ごとの切り替え
  - `nmpc_controller/src/quad_nlp.cpp: update_structure()`
    (`complexity_schedule` = fixed ∪ adaptive → `sys_id_schedule_` /
    `n_vec_` / `g_vec_`)
  - `nmpc_controller/src/adaptive_complexity_utils.cpp`
    (`updateAdaptiveComplexitySchedule` / `updateHorizonLength`)
  - `nmpc_controller/src/quad_nlp.cpp`
    (`eval_g_single_complex_fe` `:605` / `get_bounds_info_single_complex_fe` `:175`)
- 制約の命名(中身の対応)
  - `nmpc_controller/src/quad_nlp_utils.cpp: loadConstraintNames()`(`:414`)
- CasADi 生成コード
  - `nmpc_controller/src/gen/eval_g_go2.cpp`(simple、go2)
  - `nmpc_controller/src/gen/eval_g_leg_simple.cpp` /
    `eval_g_leg_complex.cpp` /
    `eval_g_leg_simple_to_complex.cpp` /
    `eval_g_leg_complex_to_simple.cpp`(区間モデルと遷移)
- パラメータ
  - `nmpc_controller/config/nmpc_controller.yaml`
    (`enable_mixed_complexity` / `enable_adaptive_complexity` /
    `fixed_complex_idxs` / `fixed_complex_head` / `fixed_complex_tail`)
  - `quad_utils/config/go2.yaml`
    (`nmpc_controller.body` / `.feet` / `.joints` の `x_dim` / `u_dim` /
    `g_dim` / `x_weights` / 境界)
