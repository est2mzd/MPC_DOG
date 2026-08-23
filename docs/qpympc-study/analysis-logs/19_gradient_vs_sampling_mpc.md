# Log 19: Gradient MPC と Sampling MPC の同一境界比較

対応プロンプト: 同じ入出力境界での比較。Weight自動調整という誤解の防止。不整地範囲。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

比較の主対象: 標準勾配 `type='nominal'`（`Acados_NMPC_Nominal`）と `type='sampling'`（`Sampling_MPC`）。Factoryにある `input_rates` / `lyapunov` / `kinodynamic` は勾配族だが、本表では「他勾配」と注記するだけにする。

配置: どちらも `WBInterface` の下流、`compute_stance_and_swing_torque` の上流。Factoryは `SRBDControllerInterface.__init__`。Wrapperは同じ `compute_control(...)` を呼ぶ。

---

## 1. Controller選択

| Controller種類 | Config値 | 生成クラス | 呼出関数 | 標準設定か |
|---|---|---|---|---|
| Gradient nominal | `mpc_params['type']='nominal'` | `Acados_NMPC_Nominal` | `controller.compute_control` | **はい** |
| Gradient input_rates | `'input_rates'` | `Acados_NMPC_InputRates` | 同上 | いいえ |
| Gradient lyapunov | `'lyapunov'` | `Acados_NMPC_Lyapunov` | 同上 | いいえ |
| Gradient kinodynamic | `'kinodynamic'` | `Acados_NMPC_KinoDynamic` | 同上 + 関節 | いいえ |
| Gradient batched freq | `type!='sampling'` かつ `optimize_step_freq` | `Acados_NMPC_GaitAdaptive` via `SRBDBatchedControllerInterface` | `optimize_gait` | いいえ |
| Sampling（固定gait） | `'sampling'` かつ `optimize_step_freq=False` | `controllers/sampling/centroidal_nmpc_jax.py` `Sampling_MPC` | `jitted_compute_control` | いいえ |
| Sampling gait adaptive | `'sampling'` かつ `optimize_step_freq=True` | `centroidal_nmpc_jax_gait_adaptive.py` `Sampling_MPC` | 同上 | いいえ |

Sampling手法の下位: `sampling_method` が `'mppi'`（既定）、`'cem_mppi'`、`'random_sampling'`。GPUは `device='gpu'`。いずれも `type='sampling'` のときだけ。

---

## 2. 共通入力

Interface引数は同じ。中で使う量は違う。

| 入力 | Gradient（nominal） | Sampling | 同一か |
|---|---|---|---|
| Current state | `state_current` dict。OCPは位置・速度・Euler・角速度・4足 + 積分6 → x `(30,)` | 同じdictから `(24,)` を連結。積分なし。遊脚足は参照へteleport | **境界は同じdict。内部次元が違う** |
| Reference state | `ref_state`。yref は x+u `(54,)`。足は `(1,3)` | 同じキーから `(24,)`。足を flatten | 境界同じ。終端Qの扱いが違う |
| Contact sequence | `(4, horizon)` を各段 `p[0:4]` | 非adaptive: 同配列をロールアウトに使用。gait adaptive: ロールアウトは **別PGG Jax** で接触を再計算。適用GRFの mask は渡された列の列0 | 境界同じ。adaptive時の内部接触は不一致し得る |
| Friction \(\mu\) | `p` の `mu`（config 0.42） | `self.mu`。サンプルを錐へ **射影** | 値の出所は同じconfig。課し方が違う |
| Inertia | `compute_control(..., inertia)` → 各段 `p` | **未使用**。`Centroidal_Model_JAX` は初期化時 `config.inertia` | **違う** |
| Mass | `config.mass` を `p` へ | 初期化時 `config.mass`。引数なし | 値は同じ定数。再計算慣性は勾配だけ |
| External wrench | 引数あり。標準wrapperは `zeros(6)` | **渡さない**。モデルに外力項なし | 境界に引数はあるがSamplingは無視 |
| Foothold reference | `ref_foot_*` を yref と（制約ONなら）箱 | 遊脚初期足の置換と、出力上書きに使用。OCP決定にはしない | 参照は共有。使い方が違う |

Frame: どちらも Interface 手前は World（角速度は base）。Samplingは平行移動スケーリングをしない。Gradientは `perform_scaling` で原点相対。

---

## 3. 最適化対象

| 対象 | Gradient MPC | Sampling MPC |
|---|---|---|
| GRF | **決定変数** \(u[12:24]\) | **決定変数**（スプライン係数 → 力）。Fzは \(mg/n_s\) へのデルタ |
| Foot velocity | **決定変数** \(u[0:12]\)。実行には未使用 | **対象外**。入力の前半12は力学で未使用 |
| Foothold | 遊脚位置が状態。`use_foothold_optimization` で \(\dot p\) 可 | **対象外**。足は初期値のまま固定（`state[12:]` をコピー） |
| Gait frequency | 本OCPでは固定。batchedが別評価 | 非adaptiveは固定。adaptiveはサンプルごとに候補から選ぶ |
| Duty factor | 対象外 | 対象外。Jax PGGは **ハードコード duty=0.65**（外側Trot 0.74と不一致） |
| Contact timing | 対象外（`p`） | 非adaptiveは対象外。adaptiveは \(f\) 経由で間接に変わる |
| Gait type | 対象外 | 対象外。Jax PGGは Trot offset 固定 |
| MPC weight | 対象外。`set_weight` 固定 | **対象外**。Q/Rは定数。サンプルは入力（と任意で \(f\)）だけ |

SamplingはWeightを自動調整しない。これは Inner の入力探索である（log 18）。

---

## 4. Cost

状態の対角のうち高さ・速度・Roll/Pitch・角速度は数値が揃っている。一致しない項がある。

| 物理量 | Gradient側Cost | Sampling側Cost | Weight | 同一か |
|---|---|---|---|---|
| CoM xy | Q=0 | Q=0 | 0 | 同一 |
| CoM height | Q=1500 | Q=1500 | 1500 | 同一 |
| Linear velocity | `[200]*3` | `[200]*3` | 200 | 同一 |
| Roll/Pitch | 500 | 500 | 500 | 同一 |
| Yaw | 0 | 0 | 0 | 同一 |
| Angular velocity | `[20,20,50]` | `[20,20,50]` | 同上 | 同一 |
| Foot position | `[300]*3`×4 | **Q=0**（24次元の12:24を未設定） | 300 vs 0 | **不一致** |
| Integral | Q 50/10 | 状態に無い | — | **不一致** |
| Foot velocity | R `[1e-4,1e-4,1e-5]` | 変数なし | — | **不一致** |
| GRF | R `0.001`、Fz参照 \(mg/n_s c_i\) | Rは 0.1/0.001 と書いてあるが、加算行が **コメントアウト** | 実効0 | **不一致**（Samplingは状態Costのみ） |
| Terminal | `W_e=Q` | Horizon各段の状態Cost合計。別 \(Q_N\) なし | — | 構造は近いが足/積分が違う |
| Frequency penalty | batchedのみ `3(f-1.4)^2` | adaptiveのみ `100(f-1.3)^2` | — | 別物。標準どちらもオフ |

「同じQを使っている」は胴体6+角速度まで。足と入力Costは同じ問題ではない。

---

## 5. Constraints

| 制約 | Gradient側 | Sampling側 | Hard/Soft/Penalty |
|---|---|---|---|
| Friction cone | Focchi線形4辺。全脚常時 | サンプル後に \(\|F_{xy}\|\le\mu F_z\) へ **clip** | 勾配: Hard（OCP）。Sampling: 射影（実行時強制、最適性保証なし） |
| Normal force | \(F_z\in[0,mg]\) Hard。接触で消さない | \(F_z\) を `[grf_min, grf_max]` にclip。遊脚は接触乗算で0 | 勾配Hard。Sampling射影+mask |
| Foothold | 標準オフ。ONなら箱+slack | **なし** | — |
| Stability | 標準オフ | **なし** | — |
| Torque | OCPに無し | 無し | 下流 0.9 clip が共通 |
| Joint limit | 無し | 無し | — |
| Contact schedule | パラメータ。破れない | 非adaptiveは与列。adaptiveロールアウトは独自列 | 等式というより外生 |
| Terrain collision | 無し | 無し | — |

Samplingの錐は「破ったサンプルを直す」ので、勾配の実行可能集合と同じHard制約ではない。

---

## 6. 出力

Interface戻りは同じタプル。中身の作り方が違う。

| 出力 | Gradient側変数 | Sampling側変数 | Shape | Interfaceで共通化されるか |
|---|---|---|---|---|
| GRF | `u0[12:24]` | 最良スプラインの t=0 の力（+重力補償、接触mask、射影） | `(12,)` → 脚 `(3,)` | **はい**。その後 `* current_contact` |
| Foothold | 立脚=現在足、遊脚=次切替の予測x（±0.15 clip） | 関数はゼロを返したあと **`ref_state['ref_foot_*']` で上書き** | 脚 `(3,)` | **形は共通。中身の定義が違う** |
| Frequency | `pgg_step_freq` をそのまま（batchedは別関数） | adaptiveなら `best_step_frequency`。非adaptiveも戻りにあるが適用は `optimize_swing` | scalar | 同じスロット `best_sample_freq` |
| Predicted state | `x` の k=2（dt≤0.02）の24 | `integrate_jax` 1段の12+足 | 混在 | 同じ戻り名。中身次元が違う |
| Joint pos/vel/acc | None（kino以外） | None | — | 共通してNone |
| Solver status | 1/4で前回GRF | NaN/Inf costを 1e6 にして argmin | — | **共通化されない**。Samplingに status fallback なし |

下流 Stance/Swing は `nmpc_GRFs` と `nmpc_footholds` だけ見る。Controller種類を知らない。

---

## 7. 計算方式

| 項目 | Gradient | Sampling |
|---|---|---|
| 自動微分 | CasADi → acados（ERK, GN Hessian） | JAX jit。コスト最小化に勾配は使わない（MPPIはコスト重み） |
| 勾配 | SQPが用いる | 入力空間のランダム/MPPI。Qの学習なし |
| Rollout数 | 1本の予測軌道（NLP） | `num_parallel_computations=800`、`num_sampling_iterations=1` |
| GPU並列性 | CPU acados | `device='gpu'` なら vmap。既定 cpu |
| Warm start | フラグFalse。solver内部保持はあり得る | `best_control_parameters` を次周期の平均に使う。`shift_solution` 既定False |
| Local optimum | 非凸NLPの局所解 | サンプル依存。大域保証なし |
| Constraint保証 | 摩擦はQP制約（標準） | 射影のみ。箱・安定は無い |
| 計算時間 | 本ログでは未計測。1 SQP iter、N=12 | 未計測。800本 Euler | 断定しない |
| Determinism | 同じ入力なら概ね再現（QP数値差はあり） | PRNG `master_key`。非決定 |
| Debug容易性 | yref/p/status/get_cost | サンプル列とjax。中間OCP変数なし |

---

## 8. 不整地適用（コードにあることだけ）

| 操作 | Gradient | Sampling | 備考 |
|---|---|---|---|
| TerrainをCostへ入れる | **しない**。高さmapはCostに無い | **しない** | VFAが参照xy/zを変えた結果が yref / 初期足になるだけ |
| Safe foothold候補を与える | 参照として与えられる。制約ONなら箱 | 参照へteleportし、出力も参照 | Samplingは候補を最適化しない |
| Contact timingを変える | **しない** | 非adaptiveはしない。adaptiveは \(f\) だけ | Duty/位相の再計画なし |
| Gait frequencyを変える | batched経路（オフ） | adaptive経路（オフ） | どちらも標準オフ。速度は変えない |
| Base速度を下げる | MPCはしない | しない | VMと急傾斜はMPCの外 |
| Plannerと連携する | **しない** | **しない** | モジュールなし |

「Samplingの方が不整地に強い」は本コードからは言えない。不整地情報の入口は共通して FRG/VFA/参照であり、Sampling側に地形Costも衝突制約も無い。足を動かさないため、勾配の foothold optimization より着地の自由度は小さい。

---

## 9. 学習資料の不足（未修正）

| 資料 | 不足 |
|---|---|
| `02_System_Architecture` | `compute_control` が type で中身が分岐すること。Sampling時 `nmpc_footholds` が参照直写であることの未記 |
| `07_MPC_Formulation` | 勾配OCPのみ。Samplingの24状態・足固定・R無効・射影制約が無い。\(Q_N\) 表記は勾配でも `W_e=Q` |
| `13_Feasibility` | 方式差なし。SamplingがTDを最適化しない、adaptive duty=0.65 のずれ |
| `15_Automatic_Tuning` | Sampling=Weight調整 と読める余地。入力探索であることの強調が足りない（log 18と重複） |
| `16_Code_Map` | Factory分岐は無効表にある。Samplingのファイル・Jax PGG・出力上書きがCall graphに無い |

---

## 10. 事実 / 解釈

**事実**

- 配置は同一Interface。標準は勾配nominal。
- SamplingはWeightを変えない。足位置Costは0。GRF Costはコメントアウト。
- Samplingの実行TDは参照Foothold。勾配は予測抽出。
- 慣性再計算と外力は勾配経路だけが引数を使う。

**解釈**

- 公平比較には同じ \(Q\)（足も含む）と同じ接触列と、Sampling側のR有効化が要る。現行のまま「同一Cost」ではない。

**未確認**

- 同一指令でのsolve時間と成功率（未実行）。
- `conj_euler_rates` の勾配CasADi式とJax式の数値一致。
