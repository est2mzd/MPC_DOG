# Log 18: 自動化済み調整と Outer-loop

対応プロンプト: Sampling入力探索とWeight調整、周波数選択と速度決定、Sim自動化と人の安全判断、Sim-to-Real。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

標準: `type='nominal'`, `optimize_step_freq=False`, `use_integrators=False`, `device='cpu'`。Outer-loop tuner はリポジトリに無い。

区別の前提:

- **Inner-loop**: 1回のMPC（または1 Swing）で \(u,p_{td},f\) を決める。
- **Outer-loop**: Episodeまたは複数Episodeで \(Q,R,f,d,K_p,K_d\) を決める。現行に実装なし。
- Sampling MPC の並列ロールアウトは Inner の入力探索であり、Weight自動調整ではない。
- Frequency候補評価は Cadence 選択であり、目標速度の決定ではない。

---

## 1. 現行コードに存在する自動化

| 機能 | 実装ファイル | 標準で有効か | 調整する対象 | 出力 | Weightを自動調整するか |
|---|---|---|---|---|---|
| Gradient-based MPC | `centroidal_nmpc_nominal.py` `Acados_NMPC_Nominal` | **有効** | Horizon内の \(u\)（足速度+GRF）、状態 \(x\) | `u0` GRF、予測足 | **しない**。`set_weight` 固定 |
| Sampling-based MPC | `centroidal_nmpc_jax.py` / `_gait_adaptive.py` | 無効（`type!='sampling'`） | GRFスプライン係数（と任意で \(f\)） | サンプル最良のGRF | **しない** |
| MPPI | 同上 `compute_control_mppi` | 無効。sampling時既定 `'mppi'` | 同上。重み付き平均で係数更新 | 同上 | **しない** |
| CEM-MPPI | `compute_control_cem_mppi` | 無効。`'cem_mppi'` のとき | 同上 + \(\sigma\) 更新 | 同上 | **しない** |
| Batched simulation | `simulation/batched_simulations.py` | 手動起動。標準simではない | **同じconfig**で並列Episode | プロセス並列の走行（h5任意） | **しない**。\(\theta\) を変えない |
| Batched frequency evaluation | `srbd_batched_controller_interface.py`, `centroidal_nmpc_gait_adaptive.py` | 無効 | \(f\in\{1.4,2.0,2.4\}\) の接触列 | `best_freq` | **しない**。\(J_{MPC}+3(f-1.4)^2\) |
| Frequency candidate selection | 上 + Sampling gait adaptive | 無効 | 同上。速度は固定 | `pgg.step_freq` 更新 | **しない**。\(v^{ref}\) も変えない |
| Foothold optimization | `use_foothold_optimization` → 遊脚 \(\dot p\) | **有効** | Horizon内の足位置 | `nmpc_footholds` | **しない** |
| Integral action | `use_integrators`, `alpha_integrator` | 無効 | 高さ・速度・roll/pitchの積分状態 | バイアス補償 | **しない**。積分ゲインは人手 |
| External wrench compensation | `external_wrenches_compensation` | フラグTrueだが **実質無効** | 渡された外力6 | 予測に加算 | wrapperは `zeros(6)`。推定なし |
| Residual dynamics | Lyapunov `residual_dynamics_upper_bound` | 無効（`type!='lyapunov'`） | 残差ノルムの制約上限 | 実行可能集合 | **しない**。オンライン同定なし |
| Adaptive dynamics | `use_residual_dynamics_decay` | 無効 | 残差上限の減衰 | 制約 | **しない**。Q/Rもモデルも学習しない |
| GPU並列化 | `mpc_params['device']='gpu'` + JAX | 無効。既定 `'cpu'` | Samplingロールアウト並列 | サンプルコスト | **しない** |
| 慣性再計算 | `use_inertia_recomputation` | **有効** | 現在姿勢の `mj_fullM[3:6,3:6]` | MPC `p` の慣性 | Weightではない |
| Velocity modulator | `velocity_modulator` | **有効** | 指令を0にする条件 | \(v^{ref}\) | Weightではない。安全側ヒューリスティック |

結論: 標準で動いている「自動化」は、固定 \(Q,R\) の下での GRF/足位置求解と、姿勢依存慣性だけである。\(Q,R,f,d,K_p,K_d\) の自動探索は無い。

---

## 2. Inner-loop と Outer-loop

| Loop | 最適化対象 | 1回の評価単位 | 実行周期 | 現行実装 |
|---|---|---|---|---|
| MPC Control input探索 | \(u_{0:N-1}\)（GRF、OCP内足速度） | 1回のOCP / 1回のサンプル集合 | 100 Hz | **実装済**。勾配SQPまたはSampling |
| MPC Foothold探索 | 遊脚 \(p_i\)（予測区間） | 同じOCP | 100 Hz | **実装済**（`use_foothold_optimization=True`）。地形∩残時間は見ない |
| Gait frequency候補選択 | 離散 \(f\) | 接触列バッチ1回（4脚overlap付近） | 選択時のみ | **実装あり・標準オフ**。\(v^{ref}\) 固定 |
| MPC weight探索 | \(Q,R\) | Episodeまたは複数 | 実験間 | **未実装** |
| Swing gain探索 | \(K_p,K_d\) | Episode | 実験間 | **未実装** |
| Gait parameter探索 | \(f,d\), gait type | Episode | 実験間 | 人手。候補 \(f\) だけ Inner に部分実装 |
| Domain randomization下のRobust tuning | \(\theta\) を乱れたPlantで評価 | Episode集団 | 実験間 | **未実装**。摩擦範囲サンプリングのみ reset にある |

Sampling/MPPI を Outer と混同しない。あれは **同じ \(Q,R\)** の下で入力軌跡を探す Inner である。

Batched simulation も Outer ではない。同一 `config` をプロセス複製する。

---

## 3. Outer-loop 調整ベクトル（案）

現行に tuner は無い。対応はコード変数への写像と、探索範囲の**設定方法の案**である。仕様・実機データなしに範囲を断定しない。

\[
\theta
=
[
Q_v,\,Q_{angle},\,Q_{foot},\,R_F,\,R_{\dot F},\,R_{footVel},\,f,\,d,\,K_p^{swing},\,K_d^{swing}
]
\]

| 数式変数 | Config/コード変数 | Shape | 既定値 | 探索範囲候補（案） | 制約 |
|---|---|---|---|---|---|
| \(Q_v\) | `Q_velocity` in `set_weight()` | `(3,)` | `[200,200,200]` | 対数スケールで既定の 0.1–10 倍を格子または連続 | xy/zを独立にしてよい。configキーなし |
| \(Q_{angle}\) | `Q_base_angle[0:2]` | `(2,)` または `(3,)` | `[500,500]`（Yaw=0） | 同上 | Yawは0固定を推奨（指令がrateのため） |
| \(Q_{foot}\) | `Q_foot_pos` | `(3,)`×4共通 | `[300,300,300]` | 同上 | 4脚共通が現行 |
| \(R_F\) | `R_foot_force` | `(3,)`×4 | `[0.001]*3` | 対数 1e-5–1e-2 | 全脚共通 |
| \(R_{\dot F}\) | — | — | **未実装** | nominalからは外す。`type='input_rates'` 専用 | \(\theta\) に入れない |
| \(R_{footVel}\) | `R_foot_vel` | `(3,)`×4 | `[1e-4,1e-4,1e-5]` | 対数 | OCP内のみ。Swing実行には未使用 |
| \(f\) | `gait_params[gait]['step_freq']` | scalar | 1.35 | 例: 1.0–2.4 Hz 連続、または候補集合 | 幾何 \(L=v/f\) が clip 内か監視 |
| \(d\) | `gait_params[gait]['duty_factor']` | scalar | 0.74 | 例: 0.55–0.80 | Trot overlap と支持。phase offsetは固定 |
| \(K_p^{swing}\) | `swing_position_gain_fb` | scalar | 500 | 例: 100–1500 | 振動監視 |
| \(K_d^{swing}\) | `swing_velocity_gain_fb` | scalar | 10 | 例: 2–40 | Kpと連成 |

追加で Outer に載せられるが \(\theta\) に無いもの: `ref_z`, `step_height`, `mu`（計画）、`hip_offset`。仕様値（質量、ctrlrange）は \(\theta\) に入れない。

\(R_{\dot F}\) を \(\theta\) に残すなら **type変更が必要** で、標準比較実験と混ざる。

---

## 4. Outer-loop 評価関数

\[
J_{outer}
=
w_1E_v+w_2E_{angle}+w_3E_{height}+w_4E_{slip}+w_5E_{impact}+w_6E_{energy}+w_7N_{sat}+w_8N_{solverFailure}+w_9N_{fall}
\]

現行標準 `run_simulation` は `state_obs_names=[]`。render時だけ実GRFを読む。`J_{outer}` は計算されない。

| 評価値 | 必要Log | 現行取得可能か | 追加実装 | 単位 |
|---|---|---|---|---|
| \(E_v\) | \(v\), \(v^{ref}\) | **取得可能**（env getter）。標準h5には入らない | episode平均の記録 | (m/s)² |
| \(E_{angle}\) | roll, pitch | **取得可能** `base_ori_euler_xyz` | 同上 | rad² |
| \(E_{height}\) | \(z\), `ref_z` | **取得可能**。wrapperは `base_poz_z_err` を内部計算するが既定observableに無い | 観測名を足すか別hook | m² |
| \(E_{slip}\) | 実接触、足速度、計画接触 | **部分的**。`feet_contact_state` と `feet_vel` はある。滑り距離の定義は未実装 | 接地中水平速度積分など | m または (m/s)² |
| \(E_{impact}\) | 着地瞬間の実 \(F_z\) または \(\dot z_{foot}\) | **部分的**。実GRFは関数あり、標準非renderでは未呼出 | TDエッジで記録 | N または m/s |
| \(E_{energy}\) | \(\tau,q\) または \(F,v\) | **部分的**。\(\tau\) と `qvel` はループにある | \(\sum\|\tau\odot\dot q\|\Delta t\) 等 | J 相当 |
| \(N_{saturation}\) | clip前後の \(\tau\) | **要追加**。clipが上書きし、飽和flagを残さない | clip前コピーと bool | 回数 / 時間率 |
| \(N_{solverFailure}\) | acados `status` | **要追加**。`compute_control` 内にあるが wrapper 観測に出ない | status/solve timeをobsへ | 回数 |
| \(N_{fall}\) | `is_terminated` | **取得可能** | episode集計。閾値はenv定義 | 回数 |

`18` が提案するログ表は設計であり、現行ディスク実装ではない。

\(w_i\) は人の重みである。自動探索の対象にすると安全と性能が混ざる。

---

## 5. 探索手法

| 手法 | 連続変数 | 離散変数 | 並列性 | Sample効率 | 適用対象 | 依存 |
|---|---|---|---|---|---|---|
| Grid search | 粗い | 得意（\(f,d\)） | Episode並列可 | 低い | 最初の2–3軸 | **既存**（numpy + `run_simulation`） |
| Random search | 可 | 可 | 高い | 低い | 次元が増えたとき | **既存** |
| Bayesian optimization | 得意 | 工夫が必要 | 低い（逐次） | 高い（低次元） | \(\theta\) の 4–8 次元 | **新規**（例: scikit-optimize） |
| Optuna/TPE | 得意 | 得意 | 中（study並列） | 中–高 | 混在 \(\theta\) | **新規**（optuna） |
| CMA-ES | 得意 | 不向き | 集団並列 | 中 | 連続重み | **新規**（cma） |
| Population-based | 可 | 可 | 高い | 中 | 大規模sim | **新規** または自前 |
| RL（gain/residual） | 可 | 可 | 高い | データ量大 | 研究 | **新規**（torch/jax RL）。Sampling MPCとは別 |

既存依存（本スタック）: numpy, scipy, casadi, acados, mujoco, gym-quadruped。Sampling用 jax。optuna / cma / nevergrad は入っていない。

最初の実装単位の案: `batched_simulations.py` を、プロセスごとに **違う \(\theta\)** を渡す grid/random にする。制御式は変えない。

---

## 6. Domain randomization

| Randomization対象 | 変更箇所 | Episodeごとに変更可能か | 現行実装 | 推奨範囲の根拠 |
|---|---|---|---|---|
| Robot質量 | `mjModel.body_mass` と `config.mass` | 可能（API自作） | **なし**。両者は既に不一致 | 実機秤。範囲は同定後 |
| CoM | body `ipos` / payload body | 可能 | **なし** | 実機CoM試験 |
| 慣性 | `config.inertia` と `mj_fullM` | 可能 | 再計算は姿勢のみ。乱択なし | 振り試験 |
| Ground friction | `friction_coeff` → `_set_ground_friction` | **可能** | **あり**。resetで範囲サンプリング。既定 (0.5, 1.0) | 床の実測μ。MPC μ=0.42とは別 |
| Ground height | `scene`（perlin, boxes, pyramids） | scene切替はepisode可 | **あり**（シーン選択）。連続高さ乱数APIなし | 実験地形 |
| Ground slope | 地形メッシュ / 明示傾斜 | 部分的 | Estimatorは反応する。傾斜乱数なし | 実スロープ |
| Contact softness | `solref`/`solimp` | 可能 | **なし** | 足裏同定。根拠なしに動かさない |
| State estimation noise | getterにノイズ | 可能 | **なし**。完全状態 | 実IMU分散 |
| Communication delay | 指令バッファ | 可能 | **なし** | 実周期計測 |
| Torque delay | \(\tau\) 遅延線 | 可能 | **なし** | モータ同定 |
| Motor strength | `ctrlrange` または gain | 可能 | soft 0.9 のみ固定 | 定格と温度 |
| External force | `xfrc_applied` | 可能 | **なし**。MPC補償もゼロ入力 | 安全な外力上限は人 |
| Payload | 追加質量 | 可能 | **なし** | 実搭載 |

現行で「乱れている」のは摩擦サンプルと scene の幾何だけである。Sim-to-Real の randomization 一式は未実装。

---

## 7. 人が決める項目

自動探索から外す。Outer の \(J\) の重みや停止条件にする。

| 項目 | 理由 | 現行コード |
|---|---|---|
| Hard safety limit | トルク・速度・高さの絶対上限 | torque 0.9、指令 clip \(6hip\)、VM 0.2 m。それ以上の安全層なし |
| Fall判定 | 何を転倒とするか | `env.step` の `is_terminated`。閾値はenv。人の合格基準とは別になり得る |
| 衝撃許容値 | 関節・胴体のピーク | 未定義。\(E_{impact}\) の閾値は人 |
| 実機試験の速度上限 | 段階拡大 | 指令は人が出す。自動 \(v^{ref}\) 決定は無い |
| Emergency stop | 即時全stance / トルクゼロ | simの `start_and_stop` はオフ。E-stopポリシーなし |
| Sensor failure時 | 推定不能 | 標準はセンサ未使用。方針なし |
| Communication loss時 | hold / 停止 | 未実装 |
| 不自然な歩容の許容 | 評価関数に出ない見た目・音 | 人の視聴覚 |

周波数最適化や速度指令を Outer が動かす場合も、上限 \(v_{\max}\)、禁止 gait、禁止 \(d\) は人が先に固定する。

---

## 8. 資料照合

### `15_Automatic_Tuning_and_Sim_to_Real.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §1 Sampling入力とWeight外側は別 | 一致 | 正しい | 本ログ §1–2 |
| §2 既存自動化の列挙 | 慣性再計算、foothold opt、batched freq、GPU sampling、integral、wrench、residual | 不完全 | 標準で有効なのは慣性とfoothold optだけ。他はオフまたは実質Dead。residualは制約であり適応同定ではない |
| §2 「Q,Rを自動調整しない」 | 一致 | 正しい | — |
| §3 \(\theta\) | \(R_{\dot F}\) なし | 正しい（nominal） | ユーザ式の \(R_{\dot F}\) は未実装 |
| §3 \(J_{outer}\) | height/sat/solverなし | 不完全 | ユーザ式の方がログ計画 `18` に近い |
| §4 自動化の順序 | 指針 | 正しい（指針） | 実装ではない |
| §5 DR対象 | 列挙 | 正しい（対象案） | 現行実装は摩擦とsceneのみ、と未記 |
| §6 人が残す判断 | 一致 | 正しい | 本ログ §7 |
| §7 batched_simulations を基礎 | ファイルは存在 | 不完全 | 同一config並列であり tuner ではない |

### `18_Experiments_and_Research_Roadmap.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §2 段階実験 | 指針 | 正しい | Outerはその上に載せる |
| §3–4 ログと指標 | 設計 | 正しい（設計） | 現行標準ループは未記録 |
| §5.7 Outer-loop auto-tuning | 研究候補 | 正しい | 未実装 |
| §8.1 PyMPCを `3adfad9` | 識別子 | 誤り | wrapper HEAD。PyMPCはgit外 |
| §8.2 ログhook | 未実装の計画 | 正しい（計画） | 本ログの「追加実装」と一致 |

### `appendices/F_Open_Questions.md`

| 項目 | 判定 | 本ログ |
|---|---|---|
| Auto-tuned重みのSim-to-Real再現性 | 研究のまま | tuner自体が未実装 |
| Residual学習 | 研究のまま | Lyapunov残差制約 ≠ 学習 |
| Frequency候補の実機範囲 | 未確認のまま | Inner選択と速度決定は別 |
| 外乱補償の内部推定 | 実質なしと確定してよい | 入力ゼロ |

---

## 9. 事実 / 解釈 / 未確認

**事実**

- Weight / Swing gain / Duty の自動探索は無い。
- Sampling/MPPI は入力探索。Batched freq は Cadence 選択。どちらも \(v^{ref}\) と \(Q,R\) を変えない。
- 標準で効く自動化は OCP と慣性再計算と VM と（任意の）摩擦サンプル。
- \(J_{outer}\) の項の多くはセンサ相当が関数としては存在するが、標準ログに乗っていない。

**解釈**

- Outer-loop は新規実験層である。最初は grid/random + 既存 `run_simulation` で足り、optuna は次元が増えてからでよい。
- Sim で絞った \(\theta\) を実機に写す判断（速度上限、転倒定義、衝撃）は人のままにする。

**未確認**

- gym-quadruped の `is_terminated` の正確な不等式（本ログでは「envが返す」まで）。
- 実機ROS2に別tunerがあるか（このtreeには無い）。
