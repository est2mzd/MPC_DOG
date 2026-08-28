# Log 20: 実験評価ログの設計（未実装）

対応プロンプト: Baseline制御を変えず、理論値とMuJoCo実値を層別に比較できるログの設計。
記録日: 2026-08-23。**コードは未変更。実装しない。**

前提:

- 制御式、重み、Gait、clip、`mj_step` 入力を変えない。
- 正本の変数は [A_Variable_Dictionary](../appendices/A_Variable_Dictionary.md) と本会話の解析ログ。
- 既存 `run_simulation` は `state_obs_names=[]` のため `env.step` の `state` は空。`ep_state_history` は評価に使えない。
- 既存 H5 は `recording_path` 指定時だけ。中身は env obs（標準では空）+ 時刻。
- `get_obs()` は一部を返す。`ref_feet_constraints` 分岐のキーはコードと不一致（`ref_foot_FL_constraints`）。ログ設計では正しいキー `ref_foot_constraints_*` を使う。
- Quadruped-PyMPC 展開ディレクトリに `.git` はない。wrapper の `mpc_dog` HEAD と、PyMPC を treefile ハッシュ相当で識別する。

---

## 0. 設計原則

1. 取得は **copy**。トルク・接触・`ctrl` を書き換えない。
2. `mj_contactForce` は viewer 用と同じ読取専用。MPC へ戻さない。
3. 制御周期の同期 I/O をしない。RAM に積み、episode 終了またはバックグラウンドで書く。
4. 保存周期は取得周期以下で間引ける。
5. ログ失敗は `try/except` で握り、sim は続ける。
6. 無効時は logger が no-op。
7. 出力は Git 管理外（例: `experiments/runs/` + `.gitignore`）。
8. ディレクトリ名に UTC 時刻と短い id。同名上書き禁止。

層の分離:

| 層 | 理論/指令 | 実/Plant |
|---|---|---|
| 指令 | `ref_base_*`, `ref_state` | `base_lin_vel`, Euler, `base_pos[2]` |
| Gait | `phase_signal`, `contact_sequence`, `current_contact` | `feet_contact_state` |
| Foothold | FRG / VFA / `nmpc_footholds` | `feet_pos`, `frg.touch_down_positions` |
| MPC | `nmpc_GRFs`, status | 実GRF |
| Swing | `last_des_foot_pos`, `swing_time` | `feet_pos`（遊脚） |
| 関節 | `des_joints_*`（未使用でも記録可） | `qpos[7:]`, `qvel[6:]`, `tau` |

---

## 1. Log対象

取得周期: ループは 500 Hz。`mpc_solve_flag = (step_num % 5 == 0)`。保存周期は推奨。Shape は Go2。

| Log項目 | コード変数 | 生成元 | Shape | 単位 | Frame | 取得周期 | 保存周期 | 最小案で取れるか |
|---|---|---|---|---|---|---|---|---|
| Simulation time | `env.simulation_time` | `QuadrupedEnv` | scalar | s | — | 500 Hz | 500 または 100 | はい。loop済み |
| Simulation step | `env.step_num` | 同上 | int | — | — | 500 Hz | 同上 | はい |
| MPC solve flag | `step_num % 5 == 0` | wrapper条件と同じ | bool | — | — | 500 Hz | 500 Hz | はい。再計算するだけ |
| MPC solve time | `acados_ocp_solver.get_stats('time_tot')` | acados。Samplingは別 | scalar | s | — | 100 Hz | 100 Hz | **probe**。API変更なしで solver を読む |
| Solver status | `solve()` 戻り。現状 loop に未露出 | `Acados_NMPC_Nominal.compute_control` | int | — | — | 100 Hz | 100 Hz | 最小: probe `previous_status`。詳細: 戻りに追加 |
| Current state pos/vel/ori | `com_pos`, `base_lin_vel`, `base_ori_euler_xyz`, `base_ang_vel` | env getters | `(3,)`×4 | m, m/s, rad, rad/s | W / W / xyz / B | 500 Hz | 100 Hz | はい |
| Reference state | `ref_state` または `get_obs` の高さ・角度・足 | `WBInterface` | dict | 混在 | Aどおり | 500 Hz | 100 Hz | 一部 `get_obs`。全文は wrapper メンバ読取 |
| 指令速度（回転前） | `ref_base_lin_vel`（`target_base_vel`直後） | env | `(3,)` | m/s | W | 500 Hz | 100 Hz | はい。**MPC参照と別変数として保存** |
| Gait phase | `pgg._phase_signal` | PGG | `(4,)` | 0–1 | — | 500 Hz | 500 Hz | `get_obs` に既にある |
| Contact sequence | `contact_sequence` | PGG | `(4,12)` | 0/1 | — | 500 Hz | 100 Hz（列0は500） | wrapper戻り。loop未保持 → メンバ or 戻り拡張 |
| Current planned contact | `current_contact` | `contact_sequence[:,0]` | `(4,)` | 0/1 | — | 500 Hz | 500 Hz | `wb_interface.current_contact` |
| Actual MuJoCo contact | `feet_contact_state()` の bool | env | `(4,)` | 0/1 | — | 500 Hz | 500 Hz | **追加呼出**。制御に書かない |
| MPC GRF | `nmpc_GRFs.*` | interface mask後 | 各`(3,)` | N | W | 500 Hz読（中身100 Hz hold） | 100 Hz | `get_obs` 済み |
| MuJoCo actual GRF | `feet_contact_state(..., True)` | `mj_contactForce` | 各`(3,)` | N | W（env変換後） | 500 Hz可 | 100 Hz | 現在は **render時のみ**。ログONなら毎保存周期に読む |
| Nominal foothold | `frg.last_reference_footholds` または VFA前の `ref_feet` | FRG | 各`(3,)` | m | W | 500 Hz | 100 Hz | メンバ読取。blindでは adapted と同じ |
| Terrain-adapted foothold | `ref_state['ref_foot_*']` | VFA後 | `(1,3)` | m | W | 500 Hz | 100 Hz | `get_obs` `ref_feet_pos`。blindでは nominal |
| MPC foothold | `nmpc_footholds.*` | interface | 各`(3,)` | m | W | hold | 100 Hz | `get_obs` 済み |
| Swing desired position | `wb_interface.last_des_foot_pos` | STC直後 | 各`(3,)` | m | W | 500 Hz | 500 Hz（遊脚評価） | メンバ。公開APIなし |
| Swing actual position | `feet_pos.*` | env | 各`(3,)` | m | W | 500 Hz | 500 Hz | はい |
| Actual touchdown | `frg.touch_down_positions` | 遊脚→立脚エッジ | 各`(3,)` | m | W | エッジ | 500 Hz | メンバ |
| Joint position | `qpos[7:19]` または `qpos[legs_qpos_idx]` | mjData | `(12,)` | rad | 関節 | 500 Hz | 100–500 | はい。**`joints_pos` はindexなので使わない** |
| Joint velocity | `qvel[legs_qvel_idx]` | mjData | `(12,)` | rad/s | 関節 | 500 Hz | 同上 | はい |
| Torque before clip | clip前の `tau` の copy | WBC戻り | 各`(3,)` | N·m | 関節 | 500 Hz | 500 Hz | **必須copy**。現行は上書き |
| Torque after clip | clip後 `tau` / `action` | `simulation.py` | `(12,)` | N·m | actuator | 500 Hz | 500 Hz | はい |
| Torque saturation flag | `\|τ_pre\|` が soft limit 以上 | 上2つから算出 | `(12,)` bool | — | — | 500 Hz | 500 Hz | 最小案で新変数。制御に使わない |
| Selected gait frequency | `pgg.step_freq` / `best_sample_freq` | PGG / wrapper | scalar | Hz | — | 500 Hz | 変化時+100 Hz | メンバ |
| Early stance/reflex | `esd.early_stance`, `hitpoints` | ESD | 脚ごと | — | — | 500 Hz | 500 Hz | 標準は常に False/None |

`contact_sequence` 全文は 100 Hz で足りる。列0と実接触の差は 500 Hz で見る。

MPC内部の `u0` 足速度は最小案に入れない（実行されない）。詳細案へ。

---

## 2. 評価指標

離散化は台形または `sum(e^2)*dt / T`。`dt=0.002`。保存が 100 Hz ならその \(\Delta t\) を使う。指標は **episode後** にオフライン計算する（制御ループに載せない）。

与式:

\[
E_v=\frac1T\int_0^T\|v-v^{ref}\|^2dt,\quad
E_{ori}=\frac1T\int_0^T\|\Theta-\Theta^{ref}\|^2dt,\quad
E_h=\frac1T\int_0^T(h-h^{ref})^2dt
\]

コード対応案: \(v=\) `base_lin_vel`（W）、\(v^{ref}=\) **地形回転後** `ref_state['ref_linear_velocity']`（MPCと同じ）。回転前指令との差は別指標 `E_v_cmd` にする。\(\Theta=\) `base_ori_euler_xyz[0:2]`（roll/pitch）。Yawは参照0なので \(E_{ori}\) から外すか、Yaw誤差を別記。\(h=\) `base_pos[2]`、\(h^{ref}=\) `ref_state['ref_position'][2]`（terrain+CoM補正済み）。wrapperの `base_poz_z_err` は `ref_z - base_pos[2]` で、MPCの `ref_position[2]` と一致しない場合がある。指標は **`ref_state` 側** を正とする。

追加:

| 評価指標 | 数式（案） | 必要Log | 単位 | 計算周期 | 注意点 |
|---|---|---|---|---|---|
| Slip distance | \(\sum_i\int_{c_i^{act}=1}\|v_{foot,i,xy}\|dt\) | 実接触、`feet_vel` | m | episode | 計画接触ではなく **実接触**。滑りと沈み込みを混同し得る |
| Foothold error | TDエッジで \(\|p_{td}^{act}-p_{td}^{mpc}\|\) の平均 | `touch_down_positions`, `nmpc_footholds` | m | エッジ | 立脚中の追従誤差とは別 |
| Touchdown timing error | \(t_{td}^{act}-t_{td}^{plan}\) | `current_contact` 立上がり、実接触立上がり | s | エッジ | 計画はPGG。遅延1 step=2 ms |
| GRF tracking error | \(\frac1T\int\|F^{cmd}-F^{act}\|^2dt\)（立脚のみ） | `nmpc_GRFs`, 実GRF, `current_contact` | N² | 100 Hz | 遊脚は指令0。frame差に注意 |
| Torque peak | \(\max_t\|\tau^{after}\|_\infty\) | clip後 | N·m | episode | 脚成分別も残す |
| Torque RMS | \(\sqrt{\frac1T\int\|\tau\|^2dt}\) | clip後 | N·m | episode | before/after両方 |
| Saturation率 | \(\frac1{12T}\sum_j\int \mathbf{1}_{sat,j}dt\) | sat flag | — | episode | soft 0.9 基準 |
| Energy proxy | \(\int\sum_j\|\tau_j\dot q_j\|dt\) | \(\tau^{after}\), 関節速度 | J相当 | episode | 電気損失ではない |
| Impact | 実接触立上がりの \(\|F_z^{act}\|\) または \(\|\dot z_{foot}\|\) | 実GRFまたは `feet_vel` | N または m/s | エッジ | 閾値は人（log 18） |
| Solver failure rate | \(N_{status\in\{1,4\}}/N_{solve}\) | status | — | episode | Samplingは status なし。NaN costは別定義 |
| Fall rate | \(N_{term}/N_{ep}\) | `is_terminated` | — | 実験 | env閾値≠人の転倒定義 |

---

## 3. 保存形式

| 形式 | 長所 | 短所 | 推奨用途 |
|---|---|---|---|
| CSV | 目視・Excel | 遅い、型なし、巨大 | Metadataと指標サマリ（1枚） |
| NPZ | **新規依存なし**（numpy）。辞書一括 | 部分読込が弱い。巨大配列はRAM前提 | **最小案の時系列** |
| HDF5 | 部分読込、既存 `H5Writer` | 標準経路は空obs。h5pyは gym-quadruped 経由であり得るが制御コア非必須 | 長時間・詳細案 |
| Parquet | 列指向、分析向き | **新規依存**（pyarrow） | 採用しない（最小優先） |
| ROS bag相当 | ROS2実機と揃う | simにrosbag依存 | 実機C。sim最小では使わない |

最小案: `meta.json` + `metrics.json` + `timeseries.npz`。episodeごと1セット。

既存 H5 を再利用しない。obsが空で、単位/frameが無い。

---

## 4. 実験Metadata

| 項目 | 取得方法 | 注意 |
|---|---|---|
| Git commit hash | `mpc_dog` の `git rev-parse HEAD` | PyMPCは `.git` なし。`None` + 展開パスを書く |
| Git dirty | `git status --porcelain` | 解析ログやipynbがdirtyでも記録 |
| Config | `mpc_params` / `simulation_params` の deepcopy（numpyはtolist） | `set_weight` はconfig外。`Q,R` を `set_weight` からコピーして別キーで保存 |
| Random seed | `run_simulation(seed=)` | — |
| Controller type | `mpc_params['type']` | — |
| Gait type | `gait` と `gait_params` | freq/dutyも |
| Robot model | `robot`, XML path | gym-quadruped `go2.xml` |
| MuJoCo / Python | `importlib.metadata` / `sys.version` | — |
| 開始時刻 | UTC ISO | フォルダ名にも使う |
| Scene/Terrain | `scene`, `friction_coeff` | 実行時 `geom_friction` もepisodeで |
| User note | 起動引数 | 空文字可 |
| 終了理由 | `is_terminated` / `is_truncated` / step上限 / 例外 | episodeごと |
| 単位・Frame表 | 本ログ §1 の固定JSON | 生データと分離しない |

`18` §8.1 の「PyMPC = 3adfad9」は使わない。wrapper hash と「PyMPC not a git repo」を書く。

---

## 5. 実装位置（2案）

どちらも **未実装**。制御分岐を足さない。

共通: 新規モジュール案 `simulation/experiment_logger.py`（または `docs` 外の `tools/`）。`run_simulation` からだけ呼ぶ。

### 5.1 最小変更案

Main loop と既存公開メンバだけ。`compute_control` の入出力は変えない。clip前 copy と、任意の `feet_contact_state` 読取を loop に足す。

| 変更ファイル | 追加する処理 | API変更 | 制御への影響 | Test方法 |
|---|---|---|---|---|
| `simulation/simulation.py` | logger有効時: clip前 `tau` copy、step後にレコード、episode終了で npz。`feet_contact_state` を保存周期だけ呼ぶ | `run_simulation(..., log_dir=None, user_note="")` 追加。既定Noneで現行どおり | 無効時ほぼゼロ。有効時はcopyと接触読取。`ctrl` は不変 | log_dir=None で既存軌跡と同一（非回帰）。有効時はファイル存在とshape |
| 新規 `experiment_logger.py` | buffer、間引き、meta.json、失敗時swallow | 制御APIなし | なし | 単体: dummy stepでnpzキー |
| `.gitignore` | `experiments/runs/` | なし | なし | — |
| wrapper / WBC / OCP | **変更しない** | なし | なし | — |

最小案で取れない・弱いもの:

- Solver status / time: 勾配なら `controller.previous_status` と `get_stats` を **読むだけ**（loopから）。OCPを再solveしない。無ければ NaN。
- `contact_sequence` 全文: `wb_interface` に保持されていない。列0は `current_contact`。全文が要るなら詳細案か、`update_state_and_reference` の戻りを loop で受ける（戻りは既にあるが wrapper が捨てている）。最小の拡張: wrapper が last `contact_sequence` をメンバに残す（制御に未使用）。これは API ではなく保持。制御式は不変。
- `des_foot_pos`: `last_des_foot_pos` で代替。

推奨保存: 状態・参照・GRF 100 Hz。接触・トルク・sat 500 Hz。

### 5.2 詳細Log案

最小に加え、MPC内部。制御結果は同じ。観測用の get を増やす。

| 変更ファイル | 追加する処理 | API変更 | 制御への影響 | Test方法 |
|---|---|---|---|---|
| `centroidal_nmpc_nominal.py` | solve後に `last_debug` dict（status, time, cost, u0全体, x軌跡任意, yref[0]） | `compute_control` の戻りを増やさない。属性に置く | 代入のみ。solveは既存1回 | debug無しで status 経路が変わらないこと |
| `srbd_controller_interface.py` | `last_debug` を転送 | なし | なし | type=sampling では空dict |
| `wb_interface.py` | 戻りまたはメンバに `contact_sequence`, `ref_state`, `des_foot_pos` | 既存戻りをwrapperが既に受ける。捨てている値を保持 | なし | 標準経路の tau ビット一致 |
| `quadruped_pympc_wrapper.py` | observable キー追加。**壊れている constraints キーは直さず** 新キーだけ足す | `quadrupedpympc_observables_names` に任意キー | 未知キーは現在 ValueError。追加は列挙を増やすだけ | 旧名前リストで例外なし |
| logger | Cost内訳（対角ごとの \(e^\top Q e\) 概算）、摩擦違反（\(F\) と \(\mu\) から事後）、warm-start有無 | — | オフライン | — |

詳細に入れ、最小に入れない:

- Horizon 全 `x,u`（重い。100 Hz・12段・間引き推奨）
- Slack / constraint residual（制約ON時だけ意味がある）
- Sampling の `costs` ベクトル（800本）

負荷: 全Horizonを毎100 HzでRAMに積むと長いepisodeで大きくなる。既定は `u0` + `x` の k=0,2,N だけ。

---

## 6. 安全条件（実装時の契約）

| 条件 | 設計での満たし方 |
|---|---|
| Log失敗でsim停止しない | `record`/`flush` を try。失敗カウンタだけmetaへ |
| 無効時overhead最小 | `if logger is None: skip`。importも遅延可 |
| 制御中の同期I/Oを多用しない | ステップでは list.append。書込は episode 終了 |
| 出力をGit対象外 | `experiments/runs/` を ignore |
| 同名上書きしない | `run_YYYYmmddTHHMMSSZ_<6hex>/`。存在なら suffix |
| 単位とFrame | `meta.json` に §1 表を埋め込む |

`feet_contact_state` は MuJoCo 内部を読むだけだが、毎500 Hzはコストがある。最小案は 100 Hz、または `log_level='contact'` のとき500 Hz。

---

## 7. 資料照合

### `18_Experiments_and_Research_Roadmap.md`

| 箇所 | 判定 | 差分 |
|---|---|---|
| §3 ログ項目の列挙 | 正しい（必要集合） | 本設計で変数名と周期を固定 |
| §4 \(E_v\), \(E_{angle}\) | 正しい | \(\Theta^{ref}\) は roll/pitch。Yaw重み0 |
| §8.1 PyMPC=`3adfad9` | **誤り** | wrapper HEAD。本metaでは分離 |
| §8.2 hook位置（step前後のcopy） | 正しい | clip前copyを明示した |
| §8.2 実GRFをトルクに使わない | 正しい | 本設計も読取専用 |
| §8.2 最初の単位を npz hook | 正しい | 本最小案と一致 |
| §8.3 テスト分離 | 正しい | 非回帰は log 無効時 |

### `appendices/A_Variable_Dictionary.md`

| 箇所 | 判定 | ログ設計への含意 |
|---|---|---|
| 指令の二重 `ref_base_lin_vel` | 正しい | 回転前/後を別キーで保存 |
| `joints_pos` が index | 正しい | 関節角ログは `qpos` |
| `nmpc_GRFs` は mask後 | 正しい | 生 `u0[12:]` は詳細案 |
| 実GRFがAに無い | 不完全 | Aへ後で足す対象。本設計は `feet_contact_state` |
| `des_foot_pos` / sat / status | Aに薄い | 本§1が補う |

---

## 8. 実装しないこと（再掲）

- 本ターンでファイルを作らない、loggerを入れない、gitignore以外も触らない。
- 制御の `if` をログのために増やさない（詳細案の debug 属性は観測専用）。
- Outer-loop tuner を同時に作らない。

実装を依頼されたときの順序:

1. `experiment_logger.py` + `run_simulation` の opt-in + ignore。
2. clip前copy、実接触、meta/npz。
3. pytest: 無効時の非侵入、有効時shape。
4. 必要なら `last_debug`（詳細）。
