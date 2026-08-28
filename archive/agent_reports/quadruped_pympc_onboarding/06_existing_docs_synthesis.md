# 06 — 既存分析の統合とコード読解順序の提案

日付: 2026-08-26
対象: `external/Quadruped-PyMPC`（`mpc_params['type'] = 'nominal'`）
関連: [AGENTS.md](../../AGENTS.md)（Canonical execution path、10ステップ表）、
本フォルダの `01`〜`05`（既存オンボーディング系列）、
`docs/qpympc-study/`（独立に書かれた別系統の分析コーパス、20ファイル+付録A–F）、
`docs/pympc_2day/`（ワークショップ教材）、`notebook_pympc/README.md`

## 本ファイルの位置づけ

本ファイルは新規のコード読解を行わない。既存の3系統の分析（本フォルダのオンボーディング系列、
`docs/qpympc-study/`、`docs/pympc_2day/`）を突き合わせ、(1) 何がすでに分かっているか、
(2) 理論を理解するためにどの順でコードを読むべきか、を1枚にまとめる統合作業である。

---

## C. 事実 / 文書の記述 / 未確認事項 の凡例と適用

本ファイル全体で、以下の3種の注記を非自明な記述すべてに付す（file `05` と同じ凡例）。

- **事実**: `AGENTS.md`、または本フォルダのオンボーディング系列（`01`–`05`）が、実際に
  `external/Quadruped-PyMPC` のソースコードを読み、行番号・関数名を引用して確認した内容。
- **文書の記述**: `docs/qpympc-study/` または `docs/pympc_2day/` に書かれているが、
  本統合パス（本ファイル作成作業）ではソースコードへ立ち戻って再検証していない内容。
  `docs/qpympc-study/00_README.md` 自身が「本文（00–19、appendices A–F）が学習資料の正本」と
  宣言する独立系統であり、AGENTS.md・オンボーディング系列とは別の著者・別のパスで書かれている
  （`docs/qpympc-study/00_README.md` §2, §3）。
- **未確認事項**: `docs/qpympc-study/appendices/F_Open_Questions.md` に列挙されている項目、
  またはオンボーディング系列各ファイルの「未確認事項」節に列挙されている項目。

3系統間で記述が食い違う場合は、本ファイルでは食い違いを指摘するにとどめ、ソースを読んで
どちらが正しいか判定することはしない（タスク要件どおり）。

---

## A. 分析結果の整理（トピック別）

### A-1. MuJoCo Go2 プラントモデル（自由度・アクチュエータ・接触・センサ）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/01_MuJoCo_Go2_Plant_Model.md` | **文書の記述**（唯一の深掘り章） |
| 本フォルダ `04_state_and_reference_assembly_v2.md` 1節 | **事実**（`state_current`への入力元だけを`simulation.py`から遡って確認） |
| `docs/pympc_2day/WORKSHOP.md` §4.1, §5 | **文書の記述**（教育用の簡略図・簡略式） |

- `qpympc-study/01`は`nq=19`, `nv=18`, `nu=12`, `nsensor=16`、XMLリンク質量合計15.206 kg、
  MPCが使う`config.mass=15.019`（XMLとは別値）、足`condim=6`、`reset()`時に摩擦を上書き、
  といった内容を報告している（**文書の記述**、本パスでは未再検証）。
- オンボーディング系列は`simulation.py`から`state_current`へ渡る各変数（`env.com`, `env.base_lin_vel(frame='world')`,
  `env.feet_pos(frame='world')`等）の生成元を確認した限りで**事実**として`04_state_and_reference_assembly_v2.md`
  1節に記録しているが、`go2.xml`自体（関節可動域、慣性、センサ構成）は本フォルダのどのファイルも読んでいない。
- MuJoCoプラント自体（`go2.xml`、`gym_quadruped/quadruped_env.py`）に対するオンボーディング系列の
  独立コード読解は**存在しない**（AGENTS.mdもこの層を「Entry points to run things」節で言及するのみで
  詳細な自由度表は持たない）。→ 本節がA全体の中で最も「文書の記述のみ」の比率が高いトピックである。

### A-2. ユーザー速度指令の生成（`_sample_ref_vel`, キー入力, `target_base_vel`）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/03_User_Command_and_Reference_Generation.md` | **文書の記述** |
| 本フォルダ `04_state_and_reference_assembly_v2.md` | **事実**（`ref_base_lin_vel`/`ref_base_ang_vel`が`env.target_base_vel()`由来である点まで） |

- `qpympc-study/03`は`gym_quadruped/quadruped_env.py`（外部パッケージ）内部の`_sample_ref_vel()`・
  `_key_callback()`まで読み込んでおり、`config.simulation_params['mode']`は定義されるが
  `run_simulation()`からは参照されず、実際の指令種別は`run_simulation()`の引数
  `base_vel_command_type`である、と報告している（**文書の記述**）。
- **AGENTS.mdとの食い違い（要注意）**: `AGENTS.md`の「Canonical execution path」表2行目は
  「Velocity command | `simulation_params['mode']` in `config.py`」としているが、
  `qpympc-study/03`の記述が正しければ、この`mode`キーは`run_simulation()`から読まれない
  デッドキーであり、実際の指令経路は別（`base_vel_command_type`引数、`_sample_ref_vel`）
  ということになる。本パスではどちらが正しいかソースを読んで確定していない。これは
  AGENTS.mdを「事実」として扱う本タスクの前提と、qpympc-studyの独立記述が
  直接ぶつかる箇所であり、次に検証すべき最有力候補である（D節参照）。
- `gym_quadruped`は外部パッケージであり、`external/Quadruped-PyMPC`自体（AGENTS.mdの対象範囲）
  には含まれない。オンボーディング系列はこの層を意図的にスコープ外としている。

### A-3. 歩容・接触スケジュール（`PeriodicGaitGenerator`）

| 出典 | 種別 |
|---|---|
| 本フォルダ `02_gait_and_contact_sequence_v3.md`（v1,v2を統合・補強） | **事実**（行番号付きで`run()`/`compute_contact_sequence()`の実装を確認） |
| `docs/qpympc-study/04_Gait_Generator_and_Contact_Schedule.md` | **文書の記述** |

- 両者は独立に同じ結論へ到達しており、強い相互検証になっている：位相更新式
  $\phi_i \leftarrow (\phi_i + \Delta t f) \bmod 1$、接地判定 $c_i = [\phi_i < d]$、
  `compute_contact_sequence()`が位相を一時的に進めてから元へ復元する構造、
  `contact_sequence`の列0が現在・列1以降が先読み、trotの`phase_offset=[0.5,1.0,1.0,0.5]`
  （正規化後`[0.5,0,0,0.5]`）、既定`step_freq=1.35`・`duty_factor=0.74`。
- `02_v3`は`GaitType`全8種の位相配置表を追加している点でqpympc-study/04より詳細（**事実**、
  ソース読解に基づく）。qpympc-study/04はTrotの概念行列（overlap非表示）を注記付きで示す。
- 矛盾は見つからなかった。

### A-4. 着地点参照（Raibertヒューリスティック、`FootholdReferenceGenerator`）

| 出典 | 種別 |
|---|---|
| 本フォルダ `03_foothold_reference_generation_v2.md`（v1を整理） | **事実** |
| `docs/qpympc-study/05_Foothold_Reference_and_Terrain_Adaptation.md` | **文書の記述** |

- 両者は数式レベルで一致：horizontal frame変換 $R_{W\to H}$、目標速度項
  $\Delta p_{ref}=\mathrm{clip}(T_{stance}/2 \cdot v_H^{ref}, \pm 1.5 h_{hip})$、
  速度誤差項 $\Delta p_{err}=\mathrm{clip}(\sqrt{h/g}(\bar v_H - v_H^{ref}), \pm 0.05\,\mathrm{m})$
  （符号が「実測−目標」であり教科書的Raibert則と逆順である点まで両者一致）、
  Z成分は`lift_off_positions`からコピー、`hip_offset=0.1`はハードコード。
  `touch_down_positions`が現行コードパスで消費されていない、という指摘も両者一致。
- `03_v2`は`use_foothold_optimization=False`時の挙動について、`05`ファイル（SRBD状態方程式の
  足位置微分項、5.5節）を読むことで解決した経緯を残しており、qpympc-study/05にはこの
  導出過程の記載はない（qpympc-studyは`06_Centroidal_SRBD_Model.md` §7で同じ結論
  `\dot p_i=(1-c_i)(1-s_i)v_{foot,i}`のみ記載）。
- 矛盾は見つからなかった。

### A-5. 状態・参照アセンブリ（`WBInterface.update_state_and_reference`）

| 出典 | 種別 |
|---|---|
| 本フォルダ `04_state_and_reference_assembly_v2.md`（v1を整理） | **事実** |
| `docs/qpympc-study/02_System_Architecture_and_Dataflow.md`, `03`, appendix A | **文書の記述** |

- 強い相互検証: `joints_pos`に実際は`legs_qvel_idx`（インデックス配列）が入り関節角ではない、
  `TerrainEstimator`が実測`feet_pos`ではなく`lift_off_positions`を使う、`terrain_roll`は
  `roll_activated=False`のため常に`0`、`current_contact`引数は`TerrainEstimator`内で
  実行されるコードからは未使用（三重引用符でコメントアウト）——これらはオンボーディング系列
  `04_v2`とqpympc-study双方（`appendices/E_Corrections_and_Clarifications.md` §13, §14、
  `05_Foothold_Reference_and_Terrain_Adaptation.md` §2）が**独立に**同じ結論に達している。
- `state_current["position"]`がCoM位置、`state_current["linear_velocity"]`がbase速度（CoM速度でない）
  という非対称性も両者一致（qpympc-study appendix Aの変数辞書、オンボーディング`04_v2`7節）。
- 矛盾は見つからなかった。

### A-6. SRBD/centroidal 力学モデル（状態・入力・パラメータ・運動方程式）

| 出典 | 種別 |
|---|---|
| 本フォルダ `05_nominal_ocp_variables_and_dynamics.md` | **事実**（`cs.vertcat()`引数を数えて次元を実測、docstringの誤りを発見） |
| `docs/qpympc-study/06_Centroidal_SRBD_Model.md` | **文書の記述** |

- 状態30次元（24基本+6積分）、入力24次元という結論は完全に一致する。
  `omega_x/y/z_integral`という3つのシンボルが定義されるが状態ベクトルに組み込まれない
  （デッドシンボル）という指摘も両者一致（オンボーディング`05` 2節、qpympc-study `06` §2）。
  `stance_proximity`が実装上常に`0`（`1*0`）である点も一致（オンボーディング`05` 6節、
  qpympc-study `06` §7、`appendices/E` §25）。
- `inertia`パラメータのframe（world系かbody系か）について、`simulation.py`のコメント
  「world frame」と`forward_dynamics()`内の使われ方（body系オイラー方程式の形）が食い違う
  という**同じ疑問**を、オンボーディング`05` 7節とqpympc-study
  （`01_MuJoCo_Go2_Plant_Model.md` §4.2、`appendices/F_Open_Questions.md`）が**独立に**
  提起している。矛盾ではなく、2系統が同一の未確認事項に別々に到達した例。
- オンボーディング`05`は`forward_dynamics()`のdocstringが状態・入力・パラメータの次元を
  すべて誤記している（例: 実30次元に対し「29」と記載）ことを発見しているが、これは
  qpympc-studyには明記されていない（qpympc-studyの方が新しい発見ではなく、単に
  docstring不一致という切り口で見ていない）。

### A-7. OCP定式化（コスト関数・摩擦錐・foothold/安定性制約・ソルバー）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/07_MPC_Formulation.md` | **文書の記述**（唯一の深掘り章） |
| 本フォルダ `05_nominal_ocp_variables_and_dynamics.md` | 明示的にスコープ外（「評価関数（Q/R重み）、摩擦錐制約、着地点制約、安定性制約...はスコープ外」と明記） |

- **これが既存分析全体で最大のギャップである**。`qpympc-study/07`は`set_weight()`の重み表
  （位置`[0,0,1500]`、速度`[200,200,200]`、姿勢`[500,500,0]`等）、Focchi線形摩擦錐20式、
  終端コスト`W_e=Q`（別の`Q_N`は存在しない、旧誤り訂正済み: `appendices/E` §20）、
  Solver失敗時`status in {1,4}`で`previous_optimal_GRF`を使い直後に`mg/n_s`代入が
  死文化する（`appendices/E` §21）、といった具体的な内容を報告しているが、これらはすべて
  **文書の記述**であり、本フォルダのオンボーディング系列によるコード読解での再確認は
  一切行われていない。
- AGENTS.mdの10ステップ表・ステップ7（「OCP (acados NLP)」→`centroidal_nmpc_nominal.py`）は
  ファイルを指すのみで、コスト・制約の中身には立ち入っていない。

### A-8. Gait–MPC結合（接触フラグ`c_i,k`の力学への入れ方）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/08_Gait_MPC_Coupling.md` | **文書の記述** |
| 本フォルダ `05_nominal_ocp_variables_and_dynamics.md` 5.2, 5.4, 6節 | **事実**（並進・回転運動方程式内での`stance_i`乗算までは確認済み） |

- オンボーディング`05`は運動方程式内で`stance_i`（0/1）がGRFに乗算される構造を実装レベルで
  確認しているが、「なぜ遊脚GRFをOCP内で明示的に等式ゼロにしないのか」という設計意図の
  議論はしていない。qpympc-study/08・09はこれを「遊脚GRFの3段」（力学Gate／摩擦錐は
  全脚常時／出力Mask）として整理しているが、この整理自体はオンボーディング系列の
  コード読解では再確認されていない**文書の記述**である。

### A-9. レシーディングホライズンとGRF出力（`SRBDControllerInterface`）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/09_MPC_Output_and_Receding_Horizon.md` | **文書の記述**（`perform_scaling`、遊脚足teleport、3段GRF、`nmpc_predicted_state`のスライスまで深掘り） |
| 本フォルダ `01_execution_order_trace_v2.md` B1-2節 | **事実**（ただし呼び出し順序の記録にとどまり、`perform_scaling`が実際に何をするか、GRFにcontact maskが掛かる、という中身までは踏み込んでいない） |

- `01`は`self.perform_scaling(...)`という呼び出しがある事実は記録しているが、「原点への
  平行移動である」という中身の解明はしていない。同様に`SRBDControllerInterface.compute_control()`
  が`nmpc_GRFs = c_{i,0} \cdot F^{MPC}`というMaskを行う、という点も`01`のB1-2節には
  明記されていない（GRF抽出のみ記載）。→ 出力側のMask機構は**文書の記述のみ**でギャップがある。
- オンボーディング`05` 9節が`optimal_next_state`が状態30次元中先頭24次元だけを取り出す
  （`get(k,"x")[0:24]`）ことを実装レベルで確認しており、これはqpympc-study/09の
  `nmpc_predicted_state`記述と一致する（**事実**同士の裏付け）。

### A-10. 立脚・遊脚トルク（`WBInterface.compute_stance_and_swing_torque`）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/10_Stance_and_Swing_Control.md` | **文書の記述**（$\tau^{stance}=-J^T F^{cmd}$、swingのPDが二重に入る構造、摩擦補償が全脚に適用、まで深掘り） |
| 本フォルダ `01_execution_order_trace_v2.md` B1-4節 | **事実**（呼び出し順序のみ。式の中身・符号・ゲイン値の確認はしていない） |
| `docs/pympc_2day/WORKSHOP.md` §5.4 | **文書の記述**、**式が他2系統と食い違う（下記）** |

- **系統間の矛盾**: `docs/pympc_2day/WORKSHOP.md` §5.4の表は立脚トルクを
  `τ = J_i^⊤ F_i + PD` と記載している。これに対し`qpympc-study/10`（および`AGENTS.md`の
  「Non-obvious gotcha」直前の10ステップ表9行目、`τ = -Jᵀf`）は符号が逆で
  （`τ^{stance} = -J^T F^{cmd}`）、かつ立脚側にPD項は無い（PDはswing側のみ、しかも
  二重に入る）としている。`pympc_2day/WORKSHOP.md`は教育目的の簡略化資料であり
  （§2.2で「acados内部のCasADiモデル編集」等を非目標と明記）、符号と項の単純化・誤記の
  可能性がある。本パスではソースを読んで確定していないため、**食い違いの指摘にとどめる**。
- オンボーディング系列にはこの層（立脚・遊脚トルクの式そのもの、ゲイン値、二重PD構造）の
  独立した深掘りが存在しない。`01`は関数呼び出し順序を列挙するのみ。→ 次の有力な
  オンボーディング系列エントリ候補（D節）。

### A-11. 実機トルク適用・MuJoCoクローズドループ

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/11_Joint_Torque_and_MuJoCo_Closed_Loop.md` | **文書の記述**（`0.9 * ctrlrange`クリップ、`action`組立、MPC目標GRFと実接触力`λ`の区別まで） |
| 本フォルダ `01_execution_order_trace_v2.md` B3節 | **事実**（クリップと`env.step()`の呼び出し自体は確認済みだが、クリップ係数`0.9`の値やactuator順の詳細までは踏み込んでいない） |

- 大きな矛盾はない。qpympc-studyの方が数値（`0.9`係数、`actuator_ctrlrange`の具体値）まで
  踏み込んでいる分、詳細度が高い。

### A-12. 速度・周波数・duty・ストライドの関係

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/12_Speed_Frequency_Duty_and_Stride.md` | **文書の記述** |
| オンボーディング系列 | 対応する専用ファイルなし |

- $L_{footprint}=v/f$（同一脚の連続接地点間隔、胴体相対の踏み出し量ではない）という訂正済みの
  解釈、`optimize_step_freq`標準OFF、候補周波数`{1.4, 2.0, 2.4}`（既定trotの1.35は候補に
  含まれない）といった内容。すべて**文書の記述**であり、本フォルダに対応する深掘りはない。

### A-13. 不整地での実現可能性（3集合の交差、VFA）

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/13_Feasibility_on_Rough_Terrain.md` | **文書の記述** |
| オンボーディング系列 | 対応する専用ファイルなし（`03_foothold_reference_generation_v2.md`はVFAを明示的にスコープ外としている） |

- $p_{td}\in\mathcal S_{terrain}\cap\mathcal R_{kinematic}\cap\mathcal R_{timing}$という
  理論的必要条件は「標準コードには交差を取る関数は無い」と明記されており（**文書の記述**）、
  オンボーディング系列はVFA自体を未読なので検証不能。

### A-14. MPC・コントローラのチューニング

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/14_MPC_and_Controller_Tuning.md`, `appendices/C_Parameter_Index.md` | **文書の記述**（`set_weight()`の重み値の出典として`07`を参照） |
| `docs/pympc_2day/TUNING_GUIDE.md`, `MPC_TUNING_JOURNEY.md`, `SPEED_TERRAIN_TRIAL_LOG.md` | **文書の記述だが実測ベース**（実際にシミュレーションを走らせた結果の記録） |

- `pympc_2day`系列は「コードを読んで導いた事実」ではなく「実際にシミュレーションを多数回
  実行して得た経験的パラメータ」（例: trot既定`step_freq=1.35`に対しワークショップ実験では
  `1.2–1.75`、`mu`既定`0.42`に対し実験では`0.35–0.55`をスイープ）であり、コード上の
  デフォルト値の**確認**ではなく**別のconfig/プリセットでの実行結果**である点に注意。
  両者は矛盾するのではなく、対象（デフォルトconfig vs. ワークショップ用プリセットYAML）が
  異なる。
- `qpympc-study/07`の重み表（`Q_position=[0,0,1500]`等）とオンボーディング系列（`05`が
  明示的にスコープ外としている値）を突き合わせる作業はまだ行われていない。

### A-15. 自動チューニング・Sim-to-Real

| 出典 | 種別 |
|---|---|
| `docs/qpympc-study/15_Automatic_Tuning_and_Sim_to_Real.md` | **文書の記述**（標準ONの自動化は慣性再計算とfoothold最適化のみ、外側`Q,R`探索は未実装、と整理） |
| オンボーディング系列 | 対応ファイルなし |

### A-16. コードマップ・呼び出しグラフ・無効経路

| 出典 | 種別 |
|---|---|
| `AGENTS.md`「Module map」節、「Controller-type dispatch」節 | **事実** |
| `docs/qpympc-study/16_Code_Map_and_Call_Graph.md`, `appendices/D_File_Function_Index.md` | **文書の記述**（AGENTS.mdより詳細。標準設定で無効化される経路の一覧を持つ点が付加価値） |
| 本フォルダ `01_execution_order_trace_v2.md` | **事実**（呼び出し順序という一次元でAGENTS.mdの10ステップ表を関数レベルへ展開） |

- 3者は矛盾しない。qpympc-study/16は「標準設定で無効、または到達不能な経路」の一覧
  （`_key_callback`、VFA、`optimize_step_freq`、`use_RTI`、Early stance、関節PD加算のコメント
  アウト等）を持ち、これはAGENTS.md・オンボーディング系列のどちらにも対応する一覧がない
  （**文書の記述のみ**、付加価値が高い一覧）。

---

## B. 理論を理解するためのコードを読む順序（AGENTS.md 10ステップを背骨として）

`AGENTS.md`の「Canonical execution path」表（10ステップ、**事実**）を骨格とし、
各ステップについて既存の深掘り資料と、次に読むべき理由を付す。

| # | Step（AGENTS.md） | ファイル | 既存の深掘り資料 | この順で読む理由（依存関係） |
|---|---|---|---|---|
| 1 | Plant observation | `simulation/simulation.py::run_simulation` | **事実**: `04_state_and_reference_assembly_v2.md` 1節（`state_current`への入力元のみ）。**文書の記述**: `qpympc-study/01_MuJoCo_Go2_Plant_Model.md`（自由度・アクチュエータ・接触の全体像、**唯一の深掘り**）。**ギャップ**: オンボーディング系列に対応する専用ファイルなし | すべての後段（状態・力学モデル・トルク変換）が扱う変数（`feet_pos`, `qvel`, `inertia`等）の生成元。ここを誤解すると、後段のframe・単位の混同に気づけない（`04`が発見した`state_current`内のframe混在も、この層の理解が前提） |
| 2 | Velocity command | `config.py::simulation_params['mode']` | **文書の記述**: `qpympc-study/03_User_Command_and_Reference_Generation.md`（`mode`は未参照でqpympc-study独自記述。AGENTS.mdとの食い違いはA-2節参照）。オンボーディング系列に専用ファイルなし | 目標速度がGaitの位相そのものを変えないこと（A-3節）、Footholdとref_stateで異なる時点の速度が使われること（A-4, A-5節）を理解する前提として、まず「速度は何によっていつ決まるか」を固定する必要がある |
| 3 | Gait / contact schedule | `helpers/periodic_gait_generator.py::PeriodicGaitGenerator` | **事実**: `02_gait_and_contact_sequence_v3.md`（v1,v2の内容を含み最新）。**文書の記述**: `qpympc-study/04` | `contact_sequence`（接地フラグ$c_{i,k}$）はSRBD運動方程式（ステップ6）にパラメータとして入る。力学モデルを読む前に、この$c_{i,k}$がMPCの決定変数ではなく外部から与えられる既知パラメータであることを理解しておく必要がある（AGENTS.mdの表自体がこの順序で示している） |
| 4 | Foothold reference | `helpers/foothold_reference_generator.py::FootholdReferenceGenerator` | **事実**: `03_foothold_reference_generation_v2.md`。**文書の記述**: `qpympc-study/05` | `contact_sequence`の立脚/遊脚遷移（ステップ3の出力）を使って離地・着地位置を更新する（`03_v2`1–2節）。この着地点参照`ref_feet_pos`はステップ7のOCPコストの参照値になるため、OCPを読む前に「参照はどこから来るか」を先に固定する |
| 5 | State/reference assembly | `interfaces/wb_interface.py::WBInterface` | **事実**: `04_state_and_reference_assembly_v2.md`。**文書の記述**: `qpympc-study/02`, `03` | ステップ1–4の出力（観測・速度指令・接地列・着地点）をすべて集約して`state_current`/`ref_state`/`contact_sequence`という、ステップ6・7のOCPが直接受け取る形へ変換する層。ここを読まずにOCPへ進むと、`position`がCoMで`linear_velocity`がbase速度という非対称性（A-5節）に気づけないままOCPの状態変数を誤解する |
| 6 | SRBD model | `controllers/gradient/nominal/centroidal_model_nominal.py::Centroidal_Model_Nominal` | **事実**: `05_nominal_ocp_variables_and_dynamics.md`（状態30/入力24/パラメータ29の実測、運動方程式の数式化）。**文書の記述**: `qpympc-study/06` | ステップ5が組み立てた`state_current`/`ref_state`/`contact_sequence`が、実際にどの状態変数・パラメータへ対応するかを固定する。ステップ7（OCP）のコスト関数・制約はこの状態・入力次元を前提に書かれるため、先に力学モデルの次元と物理量を把握しないとコスト関数の重み表（$Q$の対角30個等）が読めない |
| 7 | OCP (acados NLP) | `controllers/gradient/nominal/centroidal_nmpc_nominal.py::Acados_NMPC_Nominal` | **文書の記述のみ**: `qpympc-study/07_MPC_Formulation.md`（コスト重み・摩擦錐・soft constraint・solver失敗時挙動）。**ギャップ（最大）**: オンボーディング系列は`05`で明示的にスコープ外としており、コード読解による再確認が存在しない | 力学モデル（ステップ6）の上に、最小化する目的関数と制約を載せる層。ここを読まないと「なぜ既定trotで沈み込みが起きるか／振動するか」といったチューニング相談（`docs/pympc_2day/TUNING_GUIDE.md`）の根拠に手が届かない |
| 8 | Receding horizon（stage-0のみ採用） | `interfaces/srbd_controller_interface.py::SRBDControllerInterface` | **事実（呼び出し順序のみ）**: `01_execution_order_trace_v2.md` B1-2節。**文書の記述（中身）**: `qpympc-study/09_MPC_Output_and_Receding_Horizon.md`（`perform_scaling`の中身、遊脚足teleport、GRFの3段構造とMask）。**ギャップ**: Mask機構`nmpc_GRFs=c_{i,0}F^{MPC}`のコード読解による再確認なし | OCP（ステップ7）が返す予測ホライズン全体の解のうち、実際に使うのは先頭段だけであるという「receding horizon」の原理を理解しないと、ステップ9のトルク計算に渡る`nmpc_GRFs`がホライズン全体の解ではなく先頭段でMaskされた値であることを見誤る |
| 9 | GRF → joint torque | `wb_interface.py::WBInterface`（stance: `τ=-Jᵀf`; swing: PD + feedback linearization） | **事実（呼び出し順序のみ）**: `01_execution_order_trace_v2.md` B1-4節。**文書の記述（式・符号・ゲイン）**: `qpympc-study/10_Stance_and_Swing_Control.md`。**注意**: `docs/pympc_2day/WORKSHOP.md`の式表記と符号が食い違う（A-10節） | ステップ8で得たMask後GRFと着地点を、実際にロボットを動かすトルクへ変換する最終段。MPCの出力（力）と実際の駆動指令（トルク）を混同しないために、ステップ7・8で「MPCは何を最適化し何を返すか」を先に固定してから読む必要がある |
| 10 | Actuate plant | `simulation/simulation.py` → `env.step(action)` | **事実（呼び出しのみ）**: `01_execution_order_trace_v2.md` B3節。**文書の記述（クリップ係数・action組立の詳細）**: `qpympc-study/11_Joint_Torque_and_MuJoCo_Closed_Loop.md` | 制御器の出力が実際にMuJoCo物理エンジンへ渡り、次周期の観測（ステップ1へ戻る）を生む閉ループの最終リンク。MPC目標GRFと実接触力`λ`が別物である（`qpympc-study/11` §5）ことを理解して初めて、AGENTS.mdの10ステップが1つの閉ループであることが完結する |

**読む順序についての補足（AGENTS.mdの依存関係に基づく）**:
- ステップ3の$c_{i,k}$がステップ6の運動方程式に**パラメータとして**（決定変数ではなく）入る、
  という事実（A-6, A-8節）が、この順序全体を貫く最重要ポイントである。Gait（ステップ3）を
  先に理解しないままSRBD力学（ステップ6）を読むと、「なぜ`stance_i`が単に運動方程式に乗算
  されるだけなのか」を設計判断ではなく実装の偶然と誤解しやすい。
- ステップ7（OCP定式化）を読まずにステップ8・9（Receding horizon、トルク変換）へ進むと、
  「MPCの出力＝実際に使われる指令」という誤解をしたまま`nmpc_GRFs`の3段構造
  （力学Gate／摩擦錐は全脚常時／出力Mask、A-9節）を理解できない。

---

## D. 次の一歩（優先度つき）

1. **最優先: ステップ7（OCP定式化）の新規オンボーディング系列エントリ（`07_...md`）を作る。**
   `05_nominal_ocp_variables_and_dynamics.md`が明示的にスコープ外とした`create_ocp_solver_description()`、
   `set_weight()`、`create_friction_cone_constraints()`、`set_stage_constraint()`を実際に読み、
   `qpympc-study/07_MPC_Formulation.md`の重み表・摩擦錐20式・soft constraint・solver失敗時挙動
   （`status in {1,4}`時の`mg/n_s`死文化、`appendices/E` §21）をソースで再検証する。
   本節はAGENTS.mdの10ステップ中もっとも既存のコード読解カバレッジが薄い（A-7節）。

2. **次点: ステップ8・9（Receding horizon抽出とstance/swingトルク）のオンボーディング系列エントリ。**
   `srbd_controller_interface.py`のGRF Mask（`nmpc_GRFs=c_{i,0}F^{MPC}`）、`perform_scaling()`の中身、
   `wb_interface.py::compute_stance_and_swing_torque()`の式・符号・ゲイン・二重PD構造
   （`qpympc-study/10`）を実際にコードで確認する。`docs/pympc_2day/WORKSHOP.md`の
   `τ = J^⊤F + PD`という記述（A-10節）との食い違いも、この過程で自然に解消できる。

3. **`AGENTS.md`ステップ2の記述とqpympc-study/03の食い違いを検証する。**
   `simulation_params['mode']`が実際に`run_simulation()`から参照されているか、
   実際の速度指令経路は`base_vel_command_type`引数なのか、`simulation/simulation.py`を
   直接読んで確定する（A-2節）。AGENTS.mdは本タスクでは「事実」として扱う前提だが、
   この1点はqpympc-studyの独立記述と直接対立しており、AGENTS.md自体の記述精度に関わる
   ため優先度が高い。

4. **`inertia`パラメータのframe（world/body）を`gym_quadruped`側の`get_base_inertia()`実装まで
   遡って確認する。** オンボーディング`05`とqpympc-study双方が独立に同じ疑問へ到達しており
   （A-6節）、`external/Quadruped-PyMPC`の外（`gym_quadruped`パッケージ）を読む必要がある点で
   単独では解決しないが、2系統が同じ疑問に到達したことは検証価値の高さを示す。

5. **`qpympc-study/07`のコスト重み表（`Q_position=[0,0,1500]`等）と`config.py`の実際の値を
   照合する。** 上記1のOCPエントリ作成と同時にやると効率的。
