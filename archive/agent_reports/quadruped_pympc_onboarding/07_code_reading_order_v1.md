# 07 — コードを読む順序(実装ベース、理論理解ロードマップ)

日付: 2026-08-26
対象: `external/Quadruped-PyMPC`(`mpc_params['type'] = 'nominal'`)
位置づけ: **本ファイルの主題はソースコードそのものである。** `docs/qpympc-study/`、
`docs/pympc_2day/`、本フォルダ`06_existing_docs_synthesis.md`は参照(裏付け・補足)としてのみ
使う。`06`は逆に「既存ドキュメントの整理」が主題になっていた誤りがあり、本ファイルで置き換える。

凡例:
- **事実(本パス)**: 本ファイル作成時に自分で該当ソースを直接読んで確認した内容(行番号を付す)。
- **事実(既存)**: 本フォルダ`01`–`05`が既に直接コードを読んで確認済みの内容。
- **未読**: 本パスでは読んでいない関数・ファイル。次に読むべき箇所として明記する。

---

## 読む順序(10ステップ、依存関係の根拠つき)

各ステップは「前のステップの出力が次のステップの入力になる」という実装上の直接的な変数の
受け渡しで並んでいる。根拠は全てソースの変数名・行番号で示す。

### 1. `simulation/simulation.py::run_simulation`(1–365行、全読了)

**内容(事実・本パス)**:
- `QuadrupedEnv`生成(55–64行)。速度指令は`base_vel_command_type`引数(62行、既定`"human"`、
  36行)で`QuadrupedEnv`コンストラクタへ直接渡される。
- 毎周期(171–336行のループ)、環境から観測を取り出す(173–205行):
  `feet_pos`, `feet_vel`, `hip_pos`, `base_lin_vel`, `base_ang_vel`, `base_ori_euler_xyz`,
  `base_pos`, `com_pos`, `ref_base_lin_vel/ref_base_ang_vel = env.target_base_vel()`(183行),
  `inertia`(`use_inertia_recomputation`なら`env.get_base_inertia()`、そうでなければ`config.inertia`、
  186–189行), `qpos/qvel`, `legs_qvel_idx/legs_qpos_idx`, `legs_mass_matrix`, `legs_qfrc_bias`,
  `legs_qfrc_passive`, `feet_jac/feet_jac_dot`(204–205行)。
- `quadrupedpympc_wrapper.compute_actions(...)`を呼び`tau`を得る(208–236行)。
- `tau`を`0.9 * actuator_ctrlrange`でクリップ(83, 238–240行)、`action`配列へ
  `legs_tau_idx`でleg毎に詰めて`env.step(action)`(243–251行)。

**訂正(本パスでの発見)**: `AGENTS.md`のCanonical execution path表2行目は速度指令の経路を
`simulation_params['mode']` としているが、`run_simulation()`本文のどこにも
`simulation_params["mode"]`という参照はない(grep済み、ヒット0件)。実際の経路は
上記の`base_vel_command_type`引数と`env.target_base_vel()`である。これは
`docs/qpympc-study/03_User_Command_and_Reference_Generation.md`の記述と一致し、
本パスで直接コードを読んで**再現・確認した**(qpympc-study側は`gym_quadruped`内部の
`_sample_ref_vel()`まで踏み込んでいるが、その部分は`external/Quadruped-PyMPC`の外なので
本パスでは未読)。`AGENTS.md`は次回更新が必要。

**この順で読む理由**: このループが生成する変数名・単位・frame(world/base)が、以降すべての
ステップの入力の正本になる。ここを読まずに他へ進むと、たとえば`base_ang_vel`が
base frame(177行、`frame="base"`)なのに`base_lin_vel`はworld frame(176行、`frame="world"`)
という非対称性(2フレーム混在)に気づけない。

### 2. `quadruped_pympc/config.py` (随時参照。今回は主要キーの使用箇所からの逆引きのみ、全文通読はしていない)

**内容(事実・本パス、他ファイルの参照から確認)**: `mpc_params`(`type`, `dt`, `horizon`,
`optimize_step_freq`, `use_foothold_optimization`, `use_RTI`, `use_DDP`, `solver_mode`,
`grf_max`, `grf_min`, `num_qp_iterations`, `as_rti_type`, `use_nonuniform_discretization`等)と
`simulation_params`(`dt`, `mpc_frequency`, `gait`, `gait_params`, `scene`,
`visual_foothold_adaptation`, `ref_z`, `step_height`, `swing_position_gain_fb`,
`swing_velocity_gain_fb`, `use_inertia_recomputation`, `impedence_joint_position_gain/velocity_gain`
等)の2辞書である点は`AGENTS.md`と一致(**事実・既存**、本パスでも各使用箇所から再確認)。

**この順で読む理由**: 独立した1ステップとしてではなく、3–9のどのファイルを読んでいても
キー名が出てきたら都度`config.py`へ戻って値を確認する、という使い方が実際的(このファイル単体には
制御フローが無く、辞書リテラルの塊であるため)。

### 3. `quadruped_pympc/helpers/periodic_gait_generator.py::PeriodicGaitGenerator` (1–197行、全読了)

**内容(事実・本パス)**:
- `run(dt, new_step_freq)`(48–76行): 各脚の位相を`phase[leg] += dt*step_freq`で進め`%1.0`、
  `phase < duty_factor`なら接地(`contact[leg]=1`)。
- `compute_contact_sequence(...)`(93–118行): 現在の位相を保存(101–102行)→ホライズン分
  `run()`を先読みで繰り返し呼んで`contact_sequence`(4×horizon)を構築→元の位相へ復元
  (117行`set_phase_signal(t_init, init_init)`)。**副作用なしの先読み**であることをこの
  save/restore構造から直接確認した。
- trotの`phase_offset = [0.5, 1.0, 1.0, 0.5]`(25行、脚順は`FL,FR,RL,RR`)。

**事実(既存)との一致**: 本フォルダ`02_gait_and_contact_sequence_v3.md`が同じ結論(位相更新式、
`compute_contact_sequence`の一時進行・復元構造、`GaitType`全種の位相表)に到達済み。本パスの
再読はその内容と完全に一致した。

**この順で読む理由**: `contact_sequence`(特にその列0、`current_contact`)は、後段の
`centroidal_model_nominal.py::forward_dynamics`で**状態ではなくparam**(`stanceFL/FR/RL/RR`、
197–199行に対応、後述ステップ6参照)として力学方程式に直接乗算される。Gaitを先に理解しないと、
「なぜ`stance_i`が力学モデルの中で単純な乗数として現れるのか」を設計判断ではなく偶然の実装と
誤解する。

### 4. `quadruped_pympc/helpers/foothold_reference_generator.py::FootholdReferenceGenerator` (1–229行、全読了)

**内容(事実・本パス)**:
- `compute_footholds_reference(...)`(53–157行): world→horizontal frame回転(88–90行)、
  `delta_ref_H = (stance_time/2) * ref_base_lin_vel_H`を`±1.5*hip_height`でclip(103–104行)、
  速度誤差補正`error_compensation = sqrt(h/g)*(base_vel_mvg - ref_base_lin_vel_H)`を`±0.05`で
  clip(108–111行、符号は「実測平均−目標」)、hip位置基準に`hip_offset=0.1`固定オフセット
  (44, 126–129行)、Z成分は`lift_off_positions`からコピー(151行)。
- `update_lift_off_positions`/`update_touch_down_positions`(159–199行): 接地状態の
  0→1、1→0遷移を検出してworld/horizontal frame双方を更新。`touch_down_positions`は計算されるが、
  本ファイル内では他のどこからも読み出されていない(呼び出し元`wb_interface.py`側でも未使用、
  ステップ5で確認)。

**事実(既存)との一致**: 本フォルダ`03_foothold_reference_generation_v2.md`と数式レベルで完全一致
(clip範囲、符号、`hip_offset`ハードコード、`touch_down_positions`未消費、すべて本パスで再確認)。

**この順で読む理由**: ここが生成する`ref_feet_pos`が、ステップ5で`ref_state['ref_foot_FL'...]`
としてOCPの参照値になる(ステップ5・7参照)。また`update_lift_off_positions`が更新する
`lift_off_positions`は、ステップ6の力学モデルにおける遊脚状態(`stance_proximity`と並んで
`(1-stanceFL)`項、後述)の意味を理解する前提になる。Gait(ステップ3)の`current_contact`を
入力として使うため、ステップ3の後に読む必要がある(`update_lift_off_positions`の引数
`previous_contact, current_contact`は`wb_interface.py`でステップ3の出力から作られる、ステップ5参照)。

### 5. `quadruped_pympc/interfaces/wb_interface.py::WBInterface.update_state_and_reference` (108–305行、全読了)

**内容(事実・本パス)**:
- `self.pgg.run(...)` → `self.pgg.compute_contact_sequence(...)`(202–205行、ステップ3の呼び出し)。
- `self.current_contact = contact_sequence[:,0]`(207–210行、ステップ3出力の列0を取り出し)。
- `self.frg.update_lift_off_positions(...)`/`update_touch_down_positions(...)`
  (213–230行、ステップ4呼び出し、`previous_contact`/`current_contact`を渡す)。
- `self.frg.compute_footholds_reference(...)`(231–238行、ステップ4呼び出し)。
- `state_current`辞書組み立て(163–177行): `position=com_pos+com_pos_offset_w`(CoM位置、
  base位置ではない)、`linear_velocity=base_lin_vel`(**base速度**、CoM速度ではない — 非対称)、
  `orientation=base_ori_euler_xyz`、`angular_velocity=base_ang_vel`、`foot_FL..RR=feet_pos.FL..RR`、
  `joint_FL..RR=joints_pos.FL..RR`。ここで**`joints_pos`の中身は`simulation.py`側で
  `legs_qvel_idx`(インデックス配列)が渡されている**(`run_simulation`196行
  `joints_pos = LegsAttr(FL=legs_qvel_idx.FL, ...)`)ため、`state_current['joint_FL']`は
  関節角度ではなくインデックス配列である(本パスで`simulation.py`とこのファイルを
  突き合わせて直接確認)。
- `ref_state`辞書組み立て(279–294行): `ref_foot_FL..RR`(ステップ4の出力を`(1,3)`に整形)、
  `ref_linear_velocity`/`ref_angular_velocity`(地形roll/pitchで回転補正、262–268行)、
  `ref_orientation=[terrain_roll, terrain_pitch, 0]`、`ref_position`(z成分は
  `simulation_params['ref_z'] + terrain_height`から更に`base_pos[2]-(com_pos[2]+offset)`を
  引いてbase高さ指令をCoM座標系へ変換、271–275行)。

**事実(既存)との一致**: 本フォルダ`04_state_and_reference_assembly_v2.md`の指摘
(`joints_pos`がインデックス配列である点、`position`=CoM/`linear_velocity`=base速度の非対称性、
`TerrainEstimator`が`lift_off_positions`を使い実測`feet_pos`を使わない点)を本パスで再確認した。

**この順で読む理由**: ステップ1(観測)・3(gait)・4(foothold)の出力をすべて集約し、
ステップ6・7のOCPが直接受け取る`state_current`/`ref_state`/`contact_sequence`という3点セットへ
変換する層。ここを読まずにOCP(ステップ6・7)へ進むと、辞書のキー名と実際の物理量
(CoM vs base、インデックス vs 角度)の対応を誤る。

### 6. `quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py::Centroidal_Model_Nominal` (1–340行、全読了)

**内容(事実・本パス、`cs.vertcat`の実引数を数えて次元を実測)**:
- 状態`self.states`(54–77行): 位置3+速度3+姿勢角3+角速度3+脚位置4×3=12+積分6
  = **30次元**(内訳: `com_position_z_integral`, `com_velocity_x/y/z_integral`,
  `roll_integral`, `pitch_integral`の6個)。
- `forward_dynamics`のdocstring(161, 163, 166行)は状態・入力とも`(29,)`と記載しているが、
  実際は状態30・入力24であり、**docstringは誤り**(本パスで直接確認。既存`05`の指摘と一致)。
- `self.inputs`(106–115行): 脚速度4×3+脚力4×3=**24次元**。
- `param`(141–150行): `stance_param(4) + mu_friction(1) + stance_proximity(4) + base_position(3)
  + base_yaw(1) + external_wrench(6) + inertia(9) + mass(1)` = **29次元**。
- `omega_x_integral, omega_y_integral, omega_z_integral`という3シンボルが35–52行で定義されるが、
  `self.states`のvertcat(54–77行)には含まれない — **死んだシンボル**であることを本パスで
  直接確認(既存`05`の指摘と一致)。
- `forward_dynamics`の物理(169–308行):
  - 並進: `linear_com_acc = (1/mass) * Σ(F_i * stance_i) + external_wrench_linear + gravity`
    (201–211行)。`stance_i`は0/1のparamで、**決定変数ではなくOCPに外から与えられる定数**。
  - 回転: body-frame オイラー運動方程式
    `angular_acc_base = inertia⁻¹ @ (b_R_w @ (Σ(skew(p_i-p_com)@F_i@stance_i) + wrench_angular)
    - skew(w) @ inertia @ w)`(226–272行)。
  - 脚位置: `linear_foot_vel_i = v_i @ (1-stance_i) @ (1-stance_proximity_i)`
    (283–286行) — 立脚中(`stance_i=1`)または`stance_proximity_i=1`(接地間近)のときは
    脚位置の時間微分を強制的に0にする(脚位置は立脚中は定数として扱われる)。さらに
    `use_foothold_optimization=False`のときは全脚で`foot_velocity`自体を0にする(278–282行)。

**事実(既存)との一致**: 本フォルダ`05_nominal_ocp_variables_and_dynamics.md`の結論
(状態30/入力24、docstring誤り、死んだシンボル、`stance_i`が力学のparamとして乗算される構造)を
本パスで独立に再現・確認した。強い相互検証。

**この順で読む理由**: ステップ5が組み立てた`state_current`/`ref_state`/`contact_sequence`が、
実際にどの状態変数・パラメータへ対応するかをここで固定する。ステップ7のコスト関数
(`Q_mat`の対角30個)・制約は、この状態・入力次元をそのまま前提に書かれているため、次元を
把握しないとステップ7の重み配列が何に対応するか読めない。

### 7. `quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py::Acados_NMPC_Nominal` (部分読了: 78–277行`create_ocp_solver_description`、430–553行`create_friction_cone_constraints`/`set_weight`。**未読**: 277–430行`create_stability_constraints`/`create_foothold_constraints`、562–1705行`set_stage_constraint`/`set_warm_start`/`perform_scaling`/`compute_control`)

**内容(事実・本パス)**:
- コスト: `LINEAR_LS`、`W = block_diag(Q_mat, R_mat)`、**`W_e = Q_mat`をそのまま終端コストに使う**
  (97行) — 別の終端専用重み`Q_N`は存在しない(97行を直接確認、既存`docs/qpympc-study`appendix E
  §20の訂正と一致)。
- `set_weight`の具体的な重み(504–522行、本パスで直接確認):
  `Q_position=[0,0,1500]`(x,yは0!)、`Q_velocity=[200,200,200]`、`Q_base_angle=[500,500,0]`
  (yawは0)、`Q_base_angle_rates=[20,20,50]`、`Q_foot_pos=[300,300,300]`(4脚共通)、
  積分項は`[50,10,10,10,10,10]`。`R_foot_vel=[1e-4,1e-4,1e-5]`、
  `R_foot_force=[1e-3,1e-3,1e-3]`(hyqrealのみ`1e-5`固定)。**x,y位置とyaw角のコストが0**、
  すなわち位置・向きの絶対誤差ではなく速度追従が主目的、という設計であることが重み配列の値から
  直接読み取れる。
- 摩擦錐(`create_friction_cone_constraints`、430–499行): Focchi論文の線形近似
  ("High-slope terrain locomotion..." 447–448行コメントに明記)、脚1本あたり5式×4脚=**20の
  不等式制約**、`f_min ≤ n·F ≤ f_max`と2方向のpyramid近似(450–472行)。`grf_min`/`grf_max`は
  `config.mpc_params`から(443–444行)。
- ソルバー(202–274行): `qp_solver="PARTIAL_CONDENSING_HPIPM"`、`hessian_approx="GAUSS_NEWTON"`、
  `integrator_type="ERK"`。`use_DDP`/`use_RTI`/それ以外(`SQP`、既定)で`nlp_solver_type`が分岐
  (206–239行)。`solver_mode`(`balance`/`robust`/`fast`/`crazy_speed`)で`hpipm_mode`と
  `qp_solver_iter_max`が変わる(242–251行)。
- `use_foothold_constraints`/`use_stability_constraints`フラグでソフト制約(スラック変数
  `zl/Zl/zu/Zu`、147–163行)として追加されるが、その中身の関数(`create_stability_constraints`,
  `create_foothold_constraints`)自体は**本パスでは未読**。

**未読部分についての誠実な申告**: `set_stage_constraint`(562–1047行、486行と大きい)・
`compute_control`(1138行以降)・`perform_scaling`(1116–1138行)は本パスでは中身を読んでいない。
これらは「`state_current`/`ref_state`辞書を実際にacadosの`x0`/`yref`/パラメータ配列へどう
詰め替えるか」「ソルバー失敗時にどうフォールバックするか」を扱う箇所であり、
`docs/qpympc-study/07_MPC_Formulation.md`・`09_MPC_Output_and_Receding_Horizon.md`が
これらに触れているが、**未検証(文書の記述のみ)**として扱うべき。次に読むべき最有力候補。

**この順で読む理由**: 力学モデル(ステップ6)の上に、最小化する目的関数と制約を載せる層。
重みの値(x,y,yawが0)を実際に読むことで初めて、「なぜ既定trotで横方向の位置が漂うのか」
「なぜyawの絶対値でなく角速度が効くのか」といったチューニング上の疑問に実装根拠を持って
答えられる。

### 8. `quadruped_pympc/interfaces/srbd_controller_interface.py::SRBDControllerInterface` (1–246行、全読了)

**内容(事実・本パス)**:
- `__init__`(10–83行): `mpc_params['type']`の値で`if/elif`分岐し、`nominal`→
  `Acados_NMPC_Nominal`、`input_rates`→`Acados_NMPC_InputRates`、`lyapunov`→
  `Acados_NMPC_Lyapunov`、`kinodynamic`→`Acados_NMPC_KinoDynamic`、`sampling`→`Sampling_MPC`
  (JAX)をインポートして`self.controller`に格納。`optimize_step_freq=True`なら追加で
  `Acados_NMPC_GaitAdaptive`を`self.batched_controller`として保持。
- `compute_control`(85–240行): `current_contact = contact_sequence[:,0]`(113–115行、
  ステップ5の`contact_sequence`列0を再度取り出し)。`sampling`かそれ以外かで分岐し、最終的に
  `self.controller.compute_control(...)`(未読の内部)から`nmpc_GRFs`(12,)・`nmpc_footholds`
  ・`nmpc_predicted_state`を得る。
- **Contact mask(225–230行、本パスで直接確認)**:
  ```python
  nmpc_GRFs = LegsAttr(
      FL=nmpc_GRFs[0:3] * current_contact[0],
      FR=nmpc_GRFs[3:6] * current_contact[1],
      RL=nmpc_GRFs[6:9] * current_contact[2],
      RR=nmpc_GRFs[9:12] * current_contact[3],
  )
  ```
  OCPが返す先頭段のGRF解を、**現在の接地フラグでもう一度マスクしてから返す**。摩擦錐制約
  (ステップ7)はOCP内で全脚常時有効だが、遊脚のGRFはこのマスクで強制的にゼロへ落とされる、
  という「3段構え」(力学Gate→摩擦錐は全脚→出力Mask)であることを、本パスでステップ6・7・8を
  通しで読むことで自分で追跡できた。

**この順で読む理由**: ステップ7のOCPはホライズン全体の解を返すが、実際に使うのは先頭段
(receding horizon)だけであり、かつその先頭段のGRFがステップ3の`current_contact`で
もう一段フィルタされる。この関係を理解しないと、ステップ9で受け取る`nmpc_GRFs`が
「OCPの生の出力」だと誤解する。

### 9. `quadruped_pympc/interfaces/wb_interface.py::WBInterface.compute_stance_and_swing_torque` (307–470行、全読了)

**内容(事実・本パス)**:
- **立脚トルク(372–375行、本パスで直接確認)**:
  ```python
  tau.FL = -np.matmul(feet_jac.FL[:, legs_qvel_idx.FL].T, nmpc_GRFs.FL)
  ```
  すなわち `τ_stance = -Jᵀ F`。**PD項は無い**。
- **訂正(本パスでの発見)**: `docs/pympc_2day/WORKSHOP.md` §5.4は立脚トルクを
  `τ = J⊤F + PD`と記載しているが、これは符号(`-`が抜けている)とPD項の有無の両方で
  実装と食い違う。`WORKSHOP.md`は教育用の簡略化資料であり(`§2.2`でacados内部編集を
  非目標と明記)、この記述は誤記または過度な単純化と判断できる。`AGENTS.md`・
  `docs/qpympc-study/10_Stance_and_Swing_Control.md`の`τ=-Jᵀf`という記述が正しい
  (本パスで実コードにより確定)。
- 遊脚(384–408行): `current_contact[leg_id]==0`のときだけ
  `self.stc.compute_swing_control_cartesian_space(...)`(**未読**、
  `helpers/swing_trajectory_controller.py`内)を呼びトルクと目標足位置・速度を得る。
  立脚中は`des_foot_pos[leg]=nmpc_footholds[leg]`(406行)。
- 摩擦補償(413–418行): `stc.use_friction_compensation`が真なら**全脚**(立脚・遊脚問わず)
  `tau -= legs_qfrc_passive`。
- 逆運動学(424–441行): `self.ik.compute_solution(...)`(**未読**、
  `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py`内)で
  `des_foot_pos`から`des_joints_pos`を計算、`des_joints_vel`は`pinv(J) @ des_foot_vel`。
- 飽和(448–468行): 目標関節位置・速度を実測との差分で`±3.0`/`±10.0`にクリップ。

**この順で読む理由**: ステップ8で得たマスク後GRFと着地点を、実際にロボットを動かす
トルクへ変換する最終段。ステップ7・8で「MPCは力(GRF)を最適化し、receding horizonで
1段だけ取り出す」ことを先に理解していないと、このステップで「MPCの出力=最終トルク」と
混同する。

### 10. `simulation/simulation.py`(ステップ1と同一ファイル、237–251行)

**内容(事実・本パス、ステップ1で既読)**: `tau`を`tau_limits`(`0.9*actuator_ctrlrange`)で
クリップ→`action`配列へ`legs_tau_idx`で詰める→`env.step(action)`でMuJoCo物理を1ステップ進める
→ループ先頭(ステップ1)へ戻る。

**この順で読む理由**: 制御器の出力(トルク)が実際に物理エンジンへ渡り、次周期の観測
(ステップ1)を生む閉ループの最終リンク。ステップ1–9を「1回の制御周期の中の一方向の処理」として
読んだ後、ここで初めて「これが毎周期(`mpc_frequency=100Hz`相当、5 simステップおき)繰り返される
閉ループである」ことが実装として完結する。

---

## この順序全体を貫く2つの依存関係(本パスで実コードを跨いで確認)

1. **`contact_sequence`(ステップ3の出力)は、ステップ6の力学モデルに決定変数ではなく
   paramとして入る。** `centroidal_model_nominal.py`の`forward_dynamics`で
   `stanceFL = param[0]`(185行)という形で読まれ、力・トルクに単純に乗算される(205–208, 226–229行)。
   Gaitを先に読まずに力学モデルを読むと、この乗数を設計意図ではなく実装の偶然と誤解しやすい。
2. **OCP(ステップ7)の出力は、そのまま使われるのではなく2段階でフィルタされる。**
   (a) receding horizon: ホライズン全体ではなく先頭段のみ(ステップ8、`compute_control`内部・未読)。
   (b) 接地マスク: `srbd_controller_interface.py`225–230行で`current_contact`を再度掛ける
   (ステップ8で本パスにより直接確認)。この2段を知らずにステップ9のトルク式を読むと、
   「MPCが出したGRF=そのままトルクに使われる力」と誤解する。

---

## 本パスで解決した、既存資料間の食い違い

`06_existing_docs_synthesis.md`が指摘だけに留めていた3件のうち、2件は本パスの直接コード読解で
確定できた。

| 争点 | 06での状態 | 本パスでの確定 |
|---|---|---|
| 速度指令の経路 | `AGENTS.md`(`simulation_params['mode']`)と`qpympc-study/03`(`base_vel_command_type`)が対立 | **`qpympc-study/03`が正しい**。`run_simulation()`に`simulation_params["mode"]`という参照は無い(grep 0件)。`AGENTS.md`は要修正 |
| 立脚トルクの式 | `pympc_2day/WORKSHOP.md`(`τ=J⊤F+PD`)と`AGENTS.md`/`qpympc-study/10`(`τ=-JᵀF`)が対立 | **`AGENTS.md`/`qpympc-study/10`が正しい**。`wb_interface.py`372–375行に`-np.matmul(...)`とPD項なしを直接確認 |
| `inertia`パラメータのframe(world/body) | オンボーディング`05`とqpympc-study双方が同じ疑問を提起、未解決 | **本パスでも未解決**。`simulation.py`186–189行のコメントは"world frame"だが、`centroidal_model_nominal.py`226–272行の使われ方はbody-frameオイラー方程式の形。`gym_quadruped.get_base_inertia()`(外部パッケージ、未読)まで遡る必要がある |

---

## 次に読むべき箇所(優先度順、実コードベース)

1. `centroidal_nmpc_nominal.py::compute_control`(1138行以降)と`set_stage_constraint`
   (562–1047行)。`state_current`/`ref_state`辞書が実際にどうacadosの`x0`/`p`/`yref`配列へ
   詰め替えられるかが、この順序の中で唯一「未読」のまま残っている変換点。
2. `centroidal_nmpc_nominal.py::create_stability_constraints`(277–384行)・
   `create_foothold_constraints`(384–430行)。既定で有効かどうか(`use_stability_constraints`/
   `use_foothold_constraints`のデフォルト値)を`config.py`で確認した上で読む。
3. `helpers/swing_trajectory_controller.py::compute_swing_control_cartesian_space`。
   遊脚トルクの中身(PDがどう二重に入るか、`docs/qpympc-study/10`が指摘する構造)は
   本パスでは未読。
4. `AGENTS.md`のCanonical execution pathステップ2の記述を、本パスの発見(表の食い違い)に
   基づいて修正する。
