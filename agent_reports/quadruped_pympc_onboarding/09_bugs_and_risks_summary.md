# バグ・危険箇所 総まとめ(read_code_01〜20 横断)

## この文書の役割

`read_code_01`〜`read_code_20`(`simulation.py`経路 + ROS2経路)を1ファイルずつ
逐次解説する過程で見つかった、実装上の問題点・バグ・危険箇所だけを1か所に
集約した文書です。各項目は該当する`read_code_NN`ファイルの該当箇所を
直接読んだ上でのコード上の事実として記載しています(推測にとどまるものは
「推測」「未確認」と明記)。詳細な行番号付きコード引用・前後の文脈は、
リンク先の`read_code_NN`ファイルを参照してください。

深刻度の目安:

- 🔴 **高**:既定設定で実際に動く経路に影響する、または実機投入時に安全上の
  問題になりうるもの
- 🟡 **中**:既定では無害だが、設定を変える・機能を有効化すると問題になるもの、
  または実機投入時にのみ顕在化するもの
- ⚪ **低**:デッドコード・未使用変数・docstringの不一致など、動作への
  直接的な影響が無いもの

---

## 1. 🔴 安全性・実機投入時に影響しうるもの

### 1.1 後退方向の速度指令で脚の伸びきりチェックが素通りする

```python
if(ref_base_lin_vel[0] < 0.01 and ref_base_lin_vel[1] < 0.01):
    return ref_base_lin_vel, ref_base_ang_vel
```

- ファイル:`helpers/velocity_modulator.py`(`VelocityModulator.__call__`)
- `abs()`が付いていないため、後退方向(負の値、例`-2.0`)の目標速度は
  常にこの条件を満たしてしまい、この関数本来の目的である「股関節からの
  水平距離が閾値を超えたら速度指令を強制的にゼロにする」安全チェックが
  **後退方向では一切実行されません**。前進方向でのみ意図通りに機能する、
  左右非対称な安全機構になっています(コードから論理的に導いた指摘。
  実際に後退動作を試して確認したものではない)
- 詳細:[read_code_05](read_code_05_velocity_modulator.md)

### 1.2 `kinodynamic`タイプへ切り替えると`UnboundLocalError`で例外になる可能性が高い

```python
if cfg.mpc_params['type'] != 'kinodynamic':
    ...
    ref_state = ClassOrDictHoldingRefState(...)
...
return state_current, ref_state, contact_sequence, self.step_height, optimize_swing
```

- ファイル:`interfaces/wb_interface.py`(`WBInterface.update_state_and_reference`)
- `ref_state`は`type != 'kinodynamic'`の`if`ブロック内でしか定義されないが、
  関数末尾の`return`は`ref_state`を無条件に参照する。既定の`'nominal'`
  では問題ないが、`mpc_params['type']`を`'kinodynamic'`に変更すると、
  この関数がここで例外を送出して落ちるはずです(コードから導いた指摘、
  実際に`kinodynamic`を指定して実行確認したものではない)
- 詳細:[read_code_06](read_code_06_wb_interface_update_state_and_reference.md)

### 1.3 `get_base_state_callback`のコメントが示す角速度の座標系変換が未実装

```python
# For the angular velocity, mujoco is in the base frame, and DLS2 is in the world frame
self.angular_velocity = np.array(msg.velocity.angular) 
```

- ファイル:`ros2/run_controller.py`(`Quadruped_PyMPC_Node.get_base_state_callback`)
- コメントは「MuJoCoはbase座標系、DLS2はworld座標系」という座標系の違いを
  明記しているが、実際のコードは変換を一切行わず受信値をそのまま
  `self.angular_velocity`に代入し、そのまま`env.mjData.qvel[3:6]`
  (MuJoCo側はbase座標系として扱う場所)へ書き込みます。もし実機側の
  publisherが本当にworld座標系で角速度を送っているなら、姿勢によっては
  MPCへ渡る角速度が実際の値と乖離します(実機側のpublisher実装は対象
  リポジトリに無いため**未確認**)
- 詳細:[read_code_19](read_code_19_ros2_controller_callbacks.md)

### 1.4 `compute_control_callback`の安全チェックが`and`条件になっている

```python
if(self.first_message_base_arrived==False and self.first_message_joints_arrived==False):
    return
```

- ファイル:`ros2/run_controller.py`(`Quadruped_PyMPC_Node.compute_control_callback`)
- 「両方とも届いていない場合のみreturn」という条件のため、片方のトピック
  だけが届いている状態でも処理が続行されます。本来は`or`にすべき箇所と
  考えられます(既定では`get_blind_state_callback`からしか呼ばれないため
  実害は限定的)
- 詳細:[read_code_19](read_code_19_ros2_controller_callbacks.md)

### 1.5 `com_pos[2] = robot_height`が開発者自身により「バグ」と明記され無効化中

```python
#base_pos[2] = robot_height
#com_pos[2] = robot_height #TODO, this is an error
```

- ファイル:`interfaces/wb_interface.py`(`WBInterface.update_state_and_reference`)
- 現状はコメントアウトされ無効なので実害は無いが、`#TODO, this is an
  error`という開発者自身のコメントが残ったまま放置されている。将来
  誰かがこの行を安易に有効化すると、地形推定由来の`robot_height`で
  胴体・重心のZ座標を強制上書きするバグが再発する
- 詳細:[read_code_06](read_code_06_wb_interface_update_state_and_reference.md)

---

## 2. 🟡 既定では無効・無害だが、有効化すると壊れるもの

### 2.1 共有メモリMPC変種でのコピー&ペーストバグ(速度・加速度に位置の値が入る)

```python
arr[IDX_JP]   = (nmpc_joints_pos if nmpc_predicted_state is not None else ...)
arr[IDX_JV]   = (nmpc_joints_pos if nmpc_predicted_state is not None else ...)  # ← nmpc_joints_vel であるべき
arr[IDX_JA]   = (nmpc_joints_pos if nmpc_predicted_state is not None else ...)  # ← nmpc_joints_acc であるべき
```

- ファイル:`ros2/run_controller.py`
  (`Quadruped_PyMPC_Node.compute_mpc_process_shared_memory_callback`)
- `USE_PROCESS_SHARED_MEMORY_MPC=False`が既定のため無害だが、有効化すると
  共有メモリ経由で読み出される関節速度・加速度が常に関節角度の値になる
- 詳細:[read_code_19](read_code_19_ros2_controller_callbacks.md)

### 2.2 `kinodynamic`分岐でのコピー&ペーストバグ

```python
des_joints_pos = nmpc_joints_pos
des_joints_pos = nmpc_joints_vel  # ← des_joints_vel であるべき
```

- ファイル:`interfaces/wb_interface.py`(`WBInterface.compute_stance_and_swing_torque`)
- 既定の`'nominal'`型では到達しない分岐(`kinodynamic`専用)。もし
  `kinodynamic`型を使うと、`des_joints_vel`が設定されないまま
  `des_joints_pos`が2回代入される
- 詳細:[read_code_12](read_code_12_wb_interface_torque.md)

### 2.3 `EarlyStanceDetector`:`sampling`型向けの無効化が直後で上書きされる(開発者が`# TO FIX`と自認)

```python
if(cfg.mpc_params['type'] == 'sampling'):
    self.activated = False # TO FIX
self.trigger_mode = cfg.simulation_params['reflex_trigger_mode']
if(self.trigger_mode == False):
    self.activated = False
else:
    self.activated = True
```

- ファイル:`helpers/early_stance_detector.py`(`EarlyStanceDetector.__init__`)
- `trigger_mode`が既定`'tracking'`(truthy)である限り、`sampling`型で
  無効化したはずの`self.activated`が直後に無条件で`True`に再設定される。
  既定の`'nominal'`型では17行目の`if`自体に入らないため、標準経路には
  影響しない
- 詳細:[read_code_14](read_code_14_early_stance_detector.md)

### 2.4 `set_stage_constraint`は既定では一度も呼ばれない(かつ、呼ばれても大半が捨てられる)

- ファイル:`controllers/gradient/nominal/centroidal_nmpc_nominal.py`
- `set_warm_start`・`set_stage_constraint`の呼び出し条件フラグ
  (`use_warm_start`・`use_foothold_constraints`・`use_stability_constraints`)
  が既定すべて`False`のため、既定のトロット歩行では
  `set_stage_constraint`(486行の関数)自体が**そもそも呼ばれません**
- 呼ばれた場合でも、関数内の接地中box・着地予定box制約の計算
  (約280行、4脚×3パターンの重複コード)は`use_foothold_constraints=False`
  だと計算されるだけで最終的に捨てられ、`if j == self.horizon:`という
  分岐は`for j in range(0, self.horizon)`の範囲外を指すため**構造的に
  到達不能**、さらに関数全体を覆う`bare except:`が`verbose=False`
  (既定)のとき例外を完全に無音化する
- 詳細:[read_code_10](read_code_10_set_stage_constraint.md)、
  [read_code_11](read_code_11_compute_control.md)

### 2.5 `stance_proximity`(着地間近の着地点最適化抑制)が常に無効化されている

```python
stance_proximity_FL[j] = 1 * 0  # 常に 0
```

- ファイル:`controllers/gradient/nominal/centroidal_nmpc_nominal.py`
  (`compute_control`)
- コメントが示唆する「接地間近では着地点最適化を弱める」機能が、`1*0`と
  いう式によって実質的に常にゼロへ固定されており、機能していない
- 詳細:[read_code_11](read_code_11_compute_control.md)

### 2.6 `external_wrenches_compensation`は既定ONだが、実際の外力推定値が供給されず空回り

- ファイル:`controllers/gradient/nominal/centroidal_nmpc_nominal.py`
  (`compute_control`)
- `external_wrenches_compensation=True`(既定)だが、`external_wrenches`
  引数自体は常にゼロのデフォルト値のまま渡ってくるため、「外力を
  補償する」機能は現状常に「ゼロを補償する」だけの空回りになっている
- 詳細:[read_code_11](read_code_11_compute_control.md)

### 2.7 ソルバー失敗時のフォールバックGRF計算が直後に上書きされて捨てられる

- ファイル:`controllers/gradient/nominal/centroidal_nmpc_nominal.py`
  (`compute_control`)
- ソルバー失敗(`status in {1,4}`)時、体重按分で計算する丁寧なフォールバック
  GRFが用意されているが、直後に`self.previous_optimal_GRF`で上書きされ、
  実際に使われるのは「前回成功したGRFをそのまま使い続ける」という
  より単純な経路
- 詳細:[read_code_11](read_code_11_compute_control.md)

### 2.8 `console.py`:クロール歩容のメニュー表示と実際の動作が一致しない

```python
elif(gait_type == 3): gait_name = "crawl"  # メニュー表示は CIRCULARCRAWL
elif(gait_type == 4): gait_name = "crawl"  # メニュー表示は BFDIAGONALCRAWL
elif(gait_type == 5): gait_name = "crawl"  # メニュー表示は BACKDIAGONALCRAWL
elif(gait_type == 6): gait_name = "crawl"  # メニュー表示は FRONTDIAGONALCRAWL
```

- ファイル:`ros2/console.py`(`Console.interactive_command_line`、
  `"setGaitTimer"`コマンド)
- メニューは4種類の異なるクロール歩容を提示するが、`config.py`の
  `gait_params`辞書には`'crawl'`エントリが1つ(実体は
  `GaitType.BACKDIAGONALCRAWL`)しか無いため、`3`〜`6`のどれを選んでも
  実際に適用される歩容タイプは常に同じ(`BACKDIAGONALCRAWL`)になる
- 詳細:[read_code_20](read_code_20_ros2_console.md)

---

## 3. 🟡 命名と実際の中身が食い違っているもの(データの誤読につながりうる)

### 3.1 `simulation.py`の`joints_pos`が実は「関節角度」ではなく「インデックス配列」

```python
joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)
```

- ファイル:`simulation/simulation.py`(`run_simulation`)
- 変数名`joints_pos`からは関節角度の値を想像するが、実際に代入されて
  いるのは`legs_qvel_idx`(`qvel`配列上のインデックス)である。この値は
  そのまま`WBInterface`の状態辞書へ`state_current['joint_FL']`として
  格納される。MPCの状態ベクトル自体には関節角度に相当する成分が
  見当たらないため、この値が実際にOCPの計算で数値として使われているか
  どうかは疑わしく、使われていない可能性がある(変換処理の中身までは
  追い切れておらず**未確認**)。**この経路のROS2版
  (`run_controller.py`)では、同じ変数名`joints_pos`に実際の関節角度
  スライス`qpos[7:10]`等が使われており、この問題は存在しない**
  (read_code_19で確認済み)。つまり同じ変数名でも`simulation.py`経路と
  ROS2経路とで中身の意味が異なる
- 詳細:[read_code_01](read_code_01_simulation_py.md)、
  [read_code_19](read_code_19_ros2_controller_callbacks.md)

### 3.2 `phase_signal`が内部のミュータブル配列をコピーせずに公開される

```python
data = {'phase_signal': self.wb_interface.pgg._phase_signal}
```

- ファイル:`quadruped_pympc/quadruped_pympc_wrapper.py`
  (`QuadrupedPyMPC_Wrapper.get_obs`が呼ぶ観測辞書組み立て処理)
- `PeriodicGaitGenerator`には`phase_signal`という公開プロパティ
  (`return np.array(self._phase_signal)`、コピーを返す)が別に用意
  されているが、ここではそれを使わず、先頭に`_`が付いた内部属性
  `_phase_signal`(コピーではない、生の配列そのもの)を直接辞書に
  入れている。この配列は毎周期`pgg.run()`の中でin-placeに書き換え
  られるため、`get_obs()`の呼び出し元がこの値を保持し続けると、
  知らないうちに中身が後から変わってしまう可能性がある
- 詳細:[NN_quadruped_pympc_wrapper_walkthrough](NN_quadruped_pympc_wrapper_walkthrough.md)

### 3.3 `run_simulator.py`が送らないフィールドを`run_controller.py`が受け取る前提になっている

- ファイル:`ros2/run_simulator.py`(`BlindState`のpublish)・
  `ros2/run_controller.py`(`get_blind_state_callback`)
- `run_simulator.py`は`BlindState.msg`の`feet_contact`フィールドに
  一度も値を代入していないが、`run_controller.py`側は
  `self.feet_contact = np.array(msg.feet_contact)`として無条件に
  受け取る。送信側が値を入れていないため、`self.feet_contact`は常に
  未初期化(空配列またはデフォルト値)のまま制御ロジックに渡る
- 詳細:[read_code_17](read_code_17_ros2_run_simulator.md)、
  [read_code_19](read_code_19_ros2_controller_callbacks.md)

---

## 4. ⚪ Python言語的な落とし穴:可変オブジェクトのデフォルト引数

同じパターンのバグが、独立した2箇所に存在します。Pythonの関数デフォルト
引数は**関数定義時(モジュール読み込み時)に1度だけ評価**されるため、
可変オブジェクト(numpy配列など)をデフォルト値にすると、複数回の呼び出しで
**同じオブジェクトが使い回されます**。現状はどちらもその引数を関数内で
書き換えるコードが無いため実害はありませんが、将来in-placeに変更する
コードが追加されると、意図しない副作用(前回呼び出しの値が次回に
漏れ出す)を生む典型的な落とし穴です。

| ファイル | 関数 | 該当引数 |
|---|---|---|
| `interfaces/srbd_controller_interface.py` | `SRBDControllerInterface.compute_control` | `external_wrenches=np.zeros((6,))` |
| `controllers/gradient/nominal/centroidal_nmpc_nominal.py` | `Acados_NMPC_Nominal.compute_control` | `external_wrenches=np.zeros((6,))`、`inertia=config.inertia.reshape((9,))` |

詳細:[read_code_07](read_code_07_srbd_controller_interface.md)、
[read_code_11](read_code_11_compute_control.md)

---

## 5. ⚪ 未使用変数・デッドコード一覧

| ファイル | 該当箇所 | 内容 |
|---|---|---|
| `simulation/simulation.py` | 冒頭の設定読み込み | `robot_leg_joints`・`robot_feet_geom_names`を取得するが、このファイル内では一度も参照されない |
| `simulation/simulation.py` | `run_simulation`冒頭 | `state_obs_history`・`ctrl_state_history`は`recording_path=None`の既定実行では最後まで使われない |
| `quadruped_pympc_wrapper.py` | `compute_actions` | `kp_joint_motor`・`kd_joint_motor`を取得するが、それを使う`for`ループが丸ごとコメントアウトされ未使用 |
| `helpers/periodic_gait_generator.py` | `PeriodicGaitGenerator` | `time_before_switch_freq`が初期化されるだけで本ファイル内では未使用 |
| `helpers/foothold_reference_generator.py` | `FootholdReferenceGenerator` | `touch_down_positions`は毎周期計算されるが、`compute_footholds_reference`の中では一度も読まれない |
| `controllers/gradient/nominal/centroidal_model_nominal.py` | `Centroidal_Model_Nominal.__init__` | `omega_x_integral`/`omega_y_integral`/`omega_z_integral`シンボルが定義されるが状態ベクトル(`self.states`)に組み込まれない |
| `controllers/gradient/nominal/centroidal_nmpc_nominal.py` | `set_stage_constraint` | `stance_proximity`引数が受け取られるだけで一度も使われない |
| `controllers/gradient/nominal/centroidal_nmpc_nominal.py` | `compute_control` | `FL/FR/RL/RR_previous_contact_sequence`が代入されるだけで一度も使われない |
| `helpers/swing_trajectory_controller.py` | `compute_swing_control_cartesian_space` | `passive_force`引数が受け取られるだけで使われない |
| `helpers/swing_trajectory_controller.py` | `compute_swing_control_joint_space` | 本シリーズで読んだ範囲では呼び出し元が見つからない(未使用の疑い) |
| `ros2/run_controller.py` | `Quadruped_PyMPC_Node.__init__` | `self.feet_traj_geom_ids`・`self.feet_GRF_geom_ids`(デバッグ描画用)が宣言されるだけでどこにも使われない |
| `ros2/run_controller.py` | 冒頭のimport | `SRBDBatchedControllerInterface`がimportされるが一度もインスタンス化されない |
| `ros2/run_simulator.py` | モジュール定数 | `USE_SCHEDULER=True`が定義されるが、このファイル内では一度も参照されない(死んだ変数) |
| `ros2/run_simulator.py` | `get_trajectory_generator_callback` | 受信した`joints_position`を`self.desired_joints_position`へ保存するが、後段のどこからも読まれない |

---

## 6. ⚪ docstring・コメントと実装の不一致(動作には影響しないが誤解を招く)

| ファイル | 該当関数 | 不一致の内容 |
|---|---|---|
| `centroidal_model_nominal.py` | `forward_dynamics` | docstringのshape記載が4箇所すべて誤り(states 29→実際30、inputs 29→24、param 4→29、戻り値29→30) |
| `centroidal_nmpc_nominal.py` | `create_foothold_constraints` | docstringは`shape (8,1)`と書くが実際は12要素(4脚×3) |
| `helpers/terrain_estimator.py` | `compute_terrain_estimation` | docstringは戻り値2つ(roll, pitch)と書くが実際は4つ返す |
| `interfaces/srbd_controller_interface.py` | `SRBDControllerInterface`クラス | クラスdocstringは「gaitを最適化する」と書くが、実際に最適化するのはGRF |
| `interfaces/wb_interface.py` | `update_state_and_reference` | 戻り値の型注釈は8要素だが、実際の`return`文は5要素 |

---

## 7. ⚪ その他の構造的な違和感(実害は小さいか未確認)

- `centroidal_nmpc_nominal.py`の`R_foot_force`重み設定：
  `config.robot == "hyqreal"`という条件分岐があるが、`config.py`の
  実在するロボット名は`hyqreal1`/`hyqreal2`であり、この分岐は死んでいる
  可能性が高い(**推測**)。詳細:[read_code_09](read_code_09_centroidal_nmpc_nominal_setup.md)
- `PeriodicGaitGenerator.compute_contact_sequence`の`FULL_STANCE`分岐：
  戻り値のshapeが`(4, horizon*2)`になり、通常経路の`(4, horizon)`と
  食い違う。詳細:[read_code_02](read_code_02_periodic_gait_generator.md)
- `FRONTDIAGONALCRAWL`の`phase_offset=1.25`(他の歩容と違い`1.0`を超える)：
  `_init[leg]`フラグが永遠に`True`のまま固定される可能性がある
  (**推測、実行確認はしていない**)。詳細:[read_code_02](read_code_02_periodic_gait_generator.md)
- `helpers/terrain_estimator.py`:`roll_activated=False`により、計算された
  roll値が最終的に常に`0.0`へ捨てられる(無駄な計算)。このため
  read_code_06で見た通り、MPCの目標roll角は既定で常にちょうど`0`になる。
  詳細:[read_code_04](read_code_04_terrain_estimator.md)
- `helpers/swing_trajectory_controller.py`:
  `compute_swing_control_cartesian_space`で、PDフィードバック項が
  本体計算とフィードバック線形化の`accelleration`項の両方に現れ、
  実質二重に加算されている。詳細:[read_code_13](read_code_13_swing_trajectory_controller.md)
- `interfaces/wb_interface.py`の`reset`:`FootholdReferenceGenerator`・
  `SwingTrajectoryController`・`TerrainEstimator`のリセット呼び出しが
  コメントアウトされており、実質`PeriodicGaitGenerator`の位相と
  `current_contact`・`lift_off_positions`しかリセットされない。
  詳細:[read_code_12](read_code_12_wb_interface_torque.md)
- `helpers/foothold_reference_generator.py`:`lift_off_positions_h`/
  `touch_down_positions_h`の初期化に`# TODO wrong`という開発者自身の
  コメントが残ったまま。詳細:[read_code_03](read_code_03_foothold_reference_generator.md)
- `helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py`:
  誤差ベクトルに掛かる`100`倍のスケーリング根拠が不明、かつ収束判定
  なしの固定5回反復(誤差が残っても打ち切られる)。
  詳細:[read_code_15](read_code_15_inverse_kinematics.md)
- `ros2/run_controller.py`:プロセス優先度を上げる`sudo renice`/
  `autogroup`書き込みが失敗しても戻り値を確認しておらず、エラーが
  無視される。詳細:[read_code_18](read_code_18_ros2_controller_init.md)
- `ros2/run_controller.py`:`heightmaps`が設定値に関わらず常に`None`
  (`simulation.py`側にあった条件付き`HeightMap`構築が実装されていない)。
  既定設定(`visual_foothold_adaptation='blind'`)では挙動は一致するが、
  設定を変えてもROS2経路ではハイトマップ機能が有効化できない。
  詳細:[read_code_19](read_code_19_ros2_controller_callbacks.md)
- `ros2/run_controller.py`:`time_debug_msg.time_mpc`は既定の同期MPC
  分岐では常に`0.0`のまま送信され続ける(非同期MPC変種でのみ実測される)。
  詳細:[read_code_19](read_code_19_ros2_controller_callbacks.md)
- `ros2/console.py`:`"narrowStance"`/`"wideStance"`(`hip_offset`の増減)、
  `"ictp"`のキーボード速度操作は、どちらも上限・下限のクランプが無く
  繰り返し操作で際限なく値が変化しうる(`"setupGaitTimer"`等の数値入力
  コマンドにはクランプがあるのと対照的)。詳細:[read_code_20](read_code_20_ros2_console.md)

---

## 深刻度別 件数まとめ

| 深刻度 | 件数(節1〜3の項目数) |
|---|---:|
| 🔴 高 | 5 |
| 🟡 中 | 11 |
| ⚪ 低(節4〜7、docstring・デッドコード等) | 25以上 |

体系的な傾向として、このリポジトリには「コードは書かれているが、既定設定
では通らない・値が使われない・上書きされて捨てられる」という**静かに
無効化された機能**が非常に多く見つかりました。既定のトロット歩行で実際に
動いている経路は、コードの見た目の複雑さに対してかなりシンプルです
(摩擦錐制約のみ有効、warm start無し、着地点/安定性制約無し、MPC並列化無し、
VFA無し、というのが実質的な既定構成)。
