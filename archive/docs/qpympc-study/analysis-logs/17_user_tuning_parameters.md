# Log 17: ユーザー調整パラメータ

対応プロンプト: フェーズ14。仕様固定値を除き、実際にチューニングする値をコードから抽出。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

標準: `type='nominal'`, `gait='trot'`, `optimize_step_freq=False`, `use_foothold_constraints=False`, `reflex_trigger_mode=False`。

除外（仕様）: `mass`, `inertia`, `hip_height` そのもの、関節 `ctrlrange`、リンク長、センサ。ただし `hip_height` から作る `ref_z` と `step_height` の係数は調整対象。

凡例（使用列）:

- 標準使用: 標準経路で効く
- 無効: フラグで分岐し標準では通らない
- 専用: 別 `type` / 非標準機能
- Dead: 読まれない、または設定しても効果が無い
- ハードコード: `config.py` にキーが無い

優先度: A Baseline成立 / B 性能 / C 不整地・実機 / D 研究。

---

## 1. パラメータ一覧

| 優先度 | レイヤー | パラメータ | 設定キー | ファイル | 既定値 | 単位 | 有効条件 | 主効果 | 副作用 | 使用 |
|---|---|---|---|---|---|---|---|---|---|---|
| A | 運用 | 高さ参照 | `simulation_params['ref_z']` | `config.py` | `0.28*1.08=0.3024` | m | 常時 | 胴体高さ目標、FRG誤差項の \(h\) | 高すぎると脚伸び・沈み | 標準使用 |
| A | 運用 | 目標速度 | 実行時指令（configなし） | `QuadrupedEnv._ref_*` | human初期0。clip \(\pm6 hip\) | m/s | 常時 | 歩幅・FRG先送り | 過大で clip/非実現 | 標準使用 |
| A | Gait | Gait種類 | `simulation_params['gait']` | `config.py` | `'trot'` | — | 常時 | 脚順・支持 | 種類変更は offset も変わる | 標準使用 |
| A | Gait | Step frequency | `gait_params[gait]['step_freq']` | `config.py` | 1.35 | Hz | 常時（標準は固定） | \(T,T_{st},T_{sw}\)、歩幅 | 高すぎると Swing 鉛直速度 | 標準使用 |
| A | Gait | Duty factor | `gait_params[gait]['duty_factor']` | `config.py` | 0.74 | — | 常時 | Stance/Swing比、overlap | 低すぎると支持不足 | 標準使用 |
| A | Gait | Gait type enum | `gait_params[gait]['type']` | `config.py` | `TROT=0` | int | `gait` と対 | `phase_offset` 選択 | 名前と enum をずらすと別歩容 | 標準使用 |
| A | Swing | Step height | `simulation_params['step_height']` | `config.py` | `0.2*0.28=0.056` | m | 常時 | 遊脚頂点 | 高すぎると時間不足・衝撃 | 標準使用 |
| A | Swing | Cartesian Kp | `swing_position_gain_fb` | `config.py` | 500 | N/m 相当 | 遊脚 | 位置追従 | 振動 | 標準使用 |
| A | Swing | Cartesian Kd | `swing_velocity_gain_fb` | `config.py` | 10 | N·s/m 相当 | 遊脚 | 減衰 | 遅すぎると届かない | 標準使用 |
| A | MPC | 高さ重み | `Q_position[2]` | `centroidal_nmpc_nominal.py` `set_weight` | 1500 | — | 常時 | 沈み抑制 | 他軸を無視して高さ優先 | 標準使用。**configに無し** |
| A | MPC | 速度重み | `Q_velocity` | 同上 | `[200,200,200]` | — | 常時 | 指令追従 | 大きすぎると力要求増 | 標準使用。ハードコード |
| A | MPC | Roll/Pitch重み | `Q_base_angle[0:2]` | 同上 | `[500,500]` | — | 常時 | 水平 | 接触切替と干渉し振動 | 標準使用。ハードコード |
| A | MPC | Yaw重み | `Q_base_angle[2]` | 同上 | 0 | — | 常時 | 既定はYaw非追従 | 上げると旋回と争う | 標準使用（0） |
| A | MPC | 角速度重み | `Q_base_angle_rates` | 同上 | `[20,20,50]` | — | 常時 | 姿勢減衰、Yaw rate | 過減衰で鈍い | 標準使用。ハードコード |
| A | MPC | GRF重み | `R_foot_force` | 同上 | `[0.001]*3`×4 | — | 常時 | 力を小さくする | 大きすぎると追従犠牲 | 標準使用。ハードコード |
| A | 制約 | 摩擦 \(\mu\) | `mpc_params['mu']` | `config.py` | 0.42 | — | 常時 | 錐の傾き | 実床より大きいと滑りを計画 | 標準使用。質量仕様ではない |
| B | MPC | 足位置重み | `Q_foot_pos` | `set_weight` | `[300]*3`×4 | — | `use_foothold_optimization` | 参照着地へ寄せる | 地形ずれを嫌う | 標準使用。ハードコード |
| B | MPC | 足速度重み | `R_foot_vel` | `set_weight` | `[1e-4,1e-4,1e-5]`×4 | — | 同上 | OCP内の足運び | **Swing実行には未使用** | 標準使用（OCPのみ） |
| B | MPC | Horizon | `mpc_params['horizon']` | `config.py` | 12 | step | 常時 | 予見 0.24 s | 計算量 | 標準使用 |
| B | MPC | MPC dt | `mpc_params['dt']` | `config.py` | 0.02 | s | 常時 | 離散化 | 接触列解像度 | 標準使用 |
| B | MPC | MPC周波数 | `simulation_params['mpc_frequency']` | `config.py` | 100 | Hz | 常時 | 再計画周期 | 低すぎるとholdが長い | 標準使用 |
| B | MPC | Foothold最適化 | `use_foothold_optimization` | `config.py` | True | bool | 常時 | 遊脚 \(\dot p\) を許可 | Falseだと足固定 | 標準使用 |
| B | Foothold | Hip offset | `FootholdReferenceGenerator.hip_offset` | `foothold_reference_generator.py` | 0.1 | m | 常時 | 左右スタンス幅 | 狭すぎ/広すぎ | ハードコード |
| B | Foothold | 先送りclip | `hip_height*1.5` | 同上 | ±0.42 | m | 常時 | 過大歩幅抑制 | 高速で歩幅頭打ち | ハードコード |
| B | Foothold | 誤差補正clip | `±0.05` | 同上 | ±0.05 | m | 常時 | 速度誤差補償上限 | 補正不足 | ハードコード |
| B | Foothold | 誤差時定数相当 | \(\sqrt{h/g}\) | 同上 | \(\sqrt{ref_z/g}\) | s | 常時 | capture的補正 | `ref_z` に連動 | 式固定 |
| B | Solver | NLP反復 | `num_qp_iterations` | `config.py` | 1 | — | `use_RTI=False` | 精度 | 時間 | 標準使用 |
| B | Solver | HPIPM mode | `solver_mode` | `config.py` | `'balance'` | — | 常時 | 速度/頑健 | `'speed'` は未分岐（コードは `'fast'`） | 標準使用（balance） |
| B | 運用 | 指令速度clip | env 内部 \(6 hip\) | gym-quadruped | ±1.68 | m/s | human | 過大指令防止 | 上限が隠れる | ハードコード |
| C | 上位 | 周波数最適化 | `optimize_step_freq` | `config.py` | False | bool | 明示True | 候補から \(f\) | 評価時Foothold非再計算 | 無効 |
| C | 上位 | 周波数候補 | `step_freq_available` | `config.py` | `[1.4,2.0,2.4]` | Hz | 上ON | Cadence選択 | 1.35∉候補。penaltyで1.4寄り | 無効 |
| C | 地形 | VFAモード | `visual_foothold_adaptation` | `config.py` | `'blind'` | — | ≠blind | z置換 or virall | HeightMap必要。`vfa`は非公開 | 無効 |
| C | 制約 | Foothold制約 | `use_foothold_constraints` | `config.py` | False | bool | Trueかつ最適化ON | 足を箱に閉じ込める | slack。blindでは箱が参照±0.15 | 無効 |
| C | 制約 | 静的安定 | `use_static_stability` | `config.py` | False | bool | 勾配のみ。ZMPと排他 | 支持多角形 | 保守的で動けない | 無効 |
| C | 制約 | ZMP安定 | `use_zmp_stability` | `config.py` | False | bool | 同上 | ZMP | 同上 | 無効 |
| C | 制約 | 安定margin | `trot/pace/crawl_stability_margin` | `config.py` | 0.04/0.1/0.04 | m | 安定ON | 多角形縮小 | 大きすぎて infeasible | 無効 |
| C | MPC | 積分補償 | `use_integrators` | `config.py` | False | bool | True | 定常偏差 | 巻き上がり | 無効。重み自体はQに在る |
| C | MPC | 積分 \(\alpha\) | `alpha_integrator` | `config.py` | 0.1 | — | 積分ON | 積分速度 | — | 無効 |
| C | MPC | 積分cap | `integrator_cap` | `config.py` | `[0.5,0.2,0.2,0,0,1]` | 混在 | 積分ON | 飽和 | 軸割当は実装依存 | 無効 |
| C | MPC | Slack線形 | `ocp.cost.zl/zu` | `centroidal_nmpc_nominal.py` | 1000 | — | 足/安定制約ON | 違反を嫌う | 硬すぎてQP失敗 | 無効。ハードコード |
| C | MPC | Slack二次 | `Zl/Zu` | 同上 | 1 | — | 同上 | 同上 | — | 無効 |
| C | 接触 | Reflexモード | `reflex_trigger_mode` | `config.py` | `False` | — | `'tracking'`/`'geom_contact'` | 早期接地で軌道変更 | 誤検出 | 無効 |
| C | 接触 | Reflex高さ | `reflex_max_step_height` | `config.py` | `0.5*0.28=0.14` | m | Reflex ON | 回避高さ | 時間不足 | 無効 |
| C | 接触 | 次歩高さ増 | `reflex_next_steps_height_enhancement` | `config.py` | False | bool | Reflex ON | 数周期高くする | 過剰ジャンプ | 無効 |
| C | 接触 | Early stance時間 | `early_stance_time_threshold` | `early_stance_detector.py` | 0.07 | s | Reflex ON | 始終端を無視 | — | 無効。ハードコード |
| C | 接触 | 追従誤差閾値 | `relative/absolute_*` | 同上 | 0.3 / 0.1 | — / m | `tracking` | 発火感度 | 誤発火 | 無効。ハードコード |
| C | 低レベル | 関節Kp | `impedence_joint_position_gain` | `config.py` | 10 | N·m/rad | wrapper PD（コメントアウト） | インピーダンス | 実機向け | **sim Dead**。ROS2は使う |
| C | 低レベル | 関節Kd | `impedence_joint_velocity_gain` | `config.py` | 2 | N·m·s/rad | 同上 | 同上 | — | **sim Dead** |
| C | 低レベル | Torque soft | `tau_soft_limits_scalar` | `simulation.py` | 0.9 | — | 常時 | 定格の90%でclip | 飽和で追従低下 | ハードコード |
| C | 運用 | 速度変調 | `velocity_modulator` | `config.py` | True | bool | 常時 | 脚伸びで指令ゼロ | 突然停止 | 標準使用 |
| C | 運用 | VM距離 | `max_distance` | `velocity_modulator.py` | 0.2 | m | VM ON | 発火閾値 | 早すぎ/遅すぎ | ハードコード |
| C | Solver | RTI | `use_RTI` | `config.py` | False | bool | 勾配 | 遅延短縮 | 精度 | 無効 |
| C | Solver | AS-RTI型 | `as_rti_type` / `as_rti_iter` | `config.py` | Standard / 1 | — | RTI ON | RTI詳細 | — | 無効 |
| C | Solver | DDP | `use_DDP` | `config.py` | False | bool | RTIと非両立 | 別NLP | 制約制限 | 無効 |
| C | Solver | Warm start | `use_warm_start` | `config.py` | False | bool | 勾配 | 足初期化 | — | 無効 |
| C | 地形 | 非一様離散 | `use_nonuniform_discretization` 他 | `config.py` | False | — | True | 初期細かいdt | 接触列複雑 | 無効 |
| D | MPC | type切替 | `mpc_params['type']` | `config.py` | `'nominal'` | — | 起動時 | コントローラ全体 | 重み・制約が別 | 標準はnominal |
| D | MPC | GRF rate重み | — | `input_rates` 専用 | — | — | `type='input_rates'` | \(\Delta F\) 滑らかさ | **nominal未実装** | 専用 |
| D | MPC | Lyapunov K | `K_z1`,`K_z2`, residual | `config.py` | 配列 | — | `type='lyapunov'` | 安定制約 | — | 専用 |
| D | Sampling | 手法/σ/並列 | `sampling_*` | `config.py` | mppi, 800, … | 混在 | `type='sampling'` | サンプルMPC | GPU/時間 | 専用 |
| D | 外力 | 外乱補償 | `external_wrenches_compensation` | `config.py` | True | bool | 呼び元がwrenchを渡す | 予測補償 | wrapperは `zeros(6)` | **実質Dead**（標準） |
| D | 外力 | 補償ステップ | `external_wrenches_compensation_num_step` | `config.py` | 15 | — | 上 | — | — | 同上 |
| D | 協調 | アーム補償 | `passive_arm_compensation` | `config.py` | True | bool | collaborative | — | 本typeなし | Dead（本構成） |
| D | 入力予測 | `use_input_prediction` | 同上 | False | bool | input_rates | 遅延補償 | — | 専用 |
| D | 実験 | scene | `simulation_params['scene']` | `config.py` | `'perlin'` | — | sim | 地形 | 制御ゲインではない | 標準使用 |
| D | 実験 | sim dt | `simulation_params['dt']` | `config.py` | 0.002 | s | sim | 積分周期 | 500 Hz前提 | 標準使用 |
| D | 実験 | Swing生成器 | `swing_generator` | `config.py` | `'scipy'` | — | 常時 | 軌道族 | explicitは別形 | 標準使用 |
| D | 実験 | 慣性再計算 | `use_inertia_recomputation` | `config.py` | True | bool | 常時 | `mj_fullM` をpへ | 仕様慣性と差 | 標準使用 |
| Dead | 運用 | `simulation_params['mode']` | `config.py` | `'human'` | — | — | **未読**。実指令は `base_vel_command_type` | — | Dead |
| Dead | Solver | `solver_mode='speed'` | コメント | — | — | コードは `'fast'` | 設定してもBALANCEのまま | — | Deadキー名 |
| 仕様固定 | Gait | Phase offset | `PGG.reset` 内 | `periodic_gait_generator.py` | Trot `[0.5,1,1,0.5]` | 周期 | gait_type | 歩容の定義 | **通常チューニング対象ではない** | 標準使用 |

`grf_max=mass*g`, `grf_min=0` は質量仕様から決まる上限。\(\mu\) と違って「ロボット質量を変える」調整ではない。必要なら安全側に係数を掛ける研究変更になる（D）。

---

## 2. 使用確認の要約

| 判定 | 例 |
|---|---|
| Configにあり標準で効く | gait / freq / duty / step_height / ref_z / μ / horizon / dt / mpc_frequency / swing gains / VM |
| Configに無くハードコードだが効く | 全MPC重み、hip_offset、FRG clip、torque 0.9、Slack数値 |
| Configにあり標準で無効 | optimize_step_freq、VFA、foothold/stability制約、integrators、RTI、DDP、reflex |
| 別Controller専用 | GRF rate、Lyapunov、sampling_*、input_prediction |
| Dead | `simulation_params['mode']`、標準経路の関節impedance加算、`external_wrenches`（渡す値がゼロ）、`solver_mode='speed'` |
| 単位 | 上表。重みは無次元のLS重み |
| 形 | 重みは対角ベクトル。marginはscalar。gait_paramsはdict |
| 脚別 | hip_offsetの符号のみ。重み・gainは4脚共通 |

MPC重みを変えるには `set_weight()` を編集するか、外出しが必要。`config.py` を触っても高さ/速度重みは変わらない。

---

## 3. MPC重み（nominal `set_weight`）

`W=blkdiag(Q,R)`, `W_e=Q`。終端に別 \(Q_N\) は無い。Stage+Terminalは同じQ。

| 物理量 | コード変数 | Index | 既定重み | Stage/Terminal | 調整効果 | 大きすぎる場合 | 小さすぎる場合 |
|---|---|---|---|---|---|---|---|
| CoM位置 xy | `Q_position[0:2]` | x 0:2 | `[0,0]` | 両方 | 水平位置は追わない（速度経由） | xy位置ホバー | 既定どおり位置自由 |
| CoM height | `Q_position[2]` | x 2 | 1500 | 両方 | 高さ維持 | 他目的を犠牲、脚伸び | 沈む |
| Linear velocity | `Q_velocity` | x 3:6 | `[200,200,200]` | 両方 | 速度追従 | GRF過大・滑り | 遅い・流れ |
| Base orientation | `Q_base_angle` | x 6:9 | `[500,500,0]` | 両方 | Roll/Pitch水平。Yawは0 | 振動、旋回阻害 | 傾く |
| Angular velocity | `Q_base_angle_rates` | x 9:12 | `[20,20,50]` | 両方 | 姿勢減衰、Yaw rate | 鈍い | 振動 |
| Foot position | `Q_foot_pos` | x 12:24 | `[300]*3`×4 | 両方 | 参照着地 | 地形ずれを嫌う | 足が流れる |
| Integral z | `Q_com_position_z_integral` | x 24 | 50 | 両方 | 積分ON時の高さバイアス | 巻き上がり | 無効時はほぼ無効果 |
| Integral vx,vy,vz | `Q_com_velocity_*_integral` | x 25:28 | 10 | 両方 | 積分ON時 | 同上 | 同上 |
| Integral roll/pitch | `Q_roll/pitch_integral` | x 28:30 | 10 | 両方 | 積分ON時 | 同上 | 同上 |
| Foot velocity | `R_foot_vel` | u 0:12 | `[1e-4,1e-4,1e-5]`×4 | Stageのみ | OCP内足速度 | 足が動かない | 足が跳ねる（予測内） |
| GRF | `R_foot_force` | u 12:24 | `[0.001]*3`×4 | Stageのみ | 力最小化 | 追従不足・沈み | 大きな力・滑り |
| GRF rate | — | — | — | — | **未実装**（`input_rates`） | — | — |
| Slack linear | `zl`,`zu` | slack | 1000 | 制約ON時 | ソフト制約違反 | QP厳しい | 領域を破る |
| Slack quadratic | `Zl`,`Zu` | slack | 1 | 制約ON時 | 同上 | — | — |

積分状態のQは `use_integrators=False` でも行列に入る。積分状態が動かなければ実効ゼロに近い。

---

## 4. Gait / Foothold

| 項目 | 場所 | 既定 | チューニングか | 注 |
|---|---|---|---|---|
| Gait type | `gait` + `gait_params[]['type']` | trot / 0 | A。種類切替は手動 | 自動Envelopeなし |
| Step frequency | `step_freq` | 1.35 Hz | A | 候補最適化はCかつオフ |
| Duty factor | `duty_factor` | 0.74 | A | 周波数更新でも不変 |
| Phase offset | `PGG.reset` | Trot `[0.5,1.0,1.0,0.5]` | **Gait仕様。通常は触らない** | 変えると別歩容。configに無し |
| Step height | `step_height` | 0.056 m | A | 係数 0.2 が実質ノブ |
| Frequency candidates | `step_freq_available` | 1.4, 2.0, 2.4 | C。標準無効 | 初期1.35を含まない |
| 速度先送り | `stance_time/2 * v_ref` | 式固定 | Bは clip と `stance_time`（=f,d） | 係数 1/2 はハードコード |
| 速度誤差補正 | \(\sqrt{h/g}(\bar v-v^{ref})\) | 式固定 | Bは ±0.05 clip と `ref_z` | 移動平均長20 |
| Foothold clip | ±1.5 hip / ±0.05 | ハードコード | B | configなし |
| Stability margin | `*_stability_margin` | 0.04 等 | C。標準無効 | 安定制約ON時のみ |
| Terrain margin | VFA頂点 / MPC箱 ±0.15 | ハードコード | C。標準無効 | 「terrain margin」キーは無い |

Phase offset は Trot の対角同期を定義する。ユーザー調整表のA項目ではなく、歩容IDに属する。

---

## 5. Swing / Low-level

| 項目 | キー / 変数 | 既定 | 標準で効くか |
|---|---|---|---|
| Cartesian position gain | `swing_position_gain_fb` | 500 | 効く。PDが二重（トルク項とFF加速度） |
| Cartesian velocity gain | `swing_velocity_gain_fb` | 10 | 効く |
| Joint position gain | `impedence_joint_position_gain` | 10 | **sim無効**（加算コメントアウト）。ROS2 publish |
| Joint velocity gain | `impedence_joint_velocity_gain` | 2 | 同上 |
| Early stance threshold | `early_stance_time_threshold` 他 | 0.07 s, 0.3, 0.1 m | Reflexオフなら未使用 |
| Reflex height | `reflex_max_step_height` | 0.14 m | Reflexオフなら未使用 |
| Torque soft limit | `0.9 * ctrlrange` | 0.9 | 効く。configなし |
| Joint impedance | 上記Kp/Kd | — | simでは未適用 |
| IK差clip | ±3 rad / ±10 rad/s | ハードコード | 目標関節だけ。標準sim未使用 |

---

## 6. 症状逆引き

| 症状 | 最初に確認するLog | 原因候補 | 最初の調整項目 | 次の調整項目 | 副作用 |
|---|---|---|---|---|---|
| 胴体が沈む | [10](10_mpc_ocp.md), [13](13_stance_swing_torque.md) | \(F_z\) 上限、`ref_z`、高さ重み、立脚が `-J.T F` のみ | `ref_z`、`Q_position[2]` | `R_foot_force` を下げる、Duty↑ | 脚伸び、高さ振動 |
| Roll/Pitch振動 | [07](07_periodic_gait_generator.md), [11](11_gait_mpc_coupling_e2e.md) | 接触切替、角度重み、推定 | 接触列と `current_contact` | `Q_base_angle` / rate | 鈍い・旋回阻害 |
| 速度追従が遅い | [06](06_user_command_dataflow.md), [15](15_speed_frequency_duty_stride.md) | 指令clip、VM、トルク飽和、速度重み、歩幅clip | 指令と `Q_velocity` | `R` GRF↓、freq↑、FRG clip | 滑り、力増大 |
| 足が滑る | [14](14_mujoco_closed_loop.md), [10](10_mpc_ocp.md) | 実μ、計画μ=0.42、水平GRF | 速度を下げる | `mu` を保守側へ、`R` GRF↑ | 加速不足 |
| Torque saturation | [13](13_stance_swing_torque.md), [14](14_mujoco_closed_loop.md) | 0.9 clip、大きなF、Swing Kp | 速度・高さ指令 | GRF重み↑、Swing gain↓ | 追従低下 |
| 着地衝撃が大きい | [13](13_stance_swing_torque.md), [15](15_speed_frequency_duty_stride.md) | step_height、短い \(T_{sw}\)、高Cadence | `step_height`↓ | freq↓ または Duty | 障害物余裕減 |
| Swing legが振動 | [13](13_stance_swing_torque.md) | Kp=500、二重PD | `swing_position_gain_fb`↓ | Kd↑ | 追従遅れ、未到達 |
| Footholdへ届かない | [08](08_foothold_reference.md), [16](16_rough_terrain_feasibility.md) | 残時間未制約、遠いTD、clip | freq↑ または速度↓ | Foothold clip、Swing gain | 鉛直Swingがキツい |
| 穴の縁へ着地 | [16](16_rough_terrain_feasibility.md), [08](08_foothold_reference.md) | 標準blind、VFAなし | 速度↓、scene確認 | VFA+制約（非標準） | virall依存。残時間非保証 |
| Solver infeasible | [10](10_mpc_ocp.md), [12](12_mpc_output_receding.md) | 摩擦+安定/足箱、status 1/4 | 安定・足制約を切る（標準は既に切） | slack、margin、μ | 前回GRF hold |
| Solver時間超過 | [10](10_mpc_ocp.md) | N、iter、batched、sampling 800 | `horizon`↓、`num_qp_iterations` | `solver_mode`、RTI | 精度低下 |

「接触時Torque急変 → GRF rate」は `14` の逆引きだが、**nominalにGRF rate重みは無い**。実際は `R_foot_force`、接触mask、Swing切替を見る。

---

## 7. 推奨調整順序（固定 vs 調整）

資料 `14` の8段を、現行コードで触る値に落とす。

| 段階 | 固定する | 調整する | 標準で触らない（無効） |
|---|---|---|---|
| 1. Full stance | `gait='full_stance'` または位相停止、μ、質量 | `ref_z`、`Q` 高さ/角度/rate、`R` GRF | VFA、周波数最適化 |
| 2. Swing単脚 | `swing_period`（f,dから）、軌道族 scipy | Kp/Kd、`step_height` | Reflex、関節PD |
| 3. 低速Trot | `gait='trot'`、phase offset、μ | `step_freq`、`duty_factor`、`step_height` | 候補周波数 |
| 4. 速度Step | Horizon/dt、mpc_frequency | 指令、`Q_velocity`、`R` GRF、FRG clip意識 | — |
| 5. Frequency/Duty sweep | Gait type | `step_freq`×`duty` 格子。幾何は log 15 | `optimize_step_freq`（中身は不完全） |
| 6. 高速化 | 可動域clipの存在 | freq↑、指令上限、hip_offset | 自動Envelope（未実装） |
| 7. 不整地 | — | 速度↓。研究なら VFA+制約+margin | 交差保証は未実装（log 16） |
| 8. Sim-to-Real | XML定格 | Integral、Reflex、関節impedance（ROS2）、RTI、実μ | simのコメントアウトPD |

各段階で仕様値（質量、ctrlrange、リンク長）は固定。

---

## 8. 資料照合

### `14_MPC_and_Controller_Tuning.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §1 仕様値除外 | 一致 | 正しい | — |
| §2 A項目（gait/freq/duty/height/重み/swing） | 概ね一致 | 正しい | 重みはconfigではなく `set_weight` |
| §2 GRF rate をB | nominalの調整項目 | **誤り** | `input_rates` 専用。標準未実装 |
| §2 Foothold margin / Stability をB | 標準で効くように読める | 不完全 | 両方オフ。キーも「terrain margin」は無い |
| §2 Gait切替点・減速停止をC | 調整項目 | 不完全 | **パラメータ未実装**。運用ルールもない |
| §3 逆引き「GRF rate」「Foothold margin」 | 次に調整 | 不完全 | 標準経路にノブが無い |
| §4 推奨順 | 指針 | 正しい（指針） | 段5のGRF rateは実装と不一致 |
| §5 ADAS対応 | 概念 | 正しい | GRF rate対応は type依存 |

### `appendices/C_Parameter_Index.md`

| 記載 | 判定 | 理由 |
|---|---|---|
| Gait/freq/duty/height の既定 | 正しい | configと一致 |
| 重み数値（高さ1500等） | 正しい | `set_weight` |
| Foot vel "small" | 不完全 | `[1e-4,1e-4,1e-5]` |
| Reflex "tracking" | **誤り** | ディスクは `False` |
| Joint impedance 10/2 をC | 正しい（実機） | 標準simは未適用と未記 |
| μ、`ref_z`、mpc_frequency、hip_offset 欠落 | 不完全 | 標準で効く |
| 重みの場所が config に見える | 不完全 | 実体は `set_weight` |
| GRF rate を載せていない | 正しい | 未実装を黙って略している |

---

## 9. 事実 / 解釈

**事実**

- Baselineで効くノブの半数は `config.py` に無く、`set_weight` と FRG/sim の定数である。
- `14`/`C` がCに置いた機能の多くは標準オフ、または未実装。
- Phase offset は歩容定義であり、通常のゲイン表に入れない。
- GRF rate weight は nominal に無い。

**解釈**

- 平地低速の最初の調整は `ref_z`、Trot の f/d、Swing Kp/Kd、高さ/速度/姿勢/GRF重みに限る。
- 不整地・自動周波数・インピーダンスはフラグを立ててから別実験として扱う。
