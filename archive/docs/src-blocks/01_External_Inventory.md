# External 在庫 — 2リポジトリに何があるか

## 1. 結論

`external/` には制御スタックが2本ある。両方とも「速度指令から関節トルクまで」を持つが、言語・ソルバ・下位制御・推定の有無が違う。在庫の正本は本章である。分割判定は[02](02_Can_We_Split.md)。

## 2. 実装事実 — ディスク上にあるもの

```text
external/
├── Quadruped-PyMPC/     # Python。MuJoCo sim + acados NMPC
└── legged_control/      # C++ / ROS。Gazebo・実機 + OCS2 NMPC + QP WBC
```

`scripts/setup_references.sh` は muse、mujoco_mpc、ocs2_ros2 等も clone できる。**いまの `external/` には上記2個だけがある**。

| 項目 | Quadruped-PyMPC | legged_control |
|---|---|---|
| 言語 | Python | C++（catkin） |
| 標準ロボット | Go2（`config.py` の `robot = 'go2'`） | A1 / Go1 / Aliengo（`ROBOT_TYPE`） |
| Plant | MuJoCo + gym-quadruped | Gazebo または Unitree UDP |
| NMPC | acados + CasADi SRBD（標準 `nominal`） | OCS2 `SqpMpc`（ソースは repo 外） |
| 下位制御 | Stance `-J^T F` + Swing Cartesian PD | `WeightedWbc`（qpOASES、42変数） |
| 状態推定 | なし（sim 真値） | 線形 Kalman（並進のみ） |
| ゲイト | `PeriodicGaitGenerator`（同パッケージ） | OCS2 `GaitReceiver`（repo 外） |
| 足場 | `FootholdReferenceGenerator` | `SwingTrajectoryPlanner`（主に z） |
| 周期 | sim 500 Hz、MPC 100 Hz | HW/WBC 500 Hz、NMPC 100 Hz |
| 照合コミット | `3adfad9` | `a7f381c`（2025-02-13） |

## 3. 実装事実 — PyMPC の中身

プロジェクト `.py` は `quadruped_pympc/` 配下 43 本（`acados/` サブモジュールを除く）。

| 層 | パス | 役割 |
|---|---|---|
| 入口 | `simulation/simulation.py` | `run_simulation()`。MuJoCo 閉ループ |
| ファサード | `quadruped_pympc/quadruped_pympc_wrapper.py` | `compute_actions` |
| 配線 | `interfaces/wb_interface.py` | 地形・ゲイト・足場・参照・トルク |
| 配線 | `interfaces/srbd_controller_interface.py` | MPC バックエンド選択と GRF mask |
| 設定 | `config.py` | モジュールグローバル。25本以上が import |
| ゲイト | `helpers/periodic_gait_generator.py` | 位相と接地列 |
| 足場 | `helpers/foothold_reference_generator.py` | 着地点ヒューリスティック |
| 地形 | `helpers/terrain_estimator.py` | roll/pitch/高さ |
| 遊脚 | `helpers/swing_trajectory_controller.py` + `swing_generators/` | Cartesian PD |
| 予測モデル | `controllers/gradient/nominal/centroidal_model_nominal.py` | CasADi SRBD |
| NMPC | `controllers/gradient/nominal/centroidal_nmpc_nominal.py` | acados OCP |
| 代替 MPC | `input_rates/`, `lyapunov/`, `collaborative/`, `kinodynamic/`, `sampling/` | 標準経路では未使用 |
| 実機寄り | `ros2/` | 本ノートの第一抽出対象外 |

標準設定（`config.py`）:

- `mpc_params['type'] = 'nominal'`
- `horizon = 12`, `dt = 0.02`（予測 0.24 s）
- `simulation_params['dt'] = 0.002`, `gait = 'trot'`, `visual_foothold_adaptation = 'blind'`
- trot: `step_freq = 1.35`, `duty_factor = 0.74`

詳細: [docs/qpympc-study/16_Code_Map_and_Call_Graph.md](../qpympc-study/16_Code_Map_and_Call_Graph.md)。

## 4. 実装事実 — legged_control の中身

11 catkin パッケージ、C++ ヘッダ/ソース約 65 本。

| パッケージ | 役割 | アルゴリズム核か配線か |
|---|---|---|
| `legged_controllers` | `LeggedController`、参照ノード、安全 | 配線が主。参照関数は核 |
| `legged_interface` | OCS2 OCP の組み立て（コスト・制約） | 核だが OCS2 API 依存 |
| `legged_estimation` | 線形 KF、cheater | 核 |
| `legged_wbc` | `WeightedWbc` / `HierarchicalWbc` | 核 |
| `legged_hw` / `legged_gazebo` / `legged_unitree_hw` | 500 Hz ループ、Gazebo、UDP | 配線 |
| `legged_common` | HybridJoint / ContactSensor | 配線（ros-control 型） |
| `qpoases_catkin` | qpOASES 取得 | ビルドだけ |

**この tree に無いもの（include と README からの実装事実）:**

- `SqpMpc`、`MPC_MRT_Interface`
- `LeggedRobotDynamicsAD`
- `CentroidalModelRbdConversions`
- `GaitSchedule` / `GaitReceiver`
- HPIPM、CppAD 生成モデル

作者 README は新規開発停止、後継は `legged_perceptive` と書く。

詳細: [docs/legged_control/01_Packages_and_Control_Loop.md](../legged_control/01_Packages_and_Control_Loop.md)。

## 5. 実装事実 — 両リポが「すでにファイルとして分けている」もの

どちらもモノリス1ファイルではない。役割ごとのファイル/クラスがある。

| 役割 | PyMPC | legged_control |
|---|---|---|
| 指令 → 参照 | `wb_interface.update_state_and_reference` 内 | `TargetTrajectoriesPublisher.cpp` の自由関数 |
| ゲイト | `PeriodicGaitGenerator` | OCS2（外部） |
| 予測モデル | `Centroidal_Model_Nominal` | OCS2 `LeggedRobotDynamicsAD`（外部） |
| NMPC | `Acados_NMPC_Nominal` | `SqpMpc`（外部）+ `LeggedInterface` |
| 下位 | `compute_stance_and_swing_torque` | `WeightedWbc` |
| 推定 | なし | `KalmanFilterEstimate` |
| 閉ループ | `simulation.py` + Wrapper | `LeggedController::update` |

## 6. 実装事実 — ワークショップが実際に動かしている経路

本 repo の sim 教材が呼ぶのは **PyMPC の標準 nominal** である。legged_control は学習・比較用に clone してあり、`mpc_dog` の Notebook パイプラインからは実行していない（[README.md](../../README.md)）。

## 7. 推測 — 「在庫」の読み方

- PyMPC は **Python で完結した SRBD-MPC + 簡易下位制御** の教材・研究実装である。推定と QP-WBC は意図的に薄い。
- legged_control は **OCS2 の薄いロボット層** である。NMPC 本体を切り出しても、OCS2 なしでは動かない。
- 両リポを「同じ製品の2版」と見るのは誤りである。問題設定（centroidal 歩行）は近いが、状態定義と下位制御が違う。

## 8. 次

分割できるかは、ファイルがあることではなく **依存の向きと境界の型** で決まる。[02](02_Can_We_Split.md)。
