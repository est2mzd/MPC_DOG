# Quadruped-PyMPC → ブロック対応

## 1. 結論

PyMPC は **B03, B04, B05, B07, B08, B10 がすでにクラスとして存在する**。足りないのは B06 と B12、そして B00 の明示契約である。`WBInterface` と `simulation.py` は分解対象の配線である。

理論の正本: [docs/qpympc-study](../qpympc-study/00_README.md)。

## 2. 実装事実 — ファイル → ID

| ファイル | ID | 核 / 配線 |
|---|---|---|
| `helpers/periodic_gait_generator.py` | B03 | 核 |
| `helpers/periodic_gait_generator_jax.py` | B03（sampling 用） | 核。標準未使用 |
| `helpers/foothold_reference_generator.py` | B04 | 核 |
| `helpers/terrain_estimator.py` | B05 | 核 |
| `helpers/velocity_modulator.py` | B01 の後処理 | 核に近い |
| `helpers/swing_trajectory_controller.py` | B10 | 核 |
| `helpers/swing_generators/*.py` | B10 | 核 |
| `helpers/visual_foothold_adaptation.py` | B04 拡張 | 標準 OFF |
| `helpers/early_stance_detector.py` | B10 拡張 | 標準 OFF |
| `helpers/inverse_kinematics/*` | B10 周辺 | 標準で τ に未使用。IK は Plant 所有 |
| `controllers/gradient/nominal/centroidal_model_nominal.py` | B07 | 核 |
| `controllers/gradient/nominal/centroidal_nmpc_nominal.py` | B08 | 核 |
| `controllers/gradient/{input_rates,lyapunov,collaborative,kinodynamic}/` | B07/B08 代替 | 標準未使用 |
| `controllers/sampling/*` | B07/B08 代替 | 標準未使用 |
| `interfaces/srbd_controller_interface.py` | B08 のアダプタ + GRF mask | 配線 |
| `interfaces/wb_interface.py` | B02+B03+B04+B05+B09+B10 | 配線 |
| `quadruped_pympc_wrapper.py` | ループ + MPC 間引き | 配線 |
| `simulation/simulation.py` | B13 + ループ | 配線 |
| `config.py` | 全ブロックのパラメータ | 分解する |
| `ros2/*` | デプロイ | 対象外 |

## 3. 実装事実 — 標準経路で呼ばれる核メソッド

| ID | クラス.メソッド | 計算するもの |
|---|---|---|
| B05 | `TerrainEstimator.compute_terrain_estimation` | terrain roll/pitch/height, robot_height |
| B01' | `VelocityModulator.modulate_velocities` | 脚が伸び過ぎなら指令を 0 |
| B03 | `PeriodicGaitGenerator.run` | 位相 \(\phi_i\)。戻り接触は捨てる |
| B03 | `PeriodicGaitGenerator.compute_contact_sequence` | `(4,12)` 接地列 |
| B04 | `FootholdReferenceGenerator.update_lift_off_positions` 等 | イベント位置 |
| B04 | `FootholdReferenceGenerator.compute_footholds_reference` | 着地点 |
| B02 | `WBInterface.update_state_and_reference` の組立部 | `state_current`, `ref_state` |
| B08 | `Acados_NMPC_Nominal.compute_control` | GRF `(12,)`, footholds, next state `(24,)` |
| B08' | `SRBDControllerInterface` の mask | \(F^{cmd}=c_{i,0} F^{MPC}\) |
| B09 | `WBInterface.compute_stance_and_swing_torque` の stance | \(\tau=-J^T F^{cmd}\) |
| B10 | `SwingTrajectoryController.compute_swing_control_cartesian_space` | 遊脚 τ |
| B11 | `simulation.py` の `np.clip` | 0.9 × `actuator_ctrlrange` |
| B13 | `QuadrupedEnv.step` | `mj_step` |

`WBInterface` は B02–B05 と B09–B10 を同時に持つ。抽出時はメソッド単位で分ける。

## 4. 実装事実 — 標準で動かない（切らない）経路

[qpympc-study/16](../qpympc-study/16_Code_Map_and_Call_Graph.md) と一致。

- VFA（`visual_foothold_adaptation != 'blind'`）
- `optimize_step_freq`
- RTI / DDP
- `use_integrators`（状態は 30 次元だが既定 False）
- early stance detector
- 関節空間 swing、関節 PD（計算してもプラント未使用）
- sampling / lyapunov / kinodynamic / collaborative / input_rates
- 非ゼロ外乱レンチ（インタフェースは持つ）

第一抽出は **trot + nominal + blind** だけに閉じる（方針）。

## 5. 実装事実 — 既知の配線バグ（抽出時に直す対象）

`simulation.py` が `joints_pos` に **qvel の index** を渡している。nominal MPC は関節を OCP に使わないため、標準経路の歩行には効かない。自分の `src/` にコピーしない。

## 6. 推測 — 抽出の具体手順（PyMPC 側）

順番は依存の逆、テストしやすいものから。

1. **B03** `PeriodicGaitGenerator` を `config` なしで再実装または移植。入力: dt, freq, duty, offset。出力: phase, contact, sequence。
2. **B05** `TerrainEstimator`。入力を配列 `(4,3)` にする（`LegsAttr` を外す）。
3. **B04** foothold。重力と hip 高さを引数化。
4. **B10** swing 軌道だけ先（PD ゲインは引数）。
5. **B09** stance を独立関数 `tau = -J.T @ F`。
6. **B07** `forward_dynamics` を NumPy または CasADi 関数として、acados から切り離してテスト。
7. **B08** は acados を **包む**。codegen を自分で書き直さない。
8. `WBInterface` は移植しない。`loop/` で上記を呼ぶ。

## 7. PyMPC にあってカタログに無いもの

| 機能 | 扱い（方針） |
|---|---|
| Velocity modulator | B01 の optional 後段。独立モジュールにしてよい |
| Batched gait-frequency OCP | B03/B08 の拡張。Phase 1 対象外 |
| JAX sampling MPC | B08 の第2バックエンド候補。Phase 3 以降 |
| Lyapunov / input_rates | 研究用。標準ブロックに混ぜない |

## 8. 次

legged_control 側の対応。[05](05_LeggedControl_to_Blocks.md)。
