# Analysis Logs

このディレクトリは、学習資料本文（`docs/qpympc-study/00`〜`19`、appendices）を修正せず、コード照合の結果だけを残す場所である。

正本はコードである。各ログの判定は次の4種。

- 正しい
- 不完全
- 誤り
- コードから確認不能

| ファイル | 対応プロンプト | 状態 |
|---|---|---|
| [00_user_chat_prompts.md](00_user_chat_prompts.md) | 入力チャット原文 | 完了 |
| [01_baseline.md](01_baseline.md) | Baseline記録 | 完了 |
| [02_readme_comparison.md](02_readme_comparison.md) | `00_README`照合 | 完了 |
| [03_call_graph.md](03_call_graph.md) | 標準実行経路 | 完了 |
| [04_docs_02_16_D_comparison.md](04_docs_02_16_D_comparison.md) | `02`/`16`/`D`照合 | 完了 |
| [05_go2_plant.md](05_go2_plant.md) | Go2 Plant | 完了 |
| [06_user_command_dataflow.md](06_user_command_dataflow.md) | 指令→`ref_state` | 完了 |
| [07_periodic_gait_generator.md](07_periodic_gait_generator.md) | PGG | 完了 |
| [08_foothold_reference.md](08_foothold_reference.md) | Foothold | 完了 |
| [09_centroidal_srbd.md](09_centroidal_srbd.md) | SRBD | 完了 |
| [10_mpc_ocp.md](10_mpc_ocp.md) | nominal OCP | 完了 |
| [11_gait_mpc_coupling_e2e.md](11_gait_mpc_coupling_e2e.md) | `contact_sequence` E2E | 完了 |
| [12_mpc_output_receding.md](12_mpc_output_receding.md) | MPC出力 | 完了 |
| [13_stance_swing_torque.md](13_stance_swing_torque.md) | Stance/Swing torque | 完了 |
| [14_mujoco_closed_loop.md](14_mujoco_closed_loop.md) | Torque→MuJoCo→次状態 | 完了 |
| [15_speed_frequency_duty_stride.md](15_speed_frequency_duty_stride.md) | 速度・周波数・Duty・歩幅 | 完了 |
| [16_rough_terrain_feasibility.md](16_rough_terrain_feasibility.md) | 不整地実現可能性 | 完了 |
| [17_user_tuning_parameters.md](17_user_tuning_parameters.md) | ユーザー調整パラメータ | 完了 |
| [18_automatic_tuning_and_outer_loop.md](18_automatic_tuning_and_outer_loop.md) | 自動化とOuter-loop | 完了 |
| [19_gradient_vs_sampling_mpc.md](19_gradient_vs_sampling_mpc.md) | Gradient vs Sampling | 完了 |
| [20_experiment_log_design.md](20_experiment_log_design.md) | 実験ログ設計（未実装） | 完了 |
| [21_experiment_research_roadmap.md](21_experiment_research_roadmap.md) | 実験・研究ロードマップ | 完了 |
| [22_integrated_audit.md](22_integrated_audit.md) | 学習資料の統合監査 | 完了 |
| [23_study_docs_applied.md](23_study_docs_applied.md) | 監査の本文反映（制御コード未変更） | 完了 |
| [24_study_docs_verification.md](24_study_docs_verification.md) | 修正後本文の機械・意味検証 | 完了 |
| [25_final_dataflow.md](25_final_dataflow.md) | 指令→Feedback最終データフロー（本文02へ反映） | 完了 |
| [26_index_and_entry.md](26_index_and_entry.md) | 入口・索引の同期 | 完了 |

標準設定（ディスク上 `quadruped_pympc/config.py`）:

- `robot='go2'`
- `mpc_params['type']='nominal'`
- `simulation_params['gait']='trot'`
- `visual_foothold_adaptation='blind'`
- `optimize_step_freq=False`
- `use_RTI=False`
- `use_foothold_constraints=False`
- `use_static_stability=False`
- `use_zmp_stability=False`
- `use_integrators=False`
- `use_warm_start=False`
- `reflex_trigger_mode=False`
- `simulation_params['dt']=0.002`
- `mpc_frequency=100`
