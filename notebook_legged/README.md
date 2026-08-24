# legged_control 理論・コード学習Notebook

大学院の初心者が `qiayuanliao/legged_control` を、完成したROSシステムとして眺めるのではなく、
数式・データ契約・小さな数値実験・C++対応箇所へ分解して学ぶ教材です。

照合した上流: commit `a7f381c0367e98e31c01336e678eef47e304d40d`（2025-02-13、master）

## 特徴

- `00` から `14` まで順番に読む
- NumPy / SciPy / Matplotlibだけで理論実験を再実行可能
- ROS / OCS2 / Gazebo / Unitreeが必要な実装事実と、教育用縮約実験を明確に区別
- 背景・目的・結論、ASCIIデータフロー、数式、コメント付きblock codeを接続
- `13` は4秒のequation-level proxy、`14` はA1 MuJoCo adapterの20秒以上×30 scenario
- 各章末にチューニング・変更時の観測項目を記載
- 詳細な実装監査は `../docs/legged_control/` を正本として参照

## 起動

```bash
uv sync --extra workshop
uv run jupyter lab notebook_legged/
```

最終benchmarkの厳密な再現command:

```bash
uv run python scripts/run_legged_control_benchmark.py --all
```

30本の20秒以上GIFに加えてJSON/CSV/summaryを保存するため、`notebook_legged/assets/scenarios/`
には数百MB規模の空き容量を見込む。既存の有効なscenarioは既定で再利用され、`--overwrite` を
明示しない限り一致する出力を置換しない。

## 章

| No. | Notebook | 到達点 |
|---:|---|---|
| 00 | `00_learning_map.ipynb` | 閉ループ全体と責務境界 |
| 01 | `01_packages_and_loop.ipynb` | package・thread・周期 |
| 02 | `02_state_input_frames.ipynb` | shape・単位・frame |
| 03 | `03_command_reference_gait.ipynb` | 2点参照と独立Gait |
| 04 | `04_state_estimation.ipynb` | 接地足を使う並進KF |
| 05 | `05_centroidal_dynamics.ipynb` | 合力・moment・centroidal |
| 06 | `06_nmpc_ocp_and_tuning.ipynb` | OCP・Q/R・horizon |
| 07 | `07_contact_constraints_and_swing.ipynb` | 接触・摩擦・遊脚 |
| 08 | `08_weighted_wbc.ipynb` | 42変数の瞬間QP |
| 09 | `09_hybrid_joint_hardware.ipynb` | torque FF + low-gain PD |
| 10 | `10_multirate_integration.ipynb` | policyと100/500 Hz統合 |
| 11 | `11_tuning_and_equation_changes.ipynb` | 再現可能な調整・式変更 |
| 12 | `12_repository_code_walkthrough.ipynb` | 実C++の端から端までのcall graph |
| 13 | `13_model_benchmark_30_scenarios.ipynb` | 4秒のequation-level proxy benchmark |
| 14 | `14_a1_mujoco_benchmark_30_scenarios.ipynb` | A1 MuJoCo adapterの30条件・20秒GIF・物理metric |

## 事実の境界

上流C++は `external/legged_control/` に上記commitでclone済みですが、gitignore対象です。
Notebookの実装説明は主要C++経路と `docs/legged_control/` の照合結果に基づきます。
OCS2本体はworkspaceに無いため、`LeggedRobotDynamicsAD` の完全な成分式など、
未照合の箇所は断定していません。

上流commitはROS1/OCS2原実装です。project所有の `src/legged_control_mujoco/` は
gait/state/input/WBC/hybrid-command契約をMuJoCoへ接続するadapterですが、
**OCS2 SQPではありません**。瞬時force plannerとacceleration-level WBCへ置換した実行境界であり、
Notebook 14の結果は上流ROS1/OCS2や実機A1の性能主張ではありません。
このcurriculumとbenchmarkはQuadruped-PyMPCを使用しません。

`build_notebooks.py` は教材の再生成用です。Notebookを直接編集した後に実行すると上書きするため、
生成元を更新してから実行してください。
