# legged_control 理論・コード学習Notebook

大学院の初心者が `qiayuanliao/legged_control` を、完成したROSシステムとして眺めるのではなく、
数式・データ契約・小さな数値実験・C++対応箇所へ分解して学ぶ教材です。

照合した上流: commit `a7f381c0367e98e31c01336e678eef47e304d40d`（2025-02-13、master）

## 特徴

- `00` から `15` まで順番に読む
- NumPy / SciPy / Matplotlibだけで理論実験を再実行可能
- ROS / OCS2 / Gazebo / Unitreeが必要な実装事実と、教育用縮約実験を明確に区別
- 背景・目的・結論、ASCIIデータフロー、数式、コメント付きblock codeを接続
- `13` は4秒のequation-level proxy、`14` はproject adapter保存結果の検証だけ
- `15` はROS1 baseline hashとROS2 migration parityのfail-closed契約
- 全code cellの各実行行に日本語の `背景:` / `目的:`、式・変換行に `数式:` を自動付与
- 各章末にチューニング・変更時の観測項目を記載
- 詳細な実装監査は `../docs/legged_control/` を正本として参照

## 起動

```bash
uv sync --extra workshop
uv run jupyter lab notebook_legged/
```

Notebook 14は既存JSON/CSV/GIFを読むだけで、benchmarkを再生成しません。保存済みdataは
project adapterの挙動だけを示し、上流stackやROS2の性能証拠にはなりません。

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
| 14 | `14_a1_mujoco_benchmark_30_scenarios.ipynb` | project adapter保存結果の完全性検証 |
| 15 | `15_ros_migration_logic_parity.ipynb` | ROS1 baseline凍結とROS2 parity fail-closed契約 |

## 事実の境界

上流C++は `external/legged_control/` に上記commitでclone済みですが、gitignore対象です。
Notebookの実装説明は主要C++経路と `docs/legged_control/` の照合結果に基づきます。
OCS2本体はworkspaceに無いため、`LeggedRobotDynamicsAD` の完全な成分式など、
未照合の箇所は断定していません。

上流commitはROS1/OCS2原実装です。project所有の `src/legged_control_mujoco/` は
gait/state/input/WBC/hybrid-command契約をMuJoCoへ接続するadapterですが、
**OCS2 SQPではありません**。瞬時force plannerとacceleration-level WBCへ置換した実行境界であり、
元のestimator/hardware pathもありません。Notebook 14の結果は上流ROS1/OCS2や実機A1の性能主張ではありません。
このcurriculumとbenchmarkはQuadruped-PyMPCを使用しません。

**ROS2 portは作成・compile・実行されていません。ROS2 parityは NOT VERIFIED / FAIL-CLOSEDです。**
制御ロジック保存にはoriginal ROS1 Noetic stackを使い、将来のROS2 wrapperはNotebook 15の
reference/gait/observation/policy/WBC/torque golden traceを全て通過させる必要があります。
`ros1_logic_baseline_manifest.json` のhash一致はROS1 baselineを凍結するだけで、ROS2 parityを証明しません。

annotationとNotebook形式の検査:

```bash
uv run python notebook_legged/validate_notebook_annotations.py
```

`build_notebooks.py` は教材の再生成用です。Notebookを直接編集した後に実行すると上書きするため、
生成元を更新してから実行してください。
