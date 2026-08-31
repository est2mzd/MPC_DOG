


# Squad-SDK のためのREADME

 - [README](./docs/quad_sdk_environment_and_step01.md)
 - [PyMPCとSquad-SDKの使い分け](./docs/quad_sdk_pympc_selection_and_distribution.md)

## Quad-SDK

 - [Step 01 検証記録(経緯・実験ログ)](./docs/quad_sdk_step01_investigation.md)
 - [Step 01 変更点と実行方法(要点まとめ)](./docs/quad_sdk_step01_changes_and_usage.md)
 - [記録ハーネス(quadsdk_step01_baseline.py)の構造説明](./docs/quadsdk_step01_baseline_py_structure.md)
 - [Step 01 の制御パイプライン(map → sensing → MPC → WBC のノード構成)](./agent_reports/quadsdk_step01_control_pipeline.md)
 - [Step 01 の地形マップ(map)の作り方とデータ構造(事実と推測を分離)](./agent_reports/quadsdk_step01_terrain_map.md)
 - [Step 01 のセンシング(状態推定)の仕組みとデータ構造(事実と推測を分離)](./agent_reports/quadsdk_step01_sensing.md)
 - [Step 01 の MPC(NMPC)の理論・コスト・制約・最適化とパラメータ(事実と推測を分離)](./agent_reports/quadsdk_step01_mpc.md)
 - [Step 01 の WBC(脚コントローラ/逆動力学)の理論・コード・パラメータ(事実と推測を分離)](./agent_reports/quadsdk_step01_wbc.md)
 - [Step 01 の GAIT(歩容)と MPC の関係 — 理論式・コード(事実と推測を分離)](./agent_reports/quadsdk_step01_gait_and_mpc.md)
 - [NMPC の simple モデルと complex モデルの差分 / MPC でできることの違い(事実と推測を分離)](./agent_reports/quadsdk_step01_mpc_simple_vs_complex.md)
 - [simple モデルで地形対応(高さ考慮の足場選び・穴超え)はどこまでできるか(事実と推測を分離)](./agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md)
 - [go2 の寸法・質量・関節・パラメータ(Quad-SDK モデル + 公称スペック)](./agent_reports/quadsdk_go2_dimensions_and_params.md)
 - [Step 03_1m / 04_1m:1m 深の穴を「足を入れずに」複数本連続で渡る(成功／歩容をクロールに調整・大学院初心者向け解説つき)](./docs/steps/step_03_04_1m_quadsdk_gap_crossing.md)

## Quadruped-PyMPC

 - [環境構築(acadosビルド・インストール)と実行方法](./docs/pympc_step01_changes_and_usage.md)
 - [Step 01 検証記録(経緯)](./docs/steps/step_01_reference_baseline.md)
 - [記録ハーネス(step_01_baseline.py)の構造説明](./docs/step_01_baseline_py_structure.md)
 - [Step 02 検証記録(平面マップ・歩容周波数と前進速度／成功)](./docs/steps/step_02_frequency.md)
 - [Step 03 検証記録(前進方向に並ぶ穴／轍を落ちずに越える・大学院初心者向け解説つき／成功)](./docs/steps/step_03_gap_crossing.md)
 - [Step 04 検証記録(穴の間隔を 1.5 m に詰めて同様／大学院初心者向け解説つき／成功)](./docs/steps/step_04_gap_crossing_1p5m.md)