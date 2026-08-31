


# Squad-SDK のためのREADME

 - [README](./agent_reports/step01/quad_sdk_environment_and_step01.md)
 - [PyMPCとSquad-SDKの使い分け](./agent_reports/step01/quad_sdk_pympc_selection_and_distribution.md)

## Quad-SDK

 - [Step 01 検証記録(経緯・実験ログ)](./agent_reports/step01/quad_sdk_step01_investigation.md)
 - [Step 01 変更点と実行方法(要点まとめ)](./agent_reports/step01/quad_sdk_step01_changes_and_usage.md)
 - [記録ハーネス(quadsdk_step01_baseline.py)の構造説明](./agent_reports/step01/quadsdk_step01_baseline_py_structure.md)
 - [Step 01 の制御パイプライン(map → sensing → MPC → WBC のノード構成)](./agent_reports/quadsdk_step01_control_pipeline.md)
 - [Step 01 の地形マップ(map)の作り方とデータ構造(事実と推測を分離)](./agent_reports/quadsdk_step01_terrain_map.md)
 - [Step 01 のセンシング(状態推定)の仕組みとデータ構造(事実と推測を分離)](./agent_reports/quadsdk_step01_sensing.md)
 - [Step 01 の MPC(NMPC)の理論・コスト・制約・最適化とパラメータ(事実と推測を分離)](./agent_reports/quadsdk_step01_mpc.md)
 - [Step 01 の WBC(脚コントローラ/逆動力学)の理論・コード・パラメータ(事実と推測を分離)](./agent_reports/quadsdk_step01_wbc.md)
 - [Step 01 の GAIT(歩容)と MPC の関係 — 理論式・コード(事実と推測を分離)](./agent_reports/quadsdk_step01_gait_and_mpc.md)
 - [NMPC の simple モデルと complex モデルの差分 / MPC でできることの違い(事実と推測を分離)](./agent_reports/quadsdk_step01_mpc_simple_vs_complex.md)
 - [simple モデルで地形対応(高さ考慮の足場選び・穴超え)はどこまでできるか(事実と推測を分離)](./agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md)
 - [go2 の寸法・質量・関節・パラメータ(Quad-SDK モデル + 公称スペック)](./agent_reports/quadsdk_go2_dimensions_and_params.md)
 - [穴対応:Foot Placement と NMPC 連携のコード解析(資料⇔コード照合表・terrain map の式・足場計画の入出力・NMPC への受け渡し・資料主張の再判定・改善フェーズ計画)](./agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md)
 - [穴対応 Foot Placement 改善:フェーズ実施ログ(何をどのコミットで変えたか。Phase 0 資料訂正 / Phase 1 FootholdResult 診断値)](./agent_reports/quadsdk_gap_foothold_phase_progress.md)
 - [Step 03_1m / 04_1m:1m 深の穴を「足を入れずに」複数本連続で渡る(成功／歩容をクロールに調整・大学院初心者向け解説つき)](./agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md)
 - [Step 03_1m / 04_1m(gbpl 版):global_body_planner で穴を渡る試み + Quad-SDK 公式マニュアルに基づく正しい設定方法 + センシング→foot plan→MPC→WBC→トルクの工程別ボトルネック分析(穴 1 本は跳んで可・連続区間は未達)](./agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md)

### 溝渡りの実行例(1 m 深・0.3 m 幅の溝を、足を溝に入れずに連続で渡る／`reference:=twist` + クロール歩容)

固定カメラ。凸条の目盛りは 5 m 間隔。詳細・CSV 根拠は
[Step 03_1m / 04_1m の検証記録](./agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md)。

| step03(溝の間隔 2.0 m、0.15 m/s、12–35 s 切り抜き) | step04(溝の間隔 1.5 m、0.3 m/s、15–40 s 切り抜き) |
|---|---|
| ![step03 溝渡り](./artifacts/gifs/quadsdk_step03_1m_v0p15_12to35s.gif) | ![step04 溝渡り](./artifacts/gifs/quadsdk_step04_1m_v0p3_15to40s.gif) |

## Quadruped-PyMPC

 - [環境構築(acadosビルド・インストール)と実行方法](./agent_reports/step01/pympc_step01_changes_and_usage.md)
 - [Step 01 検証記録(経緯)](./agent_reports/steps/step_01_reference_baseline.md)
 - [記録ハーネス(step_01_baseline.py)の構造説明](./agent_reports/step01/step_01_baseline_py_structure.md)
 - [Step 02 検証記録(平面マップ・歩容周波数と前進速度／成功)](./agent_reports/steps/step_02_frequency.md)
 - [Step 03 検証記録(前進方向に並ぶ穴／轍を落ちずに越える・大学院初心者向け解説つき／成功)](./agent_reports/steps/step_03_gap_crossing.md)
 - [Step 04 検証記録(穴の間隔を 1.5 m に詰めて同様／大学院初心者向け解説つき／成功)](./agent_reports/steps/step_04_gap_crossing_1p5m.md)