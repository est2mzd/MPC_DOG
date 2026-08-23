# Conversation Coverage Map

## 1. 目的

本ファイルは、この学習資料作成までの会話で扱った論点が、どのMarkdownへ統合されたかを確認するための監査表である。本文と同じ説明を再掲せず、各情報の正本だけを示す。

## 2. 論点と収録先

| 会話で扱った論点 | 正本ファイル | 主な節 |
|---|---|---|
| 確認コミットとgym-quadruped版 | `00_README.md` | 対象コード |
| MuJoCo Go2の自由度・関節構造 | `01_MuJoCo_Go2_Plant_Model.md` | 自由度と状態ベクトル |
| `nq/nv/nu`と`actuator_ctrlrange` | `01` | 自由度、アクチュエータ |
| `qpos`/`qvel` indexと前後thigh可動域 | `01` | 自由度と状態ベクトル |
| 質量・慣性・全身運動方程式 | `01` | 全身運動方程式、質量と慣性 |
| XML質量15.206と`config.mass`15.019 | `01` | 質量と慣性 |
| Torque actuatorと関節PD不在 | `01`, `11` | アクチュエータ、PDなしで立つ理由 |
| 足`condim=6`と楕円錐、reset時摩擦上書き | `01` | 足と接触 |
| Visual meshとCollision geom | `01` | VisualとCollision |
| XMLセンサ16個と標準ループ未使用 | `01` | センサ |
| MJX経路が本スタックにないこと | `01`, Appendix F | MJX版、Plant / XML |
| Menagerieは上流参照で実行時未ロード | `00`, `01`, Appendix E §17 | 対象コード、結論 |
| ユーザー指令から歩行まで | `02` | 全体フロー、境界表、分割Mermaid、最終確認 |
| 処理周期100 Hz/500 Hz | `02`, `09` | `02` §5 が周期の正本 |
| `_sample_ref_vel`とキー入力の2経路 | `03` | 指令の2経路 |
| `simulation_params['mode']`未使用 | `03` | 結論 |
| キーボードから目標速度生成 | `03` | キー入力 |
| Heading/World frame | `03`, Appendix A | 座標変換、変数辞書 |
| Velocity Modulator | `03` | 安全補正 |
| `ref_state`実キーと地形回転後速度 | `03` | 地形回転と参照状態 |
| 標準構成にPlannerがないこと | `02`, `03`, `13` | 未実装機能、Planner接続 |
| Trotの対角脚と位相 | `04` | Trot |
| `pgg.run`とlookaheadの二重呼び出し | `04` | 2回の`run` |
| Step frequency、Duty factor | `04`, `12` | Gaitパラメータ、Timing |
| `contact_sequence`の意味 | `04`, `08` | 出力、固定Schedule OCP |
| `start_and_stop`が標準で無効 | `04`, `16` | 速度指令との関係、無効経路 |
| MPCが逆相を出せない理由 | `08` | 接触Gate、acados parameter |
| 接地Scheduleが数式に必要な理由 | `08` | 並進・回転・足位置Gate |
| 遊脚GRFの出力Mask | `09` | §6 段3 |
| 遊脚GRFのOCP内部（等式ゼロではない） | `09` | §6 段1–2。設計意図だけ Appendix F |
| Mixed-integer/Contact-implicitとの違い | `08` | Schedule不要となる別方式 |
| TerrainEstimator | `05` | TerrainEstimator |
| lift-off / touch-down更新 | `05` | Lift-off / Touch-down |
| Foothold Reference Generator | `05` | Nominal Foothold |
| 速度から着地点を作る式 | `05` | 速度・誤差補正 |
| VFA、Heightmap、blind | `05`, `13` | 地形適応、実現可能性 |
| Centroidal/SRBD状態・入力 | `06` | 状態、入力 |
| MuJoCo PlantとMPCモデルの差 | `01`, `06` | モデル分担、省略要素 |
| MPCコスト・重み | `07`, `14` | 定式化、チューニング |
| 摩擦錐・Foothold・安定制約 | `07` | 各制約 |
| Soft/Hard constraint | `07`, `14` | Slack、Penalty |
| acados、CasADi、SQP/RTI/DDP | `07` | Solver |
| `perform_scaling`と遊脚teleport | `09` | 求解前処理 |
| MPCの先頭入力だけを使用 | `09` | Receding Horizon |
| MPC出力GRF・Foothold | `09` | 外部出力 |
| `nmpc_predicted_state`が`(24,)` | `09`, Appendix A | 外部出力 |
| 立脚`-J.T @ F` | `10` | 立脚制御 |
| Swing Cartesian PD | `10` | Swing軌道、追従 |
| 摩擦補償`qfrc_passive` | `10` | 摩擦補償 |
| ESD / 関節空間Swingが標準で無効 | `10`, `16` | Early stance、無効経路 |
| WBInterfaceが厳密なQP-WBCではない | `10`, Appendix E | 立脚制御、訂正事項 |
| IKと関節PD目標 | `10`, `11` | IK、関節PD |
| clipが`actuator_ctrlrange` | `11`, `01` | Saturation |
| MPC GRFとMuJoCo実GRF | `11` | 目標GRFと実GRF |
| 定常速度時の力と運動学 | `12` | 定常速度と加速度 |
| 0.5 m/sと5.0 m/sの違い | `12` | 実現可能速度域 |
| 2.5 mの意味の訂正 | `12`, Appendix E | 接地点間隔 |
| 周波数候補評価の正確な意味 | `12` | 周波数候補評価 |
| 地形とGait Timingの不整合 | `13` | 3集合、対応順序 |
| 安全Footholdへ届かない場合 | `13` | Timing可到達集合 |
| 速度・Gait・Timing再計画 | `13` | Planner出力 |
| ADAS MPCとのチューニング比較 | `14` | ADASとの対応 |
| ユーザー調整項目一覧 | `14`, Appendix C | 調整表、Parameter Index |
| 自動チューニング可能範囲 | `15` | Outer-loop、Domain randomization |
| GPU Sampling MPCと重み探索の違い | `15` | 既存自動化、Outer-loop |
| `joints_pos`がqvel index | `16`, Appendix A | Call graph、変数辞書 |
| 標準設定の無効経路 | `16` | 無効経路 |
| コード・関数との対応 | `16`, Appendix D | Code map、Function index |
| Cursorへ分析させる方法 | `17` | Context、11段手順、更新規則 |
| 学習・研究実験 | `18` | 実験段階、評価指標 |
| Baseline固定とログ・テスト追加計画 | `18` | Baseline固定 |
| 会話中の訂正事項 | Appendix E | 全節 |
| 公開コードだけで未確定の事項 | Appendix F | Open questions |
| wrapper HEAD と PyMPC tree の分離 | `00` | 対象コード |
| 遊脚GRFの3段（力学/摩擦常時/Mask） | `09` §6 | 遊脚GRF |
| `s\equiv0`、omega積分未接続 | `06` | 足位置、状態 |
| 終端 `W_e=Q`、Focchi20式、失敗時前回GRF | `07` | Cost、摩擦、Failure |
| GRF rate は `input_rates` 専用 | `14`, C | 調整表 |
| Reflex既定 `False` | C, `16` | 無効経路 |
| 周波数候補のpenaltyとFoothold非再計算 | `12` §6 | 周波数候補評価 |
| 実GRFはviewer専用 | A, `11` | 目標GRFと実GRF |
| 標準ONの自動化は慣性とfoothold optのみ | `15` §2 | 既存の自動化 |
| 実験14段階と公平比較 | `18` | 実験段階、§4 |
| 解析ログ（調査証跡） | `analysis-logs/` | 本文へ転記しない |
| 学習3経路と26ファイル表 | `00` | §5–6 |
| コード分析順・実験開始順 | `00` | §7–8 |
| 検証コマンド | `00`, `scripts/README.md` | `verify_study_docs.py` |
| Cursor 11段手順 | `17` | §3 |
| 最終データフロー22段 | `02` | 全体フロー、境界表、§6 |
| `mpc_frequency` は `simulation_params` | C, E §27 | パラメータ索引 |

## 3. 欠損確認方法

新しい論点を追加した場合は、次のいずれかを行う。

1. 既存の正本章へ追記し、この表へ行を追加する。
2. 既存章の責務を超える場合だけ、新しいMarkdownを作る。
3. 実装で確定できない場合は`appendices/F_Open_Questions.md`へ追加する。
4. 過去の説明を訂正した場合は本文を正し、理由だけを`appendices/E_Corrections_and_Clarifications.md`へ残す。

## 4. Cursor用監査プロンプト

```text
19_Conversation_Coverage_Map.mdと全Markdownを比較してください。
各行の正本ファイルに実際に説明が存在するか確認し、
重複記述、リンク切れ、定義の矛盾、未収録論点を表にしてください。
内容の修正は行わず、まず監査結果だけを返してください。
```
