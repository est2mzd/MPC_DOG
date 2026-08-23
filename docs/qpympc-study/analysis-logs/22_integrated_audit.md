# Log 22: `docs/qpympc-study/` 統合監査

対応プロンプト: 全解析ログと学習資料 00–19・appendices A–F の統合監査。
記録日: 2026-08-23。**学習資料・制御コードは未修正。本ファイルは監査だけ。**

正本はコード。判定は 正しい / 不完全 / 誤り / コードから確認不能。
根拠ログ: [01](01_baseline.md)–[21](21_experiment_research_roadmap.md)。既存部分監査は [02](02_readme_comparison.md), [04](04_docs_02_16_D_comparison.md)。

---

## 1. 問題一覧

重要度: Critical = 閉ループの制御構造を誤解させる。High = 変数・Index・Shape・単位・Frame・数式の誤り。Medium = 説明不足、Optional/Default混同。Low = 表記・リンク・構成・重複。

| ID | 重要度 | 資料 | 節 | 現在の記載 | コード上の事実 | 必要な修正 | 根拠 |
|---|---|---|---|---|---|---|---|
| A01 | Critical | `08`, `09`, `16`, E §4, F | 遊脚GRF | OCP内が等式ゼロかは「未再検証」「Fへ」 | 等式ゼロではない。力学は \(c_i F_i\)。摩擦は全脚常時 \(F_z\in[0,mg]\)。yref 遊脚 \(F_z=0\)。出力 \(F^{cmd}=c_0 F^{MPC}\) | 「未確定」を削除。3段（力学Gate / 制約非ゲート / 出力Mask）を09正本に書く。E §4を「当時未検証、のち非ゼロと確定」へ | log 10, 11, 12 |
| A02 | Critical | `06` §5, `08` §3 | 接触Gate | 「遊脚の力は力学に寄与しない」で止まる | 寄与しないのは \(\dot v,\dot\omega\) だけ。OCPは遊脚GRFにも摩擦を課し、コストも乗る | 力学Gateと制約・コストを分離して書く | log 09, 10 |
| A03 | Critical | `15` §2 | 既存の自動化 | 慣性再計算、foothold opt、batched freq、GPU Sampling、integral、wrench、residual を並列列挙 | 標準ONは慣性再計算と foothold position opt のみ。他はオフ、未渡し、または別`type`。residualは適応同定ではない | 標準ON / 実装あり標準OFF / 未実装 の3列に分ける | log 18 |
| A04 | High | `00` §2, `01` §12, `18` §8.1, E §15 | 対象コード | PyMPC commit = `3adfad9` | `3adfad9` は wrapper `mpc_dog` HEAD。`external/Quadruped-PyMPC` に `.git` なし。zip参考は `cc145a2` | wrapper と PyMPC tree を分離。18のBaseline表を直す | log 01, 02, 21 |
| A05 | High | `07` §9 | Failure | 「前回GRF**や**基準鉛直力」 | `status in {1,4}` で `previous_optimal_GRF` のあと `reset()`。`mg/n_s` 代入は直後に上書きされ死文 | 「または」を削除。死文をEへ | log 10, 12 |
| A06 | High | `07` §2 | Cost | 終端 \(\\|x_N-x_N^{ref}\\|_{Q_N}^2\) | `LINEAR_LS`。`W_e=Q`。別 \(Q_N\) なし | \(Q_N=Q\) と書く。Index表は07へ | log 10, 17 |
| A07 | High | `07` §5 | 摩擦 | \(\|F_x\|\le\mu F_z\) と \(F_z\) 箱 | Focchi線形4辺 + \(F_z\in[0,mg]\)。接触で無効化しない | 4不等式と「全脚常時」を書く | log 10 |
| A08 | High | `14` §2, C | GRF rate | Bの調整項目 | nominal に \(R_{\dot F}\) なし。`type='input_rates'` 専用 | 表から外すか「専用コントローラ」と明記 | log 10, 17 |
| A09 | High | C | Reflex | 既定例 `tracking` | ディスク `reflex_trigger_mode=False` | `False`。`tracking` は有効時のモード名 | log 17 |
| A10 | High | `12` §6 | 周波数最適化 | \(f^*=\arg\min J_{MPC}\)。候補ごとに Stance・接地列・Foothold・コスト | 標準OFF。評価は接触列だけ作り直し。Footholdは候補ごと再計算しない。目的は \(J_{MPC}+\)周波数penalty。候補に標準 1.35 が無い | 標準OFF、評価内容、penalty、候補集合を書く | log 15 |
| A11 | High | `06` 記号 | omega積分 | 記号定義あり | `self.states` に未接続 | 記号を削除するか「未使用」 | log 09 |
| A12 | High | A | 実GRF | 未掲載 | `feet_contact_state(..., True)`。viewer専用。MPCへ戻さない | \(F^{act}\) / \(\lambda\) 行を追加 | log 14, 20 |
| A13 | Medium | `04` §4 | Trot行列 | 概念例。duty=0.74 の4脚overlapなし | 実列は overlap あり（\(d=0.74>0.5\)） | 「overlapあり」を本文に1文。実列はコード出力 | log 07 |
| A14 | Medium | `07` §7 | Slack | 一般論として導入できる | 標準では foothold/stability 未構築。slack変数なし | 「標準経路では未構築」 | log 10 |
| A15 | Medium | `07` §4, C | 重み | 場所が config に見える。数値一部欠 | 実体は `set_weight()`。Cは数値の一部のみ。Foot vel は `small` | 数値表を07へ。Cは参照。`mu`,`ref_z`,hip_offset,mpc_frequency をCへ | log 10, 17 |
| A16 | Medium | `05` §1, §6 | MPC最適化 | 「MPCがさらに最適化する」 | 足位置Cost。地形安全集合の保証ではない。標準 foothold制約OFF | 「位置Costであり \(\mathcal S\) 保証ではない」 | log 08, 16 |
| A17 | Medium | `13` §1–2 | 3集合 | 必要集合として提示。§6で同時最適化しないと書く | \(\mathcal S\cap\mathcal R_{kin}\cap\mathcal R_{time}\) を積集合にする関数はない。到達不能でも目標をそのまま使う | §1で「理論。標準未実装」を先に書く | log 16 |
| A18 | Medium | `12` §3–4 | 5 m/s例 | Duty 0.65。周波数↑で足先速度↑し得る | 標準Trotは 0.74。水平平均 \(\bar v_{xy}\) は \(f\) 非依存。鉛直・ピーク・clip時だけ変わり得る | 標準dutyを併記。水平/鉛直を分ける | log 15 |
| A19 | Medium | `10` §5 | Swing PD | 2段を書くが「PD二重」と明示しない | 段1: \(J^\top(K_p e+K_d\dot e)\)。段2: FB線形化の \(\ddot p_{cmd}\) にも同じPD | 二重適用を明示 | log 13 |
| A20 | Medium | `10` / D | Swing索引 | ID項・摩擦補償・`update_swing_time` が薄い | 全脚先に `-J.T@F`、遊脚上書き、全脚 `tau-=passive` | 10に順序を残す。Dに関数行 | log 13 |
| A21 | Medium | `16` | 関節PD | コメントアウトとある | 実装はあるが無効 | 「実装あり・標準無効」で十分。現状は不完全止まり | log 04 |
| A22 | Medium | `18` §2 | 実験段階 | 10段階。摩擦とDR、段差と穴が混在 | 監査済み仕様は14段階（log 21） | 18を21に合わせて分割するか、21を正本とリンク | log 21 |
| A23 | Medium | `18` §8.1 | Baseline地形 | `friction_coeff=(0.5,1.0)`、scene未固定 | 範囲乱択とディスク `perlin` を基準にすると段階3と不整地が交ざる | 歩行Baselineは `flat` + 摩擦一点（log 21） | log 21 |
| A24 | Medium | `02` mermaid | 型 | 概ね一致。一部shape省略。`N m` | 境界表は正しい | edgeを表へ寄せる。単位は `N·m` | log 04 |
| A25 | Medium | D / wrapper | 観測キー | Dは関数索引。観測typoは本文で分離済み | `get_obs` が `ref_foot_FL_constraints` を読む。実キーは `ref_foot_constraints_FL`。制御経路は正しい | 16またはAに「観測分岐のみtypo」 | log 04, 06 |
| A26 | Medium | `19` | カバレッジ | 学習資料作成時までの論点だけ | 以降の確定（s≡0、死文fallback、Sampling Cost差、14段階、ログ設計）が無い | 19へ行追加。正本はログ→本文反映後に章 | 本ログ |
| A27 | Medium | F | 複数項 | 未確定のまま残している | 遊脚OCP、`s≡0`、wrench未渡しは確定済み | 確定項をFから外しEへ移す（§5） | log 09, 10 |
| A28 | Low | `00` mermaid | 概念図 | 型なし | 意図どおり。正本は02 | 変更不要。00に「概念」と既にある | — |
| A29 | Low | `01`/`11` | 全身式 | 同じ \(M\ddot q+h=...\) を再掲 | 重複 | 11は「01へリンク + clip/actionだけ」 | §4 |
| A30 | Low | `00` §8 | Cursor運用 | ノートを更新すると書く | 解析ログ運用が本会話の実体 | 17に「監査中は logs のみ」を1行 | — |
| A31 | Low | B | 足Gate正本 | 正本を08にしている | 式の完全形（\(1-s\)）は06 | Bの足Gateを06へ。08は結合の説明 | — |
| A32 | Low | C | 欠落キー | μ, `ref_z`, hip_offset, mpc_frequency なし | 標準で効く | Cへ追加 | log 17 |
| A33 | Low | `17` §7 | Commit記録 | 「コードCommitをMarkdownへ」 | PyMPCにgitが無い | wrapper HEAD + treeハッシュ | log 01 |

本文がすでに正しい主要骨格（指令は速度、Trot位相はMPCが選ばない、`-J.T@F`、目標GRF≠実GRF、標準にPlannerなし、TerrainEstimatorは先頭、`ref_state`キー、`joints_pos`はindex）は再掲しない。

---

## 2. 入出力整合

前段出力と後段入力が同じオブジェクトなら Shape/単位/Frame は「一致」。意図的変換がある行は「変換あり」と書き、変換を右に示す。

| 境界 | 前段出力 | 後段入力 | Shape一致 | 単位一致 | Frame一致 | 更新周期 | 判定 |
|---|---|---|---|---|---|---|---|
| `_sample_ref_vel` → `target_base_vel` | `_ref_base_lin_vel_H` `(3,)`, `_ref_base_ang_yaw_dot` | 同名 | はい | はい | はい（H / z） | reset。キーはイベント | 一致 |
| `target_base_vel` → `compute_actions` | `ref_base_lin_vel` `(3,)` W, `ref_base_ang_vel` `(3,)` | 同名 | はい | はい | **変換あり** \(v_W=R_H^W v_H\)。\(\omega\) は回転しない `[0,0,\dot\psi]` | 500 Hzで読む | 一致（H→Wは文書化済み） |
| getters → `compute_actions` | `com`, `base_*`, `qpos`(19,), `qvel`(18,), `feet_*`, `J`(3,18), `inertia`(3,3) | 同名 + flatten | ほぼ | はい | 角速度B解釈はF | 500 Hz | 一致。`inertia` は `(3,3)`→`(9,)` |
| `simulation.py` → wrapper | `joints_pos` | `state_current['joint_*']` | はい | **名前不一致** | なし | 500 Hz | **不一致（名前）**。中身は `legs_qvel_idx`。nominal未使用。A/16は記載済 |
| VM | `ref_base_*` | 同配列を上書き | はい | はい | Wのまま | 500 Hz | 一致。値だけ0にし得る |
| TerrainEstimator → 参照組立 | roll/pitch/height | 速度回転、`ref_orientation`, `ref_position[2]` | はい | はい | rollは常に0 | 500 Hz | 一致。入力足は `lift_off` |
| PGG `run(0.002)` → 位相 | `contact` 戻り | **捨てる** | — | — | — | 500 Hz | 意図的。位相だけ残す |
| `compute_contact_sequence` → MPC / WBC | `(4,12)` | `p[0:4]` 各段、`current_contact=C[:,0]` | はい | はい（0/1） | なし | 500 Hz生成、MPC 100 Hz | 一致。lookaheadは位相復元 |
| FRG → `ref_state` | `ref_feet_pos.*` `(3,)` | `ref_foot_*` `(1,3)` | **変換あり** reshape | はい | W。zはlift-off z | 500 Hz | 一致（reshape） |
| FRG速度 | 地形回転**前** `ref_base_lin_vel[0:2]` | `compute_footholds_reference` | はい | はい | W→内部H | 500 Hz | 一致。MPC速度とは**別スナップショット** |
| 地形回転 → `ref_linear_velocity` | 回転後 `(3,)` | yref 速度 | はい | はい | 地形付きW | 500 Hz | 一致。Footholdより後 |
| `ref_position` | `[0,0,z]` | yref 位置 | はい | はい | xyは非追従 | 500 Hz | 一致。Qのxy重み0 |
| `update_*` → `compute_control` | `state_current`, `ref_state`, `contact_sequence` | 同dict | はい | はい | 主にW、ωはB | 500 Hz / 読むのは100 Hz | 一致。キー名は制御経路で正しい |
| wrapper観測 | `get_obs` | `ref_feet_constraints` | **キー不一致** | — | — | 観測時 | **不一致（観測のみ）**。`ref_foot_FL_constraints` vs `ref_foot_constraints_FL` |
| `perform_scaling` | World 状態/参照 | 原点相対 | 同shape | はい | **変換あり** CoM原点 | 100 Hz | 一致（内部）。decenterして戻す |
| 遊脚teleport | 現在足 | `ref_foot_*[0]` で x0 足を置換 | はい | はい | W相対 | 100 Hz | 意図的。OCP初期だけ |
| Solver `u0` → interface | `control[12:]` `(12,)` | 脚ごと `(3,)` | 分割 | N | W | 100 Hz | 一致 |
| Mask | \(F^{MPC}\) | \(F^{cmd}=c_0 F^{MPC}\) | はい | はい | W | 100 Hz | 一致。遊脚指令0 |
| `nmpc_footholds` | 予測/参照 | Swing終点・立脚は現在足 | はい | はい | W。制約OFF時 H で ±0.15 clip | 100 Hz hold | 一致 |
| `nmpc_predicted_state` | solve後 `(24,)` | IK行はコメントアウト | 初期プレースホルダ `(12,)` | — | — | 100 Hz | **初期shape不一致**。tau未使用 |
| Stance | `nmpc_GRFs`, `J[:,qvel_idx]` `(3,3)` | \(\tau=-J^\top F\) | はい | N→N·m | W→関節 | 500 Hz。Jは毎周期 | 一致。先に全脚適用 |
| Swing | `p_d`, `J`, `M`(3,3), `h` | 遊脚が `tau` 上書き | はい | はい | W / 関節 | 500 Hz | 一致。PD二重は内部 |
| Passive | `legs_qfrc_passive` | `tau -=` 全脚 | はい | はい | 関節 | 500 Hz | 一致 |
| IK | `des_foot_pos` | `des_joints_*` | はい | はい | 関節 | 500 Hz計算 | **下流不一致**。標準simはプラント未使用 |
| clip | `tau.*` | `0.9*ctrlrange` | はい | はい | アクチュエータ | 500 Hz | 一致。飽和フラグは未ログ |
| 組立 | `tau` LegsAttr | `action`(12,) | 連結 | はい | FL,FR,RL,RR×hip,thigh,calf | 500 Hz | 一致 |
| `env.step` | `action` | `mjData.ctrl` | はい | はい | 同上 | 1 step = 1 `mj_step` | 一致 |
| Plant → 次周期 | `qpos`,`qvel`,接触 | getters | はい | はい | §01 | 500 Hz | 一致 |
| 実接触 / 実GRF | `feet_contact_state` | **トルク計算に未使用** | — | — | 接触frame→W | viewer時 | **ループ非接続**。計画接触だけが切替 |
| `current_contact` vs 実接地 | PGG | WBC/Mask | 同shape | 0/1 | なし | 500 Hz | **意味不一致**。予定であり計測ではない |

変換の要約（不一致ではないが追跡必須）:

1. \(v_H^{ref}\xrightarrow{R_H^W}v_W^{ref}\)。
2. Footholdは回転前xy、MPC速度は回転後。
3. `ref_feet_pos (3,)` → `ref_foot_* (1,3)`。
4. World → scaling原点 → decenter。
5. \(F^{MPC}\xrightarrow{\times c_0}F^{cmd}\)。
6. \(J\in\mathbb R^{3\times18}\) → 脚3列。
7. `tau` 4×`(3,)` → `action (12,)` → clip。

---

## 3. 数式整合

| 数式 | 正本資料 | 対応コード | 変数対応 | 符号 | Frame | 判定 |
|---|---|---|---|---|---|---|
| MuJoCo全身 \(M\ddot q+h=S^\top\tau+J_c^\top\lambda\) | `01` §3（`11`は再掲） | `mj_step`。代入は `mjData.ctrl=action` | \(\tau\)=`ctrl`。\(\lambda\)=接触ソルバー。MPC GRFではない | 概念式として妥当 | 混在 | 正しい（エンジン内部の厳密形は未展開） |
| SRBD並進 \(\dot p=v\), \(\dot v=(1/m)(\sum c_i F_i+F_{ext})+g\) | `06` §5 | `forward_dynamics` `temp=Σ F@stance + F_ext` | \(c_i\)=`stance*`。\(g=[0,0,-9.81]\) | \(+g\) で鉛直負 | W（scaling後は相対） | 正しい。`08` は \(F_{ext}\) 省略（標準0） |
| SRBD回転 \(I\dot\omega=\sum c_i(p_i-p)\times F_i+\tau_{ext}-\omega\times I\omega\) | `06` §6 | `ang_acc=I^{-1}(b_R_w @ temp2 - skew(w) I w)` | \(\omega\) はBase。トルクは \(R_{BW}\) でBodyへ | 符号一致 | 回転はB、モーメント腕はW | **不完全**。本文が \(R_{BW}\) と Euler map \(\dot\Theta=E^{-1}\omega\) を略 |
| Foot \(\dot p_i=(1-c_i)(1-s_i)v_i\) | `06` §7 | `v @ (1-c) @ (1-s)` | \(s_i\) は `1*0` → 常に0 | 一致 | W | 正しい。本文に \(s\equiv0\) と `use_foothold_optimization` が無い。`08` は \(s\) 省略 |
| MPC cost \(\sum\\|x-x^{ref}\\|_Q^2+\\|u-u^{ref}\\|_R^2+\\|x_N\\|_{Q_N}^2\) | `07` §2 | `LINEAR_LS`, `W=blkdiag(Q,R)`, `W_e=Q` | Indexは log 10 | — | scaling後 | **誤り（終端）**。\(Q_N\neq Q\) と読める。数値未掲は不完全 |
| Friction cone | `07` §5 | `create_friction_cone_constraints` 20式 | \(\mu=0.42\)。n,t,b | 線形4辺 | W GRF | **不完全**。ピラミッド近似で、接触非ゲート |
| Contact gate | `08` §3–4 | `F@stance`, `v@(1-stance)` | \(c_{i,k}=p[0:4]\) | 力学のみ | — | 力学は正しい。制約非ゲートは未記載（A02） |
| Foothold \(p_{hip}^H+(T_{st}/2)v_H^{ref}+\sqrt{h/g}(\bar v-v^{ref})+offset\) | `05` §5 | `compute_footholds_reference` | clip ±1.5 hip_height と ±0.05 m。`hip_offset=0.1` | 一致 | Hで足しWへ。z=lift-off | 正しい |
| Jacobian \(\tau=-J^\top F^{MPC}\) | `10` §3 | `tau = -J[:,idx].T @ F` | \(J\) は並進3×3 | 負号は \(F_{ee}=-F^{MPC}\) | W→関節 | 正しい |
| Swing \(\ddot p_{cmd}=\ddot p_d+K_p e+K_d\dot e\) | `10` §5 | cartesian 2段 | Kp=500, Kd=10 | PDが2回 | W / 関節 | **不完全**（二重適用） |
| \(L_{footprint}=v/f\), \(T_{st}=d/f\) | `12` §2–4 | FRGは \(L_{st}/2=v T_{st}/2\) | \(L_{footprint}\) は地面印間隔。胴体相対TDではない | — | H/W | 定義は正しい。FRGが使うのは \(L_{st}/2\)。§6実装は不完全（A10） |

`13` の3集合式は理論として正しい。標準コードは交差を実装しない（A17）。Bの「Rough-terrain」行も「推奨統合条件」と書いてあり、実装正本ではない。

---

## 4. 重複

| 論点 | 重複資料 | 推奨正本 | 他資料で残す内容 |
|---|---|---|---|
| 対象コード / commit | `00`, `01` §12, `18` §8.1, E §15 | `00`（修正後） | 01はXML。18は実験meta。Eは旧誤りの理由だけ |
| 型付きデータ契約 | `00` mermaid, `02`, A | `02` + A | 00は概念図のみ |
| 呼出順・無効経路 | `02` §5, `16`, D | `16` | 02は境界だけ。Dは1行索引 |
| 指令・`ref_state` | `02`, `03`, A | `03` | 02は1行契約 |
| Heading→World | `03`, A, B | `03` | A/Bは記号行 |
| Gait / `contact_sequence` | `04`, `08`, `16` | `04` | 08はMPCへの入れ方。16は呼出 |
| 接触Gate力学 | `06`, `08` | `06` | 08は「位相を最適化しない」意味 |
| 遊脚GRF 3段 | `08`, `09`, `16`, F | `09`（修正後） | 08は1節参照。16は1行。Fから確定項を外す |
| SRBD状態Index | `06`, `09`, A | `06` + A | 09は出力とreceding |
| Cost / 重み数値 | `07`, `14`, C | `07` | 14は症状逆引き。Cは表 |
| 摩擦錐 | `01` Plant楕円, `07` MPC | 分けたまま | 01=Plant。07=OCP。同一μと書かない |
| Receding / Mask / 周期 | `02`, `09`, `16` | 周期=`02`、Mask=`09` | 16は `% 5` 1行 |
| Stance/Swing式 | `10`, `11`, E §9 | `10` | 11はclip/action/`mj_step` |
| 全身運動式 | `01`, `11` | `01` | 11はリンク |
| `v=fL` / 2.5 m | `12`, E §5–6 | `12` | Eは旧誤解の理由 |
| 3集合 | `05` §7, `13` | `13`（理論）、実装欠如は `13` §6 を強化 | 05は「保証しない」1文 |
| 無効経路一覧 | `04`, `05`, `10`, `16` | `16` | 各章は自関数の到達不能だけ |
| 調整項目 | `14`, C, `15` | `14` | Cは索引。15はOuter |
| Baseline固定 | `00`, `18` §8, log 21 | 実験は `18`（修正後）/ 当面 log 21 | 00は版情報 |
| 実験段階 | `18`, log 21 | 反映後は `18`。今は log 21 | 18は短い表 |
| Gradient vs Sampling | `15`, `18`, F, log 19 | 方式差は log 19 → 将来 `07` 追記または独立節 | 15は「探索≠重み」1文 |
| Open questions | 本文の「未確認」、F | F | 本文は「未確認」と書いてFリンクのみ |

---

## 5. 未確認事項（断定の点検とF移設案）

### 5.1 本文が断定しているが、コードから確定できないもの

現時点で「断定のまま残っている未知」は少ない。多くはFリンク済み。

| 箇所 | 記載 | 扱い |
|---|---|---|
| `01` / A | `qvel[3:6]` をBase角速度とする | コード上の解釈と書いてF。断定ではない |
| `01` / A | `env.com` が物理CoM | 「未検証」+F。可 |
| `01` / A | `get_base_inertia` frame | F。可 |
| `09` §3.3 | wrench補償の「意図」 | F。可 |
| `12` §7 | \(\mathcal V(f,d,\ldots)\) の具体境界 | 理論。数値断定なし。可 |
| `13` §4 | 対応順序 | 推奨改善。実装事実と読める余地 → A17 |

Menagerie同一性、実機重み、実床μ、遅延は本文で断定していない。

### 5.2 本文が「未確定」と断定しているが、ログで確定済み → **Fから外す**

| F/本文の現行 | 確定事実 | 移設先 |
|---|---|---|
| F「遊脚GRFのOCP内部制約は未確定」 | 等式ゼロではない。Gate + yref0 + 出力Mask。摩擦常時 | 本文09。E §4を更新。Fからは削除 |
| F `stance_proximity` は `1*0` | 常に0 | 06注記。F削除可（確定） |
| F wrench「内部推定があるか未確認」 | wrapper未渡し。標準経路に推定器なし。`zeros(6,)` | 09。Fは「設計意図」だけ残してよい |
| E §4「この監査では未再検証」 | 再検証済み | Eを「確定: 非ゼロ」へ |

### 5.3 Fに残す（今も未知または研究）

- Go2実機の最終重み・Swing gain、実床μ、推定遅延、モータ帯域、トルク遅延、Payload
- 遊脚F=0にしない**設計意図**（事実ではなく意図）
- Solver failure時の実機安全性
- 積分離散化の妥当性、Euler高角度
- Foot velocity decision の外部利用
- `env.com` / `qvel[3:6]` / `mj_fullM` の厳密定義
- gym-quadruped XML と Menagerie / MJX の差
- XML `diaginertia` と `config.inertia` と `mj_fullM` の3者
- Plant楕円錐 vs MPC μ=0.42
- Frequency実機範囲、速度別Gait切替（自動は未実装で確定。数値範囲は未知）
- **同一Costでの** Gradient vs Sampling（現行は非同一。揃えた場合の性能は未知）
- SRBD誤差の支配要因、Residual学習、Auto-tuneのSim-to-Real、Reachability追加の計算時間

### 5.4 Fへ新規に移す案（本文から）

本文に断定はないが、Cursor課題が「未確認のように」残しているもの。

| 案 | 本文から外す文 | Fへ |
|---|---|---|
| VFAと残時間の整合 | `13` §8「実装されているか検証」 | 「未実装と確定。残時間制約なし」（log 16） |
| 安全停止 | 運用文が実装に見えるなら | simで `start_and_stop` オフ。安全Foothold連動なし |
| Outer-loop再現性 | `15` が可能なように読める部分 | tuner未実装。再現性は研究 |

---

## 6. 修正計画（未実施）

依存: Criticalを先に直さないと、Highの数式表が旧前提のままになる。

| 修正順 | 対象資料 | 修正内容 | 依存する資料 | 検証方法 |
|---:|---|---|---|---|
| 1 Critical | `09` | 遊脚GRF 3段（力学 / 摩擦常時 / Mask）を正本化。Fリンクを外す | コード `centroidal_nmpc_nominal`, `srbd_controller_interface` | log 10の7点を再読し、本文に等式ゼロと書かない |
| 1 Critical | `08` §3, §6 | 力学Gateと制約を分離。§6の「未再検証」を09へ委譲して確定文に | 09 | `c=0` でも摩擦20式が残ることを関数で確認 |
| 1 Critical | `06` §5 | 「力学にだけ寄与」と明記 | 09 | 同上 |
| 1 Critical | `16` §5 | 「OCP内等式ゼロはF」を削除し09へ | 09 | 1行差し替え |
| 1 Critical | E §4, F | Eを確定訂正に。Fから内部制約項を削除 | 09 | Fに「意図」だけ残るか確認 |
| 1 Critical | `15` §2 | ON / 標準OFF / 未実装の3列 | log 18 | `config.py` フラグと突合 |
| 2 High | `00`, `01` §12, `18` §8.1, E §15 | commit分離。18 Baselineに flat・摩擦一点・viewer・logを足す | log 01, 21 | `git rev-parse` と `ls external/.../.git` |
| 2 High | `07` §2, §5, §9 | \(W_e=Q\)、Focchi20式、fallback=前回GRF、重み数値表 | log 10 | `set_weight` と failure 分岐を再読 |
| 2 High | `14` §2–4, C | GRF rateを専用列へ。Reflex既定False。欠落キー追加 | 07, log 17 | Cの既定例を `config.py` / `set_weight` でgrep |
| 2 High | `12` §6 | 標準OFF、penalty、Foothold非再計算、候補集合 | log 15 | batched 経路が作られる条件を確認 |
| 2 High | `06` | omega積分記号削除。Index表を06かAへ | log 09 | `self.states` と記号のdiff |
| 2 High | A | \(F^{act}\)、`mjData.contact`、計画接触と実接触 | log 14, 20 | 変数表に生成元がコードにあるか |
| 3 Medium | `04` | overlap一文 | log 07 | duty=0.74 で4脚1の段があるか |
| 3 Medium | `05` §1 | MPC位置Cost ≠ 地形保証 | `13` | 制約フラグFalse |
| 3 Medium | `13` §1 | 「理論・未実装」を先頭へ。§8を「未実装と確定」 | log 16 | 交差関数が無いこと |
| 3 Medium | `10` §5 | PD二重、全脚stance先行 | log 13 | cartesian 2段の行 |
| 3 Medium | `18` §2 | 14段階へ更新するか log 21 を正本リンク | log 21 | 段階名の1:1 |
| 3 Medium | `19` | 本会話後半の確定行を追加 | 本ログ | カバレッジ表に漏れがないか |
| 3 Medium | D | 観測typo、`update_swing_time` | `16`, `10` | 関数名grep |
| 4 Low | `11` | 全身式を01リンクに短縮 | `01` | 式の二重定義が消えるか |
| 4 Low | B | 足Gate正本を06へ | `06`,`08` | Bのリンク先 |
| 4 Low | `00`/`17` | 監査中は analysis-logs、PyMPCはgit外 | — | — |
| 5 Link/Mermaid | `02` | `N·m`、省略shapeは表へ | A | mermaid辺と§3表が同じ変数か |
| 5 Link/Mermaid | 全章 Cursor課題 | 解決済み課題を「確定。本文§」に更新 | 各log | 未解決だけが課題に残るか |

検証の共通手順: 本文変更後に本表IDを再判定し、残Critical/Highが0になるまで logs に差分を書く。制御コードと学習資料の同時編集はしない。

---

## 7. 資料ごとの総括

| 資料 | 骨格 | 残件の主ID |
|---|---|---|
| `00` | 経路概念は正しい | A04 |
| `01` | Plant正本として使える | A04（§12） |
| `02` | 契約正本として使える | A24 |
| `03` | 指令正本。照合済み | なし（観測typoはE済） |
| `04` | 位相正本 | A13 |
| `05` | Foothold式は正しい | A16 |
| `06` | 状態次元は正しい | A02, A11, 回転略記 |
| `07` | OCP骨格は正しい | A05–A07, A14–A15 |
| `08` | 「位相を選ばない」は正しい | A01, A02 |
| `09` | receding/Maskは正しい | A01 |
| `10` | 符号は正しい | A19–A20 |
| `11` | clip/`mj_step`は正しい | A29 |
| `12` | \(L_{footprint}\) 定義は正しい | A10, A18 |
| `13` | 理論は正しい | A17 |
| `14` | 層分けは有用 | A08 |
| `15` | Sampling≠Outer は正しい | A03 |
| `16` | 呼出順は正しい | A01, A21 |
| `17` | 手順は有用 | A33 |
| `18` | 原則は正しい | A04, A22–A23 |
| `19` | 作成時監査としては正しい | A26 |
| A | 制御変数は概ね正しい | A12 |
| B | 索引として可 | A31 |
| C | 数値の一部は正しい | A08, A09, A32 |
| D | 索引として可 | A25 |
| E | 旧誤解の倉庫として可 | A01, A04（§4, §15が古い） |
| F | 真の未知と確定済みが混在 | A27 |

解析ログ 01–21 同士の矛盾は、後半が前半の「当時未検証」を更新している点だけである。正本は新しいログ（10, 11, 15, 16, 18, 19, 21）。本文未反映が本監査の本体である。
