# Log 21: 学習・研究実験ロードマップ

対応プロンプト: 1レイヤーずつ、Baselineから不整地・Sim-to-Real、公平な Gradient vs Sampling。
記録日: 2026-08-23。コードと学習資料本文は未修正。ログ実装も未着手（[20](20_experiment_log_design.md)）。

根拠ログ: [01](01_baseline.md) Baseline、[15](15_speed_frequency_duty_stride.md) 歩幅、[16](16_rough_terrain_feasibility.md) 不整地、[17](17_user_tuning_parameters.md) 調整、[18](18_automatic_tuning_and_outer_loop.md) Outer、[19](19_gradient_vs_sampling_mpc.md) 方式差。

合格条件に、測定していない数値（例: 「0.05 m以内」）は書かない。

---

## 0. 運用規則

1. 主変更は下表の1列「変更する変数」だけ。他は Baseline。
2. 各段階の前に、直前段階の合格を満たす。飛ばす場合は理由を note に書く。
3. 成功だけでなく転倒・solver失敗・最悪 \(E\) を残す。
4. \(F^{cmd}\) と \(F^{act}\) を混ぜない。
5. ログが無い段階は、少なくとも転倒有無と目視。本ロードマップの本実施は log 20 最小案のあと。

レイヤー（同時変更禁止）:

| ID | レイヤー | このコードでの実体 |
|---|---|---|
| L1 | Prediction model | `centroidal_model_nominal` / Jax 力学、積分、外力 |
| L2 | Cost | `set_weight`、Sampling Q/R |
| L3 | Constraint | 摩擦、足箱、安定、遊脚F=0 |
| L4 | Gait | `gait`, `step_freq`, `duty`, offset |
| L5 | Foothold | FRG clip/offset、VFA |
| L6 | Swing | Kp/Kd、`step_height`、生成器 |
| L7 | Low-level | clip 0.9、関節PD（sim無効） |
| L8 | State estimation | 現行は完全状態。ノイズ追加はここ |
| L9 | Terrain perception | `scene`、HeightMap、VFA |

Plant乱択（摩擦・scene）は L9/環境であり、L1–L7 と同時に「改善」しない。診断sweepは可。

---

## 1. 実験段階

共通Log（段階1以降、実装後）: log 20 最小セット。時刻、指令/参照、状態、計画接触、実接触、`nmpc_GRFs`、実GRF（保存周期）、Foothold三種、`tau` 前後、sat、freq、終了理由。

共通Scenario箱: Go2、`dt=0.002`、MPC 100 Hz、`type='nominal'`、blind、viewer **off**（再現）、seed固定。例外は表に書く。

| 段階 | 目的 | 変更する変数 | 固定する変数 | Scenario | Log | 評価指標 | 合格条件 |
|---:|---|---|---|---|---|---|---|
| 1 Full stance | 高さ・姿勢が立脚4で持つか | `gait='full_stance'`。指令0 | 他config、Q/R、Swing未使用 | `scene='flat'`。10 s。外乱なし | 最小 + \(z,\phi,\theta\) | \(E_h\), \(E_{ori}\), sat率, fall | 転倒なし。satは Baseline Trot比で悪化しないこと（Trot未取得なら「実験で決める暫定」）。仕様: \(\tau\) が 0.9 ctrlrange 以内 |
| 2 Swing単脚 | 遊脚軌道が届くか | 低速またはその場Trot1周期。評価は遊脚脚だけ | Q/R、freq/duty、flat | flat。指令≈0。数周期 | Swing des/act、`swing_time` | \(\|p-p_d\|\), \(\tau\) peak（遊脚） | 転倒なし。未到達は「実験で決める暫定」。仕様: \(\tau\) 定格内 |
| 3 低速Trot | 閉ループ歩行の基準点 | 指令を小さい一定 \(v_x\)（clip内） | gait trot 1.35/0.74、Q/R、flat | flat。20 s。`forward` または固定指令 | 最小フル | \(E_v,E_{ori},E_h\), slip, fall, solver失敗率 | **この段階の記録が以降の Baseline比**。転倒なし。solver失敗率は「実験で決める暫定」 |
| 4 速度Step | 加減速と歩幅clip | \(v^{ref}(t)\) のステップ列のみ | f,d,Q,R,flat | flat。0→段階的に上げる。各段 ≥ 数周期 | 同上 + 指令切替時刻 | 応答（オーバーシュート）、\(L_{st}/2\) vs 0.42 clip、sat | 転倒なし。clip後も指令を上げ続けない（人の速度上限）。数値は暫定 |
| 5 Frequency/Duty sweep | 幾何と歩行の対応 | **L4のみ** `step_freq`×`duty` 格子 | Q/R、指令は段階3の低速、flat | 各格子1 episode | 幾何計算（log 15）+ 最小 | \(E_v\), slip, \(T_{sw}\), fall | 転倒なしの \((f,d)\) 集合を記録。Go2実現可能とは書かない |
| 6 MPC weight sweep | 追従と力の妥協 | **L2のみ** `Q_velocity` / `Q_position[2]` / `Q_base_angle` / `R_foot_force` を1軸ずつ | gait、指令=段階3 | flat、低速 | 最小 | \(E_v,E_h,E_{ori}\), GRF RMS, sat | Baseline比で一次元トレードオフを残す。同時に4重みを動かさない |
| 7 Friction sweep | 計画μと床μの差 | **環境** `friction_coeff` のみ。MPC `mu` は固定が既定。別系列で `mu` だけ動かす場合は同時に床を変えない | 制御L1–L7、flat | 床μを範囲で。各値複数seed | 実GRF、slip | slip, \(E_v\), fall | 転倒なしの床μ下限を記録。合格は「実験で決める暫定」 |
| 8 External disturbance | 外力への戻り | **Plant** `xfrc_applied`（実験hook）。制御の wrench 補償はゼロのまま | Q/R、gait、flat | パルス外力。大きさは人の安全判断 | \(z,\phi,\theta,v\) | 復帰時間（暫定）、fall | 転倒なし。補償ONと比較するなら **別段階**（L1）。同時にやらない |
| 9 段差 | 高さ変化への盲歩行 | **L9** `scene` を boxes/pyramids 等 | VFA=blind、gait、Q | 既存scene。速度は段階3 | 実接触、\(z_{foot}\), fall | fall, \(E_h\), 着地衝撃 | 転倒なしを要求しない。失敗率を Baseline（flat）比で報告 |
| 10 穴・飛び石 | \(\mathcal S\cap\mathcal R\) 欠如の観測 | scene / 手動穴。VFAは **まだOFF** | 制御L1–L7 | 穴がnominalに入る配置 | 三種Foothold、残Swing（オフライン） | 穴落ち、縁着地、fall | 現行は届かない想定。合格は「挙動を記録した」。VFA ONは **別実験**（L9） |
| 11 Solver stress | statusと時間 | horizon / `num_qp_iterations` / 急指令の **どれか1つ** | 他は段階3 | flatまたは段階4の急ステップ | status, `time_tot` | 失敗率、wall-clock | deadlineは **実験で測った中央値の余裕**。仕様値なし。失敗時は前回GRF（実装事実） |
| 12 Gradient vs Sampling | 方式差 | **L1+L2が不可避で異なる**（log 19）。変えるのは `type` だけ | robot、seed、指令、gait列、scene、摩擦、時間、clip | §4 | 両方の wall-clock、\(E_*\), fall | 公平条件を満たした報告。Cost一致は「可能な範囲」と明記 |
| 13 Domain randomization | 過適合検出 | 段階7+9の **環境** のみ複数因子。制御θは固定 | L1–L7 | 摩擦×scene。質量乱択は未実装なので入れないか、Plant-only別hook | 最悪と平均の \(E\), fall | 平均と最悪を両方。合格は暫定 |
| 14 Sim-to-Real準備 | 実機前チェックリスト | 制御は段階3相当。追加は記録と人の上限 | 実機PD/遅延はsimに無いと明記 | headless再現 + 実機は低速 | meta完全 | 人の安全項目（log 18 §7） | 自動合格なし。人が速度上限・E-stop・衝撃を決める |

段階9と10を `18` 本文は1段にまとめている。穴は VFA/Planner 欠如の観測なので段差と分ける。

---

## 2. Baselineとして固定するもの

実験開始時の固定セット（ディスク標準 + 再現手順）。

| 項目 | 固定値 | 根拠 |
|---|---|---|
| Git commit（wrapper） | 実験開始時の `mpc_dog` `HEAD`。作業開始時記録は `3adfad9f814c499fb996cf046c8fb4ac3a574e55` | log 01。**PyMPCのcommitではない** |
| Git dirty | 記録必須。解析docsが未追跡でも meta に書く | — |
| Quadruped-PyMPC tree | `.git` なし。zip記録 `cc145a2` は参考。実験ごとファイルハッシュまたは「未変更」宣言 | log 01 |
| gym-quadruped | 1.1.5 | log 01 |
| Config | `config.py` ディスク: `type='nominal'`, trot 1.35/0.74, `dt=0.002`, `mpc_frequency=100`, blind, `optimize_step_freq=False`, 制約オフ, reflex False | log 01, 17 |
| Gait | Trot。phase offset は触らない | log 07, 17 |
| Controller type | `nominal` | — |
| Random seed | `run_simulation(seed=)` を実験IDに保存。段階3の基準seedを1つ決める | — |
| Terrain | 基準は **`flat`**。ディスク既定 `perlin` は不整地用。Baseline歩行は flat に変える（L9を基準から外す） | 既定perlinのままにすると段階3と9が交ざる |
| Initial state | `reset(random=False)` の keyframe。段階比較では `random=True` にしない | `run_simulation` 初回 |
| Simulation時間 | 段階3基準 **20 s**（変更する段階は表のとおり） | 暫定。短すぎるとTrot数周期足りない |
| Viewer | 再現実験は **off**。動画は別run | renderが実GRF読取と壁時計を変える |
| Log設定 | log 20 最小。無効なら段階を「予備」と印 | 未実装中は目視+終了理由のみと宣言 |
| 指令モード | 基準は固定 \(v_x\)（`forward` 幅を1点に固定するか、コード外で一定指令）。`human`+キーは再現に使わない | log 06 |
| 摩擦 | 基準は `friction_coeff` を **一点**（範囲の中央を実験で固定）。範囲サンプリングは段階7 | 既定 (0.5,1.0) は episode ごとに変わる |
| Soft torque | 0.9 | L7固定 |
| Q/R | `set_weight` 既定 | L2固定 |

`18` §8.1 が PyMPC を `3adfad9` としている点は本Baselineでは採用しない。

---

## 3. 1変数群の原則

| 実験 | 動かしてよい層 | 同時に動かさない層 |
|---|---|---|
| 1–2 | L4（full_stance）または評価窓 | L2, L5–L9 |
| 3 | 指令（運用） | すべて |
| 4 | 指令スケジュール | L2–L7 |
| 5 | L4 | L2, L5, L6 |
| 6 | L2 の **1重み族** | L4、他重み |
| 7 | 環境摩擦、または MPC `mu` の一方 | 両方+scene |
| 8 | Plant外力 | L1の補償フラグ |
| 9–10 | L9 scene | VFAとgaitとweight |
| 11 | Solver数値の1つ | モデル式 |
| 12 | `type` | その他を意図的に揃える（§4） |
| 13 | 環境因子 | \(\theta\) |
| 14 | なし（記録と運用） | 制御改善を混ぜない |

禁止例: VFAを入れながら freq と Q を同時変更。Samplingに切り替えて同時に duty を 0.65 に「合わせる」（それは比較条件の明示的固定であり、改善実験ではない）。

---

## 4. Gradient vs Sampling の公平比較

`type` 以外を揃える。**Costは完全一致しない**（log 19: 足位置Q、GRFのR無効、積分なし、足固定）。報告に「同一Cost」と書かない。

| 条件 | 揃え方 |
|---|---|
| Robot model | 同じ XML、同じ `mass`/`inertia` 定数。勾配の慣性再計算は **両方オフ** にして `config.inertia` に揃える（標準ONだと勾配だけ姿勢依存） |
| Initial state | 同じ seed、`reset(random=False)` |
| Reference | 同じ `v^{ref}(t)`、同じ `ref_z` |
| Gait schedule | **同じ外側PGG**。`optimize_step_freq=False`。Sampling adaptive を使わない（Jax duty 0.65 で列がずれる） |
| Terrain / friction | 同じ scene、摩擦は一点 |
| 評価時間 | 同じ \(T\) |
| 安全Limit | 同じ 0.9 clip、同じVM |
| Costの意味 | 胴体高さ・速度・roll/pitch・角速度の対角は数値一致。足・GRF・積分は **一致させない**（コード変更になる）。変更して揃えるなら別実験（L2） |
| Hardware | CPU/GPU、コア数を meta へ |
| Wall-clock | 1周期の solve と episode 合計。制御周期超過率 |

最初の比較地形は **flat + 段階3指令**。不整地比較は段階9のあと。GPU Sampling と CPU 勾配を「速い」で混ぜない。

---

## 5. 研究課題の優先順位

難易度は本ツリーの実装量。価値はギャップの大きさ。Baseline影響は制御式を触るか。

| 研究候補 | 実装難易度 | 検証難易度 | 実機価値 | 最初に必要なBaseline | 優先順位 |
|---|---|---|---|---|---|
| 遊脚GRFの明示ゼロ制約 | 低（OCP等式または上下限を接触で0） | 低（段階3で \(F^{MPC}_{swing}\) と \(\tau\)） | 中（Maskで指令は既に0。内部実現可能性） | 段階3 + 詳細ログの生GRF | **2**。影響はL3のみ |
| Reachability-aware foothold | 中（IK検査またはhip距離を制約） | 中（段階5高速側） | 高 | 段階3–5、log 16 | **3** |
| Timing-aware foothold | 中（\(\|p-p_{lo}\|\le v_{max}T_{rem}\)） | 中 | 高 | 段階2–4 | **3**（到達と同時設計可） |
| 速度・f・Duty envelope | 低（sweep自動化。制御非変更） | 低 | 高（運用） | 段階5 | **1**。L4診断。コード非侵襲 |
| Terrain-aware TD timing | 高（Planner層が無い） | 高 | 高（不整地） | 段階10の失敗記録 | **6**。先に観測 |
| GRF Residual | 中（ログ→回帰。制御に入れるとL1） | 中 | 高 | 段階3+7、実GRFログ | **4**。最初はオフライン |
| Gradient vs Sampling | 低（`type`切替） | 中（公平条件の解釈） | 中 | 段階3、§4 | **5**。論文比較用。改善ではない |
| Outer-loop tuning | 中（gridは既存依存） | 中 | 中 | 段階3+6、log 20 | **7**。θ探索は段階6の後 |
| Domain randomization | 低–中（摩擦/sceneは既存。質量はhook） | 中 | 高（Sim-to-Real） | 段階7,9,13 | **5**（環境のみ） |
| Safe stopping policy | 中（`start_and_stop` はsimオフ。安全Foothold連動は未実装） | 高（実機判断） | **高** | 段階3、人の停止定義 | **4**。実機前に方針だけでも |

優先の読み方:

1. Envelope（制御を変えず地図を作る）
2. 遊脚F=0（小さいL3、OCP内部の未確定を閉じる）
3. 可到達・残時間（不整地の前提、段階10の前に理論実装）
4. Residual（オフライン）と停止方針
5. 方式比較とDR
6. Timing再計画（新Planner）
7. Outer-loop（指標と段階6が先）

`15` が「既存自動化」に挙げる integral / wrench / residual は、標準経路では Outer の代替にならない（log 18）。

---

## 6. 資料照合

### `18_Experiments_and_Research_Roadmap.md`

| 箇所 | 判定 | 本ログ |
|---|---|---|
| §1 1群変更、失敗率、GRF区別 | 正しい | §0 |
| §2 10段階 | 不完全 | 穴と段差を分離。外乱・DR・S2Rを追加。摩擦とrandomizationを分離 |
| §5 研究候補7つ | 正しい（列挙） | 停止方針・timing制約・DRを追加し優先を付けた |
| §7 層分離 | 正しい | L1–L9 に具体化 |
| §8.1 PyMPC=`3adfad9` | 誤り | §2 |
| §8 ログ計画 | 正しい（未実装） | 本実施の前置条件 |

### `15_Automatic_Tuning_and_Sim_to_Real.md`

| 箇所 | 判定 | 本ログ |
|---|---|---|
| Sampling≠Weight調整 | 正しい | 段階12、優先5 |
| 既存自動化の列挙 | 不完全 | 標準オフを前提にしない |
| Outer順序 | 指針として正しい | 段階6の後が Outer（優先7） |
| 人が残す判断 | 正しい | 段階14 |

### `appendices/F_Open_Questions.md`

| 項目 | 本ロードマップでの扱い |
|---|---|
| 遊脚GRF内部制約 | 優先2で閉じる実験 |
| VFAと残時間 | 段階10観測 → 優先3 |
| 安全Footholdなしの停止 | 優先4 |
| Frequency実機範囲 | 段階5の地図 |
| Auto-tune Sim-to-Real | 段階14の後。Fのまま |
| Residual学習 | 優先4オフライン |
| Gradient vs Sampling公平比較 | 段階12 + §4。F「同一Cost」は現行不可 |

---

## 7. 直後にやること（実装は依頼後）

1. log 20 最小ログ（制御非変更）。
2. 段階3を flat・一点摩擦・固定指令で取り、以降の Baseline比にする。
3. 段階5格子（低速、L4のみ）。
4. 研究は遊脚F=0または envelope の論文用整理。

本ファイルは仕様である。`18` 本文の置換ではない（未修正）。
