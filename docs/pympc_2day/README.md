# PyMPC 2日間ワークショップ — 教材インデックス

**GRF · MPC · WBC** を四足ロボット（Unitree Go2）の OSS 実装で体感する教育パッケージです。

---

## 誰が何を読むか

| 役割 | 最初に読む | 進行中 | 復習 |
|------|-----------|--------|------|
| **受講者** | [LEARNER_GUIDE.md](./LEARNER_GUIDE.md) | [notebooks/](./notebooks/) | [MPC_TUNING_JOURNEY.md](./MPC_TUNING_JOURNEY.md) |
| **講師・ファシリテータ** | [INSTRUCTOR_GUIDE.md](./INSTRUCTOR_GUIDE.md) | [WORKSHOP.md](./WORKSHOP.md) §7 | [SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md) |
| **技術深掘り** | [WORKSHOP.md](./WORKSHOP.md) | 外部 PyMPC ソース | [learning_paths_for_consulting.md](../learning_paths_for_consulting.md) |

---

## 教材一覧

### ガイド（Markdown）

| ファイル | 内容 |
|----------|------|
| [LEARNER_GUIDE.md](./LEARNER_GUIDE.md) | 受講者向け：前提知識・2日スケジュール・セッション別チェックリスト |
| [INSTRUCTOR_GUIDE.md](./INSTRUCTOR_GUIDE.md) | 講師向け：タイムテーブル・デモ台本・よくある質問・トラブルシュート |
| [TUNING_GUIDE.md](./TUNING_GUIDE.md) | パラメータ調整早見表（成功/失敗パターン） |
| [MPC_TUNING_JOURNEY.md](./MPC_TUNING_JOURNEY.md) | **MPC 設計者体験** — Phase 1–4 失敗・成功 + lab 連携 |
| [WORKSHOP.md](./WORKSHOP.md) | 統合技術資料（理論・Architecture・全セッション解説） |
| [SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md) | Session 4 試行錯誤ログ（5 kph × 凸凹地形） |

### Notebook（実行済み ipynb）

| # | Notebook | テーマ |
|---|----------|--------|
| 00 | [00_theory_grf_mpc_wbc.ipynb](./notebooks/00_theory_grf_mpc_wbc.ipynb) | 理論：GRF / MPC / WBC |
| 01 | [01_demo_session01_flat_smoke.ipynb](./notebooks/01_demo_session01_flat_smoke.ipynb) | 平坦スモーク + GRF 可視化 |
| 02 | [02_demo_session02_flat_tune.ipynb](./notebooks/02_demo_session02_flat_tune.ipynb) | μ / 歩調チューニング |
| 03 | [03_demo_session03a_rough_boxes.ipynb](./notebooks/03_demo_session03a_rough_boxes.ipynb) | 不整地：箱障害 |
| 04 | [04_demo_session03b_rough_perlin.ipynb](./notebooks/04_demo_session03b_rough_perlin.ipynb) | 不整地：連続起伏 |
| 05 | [05_demo_session04_speed_bumpy.ipynb](./notebooks/05_demo_session04_speed_bumpy.ipynb) | 5 kph × 凸凹坂道（20 m） |
| **06** | **[06_mpc_tuning_journey.ipynb](./notebooks/06_mpc_tuning_journey.ipynb)** | **MPC 調整ジャーニー統合（fail/success 体験）** |
| **07–10** | **[07–10 高度シナリオ](./notebooks/07_scenarios_flat_foundation.ipynb)** | **20 シナリオ（路面·速度·勾配·遷移）— お客様 QA 用** |
| **11** | **[11_qa_discussion_master.ipynb](./notebooks/11_qa_discussion_master.ipynb)** | **QA ディスカッション索引（全 20 シナリオ横断）** |
| **12–15** | **[Sampling MPC トラック](./notebooks/12_demo_sampling_session01_flat.ipynb)** | **IROS 2024 MPPI — Session 1–4 同シナリオ（nominal と並行）** |
| **16–19** | **[Lyapunov MPC トラック](./notebooks/16_demo_lyapunov_session01_flat.ipynb)** | **RAL 2025 安定性制約 — Session 1–4 同シナリオ（nominal と並行）** |

### デモ映像・計算結果

| 種別 | パス |
|------|------|
| GIF / MP4 | [assets/](./assets/) |
| headless 検証 | [assets/headless_results.json](./assets/headless_results.json) |
| パラメータスタディ | [assets/param_study_results.json](./assets/param_study_results.json) |
| Session 4 勝者パラメータ | [assets/speed_terrain_results.json](./assets/speed_terrain_results.json) |
| Sampling MPC ベンチ | [assets/sampling_lab_results.json](./assets/sampling_lab_results.json) |
| Lyapunov MPC ベンチ | [assets/lyapunov_lab_results.json](./assets/lyapunov_lab_results.json) |

---

## フォルダ構成（本ディレクトリ）

```
docs/pympc_2day/
├── README.md                          # 本ファイル — 教材インデックス
├── LEARNER_GUIDE.md                   # 受講者向け（スケジュール・チェックリスト）
├── INSTRUCTOR_GUIDE.md                # 講師向け（台本・Q&A）
├── TUNING_GUIDE.md                    # パラメータ調整早見表
├── MPC_TUNING_JOURNEY.md              # MPC 設計者体験（Phase 1–4 fail/success）
├── WORKSHOP.md                        # 統合技術資料（理論・コード・全セッション）
├── SPEED_TERRAIN_TRIAL_LOG.md         # Session 4 試行錯誤ログ
│
├── notebooks/                         # 実行済み Jupyter Notebook
│   ├── 00_theory_grf_mpc_wbc.ipynb    #   理論：GRF / MPC / WBC
│   ├── 01_demo_session01_flat_smoke.ipynb   # S1 平坦スモーク
│   ├── 02_demo_session02_flat_tune.ipynb     # S2 パラメータチューニング
│   ├── 03_demo_session03a_rough_boxes.ipynb  # S3a 箱障害
│   ├── 04_demo_session03b_rough_perlin.ipynb # S3b 連続起伏
│   ├── 05_demo_session04_speed_bumpy.ipynb   # S4 5 kph × 凸凹坂
│   ├── 06_mpc_tuning_journey.ipynb           # MPC 調整ジャーニー統合
│   ├── 07_scenarios_flat_foundation.ipynb    # 高度シナリオ 01–05（平坦基礎）
│   ├── 08_scenarios_rough_speed.ipynb        # 高度シナリオ 06–10（不整地·速度）
│   ├── 09_scenarios_slope_ramp.ipynb         # 高度シナリオ 11–15（勾配·ランプ）
│   ├── 10_scenarios_transition_limits.ipynb  # 高度シナリオ 16–20（遷移·限界）
│   └── 11_qa_discussion_master.ipynb         # QA 索引（お客様ディスカッション用）
│
└── assets/                            # 計算結果・デモ映像
    ├── demo_*.gif / *.meta.json       #   デモ + 検証メタデータ
    ├── tuning_lab_results.json        #   tuning_labs 実行キャッシュ
    ├── speed_terrain_trial_log.json   #   S4 全試行
    └── speed_terrain_results.json     #   S4 勝者
```

**リポジトリ全体のツリー:** [../../README.md#フォルダ構成](../../README.md#フォルダ構成)

---

## 5 セッション概要

| Session | 地形 | 足場 opt | 主な学び | Notebook |
|---------|------|----------|----------|----------|
| **1** | flat | OFF | 3 層パイプラインが動く | [01](./notebooks/01_demo_session01_flat_smoke.ipynb) |
| **2** | flat | OFF | μ / step_freq のトレードオフ | [02](./notebooks/02_demo_session02_flat_tune.ipynb) |
| **3a** | random_boxes | ON | 離散障害と足場最適化 | [03a](./notebooks/03_demo_session03a_rough_boxes.ipynb) |
| **3b** | perlin | ON | 連続起伏・保守 gait | [03b](./notebooks/04_demo_session03b_rough_perlin.ipynb) |
| **4** | bumpy × 3 | ON | 高速指令 + 坂道 + 試行錯誤 | [05](./notebooks/05_demo_session04_speed_bumpy.ipynb) |

詳細比較表は [WORKSHOP.md §7](./WORKSHOP.md#4-セッションの違い必読) を参照。

---

## クイックスタート

```bash
source .venv/bin/activate && . .env.workshop
jupyter lab docs/pympc_2day/notebooks/

# 高度シナリオ（お客様 QA 用）
python scripts/scenario_labs.py --list
python scripts/scenario_labs.py --scenario sc11_bumpy_uphill_gravity
```

```bash
# 初回セットアップ
./scripts/setup_references.sh && ./scripts/setup_uv_workshop.sh
source .venv/bin/activate && . .env.workshop

# Jupyter 起動
jupyter lab docs/pympc_2day/notebooks/

# 並行論文トラックのベンチ（nominal Session 1–4 は変更なし）
python scripts/run_sampling_benchmark.py --list
python scripts/run_lyapunov_benchmark.py --list

# 計算結果・GIF・Notebook 一括再生成
python scripts/run_workshop_pipeline.py
```
