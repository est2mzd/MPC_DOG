# mpc_dog — 四足 MPC ワークショップ & OSS 選定リポジトリ

四足ロボット（Unitree Go2）の **GRF · MPC · WBC** を、実機実績のある OSS で学習・デモ・コンサル説明するためのリポジトリです。

論文デモの単純再現ではなく、**実機実績 · 活発メンテ · 2024–2026 技術** を満たすスタックを選び、再現可能な教材（Notebook・GIF・パラメータスタudy）まで一式を揃えています。

---

## 概要

| 項目 | 内容 |
|------|------|
| **主目的** | お客様向けコンサルで「接地反力 → MPC → WBC」を 2 日で説明・デモできる状態にする |
| **採用 OSS** | [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC)（IIT DLS Lab、Unitree 実機実績） |
| **ロボット** | Unitree Go2（MuJoCo sim） |
| **環境管理** | [uv](https://github.com/astral-sh/uv)（Python 3.11、`.venv`） |
| **教材** | [docs/pympc_2day/WORKSHOP.md](docs/pympc_2day/WORKSHOP.md) + 実行済み Jupyter Notebook + 計算結果 GIF/MP4 |

**制御パイプライン（3 層）**

```
速度指令 / ゲイト (trot)
        ↓
MPC (SRB)  … 最適化変数 = 接地反力 GRF（12 次元）· 摩擦円錐
        ↓
WBC 相当   … Stance: GRF→関節τ / Swing: 足軌道+PD
        ↓
MuJoCo Sim
```

ADAS 操舵 MPC 経験者向けの対応: 車両モデル → SRB、タイヤ力 → **GRF**、下位実行 → WBC。

---

## 背景

### なぜこのリポジトリがあるか

- 四足制御の定番は **GRF-MPC + WBC** の 3 段構成だが、OSS は MuJoCo iLQR・Convex MPC 再実装・RL など選択肢が多く、**コンサルで説明しやすい実装** を選ぶ必要がある。
- [go2-convex-mpc](https://github.com/erwincoumans/go2-convex-mpc) 等の 2018 系再実装は教育向きだが、実機実績・不整地・2024 以降の技術更新が不足（詳細: [docs/stack_selection.md](docs/stack_selection.md)）。
- Quadruped-PyMPC は **acados 勾配 MPC + 足場最適化 + Unitree 実機** を公開しており、「摩擦円錐 → GRF → 関節トルク」の流れをそのまま説明できる。

### 採用スタック（優先順）

| 優先 | スタック | 先端性 | 安定性 | 本リポジトリでの位置づけ |
|------|----------|--------|--------|--------------------------|
| **★1** | Quadruped-PyMPC | acados MPC + 足場 opt | Unitree 実機、IROS'24/RAL'25 | **メイン・ワークショップ土台** |
| **★2** | mujoco_mpc + deploy | Whole-body iLQR | DeepMind / CMU、Go2 実機 | Phase 2 拡張候補 |
| **★3** | ocs2_ros2 + quadruped_ros2_control | ETH OCS2 NMPC | ROS2 産業向け | 知覚統合 NMPC 拡張候補 |

---

## 目的

1. **学習:** GRF / MPC / WBC の役割を数式・Notebook・sim 結果で理解する  
2. **チューニング:** `mu` / `step_freq` / 足場 opt 等を変え、**成功と失敗のパターン** を体感する  
3. **デモ:** 平坦 → パラメータ変更 → 不整地の順で、計算結果（GIF・グラフ）を見せながら説明する  
4. **再現:** uv 環境 + 1 コマンド（`run_workshop_pipeline.py`）で教材を再生成できる

**非目標:** 実機 ROS2 デプロイ、RL 実装、acados 内部モデル編集（参考リンクのみ）

---

## 結論

| 論点 | 結論 |
|------|------|
| **出発点 OSS** | Quadruped-PyMPC を採用。GRF を明示最適化し、WBC 相当層まで一気通貫 |
| **環境** | conda ではなく **uv** で `.venv` を管理（再現性・依存の明示化） |
| **教材形式** | Markdown（WORKSHOP.md）+ **実行済み Notebook** + **生成済み assets**（GIF/MP4/JSON） |
| **検証** | 4 セッション preset の headless sim OK、param study 完了、Notebook 01–04 全セル実行済み |
| **拡張方針** | 不整地・犬速度 → PyMPC ベース。必要なら MuJoCo iLQR または OCS2 perceptive へ（[top2_stack_comparison.md](docs/top2_stack_comparison.md)） |

---

## 環境構築

### 前提

- **OS:** Linux（Ubuntu 22.04 / 24.04 想定）
- **CPU/GPU:** x86_64。headless sim は EGL（`MUJOCO_GL=egl`）
- **ツール:** `git`, `cmake`, `gcc`, `python3`（3.10+）
- **uv:** 未インストールなら setup スクリプトが `pip install --user uv` を試行

### 手順（初回）

```bash
git clone <this-repo> mpc_dog && cd mpc_dog

# 1. Quadruped-PyMPC を external/ に取得
./scripts/setup_references.sh

# 2. uv で .venv 作成 + 依存 + acados ビルド + PyMPC インストール
./scripts/setup_uv_workshop.sh

# 3. セッション有効化（毎回）
source .venv/bin/activate
. .env.workshop
```

**初回所要時間:** acados ビルド含め **30–90 分**（2 回目以降は数秒〜数分）。

### 動作確認

```bash
source .venv/bin/activate && . .env.workshop

# 一括: param study + headless + GIF/MP4 + Notebook 実行
python scripts/run_workshop_pipeline.py

# Notebook のみ再実行
python scripts/run_workshop_pipeline.py --from-step notebooks

# Jupyter で教材を開く
jupyter lab docs/pympc_2day/notebooks/
```

カーネル名: `mpc-dog-workshop`（uv `.venv`）

### トラブルシュート

| 症状 | 対処 |
|------|------|
| `acados submodule missing` | `./scripts/setup_references.sh` の後、`cd external/Quadruped-PyMPC && git submodule update --init --recursive` |
| acados codegen 失敗 | `ACADOS_WITH_SYSTEM_BLASFEO=OFF` でビルド（`setup_uv_workshop.sh` 反映済み） |
| 初回 sim が遅い | acados codegen + t_renderer ダウンロードで ~5 min。2 回目以降はキャッシュで高速 |
| Notebook が import 失敗 | `source .venv/bin/activate && . .env.workshop` を確認 |

---

## ファイル構成（背景 · 目的）

### ドキュメント

| ファイル | 背景 · 目的 |
|----------|-------------|
| [docs/pympc_2day/WORKSHOP.md](docs/pympc_2day/WORKSHOP.md) | **統合教材**。理論・数式・アーキ・コードリンク・デモ結果 GIF を 1 本に集約 |
| [docs/pympc_2day/notebooks/](docs/pympc_2day/notebooks/) | **Step-by-step 実習**。理論 NB + セッション別デモ NB（01–04）。実行済み outputs 付き |
| [docs/pympc_2day/assets/](docs/pympc_2day/assets/) | **計算結果**。GIF/MP4/PNG、param study JSON、headless 検証結果 |
| [docs/stack_selection.md](docs/stack_selection.md) | OSS 選定の評価軸と Phase 1–3 結論 |
| [docs/top2_stack_comparison.md](docs/top2_stack_comparison.md) | PyMPC vs MuJoCo iLQR の比較（コンサル説明用） |
| [docs/quadruped_mpc_rl_survey.md](docs/quadruped_mpc_rl_survey.md) | 論文・技術サーベイ（GRF/MPC/WBC/RL の位置づけ） |

### スクリプト

| ファイル | 背景 · 目的 |
|----------|-------------|
| [scripts/setup_references.sh](scripts/setup_references.sh) | `external/Quadruped-PyMPC` の clone / 取得 |
| [scripts/setup_uv_workshop.sh](scripts/setup_uv_workshop.sh) | **環境構築の入口**。uv venv、workshop 依存、acados ビルド、`.env.workshop` 生成 |
| [scripts/run_workshop_pipeline.py](scripts/run_workshop_pipeline.py) | **実行の入口**。param study → headless → GIF → Notebook 一括 |
| [scripts/pympc_lab.py](scripts/pympc_lab.py) | Notebook 用 sim・プロット・MPC 設計者向け TUNING_GUIDE |
| [scripts/apply_pympc_preset.py](scripts/apply_pympc_preset.py) | YAML preset → `external/.../config.py` パッチ |
| [scripts/run_parameter_study.py](scripts/run_parameter_study.py) | μ / step_freq スイープ → assets JSON/PNG |
| [scripts/run_pympc_headless.py](scripts/run_pympc_headless.py) | 8 s headless sim 検証（パイプラインから呼び出し） |
| [scripts/capture_demo_frames.py](scripts/capture_demo_frames.py) | offscreen フレーム → GIF/MP4 生成 |
| [scripts/generate_workshop_notebooks.py](scripts/generate_workshop_notebooks.py) | Notebook テンプレ再生成（編集後に実行） |

### 設定 · その他

| ファイル | 背景 · 目的 |
|----------|-------------|
| [pyproject.toml](pyproject.toml) | uv プロジェクト定義（sim 依存 + `[workshop]` extra: Jupyter 等） |
| [configs/pympc_presets/*.yaml](configs/pympc_presets/) | セッション別 PyMPC 設定（平坦スモーク / チューニング / 不整地） |
| [tests/test_apply_pympc_preset.py](tests/test_apply_pympc_preset.py) | preset パッチの単体テスト |
| `external/Quadruped-PyMPC/` | 上流 OSS（gitignore）。sim・MPC・acados の本体 |
| `.env.workshop` | acados パス・`LD_LIBRARY_PATH`・`MUJOCO_GL`（setup 時に生成） |
| `logs/pympc_sessions/` | preset 適用バックアップ・headless ログ（gitignore） |

### プリセット一覧

| preset | 目的 |
|--------|------|
| `session01_flat_smoke` | 平坦 trot。足場 opt OFF。初回動作確認 |
| `session01_flat_smoke_retry` | ref_z 微増・歩調保守。転倒時の retry 用 |
| `session02_flat_tune` | μ / step_freq / gain チューニングのベースライン |
| `session03_rough_boxes` | 段差 terrain + 足場 opt ON |
| `session03_rough_perlin` | 連続起伏 terrain。本番デモ用 |

---

## ワークショップの進め方（2 日目安）

| 日 | 午前 | 午後 |
|----|------|------|
| **1 日目** | [00 理論 NB](docs/pympc_2day/notebooks/00_theory_grf_mpc_wbc.ipynb) + [01 平坦デモ](docs/pympc_2day/notebooks/01_demo_session01_flat_smoke.ipynb) | [02 チューニング](docs/pympc_2day/notebooks/02_demo_session02_flat_tune.ipynb) |
| **2 日目** | [03a boxes](docs/pympc_2day/notebooks/03_demo_session03a_rough_boxes.ipynb) + [03b perlin](docs/pympc_2day/notebooks/04_demo_session03b_rough_perlin.ipynb) | WORKSHOP.md + GIF でデモリハーサル |

---

## 参考リンク

- [Quadruped-PyMPC（上流）](https://github.com/iit-DLSLab/Quadruped-PyMPC)
- [muse（実機推定）](https://github.com/iit-DLSLab/muse)
- [MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc)
- [OCS2 ROS2](https://github.com/legubiao/ocs2_ros2)
