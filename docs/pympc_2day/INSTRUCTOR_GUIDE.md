# 講師・ファシリテータガイド — PyMPC 2日間ワークショップ

このガイドは **講師・ファシリテータ** 向けです。タイムテーブル、デモ台本、想定 Q&A、トラブルシュートをまとめています。

技術詳細は [WORKSHOP.md](./WORKSHOP.md)、受講者向け手順は [LEARNER_GUIDE.md](./LEARNER_GUIDE.md) を参照してください。

---

## 1. 開催前チェックリスト

### 1.1 環境

- [ ] `./scripts/setup_uv_workshop.sh` 完了
- [ ] `source .venv/bin/activate && . .env.workshop` で import OK
- [ ] `python scripts/run_workshop_pipeline.py --from-step capture` で GIF 生成確認（EGL / GPU）
- [ ] Jupyter Lab 起動確認
- [ ] **初回 sim を事前実行**（acados codegen ~5 min を受講者待ち時間から除外）

### 1.2 教材

- [ ] [notebooks/](./notebooks/) が **実行済み ipynb** であること
- [ ] [assets/headless_results.json](./assets/headless_results.json) で 4 プリセット OK
- [ ] Session 4 GIF 3 本（flat / uphill / downhill）が存在
- [ ] プロジェクター用：GIF をブラウザで開ける状態

### 1.3 受講者への事前連絡

- Linux + Python 環境、16 GB RAM 推奨
- ADAS / 操舵 MPC 経験があるとスムーズ（必須ではない）
- [LEARNER_GUIDE.md §2](./LEARNER_GUIDE.md#2-前提知識) を事前読了推奨

---

## 2. 2 日間タイムテーブル（ファシリテーション）

| 日 | ブロック | 時間 | 形式 | 資料 |
|----|----------|------|------|------|
| **1** | 理論 | 90 min | 講義 + NB 00 | [00_theory](./notebooks/00_theory_grf_mpc_wbc.ipynb) |
| **1** | Session 1 | 75 min | デモ + ハンズオン | [01](./notebooks/01_demo_session01_flat_smoke.ipynb) |
| **1** | Session 2 | 120 min | 実験中心 | [02](./notebooks/02_demo_session02_flat_tune.ipynb) |
| **2** | Session 3a | 90 min | デモ + 比較 | [03a](./notebooks/03_demo_session03a_rough_boxes.ipynb) |
| **2** | Session 3b | 75 min | 本番デモ脚本 | [04](./notebooks/04_demo_session03b_rough_perlin.ipynb) |
| **2** | Session 4 | 120 min | 試行錯誤体験 | [05](./notebooks/05_demo_session04_speed_bumpy.ipynb) |
| **2** | 総合 | 45 min | 質疑 + チェックリスト | [LEARNER_GUIDE §5](./LEARNER_GUIDE.md#5-セッション別学ぶことやること確認) |

**ペース調整:** Session 2 と 4 は sim 待ちが長い。先に GIF を見せ、Notebook は代表 1 セルだけ実行する短縮版も可。

---

## 3. セッション別デモ台本

### 3.1 理論（Notebook 00）— 15 分口頭 + 75 分 NB

**オープニング（2 分）**

> 「四足は **GRF を決める MPC** と **GRF を関節トルクに変える下位層** の 2 段が定番です。操舵 MPC の『タイヤ力』に相当するのが GRF です。」

**3 層説明（5 分）— ホワイトボード**

```
速度指令 / trot ゲイト
    ↓
MPC（SRB）→ 最適 GRF（12D）
    ↓
WBC 相当 → 関節トルク τ
    ↓
MuJoCo / 実機
```

**摩擦円錐（3 分）**

> 「μ は ADAS のタイヤ摩擦係数と同じ役割。μ を上げると横・前後力を積極的に取れるが、モデルと実際がズレると転倒します。」

**NB 00 ハンズオン:** Step 5（摩擦円錐プロット）まで必須。Step 7（TUNING_GUIDE 表）は [TUNING_GUIDE.md](./TUNING_GUIDE.md) と同内容。

---

### 3.2 Session 1 — 平坦スモーク（20 分デモ + 55 分 NB）

**お客様向け 1 文**

> 「最小構成で **3 層パイプラインが動いている** ことを確認します。足場最適化は OFF にして変数を減らしています。」

**デモの見せ方**

1. [demo_s01_flat.gif](./assets/demo_s01_flat.gif) を再生
2. 左上ラベル `Session 1 | scene=flat` を指す
3. 緑矢印（GRF）が接地脚に出ていることを指す

**意図的失敗（NB Step 5）**

> 「`ref_z` を下げると即転倒。**胴体高さの参照** は最初に確認するパラメータです。」

**よくある質問**

| Q | A |
|---|---|
| なぜ足場 opt OFF？ | 平坦では不要。デバッグ変数を減らす |
| GRF 矢印はどこで描画？ | `simulation/simulation.py`（visual モード） |
| headless では見えない？ | その通り。GIF は offscreen capture |

---

### 3.3 Session 2 — 平坦チューニング（30 分デモ + 90 分 NB）

**お客様向け 1 文**

> 「同じ平坦でも **μ と歩調** を変えると、加速と安定性のトレードオフがはっきり出ます。」

**デモの見せ方**

1. S1 GIF（step_freq=1.4）と S2 GIF（1.75 Hz）を **並べて** 再生
2. S2 の方が足回りが速いことを指す

**実験 A（μ）— 台本**

> 「μ=0.55 は積極的。μ=0.35 は保守的。転倒したら **まず μ を下げる** というトリアージを覚えてください。」

**実験 B（step_freq）— 台本**

> 「歩調を上げすぎると MPC の予測が追いつかず、足が地面に刺さることがあります。」

**パラメータスタディ**

[`param_study_results.json`](./assets/param_study_results.json) の表を [WORKSHOP.md §7.2](./WORKSHOP.md#72-session-2--平坦パラメータチューニング) から投影。

---

### 3.4 Session 3a — 箱障害（25 分デモ + 65 分 NB）

**お客様向け 1 文**

> 「段差や箱のような **離散障害** では、MPC に **足場最適化** を入れ、どこに足を置くかも計画します。」

**デモの見せ方**

1. [demo_s03_boxes.gif](./assets/demo_s03_boxes.gif) — 約 9 秒で箱エリアに進入
2. intro カメラ（ワイド）→ 追従カメラの切替を指す
3. S1/S2 との **地形の違い** を強調

**意図的失敗**

> 「step_freq=1.6 は不整地では速すぎ。1.1 + duty=0.75 が安定側。」

---

### 3.5 Session 3b — 連続起伏（20 分デモ + 55 分 NB）

**お客様向け 1 文**

> 「boxes は段差、perlin は **うねり**。見た目も難易度も違います。本番デモは perlin を推奨。」

**本番デモ脚本（15 分）**

1. S1 GIF → 「平坦・足場 opt OFF・3 層の骨格」
2. S2 NB 抜粋 → 「μ / 歩調を触った」
3. S3 perlin GIF → 「不整地・足場 opt ON・連続起伏」
4. 締め：「MPC が **どこに足を置き、どれだけ蹴るか** を計画」

---

### 3.6 Session 4 — 5 kph × 凸凹坂（30 分デモ + 90 分 NB）

**お客様向け 1 文**

> 「指令 5 kph・20 m 走行を凸凹の平坦・上り・下りで試しました。**転倒なしは未達** ですが、試行錯誤の過程と **resilient 評価** で 20 m 達成まで持っていきました。」

**正直に伝えるポイント**

| 事実 | 説明 |
|------|------|
| no-fall @ 5 kph 未達 | 3 地形とも数 m で転倒。現行 trot + nominal MPC の限界 |
| resilient で 20 m | 転倒後 reset、累積距離で評価（実機では別問題） |
| 平均速度 << 5 kph | 転倒・ランプ・地形で実 vx は低い。指令は 5 kph |

**3 GIF の見せ方**

| GIF | 強調 |
|-----|------|
| [demo_s04_flat.gif](./assets/demo_s04_flat.gif) | ベース。17 falls |
| [demo_s04_uphill.gif](./assets/demo_s04_uphill.gif) | μ↓ duty↑ |
| [demo_s04_downhill.gif](./assets/demo_s04_downhill.gif) | 最難。duty=0.82 |

**試行錯誤の見せ方（NB Step 1–4）**

1. `speed_terrain_trial_log.json` の表を投影
2. no-fall スイープが全部 ❌ であることを見せる
3. [SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md) の第 2–4 節を朗読

**NB Step 4（resilient）**

> 「受講者には **パラメータ 1 つ変えて再実行** を課題にすると効果的（例：downhill の duty を 0.80 に下げて falls が増えるか）」

---

### 3.7 Session 6 — MPC 調整ジャーニー（Notebook 06）

**資料:** [MPC_TUNING_JOURNEY.md](./MPC_TUNING_JOURNEY.md) · [06_mpc_tuning_journey.ipynb](./notebooks/06_mpc_tuning_journey.ipynb)

**進行（90 min）**

1. Phase 1–2: `run_lab_pair("s1_ref_z_fail", "s1_ref_z_ok")` — 「MPC 以前の ref_z」
2. Phase 2: μ / step_freq の fail vs success
3. Phase 3: boxes GIF を見せながら freq fail vs ok
4. Phase 4: `plot_speed_trial_journey()` — 試行ログの棒グラフ
5. Step 7 演習: 受講者が 1 パラメータ変更

**CLI デモ（投影用）**

```bash
python scripts/tuning_labs.py --list
python scripts/tuning_labs.py --lab s2_mu_aggressive
python scripts/verify_workshop_assets.py
```

---

## 4. 5 セッション比較（投影用）

| | S1 | S2 | S3a | S3b | S4 |
|---|----|----|-----|-----|-----|
| scene | flat | flat | boxes | perlin | bumpy×3 |
| 足場 opt | OFF | OFF | ON | ON | ON |
| 速度 | 低 | 中 | 低 | 低 | **5 kph 指令** |
| 距離 | ~数 m | ~数 m | ~数 m | ~数 m | **20 m 累積** |
| 主目的 | 動く確認 | 触る | 離散障害 | 連続起伏 | 高速+坂 |

---

## 5. よくある質問（全セッション）

| Q | A |
|---|---|
| PyMPC vs iLQR 系の違い | GRF 明示・足場 opt・実機実績。[top2_stack_comparison.md](../top2_stack_comparison.md) |
| acados が遅い | 初回 codegen のみ ~5 min。2 回目以降はキャッシュ |
| 実機に載る？ | muse + `ros2/run_controller.py`（本 WS 外） |
| RL は？ | 本 WS は model-based。サーベイ doc 参照 |
| Session 4 の resilient は実機向き？ | **教育用評価**。実機で転倒 reset は非現実的。指令設計・gait tuning の学習が目的 |

---

## 6. トラブルシュート（当日）

| 症状 | 対処 |
|------|------|
| `MUJOCO_GL` エラー | `export MUJOCO_GL=egl`（`.env.workshop` 確認） |
| acados codegen 失敗 | `ACADOS_WITH_SYSTEM_BLASFEO=OFF` で再 setup |
| Notebook kernel 死 | `.venv` kernel 選択、`source .env.workshop` 後に jupyter 再起動 |
| sim 即 terminated | [TUNING_GUIDE.md](./TUNING_GUIDE.md) → ref_z / mu / step_freq |
| GIF が平坦に見える | 古い GIF。`capture_demo_frames.py` 再実行（S3 は 4500 step） |
| bumpy scene エラー | `workshop_terrain.install_custom_terrains()` を QuadrupedEnv import 前に |

---

## 7. クロージング（5 分）

**修了チェック（口頭）**

1. 3 層を 1 文で説明
2. μ を 1 つ上げ下げした効果
3. 不整地で足場 opt が必要な理由
4. Session 4 で no-fall と resilient の違い

**次のステップ案内**

- 復習：[LEARNER_GUIDE.md §7](./LEARNER_GUIDE.md#7-ワークショップ後の復習)
- コンサル到達：[learning_paths_for_consulting.md](../learning_paths_for_consulting.md)
- 実機：[WORKSHOP.md §8.2](./WORKSHOP.md#82-技術的な拡張)

---

## 8. 再生成コマンド（講師用）

```bash
source .venv/bin/activate && . .env.workshop

# 全部
python scripts/run_workshop_pipeline.py

# GIF のみ（Session 1–3 + 4）
python scripts/capture_demo_frames.py
python scripts/capture_speed_terrain_demos.py

# Notebook 05 のみ再生成・実行
python scripts/generate_workshop_notebooks.py
python -m jupyter nbconvert --execute --inplace docs/pympc_2day/notebooks/05_demo_session04_speed_bumpy.ipynb
```
