# MPC 設計者の調整ジャーニー — 失敗・成功・体験ガイド

**対象:** 操舵 MPC 経験者が四足 PyMPC で「何を触ると何が起きるか」を **体験** するための教材。  
Notebook · Python スクリプト · プリセット YAML · デモ GIF が **同じ試行錯誤の物語** を共有します。

---

## 使い方（3 つの入口）

| やり方 | 入口 | 所要時間 |
|--------|------|----------|
| **読む** | 本ファイル（Phase 1→4） | 30 min |
| **触る（Notebook）** | [06_mpc_tuning_journey.ipynb](./notebooks/06_mpc_tuning_journey.ipynb) | 2–3 h |
| **触る（CLI）** | `python scripts/tuning_labs.py --list` |  lab 単位 10 s–3 min |

```bash
source .venv/bin/activate && . .env.workshop

#  lab 一覧
python scripts/tuning_labs.py --list

# 失敗→成功ペアを Notebook と同じ API で
python -c "
import sys; sys.path.insert(0,'scripts')
from tuning_labs import run_lab; print(run_lab('s2_mu_aggressive')['result'])
"

# S4 試行ログ可視化
python -c "
import sys; sys.path.insert(0,'scripts')
from tuning_labs import plot_speed_trial_journey; plot_speed_trial_journey(); import matplotlib.pyplot as plt; plt.show()
"
```

**関連資料:** [TUNING_GUIDE.md](./TUNING_GUIDE.md)（早見表） · [SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md)（S4 詳細） · [INSTRUCTOR_GUIDE.md](./INSTRUCTOR_GUIDE.md)（講師台本）

---

## ジャーニー全体像

```
Phase 1  平坦スモーク     ref_z · 足場opt OFF · 3層が動く
    ↓
Phase 2  平坦チューニング  μ · step_freq · 失敗と成功の対比
    ↓
Phase 3  不整地          足場opt ON · 地形別 gait · boxes vs perlin
    ↓
Phase 4  5kph×凸凹坂     speed_ramp · resilient · 地形別 YAML
```

| Phase | Notebook | プリセット | tuning_labs.py | デモ GIF |
|-------|----------|------------|----------------|----------|
| 1 | [01](./notebooks/01_demo_session01_flat_smoke.ipynb) | `session01_flat_smoke` | `s1_ref_z_fail` / `s1_ref_z_ok` | demo_s01_flat |
| 2 | [02](./notebooks/02_demo_session02_flat_tune.ipynb) | `session02_flat_tune` | `s2_mu_*` / `s2_step_freq_fast` | demo_s02_tune |
| 3a | [03a](./notebooks/03_demo_session03a_rough_boxes.ipynb) | `session03_rough_boxes` | `s3_boxes_freq_*` | demo_s03_boxes |
| 3b | [04](./notebooks/04_demo_session03b_rough_perlin.ipynb) | `session03_rough_perlin` | `s3_perlin_mu_fail` | demo_s03_perlin |
| 4 | [05](./notebooks/05_demo_session04_speed_bumpy.ipynb) | `session04_bumpy_*` | `s4_*` | demo_s04_{flat,uphill,downhill} |
| **統合** | **[06](./notebooks/06_mpc_tuning_journey.ipynb)** | 上記すべて | **全 lab** | 上記すべて |

---

## Phase 1 — 平坦スモーク {#phase-1-flat-smoke}

**MPC 屋の問い:** 「まず何を固定すれば、GRF-MPC が動いていると言える？」

### 固定するもの

- `scene=flat`
- `use_foothold_optimization=False`（変数を減らす）
- `gait=trot`, `mu=0.5`

### ❌ 失敗体験 — ref_z 低すぎ

| 項目 | 値 |
|------|-----|
| Lab ID | `s1_ref_z_fail` |
| 変更 | `ref_z_scale=0.95` |
| 症状 | 数 step で `terminated=True` |
| 原因 | 目標胴体高さ不足 → 足が地面にめり込む |

```python
from tuning_labs import run_lab
run_lab("s1_ref_z_fail")  # → terminated, min_z 低下
```

### ✅ 成功体験 — baseline

| Lab ID | `s1_ref_z_ok` |
| 変更 | `ref_z_scale=1.05` |
| 確認 | 4 s headless、vx 安定 |

**MPC 屋メモ:** 摩擦円錐や Q/R を触る前に **ref_z** と **ゲイト** を確認。デモ GIF の緑矢印（GRF）を指し示せる状態を作る。

---

## Phase 2 — 平坦チューニング {#phase-2-flat-tune}

**MPC 屋の問い:** 「μ と step_freq は操舵 MPC の何に相当する？」

→ **μ ≈ タイヤ摩擦**、**step_freq ≈ 制御更新に対する指令レート**

### ❌ 失敗 — μ 積極的（0.55）

| Lab ID | `s2_mu_aggressive` |
| 症状 | vx は出るが roll 増、転倒しやすい |
| 対処 | μ↓ |

### ✅ 成功 — μ 保守的（0.35）

| Lab ID | `s2_mu_conservative` |
| 学び | vx↓ だが安定。不整地前の **安全側** の感覚 |

### ❌ 失敗 — step_freq 高すぎ（1.6 Hz）

| Lab ID | `s2_step_freq_fast` |
| 症状 | 足回しは速いが MPC 予測とずれる |
| 対処 | step_freq↓（1.2–1.4） |

**データ:** [param_study_results.json](./assets/param_study_results.json) — μ=0.55 で mean_vx 最大だが、S3/S4 では使わない。

```python
from tuning_labs import run_lab_pair
from pympc_lab import compare_runs
pairs = run_lab_pair("s2_mu_aggressive", "s2_mu_conservative")
compare_runs(pairs);  # plt.show() in notebook
```

---

## Phase 3 — 不整地 {#phase-3-rough-terrain}

**MPC 屋の問い:** 「平坦で得た μ・freq の知見をそのまま使っていい？」→ **いいえ**

### 追加ブロック

- `use_foothold_optimization=True`
- `scene=random_boxes` または `perlin`

### S3a — 箱障害

| | fail | success |
|---|------|---------|
| Lab | `s3_boxes_freq_fail` | `s3_boxes_freq_ok` |
| step_freq | 1.6 | 1.1 |
| duty | 既定 | 0.75 |
| 学び | S2 の速い trot は不可 | 保守 gait + 足場 opt |

### S3b — 連続起伏

| Lab | `s3_perlin_mu_fail` |
| mu | 0.55（積極）→ 転倒 |
| 成功側 | preset `session03_rough_perlin`（μ=0.45） |

**MPC 屋メモ:** boxes=離散障害、perlin=連続。同じ「不整地」でも **μ の安全側** が異なる。

---

## Phase 4 — 5 kph × 凸凹坂 {#phase-4-speed-bumpy}

**MPC 屋の問い:** 「指令 5 kph・20 m を凸凹の上り/下りでも達成するには？」

### 成功判定（2 モード）

| モード | 条件 | 結果 |
|--------|------|------|
| no-fall | 転倒なし & 20 m & mean≥4 kph | **3 地形とも未達** |
| resilient | 累積 20 m & falls≤max | **3 地形とも達成** |

### ❌ 失敗体験 — no-fall @ 5 kph

| Lab ID | `s4_no_fall_fail` |
| 距離 | ~3–4 m |
| ログ | [speed_terrain_trial_log.json](./assets/speed_terrain_trial_log.json) |

### 試行錯誤の方向（MPC 屋チェックリスト）

1. `speed_ramp_s` ↑（18–22 s）— 指令を漸増
2. `step_freq` ↓（1.05–1.20）
3. `duty_factor` ↑（0.76–0.82）
4. `mu` ↓（0.35–0.42）
5. 地形別に YAML 分離

### ✅ 勝ちパラメータ

| 地形 | μ | freq | duty | ramp | falls | preset |
|------|-----|------|------|------|-------|--------|
| bumpy_flat | 0.42 | 1.20 | 0.76 | 18s | 17 | `session04_bumpy_flat` |
| bumpy_uphill | 0.38 | 1.10 | 0.78 | 20s | 19 | `session04_bumpy_uphill` |
| bumpy_downhill | 0.35 | 1.05 | 0.82 | 22s | 28 | `session04_bumpy_downhill` |

| Lab ID | 用途 |
|--------|------|
| `s4_resilient_flat_win` | 凸凹平坦 20 m 再現 |
| `s4_resilient_downhill_win` | 最難関再現 |

```python
from tuning_labs import run_lab, plot_speed_trial_journey
run_lab("s4_resilient_flat_win")
plot_speed_trial_journey()
```

**デモ検証:** `python scripts/verify_workshop_assets.py` — GIF/PNG の `demo_*.meta.json` で dist≥20 m を確認。

---

## MPC 屋のトリアージフロー（全 Phase 共通）

```
転倒？
 ├─ 即倒れ        → ref_z ↑        [Phase 1]
 ├─ 加速時        → μ ↓, ramp ↑    [Phase 2, 4]
 ├─ 不整地        → freq ↓, duty ↑, 足場 opt ON [Phase 3]
 └─ 高速指令      → ramp ↑, 地形別 YAML [Phase 4]
```

詳細: [TUNING_GUIDE.md](./TUNING_GUIDE.md)

---

## 自分で 1 つ変えてみる（演習）

1. Notebook [06](./notebooks/06_mpc_tuning_journey.ipynb) Step 7 を開く
2. `run_lab("s4_resilient_flat_win")` の kwargs で **1 パラメータだけ** 変更
3. `distance_m` / `falls` / `success` を記録
4. [SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md) に 1 行追記（optional）

---

## スクリプト・ファイル対応表

| ファイル | 役割 |
|----------|------|
| [scripts/tuning_labs.py](../../scripts/tuning_labs.py) | 失敗/成功 lab 定義・実行 |
| [scripts/pympc_lab.py](../../scripts/pympc_lab.py) | sim API · TUNING_GUIDE 定数 |
| [scripts/run_speed_terrain_benchmark.py](../../scripts/run_speed_terrain_benchmark.py) | S4 グリッド試行 |
| [scripts/verify_workshop_assets.py](../../scripts/verify_workshop_assets.py) | デモ GIF/PNG 検証 |
| [assets/tuning_lab_results.json](./assets/tuning_lab_results.json) | lab 実行結果キャッシュ |
| [assets/speed_terrain_trial_log.json](./assets/speed_terrain_trial_log.json) | S4 全試行 |
| [assets/speed_terrain_results.json](./assets/speed_terrain_results.json) | S4 勝者 |

---

## 修了チェック（MPC 設計者）

- [ ] Phase 1–4 それぞれ **1 つの失敗と 1 つの成功** を lab ID で再現した
- [ ] μ / step_freq / duty / ramp の **調整方向** を地形別に説明できる
- [ ] no-fall vs resilient の違いを **評価設計** の観点で説明できる
- [ ] `tuning_labs.py --list` の lab を 1 つ選び、kwargs を 1 つ変えて再実行した
- [ ] デモ GIF を見て **正しい terrain** であることを meta JSON で確認した

**次:** [LEARNER_GUIDE.md](./LEARNER_GUIDE.md) · [WORKSHOP.md](./WORKSHOP.md)
