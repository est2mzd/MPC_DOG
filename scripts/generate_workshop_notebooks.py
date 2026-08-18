#!/usr/bin/env python3
"""Generate PyMPC workshop Jupyter notebooks."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "pympc_2day" / "notebooks"

SESSION_COMPARE = """\
## 4セッションの違い（必読）

| | **S1 本Notebook** | S2 tune | S3a boxes | S3b perlin |
|---|-------------------|---------|-----------|------------|
| **scene** | **flat（平坦）** | flat | **random_boxes（箱）** | **perlin（連続起伏）** |
| **足場最適化** | **OFF** | OFF | ON | ON |
| **主な目的** | 最小構成で動作確認 | μ / 歩調チューニング | 段差・離散障害 | 連続起伏 |
| **デモGIFで見る点** | 平坦＋標準trot | 平坦＋**速いtrot** | **箱が見える** | **うねり地形** |
| **GIF** | demo_s01_flat | demo_s02_tune | demo_s03_boxes | demo_s03_perlin |

> **S1 と S2 は地形とも平坦**です。GIFの違いは **歩調（S2は step_freq=1.75 Hz の速い trot）** と **Notebook内の実験内容** です。  
> **S3a/S3b は約9秒走行**して箱・起伏地形に入るようキャプチャしています（旧GIFは短すぎて全部平坦に見えていました）。
"""


def md(source: str) -> dict:
    return {"cell_type": "markdown", "metadata": {}, "source": source.splitlines(keepends=True)}


def code(source: str) -> dict:
    return {
        "cell_type": "code",
        "metadata": {},
        "source": source.splitlines(keepends=True),
        "outputs": [],
        "execution_count": None,
    }


def nb(cells: list, name: str) -> dict:
    return {
        "nbformat": 4,
        "nbformat_minor": 5,
        "metadata": {
            "kernelspec": {
                "display_name": "mpc-dog (uv workshop)",
                "language": "python",
                "name": "mpc-dog-workshop",
            },
            "language_info": {"name": "python", "pygments_lexer": "ipython3"},
        },
        "cells": cells,
    }


SETUP = """\
import sys
from pathlib import Path

# mpc_dog ルートを sys.path に追加
ROOT = Path.cwd()
for p in [ROOT, *ROOT.parents]:
    if (p / "scripts" / "pympc_lab.py").exists():
        ROOT = p
        break
sys.path.insert(0, str(ROOT / "scripts"))

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from pympc_lab import (
    TUNING_GUIDE,
    apply_preset,
    compare_runs,
    load_param_study,
    load_preset_yaml,
    plot_friction_cone,
    run_flat_sim,
    run_speed_terrain_sim,
    run_speed_terrain_sim_resilient,
)

%matplotlib inline
plt.rcParams["figure.figsize"] = (9, 4)
print(f"repo: {ROOT}")
"""


def theory_notebook() -> dict:
    cells = [
        md("""# 00 — 理論理解: GRF · MPC · WBC（MPC設計者向け）

**対象:** 四足制御初心者 + ADAS操舵MPC経験者  
**ゴール:** 「何を調整すると何が起きるか」を **失敗と成功のパターン** として理解する

---

## この Notebook の進め方

1. **Step 1–4:** 3層パイプラインと数式を「直觉」で理解  
2. **Step 5–6:** 摩擦円錐を **目で見る**（μ の意味）  
3. **Step 7:** MPC設計者向け **調整マトリクス** を読む  
4. **Step 8:** 簡単な数値実験（SRB の合力）  
5. **Step 9:** デモ Notebook へ進む

> 詳細版: [WORKSHOP.md](../WORKSHOP.md)
"""),
        code(SETUP),
        md("""## Step 1 — 四足制御の「定番3層」（対立ではなく接続）

```
速度指令 / ゲイト
      ↓
MPC (SRB)  … 最適化変数 = 接地反力 GRF（12次元）
      ↓
WBC相当     … Stance: GRF→関節τ / Swing: 足軌道+PD
      ↓
MuJoCo / 実機
```

**ADAS MPC との対応**

| 操舵MPC | 四足PyMPC |
|---------|-----------|
| 車両モデル | SRB（箱1個） |
| 横G・舵角制約 | **摩擦円錐** |
| MPC出力 | **GRF**（タイヤ力相当） |
| 下位 | WBC → 関節トルク |

**初心者向け一言:** MPCは「各足が地面を **どれだけ蹴るか**」を決める。WBCは「その蹴りを **関節で実現** する」。
"""),
        md("""## Step 2 — GRF（Ground Reaction Force）とは

- 足先と地面の間の力 $\\mathbf{F}_i = (F_{ix}, F_{iy}, F_{iz})$
- 4足 → **12次元** の入力（低次元で物理制約を入れやすい）
- ロボットの加速は $\\sum_i \\mathbf{F}_i$（+ 重力）で決まる

**なぜ関節角度を直接MPCしないのか？**
- 次元が高い（12関節以上）
- 摩擦円錐など **接触力の物理** を入れにくい
- GRFを決めれば CoM 運動を計画できる → 関節はWBCに任せる
"""),
        md("""## Step 3 — MPC の数式（口頭説明用）

離散時間ホライゾン $N$、サンプリング $\\Delta t$:

$$\\min \\sum_{k=0}^{N-1} \\|x_k - x_k^{ref}\\|_Q + \\|u_k\\|_R
\\quad \\text{s.t.} \\quad x_{k+1} = f_{SRB}(x_k, u_k)$$

**摩擦円錐**（各足 $i$、接触中）:

$$\\sqrt{F_{ix}^2 + F_{iy}^2} \\le \\mu F_{iz}, \\quad F_{iz}^{min} \\le F_{iz} \\le F_{iz}^{max}$$

**Convex化のコツ:** どの足が stance/swing かを **ゲイトで固定** → GRFについて凸に近づける。

PyMPC デフォルト: $N=12$, $\\Delta t=0.02$ s → **0.24 s 先読み**
"""),
        md("""## Step 4 — WBC 相当層

| 脚 | 処理 |
|----|------|
| **Stance** | MPCの $\\mathbf{F}_i$ → $\\boldsymbol{\\tau} = \\mathbf{J}^\\top \\mathbf{F} + \\text{PD}$ |
| **Swing** | Bezier足軌道 + PD（MPCのGRFは使わない） |

MPC設計者が触るのは主に **MPC層**。WBCゲインは「追従の硬さ」= 計画通りに蹴れるかどうか。
"""),
        md("## Step 5 — 摩擦円錐を可視化（μ を上げ下げすると？）"),
        code("""\
fig, axes = plt.subplots(1, 3, figsize=(12, 4))
for ax, mu in zip(axes, [0.3, 0.5, 0.8]):
    plot_friction_cone(mu=mu, f_max=120, ax=ax)
fig.suptitle("Higher mu allows larger horizontal Fx for the same Fz", y=1.02)
plt.tight_layout()
"""),
        md("""**MPC設計者メモ**

| μ | 典型の使いどころ | 失敗パターン |
|---|----------------|--------------|
| 低 (0.35–0.45) | 氷・濡れ床・不整地 | 加速不足、指令追従が鈍い |
| 中 (0.5) | 平坦デフォルト | — |
| 高 (0.6+) | 高摩擦床・積極走行 | 転倒・横滑り・オーバーシュート |

⚠️ **sim の地面摩擦** と **MPC の μ** は別パラメータ。両方の意味を混同しないこと。
"""),
        md("## Step 6 — SRB の合力デモ（F=ma の直觉）"),
        code("""\
m = 15.0  # Go2 近似 [kg]
g = 9.81
# 4足がそれぞれ垂直100N、前足2本が追加で水平30N前向き
Fz_total = 4 * 100
Fx_total = 2 * 30
ax = Fx_total / m
print(f"合力 Fx={Fx_total}N → ax={ax:.2f} m/s²")
print(f"垂直 Fz={Fz_total}N vs mg={m*g:.0f}N → 浮き/沈み: {(Fz_total - m*g):+.0f}N")

# mu=0.5 の摩擦円錐内か？
mu = 0.5
for name, fx, fz in [("前足FL", 30, 100), ("過剰水平", 80, 100)]:
    ok = abs(fx) <= mu * fz
    print(f"{name}: |Fx|={abs(fx)} <= mu*Fz={mu*fz:.0f} ? {ok}")
"""),
        md("## Step 7 — MPC設計者向け 調整マトリクス（成功/失敗パターン）"),
        code("""\
df = pd.DataFrame(TUNING_GUIDE)
cols = ["param", "what", "raise", "lower", "failure_symptom", "failure_fix", "success_sign"]
display(df[cols])
"""),
        md("""### 現場で使うトリアージ（転倒したら）

1. **即転倒（数秒以内）** → `ref_z`↑, `step_freq`↓, `mu`↓, 足場opt OFF  
2. **加速しない** → `mu`↑（ただし転倒リスク）, `grf_max`↑, 速度指令↓  
3. **滑る・横倒れ** → `mu`↓, `duty_factor`↑  
4. **不整地で変な足場** → 足場opt OFF で比較 → 地形推定を疑う  
5. **MPCが遅い** → `solver_mode='speed'`, horizon↓

次の Notebook で **実際に sim を回して** 体感します。
"""),
        md("""## Step 8 — チェックリスト

- [ ] GRF / MPC / WBC の役割を1文ずつ説明できる  
- [ ] 摩擦円錐が何を制約しているか説明できる  
- [ ] `mu` を上げると **何が起きやすいか**（加速 vs 転倒）を説明できる  
- [ ] 転倒時の最初の3つのアクションを言える  

---

## 次へ

| Notebook | 内容 |
|----------|------|
| [01_demo_session01_flat_smoke.ipynb](./01_demo_session01_flat_smoke.ipynb) | 平坦スモーク + GRF可視化 |
| [02_demo_session02_flat_tune.ipynb](./02_demo_session02_flat_tune.ipynb) | μ / 歩調チューニング |
| [03_demo_session03a_rough_boxes.ipynb](./03_demo_session03a_rough_boxes.ipynb) | 不整地 boxes |
| [04_demo_session03b_rough_perlin.ipynb](./04_demo_session03b_rough_perlin.ipynb) | 不整地 perlin |
"""),
    ]
    return nb(cells, "00")


def demo_s1_notebook() -> dict:
    cells = [
        md("""# 01 — デモ: Session 1 平坦スモーク（GRF可視化）

**目的:** 3層パイプラインが **動く** ことを確認。MuJoCo GUI で **緑矢印 = GRF** を見せる。

**所要:** uv workshop 環境、Quadruped-PyMPC clone 済み
"""),
        md(SESSION_COMPARE.replace("**S1 本Notebook**", "**S1 本Notebook ← 今ここ**")),
        md("""### このセッション固有のポイント

- **地形:** `scene=flat` — チェッカー模様の平坦床のみ（障害物なし）
- **足場最適化:** OFF — デバッグを単純化
- **デモGIF:** 標準 trot（step_freq=1.4 Hz）。画面左上に `Session 1 | scene=flat` と表示
- **Notebook:** ref_z を意図的に下げて **転倒 vs 成功** をグラフで比較

![Session 1 demo](../assets/demo_s01_flat.gif)
"""),
        md("""## Step 0 — このデモで学ぶこと

| 学習項目 | 成功のサイン |
|----------|--------------|
| プリセット適用 | config.py が意図通りにパッチされる |
| 平坦trot | 30s以上転倒しない |
| GRF可視化 | 各足に緑矢印 |
| 失敗からの復帰 | ref_z 調整で立ち直れる |
"""),
        code(SETUP),
        md("## Step 1 — 環境確認"),
        code("""\
pympc = ROOT / "external" / "Quadruped-PyMPC"
assert pympc.is_dir(), "Run: ./scripts/setup_references.sh"
print("PyMPC OK:", pympc)
print("Preset:", ROOT / "configs/pympc_presets/session01_flat_smoke.yaml")
"""),
        md("## Step 2 — プリセット YAML を読む（何を最小構成にしているか）"),
        code("""\
preset = load_preset_yaml("session01_flat_smoke")
import yaml
print(yaml.dump(preset, allow_unicode=True))
"""),
        md("""**設計意図（MPC設計者向け）**

- `use_foothold_optimization: False` → 初回デバッグの失敗要因を排除  
- `scene: flat` → 地形変数ゼロ  
- `gait: trot` → 最安定ゲイト  
- `mu: 0.5` → 標準摩擦モデル
"""),
        md("## Step 3 — プリセットを config.py に適用"),
        code("""\
cfg_path = apply_preset("session01_flat_smoke")
print("Applied ->", cfg_path)
"""),
        md("## Step 4 — 短時間 headless sim（4秒）で動作確認"),
        code("""\
# 初回は acados codegen で数分かかることがあります
metrics = run_flat_sim(seconds=4.0)
print(f"mean_vx={metrics['mean_vx']:.3f} m/s, min_z={metrics['min_z']:.3f} m, terminated={metrics['terminated']}")
"""),
        md("""## Step 5 — ❌ 意図的失敗: ref_z が低すぎる

**症状:** 即転倒 / 胴体が沈む / 足が地面に刺さる  
**原因:** CoM目標高度が低く、MPCが十分なGRFを計画できない  
**教訓:** まず `ref_z` を疑う（Session 1 で最も多い失敗）
"""),
        code("""\
apply_preset("session01_flat_smoke")
bad = run_flat_sim(seconds=3.0, ref_z_scale=0.85)
good = run_flat_sim(seconds=3.0, ref_z_scale=1.05)
fig = compare_runs([("FAIL ref_z×0.85", bad), ("OK ref_z×1.05", good)])
plt.show()
print("FAIL terminated:", bad["terminated"], "| OK terminated:", good["terminated"])
"""),
        md("""## Step 6 — ✅ 成功パターンの確認

- `min_z` が一定（大きく落ちない）  
- `terminated=False`  
- `mean_vx > 0`（forward 指令時）

**retry プリセット:** `session01_flat_smoke_retry`（ref_z 微増）— 本番前に試す
"""),
        code("""\
try:
    apply_preset("session01_flat_smoke_retry")
    retry = run_flat_sim(seconds=4.0)
    print(retry)
except Exception as e:
    print("retry preset optional:", e)
"""),
        md("""## Step 7 — デモ映像（生成済み）

[`../assets/demo_s01_flat.gif`](../assets/demo_s01_flat.gif) — `capture_demo_frames.py` で生成（平坦＋標準trot）

---

## Step 8 — チェックリスト

- [ ] プリセット → sim の流れを再現できた  
- [ ] ref_z 失敗 vs 成功を **グラフで説明** できる  
- [ ] 「MPCはGRFを計画、GUI矢印で見える」と言える  
- [ ] S2/S3 との違い（地形・足場opt・目的）を説明できる  

**次:** [02_demo_session02_flat_tune.ipynb](./02_demo_session02_flat_tune.ipynb) で μ / 歩調を触る
"""),
    ]
    return nb(cells, "01")


def demo_s2_notebook() -> dict:
    cells = [
        md("""# 02 — デモ: Session 2 平坦パラメータチューニング

**目的:** MPC設計者として **μ / step_freq / gain** を触り、成功と失敗の体感を得る。
"""),
        md(SESSION_COMPARE.replace("S2 tune", "**S2 tune ← 今ここ**")),
        md("""### このセッション固有のポイント

- **地形:** S1 と同じ `scene=flat`（平坦）。GIF だけでは S1 と区別しにくい  
- **GIFとの違い:** step_freq=**1.75 Hz**（S1 の 1.4 Hz より速い trot → 足振りが速く見える）  
- **Notebookとの違い:** μ / step_freq / duty_factor を **数値スイープ** して成功・失敗パターンを体感  
- 画面左上: `Session 2 | scene=flat | step_freq=1.75 Hz (fast trot)`

![Session 2 demo](../assets/demo_s02_tune.gif)
"""),
        md("""## Step 0 — このデモのゴール

| 触るパラメータ | 体感すべきこと |
|----------------|----------------|
| `mu` | 加速 vs 転倒のトレードオフ |
| `step_freq` | 歩調と MPC 追従性 |
| `duty_factor` | 安定 vs 敏捷 |
"""),
        code(SETUP),
        md("## Step 1 — ベースラインプリセット適用"),
        code("""\
apply_preset("session02_flat_tune")
preset = load_preset_yaml("session02_flat_tune")
print("tuning hints:", preset.get("tuning_hints"))
"""),
        md("## Step 2 — ベースライン計測（4秒）"),
        code("""\
baseline = run_flat_sim(seconds=4.0)
print(baseline)
fig = compare_runs([("baseline", baseline)])
plt.show()
"""),
        md("""## Step 3 — ❌ vs ✅ 実験 A: 摩擦係数 mu

**仮説:** μ↑ → 水平GRFを取りやすく加速するが、高すぎると不安定

| ケース | mu | 期待 |
|--------|-----|------|
| 保守的 | 0.35 | 安定、加速弱い |
| 標準 | 0.5 | バランス |
| 積極 | 0.65 | 加速↑、転倒リスク↑ |
"""),
        code("""\
apply_preset("session02_flat_tune")
runs = []
for mu, label in [(0.35, "mu=0.35 conservative"), (0.5, "mu=0.5 baseline"), (0.65, "mu=0.65 aggressive")]:
    m = run_flat_sim(seconds=4.0, mu=mu)
    runs.append((label, m))
    print(label, "mean_vx=", f"{m['mean_vx']:.3f}", "terminated=", m["terminated"])

fig = compare_runs(runs)
plt.suptitle("mu sweep — vx vs stability (check terminated flag)", y=1.02)
plt.show()
"""),
        md("""**MPC設計者メモ — mu**

- ✅ 成功: 狙い速度に近い、姿勢安定、`terminated=False`  
- ❌ 失敗: 横滑り・ピッチ/ロール増大 → **μを下げる**  
- ❌ 失敗: 加速不足 → μ↑を試すが、**sim地面摩擦も確認**
"""),
        md("""## Step 4 — ❌ vs ✅ 実験 B: 歩調 step_freq

**仮説:** 速すぎると MPC の 0.24s 先読みが追いつかない
"""),
        code("""\
apply_preset("session02_flat_tune")
runs = []
for freq, label in [(1.0, "1.0Hz slow"), (1.4, "1.4Hz baseline"), (1.8, "1.8Hz fast")]:
    m = run_flat_sim(seconds=4.0, step_freq=freq)
    runs.append((label, m))
    print(label, "mean_vx=", f"{m['mean_vx']:.3f}", "max_roll=", f"{m['max_roll_deg']:.1f}°")

fig = compare_runs(runs)
plt.show()
"""),
        md("""**MPC設計者メモ — step_freq**

- ✅ 成功: 周期安定、roll 小さい  
- ❌ 失敗: 足刺さり・振動 → **step_freq↓** or **duty_factor↑**  
- ❌ 失敗: MPC solve timeout → `solver_mode='speed'`
"""),
        md("## Step 5 — パラメータスタディ結果（事前計測データ）"),
        code("""\
results = load_param_study()
df = pd.DataFrame(results)
display(df)

mu_df = df[df["step_freq"] == 1.4].sort_values("mu")
freq_df = df[df["mu"] == 0.5].sort_values("step_freq")

fig, axes = plt.subplots(1, 2, figsize=(11, 4))
axes[0].plot(mu_df["mu"], mu_df["mean_vx"], "o-")
axes[0].set(xlabel="mu", ylabel="mean vx [m/s]", title="mu sweep (6s headless)")
axes[1].plot(freq_df["step_freq"], freq_df["mean_vx"], "s-", color="green")
axes[1].set(xlabel="step_freq [Hz]", ylabel="mean vx [m/s]", title="step_freq sweep")
plt.tight_layout()
"""),
        md("""> 6秒 headless ではトレンドが単調にならないことがある。**Step 3–4 の比較実験** の方が体感に適す。

## Step 6 — ❌ vs ✅ 実験 C: duty_factor（支持比率）

不整地前に平坦で体感しておくと Session 3 が楽。
"""),
        code("""\
apply_preset("session02_flat_tune")
runs = []
for duty, label in [(0.55, "duty=0.55 short stance"), (0.65, "duty=0.65 baseline"), (0.75, "duty=0.75 long stance")]:
    m = run_flat_sim(seconds=4.0, duty_factor=duty)
    runs.append((label, m))
fig = compare_runs(runs)
plt.show()
"""),
        md("""## Step 7 — 自分で1つ設計（成功体験）

**課題:** 「安定優先で vx=0.3m/s 程度」を目指すパラメータを1組決めよ。

ヒント: `mu=0.45`, `step_freq=1.2`, `duty_factor=0.72`
"""),
        code("""\
# ここを編集して試す
MY_MU = 0.45
MY_FREQ = 1.2
MY_DUTY = 0.72

apply_preset("session02_flat_tune")
mine = run_flat_sim(seconds=5.0, mu=MY_MU, step_freq=MY_FREQ, duty_factor=MY_DUTY)
print("My config:", mine)
fig = compare_runs([("baseline", baseline), ("my design", mine)])
plt.show()
"""),
        md("""## Step 8 — チェックリスト

- [ ] μ を1回変えて効果を説明できる  
- [ ] step_freq を1回変えて効果を説明できる  
- [ ] 転倒時に TUNING_GUIDE から **最初の対処** を選べる  
- [ ] S1 との違い（GIF=速いtrot、Notebook=パラメータ実験）を説明できる  

**次:** [03_demo_session03a_rough_boxes.ipynb](./03_demo_session03a_rough_boxes.ipynb)
"""),
        code("""\
pd.DataFrame(TUNING_GUIDE)[["param", "failure_symptom", "failure_fix"]]
"""),
    ]
    return nb(cells, "02")


def demo_s3a_notebook() -> dict:
    cells = [
        md("""# 03 — デモ: Session 3a 不整地 random_boxes

**目的:** 足場最適化 ON + 段差 terrain で **MPCが着地点も計画** する様子を理解する。

**難度:** 中（perlin より先に試す）
"""),
        md(SESSION_COMPARE.replace("S3a boxes", "**S3a boxes ← 今ここ**")),
        md("""### このセッション固有のポイント

- **地形:** `scene=random_boxes` — x≈1 m 以降に **MuJoCo の箱ジオメトリ** が並ぶ（平坦スタート→障害物エリアへ進入）
- **足場最適化:** ON — 着地点を MPC が計画
- **デモGIF:** 約9秒走行＋ワイドショット intro で **箱がはっきり見える**。左上 `Session 3a | scene=random_boxes`
- **旧GIFが平坦に見えた理由:** 600 step（≈1.2 s）では箱エリア（x≈1 m）に到達する前に終了していた

![Session 3a demo](../assets/demo_s03_boxes.gif)
"""),
        code(SETUP),
        md("## Step 1 — プリセット確認"),
        code("""\
import yaml
preset = load_preset_yaml("session03_rough_boxes")
print(yaml.dump(preset, allow_unicode=True))
"""),
        md("""**設計意図**

- `use_foothold_optimization: True` → 不整地の核心  
- `step_freq: 1.2` → 低め（安定優先）  
- `mu: 0.48` → やや保守的
"""),
        md("## Step 2 — プリセット適用"),
        code("""\
apply_preset("session03_rough_boxes")
"""),
        md("""## Step 3 — ❌ vs ✅ 足場 opt OFF vs ON 比較

**教訓:** OFF で転倒しやすくても、ON で変な足場 → 地形推定/制約の問題かも。必ず **両方** 見る。
"""),
        code("""\
apply_preset("session03_rough_boxes")
# OFF: 平坦用設定に近い
off = run_flat_sim(seconds=5.0, scene="random_boxes", use_foothold_optimization=False, step_freq=1.4)
# ON: preset 通り
apply_preset("session03_rough_boxes")
on = run_flat_sim(seconds=5.0, scene="random_boxes", use_foothold_optimization=True, step_freq=1.2, mu=0.48)
fig = compare_runs([("FOOTHOLD OFF", off), ("FOOTHOLD ON", on)])
plt.show()
print("OFF terminated:", off["terminated"], "min_z:", f"{off['min_z']:.3f}")
print("ON  terminated:", on["terminated"],  "min_z:", f"{on['min_z']:.3f}")
"""),
        md("""## Step 4 — ❌ 典型失敗: step_freq 高すぎ

不整地では **速さより安定**。Session 2 の知見を適用。
"""),
        code("""\
apply_preset("session03_rough_boxes")
fast = run_flat_sim(seconds=4.0, scene="random_boxes", step_freq=1.6, use_foothold_optimization=True)
slow = run_flat_sim(seconds=4.0, scene="random_boxes", step_freq=1.1, duty_factor=0.75, use_foothold_optimization=True)
fig = compare_runs([("FAIL freq=1.6", fast), ("OK freq=1.1 duty=0.75", slow)])
plt.show()
"""),
        md("""## Step 5 — デモ映像

[`../assets/demo_s03_boxes.gif`](../assets/demo_s03_boxes.gif) — 箱地形＋足場 opt ON

---

## Step 6 — MPC設計者チェックリスト

- [ ] 足場 opt ON の **目的** を1文で言える  
- [ ] OFF/ON 比較で何が変わるか説明できる  
- [ ] 不整地では step_freq↓ / duty↑ / mu↓ の **トリアージ** ができる  
- [ ] S1/S2（平坦）との違いを **地形と足場opt** で説明できる  

**次:** [04_demo_session03b_rough_perlin.ipynb](./04_demo_session03b_rough_perlin.ipynb)
"""),
    ]
    return nb(cells, "03a")


def demo_s3b_notebook() -> dict:
    cells = [
        md("""# 04 — デモ: Session 3b 不整地 perlin（本番デモ）

**目的:** 連続起伏 terrain でコンサル本番デモ。Session 3a の知見 + さらに保守的チューニング。
"""),
        md(SESSION_COMPARE.replace("S3b perlin", "**S3b perlin ← 今ここ**")),
        md("""### このセッション固有のポイント

- **地形:** `scene=perlin` — **height field** による連続的なうねり（箱のような段差ではない）
- **足場最適化:** ON（S3a と同様）
- **チューニング:** step_freq=1.15 / duty=0.75 / mu=0.45 と **S3a より保守的**
- **デモGIF:** 低めカメラ＋約9秒走行で **地面の起伏** が見える。左上 `Session 3b | scene=perlin`
- **S3a との違い:** boxes=離散障害、perlin=連続起伏（難易度・見た目・調整方針が異なる）

![Session 3b demo](../assets/demo_s03_perlin.gif)
"""),
        code(SETUP),
        md("## Step 1 — プリセット確認"),
        code("""\
import yaml
print(yaml.dump(load_preset_yaml("session03_rough_perlin"), allow_unicode=True))
"""),
        md("""| 項目 | 値 | 意図 |
|------|-----|------|
| scene | perlin | 連続起伏 |
| step_freq | 1.15 | 低め |
| duty_factor | 0.75 | 支持長め |
| mu | 0.45 | 安全側 |
"""),
        md("## Step 2 — headless 5秒検証"),
        code("""\
apply_preset("session03_rough_perlin")
m = run_flat_sim(seconds=5.0, scene="perlin")
print(m)
fig = compare_runs([("perlin preset", m)])
plt.show()
"""),
        md("""## Step 3 — ❌ vs ✅ mu の安全側調整

perlin では **積極的な mu は転倒要因** になりやすい。
"""),
        code("""\
apply_preset("session03_rough_perlin")
runs = []
for mu, label in [(0.55, "mu=0.55 aggressive"), (0.45, "mu=0.45 baseline"), (0.35, "mu=0.35 conservative")]:
    r = run_flat_sim(seconds=4.0, scene="perlin", mu=mu)
    runs.append((label, r))
fig = compare_runs(runs)
plt.show()
"""),
        md("""## Step 4 — boxes vs perlin 比較（難易度の違い）

Session 3a の結果と並べて説明すると効果的。
"""),
        code("""\
apply_preset("session03_rough_boxes")
boxes = run_flat_sim(seconds=4.0, scene="random_boxes")
apply_preset("session03_rough_perlin")
perlin = run_flat_sim(seconds=4.0, scene="perlin")
fig = compare_runs([("random_boxes", boxes), ("perlin", perlin)])
plt.suptitle("terrain difficulty comparison", y=1.02)
plt.show()
"""),
        md("""## Step 5 — 本番デモ脚本

1. Session 1 GIF → 3層説明（平坦・足場opt OFF）  
2. Session 2 Notebook → μ / 歩調（平坦・パラメータ実験）  
3. Session 3 perlin GIF → 足場 opt + 連続起伏  

映像: [`../assets/demo_s03_perlin.gif`](../assets/demo_s03_perlin.gif)

---

## Step 6 — ワークショップ完了チェック

- [ ] Session 1–3 すべて再現可能  
- [ ] 失敗時のトリアージを **TUNING_GUIDE** から選べる  
- [ ] 4セッションの **地形・足場opt・目的** の違いを説明できる  
- [ ] お客様向け1文: 「MPCが **どこに足を置き、どれだけ蹴るか** を計画」  

---

## 次へのステップ

- 実機: muse + `ros2/run_controller.py`  
- 理論復習: [00_theory_grf_mpc_wbc.ipynb](./00_theory_grf_mpc_wbc.ipynb)  
- 統合資料: [WORKSHOP.md](../WORKSHOP.md)
"""),
    ]
    return nb(cells, "04")


SESSION04_COMPARE = """\
## 5セッションの違い（Session 4 追加）

| | S1 flat | S2 tune | S3a boxes | S3b perlin | **S4 speed+bumpy** |
|---|---------|---------|-----------|------------|---------------------|
| **scene** | flat | flat | random_boxes | perlin | **bumpy_flat / uphill / downhill** |
| **速度** | 低速 smoke | 歩調チューニング | 保守 trot | 保守 trot | **指令 5 kph + ランプ** |
| **距離** | ~数 m | ~数 m | ~数 m | ~数 m | **累積 20 m** |
| **足場 opt** | OFF | OFF | ON | ON | **ON** |
| **モード** | 単一 run | 単一 run | 単一 run | 単一 run | **resilient（転倒 reset）** |
"""


def demo_s4_notebook() -> dict:
    cells = [
        md("""# 05 — デモ: Session 4 高速 × 凸凹地形（5 kph / 20 m）

**目的:** 指令速度 **5 kph** で、凸凹の **平坦・上り坂・下り坂** を **20 m 以上** 走る。  
試行錯誤の過程を Notebook 内に残し、うまくいくパラメータまで到達する。

> **正直な結果:** 5 kph **no-fall** は 3 地形とも未達。**resilient モード**（転倒後 reset、累積距離）で 20 m を達成。  
> 詳細ログ: [SPEED_TERRAIN_TRIAL_LOG.md](../SPEED_TERRAIN_TRIAL_LOG.md)
"""),
        md(SESSION04_COMPARE.replace("**S4 speed+bumpy**", "**S4 speed+bumpy ← 今ここ**")),
        md("""### 3 シナリオのデモ GIF

| 地形 | 説明 | GIF |
|------|------|-----|
| 凸凹平坦 | Perlin、傾斜なし | ![flat](../assets/demo_s04_flat.gif) |
| 凸凹上り | Perlin + pitch +0.08 rad | ![uphill](../assets/demo_s04_uphill.gif) |
| 凸凹下り | Perlin + pitch -0.08 rad | ![downhill](../assets/demo_s04_downhill.gif) |
"""),
        code(SETUP),
        md("""## Step 1 — 試行ログの読み込み

`speed_terrain_trial_log.json` に no-fall / resilient の全試行が記録されています。
"""),
        code("""\
import json
from pathlib import Path

log_path = ROOT / "docs/pympc_2day/assets/speed_terrain_trial_log.json"
results_path = ROOT / "docs/pympc_2day/assets/speed_terrain_results.json"
trial_log = json.loads(log_path.read_text())
winners = json.loads(results_path.read_text())

df = pd.DataFrame(trial_log)
summary = df.groupby(["scene", "mode"]).agg(
    trials=("success", "count"),
    wins=("success", "sum"),
    max_dist=("distance_m", "max"),
)
print(summary)
print("\\n=== Winners ===")
for scene, w in winners.items():
    r = w["result"]
    print(f"{scene}: {r['distance_m']:.1f} m, falls={r.get('falls','-')}, mean={r.get('mean_kph',0):.2f} kph")
"""),
        md("""## Step 2 — ❌ no-fall @ 5 kph（最初の失敗）

凸凹平坦で **転倒なし 20 m** を試みると、数 m で終了します。
"""),
        code("""\
apply_preset("session04_speed_bumpy_base")
fail = run_speed_terrain_sim(
    scene="bumpy_flat",
    target_speed_kph=5.0,
    min_distance_m=20.0,
    max_seconds=30.0,
    mu=0.42,
    step_freq=1.35,
    duty_factor=0.74,
    ref_z_scale=1.08,
    speed_ramp_s=12.0,
)
print(f"distance={fail['distance_m']:.2f} m  terminated={fail['terminated']}  mean_kph={fail['mean_kph']:.2f}")
fig = compare_runs([("no-fall 5kph (fail)", fail)])
plt.show()
"""),
        md("""## Step 3 — 試行錯誤: 速度ランプ + 保守 gait

- `speed_ramp_s` ↑ … 指令を漸増（18–22 s）
- `step_freq` ↓ / `duty_factor` ↑ … 支持長め
- `mu` ↓ … 摩擦円錐を保守的に
"""),
        code("""\
trials = []
for ramp, freq, duty, mu in [
    (12.0, 1.35, 0.74, 0.42),
    (18.0, 1.20, 0.76, 0.42),
    (20.0, 1.10, 0.78, 0.38),
]:
    r = run_speed_terrain_sim(
        scene="bumpy_uphill",
        target_speed_kph=5.0,
        min_distance_m=20.0,
        max_seconds=40.0,
        mu=mu,
        step_freq=freq,
        duty_factor=duty,
        ref_z_scale=1.08,
        speed_ramp_s=ramp,
    )
    trials.append((f"ramp={ramp}s f={freq} d={duty} mu={mu}", r))
    print(r["distance_m"], r["success"])

fig = compare_runs(trials)
plt.suptitle("no-fall tuning attempts (still failing at 20m)", y=1.02)
plt.show()
"""),
        md("""## Step 4 — ✅ resilient モードで 20 m 達成

転倒時に env を reset し、**累積距離** が 20 m に達するまで再試行。  
各地形の勝ちパラメータは `speed_terrain_results.json` / プリセット YAML に保存。
"""),
        code("""\
WINNERS = {
    "bumpy_flat": dict(mu=0.42, step_freq=1.20, duty_factor=0.76, ref_z_scale=1.07, speed_ramp_s=18.0, max_falls=22),
    "bumpy_uphill": dict(mu=0.38, step_freq=1.10, duty_factor=0.78, ref_z_scale=1.08, speed_ramp_s=20.0, max_falls=21),
    "bumpy_downhill": dict(mu=0.35, step_freq=1.05, duty_factor=0.82, ref_z_scale=1.10, speed_ramp_s=22.0, max_falls=25),
}

runs = []
for scene, spec in WINNERS.items():
    p = {k: v for k, v in spec.items() if k != "max_falls"}
    mf = spec["max_falls"]
    r = run_speed_terrain_sim_resilient(
        scene=scene,
        target_speed_kph=5.0,
        min_distance_m=20.0,
        max_seconds=120.0,
        max_falls=mf,
        **p,
    )
    runs.append((f"{scene} OK={r['success']}", r))
    print(scene, r["distance_m"], r["falls"], r["success"])

fig, ax = plt.subplots(figsize=(9, 4))
for label, r in runs:
    ax.plot(r["x"], r["vx"] * 3.6, label=label, lw=1.2)
ax.axhline(5.0, color="k", ls="--", lw=0.8, alpha=0.5, label="target 5 kph")
ax.set_xlabel("cumulative distance [m]")
ax.set_ylabel("vx [kph]")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_title("Resilient runs: velocity vs cumulative distance")
plt.tight_layout()
plt.show()
"""),
        md("""## Step 5 — 3 地形の比較表

| 地形 | mu | step_freq | duty | ramp [s] | 累積距離 | 転倒 |
|------|-----|-----------|------|----------|----------|------|
| bumpy_flat | 0.42 | 1.20 | 0.76 | 18 | 20 m | 17 |
| bumpy_uphill | 0.38 | 1.10 | 0.78 | 20 | 20 m | 16 |
| bumpy_downhill | 0.35 | 1.05 | 0.82 | 22 | 20 m | 24 |

**下り坂が最難:** duty↑・mu↓・長いランプが必要。

---

## Step 6 — チェックリスト

- [ ] 5 kph = 1.39 m/s、`vel_mult = target_mps / hip_height` の関係を説明できる  
- [ ] no-fall vs resilient の **成功条件の違い** を説明できる  
- [ ] 上り / 下りで **mu・duty・ramp** の調整方向を説明できる  
- [ ] 試行ログ JSON を読み、自分で 1 パラメータ変えて再試行できる  

**関連:** [WORKSHOP.md](../WORKSHOP.md) §7 Session 4 · [SPEED_TERRAIN_TRIAL_LOG.md](../SPEED_TERRAIN_TRIAL_LOG.md)
"""),
    ]
    return nb(cells, "05")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    notebooks = [
        ("00_theory_grf_mpc_wbc.ipynb", theory_notebook()),
        ("01_demo_session01_flat_smoke.ipynb", demo_s1_notebook()),
        ("02_demo_session02_flat_tune.ipynb", demo_s2_notebook()),
        ("03_demo_session03a_rough_boxes.ipynb", demo_s3a_notebook()),
        ("04_demo_session03b_rough_perlin.ipynb", demo_s3b_notebook()),
        ("05_demo_session04_speed_bumpy.ipynb", demo_s4_notebook()),
    ]
    for name, content in notebooks:
        path = OUT / name
        path.write_text(json.dumps(content, ensure_ascii=False, indent=1), encoding="utf-8")
        print("wrote", path)


if __name__ == "__main__":
    main()
