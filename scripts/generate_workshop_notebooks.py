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

from tuning_labs import (
    TUNING_LABS,
    list_labs,
    run_lab,
    run_lab_pair,
    plot_speed_trial_journey,
    plot_param_study_mu,
    load_cached_lab_results,
)

%matplotlib inline
plt.rcParams["figure.figsize"] = (9, 4)
print(f"repo: {ROOT}")
"""

THEORY_SETUP = """\
import sys
from pathlib import Path

ROOT = Path.cwd()
for p in [ROOT, *ROOT.parents]:
    if (p / "scripts" / "pympc_lab.py").exists():
        ROOT = p
        break
sys.path.insert(0, str(ROOT / "scripts"))

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from pympc_lab import TUNING_GUIDE, plot_friction_cone

%matplotlib inline
plt.rcParams["figure.figsize"] = (9, 4)
print(f"repo: {ROOT}")
"""


def theory_notebook() -> dict:
    cells = [
        md("""# 00 — 理論理解: GRF · MPC · WBC（MPC設計者向け）

**対象:** 四足制御初心者 + ADAS操舵MPC経験者  
**ゴール:** 3 層アーキテクチャを **数式と数値デモ** で結びつけ、「何を調整すると何が起きるか」を理解する

---

## この Notebook の進め方

1. **Step 1:** 3 層パイプライン概観  
2. **Step 2:** 機体モデル化（SRB）の理論式 + **数値デモ A**  
3. **Step 3:** 足接触反力（GRF）の理論式 + **数値デモ B**  
4. **Step 4:** 3 層アーキテクチャと数式の紐付け + **数値デモ C**  
5. **Step 5–6:** MPC 定式化 · WBC 相当層  
6. **Step 7–8:** 摩擦円錐可視化 · SRB 合力  
7. **Step 9:** 調整マトリクス · チェックリスト  

> 詳細版: [WORKSHOP.md](../WORKSHOP.md)
"""),
        code(THEORY_SETUP),
        md("""## Step 1 — 四足制御の「定番3層」（対立ではなく接続）

```
Layer 1  速度指令 / ゲイト (trot)     →  接触スケジュール s_i(k)
Layer 2  MPC (SRB)                   →  最適 GRF u = [F_1…F_4]  (12D)
Layer 3  WBC 相当                    →  Stance: τ=J^T F / Swing: 軌道+PD
         MuJoCo / 実機               →  全関節トルク τ
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
        md("""## Step 2 — 機体モデル化（SRB）の理論式

四足ロボットを **質量 $m$・慣性テンソル $\\mathbf{I}$** の剛体 1 個（Single Rigid Body, SRB）で近似する。

### 状態ベクトル（centroidal 系のイメージ）

$$
\\mathbf{x} = \\big[ \\mathbf{p}^\\top, \\boldsymbol{\\Theta}^\\top, \\mathbf{v}^\\top, \\boldsymbol{\\omega}^\\top \\big]^\\top
$$

| 記号 | 意味 |
|------|------|
| $\\mathbf{p} \\in \\mathbb{R}^3$ | CoM（重心）位置 |
| $\\boldsymbol{\\Theta}$ | 姿勢（Euler 角または quaternion） |
| $\\mathbf{v}$ | CoM 並進速度 |
| $\\boldsymbol{\\omega}$ | 角速度（ボディ or 世界系 — 実装に依存） |

### 入力

各足 $i \\in \\{\\mathrm{FL,FR,RL,RR}\\}$ の GRF:

$$
\\mathbf{F}_i = (F_{ix}, F_{iy}, F_{iz})^\\top, \\quad
\\mathbf{u} = [\\mathbf{F}_1^\\top, \\ldots, \\mathbf{F}_4^\\top]^\\top \\in \\mathbb{R}^{12}
$$

### 並進（Newton の第 2 法則）

$$
m \\dot{\\mathbf{v}} = \\sum_{i=1}^{4} \\mathbf{F}_i + m \\mathbf{g}
$$

### 回転（Euler の運動方程式, CoM 周り）

$$
\\mathbf{I} \\dot{\\boldsymbol{\\omega}} + \\boldsymbol{\\omega} \\times (\\mathbf{I}\\boldsymbol{\\omega})
  = \\sum_{i=1}^{4} \\mathbf{r}_i \\times \\mathbf{F}_i
$$

$\\mathbf{r}_i$ は CoM から足 $i$ の作用点へのベクトル。

### 離散化（MPC で使う形）

サンプリング $\\Delta t$ で

$$
\\mathbf{x}_{k+1} = f_{\\mathrm{SRB}}(\\mathbf{x}_k, \\mathbf{u}_k)
$$

PyMPC デフォルト: $N=12$, $\\Delta t=0.02$ s → **0.24 s 先読み**。

> **MPC 屋メモ:** 全 12 関節を直接 MPC するのではなく、**低次元の $\\mathbf{u}$（GRF）** で CoM 運動を計画し、関節は Layer 3 に任せる。
"""),
        code("""\
# --- 数値デモ A: SRB 並進・回転（足反力 → 加速度）---
m = 15.0          # Go2 近似 [kg]
Izz = 0.35        # ヨー慣性 [kg·m²]（教学用の代表値）
g = 9.81

# 4 足の GRF [N] と CoM から見た足位置 [m]（平面近似: x 前, z 上）
feet = ["FL", "FR", "RL", "RR"]
F = np.array([
    [40, 0, 100],   # FL: 前向きに蹴る
    [40, 0, 100],   # FR
    [-10, 0, 100],  # RL
    [-10, 0, 100],  # RR
], dtype=float)
r = np.array([
    [0.25, 0, -0.15],   # FL: 前左
    [0.25, 0, 0.15],    # FR
    [-0.25, 0, -0.15],  # RL
    [-0.25, 0, 0.15],   # RR
])

F_sum = F.sum(axis=0)
a = F_sum[:3] / m + np.array([0.0, 0.0, -g])  # v_dot = sum(F)/m + g

# 2D トルク（y 軸回り）: tau_y = sum(r_x * F_z - r_z * F_x)
tau_y = np.sum(r[:, 0] * F[:, 2] - r[:, 2] * F[:, 0])
alpha_y = tau_y / Izz

print("=== SRB dynamics from foot forces ===")
print(f"sum Fx={F_sum[0]:.1f} N  sum Fz={F_sum[2]:.1f} N  (mg={m*g:.1f} N)")
print(f"CoM accel ax={a[0]:.2f} m/s²  az={a[2]:.2f} m/s²")
print(f"yaw torque={tau_y:.1f} N·m → alpha_y={alpha_y:.2f} rad/s²")

# 0.2 s 一定力で CoM 速度を更新（Euler 積分）
dt = 0.02
v = np.zeros(3)
p = np.zeros(3)
traj_p, traj_v = [p.copy()], [v.copy()]
for _ in range(10):
    v = v + a * dt
    p = p + v * dt
    traj_p.append(p.copy())
    traj_v.append(v.copy())

traj_p = np.array(traj_p)
traj_v = np.array(traj_v)
t = np.arange(len(traj_p)) * dt

fig, axes = plt.subplots(1, 2, figsize=(10, 3.5))
axes[0].plot(t, traj_p[:, 0], "o-", label="CoM x [m]")
axes[0].set_xlabel("time [s]")
axes[0].set_ylabel("position")
axes[0].grid(True, alpha=0.3)
axes[0].legend()
axes[1].plot(t, traj_v[:, 0], "o-", label="vx [m/s]")
axes[1].set_xlabel("time [s]")
axes[1].set_ylabel("velocity")
axes[1].grid(True, alpha=0.3)
axes[1].legend()
fig.suptitle("Demo A: constant GRF → SRB CoM motion (0.2 s)", y=1.02)
plt.tight_layout()
plt.show()
"""),
        md("""## Step 3 — 足の接触反力（GRF）の取り扱い — 理論式

### 接触スケジュール（ゲイト）

足 $i$ の stance / swing をバイナリ $s_i(k) \\in \\{0,1\\}$ で表す（Layer 1 が生成）:

$$
s_i(k) = 1 \\Rightarrow \\text{stance（地面反力あり）}, \\quad
s_i(k) = 0 \\Rightarrow \\text{swing（}$\\mathbf{F}_i = \\mathbf{0}$\\text{）}
$$

### 非負性（片方向接触）

$$
F_{iz} \\ge 0 \\quad \\text{（地面は引っ張れない）}
$$

### 摩擦円錐（Coulomb 近似）

$$
\\sqrt{F_{ix}^2 + F_{iy}^2} \\le \\mu F_{iz}
$$

MPC パラメータ `mpc_params.mu` がこの $\\mu$ に相当（**sim 地面摩擦とは別**）。

### 垂直 GRF 上下限

$$
F_{iz}^{\\min} \\le F_{iz} \\le F_{iz}^{\\max}
$$

`grf_max` 等でソフト制限。跳ねすぎ・関節飽和を防ぐ。

### Stance 足のみに力を割り当て

$$
\\sum_{i: s_i=1} F_{ix} \\approx m a_x^{\\mathrm{des}}, \\quad
\\sum_{i: s_i=1} F_{iz} \\approx mg
$$

（前後・左右の力配分は MPC が $Q, R$ と制約の下で最適化）

### Convex 化の要点

**どの足が stance かをゲイトで固定** → GRF $\\mathbf{u}$ について **凸**（QP / NLP）に近づける。  
足の ON/OFF を MPC 内で離散最適化すると NP 困難になりやすい。
"""),
        code("""\
# --- 数値デモ B: 摩擦円錐・垂直力制約の判定 ---
mu = 0.5
fz_min, fz_max = 20.0, 150.0

candidates = {
    "OK: moderate push": np.array([30.0, 0.0, 100.0]),
    "OK: vertical only": np.array([0.0, 0.0, 100.0]),
    "FAIL: |Fx|>mu*Fz": np.array([60.0, 0.0, 100.0]),
    "FAIL: Fz too small": np.array([10.0, 0.0, 15.0]),
    "FAIL: negative Fz": np.array([0.0, 0.0, -5.0]),
}


def check_contact(F, mu, fz_min, fz_max):
    fx, fy, fz = F
    friction_ok = np.hypot(fx, fy) <= mu * fz + 1e-9
    fz_ok = fz_min <= fz <= fz_max
    unilateral_ok = fz >= 0
    return friction_ok and fz_ok and unilateral_ok

rows = []
for name, F in candidates.items():
    fx, fy, fz = F
    rows.append({
        "case": name,
        "Fx": fx, "Fz": fz,
        "|F_t|": np.hypot(fx, fy),
        "mu*Fz": mu * fz,
        "feasible": check_contact(F, mu, fz_min, fz_max),
    })
print(pd.DataFrame(rows).to_string(index=False))

# 4 足の力配分: 前進 60 N を stance 2 脚で分担
F_stance = np.array([
    [30, 0, 100],
    [30, 0, 100],
    [0, 0, 0],
    [0, 0, 0],
], dtype=float)
print("\\nStance FL+FR each Fx=30, Fz=100 → all feasible:",
      all(check_contact(F_stance[i], mu, fz_min, fz_max) for i in range(2)))

fig, ax = plt.subplots(figsize=(5, 5))
plot_friction_cone(mu=mu, f_max=fz_max, ax=ax)
for name, F in candidates.items():
    c = "green" if check_contact(F, mu, fz_min, fz_max) else "red"
    ax.scatter(F[2], F[0], s=80, c=c, label=name)
ax.legend(fontsize=7, loc="upper left")
ax.set_title("Demo B: GRF candidates in friction cone (Fx-Fz)")
plt.tight_layout()
plt.show()
"""),
        md("""## Step 4 — 3 層アーキテクチャと数式の紐付け

| Layer | モジュール（PyMPC） | 数式上の役割 | 入出力 |
|-------|---------------------|--------------|--------|
| **1 ゲイト** | `periodic_gait_generator` | $s_i(k)$, $\\mathbf{v}^{ref}(k)$ を生成 | 指令速度 → 接触表 |
| **2 MPC** | `SRBDControllerInterface` | $\\min \\sum \\|x-x^{ref}\\|_Q + \\|u\\|_R$ s.t. SRB + 摩擦円錐 | $\\mathbf{x}_k \\to \\mathbf{u}_k^*$ (GRF) |
| **3 WBC** | `WBInterface` + Swing | Stance: $\\boldsymbol{\\tau}=\\mathbf{J}^\\top\\mathbf{F}^*$ / Swing: PD | GRF → $\\boldsymbol{\\tau}$ |

### 1 制御周期の信号流（数式）

1. **Layer 1:** $\\{s_i(k)\\}$, $\\mathbf{v}^{ref}(k)$ を更新  
2. **Layer 2:** $\\mathbf{u}_k^* = \\arg\\min \\|\\cdot\\|$ subject to $f_{\\mathrm{SRB}}$, $\\mu$, $F_z$ bounds, $s_i(k)$  
3. **Layer 3:** 各足 $i$ について  
   - $s_i=1$: $\\boldsymbol{\\tau}_i = \\mathbf{J}_i^\\top \\mathbf{F}_i^* + \\text{PD}$  
   - $s_i=0$: $\\boldsymbol{\\tau}_i = \\text{SwingPD}(\\mathbf{x}_{foot,i}^{ref})$

### MPC 設計者が触るパラメータ → 数式への効き

| パラメータ | 数式上の効き |
|------------|--------------|
| `mu` | 摩擦円錐の傾き $\\mu$ |
| `grf_max` | $F_{iz}^{\\max}$ |
| `Q, R` | 状態追従 vs 入力コスト |
| `step_freq`, `duty` | $s_i(k)$ パターン（Layer 1） |
| `ref_z` | $\\mathbf{x}^{ref}$ の高さ成分 |
"""),
        code("""\
# --- 数値デモ C: 3 層を 1 周期でつなぐ（教学用 toy model）---
m, g = 15.0, 9.81
mu = 0.5
v_ref = 0.5  # 目標 vx [m/s]
ax_des = 0.3  # 目標加速度 [m/s²]（定常加速のイメージ）

# Layer 1: trot duty=0.5 → FL,RR stance / FR,RL swing（対角1相のみ）
stance = {"FL": 1, "FR": 0, "RL": 0, "RR": 1}
print("Layer 1 gait mask:", stance)

# Layer 2: 必要水平力 ≈ m*ax_des, 垂直 ≈ mg を stance 脚で分担
Fx_total = m * ax_des
Fz_total = m * g
n_stance = sum(stance.values())
Fx_each = Fx_total / n_stance
Fz_each = Fz_total / n_stance
print(f"Layer 2 required per stance foot: Fx={Fx_each:.1f} N, Fz={Fz_each:.1f} N")
print("  friction check:", abs(Fx_each) <= mu * Fz_each)

# Layer 3: 2D 脚 Jacobian（教学用） J = [r_z; -r_x] → tau = J^T [Fx, Fz]
# 前左足 FL: r = (0.25, -0.15) [x,z]
r_x, r_z = 0.25, -0.15
J = np.array([[0.0, r_z], [1.0, -r_x]])  # [Fx,Fz] → [tau_hip, tau_knee] の toy
F_fl = np.array([Fx_each, Fz_each])
tau_fl = J.T @ F_fl
print(f"Layer 3 FL torque (toy 2-DOF): {tau_fl.round(1)} N·m")

# 信号流の概念図
fig, ax = plt.subplots(figsize=(9, 3))
layers = ["L1 Gait\\nstance mask", "L2 MPC\\nGRF F*", "L3 WBC\\ntau", "MuJoCo\\nintegrate"]
xpos = [0, 1, 2, 3]
ax.bar(xpos, [1, 1, 1, 1], color=["#93c5fd", "#86efac", "#fde68a", "#fca5a5"])
for x, lab in zip(xpos, layers):
    ax.text(x, 0.5, lab, ha="center", va="center", fontsize=10)
ax.set_xticks(xpos)
ax.set_xticklabels(["s_i(k)", "u=F_i", "tau", "x"])
ax.set_yticks([])
ax.set_title("Demo C: 3-layer signal flow (equations linked)")
plt.tight_layout()
plt.show()
"""),
        md("""## Step 5 — GRF（Ground Reaction Force）まとめ

- 足先と地面の間の力 $\\mathbf{F}_i = (F_{ix}, F_{iy}, F_{iz})$
- 4足 → **12次元** の入力（低次元で物理制約を入れやすい）
- ロボットの加速は $\\sum_i \\mathbf{F}_i + m\\mathbf{g}$ で決まる（Step 2）

**なぜ関節角度を直接 MPC しないのか？**
- 次元が高い（12 関節以上）
- 摩擦円錐など **接触力の物理** を入れにくい
- GRF を決めれば CoM 運動を計画できる → 関節は WBC に任せる
"""),
        md("""## Step 6 — MPC の定式化（口頭説明用）

離散時間ホライゾン $N$、サンプリング $\\Delta t$:

$$\\min \\sum_{k=0}^{N-1} \\|x_k - x_k^{ref}\\|_Q + \\|u_k\\|_R
\\quad \\text{s.t.} \\quad x_{k+1} = f_{SRB}(x_k, u_k)$$

**摩擦円錐**（各足 $i$、$s_i(k)=1$ のとき）:

$$\\sqrt{F_{ix}^2 + F_{iy}^2} \\le \\mu F_{iz}, \\quad F_{iz}^{min} \\le F_{iz} \\le F_{iz}^{max}$$

**Convex 化のコツ:** どの足が stance/swing かを **ゲイトで固定** → GRF について凸に近づける。
"""),
        md("""## Step 7 — WBC 相当層

| 脚 | 処理 |
|----|------|
| **Stance** | MPCの $\\mathbf{F}_i^*$ → $\\boldsymbol{\\tau} = \\mathbf{J}^\\top \\mathbf{F} + \\text{PD}$ |
| **Swing** | Bezier足軌道 + PD（MPCのGRFは使わない） |

MPC 設計者が触るのは主に **Layer 2（MPC）**。Layer 3 ゲインは「追従の硬さ」= 計画 GRF を関節で実現できるか。
"""),
        md("## Step 8 — 摩擦円錐を可視化（μ を上げ下げすると？）"),
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
        md("## Step 9 — SRB の合力デモ（F=ma の直觉）"),
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
        md("## Step 10 — MPC設計者向け 調整マトリクス（成功/失敗パターン）"),
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
        md("""## Step 11 — チェックリスト

- [ ] SRB の並進・回転方程式を書ける（Step 2）  
- [ ] 摩擦円錐・$F_z$ 上下限・stance/swing の意味を説明できる（Step 3）  
- [ ] 3 層それぞれが **どの変数**（$s_i$, $\\mathbf{F}_i$, $\\boldsymbol{\\tau}$）を扱うか説明できる（Step 4）  
- [ ] 数値デモ A–C を実行し、GRF → 加速度 → トルクの流れを説明できる  
- [ ] `mu` を上げると **何が起きやすいか**（加速 vs 転倒）を説明できる  
- [ ] 転倒時の最初の 3 つのアクションを言える  

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

**関連:** [WORKSHOP.md](../WORKSHOP.md) §7 Session 4 · [SPEED_TERRAIN_TRIAL_LOG.md](../SPEED_TERRAIN_TRIAL_LOG.md) · [MPC_TUNING_JOURNEY.md](../MPC_TUNING_JOURNEY.md) · [06 統合 Notebook](./06_mpc_tuning_journey.ipynb)
"""),
    ]
    return nb(cells, "05")


def tuning_journey_notebook() -> dict:
    cells = [
        md("""# 06 — MPC 設計者ジャーニー: 失敗・成功・体験（統合）

**目的:** Phase 1–4 の試行錯誤を **1 本の Notebook** で体験する。  
読み物: [MPC_TUNING_JOURNEY.md](../MPC_TUNING_JOURNEY.md) · 早見表: [TUNING_GUIDE.md](../TUNING_GUIDE.md)

> 各 Step は `scripts/tuning_labs.py` の **lab ID** と 1:1 対応。CLI でも同じ実験が再現できます。
"""),
        code(SETUP),
        md("""## Step 0 — Lab カタログ

`tuning_labs.py` に登録された fail / success ペア一覧。
"""),
        code("""\
for lab in list_labs():
    print(f"{lab.id:28} [{lab.phase:7}] session={lab.session}  {lab.title}")
print(f"\\n{len(TUNING_LABS)} labs total")
"""),
        md("""## Phase 1 — ref_z（MPC 以前の物理パラメータ）

[Phase 1 — 平坦スモーク](../MPC_TUNING_JOURNEY.md#phase-1-flat-smoke)
"""),
        code("""\
from pympc_lab import compare_runs
pair = run_lab_pair("s1_ref_z_fail", "s1_ref_z_ok")
fig = compare_runs(pair)
plt.suptitle("Phase 1: ref_z fail vs ok", y=1.02)
plt.show()
for label, m in pair:
    print(label, "terminated=", m["terminated"], "min_z=", round(m["min_z"], 3))
"""),
        md("""## Phase 2 — μ と step_freq（平坦）

[Phase 2 — 平坦チューニング](../MPC_TUNING_JOURNEY.md#phase-2-flat-tune)
"""),
        code("""\
pair_mu = run_lab_pair("s2_mu_aggressive", "s2_mu_conservative")
fig = compare_runs(pair_mu)
plt.suptitle("Phase 2a: mu aggressive vs conservative", y=1.02)
plt.show()

fail_freq = run_lab("s2_step_freq_fast")
ok_freq = run_lab("s2_mu_conservative")  # baseline gait for contrast
fig = compare_runs([
    ("step_freq=1.6 FAIL", fail_freq["metrics"]),
    ("mu=0.35 stable gait", ok_freq["metrics"]),
])
plt.suptitle("Phase 2b: step_freq too fast", y=1.02)
plt.show()
"""),
        code("""\
# 事前計測 param study（scripts/run_parameter_study.py）
fig = plot_param_study_mu()
plt.show()
"""),
        md("""## Phase 3 — 不整地（boxes / perlin）

[Phase 3 — 不整地](../MPC_TUNING_JOURNEY.md#phase-3-rough-terrain)

| デモ | GIF |
|------|-----|
| boxes | ![boxes](../assets/demo_s03_boxes.gif) |
| perlin | ![perlin](../assets/demo_s03_perlin.gif) |
"""),
        code("""\
pair_boxes = run_lab_pair("s3_boxes_freq_fail", "s3_boxes_freq_ok")
fig = compare_runs(pair_boxes)
plt.suptitle("Phase 3a: boxes step_freq fail vs ok", y=1.02)
plt.show()

fail_perlin = run_lab("s3_perlin_mu_fail")
print("perlin mu=0.55:", fail_perlin["result"]["terminated"], fail_perlin["result"]["distance_m"])
"""),
        md("""## Phase 4 — 5 kph × 凸凹坂

[Phase 4 — 5 kph × 凸凹坂](../MPC_TUNING_JOURNEY.md#phase-4-speed-bumpy)

| 地形 | GIF | meta |
|------|-----|------|
| flat | ![](../assets/demo_s04_flat.gif) | demo_s04_flat.meta.json |
| uphill | ![](../assets/demo_s04_uphill.gif) | demo_s04_uphill.meta.json |
| downhill | ![](../assets/demo_s04_downhill.gif) | demo_s04_downhill.meta.json |
"""),
        code("""\
import json
# 全試行ログの可視化
fig = plot_speed_trial_journey()
plt.show()

fail_s4 = run_lab("s4_no_fall_fail")
print("no-fall 5kph:", fail_s4["result"])
"""),
        code("""\
# ✅ resilient 勝ちパラメータ（時間がかかる — 1 地形だけ実行例）
# 全 3 地形は Notebook 05 または tuning_labs.py --lab s4_resilient_* で

win = run_lab("s4_resilient_flat_win")
print(win["result"]["distance_m"], "m", "falls=", win["result"].get("falls"))
fig, ax = plt.subplots(figsize=(9, 3))
m = win["metrics"]
ax.plot(m["x"], m["vx"] * 3.6, lw=1.5)
ax.axhline(5.0, ls="--", color="k", alpha=0.4)
ax.set_xlabel("cumulative distance [m]")
ax.set_ylabel("vx [kph]")
ax.set_title("Phase 4 success: bumpy_flat resilient 20m")
ax.grid(True, alpha=0.3)
plt.show()
"""),
        md("""## Step 7 — あなたの手で 1 パラメータ変更

`run_lab("s4_resilient_flat_win")` の kwargs を **1 つだけ** 変えて再実行し、距離・転倒を記録。

例: `duty_factor` を 0.70 に下げると falls が増えるか？
"""),
        code("""\
# --- 演習: 下の kwargs を 1 つだけ編集 ---
from pympc_lab import apply_preset, run_speed_terrain_sim_resilient

apply_preset("session04_speed_bumpy_base")
custom = run_speed_terrain_sim_resilient(
    scene="bumpy_flat",
    target_speed_kph=5.0,
    min_distance_m=20.0,
    max_seconds=90.0,
    max_falls=22,
    mu=0.42,
    step_freq=1.20,
    duty_factor=0.76,   # ← ここを 0.70 などに変更して試す
    ref_z_scale=1.07,
    speed_ramp_s=18.0,
)
print("distance_m", custom["distance_m"], "falls", custom["falls"], "success", custom["success"])
"""),
        md("""## Step 8 — 修了チェック

- [ ] Phase 1–4 各 1 つの **fail lab** と **success lab** を実行した
- [ ] [MPC_TUNING_JOURNEY.md](../MPC_TUNING_JOURNEY.md) のトリアージフローを説明できる
- [ ] `python scripts/verify_workshop_assets.py` でデモ meta を確認した
- [ ] 勝ちパラメータが `configs/pympc_presets/session04_bumpy_*.yaml` に保存されていることを確認した

**CLI 再現:**
```bash
python scripts/tuning_labs.py --list
python scripts/tuning_labs.py --lab s2_mu_aggressive
python scripts/verify_workshop_assets.py
```
"""),
    ]
    return nb(cells, "06")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    notebooks = [
        ("00_theory_grf_mpc_wbc.ipynb", theory_notebook()),
        ("01_demo_session01_flat_smoke.ipynb", demo_s1_notebook()),
        ("02_demo_session02_flat_tune.ipynb", demo_s2_notebook()),
        ("03_demo_session03a_rough_boxes.ipynb", demo_s3a_notebook()),
        ("04_demo_session03b_rough_perlin.ipynb", demo_s3b_notebook()),
        ("05_demo_session04_speed_bumpy.ipynb", demo_s4_notebook()),
        ("06_mpc_tuning_journey.ipynb", tuning_journey_notebook()),
    ]
    for name, content in notebooks:
        path = OUT / name
        path.write_text(json.dumps(content, ensure_ascii=False, indent=1), encoding="utf-8")
        print("wrote", path)


if __name__ == "__main__":
    main()
