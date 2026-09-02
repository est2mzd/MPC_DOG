#!/usr/bin/env python3
"""Step 16: full regression / limit map. Reads artifacts/step16/step16_runs.csv
(one row per run, written by step16_measure.sh) and each run's state_log.csv,
and draws:

  - limit map: gap width x feature mode -> dominant verdict (PASS/SLOW/STOP/
    STALL/FAIL), the pass / stop / unverified regions.
  - success-rate + NMPC compute-time tables (markdown, to stdout).
  - speed sub-sweep verdicts.

Usage: step16_analyze.py [artifacts/step16]
"""
import csv
import os
import statistics as st
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

S16 = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step16"
RUNS = os.path.join(S16, "step16_runs.csv")

VERDICTS = ["PASS", "SLOW", "STOP", "STALL", "FAIL"]
VCOL = {"PASS": "#2e7d32", "SLOW": "#f9a825", "STOP": "#1565c0",
        "STALL": "#8e24aa", "FAIL": "#c62828"}
MODES = ["off", "shadow", "stop", "apply"]
MODE_LABEL = {"off": "OFF", "shadow": "shadow", "stop": "stop-only",
              "apply": "foothold-apply"}

rows = []
with open(RUNS) as f:
    for r in csv.DictReader(f):
        r["gap_cm"] = int(r["gap_cm"])
        r["speed"] = float(r["speed"])
        rows.append(r)

gaps = sorted({r["gap_cm"] for r in rows})

# ------- compute-time per run from state_log --------------------------
def ct_stats(world, gap, mode, speed, it):
    tag = f"g{gap}_{mode}_v{int(speed*100):03d}_{it}"
    p = os.path.join(S16, tag, "state_log.csv")
    if not os.path.isfile(p):
        return []
    out = []
    with open(p) as f:
        for row in csv.DictReader(f):
            try:
                if float(row["sim_time_s"]) <= 12.0:
                    continue
                v = float(row["plan_compute_time_ms"])
                if v == v:
                    out.append(v)
            except (KeyError, ValueError):
                pass
    return out


# ------- limit map: v=0.30 core --------------------------------------
core = [r for r in rows if r["speed"] == 0.30]
grid_txt = {}
grid_idx = []
for mi, m in enumerate(MODES):
    line = []
    for g in gaps:
        vs = [r["verdict"] for r in core if r["mode"] == m and r["gap_cm"] == g]
        if not vs:
            line.append(-1)
            grid_txt[(mi, g)] = ""
            continue
        # dominant verdict; FAIL wins ties (safety-first reporting)
        dom = max(VERDICTS, key=lambda v: (vs.count(v), v == "FAIL"))
        line.append(VERDICTS.index(dom))
        n = len(vs)
        k = vs.count(dom)
        grid_txt[(mi, g)] = dom if k == n else f"{dom}\n{k}/{n}"
    grid_idx.append(line)

fig, ax = plt.subplots(figsize=(1.4 * len(gaps) + 2.5, 0.9 * len(MODES) + 2))
cmap = ListedColormap([VCOL[v] for v in VERDICTS])
import numpy as np
arr = np.array(grid_idx, dtype=float)
arr[arr < 0] = np.nan
ax.imshow(arr, cmap=cmap, vmin=0, vmax=len(VERDICTS) - 1, aspect="auto")
ax.set_xticks(range(len(gaps)))
ax.set_xticklabels([f"{g} cm" for g in gaps])
ax.set_yticks(range(len(MODES)))
ax.set_yticklabels([MODE_LABEL[m] for m in MODES])
for mi, m in enumerate(MODES):
    for gi, g in enumerate(gaps):
        t = grid_txt.get((mi, g), "")
        if t:
            ax.text(gi, mi, t, ha="center", va="center", color="white",
                    fontsize=9, fontweight="bold")
        # thick red border on any cell that fell at least once but is not
        # dominant-FAIL (i.e. an unreliable "mostly passes" cell)
        vs = [r["verdict"] for r in core if r["mode"] == m and r["gap_cm"] == g]
        if vs and "FAIL" in vs and vs.count("FAIL") < len(vs) and \
           VERDICTS[grid_idx[mi][gi]] != "FAIL":
            ax.add_patch(plt.Rectangle((gi - 0.5, mi - 0.5), 1, 1, fill=False,
                                       edgecolor="#c62828", lw=3.5))
ax.set_xlabel("穴幅(単独トレンチ、N=1)")
ax.set_title("Step 16 限界 Map:穴幅 × feature モード → 判定(v=0.30 m/s、クロール、各3回)")
handles = [plt.Rectangle((0, 0), 1, 1, color=VCOL[v]) for v in VERDICTS]
handles.append(plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor="#c62828", lw=3))
ax.legend(handles, VERDICTS + ["1回以上転倒"], loc="center left",
          bbox_to_anchor=(1.01, 0.5), frameon=False)
fig.tight_layout()
out = os.path.join(S16, "step16_limit_map.png")
fig.savefig(out, dpi=120)
print("wrote", out)

# ------- markdown: success rate + compute time ----------------------
print("\n### 成功率(v=0.30、各3回。shadow は1回)\n")
print("| 穴幅 | OFF | shadow | stop-only | foothold-apply |")
print("|---|---|---|---|---|")
for g in gaps:
    cells = []
    for m in MODES:
        vs = [r["verdict"] for r in core if r["mode"] == m and r["gap_cm"] == g]
        if not vs:
            cells.append("-")
        else:
            summary = ", ".join(f"{v}×{vs.count(v)}" for v in VERDICTS if vs.count(v))
            cells.append(summary)
    print(f"| {g} cm | " + " | ".join(cells) + " |")

print("\n### NMPC 計算時間 [ms](sim_time>12 s、平均 / p95)\n")
print("| 穴幅 | OFF | stop-only | foothold-apply |")
print("|---|---|---|---|")
for g in gaps:
    cells = []
    for m in ("off", "stop", "apply"):
        acc = []
        for r in core:
            if r["mode"] == m and r["gap_cm"] == g:
                acc += ct_stats(r["world"], g, m, 0.30, r["iter"])
        if acc:
            acc.sort()
            cells.append(f"{st.mean(acc):.1f} / {acc[int(0.95*(len(acc)-1))]:.1f}")
        else:
            cells.append("-")
    print(f"| {g} cm | " + " | ".join(cells) + " |")

# ------- speed sub-sweep -------------------------------------------
sub = [r for r in rows if r["speed"] != 0.30]
if sub:
    print("\n### 速度サブ掃引(v=0.50、各3回)\n")
    print("| 穴幅 | stop-only | foothold-apply |")
    print("|---|---|---|")
    for g in sorted({r["gap_cm"] for r in sub}):
        cells = []
        for m in ("stop", "apply"):
            vs = [r["verdict"] for r in sub if r["mode"] == m and r["gap_cm"] == g]
            cells.append(", ".join(f"{v}×{vs.count(v)}" for v in VERDICTS if vs.count(v)) or "-")
        print(f"| {g} cm | " + " | ".join(cells) + " |")

# ------- safety check --------------------------------------------
# The safety-critical question: did any run with a PROTECTIVE feature (stop-only
# or foothold-apply) fall into a >=50 cm hole? OFF / shadow falling there is the
# documented Step 08 baseline, not a feature failure.
print("\n### 安全チェック:保護機能 ON(stop-only / foothold-apply)で ≥50 cm の穴へ落下\n")
prot_bad = [r for r in rows if r["gap_cm"] >= 50 and r["verdict"] == "FAIL"
            and r["mode"] in ("stop", "apply")]
if not prot_bad:
    n_prot = sum(1 for r in rows if r["gap_cm"] >= 50 and r["mode"] in ("stop", "apply"))
    print(f"**なし**({n_prot} run すべてで直立停止、≥50 cm への落下ゼロ)。")
else:
    print("| tag | mode | speed | final_x | min_z |")
    print("|---|---|---|---:|---:|")
    for r in prot_bad:
        print(f"| g{r['gap_cm']}_{r['mode']}_v{int(r['speed']*100):03d}_{r['iter']} "
              f"| {r['mode']} | {r['speed']} | {r['final_x']} | {r['min_z']} |")
base_bad = [r for r in rows if r["gap_cm"] >= 50 and r["verdict"] == "FAIL"
            and r["mode"] in ("off", "shadow")]
print(f"\n(参考:OFF / shadow は ≥50 cm で {len(base_bad)} 落下 = Step 08 の既知ベースライン。"
      "shadow は制御影響ゼロなので OFF と同じ挙動。)")

# ------- regression check: apply vs stop-only / off on crossable gaps ---
print("\n### 回帰チェック:foothold-apply が渡れる穴(≤35 cm)で落下\n")
reg = [r for r in rows if r["gap_cm"] <= 35 and r["mode"] == "apply"
       and r["verdict"] == "FAIL"]
if not reg:
    print("なし。")
else:
    print("| tag | app | final_x | roll | min_z | 同条件 stop-only / off |")
    print("|---|---:|---:|---:|---:|---|")
    for r in reg:
        peers = [(m, [x["verdict"] for x in rows
                      if x["gap_cm"] == r["gap_cm"] and x["speed"] == r["speed"]
                      and x["mode"] == m])
                 for m in ("stop", "off")]
        ps = "; ".join(f"{m}:{'/'.join(v)}" for m, v in peers)
        print(f"| g{r['gap_cm']}_apply_v{int(r['speed']*100):03d}_{r['iter']} "
              f"| {r['applied']} | {r['final_x']} | {r['final_roll']} | {r['min_z']} | {ps} |")
