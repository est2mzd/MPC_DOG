#!/usr/bin/env python3
"""Step 12 analysis: multi-step foothold-sequence verdicts (step12_sequence.csv)
+ planned foothold sequences (step12_footholds.csv).

Completion conditions:
- flat / 15cm / 30cm: FEASIBLE_TO_RANGE dominates while walking.
- 50/100cm: BLOCKED_AT_STEP_K (or classified as out-of-capability).
- compute time recorded; check it fits a local-planner cycle (~33 ms @ 30 Hz).

Usage: step12_analyze.py artifacts/step12
"""
import csv
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

root = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step12"
LEGCOL = {"FL": "#1565c0", "BL": "#2e7d32", "FR": "#c62828", "BR": "#f9a825"}
VC = {"FEASIBLE_TO_RANGE": "#2e7d32", "BLOCKED_AT_STEP_K": "#c62828",
      "UNKNOWN_BEFORE_RANGE": "#f9a825"}

print(f"{'terrain':>6} | {'verdict mix (walking)':>52} | {'compute us (med/max)':>20}")
print("-" * 100)

for d in sorted(p for p in os.listdir(root) if os.path.isdir(os.path.join(root, p))):
    sp = os.path.join(root, d, "step12_sequence.csv")
    fp = os.path.join(root, d, "step12_footholds.csv")
    stp = os.path.join(root, d, "state_log.csv")
    if not os.path.exists(sp):
        print(f"{d:>6} | (no csv)")
        continue
    seq = list(csv.DictReader(open(sp)))
    for r in seq:
        r["max_feasible_progress_m"] = float(r["max_feasible_progress_m"])
        r["compute_time_us"] = float(r["compute_time_us"])
        r["blocked_step_k"] = int(r["blocked_step_k"])
    # meaningful verdicts: FEASIBLE, or BLOCKED at k>0 (a real gap ahead).
    # BLOCKED at k=0 is the STAND phase or a post-fall pose -> noise, dropped.
    walk = [r for r in seq
            if r["verdict"] == "FEASIBLE_TO_RANGE"
            or (r["verdict"] == "BLOCKED_AT_STEP_K" and r["blocked_step_k"] > 0)
            or r["verdict"] == "UNKNOWN_BEFORE_RANGE"]
    mix = Counter(r["verdict"] for r in walk)
    total = sum(mix.values()) or 1
    mixs = "  ".join(f"{k.split('_')[0]}:{v*100//total}%" for k, v in mix.most_common())
    bk = [r["blocked_step_k"] for r in walk if r["verdict"] == "BLOCKED_AT_STEP_K"]
    bk_rng = f"k={min(bk)}..{max(bk)}" if bk else "-"
    cts = sorted(r["compute_time_us"] for r in seq)
    med = cts[len(cts) // 2] if cts else 0
    mx = cts[-1] if cts else 0
    print(f"{d:>6} | {mixs+'  '+bk_rng:>52} | {med:8.0f} / {mx:8.0f}")

    # ---- plot: verdict strip over cycles + a planned foothold sequence ----
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9.0, 4.4),
                                 gridspec_kw={"height_ratios": [1, 2]})
    xs = list(range(len(walk)))
    for k, r in enumerate(walk):
        a1.plot([k, k], [0, 1], "-", color=VC.get(r["verdict"], "k"), lw=1)
    a1.set_yticks([])
    a1.set_xlabel("計画サイクル(歩行中)")
    a1.set_title(f"Step 12: {d} — 判定の推移(緑=FEASIBLE / 赤=BLOCKED)  "
                 f"compute {med:.0f} us")

    # one representative planned sequence
    if os.path.exists(fp):
        frows = list(csv.DictReader(open(fp)))
        for r in frows:
            r["x"] = float(r["x"])
            r["step_k"] = int(r["step_k"])
        bycyc = {}
        for r in frows:
            bycyc.setdefault(r["current_plan_index"], []).append(r)
        # pick the cycle whose planned footholds span the widest x range
        best_c = max(bycyc,
                     key=lambda c: max(x["x"] for x in bycyc[c]) -
                     min(x["x"] for x in bycyc[c]))
        pick = bycyc[best_c]
        for r in pick:
            a2.scatter(r["x"], r["step_k"], s=45, c=LEGCOL.get(r["leg"], "k"),
                       edgecolors="k", linewidths=0.4)
        a2.set_ylabel("計画ステップ k")
        a2.set_xlabel("足場 x [m]")
        a2.set_title(f"1 サイクル分の予定足場列(色=脚)  n={len(pick)} 歩")
        # gap band
        GAPS = {"g30": [(0.85, 1.15), (2.85, 3.15)], "g50": [(2.0, 2.55)],
                "g100": [(2.0, 3.05)], "r15": [(x, x + 0.15) for x in
                (2.15, 2.45, 2.75)]}
        for lo, hi in GAPS.get(d, []):
            a2.axvspan(lo, hi, color="0.5", alpha=0.28)
    fig.tight_layout()
    fig.savefig(os.path.join(root, d, f"step12_{d}.png"), dpi=120)
    plt.close(fig)

print("\nplots -> artifacts/step12/*/step12_*.png")
