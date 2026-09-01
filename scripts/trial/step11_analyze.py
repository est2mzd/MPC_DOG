#!/usr/bin/env python3
"""Step 11 analysis: reachable + safe + observed foothold candidates per future
touchdown (step11_candidates.csv).

Completion conditions:
- 30cm gap: valid candidates remain for the crossing (front-leg) touchdowns.
- 50/100cm gap: the far-bank candidates are out of reach -> n_valid collapses
  for the touchdown that must cross; the selected foothold fails the tests
  (sel_passes_all == 0).
- flat: n_valid > 0 near the nominal, always.

Plots n_valid over the run for the front legs (FL/FR) per terrain, plus the
robot's x, so the drop lines up with the gap.

Usage: step11_analyze.py artifacts/step11
"""
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

root = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step11"
COL = {"FL": "#1565c0", "BL": "#2e7d32", "FR": "#c62828", "BR": "#f9a825"}


def load(p):
    out = []
    for r in csv.DictReader(open(p)):
        for k in ("time", "hip_x", "hip_y", "n_in_reach", "n_safe", "n_valid",
                  "sel_x", "min_valid_reach_dist"):
            try:
                r[k] = float(r[k])
            except ValueError:
                r[k] = float("nan")
        r["sel_passes_all"] = int(r["sel_passes_all"])
        r["sel_in_reach"] = int(r["sel_in_reach"])
        out.append(r)
    return out


def base_x_at(state_csv):
    """time->x interpolator key list from state_log (sim clock)."""
    rr = list(csv.DictReader(open(state_csv)))
    return [(float(r["sim_time_s"]), float(r["base_pos_x_m"])) for r in rr]


print(f"{'terrain':>6} | {'front-leg n_valid (min while approaching gap)':>44} | "
      f"{'sel_passes_all near gap':>24}")
print("-" * 100)

for d in sorted(p for p in os.listdir(root) if os.path.isdir(os.path.join(root, p))):
    cp = os.path.join(root, d, "step11_candidates.csv")
    sp = os.path.join(root, d, "state_log.csv")
    if not os.path.exists(cp):
        print(f"{d:>6} | (no csv)")
        continue
    rows = load(cp)
    front = [r for r in rows if r["leg"] in ("FL", "FR")]

    # crude "near the gap" window: front-leg rows whose hip_x is in [1.4, 2.6]
    # (gaps in the test worlds start at x=2.0). flat has no gap -> use all.
    near = [r for r in front if 1.4 <= r["hip_x"] <= 2.8] or front
    nv_min = min((r["n_valid"] for r in near), default=float("nan"))
    nv_med = sorted(r["n_valid"] for r in front)[len(front) // 2] if front else float("nan")
    sel_pass_rate = (sum(r["sel_passes_all"] for r in near) / len(near)) if near else float("nan")
    print(f"{d:>6} | median(all)={nv_med:4.0f}  min(near gap)={nv_min:4.0f}"
          f"{'':16} | {sel_pass_rate*100:5.0f}% pass")

    # ---- plot n_valid vs x for FL/FR ----
    xs_t = base_x_at(sp) if os.path.exists(sp) else []

    def x_of(t):
        if not xs_t:
            return float("nan")
        # nearest by time; clocks differ in origin but both monots -> use rank
        return None

    fig, ax = plt.subplots(figsize=(8.8, 3.0))
    for lg in ("FL", "FR"):
        pts = [(r["time"], r["n_valid"]) for r in rows if r["leg"] == lg]
        pts.sort()
        if pts:
            ax.plot([p[0] - pts[0][0] for p in pts], [p[1] for p in pts], ".-",
                    ms=3, lw=0.8, color=COL[lg], label=f"{lg} n_valid")
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("経過時間 [s](計画サイクル)")
    ax.set_ylabel("到達可能・安全・観測済み\n候補セル数 n_valid")
    ttl = {"flat": "平地", "g30": "30 cm 穴", "g50": "50 cm 穴",
           "g100": "100 cm 穴"}.get(d, d)
    ax.set_title(f"Step 11: {ttl} — 前脚(FL/FR)の有効足場候補数")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(root, d, f"step11_{d}_n_valid.png"), dpi=120)
    plt.close(fig)

print("\nplots -> artifacts/step11/*/step11_*_n_valid.png")
