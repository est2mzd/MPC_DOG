#!/usr/bin/env python3
"""Step 10 analysis: predicted future touchdown events (step10_gait_events.csv)
vs actual contact transitions (state_log.csv).

- leg-order check: the sequence of "next leg to touch down" predicted by the
  planner vs the sequence of actual contact rising edges. Completion condition:
  they match on flat / 30cm / repeated-15cm.
- timing error: align the two sequences at their first shared touchdown, then
  compare inter-touchdown intervals (clock-origin-independent).
- plot: predicted vs actual touchdown times per leg (after offset alignment).

Usage: step10_analyze.py artifacts/step10
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

LEGS = ["FL", "BL", "FR", "BR"]
root = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step10"


def actual_touchdowns(state_csv):
    """{leg: [sim_time of each False->True contact edge]} during the walk window."""
    rows = list(csv.DictReader(open(state_csv)))
    prev = {lg: None for lg in LEGS}
    td = {lg: [] for lg in LEGS}
    for r in rows:
        t = float(r["sim_time_s"])
        for lg in LEGS:
            c = r["contact_" + lg].strip().lower() in ("true", "1")
            if prev[lg] is False and c:
                td[lg].append(t)
            prev[lg] = c
    return td


def predicted_next_leg_sequence(gait_csv):
    """Collapsed sequence of the leg the planner predicts touches down NEXT
    (event_ordinal == 0), plus its predicted time, over the run."""
    rows = [r for r in csv.DictReader(open(gait_csv)) if r["event_ordinal"] == "0"]
    rows.sort(key=lambda r: float(r["time"]))
    seq = []
    for r in rows:
        leg = r["leg"]
        t = float(r["pred_touchdown_time"])
        if not seq or seq[-1][0] != leg:
            seq.append([leg, t])
        else:
            seq[-1][1] = t  # keep the latest prediction for this pending event
    return seq  # [[leg, pred_time], ...] in touchdown order


def actual_leg_sequence(td):
    ev = sorted(((t, lg) for lg in LEGS for t in td[lg]))
    return [(lg, t) for t, lg in ev]


print(f"{'terrain':>8} | {'legs pred==actual?':>18} | timing error (inter-touchdown interval)")
print("-" * 92)

for d in sorted(p for p in os.listdir(root) if os.path.isdir(os.path.join(root, p))):
    gp = os.path.join(root, d, "step10_gait_events.csv")
    sp = os.path.join(root, d, "state_log.csv")
    if not (os.path.exists(gp) and os.path.exists(sp)):
        print(f"{d:>8} | (missing csv)")
        continue
    td = actual_touchdowns(sp)
    pred = predicted_next_leg_sequence(gp)
    act = actual_leg_sequence(td)

    # compare leg order over the overlap length (skip a couple of startup events)
    n = min(len(pred), len(act))
    s0 = 2
    pred_legs = [x[0] for x in pred[s0:n]]
    act_legs = [x[0] for x in act[s0:n]]
    m = min(len(pred_legs), len(act_legs))
    match = pred_legs[:m] == act_legs[:m]
    first_mismatch = next((k for k in range(m) if pred_legs[k] != act_legs[k]), None)

    # timing: align on first shared touchdown, compare successive intervals
    pt = [x[1] for x in pred[s0:s0 + m]]
    at = [x[1] for x in act[s0:s0 + m]]
    pd = [pt[k + 1] - pt[k] for k in range(len(pt) - 1)]
    ad = [at[k + 1] - at[k] for k in range(len(at) - 1)]
    kk = min(len(pd), len(ad))
    err = [pd[k] - ad[k] for k in range(kk)]
    mae = sum(abs(e) for e in err) / kk if kk else float("nan")
    mx = max((abs(e) for e in err), default=float("nan"))

    verdict = "YES" if match else f"NO @#{first_mismatch}"
    print(f"{d:>8} | {verdict:>18} | n={m:2d}  MAE={mae*1000:5.1f} ms  max={mx*1000:5.1f} ms")

    # ---- plot: predicted vs actual touchdown time per leg (offset-aligned) ----
    if not pt or not at:
        continue
    off = at[0] - pt[0]
    fig, ax = plt.subplots(figsize=(8.4, 3.0))
    col = {"FL": "#1565c0", "BL": "#2e7d32", "FR": "#c62828", "BR": "#f9a825"}
    for k in range(m):
        lg = pred[s0 + k][0]
        ax.plot([at[k], pt[k] + off], [0, 1], "-", lw=0.8, color=col.get(lg, "k"),
                alpha=0.5)
        ax.plot(at[k], 0, "o", color=col.get(lg, "k"), ms=5)
        ax.plot(pt[k] + off, 1, "s", color=col.get(lg, "k"), ms=5)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["実接触 (state_log)", "予測 (gait phase)"])
    ax.set_xlabel("時刻 [s](予測は先頭で原点合わせ)")
    ax.set_title(f"Step 10 shadow: {d} — 予測 touchdown ↔ 実接触  "
                 f"(脚順 {'一致' if match else '不一致'}、MAE {mae*1000:.0f} ms)")
    handles = [plt.Line2D([0], [0], marker="o", ls="", color=col[l], label=l)
               for l in LEGS]
    ax.legend(handles=handles, ncol=4, fontsize=8, loc="upper center")
    fig.tight_layout()
    fig.savefig(os.path.join(root, d, f"step10_{d}_pred_vs_actual.png"), dpi=120)
    plt.close(fig)

print("\nplots -> artifacts/step10/*/step10_*_pred_vs_actual.png")
