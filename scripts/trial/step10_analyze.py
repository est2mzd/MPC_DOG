#!/usr/bin/env python3
"""Step 10 analysis: predicted future touchdown events (step10_gait_events.csv)
vs actual contact transitions (state_log.csv).

Each plan cycle emits one event_ordinal==0 row PER LEG (that leg's next
touchdown, at pred_touchdown_horizon_index). The "next leg to touch down" in a
cycle is therefore the leg with the SMALLEST horizon index; the full per-cycle
order is the 4 legs sorted by horizon index.

- leg-order check: cyclic order of legs by ascending predicted horizon index
  (steady mid-run) vs cyclic order of actual F->True contact edges during
  steady walking. Match allows a rotation.
- timing: predicted vs actual inter-touchdown interval (clock-origin-free).
- plot: predicted order (arrows) + actual touchdown ticks per leg.

Usage: step10_analyze.py artifacts/step10
"""
import csv
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

LEGS = ["FL", "BL", "FR", "BR"]
COL = {"FL": "#1565c0", "BL": "#2e7d32", "FR": "#c62828", "BR": "#f9a825"}
root = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step10"


def rows(p):
    return list(csv.DictReader(open(p)))


def predicted_cycle_order(gait_csv):
    """Per plan cycle, the 4 legs sorted by predicted horizon index (ascending).
    Returns (modal_order tuple, list of per-cycle orders during mid-run)."""
    by_cyc = {}
    for r in rows(gait_csv):
        if r["event_ordinal"] != "0":
            continue
        c = int(r["current_plan_index"])
        by_cyc.setdefault(c, {})[r["leg"]] = int(r["pred_touchdown_horizon_index"])
    orders = []
    for c in sorted(by_cyc):
        d = by_cyc[c]
        if len(d) == 4:
            orders.append(tuple(sorted(LEGS, key=lambda lg: d[lg])))
    if not orders:
        return None, []
    mid = orders[len(orders) // 4: 3 * len(orders) // 4] or orders
    modal = Counter(mid).most_common(1)[0][0]
    return modal, orders


def predicted_touchdown_stream(gait_csv):
    """Sequence of (leg, pred_touchdown_time) for the leg with the smallest
    horizon index each cycle, consecutive duplicates collapsed (keep latest t)."""
    by_cyc = {}
    for r in rows(gait_csv):
        if r["event_ordinal"] != "0":
            continue
        c = int(r["current_plan_index"])
        by_cyc.setdefault(c, {})[r["leg"]] = (
            int(r["pred_touchdown_horizon_index"]),
            float(r["pred_touchdown_time"]),
        )
    seq = []
    for c in sorted(by_cyc):
        d = by_cyc[c]
        if len(d) != 4:
            continue
        leg = min(LEGS, key=lambda lg: d[lg][0])
        t = d[leg][1]
        if seq and seq[-1][0] == leg:
            seq[-1][1] = t
        else:
            seq.append([leg, t])
    return seq


def actual_touchdowns(state_csv):
    """[(sim_time, leg)] for each False->True contact edge while the robot is
    actually walking (base x increasing), i.e. skipping the STAND settle."""
    rr = rows(state_csv)
    # walking starts when x first exceeds 0.05 m and keeps rising
    xs = [(float(r["sim_time_s"]), float(r["base_pos_x_m"])) for r in rr]
    t_walk = next((t for t, x in xs if x > 0.05), xs[0][0])
    prev = {lg: None for lg in LEGS}
    ev = []
    for r in rr:
        t = float(r["sim_time_s"])
        for lg in LEGS:
            c = r["contact_" + lg].strip().lower() in ("true", "1")
            if prev[lg] is False and c and t >= t_walk - 0.5:
                ev.append((t, lg))
            prev[lg] = c
    return ev


def canon(order):
    """Rotate a cyclic leg order so it starts at FL, for stable display."""
    o = list(order)
    if "FL" in o:
        k = o.index("FL")
        o = o[k:] + o[:k]
    return tuple(o)


def cyclic_equal(a, b):
    if sorted(a) != sorted(b):
        return False
    aa = list(a)
    return any(aa[i:] + aa[:i] == list(b) for i in range(len(aa)))


print(f"{'terrain':>8} | {'予測 脚順(1周期)':>18} | {'実 脚順':>14} | 一致 | 予測間隔 | 実間隔 | 誤差")
print("-" * 100)

for d in sorted(p for p in os.listdir(root) if os.path.isdir(os.path.join(root, p))):
    gp = os.path.join(root, d, "step10_gait_events.csv")
    sp = os.path.join(root, d, "state_log.csv")
    if not (os.path.exists(gp) and os.path.exists(sp)):
        print(f"{d:>8} | (missing csv)")
        continue

    modal, _ = predicted_cycle_order(gp)
    ev = actual_touchdowns(sp)
    # actual repeating order: first full cycle of 4 distinct legs
    act_order = []
    for _, lg in ev[3:]:
        if lg not in act_order:
            act_order.append(lg)
        if len(act_order) == 4:
            break
    act_order = tuple(act_order)

    match = cyclic_equal(modal, act_order) if (modal and len(act_order) == 4) else False
    modal_c = canon(modal) if modal else None
    act_c = canon(act_order) if len(act_order) == 4 else act_order

    # timing: predicted next-touchdown stream intervals vs actual edge intervals
    pstream = predicted_touchdown_stream(gp)
    pt = [t for _, t in pstream][3:]
    at = [t for t, _ in ev][3:]
    pdi = [pt[i + 1] - pt[i] for i in range(len(pt) - 1) if 0 < pt[i + 1] - pt[i] < 1.0]
    adi = [at[i + 1] - at[i] for i in range(len(at) - 1) if 0 < at[i + 1] - at[i] < 1.0]
    p_mean = sum(pdi) / len(pdi) if pdi else float("nan")
    a_mean = sum(adi) / len(adi) if adi else float("nan")
    err_ms = abs(p_mean - a_mean) * 1000

    pm = "→".join(modal_c) if modal_c else "?"
    am = "→".join(act_c) if len(act_c) == 4 else "?"
    print(f"{d:>8} | {pm:>18} | {am:>14} | {'YES' if match else 'no ':>4} | "
          f"{p_mean*1000:6.0f}ms | {a_mean*1000:6.0f}ms | {err_ms:5.0f}ms")

    # ---- plot ----
    fig, ax = plt.subplots(figsize=(8.6, 2.8))
    y = {lg: i for i, lg in enumerate(LEGS)}
    t0 = at[0] if at else 0
    span = 6.0
    for t, lg in ev:
        if t0 <= t <= t0 + span:
            ax.plot([t - t0, t - t0], [y[lg] - 0.32, y[lg] + 0.32], "-",
                    color=COL[lg], lw=3)
    # predicted order as a repeating arrow strip along the bottom
    if modal:
        ax.text(0.02, -1.15, "予測 1 周期の脚順:  " + "  →  ".join(modal_c),
                transform=ax.get_yaxis_transform(), fontsize=9)
    ax.set_yticks(range(4))
    ax.set_yticklabels(LEGS)
    ax.set_ylim(-1.4, 3.6)
    ax.set_xlim(-0.1, span)
    ax.set_xlabel("時刻 [s](最初の実 touchdown を原点)")
    ax.set_title(f"Step 10: {d} — 実接触の立ち上がり(色=脚)  "
                 f"/ 脚順 {'一致' if match else '要確認'}、間隔誤差 {err_ms:.0f} ms")
    fig.tight_layout()
    fig.savefig(os.path.join(root, d, f"step10_{d}_pred_vs_actual.png"), dpi=120)
    plt.close(fig)

print("\nplots -> artifacts/step10/*/step10_*_pred_vs_actual.png")
