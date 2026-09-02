#!/usr/bin/env python3
"""Step 15: planned foothold sequence -> nominal (opt-in). Reads the per-scenario
step15_footholds.csv (planned vs Raibert vs snapped, per nearest touchdown per
leg) and state_log.csv (NMPC load) written by step15_measure.sh, and draws:

  - planned -> snapped correction magnitude (how much the final snap moved the
    planned foothold) and planned vs Raibert offset, for the crossable-gap ON
    runs, so the planned<->actual correspondence is visible.
  - NMPC compute time / iterations / cost / plan age, ON vs feature OFF, to show
    feeding the planned foothold does not inflate the solve.

Usage: step15_analyze.py [artifacts/step15]
"""
import csv
import glob
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

S15 = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step15"
CROSS_X = 4.00

ON_CROSS = [f"{g}_on_{i}" for i in (1, 2, 3) for g in ("r15", "g30")]
ON_WIDE = ["g50_on", "g100_on"]
OFF = ["r15_off", "g30_off"]


def state_stats(tag):
    p = os.path.join(S15, tag, "state_log.csv")
    if not os.path.isfile(p):
        return None
    ct, it, co, ag, xs, zs = [], [], [], [], [], []
    with open(p) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["sim_time_s"])
            except (KeyError, ValueError):
                continue
            xs.append(float(row["base_pos_x_m"]))
            zs.append(float(row["base_pos_z_m"]))
            if t <= 12.0:
                continue
            for key, acc in (("plan_compute_time_ms", ct),
                             ("plan_nmpc_iterations", it),
                             ("plan_nmpc_cost", co), ("plan_age_s", ag)):
                try:
                    v = float(row[key])
                    if math.isfinite(v):
                        acc.append(v)
                except (KeyError, ValueError):
                    pass
    if not xs:
        return None

    def ms(a):
        return (st.mean(a), (sorted(a)[int(0.95 * (len(a) - 1))] if a else 0.0)) if a else (0.0, 0.0)

    fx = xs[-1]
    minz = min(zs[i] for i in range(len(zs))) if zs else 0.0
    verdict = "FELL" if (zs and zs[-1] < 0.15) else ("CROSSED" if fx > CROSS_X else "SAFE-STOP")
    return dict(tag=tag, fx=fx, minz=minz, verdict=verdict,
               ct=ms(ct), it=ms(it), co=ms(co), ag=ms(ag), n=len(ct))


def foot_stats(tag):
    p = os.path.join(S15, tag, "step15_footholds.csv")
    if not os.path.isfile(p):
        return None
    applied, snap_d, pr_off = 0, [], []
    rows = 0
    with open(p) as f:
        for row in csv.DictReader(f):
            rows += 1
            try:
                ap = int(row["applied"])
                sd = float(row["snap_distance"])
                dx = float(row["planned_x"]) - float(row["raibert_x"])
                dy = float(row["planned_y"]) - float(row["raibert_y"])
            except (KeyError, ValueError):
                continue
            if ap:
                applied += 1
                if math.isfinite(sd):
                    snap_d.append(sd)
                pr_off.append(math.hypot(dx, dy))
    return dict(tag=tag, rows=rows, applied=applied, snap_d=snap_d, pr_off=pr_off)


on_state = [state_stats(t) for t in ON_CROSS]
on_state = [r for r in on_state if r]
off_state = [state_stats(t) for t in OFF]
off_state = [r for r in off_state if r]
wide_state = [state_stats(t) for t in ON_WIDE]
wide_state = [r for r in wide_state if r]
foot = [foot_stats(t) for t in ON_CROSS + ON_WIDE]
foot = [r for r in foot if r]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 5.0))

# --- panel 1: planned->snapped correction + planned-vs-Raibert offset -------
all_snap = [v for r in foot for v in r["snap_d"] if r["tag"] in ON_CROSS]
all_off = [v for r in foot for v in r["pr_off"] if r["tag"] in ON_CROSS]
if all_snap or all_off:
    bins = [i * 0.01 for i in range(0, 26)]
    ax1.hist([all_off, all_snap], bins=bins,
             label=[f"計画足場 − Raibert (n={len(all_off)})",
                    f"後段スナップの移動量 (n={len(all_snap)})"],
             color=["#1565c0", "#ef6c00"])
    n_snap0 = sum(1 for v in all_snap if v < 0.01)
    ymax = 130
    ax1.set_ylim(0, ymax)
    ax1.annotate(f"スナップ移動 ≈0 が {n_snap0} 件\n(計画足場をそのまま採用)",
                 xy=(0.005, ymax), xytext=(0.06, ymax * 0.86),
                 arrowprops=dict(arrowstyle="->", color="#ef6c00"),
                 color="#ef6c00", fontsize=9)
    ax1.set_xlabel("距離 [m]")
    ax1.set_ylabel("着地イベント数")
    ax1.legend(loc="upper right")
ax1.set_title("計画足場の差し込み量 と 後段スナップの微修正量\n(15/30 cm ON、applied 着地のみ)")
ax1.grid(axis="y", alpha=0.3)

# --- panel 2: NMPC compute time ON vs OFF ---------------------------------
groups = [("15/30 cm\nON", on_state), ("15/30 cm\nOFF", off_state),
          ("50/100 cm\nON", wide_state)]
xs = range(len(groups))
mean_ct = [st.mean([r["ct"][0] for r in g[1]]) if g[1] else 0.0 for g in groups]
p95_ct = [max([r["ct"][1] for r in g[1]]) if g[1] else 0.0 for g in groups]
ax2.bar([x - 0.18 for x in xs], mean_ct, width=0.36, label="平均", color="#2e7d32")
ax2.bar([x + 0.18 for x in xs], p95_ct, width=0.36, label="p95", color="#a5d6a7")
ax2.set_xticks(list(xs))
ax2.set_xticklabels([g[0] for g in groups])
ax2.set_ylabel("NMPC 計算時間 [ms]  (sim_time>12 s)")
ax2.set_title("NMPC 計算時間:計画足場 ON でも増えない")
ax2.legend()
ax2.grid(axis="y", alpha=0.3)

fig.suptitle("Step 15:計画足場列 → ノミナル(opt-in)", fontsize=13)
fig.tight_layout(rect=(0, 0.02, 1, 0.95))
out = os.path.join(S15, "step15_foothold_apply.png")
fig.savefig(out, dpi=110)
print("wrote", out)

# --- markdown tables ---------------------------------------------------
print("\n### planned foothold apply (ON, applied touchdowns)\n")
print("| シナリオ | s15 行 | applied | 計画−Raibert 中央値 | スナップ移動 中央値 |")
print("|---|---:|---:|---:|---:|")
for r in foot:
    po = f"{st.median(r['pr_off']):.3f} m" if r["pr_off"] else "-"
    sd = f"{st.median(r['snap_d']):.3f} m" if r["snap_d"] else "-"
    print(f"| {r['tag']} | {r['rows']} | {r['applied']} | {po} | {sd} |")

print("\n### NMPC load (sim_time > 12 s: mean / p95)\n")
print("| シナリオ | 判定 | final x | compute ms | iters | cost | plan age s |")
print("|---|---|---:|---:|---:|---:|---:|")
for r in on_state + off_state + wide_state:
    print(f"| {r['tag']} | {r['verdict']} | {r['fx']:.2f} | "
          f"{r['ct'][0]:.2f}/{r['ct'][1]:.2f} | {r['it'][0]:.1f}/{r['it'][1]:.0f} | "
          f"{r['co'][0]:.2f}/{r['co'][1]:.2f} | {r['ag'][0]:.3f}/{r['ag'][1]:.3f} |")
