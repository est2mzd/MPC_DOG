#!/usr/bin/env python3
"""Step 08: gap width -> outcome chart (single-trench sweep, edge_clearance:=0.15).
Data transcribed from agent_reports/steps/step_08_quadsdk_full_gap_sweep.md +
the traversability-unsafe band widths measured in Step 09.
Writes artifacts/step_charts/step08_gap_width_vs_outcome.png.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

OUT = "artifacts/step_charts/step08_gap_width_vs_outcome.png"
os.makedirs(os.path.dirname(OUT), exist_ok=True)

gap_cm = [15, 25, 30, 35, 50, 100, 1000]
outcome = ["渡り切る", "渡り切る", "渡り切る", "渡り切る", "落下", "手前で停止", "手前で停止"]
col = {"渡り切る": "#2e7d32", "手前で停止": "#1565c0", "落下": "#c62828"}

fig, ax = plt.subplots(figsize=(9.4, 3.4))
ax.axvspan(0, 30, color="#2e7d32", alpha=0.09)
ax.axvspan(30, 44, color="#f9a825", alpha=0.14)
ax.axvspan(44, 62, color="#c62828", alpha=0.14)
ax.axvspan(62, 1400, color="#1565c0", alpha=0.09)
ax.text(15, 0.86, "≤30 cm  渡り切る", ha="center", fontsize=9)
ax.text(37, 0.86, "31–43 cm\n対象外", ha="center", fontsize=8)
ax.text(52, 0.86, "44–60 cm\n落下(未対応)", ha="center", fontsize=8.5, color="#b71c1c")
ax.text(300, 0.86, "≥ ~60 cm  edge_clearance:=0.15 で手前で直立停止",
        ha="center", fontsize=9, color="#0d47a1")

for g, o in zip(gap_cm, outcome):
    ax.scatter([g], [0.42], s=170, c=col[o], edgecolors="k", linewidths=0.7, zorder=5)
    ax.annotate(f"{g}", (g, 0.42), textcoords="offset points", xytext=(0, -20),
                ha="center", fontsize=8)

ax.axvline(60, ls="--", lw=1.2, color="#1565c0", alpha=0.8)
ax.annotate("しきい値 max_crossable_gap = 0.6 m", (60, 0.42),
            textcoords="offset points", xytext=(12, 22), fontsize=8, color="#0d47a1")
ax.axvline(35, ls=":", lw=1.2, color="#2e7d32", alpha=0.8)
ax.annotate("go2 実測の跨ぎ限界 ≈ 35 cm", (35, 0.42),
            textcoords="offset points", xytext=(-8, 44), ha="right", fontsize=8,
            color="#1b5e20")

ax.legend(handles=[Line2D([0], [0], marker="o", ls="", mfc=col[k], mec="k", ms=9,
                          label=k) for k in col],
          loc="lower right", fontsize=8, framealpha=0.9)
ax.set_xscale("symlog", linthresh=110)
ax.set_xlim(6, 1500)
ax.set_ylim(0, 1.05)
ax.set_yticks([])
ax.set_xlabel("物理の穴幅 [cm](110 cm 以上は対数)")
ax.set_title("Step 08:単独トレンチの穴幅 → 挙動  /  危険帯 44–60 cm(渡れず・止まらず)")
ax.set_xticks([15, 25, 30, 35, 50, 100, 1000])
ax.set_xticklabels(["15", "25", "30", "35", "50", "100", "1000"])
fig.tight_layout()
fig.savefig(OUT, dpi=125)
print("wrote", OUT)
