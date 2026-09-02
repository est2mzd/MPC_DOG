#!/usr/bin/env python3
"""Step 14: connect the multi-step foothold-sequence verdict to the graceful
stop (opt-in). Reads the per-scenario state_log.csv + run.log written by
step14_measure.sh, classifies each run (CROSSED / SAFE-STOP / FELL), and draws:

  - wide voids ON (50/100 cm x3): body x where the robot came to rest, against
    the void near edge (x = 2.00 m in every flat_trench_s09_* world).
  - small gaps ON + feature OFF: final body x, to show ON does not false-stop a
    crossable gap and OFF still matches Step 08.

Usage: step14_analyze.py [artifacts/step14]
"""
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

S14 = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step14"
VOID_X = 2.00          # solid top ends here in every flat_trench_s09_* world
CROSS_X = 4.00         # body x past this => the robot cleared the gap field

WIDE = ["g50_on_1", "g50_on_2", "g50_on_3", "g100_on_1", "g100_on_2", "g100_on_3"]
OTHER = ["g35_on", "g30_on", "r15_on", "g30_off", "g50_off"]
LABEL = {
    "g50_on_1": "50cm ON #1", "g50_on_2": "50cm ON #2", "g50_on_3": "50cm ON #3",
    "g100_on_1": "100cm ON #1", "g100_on_2": "100cm ON #2", "g100_on_3": "100cm ON #3",
    "g35_on": "35cm ON", "g30_on": "30cm ON\n(flat_gaps_2m)", "r15_on": "15cm x3 ON",
    "g30_off": "30cm OFF", "g50_off": "50cm OFF",
}


def analyze(tag):
    d = os.path.join(S14, tag)
    csv_path = os.path.join(d, "state_log.csv")
    log_path = os.path.join(d, "run.log")
    if not os.path.isfile(csv_path):
        return None
    last_x = last_z = last_roll = float("nan")
    min_z = float("inf")
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t = float(row["sim_time_s"])
                x = float(row["base_pos_x_m"])
                z = float(row["base_pos_z_m"])
                roll = float(row["base_roll_rad"])
            except (KeyError, ValueError):
                continue
            last_x, last_z, last_roll = x, z, roll
            if t > 12.0:
                min_z = min(min_z, z)
    if min_z == float("inf"):
        min_z = last_z
    mstop = slow = 0
    if os.path.isfile(log_path):
        with open(log_path, errors="ignore") as f:
            for line in f:
                if "multistep-stop] latching" in line:
                    mstop += 1
                elif "multistep-stop] SLOW" in line:
                    slow += 1
    if abs(last_roll) > 0.8 or last_z < 0.15 or min_z < 0.15:
        verdict = "FELL"
    elif last_x > CROSS_X:
        verdict = "CROSSED"
    else:
        verdict = "SAFE-STOP"
    return dict(tag=tag, x=last_x, z=last_z, roll=last_roll, min_z=min_z,
               mstop=mstop, slow=slow, verdict=verdict)


rows = {t: analyze(t) for t in WIDE + OTHER}
rows = {k: v for k, v in rows.items() if v}

COL = {"CROSSED": "#2e7d32", "SAFE-STOP": "#1565c0", "FELL": "#c62828"}

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 5.0))

# --- panel 1: wide voids ON -> rest position vs void near edge ------------
w = [rows[t] for t in WIDE if t in rows]
y = list(range(len(w)))
ax1.barh(y, [r["x"] for r in w],
         color=[COL[r["verdict"]] for r in w], height=0.6)
ax1.axvline(VOID_X, color="#c62828", lw=2, ls="--")
ax1.text(VOID_X + 0.03, len(w) - 0.4, "空洞の手前の縁 x=2.00 m",
         color="#c62828", va="top", fontsize=10)
for i, r in enumerate(w):
    m = VOID_X - r["x"]
    ax1.text(r["x"] + 0.03, i, f"x={r['x']:.2f}  余裕 {m:.2f} m", va="center", fontsize=9)
ax1.set_yticks(y)
ax1.set_yticklabels([LABEL[r["tag"]] for r in w])
ax1.set_xlabel("停止時の胴体 x [m]  (spawn x=-2.0, v=0.3 m/s)")
ax1.set_xlim(-2.2, 2.6)
ax1.invert_yaxis()
ax1.set_title("広い空洞 ON:M 歩手前で直立停止(3/3 転倒なし)")
ax1.grid(axis="x", alpha=0.3)

# --- panel 2: small gaps ON + feature OFF -------------------------------
o = [rows[t] for t in OTHER if t in rows]
y2 = list(range(len(o)))
ax2.barh(y2, [r["x"] for r in o],
         color=[COL[r["verdict"]] for r in o], height=0.6)
ax2.axvline(CROSS_X, color="#555", lw=1.2, ls=":")
ax2.text(CROSS_X + 0.05, -0.4, "これ以降=通過", color="#555", fontsize=9)
for i, r in enumerate(o):
    ax2.text(min(r["x"], 9.2) + 0.05, i,
             f"x={r['x']:.2f}  {r['verdict']}", va="center", fontsize=9)
ax2.set_yticks(y2)
ax2.set_yticklabels([LABEL[r["tag"]] for r in o])
ax2.set_xlabel("最終の胴体 x [m]")
ax2.set_xlim(0, 10.5)
ax2.invert_yaxis()
ax2.set_title("小さい穴 ON:不要停止なし / feature OFF:Step 08 と一致")
ax2.grid(axis="x", alpha=0.3)

handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in COL.values()]
fig.legend(handles, COL.keys(), loc="lower center", ncol=3, frameon=False)
fig.suptitle("Step 14:多歩足場列プランナ → graceful stop(opt-in)", fontsize=13)
fig.tight_layout(rect=(0, 0.05, 1, 0.96))
out = os.path.join(S14, "step14_stop_position.png")
fig.savefig(out, dpi=110)
print("wrote", out)

# --- markdown tables for the doc / README ------------------------------
print("\n| シナリオ | mode | 判定 | 停止/最終 x | 空洞縁までの余裕 | roll | min z | mstop | slow |")
print("|---|---|---|---:|---:|---:|---:|---:|---:|")
for t in WIDE + OTHER:
    if t not in rows:
        continue
    r = rows[t]
    marg = f"{VOID_X - r['x']:.2f} m" if t in WIDE else "-"
    mode = "ON" if "on" in t or t.endswith("_on") else ("OFF" if "off" in t else "ON")
    print(f"| {LABEL[t].replace(chr(10),' ')} | {mode} | {r['verdict']} | "
          f"{r['x']:.2f} | {marg} | {r['roll']:+.2f} | {r['min_z']:.3f} | "
          f"{r['mstop']} | {r['slow']} |")
