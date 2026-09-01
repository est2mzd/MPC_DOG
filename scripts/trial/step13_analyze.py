#!/usr/bin/env python3
"""Step 13: calibrate the graceful-stop deceleration from the 3 flat stopping
tests, then (post-processing) apply the M-step stop margin to Step 12's
BLOCKED_AT_STEP_K verdicts and show where a shadow STOP_REQUEST would fire.

  d_stop(v) = v*t_delay + v^2/(2*a_safe) + distance_margin
  required_stop_steps(v) = ceil(d_stop / (v * td_spacing))
  final_stop_steps       = max(stop_margin_steps, required_stop_steps)

Usage: step13_analyze.py [artifacts/step13] [artifacts/step12]
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
exec(open(os.path.join(os.path.dirname(__file__), "mpl_jp.py")).read())
import matplotlib.pyplot as plt

S13 = sys.argv[1] if len(sys.argv) > 1 else "artifacts/step13"
S12 = sys.argv[2] if len(sys.argv) > 2 else "artifacts/step12"
TD_SPACING = 0.225          # one touchdown-event of body travel time [s]
STOP_MARGIN_STEPS = 2       # user setting M
DIST_MARGIN = 0.10          # latch-position slop [m]
LATCH_X = 0.10              # body x at latch (safe_stop_lookahead - max_crossable_gap ~ 1.9 m before x=2.0)
VOID_X = 2.0

VS = {"v015": 0.15, "v030": 0.30, "v050": 0.50}


def load_state(p):
    rows = list(csv.DictReader(open(p)))
    out = []
    for r in rows:
        out.append((
            float(r["sim_time_s"]), float(r["base_pos_x_m"]),
            float(r["base_lin_vel_x_mps"]),
            all(r["contact_" + l].strip().lower() in ("true", "1")
                for l in ("FL", "BL", "FR", "BR")),
        ))
    return out


def calibrate(tag, v_cmd):
    st = load_state(os.path.join(S13, tag, "state_log.csv"))
    # cruise vx over the approach (x in [-1.5, -0.2])
    cru = [vx for t, x, vx, c in st if -1.5 <= x <= -0.2 and vx > 0]
    v_cruise = sorted(cru)[len(cru) // 2] if cru else v_cmd
    # t_latch: body first reaches LATCH_X
    t_latch = next((t for t, x, vx, c in st if x >= LATCH_X), st[-1][0])
    # t_vx_drop: first sustained drop below 0.8*v_cruise after t_latch
    t_drop = None
    for i, (t, x, vx, c) in enumerate(st):
        if t <= t_latch:
            continue
        if vx < 0.8 * v_cruise and all(
                st[j][2] < 0.85 * v_cruise
                for j in range(i, min(i + 5, len(st)))):
            t_drop = t
            break
    if t_drop is None:
        t_drop = t_latch
    # t_stopped: |vx| small AND all feet down, sustained ~0.5 s
    t_stop = None
    thr = max(0.05 * v_cmd, 0.02)
    for i, (t, x, vx, c) in enumerate(st):
        if t <= t_drop:
            continue
        win = st[i:i + 25]
        if all(abs(w[2]) < thr for w in win) and any(w[3] for w in win):
            t_stop = t
            break
    if t_stop is None:
        t_stop = st[-1][0]
    x_at = lambda tt: min(st, key=lambda w: abs(w[0] - tt))[1]
    x_latch = x_at(t_latch)
    x_stop = x_at(t_stop)
    d_stop = x_stop - x_latch
    t_delay = max(0.0, t_drop - t_latch)
    d_decel = x_stop - x_at(t_drop)
    a_safe = v_cruise / max(1e-3, (t_stop - t_drop))
    return dict(v=v_cmd, v_cruise=v_cruise, t_delay=t_delay, a_safe=a_safe,
               d_stop=d_stop, d_decel=d_decel, x_stop=x_stop,
               t_decel=t_stop - t_drop)


cal = {}
for tag, v in VS.items():
    p = os.path.join(S13, tag, "state_log.csv")
    if not os.path.exists(p):
        continue
    c = calibrate(tag, v)
    # keep only runs where the robot actually cruised near v_cmd and decelerated
    if c["v_cruise"] > 0.6 * v and c["t_decel"] > 0.05:
        cal[v] = c
    else:
        print(f"  (dropped {tag}: v_cruise={c['v_cruise']:.2f}, "
              f"t_decel={c['t_decel']:.2f} - robot did not walk/stop cleanly)")

print("== stopping calibration (flat, edge_clearance:=0.15) ==")
print(f"{'v_cmd':>6} {'v_cruise':>9} {'t_delay':>8} {'a_safe':>8} {'d_decel':>8} "
      f"{'d_stop':>8} {'t_decel':>8}")
for v in sorted(cal):
    c = cal[v]
    print(f"{v:6.2f} {c['v_cruise']:9.3f} {c['t_delay']:8.2f} {c['a_safe']:8.2f} "
          f"{c['d_decel']:8.3f} {c['d_stop']:8.3f} {c['t_decel']:8.2f}")

# conservative fit: smallest a_safe, largest t_delay across the valid tests
if cal:
    A_SAFE = min(c["a_safe"] for c in cal.values())
    T_DELAY = max(c["t_delay"] for c in cal.values())
    D_MEAS = {v: c["d_stop"] for v, c in cal.items()}
else:
    A_SAFE, T_DELAY, D_MEAS = 0.44, 0.19, {}
print(f"\nconservative: a_safe = {A_SAFE:.2f} m/s^2,  t_delay = {T_DELAY:.2f} s"
      f"   measured d_stop = " +
      ", ".join(f"{v}:{d:.3f}m" for v, d in sorted(D_MEAS.items())))


def d_stop_model(v):
    # physical-decel model, then floor it at the largest measured d_stop so it
    # never under-predicts the observed stopping distance.
    m = v * T_DELAY + v * v / (2 * A_SAFE)
    return max(m, max(D_MEAS.values()) if D_MEAS else m) + DIST_MARGIN


def steps_for(v):
    d = d_stop_model(v)
    req = math.ceil(d / (v * TD_SPACING))
    return d, req, max(STOP_MARGIN_STEPS, req)


print(f"\n{'v':>5} {'d_stop(model)':>13} {'step_progress':>13} "
      f"{'required_steps':>14} {'final_stop_steps':>16}")
for v in (0.15, 0.30, 0.50):
    d, req, fin = steps_for(v)
    print(f"{v:5.2f} {d:13.3f} {v*TD_SPACING:13.3f} {req:14d} {fin:16d}")

# ---- shadow behaviour on Step 12 verdicts (at v=0.30) ----
# instruction 2.4: blocked at k <= final_stop_steps -> STOP_REQUEST (issued M
# steps out); k beyond that -> SLOW and re-plan every cycle. The robot issues
# the STOP_REQUEST at the first cycle where k <= final_stop_steps (k ~ M), then
# travels d_stop; margin ahead of the block = M*step_progress - d_stop.
print("\n== shadow behaviour vs Step 12 verdicts (v=0.30) ==")
d30, req30, fin30 = steps_for(0.30)
sp_step = 0.30 * TD_SPACING
margin_at_M = fin30 * sp_step - d30
print(f"  final_stop_steps(0.30) = {fin30}  -> margin ahead of the block point "
      f"= {margin_at_M:+.2f} m")
for d in sorted(p for p in os.listdir(S12)
                if os.path.isdir(os.path.join(S12, p))):
    sp = os.path.join(S12, d, "step12_sequence.csv")
    if not os.path.exists(sp):
        continue
    rows = list(csv.DictReader(open(sp)))
    blk = [int(r["blocked_step_k"]) for r in rows
           if r["verdict"] == "BLOCKED_AT_STEP_K" and int(r["blocked_step_k"]) > 0]
    feas = sum(1 for r in rows if r["verdict"] == "FEASIBLE_TO_RANGE")
    n = len(rows) or 1
    slow = sum(1 for k in blk if k > fin30)
    stopreq = sum(1 for k in blk if k <= fin30)
    print(f"  {d:>6}: FEASIBLE {feas*100//n:3d}%   SLOW {slow*100//n:3d}%   "
          f"STOP_REQUEST {stopreq*100//n:3d}%")

# ---- plot: d_stop vs v + required steps ----
fig, ax = plt.subplots(figsize=(7.6, 3.4))
vv = [0.05 * i for i in range(3, 12)]
ax.plot(vv, [v * T_DELAY + v * v / (2 * A_SAFE) for v in vv], "--",
        color="tab:blue", alpha=0.6, label="物理減速モデル v·t_d + v²/2a")
ax.plot(vv, [d_stop_model(v) for v in vv], "-", color="tab:blue",
        label="d_stop(保守: 実測下限 + 余裕)")
for v in sorted(cal):
    ax.scatter([v], [cal[v]["d_stop"]], s=70, color="tab:red", zorder=5,
               label="実測 d_stop" if v == min(cal) else None)
ax.set_xlabel("前進速度 v [m/s]")
ax.set_ylabel("必要停止距離 d_stop [m]")
ax.set_title(f"Step 13: 停止距離の同定  (a_safe={A_SAFE:.2f} m/s², "
             f"t_delay={T_DELAY:.2f} s)")
ax2 = ax.twinx()
ax2.step(vv, [steps_for(v)[2] for v in vv], where="mid", color="tab:green",
         alpha=0.6, label="final_stop_steps")
ax2.set_ylabel("final_stop_steps", color="tab:green")
h1, l1 = ax.get_legend_handles_labels()
h2, l2 = ax2.get_legend_handles_labels()
ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")
fig.tight_layout()
os.makedirs(S13, exist_ok=True)
fig.savefig(os.path.join(S13, "step13_stopping_distance.png"), dpi=125)
print("\nplot -> artifacts/step13/step13_stopping_distance.png")
