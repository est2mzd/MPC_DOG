#!/usr/bin/env python3
"""Step 09 analysis: from the two dumped CSVs, decide for each trench whether the
50 cm-class gap is (A) hole cells misread as traversable, (B) hole cells unsafe
but the foot snaps to the far side, or (C) both. Also renders a cross-section PNG.

Usage: step09_analyze.py artifacts/step09
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts/step09")


def fnum(s):
    try:
        v = float(s)
        return v if math.isfinite(v) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def load_map(p):
    rows = list(csv.DictReader(open(p)))
    for r in rows:
        for k in ("x", "z_raw", "z_inpainted", "traversability", "hole_mask_recon"):
            r[k] = fnum(r[k])
        r["observed"] = int(r["observed"])
        r["binary_safe"] = int(r["binary_safe"])
    rows.sort(key=lambda r: r["x"])
    return rows


def load_footholds(p):
    rows = list(csv.DictReader(open(p)))
    for r in rows:
        for k in (
            "touchdown_time",
            "nominal_x",
            "selected_x",
            "selected_z_raw",
            "snap_distance",
            "hip_distance",
        ):
            r[k] = fnum(r[k])
        r["selected_observed"] = int(r["selected_observed"])
        r["selected_binary_safe"] = int(r["selected_binary_safe"])
        r["foothold_status"] = int(r["foothold_status"])
    return rows


STATUS = {
    0: "VALID",
    1: "NOMINAL_OUTSIDE_MAP",
    2: "NO_TRAVERSABLE_CANDIDATE",
    3: "NONFINITE_HEIGHT",
    4: "EDGE_TOO_CLOSE",
    5: "IK_UNREACHABLE",
}

print(f"{'tag':>10} | {'trench x-range (unobserved)':>26} | "
      f"{'trav>0.6 in trench?':>18} | verdict")
print("-" * 90)

for d in sorted(root.glob("s09_*")):
    mp = d / "step09_map_cross_section.csv"
    fp = d / "step09_footholds.csv"
    if not mp.exists():
        print(f"{d.name:>10} | (no map csv)")
        continue
    mrows = load_map(mp)
    # trench = contiguous run of unobserved (z_raw NaN) cells
    xs_hole = [r["x"] for r in mrows if r["observed"] == 0]
    if xs_hole:
        hlo, hhi = min(xs_hole), max(xs_hole)
    else:
        hlo = hhi = float("nan")
    in_tr = [r for r in mrows if hlo - 1e-6 <= r["x"] <= hhi + 1e-6]
    trav_vals = [r["traversability"] for r in in_tr if math.isfinite(r["traversability"])]
    any_safe_in_trench = any(r["binary_safe"] == 1 for r in in_tr)
    frac_safe = (sum(r["binary_safe"] for r in in_tr) / len(in_tr)) if in_tr else float("nan")

    # A?  hole cells themselves read as traversable
    A = any_safe_in_trench
    # B?  a front-leg touchdown near the trench snapped to the far side
    B = False
    fnote = ""
    if fp.exists():
        frows = load_footholds(fp)
        # front legs = FL, FR; touchdowns whose nominal_x is within ~0.5 m of the trench
        near = [
            r
            for r in frows
            if r["leg"] in ("FL", "FR")
            and math.isfinite(r["nominal_x"])
            and (hlo - 0.6) <= r["nominal_x"] <= (hhi + 0.6)
        ]
        # snapped to far side: selected_x beyond far edge AND on observed ground AND moved a lot
        farsnap = [
            r
            for r in near
            if math.isfinite(r["selected_x"])
            and r["selected_x"] > hhi
            and r["selected_observed"] == 1
            and r["snap_distance"] > 0.10
        ]
        holeplant = [
            r for r in near if r["selected_observed"] == 0
        ]  # foot placed where raw z is NaN
        B = len(farsnap) > 0
        fnote = (
            f" near_td={len(near)} farsnap={len(farsnap)} holeplant={len(holeplant)}"
            f" statuses={sorted(set(STATUS[r['foothold_status']] for r in near))}"
        )

    verdict = "C (A+B)" if (A and B) else ("A" if A else ("B" if B else "neither"))
    print(
        f"{d.name:>10} | [{hlo:6.2f},{hhi:6.2f}] w={hhi-hlo:.2f} | "
        f"{('YES ' + f'{frac_safe*100:.0f}%'):>18} | {verdict}{fnote}"
    )

    # traversability-unsafe cells (NaN or <= 0.6)
    un = [
        r["x"]
        for r in mrows
        if (not math.isfinite(r["traversability"])) or r["traversability"] <= 0.6
    ]

    # ---- cross-section plot (zoomed on the trench) ----
    fig, ax = plt.subplots(figsize=(9, 3.6))
    xs = [r["x"] for r in mrows]
    # raw z: NaN over the void -> plot only finite, so the gap shows as a break
    ax.plot(xs, [r["z_raw"] for r in mrows], "o", ms=4, color="tab:blue",
            label="z_raw  (missing = raw NaN / void)")
    ax.plot(xs, [r["z_inpainted"] for r in mrows], "-", lw=1.4, color="tab:green",
            label="z_inpainted  (hole filled)")
    ax.set_ylabel("height [m]")
    ax.set_xlabel("x [m]")
    ax.set_ylim(-0.06, 0.06)
    if math.isfinite(hlo):
        ax.axvspan(hlo - 0.025, hhi + 0.025, color="0.5", alpha=0.25,
                   label=f"raw z NaN band  ({hhi - hlo + 0.05:.2f} m)")
    if un:
        ax.axvspan(min(un) - 0.025, max(un) + 0.025, ymin=0.0, ymax=1.0,
                   facecolor="none", edgecolor="tab:red", hatch="///", lw=0,
                   label=f"traversability unsafe band  ({max(un) - min(un) + 0.05:.2f} m)")
    if fp.exists():
        blk = mag = None
        for r in load_footholds(fp):
            if r["leg"] in ("FL", "FR") and math.isfinite(r["selected_x"]):
                if r["selected_observed"]:
                    blk = ax.plot(r["selected_x"], -0.045, "^", ms=7,
                                  color="k", alpha=0.5)[0]
                else:
                    mag = ax.plot(r["selected_x"], -0.045, "^", ms=8,
                                  color="magenta")[0]
        if blk:
            blk.set_label("selected foothold on real ground")
        if mag:
            mag.set_label("selected foothold where raw z = NaN (over the void)")
    lo = (hlo if math.isfinite(hlo) else 2.0) - 0.9
    hi = (hhi if math.isfinite(hhi) else 2.5) + 0.9
    ax.set_xlim(lo, hi)
    ax.axhline(0.0, lw=0.5, color="0.7")
    ax.set_title(f"{d.name}: terrain-map cross-section at y≈0 + front-leg footholds "
                 f"(max_crossable_gap = 0.60 m)")
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(d / f"{d.name}_cross_section.png", dpi=120)
    plt.close(fig)

print("\nplots -> artifacts/step09/*/s09_*_cross_section.png")
