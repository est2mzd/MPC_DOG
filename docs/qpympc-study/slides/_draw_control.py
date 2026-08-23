# -*- coding: utf-8 -*-
"""Control-logic figures for a new deck. Does not change _draw.py.

Same visual language: dog / point-mass / forces at contacts.
No body-to-foot bars. Force is not along a link.
"""

from __future__ import annotations

from pathlib import Path

from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

from _draw import (
    FIG,
    GOLD,
    INK,
    MUTED,
    NAVY,
    PAPER,
    TEAL,
    TERR,
    _arrow,
    _ax,
    _dog_side,
    _dog_top,
    _ground,
    _save,
    _srbd_free_body,
    _txt,
)


def _panel(ax, x0, y0, w, h, title):
    ax.add_patch(
        FancyBboxPatch(
            (x0, y0),
            w,
            h,
            boxstyle="round,pad=0.02,rounding_size=0.04",
            facecolor="#FFFFFF",
            edgecolor="#D4CFC6",
            lw=1.0,
        )
    )
    _txt(ax, x0 + 0.12, y0 + h - 0.22, title, 11, NAVY, weight="bold")


def logic_story() -> Path:
    """One cycle as five scenes, not a box pipeline."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11.2, 4.7), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.set_xlim(0, 11.2)
    ax.set_ylim(0, 4.7)
    ax.set_aspect("equal")
    ax.axis("off")
    _txt(ax, 0.25, 4.45, "1周期で、制御が犬にすること", 14, NAVY, weight="bold")

    # 1 command
    _panel(ax, 0.2, 0.35, 2.05, 3.85, "1  速さ")
    inset = ax.inset_axes([0.035, 0.12, 0.16, 0.58])
    inset.set_facecolor("white")
    inset.axis("off")
    inset.set_aspect("equal")
    _dog_top(inset, 0.0, 0.0, 0.35, 0.55)
    inset.set_xlim(-1.4, 1.6)
    inset.set_ylim(-1.2, 1.3)
    _txt(ax, 1.22, 0.62, "前へ 0.5", 10, TEAL, ha="center")

    # 2 gait
    _panel(ax, 2.4, 0.35, 2.05, 3.85, "2  誰が押す")
    inset = ax.inset_axes([0.225, 0.12, 0.17, 0.58])
    inset.set_facecolor("white")
    inset.axis("off")
    inset.set_aspect("equal")
    _dog_side(inset, 0.0, 0.0, 0.72, swing_front=True, swing_rear=False)
    inset.set_xlim(-1.3, 1.3)
    inset.set_ylim(-0.15, 1.7)
    _txt(ax, 3.42, 0.62, "対角だけ", 10, TERR, ha="center")

    # 3 foothold
    _panel(ax, 4.6, 0.35, 2.05, 3.85, "3  次の点")
    _txt(ax, 5.62, 2.55, "腰の下へ", 11, INK, ha="center")
    _txt(ax, 5.62, 2.15, "進む先を足す", 11, INK, ha="center")
    ax.plot(5.25, 1.35, "o", color=NAVY, ms=8)
    ax.plot(6.05, 0.95, "D", color=TERR, ms=8)
    ax.plot([5.25, 6.05], [1.35, 0.95], color=TERR, ls=":", lw=1.2)
    _txt(ax, 5.62, 0.62, "地形は見ない", 10, MUTED, ha="center")

    # 4 predict
    _panel(ax, 6.8, 0.35, 2.05, 3.85, "4  未来の力")
    inset = ax.inset_axes([0.615, 0.14, 0.175, 0.52])
    inset.set_facecolor("white")
    inset.axis("off")
    inset.set_aspect("equal")
    _srbd_free_body(inset, (0.0, 0.85), [(-0.7, 0.05), (0.7, 0.05)], [(-0.35, 0.35), (0.35, 0.35)], show_wrench=False)
    inset.set_xlim(-1.15, 1.15)
    inset.set_ylim(-0.15, 1.55)
    _txt(ax, 7.82, 0.62, "形も棒も無い", 10, MUTED, ha="center")

    # 5 torque
    _panel(ax, 9.0, 0.35, 2.00, 3.85, "5  回す力")
    inset = ax.inset_axes([0.81, 0.12, 0.17, 0.58])
    inset.set_facecolor("white")
    inset.axis("off")
    inset.set_aspect("equal")
    _dog_side(inset, 0.0, 0.0, 0.72, swing_front=True, swing_rear=False)
    inset.set_xlim(-1.3, 1.3)
    inset.set_ylim(-0.15, 1.7)
    _txt(ax, 10.00, 0.62, "τ 12", 10, TERR, ha="center")

    for x in (2.28, 4.48, 6.68, 8.88):
        ax.annotate("", xy=(x + 0.08, 2.2), xytext=(x - 0.08, 2.2), arrowprops=dict(arrowstyle="-|>", color=GOLD, lw=1.4))
    return _save(fig, "ctl_story.png")


def cmd_on_dog() -> Path:
    import math

    fig, ax = _ax(10.4, 4.5)
    ang = 0.42
    hx, hy = _dog_top(ax, 2.2, 1.75, ang, 0.82)
    _arrow(ax, hx, hy, hx + 1.45 * math.cos(ang), hy + 1.45 * math.sin(ang), color=TERR, lw=2.4)
    _txt(ax, hx + 0.35, hy + 1.15, "人が言う前  $v^H$", 12, TERR)
    _arrow(ax, 5.7, 1.15, 7.35, 1.15, color=NAVY, lw=2)
    _arrow(ax, 5.7, 1.15, 5.7, 2.55, color=NAVY, lw=2)
    _txt(ax, 7.45, 1.15, "地図 x", 11, NAVY)
    _txt(ax, 5.7, 2.72, "地図 y", 11, NAVY, ha="center")
    _txt(ax, 5.55, 3.55, "制御は行き先を作らない。速さだけを地図へ回す。", 13, INK)
    _txt(ax, 0.1, 3.55, "上から見た犬", 12, MUTED)
    ax.set_xlim(-0.2, 8.5)
    ax.set_ylim(0.2, 4.0)
    return _save(fig, "ctl_cmd.png")


def who_pushes() -> Path:
    fig, ax = _ax(10.4, 4.5)
    _ground(ax, -0.2, 6.6)
    com, feet = _dog_side(ax, 2.15, 0.0, 1.1, swing_front=True, swing_rear=False)
    for (fx, fy), down in feet:
        if down:
            _arrow(ax, fx, fy + 0.02, fx, fy + 0.78, color=TERR, lw=2.4)
            _txt(ax, fx + 0.08, fy + 0.62, "c=1  押してよい", 11, TERR)
        else:
            _txt(ax, fx + 0.06, fy + 0.16, "c=0  運ぶ", 11, MUTED)
    _txt(ax, com[0] + 0.16, com[1] + 0.26, "重心", 11, GOLD)
    _txt(ax, 4.85, 2.55, "時計が先。力の計算は後。", 13, INK)
    _txt(ax, 4.85, 2.10, "対角の2脚だけが地面。", 12, INK)
    _txt(ax, 4.85, 1.65, "計算はリズムを選べない。", 12, INK)
    ax.set_xlim(-0.3, 8.0)
    ax.set_ylim(-0.3, 2.9)
    return _save(fig, "ctl_gait.png")


def next_point() -> Path:
    fig, ax = _ax(10.4, 4.5)
    _ground(ax, -0.2, 7.5)
    _dog_side(ax, 1.65, 0.0, 1.02, swing_front=True, swing_rear=False)
    hip = (1.65 + 0.68, 0.97)
    land = (3.55, 0.05)
    ax.plot(*hip, "o", color=NAVY, ms=8)
    ax.plot(*land, "D", color=TERR, ms=10)
    ax.plot([hip[0], land[0]], [hip[1], land[1]], color=TERR, ls=":", lw=1.5)
    _arrow(ax, 1.85, 1.58, 3.25, 1.58, color=TEAL, lw=2.2)
    _txt(ax, 2.35, 1.80, "進む速さ v", 12, TEAL)
    _txt(ax, hip[0] - 0.22, hip[1] + 0.28, "腰", 12, NAVY)
    _txt(ax, land[0] - 0.05, land[1] + 0.40, "次に置く点", 12, TERR)
    _txt(ax, 4.55, 2.45, "点線は幾何の印。脚の棒ではない。", 12, INK)
    _txt(ax, 4.55, 2.00, "リズムは時刻だけ。場所は別。", 12, INK)
    _txt(ax, 4.55, 1.55, "標準は地形を見ない。", 12, INK)
    ax.set_xlim(-0.3, 7.8)
    ax.set_ylim(-0.3, 2.85)
    return _save(fig, "ctl_foothold.png")


def predict_points() -> Path:
    fig, ax = _ax(10.2, 4.5)
    _ground(ax, 0.2, 5.4, y=0.15)
    com = (2.85, 1.70)
    _srbd_free_body(ax, com, [(1.55, 0.20), (4.15, 0.20)], [(1.95, 0.62), (3.75, 0.62)])
    _txt(ax, com[0] + 0.22, com[1] + 0.08, "m, I", 12, GOLD)
    _txt(ax, com[0] + 0.08, com[1] + 0.70, "Σ cF", 11, GOLD)
    _txt(ax, com[0] + 0.08, com[1] - 0.70, "mg", 11, MUTED)
    _txt(ax, com[0] + 0.42, com[1] + 0.30, "Σ r×F", 11, GOLD)
    _txt(ax, 1.55, 1.05, "cF", 11, TERR, ha="center")
    _txt(ax, 4.15, 1.05, "cF", 11, TERR, ha="center")
    _txt(ax, 5.55, 2.80, "予測の中の体。", 13, INK)
    _txt(ax, 5.55, 2.35, "形は無い。棒は無い。", 12, INK)
    _txt(ax, 5.55, 1.90, "力は接地点で世界向き。", 12, INK)
    _txt(ax, 5.55, 1.45, "重心へは合力とモーメント。", 12, INK)
    ax.set_xlim(0.1, 8.4)
    ax.set_ylim(-0.25, 3.15)
    return _save(fig, "ctl_predict.png")


def gate_horizon() -> Path:
    """Future contact flags sit on isolated feet — not links."""
    fig, ax = _ax(10.6, 4.5)
    _ground(ax, 0.15, 10.2, y=0.12)
    xs = [1.15, 3.15, 5.15, 7.15, 9.15]
    labels = ["今", "+40ms", "+80ms", "+160ms", "+240ms"]
    stance_pat = [True, False, True, False, True]
    for x, lab, st in zip(xs, labels, stance_pat):
        _srbd_free_body(
            ax,
            (x, 1.55),
            ([(x - 0.42, 0.16), (x + 0.42, 0.16)] if st else []),
            ([] if st else [(x - 0.28, 0.48), (x + 0.28, 0.48)]),
            show_wrench=False,
        )
        _txt(ax, x, 2.55, lab, 11, NAVY if lab == "今" else MUTED, ha="center")
        _txt(ax, x, -0.12, "c=1" if st else "c=0", 10, TERR if st else MUTED, ha="center")
    _txt(ax, 0.2, 3.15, "接地列は人が決めた時間割。予測はそれを変えない。", 13, INK)
    ax.set_xlim(0.0, 10.4)
    ax.set_ylim(-0.35, 3.35)
    return _save(fig, "ctl_horizon.png")


def use_first_only() -> Path:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10.4, 3.7), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.axis("off")
    ax.annotate("", xy=(9.5, 1.35), xytext=(0.35, 1.35), arrowprops=dict(arrowstyle="-|>", color=NAVY, lw=1.8))
    _txt(ax, 9.6, 1.35, "時間", 11, NAVY)
    for i in range(12):
        x = 0.55 + i * 0.72
        ax.add_patch(
            Rectangle(
                (x, 1.08),
                0.62,
                0.54,
                facecolor=TERR if i == 0 else "#D7E6E6",
                edgecolor=NAVY,
                lw=0.8,
                alpha=0.95 if i == 0 else 0.7,
            )
        )
        _txt(ax, x + 0.31, 1.35, str(i), 10, "white" if i == 0 else NAVY, ha="center")
    _txt(ax, 0.85, 2.20, "実行へ渡すのはこの1コマの力", 13, TERR)
    _txt(ax, 4.4, 0.55, "残りは次の瞬間に捨てて、今の犬で解き直す", 12, MUTED)
    _txt(ax, 0.35, 2.95, "未来 0.24 s（12コマ × 0.02 s）", 13, INK)
    ax.set_xlim(0.15, 10.1)
    ax.set_ylim(0.2, 3.25)
    return _save(fig, "ctl_recede.png")


def mask_forces() -> Path:
    fig, ax = _ax(10.4, 4.4)
    _ground(ax, 0.3, 10.0)
    names = [("左前", True), ("右前", False), ("左後", False), ("右後", True)]
    for i, (name, keep) in enumerate(names):
        x = 1.3 + i * 2.35
        ax.plot(x, 0.10, "s", color=TERR if keep else MUTED, ms=12)
        _arrow(ax, x, 0.16, x, 1.45, color=GOLD, lw=2.0)
        _txt(ax, x, 1.65, "MPCの力", 10, GOLD, ha="center")
        if keep:
            _arrow(ax, x + 0.55, 0.16, x + 0.55, 1.45, color=TERR, lw=2.2)
            _txt(ax, x + 0.55, 1.65, "渡す", 10, TERR, ha="center")
        else:
            _arrow(ax, x + 0.55, 0.16, x + 0.55, 1.45, color=MUTED, lw=1.4)
            ax.plot([x + 0.35, x + 0.75], [0.55, 1.15], color=TERR, lw=2.2)
            _txt(ax, x + 0.55, 1.65, "捨てる", 10, TERR, ha="center")
        _txt(ax, x + 0.25, -0.22, name + ("  c=1" if keep else "  c=0"), 11, NAVY, ha="center")
    _txt(ax, 0.35, 2.55, "計算は4足ぶん出す。今地面の足だけ、先頭の1コマを残す。", 13, INK)
    ax.set_xlim(0.2, 10.2)
    ax.set_ylim(-0.45, 2.9)
    return _save(fig, "ctl_mask.png")


def stance_jacobian() -> Path:
    fig, ax = _ax(10.2, 4.6)
    _ground(ax, 0.25, 5.4)
    hip, knee, foot = (2.15, 2.18), (2.55, 1.18), (2.32, 0.08)
    ax.plot([hip[0], knee[0], foot[0]], [hip[1], knee[1], foot[1]], color=NAVY, lw=5.0, solid_capstyle="round")
    for p in (hip, knee, foot):
        ax.plot(*p, "o", color=TEAL, ms=10)
    _arrow(ax, foot[0], foot[1] + 0.04, foot[0], foot[1] + 1.12, color=TERR, lw=2.4)
    _txt(ax, foot[0] + 0.16, foot[1] + 0.78, "指令の力 F", 13, TERR)
    _txt(ax, foot[0] + 0.16, foot[1] + 0.48, "世界の向き", 11, MUTED)
    for (x, y), name in ((hip, "τ1"), (knee, "τ2"), ((2.38, 0.52), "τ3")):
        ax.add_patch(
            FancyArrowPatch(
                (x + 0.24, y + 0.14),
                (x + 0.24, y - 0.16),
                connectionstyle="arc3,rad=0.85",
                arrowstyle="-|>",
                mutation_scale=12,
                color=GOLD,
                lw=1.7,
            )
        )
        _txt(ax, x + 0.42, y, name, 12, GOLD)
    _txt(ax, 4.35, 2.75, "これは本物の脚。", 13, INK)
    _txt(ax, 4.35, 2.28, "力は接地点で上向き。", 12, INK)
    _txt(ax, 4.35, 1.82, "回す力は関節。J で対応させる。", 12, INK)
    _txt(ax, 4.35, 1.36, "重心へ棒を伝う力ではない。", 12, TERR)
    ax.set_xlim(0.15, 8.2)
    ax.set_ylim(-0.2, 3.15)
    return _save(fig, "ctl_stance.png")


def swing_curve() -> Path:
    fig, ax = _ax(10.4, 4.5)
    _ground(ax, -0.15, 7.6)
    _dog_side(ax, 1.55, 0.0, 0.95, swing_front=True, swing_rear=False)
    xs = [1.95, 2.45, 3.05, 3.65]
    ys = [0.42, 0.72, 0.58, 0.06]
    ax.plot(xs, ys, color=TEAL, lw=2.0)
    ax.plot(xs, ys, "o", color=TEAL, ms=6)
    ax.plot(xs[-1], ys[-1], "D", color=TERR, ms=10)
    _txt(ax, 2.85, 0.95, "空中の道", 12, TEAL)
    _txt(ax, 3.75, 0.32, "着く点", 12, TERR)
    _txt(ax, 4.7, 2.45, "遊脚は、立脚の式を使わない。", 12, INK)
    _txt(ax, 4.7, 2.00, "先に書いた τ を上書きする。", 12, INK)
    _txt(ax, 4.7, 1.55, "終点は予測が出した着地点。", 12, INK)
    ax.set_xlim(-0.25, 8.0)
    ax.set_ylim(-0.25, 2.85)
    return _save(fig, "ctl_swing.png")


def twelve_into_plant() -> Path:
    fig, ax = _ax(10.6, 4.6)
    _ground(ax, -0.15, 10.1)
    _dog_side(ax, 3.15, 0.0, 1.05, swing_front=True, swing_rear=False)
    _txt(ax, 3.15, 2.55, "今の犬  12モータ", 13, NAVY, ha="center")
    _txt(ax, 0.15, 2.55, "人", 13, NAVY)
    _arrow(ax, 0.55, 2.15, 1.55, 1.35, color=TEAL, lw=2)
    _txt(ax, 0.15, 1.85, "速さ", 12, TEAL)
    ax.plot([6.55, 7.25, 6.70], [0.95, 0.52, 0.06], color="#B8C4C4", lw=3)
    ax.add_patch(FancyBboxPatch((6.05, 0.82), 1.55, 0.30, boxstyle="round,pad=0.02", facecolor="#D7E0E0", edgecolor="#8AA0A0", lw=1, alpha=0.85))
    _txt(ax, 6.82, 1.52, "次の体", 12, MUTED, ha="center")
    _arrow(ax, 4.45, 1.05, 5.95, 1.05, color=TERR, lw=2)
    _txt(ax, 4.85, 1.32, "τ 12", 12, TERR)
    _txt(ax, 0.15, 0.45, "地面の実力が返す力は、切替には使わない", 11, MUTED)
    ax.set_xlim(-0.25, 8.6)
    ax.set_ylim(-0.28, 3.0)
    return _save(fig, "ctl_loop.png")


def three_forces() -> Path:
    fig, ax = _ax(10.2, 4.3)
    _ground(ax, 0.4, 9.5)
    ax.plot(2.0, 0.08, "s", color=NAVY, ms=13)
    _arrow(ax, 2.0, 0.14, 2.0, 1.52, color=GOLD, lw=2.2)
    _txt(ax, 2.15, 1.70, "計算が出した力", 12, GOLD)
    ax.plot(4.7, 0.08, "s", color=NAVY, ms=13)
    _arrow(ax, 4.7, 0.14, 4.7, 1.52, color=TEAL, lw=2.2)
    _txt(ax, 4.85, 1.70, "実行が使う力", 12, TEAL)
    ax.plot(7.4, 0.08, "s", color=NAVY, ms=13)
    _arrow(ax, 7.4, 0.14, 7.4, 1.52, color=TERR, lw=2.2)
    _txt(ax, 7.55, 1.70, "地面が返した力", 12, TERR)
    _txt(ax, 2.0, -0.28, "空中でも残り得る", 10, MUTED, ha="center")
    _txt(ax, 4.7, -0.28, "空中は 0 にする", 10, MUTED, ha="center")
    _txt(ax, 7.4, -0.28, "切替には使わない", 10, MUTED, ha="center")
    _txt(ax, 0.4, 2.50, "同じ足でも、3つの力は別物", 14, NAVY)
    ax.set_xlim(0.2, 9.7)
    ax.set_ylim(-0.55, 2.85)
    return _save(fig, "ctl_threeF.png")


def two_clocks() -> Path:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10.4, 3.5), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.axis("off")
    ax.annotate("", xy=(9.6, 2.15), xytext=(0.4, 2.15), arrowprops=dict(arrowstyle="-|>", color=NAVY, lw=1.6))
    for i in range(26):
        x = 0.5 + i * 0.34
        h = 0.55 if i % 5 == 0 else 0.22
        ax.plot([x, x], [2.15, 2.15 + h], color=TEAL if i % 5 == 0 else MUTED, lw=1.6 if i % 5 == 0 else 0.8)
    _txt(ax, 0.5, 3.05, "速い時計  秒間500回   リズム・回す力・物体を進める", 12, TEAL)
    _txt(ax, 0.5, 1.35, "遅い時計  秒間100回   未来の押し方を解き直す（太い目盛り）", 12, NAVY)
    _txt(ax, 0.5, 0.70, "間の4回は、直前の押し方を使い続ける", 12, MUTED)
    ax.set_xlim(0.2, 10.0)
    ax.set_ylim(0.3, 3.3)
    return _save(fig, "ctl_clocks.png")


def all_control_figures() -> dict[str, Path]:
    FIG.mkdir(parents=True, exist_ok=True)
    return {
        "story": logic_story(),
        "cmd": cmd_on_dog(),
        "gait": who_pushes(),
        "foothold": next_point(),
        "predict": predict_points(),
        "horizon": gate_horizon(),
        "recede": use_first_only(),
        "mask": mask_forces(),
        "stance": stance_jacobian(),
        "swing": swing_curve(),
        "loop": twelve_into_plant(),
        "three": three_forces(),
        "clocks": two_clocks(),
    }
