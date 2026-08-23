# -*- coding: utf-8 -*-
"""Textbook figures: dog, forces, frames, gait — not architecture boxes."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle

FONT = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FIG = Path(__file__).resolve().parent / "_figcache"
NAVY = "#1B3A4B"
TEAL = "#2A6F6F"
TERR = "#C45A20"
GOLD = "#B48A28"
INK = "#1E1E24"
MUTED = "#5A606E"
PAPER = "#F7F5F0"


def _ax(w=10.2, h=4.4):
    FIG.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(w, h), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def _txt(ax, x, y, s, size=11, color=INK, ha="left", va="center", weight="regular"):
    ax.text(x, y, s, fontproperties=FONT, fontsize=size, color=color, ha=ha, va=va, fontweight=weight)


def _arrow(ax, x1, y1, x2, y2, color=TERR, lw=2.0):
    ax.add_patch(
        FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=14, lw=lw, color=color)
    )


def _save(fig, name: str) -> Path:
    path = FIG / name
    fig.savefig(path, bbox_inches="tight", facecolor=PAPER, pad_inches=0.08)
    plt.close(fig)
    return path


def _ground(ax, x0=-0.4, x1=6.4, y=0.0):
    ax.plot([x0, x1], [y, y], color=NAVY, lw=2.2)
    for x in [i * 0.35 for i in range(-1, 20)]:
        ax.plot([x, x - 0.12], [y, y - 0.12], color=MUTED, lw=0.8)


def _dog_side(ax, x=2.2, y=0.0, scale=1.0, swing_front=False, swing_rear=False):
    """Simple side-view dog. x is hip center."""
    s = scale
    body_x, body_y = x, y + 0.95 * s
    ax.add_patch(FancyBboxPatch((body_x - 0.95 * s, body_y - 0.18 * s), 1.9 * s, 0.36 * s, boxstyle="round,pad=0.02", facecolor=TEAL, edgecolor=NAVY, lw=1.4, mutation_aspect=0.4))
    ax.add_patch(Circle((body_x + 1.05 * s, body_y + 0.08 * s), 0.16 * s, facecolor=NAVY, edgecolor=NAVY))
    ax.plot([body_x + 1.15 * s, body_x + 1.38 * s], [body_y + 0.08 * s, body_y + 0.02 * s], color=NAVY, lw=3)
    com = (body_x, body_y)
    ax.plot(*com, "o", color=GOLD, ms=9, zorder=5)
    legs = [
        (body_x + 0.62 * s, not swing_front),
        (body_x - 0.62 * s, not swing_rear),
    ]
    feet = []
    for hx, down in legs:
        knee = (hx + 0.08 * s, body_y - 0.42 * s)
        if down:
            foot = (hx + 0.02 * s, y + 0.02)
        else:
            foot = (hx + 0.28 * s, y + 0.38 * s)
        ax.plot([hx, knee[0], foot[0]], [body_y - 0.12 * s, knee[1], foot[1]], color=NAVY, lw=3.2, solid_capstyle="round")
        ax.plot(foot[0], foot[1], "s", color=TERR if down else MUTED, ms=7)
        feet.append((foot, down))
    return com, feet


def dog_and_plant_eq() -> Path:
    fig, ax = _ax(10.4, 4.6)
    _ground(ax, -0.2, 7.2)
    com, feet = _dog_side(ax, 2.4, 0.0, 1.15, swing_front=True, swing_rear=False)
    _arrow(ax, com[0], com[1], com[0], com[1] - 0.72, color=MUTED, lw=2.2)
    _txt(ax, com[0] + 0.12, com[1] - 0.38, "mg  重さ", 11, MUTED)
    _txt(ax, com[0] + 0.18, com[1] + 0.22, "重心", 12, GOLD)
    for (fx, fy), down in feet:
        if down:
            _arrow(ax, fx, fy + 0.02, fx, fy + 0.72, color=TERR, lw=2.4)
            _txt(ax, fx + 0.1, fy + 0.55, "λ  地面が返す力", 11, TERR)
        else:
            _txt(ax, fx + 0.08, fy + 0.12, "空中の足", 11, MUTED)
    _txt(ax, 5.1, 2.55, "関節で回す力 τ", 13, NAVY)
    _txt(ax, 5.1, 2.15, "が足へ伝わり、", 12, INK)
    _txt(ax, 5.1, 1.78, "地面の力 λ が決まる。", 12, INK)
    _txt(ax, 5.1, 1.25, "目標の押し方を地面へ", 12, INK)
    _txt(ax, 5.1, 0.90, "直接書き込んではいない。", 12, INK)
    ax.set_xlim(-0.3, 7.4)
    ax.set_ylim(-0.35, 2.9)
    return _save(fig, "dog_plant_eq.png")


def _dog_top(ax, cx, cy, ang, scale=1.0):
    """Top-down dog. ang is heading from +x, radians. Returns com."""
    import math

    s = scale
    c, si = math.cos(ang), math.sin(ang)
    def rot(dx, dy):
        return cx + (dx * c - dy * si) * s, cy + (dx * si + dy * c) * s

    body = [rot(-1.05, -0.38), rot(0.95, -0.38), rot(0.95, 0.38), rot(-1.05, 0.38)]
    xs, ys = zip(*body)
    ax.fill(xs, ys, facecolor=TEAL, edgecolor=NAVY, lw=1.4, zorder=2)
    head = [rot(0.95, -0.22), rot(1.38, -0.16), rot(1.38, 0.16), rot(0.95, 0.22)]
    hx, hy = zip(*head)
    ax.fill(hx, hy, facecolor=NAVY, edgecolor=NAVY, zorder=3)
    for dx, dy in ((0.62, 0.55), (0.62, -0.55), (-0.62, 0.55), (-0.62, -0.55)):
        px, py = rot(dx, dy)
        hx0, hy0 = rot(dx * 0.22, dy * 0.22)
        ax.plot([hx0, px], [hy0, py], color=NAVY, lw=2.4, zorder=1)
        ax.plot(px, py, "o", color=TERR, ms=9, zorder=4)
    ax.plot(cx, cy, "o", color=GOLD, ms=8, zorder=5)
    return cx, cy


def heading_vs_world() -> Path:
    import math

    fig, ax = _ax(10.2, 4.5)
    ang = 0.48
    hx, hy = _dog_top(ax, 2.35, 1.85, ang, 0.85)
    _arrow(ax, hx, hy, hx + 1.55 * math.cos(ang), hy + 1.55 * math.sin(ang), color=TERR, lw=2.6)
    _txt(ax, hx + 0.55, hy + 1.15, "正面（人が言う「前」）", 12, TERR)
    _arrow(ax, 5.55, 1.15, 7.15, 1.15, color=NAVY, lw=2)
    _arrow(ax, 5.55, 1.15, 5.55, 2.55, color=NAVY, lw=2)
    _txt(ax, 7.25, 1.15, "地図 x", 11, NAVY)
    _txt(ax, 5.55, 2.72, "地図 y", 11, NAVY, ha="center")
    _txt(ax, 5.55, 3.55, "同じ 0.5 m/s でも、向きを回さないと地図上でずれる", 13, INK)
    _txt(ax, 0.15, 3.55, "上から見た犬", 12, MUTED)
    ax.set_xlim(-0.2, 8.3)
    ax.set_ylim(0.25, 4.0)
    return _save(fig, "heading_world.png")


def trot_timing() -> Path:
    fig, ax = plt.subplots(figsize=(10.4, 4.3), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    names = ["左前", "右前", "左後", "右後"]
    offs = [0.5, 0.0, 0.0, 0.5]
    duty = 0.74
    colors = [TEAL, TERR, TERR, TEAL]
    for i, (name, off, col) in enumerate(zip(names, offs, colors)):
        y = 3.2 - i * 0.7
        ax.add_patch(Rectangle((0, y), 2.0, 0.42, facecolor="#E4E0D8", edgecolor="#D4CFC6"))
        for cyc in (0.0, 1.0):
            t0 = off + cyc
            t1 = t0 + duty
            ax.add_patch(Rectangle((t0, y), min(duty, 2.0 - t0), 0.42, facecolor=col, edgecolor=NAVY, lw=0.6, alpha=0.9))
            if t1 > 2.0:
                ax.add_patch(Rectangle((0, y), t1 - 2.0, 0.42, facecolor=col, edgecolor=NAVY, lw=0.6, alpha=0.9))
        _txt(ax, -0.08, y + 0.21, name, 12, NAVY, ha="right")
    ax.axvline(0.5, color=GOLD, ls="--", lw=1.2)
    _txt(ax, 0.52, 3.85, "対角が組", 11, GOLD)
    _txt(ax, 0.0, -0.25, "0  1歩の始まり", 11, MUTED)
    _txt(ax, 1.0, -0.25, "1  一周", 11, MUTED)
    _txt(ax, 0.15, 4.05, "色がついているあいだ＝地面について押してよい（立脚）", 12, INK)
    ax.set_xlim(-0.9, 2.15)
    ax.set_ylim(-0.5, 4.35)
    ax.axis("off")
    return _save(fig, "trot_timing.png")


def foothold_side() -> Path:
    fig, ax = _ax(10.4, 4.5)
    _ground(ax, -0.2, 7.4)
    com, feet = _dog_side(ax, 1.7, 0.0, 1.05, swing_front=True, swing_rear=False)
    hip = (1.7 + 0.65, 0.95)
    land = (3.55, 0.04)
    ax.plot(hip[0], hip[1], "o", color=NAVY, ms=8)
    ax.plot(land[0], land[1], "D", color=TERR, ms=10)
    ax.plot([hip[0], land[0]], [hip[1], land[1]], color=TERR, ls="--", lw=1.6)
    _arrow(ax, 1.9, 1.55, 3.3, 1.55, color=TEAL, lw=2.2)
    _txt(ax, 2.4, 1.78, "進む速さ v", 12, TEAL)
    _txt(ax, hip[0] - 0.15, hip[1] + 0.28, "腰", 12, NAVY)
    _txt(ax, land[0] - 0.15, land[1] + 0.38, "次に置く点", 12, TERR)
    _txt(ax, 4.5, 2.4, "リズムは「いつ上げるか」だけ。", 12, INK)
    _txt(ax, 4.5, 2.0, "場所は、腰の下＋進む先。", 12, INK)
    _txt(ax, 4.5, 1.55, "地形を見ない標準では、", 12, INK)
    _txt(ax, 4.5, 1.18, "届かなくても目標のまま。", 12, INK)
    ax.set_xlim(-0.3, 7.6)
    ax.set_ylim(-0.3, 2.8)
    return _save(fig, "foothold.png")


def _srbd_free_body(ax, com, stance, swing, show_wrench=True):
    """Centroidal prediction: point mass + isolated contacts. No body box, no links."""
    cx, cy = com
    ax.add_patch(Circle((cx, cy), 0.20, fill=False, ls="--", edgecolor=GOLD, lw=1.3, alpha=0.85, zorder=3))
    ax.plot(cx, cy, "o", color=GOLD, ms=13, zorder=6)
    _arrow(ax, cx, cy, cx + 0.55, cy, color=NAVY, lw=1.5)
    _arrow(ax, cx, cy, cx, cy + 0.42, color=TEAL, lw=1.5)
    if show_wrench:
        _arrow(ax, cx, cy, cx, cy - 0.62, color=MUTED, lw=1.8)
        _arrow(ax, cx, cy, cx, cy + 0.62, color=GOLD, lw=1.8)
        ax.add_patch(
            FancyArrowPatch(
                (cx + 0.28, cy + 0.10),
                (cx + 0.10, cy + 0.28),
                connectionstyle="arc3,rad=0.7",
                arrowstyle="-|>",
                mutation_scale=12,
                color=GOLD,
                lw=1.5,
            )
        )
    for x, y in stance:
        ax.plot(x, y, "s", color=TERR, ms=9, zorder=5)
        _arrow(ax, x, y + 0.04, x, y + 0.78, color=TERR, lw=2.2)
    for x, y in swing:
        ax.plot(x, y, "s", color=MUTED, ms=8, zorder=5)


def srbd_brick() -> Path:
    fig, ax = _ax(10.2, 4.5)
    _ground(ax, 0.2, 5.35, y=0.15)
    com = (2.85, 1.72)
    stance = [(1.55, 0.20), (4.15, 0.20)]
    swing = [(1.95, 0.62), (3.75, 0.62)]
    _srbd_free_body(ax, com, stance, swing)
    _txt(ax, com[0] + 0.22, com[1] + 0.08, "m, I", 12, GOLD)
    _txt(ax, com[0] + 0.58, com[1] + 0.02, "向き", 10, NAVY)
    _txt(ax, com[0] + 0.08, com[1] + 0.72, "Σ cF", 11, GOLD)
    _txt(ax, com[0] + 0.08, com[1] - 0.72, "mg", 11, MUTED)
    _txt(ax, com[0] + 0.42, com[1] + 0.32, "Σ r×F", 11, GOLD)
    _txt(ax, 1.55, 1.05, "cF", 11, TERR, ha="center")
    _txt(ax, 4.15, 1.05, "cF", 11, TERR, ha="center")
    _txt(ax, 1.55, -0.08, "押す", 10, MUTED, ha="center")
    _txt(ax, 4.15, -0.08, "押す", 10, MUTED, ha="center")
    _txt(ax, 1.95, 0.42, "空中", 10, MUTED, ha="center")
    _txt(ax, 3.75, 0.42, "空中", 10, MUTED, ha="center")
    _txt(ax, 5.55, 2.85, "形は無い。質量と慣性だけ。", 12, INK)
    _txt(ax, 5.55, 2.40, "足と重心を結ぶ棒は無い。", 12, INK)
    _txt(ax, 5.55, 1.95, "力は接地点で、世界の向きに働く。", 12, INK)
    _txt(ax, 5.55, 1.50, "重心へは合力とモーメント。", 12, INK)
    _txt(ax, 5.55, 1.05, "リンク方向には伝わらない。", 12, TERR)
    ax.set_xlim(0.1, 8.4)
    ax.set_ylim(-0.28, 3.2)
    return _save(fig, "srbd.png")


def receding() -> Path:
    fig, ax = plt.subplots(figsize=(10.4, 3.6), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.axis("off")
    ax.annotate("", xy=(9.4, 1.3), xytext=(0.4, 1.3), arrowprops=dict(arrowstyle="-|>", color=NAVY, lw=1.8))
    _txt(ax, 9.5, 1.3, "時間", 11, NAVY)
    for i, t in enumerate([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]):
        x = 0.7 + i * 0.72
        ax.add_patch(Rectangle((x, 1.05), 0.62, 0.5, facecolor="#D7E6E6" if i else TERR, edgecolor=NAVY, lw=0.8, alpha=0.95 if i == 0 else 0.7))
        _txt(ax, x + 0.31, 1.3, str(i), 10, "white" if i == 0 else NAVY, ha="center")
    _txt(ax, 1.0, 2.15, "今使うのはこの1コマだけ", 13, TERR)
    _txt(ax, 4.6, 0.55, "残り11コマは、次の瞬間に捨てて解き直す", 12, MUTED)
    _txt(ax, 0.4, 2.85, "未来 0.24 秒（12コマ × 0.02 秒）", 13, INK)
    ax.set_xlim(0.2, 10.0)
    ax.set_ylim(0.2, 3.2)
    return _save(fig, "receding.png")


def one_leg_torque() -> Path:
    fig, ax = _ax(10.0, 4.6)
    _ground(ax, 0.3, 5.2)
    hip, knee, foot = (2.2, 2.15), (2.55, 1.15), (2.35, 0.08)
    ax.plot([hip[0], knee[0], foot[0]], [hip[1], knee[1], foot[1]], color=NAVY, lw=5, solid_capstyle="round")
    for p in (hip, knee, foot):
        ax.plot(*p, "o", color=TEAL, ms=10)
    _arrow(ax, foot[0], foot[1] + 0.05, foot[0], foot[1] + 1.05, color=TERR, lw=2.4)
    _txt(ax, foot[0] + 0.15, foot[1] + 0.7, "押す力 F", 13, TERR)
    # torque curls
    for (x, y), name in ((hip, "τ1"), (knee, "τ2"), ((2.4, 0.55), "τ3")):
        ax.add_patch(FancyArrowPatch((x + 0.22, y + 0.12), (x + 0.22, y - 0.18), connectionstyle="arc3,rad=0.8", arrowstyle="-|>", mutation_scale=12, color=GOLD, lw=1.8))
        _txt(ax, x + 0.38, y, name, 12, GOLD)
    _txt(ax, 4.3, 2.6, "モータは足先には無い。", 13, INK)
    _txt(ax, 4.3, 2.15, "足先の力を、3つの回す力へ写す。", 13, INK)
    _txt(ax, 4.3, 1.65, "空中の足は、この式を使わず", 12, INK)
    _txt(ax, 4.3, 1.25, "次の点へ運ぶ力で上書きする。", 12, INK)
    ax.set_xlim(0.2, 8.0)
    ax.set_ylim(-0.2, 3.1)
    return _save(fig, "leg_tau.png")


def closed_loop_scene() -> Path:
    fig, ax = _ax(10.6, 4.6)
    _ground(ax, -0.2, 10.2)
    _dog_side(ax, 3.3, 0.0, 1.0, swing_front=True, swing_rear=False)
    # ghost next
    ax.plot([6.4, 7.1, 6.55], [0.95, 0.55, 0.05], color="#B8C4C4", lw=3)
    ax.add_patch(FancyBboxPatch((5.85, 0.82), 1.7, 0.32, boxstyle="round,pad=0.02", facecolor="#D7E0E0", edgecolor="#8AA0A0", lw=1, alpha=0.8))
    _txt(ax, 6.7, 1.55, "次の体", 12, MUTED, ha="center")
    _txt(ax, 0.15, 2.55, "人", 13, NAVY)
    _arrow(ax, 0.55, 2.15, 1.7, 1.4, color=TEAL, lw=2)
    _txt(ax, 0.15, 1.85, "「前へ 0.5」", 12, TEAL)
    _arrow(ax, 4.5, 1.1, 5.7, 1.1, color=NAVY, lw=2)
    _txt(ax, 4.7, 1.35, "回す力", 11, NAVY)
    _txt(ax, 0.15, 0.55, "地面", 11, MUTED)
    _txt(ax, 3.3, 2.55, "今の犬", 13, NAVY, ha="center")
    ax.set_xlim(-0.3, 8.6)
    ax.set_ylim(-0.3, 3.0)
    return _save(fig, "closed_loop.png")


def three_forces_foot() -> Path:
    fig, ax = _ax(10.2, 4.3)
    _ground(ax, 0.4, 9.4)
    ax.plot(2.0, 0.08, "s", color=NAVY, ms=14)
    _arrow(ax, 2.0, 0.15, 2.0, 1.55, color=GOLD, lw=2.2)
    _txt(ax, 2.15, 1.7, "計算が出した力", 12, GOLD)
    _arrow(ax, 4.6, 0.15, 4.6, 1.55, color=TEAL, lw=2.2)
    _txt(ax, 4.75, 1.7, "実行が使う力", 12, TEAL)
    _arrow(ax, 7.2, 0.15, 7.2, 1.55, color=TERR, lw=2.2)
    _txt(ax, 7.35, 1.7, "地面が返した力", 12, TERR)
    _txt(ax, 2.0, -0.25, "空中でも残り得る", 10, MUTED, ha="center")
    _txt(ax, 4.6, -0.25, "空中は 0 にする", 10, MUTED, ha="center")
    _txt(ax, 7.2, -0.25, "切替には使わない", 10, MUTED, ha="center")
    _txt(ax, 0.4, 2.55, "同じ「地面の力」でも、3つは別物", 14, NAVY)
    ax.set_xlim(0.2, 9.6)
    ax.set_ylim(-0.55, 2.9)
    return _save(fig, "three_F.png")


def clocks() -> Path:
    fig, ax = plt.subplots(figsize=(10.4, 3.4), dpi=140)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.axis("off")
    ax.annotate("", xy=(9.6, 2.15), xytext=(0.4, 2.15), arrowprops=dict(arrowstyle="-|>", color=NAVY, lw=1.6))
    for i in range(0, 26):
        x = 0.5 + i * 0.34
        h = 0.55 if i % 5 == 0 else 0.22
        ax.plot([x, x], [2.15, 2.15 + h], color=TEAL if i % 5 == 0 else MUTED, lw=1.6 if i % 5 == 0 else 0.8)
    _txt(ax, 0.5, 3.05, "速い時計  秒間500回   足のリズム・回す力・物体を進める", 12, TEAL)
    _txt(ax, 0.5, 1.35, "遅い時計  秒間100回   未来の押し方を解き直す（太い目盛り）", 12, NAVY)
    _txt(ax, 0.5, 0.7, "間の4回は、直前の押し方を使い続ける", 12, MUTED)
    ax.set_xlim(0.2, 10.0)
    ax.set_ylim(0.3, 3.3)
    return _save(fig, "clocks.png")


def plant_vs_srbd() -> Path:
    fig, ax = _ax(10.6, 4.6)
    _ground(ax, -0.2, 4.55)
    _dog_side(ax, 2.15, 0.0, 0.95, swing_front=True, swing_rear=False)
    _txt(ax, 2.15, 2.55, "物体（関節12個がある）", 13, NAVY, ha="center")
    _ground(ax, 5.45, 10.05)
    com = (7.65, 1.55)
    _srbd_free_body(ax, com, [(6.55, 0.14), (8.75, 0.14)], [(6.95, 0.52), (8.35, 0.52)], show_wrench=False)
    _txt(ax, com[0] + 0.18, com[1] + 0.06, "m, I", 11, GOLD)
    _txt(ax, 7.65, 2.55, "予測の中の体（形もリンクも無い）", 13, NAVY, ha="center")
    _arrow(ax, 4.35, 1.15, 5.55, 1.15, color=GOLD, lw=2)
    _txt(ax, 4.55, 1.45, "粗くする", 11, GOLD)
    ax.set_xlim(-0.3, 10.3)
    ax.set_ylim(-0.3, 3.0)
    return _save(fig, "plant_vs_srbd.png")


def friction_cone() -> Path:
    fig, ax = _ax(10.0, 4.4)
    _ground(ax, 0.3, 5.4)
    ax.plot(2.4, 0.08, "s", color=NAVY, ms=12)
    _arrow(ax, 2.4, 0.1, 2.4, 1.55, color=TERR, lw=2.4)
    _arrow(ax, 2.4, 0.1, 3.35, 1.15, color=TEAL, lw=1.8)
    _arrow(ax, 2.4, 0.1, 1.45, 1.15, color=TEAL, lw=1.8)
    ax.plot([1.45, 2.4, 3.35], [1.15, 0.1, 1.15], color=TEAL, ls="--", lw=1.2)
    _txt(ax, 2.55, 1.65, "垂直の力", 12, TERR)
    _txt(ax, 3.45, 1.25, "横へ滑る力", 12, TEAL)
    _txt(ax, 5.6, 2.45, "円すいの内側なら、足は滑らない。", 13, INK)
    _txt(ax, 5.6, 1.95, "計算は、4脚すべてにこの制限を置く。", 12, INK)
    _txt(ax, 5.6, 1.50, "空中の足も、式の上では残る。", 12, INK)
    _txt(ax, 5.6, 1.05, "実行へ渡すとき、空中は 0 にする。", 12, INK)
    ax.set_xlim(0.2, 9.6)
    ax.set_ylim(-0.25, 3.0)
    return _save(fig, "friction.png")


def gait_mask_dog() -> Path:
    fig, ax = _ax(10.4, 4.5)
    _ground(ax, -0.2, 7.4)
    com, feet = _dog_side(ax, 2.3, 0.0, 1.1, swing_front=True, swing_rear=False)
    for (fx, fy), down in feet:
        if down:
            _arrow(ax, fx, fy + 0.02, fx, fy + 0.78, color=TERR, lw=2.4)
            _txt(ax, fx + 0.08, fy + 0.62, "c=1  押す", 11, TERR)
        else:
            _txt(ax, fx + 0.05, fy + 0.18, "c=0  力を捨てる", 11, MUTED)
    _txt(ax, 4.9, 2.45, "計算は4足ぶんの力を出す。", 12, INK)
    _txt(ax, 4.9, 2.05, "今地面の足だけ、先頭の1コマを残す。", 12, INK)
    _txt(ax, 4.9, 1.60, "空中の足に残すと、足が地面を探す。", 12, INK)
    _txt(ax, com[0] + 0.15, com[1] + 0.28, "重心", 11, GOLD)
    ax.set_xlim(-0.3, 7.8)
    ax.set_ylim(-0.3, 2.9)
    return _save(fig, "gait_mask.png")


def stride_vfl() -> Path:
    fig, ax = _ax(10.4, 4.2)
    _ground(ax, -0.2, 8.2)
    _dog_side(ax, 1.6, 0.0, 0.85)
    _dog_side(ax, 5.05, 0.0, 0.85)
    _arrow(ax, 1.6, 2.15, 5.05, 2.15, color=TEAL, lw=2.2)
    _txt(ax, 3.2, 2.42, "1歩で進む長さ L", 13, TEAL, ha="center")
    _txt(ax, 0.1, 3.15, "速さ ＝ 歩数 × 1歩の長さ。周波数の候補は、速さを決めない。", 13, INK)
    _txt(ax, 0.1, 2.72, "標準では歩数は固定。候補を出す枝はオフ。", 12, MUTED)
    ax.set_xlim(-0.3, 8.4)
    ax.set_ylim(-0.3, 3.45)
    return _save(fig, "stride.png")


def all_figures() -> dict[str, Path]:
    return {
        "plant": dog_and_plant_eq(),
        "heading": heading_vs_world(),
        "trot": trot_timing(),
        "foothold": foothold_side(),
        "srbd": srbd_brick(),
        "receding": receding(),
        "leg": one_leg_torque(),
        "loop": closed_loop_scene(),
        "three": three_forces_foot(),
        "clocks": clocks(),
        "two_bodies": plant_vs_srbd(),
        "friction": friction_cone(),
        "mask_dog": gait_mask_dog(),
        "stride": stride_vfl(),
    }
