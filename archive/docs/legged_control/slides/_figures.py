# -*- coding: utf-8 -*-
"""A1 kinematic drawings and architecture figures for teaching slides.

Lengths from a1/const.xacro. Standing joints from reference.info.
Figures are pictures of the dog, the math on the dog, or the software
architecture — not a row of numbered boxes.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, Polygon

OUT = Path(__file__).resolve().parent / "_eqcache"
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

NAVY = "#1B3A4B"
TEAL = "#2A6F6F"
TERR = "#C45A20"
GOLD = "#B48A28"
INK = "#1E1E24"
MUTED = "#5A606E"
PAPER = "#F7F5F0"
LINE = "#D4CFC6"
WHITE = "#FFFFFF"

# a1 / const.xacro
TRUNK = np.array([0.267, 0.194, 0.114])
LEG_OFF = np.array([0.1805, 0.047, 0.0])
THIGH_LAT = 0.0838
THIGH_L = 0.20
CALF_L = 0.20
COM_H = 0.30

# reference.info defaultJointState
Q0 = {
    "LF": (-0.20, 0.72, -1.44),
    "LH": (-0.20, 0.72, -1.44),
    "RF": (0.20, 0.72, -1.44),
    "RH": (0.20, 0.72, -1.44),
}
SIGN = {"LF": (1, 1), "LH": (-1, 1), "RF": (1, -1), "RH": (-1, -1)}
ORDER = ("LF", "RF", "LH", "RH")  # contactNames comment: LF RF LH RH


def _fp(size=11, weight="regular"):
    return font_manager.FontProperties(fname=FONT_PATH, size=size, weight=weight)


def _setup_font():
    try:
        font_manager.fontManager.addfont(FONT_PATH)
    except Exception:
        pass
    plt.rcParams["font.family"] = "Noto Sans CJK JP"
    plt.rcParams["axes.unicode_minus"] = False


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _leg_chain(sx, sy, haa, hfe, kfe):
    hip = np.array([sx * LEG_OFF[0], sy * LEG_OFF[1], 0.0])
    Rhaa = Rx(haa)
    hip2 = hip + Rhaa @ np.array([0.0, sy * THIGH_LAT, 0.0])
    knee = hip2 + Rhaa @ (Ry(hfe) @ np.array([0.0, 0.0, -THIGH_L]))
    foot = knee + Rhaa @ (Ry(hfe + kfe) @ np.array([0.0, 0.0, -CALF_L]))
    return hip, hip2, knee, foot


def pose_points(p_b=None, zyx=None, joints=None, clearance=0.0):
    """World points of trunk corners, hips, knees, feet. zyx = (yaw, pitch, roll)."""
    if p_b is None:
        p_b = np.zeros(3)
    if zyx is None:
        zyx = (0.0, 0.0, 0.0)
    if joints is None:
        joints = Q0
    yaw, pitch, roll = zyx
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    chains = {}
    feet_z = []
    for name in ORDER:
        sx, sy = SIGN[name]
        haa, hfe, kfe = joints[name]
        hip, hip2, knee, foot = _leg_chain(sx, sy, haa, hfe, kfe)
        chains[name] = {
            "hip": R @ hip,
            "hip2": R @ hip2,
            "knee": R @ knee,
            "foot": R @ foot,
        }
        feet_z.append((R @ foot)[2])
    # lift so the lowest foot sits on z=0, then add p_b and optional clearance
    lift = -float(np.min(feet_z))
    origin = np.array([p_b[0], p_b[1], p_b[2] + lift + float(clearance)])
    for name in chains:
        for k in chains[name]:
            chains[name][k] = chains[name][k] + origin
    hx, hy, hz = TRUNK / 2
    corners = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                corners.append(origin + R @ np.array([dx, dy, dz]))
    com = origin + R @ np.array([0.0, 0.0, 0.0])
    return {"chains": chains, "corners": np.array(corners), "com": com, "R": R, "origin": origin}


def _proj_iso(p):
    return np.array([p[0] - 0.62 * p[1], p[2] + 0.32 * p[1]])


def _proj_side(p):
    return np.array([p[0], p[2]])


def _proj_top(p):
    return np.array([p[0], p[1]])


def _new(w=11.6, h=5.15):
    _setup_font()
    fig, ax = plt.subplots(figsize=(w, h), dpi=150)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def _save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=PAPER, pad_inches=0.08)
    plt.close(fig)
    return path


def _ground(ax, proj, x0=-0.55, x1=0.85, yspan=0.28):
    if proj is _proj_iso:
        p = [
            _proj_iso(np.array([x0, -yspan, 0])),
            _proj_iso(np.array([x1, -yspan, 0])),
            _proj_iso(np.array([x1, yspan, 0])),
            _proj_iso(np.array([x0, yspan, 0])),
        ]
        ax.add_patch(Polygon(p, closed=True, facecolor="#E8E4DA", edgecolor=LINE, lw=0.8, zorder=0))
    else:
        ax.plot([x0, x1], [0, 0], color=LINE, lw=1.4, zorder=0)


def _draw_dog(ax, pose, proj=_proj_iso, stance=None, fade=1.0, lw=2.4, show_joints=True):
    """Draw one A1. stance: dict name->bool. None = all down."""
    if stance is None:
        stance = {n: True for n in ORDER}
    # trunk: draw 3 visible faces
    c = pose["corners"]
    # indices: x-,x+ × y-,y+ × z-,z+  flattened as nested loops
    # 0: -x-y-z  1: -x-y+z  2: -x+y-z  3: -x+y+z
    # 4: +x-y-z  5: +x-y+z  6: +x+y-z  7: +x+y+z
    faces = (
        (1, 5, 7, 3),  # top
        (4, 5, 7, 6),  # front (+x)
        (2, 3, 7, 6),  # left (+y)
    )
    face_col = (f"#2A6F6F{int(50*fade):02x}", f"#1B3A4B{int(70*fade):02x}", f"#2A6F6F{int(35*fade):02x}")
    for idxs, col in zip(faces, face_col):
        pts = [proj(c[i]) for i in idxs]
        ax.add_patch(Polygon(pts, closed=True, facecolor=col, edgecolor=NAVY, lw=0.9, zorder=2, alpha=fade))

    left = {"LF", "LH"}
    for name in ORDER:
        ch = pose["chains"][name]
        pts = [proj(ch[k]) for k in ("hip", "hip2", "knee", "foot")]
        color = TEAL if name in left else TERR
        ax.plot(
            [p[0] for p in pts],
            [p[1] for p in pts],
            color=color,
            lw=lw,
            solid_capstyle="round",
            alpha=fade,
            zorder=3,
        )
        foot = proj(ch["foot"])
        on = stance[name]
        ax.add_patch(
            Circle(foot, 0.018 if on else 0.014, facecolor=GOLD if on else WHITE, edgecolor=color, lw=1.2, alpha=fade, zorder=4)
        )
        if show_joints and fade > 0.7:
            for k in ("hip2", "knee"):
                j = proj(ch[k])
                ax.add_patch(Circle(j, 0.012, facecolor=WHITE, edgecolor=color, lw=1.0, alpha=fade, zorder=4))
    com = proj(pose["com"])
    ax.plot(com[0], com[1], "o", color=GOLD, ms=7, alpha=fade, zorder=5)
    return pose


def _label(ax, xy, text, color=INK, size=10, ha="left", va="center", weight="regular"):
    ax.text(xy[0], xy[1], text, fontproperties=_fp(size, weight), color=color, ha=ha, va=va, zorder=8)


def _arrow(ax, p0, p1, color=TERR, lw=1.6):
    ax.annotate(
        "",
        xy=p1,
        xytext=p0,
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw),
        zorder=6,
    )


def fig_dog_model() -> Path:
    fig, ax = _new()
    pose = pose_points()
    _ground(ax, _proj_iso)
    _draw_dog(ax, pose)
    com = _proj_iso(pose["com"])
    lf = _proj_iso(pose["chains"]["LF"]["foot"])
    rf = _proj_iso(pose["chains"]["RF"]["foot"])
    lh = _proj_iso(pose["chains"]["LH"]["foot"])
    rh = _proj_iso(pose["chains"]["RH"]["foot"])
    hip = _proj_iso(pose["chains"]["LF"]["hip2"])
    knee = _proj_iso(pose["chains"]["LF"]["knee"])
    _label(ax, com + np.array([0.06, 0.10]), "胴体  CoM", GOLD, 12, weight="bold")
    _label(ax, com + np.array([0.06, 0.04]), "高さ 0.3 m（a1）", MUTED, 9)
    _label(ax, lf + np.array([0.04, -0.05]), "LF", TEAL, 11, weight="bold")
    _label(ax, rf + np.array([0.04, -0.05]), "RF", TERR, 11, weight="bold")
    _label(ax, lh + np.array([-0.10, -0.05]), "LH", TEAL, 11, weight="bold")
    _label(ax, rh + np.array([-0.10, -0.05]), "RH", TERR, 11, weight="bold")
    _label(ax, hip + np.array([0.04, 0.03]), "HAA", MUTED, 9)
    _label(ax, knee + np.array([0.04, 0.02]), "HFE / KFE", MUTED, 9)
    nose = _proj_iso(pose["origin"] + pose["R"] @ np.array([0.18, 0, 0]))
    _arrow(ax, com, nose, TEAL, 1.8)
    _label(ax, nose + np.array([0.03, 0.03]), "前（+x）", TEAL, 10)
    _label(ax, np.array([0.55, 0.42]), "Unitree A1", NAVY, 14, weight="bold")
    _label(ax, np.array([0.55, 0.36]), "浮動 6  +  関節 12  +  足 4", MUTED, 10)
    _label(ax, np.array([0.55, 0.30]), "各脚  HAA（開閉）HFE（腿）KFE（膝）", MUTED, 10)
    ax.set_xlim(-0.75, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_dog_model")


def fig_state_x() -> Path:
    fig, ax = _new()
    pose = pose_points()
    _ground(ax, _proj_iso)
    _draw_dog(ax, pose)
    com = _proj_iso(pose["com"])
    _arrow(ax, com, com + np.array([0.28, 0.0]), TEAL, 2.0)
    _label(ax, com + np.array([0.30, 0.02]), r"$v_{\mathrm{com}}$  3", TEAL, 11, weight="bold")
    _arrow(ax, com + np.array([-0.02, 0.04]), com + np.array([-0.02, 0.18]), GOLD, 1.6)
    _label(ax, com + np.array([0.02, 0.20]), r"$L/m$  3", GOLD, 11, weight="bold")
    _label(ax, com + np.array([-0.42, 0.08]), r"$p_b$  3" + "\n向き 3", NAVY, 11, weight="bold")
    lf_hip = _proj_iso(pose["chains"]["LF"]["hip2"])
    _label(ax, lf_hip + np.array([0.08, 0.08]), r"$q_j$  12", TERR, 12, weight="bold")
    _label(ax, np.array([0.48, 0.42]), r"$x\in\mathbb{R}^{24}$", NAVY, 14, weight="bold")
    _label(ax, np.array([0.48, 0.35]), "勢い 6  +  位置向き 6  +  関節 12", MUTED, 10)
    _label(ax, np.array([0.48, 0.28]), "「胴体12+関節12」ではない", TERR, 10)
    ax.set_xlim(-0.75, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_state_x")


def fig_input_u(stance=None, name="fig_input_u") -> Path:
    fig, ax = _new()
    pose = pose_points()
    if stance is None:
        stance = {n: True for n in ORDER}
    _ground(ax, _proj_iso)
    _draw_dog(ax, pose, stance=stance)
    for n, on in stance.items():
        foot = _proj_iso(pose["chains"][n]["foot"])
        if on:
            _arrow(ax, foot, foot + np.array([0.0, 0.16]), GOLD, 1.8)
            _label(ax, foot + np.array([0.03, 0.18]), r"$f_c$", GOLD, 10)
        else:
            _label(ax, foot + np.array([0.02, 0.06]), "0", MUTED, 9)
    knee = _proj_iso(pose["chains"]["LF"]["knee"])
    _label(ax, knee + np.array([0.08, 0.00]), r"$v_j$  12", TERR, 11, weight="bold")
    _label(ax, np.array([0.42, 0.42]), r"$u\in\mathbb{R}^{24}$", NAVY, 14, weight="bold")
    _label(ax, np.array([0.42, 0.35]), "地面反力 12  +  関節速さ 12", MUTED, 10)
    _label(ax, np.array([0.42, 0.28]), "トルクは u に無い", TERR, 10)
    ax.set_xlim(-0.75, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, name)


def fig_cmd_frames() -> Path:
    fig, ax = _new()
    yaw = 0.45
    pose = pose_points(zyx=(yaw, 0.0, 0.0))
    _ground(ax, _proj_iso, x0=-0.4, x1=0.95)
    _draw_dog(ax, pose)
    com = _proj_iso(pose["com"])
    # body +x
    bdir = _proj_iso(pose["com"] + pose["R"] @ np.array([0.28, 0, 0]))
    _arrow(ax, com, bdir, TEAL, 2.0)
    _label(ax, bdir + np.array([0.02, 0.04]), "鼻先  $v_x=0.5$", TEAL, 11, weight="bold")
    # world +x
    wdir = _proj_iso(pose["com"] + np.array([0.32, 0, 0]))
    _arrow(ax, com, wdir, NAVY, 2.0)
    _label(ax, wdir + np.array([0.02, -0.05]), "地図 $x$", NAVY, 11, weight="bold")
    _label(ax, np.array([0.40, 0.44]), r"$v_W=R(\psi,\theta,\phi)\,v_{\mathrm{cmd}}$", NAVY, 13, weight="bold")
    _label(ax, np.array([0.40, 0.37]), "人の「前」を、今の向きで地図へ回す", MUTED, 10)
    _label(ax, np.array([0.40, 0.31]), r"旋回 $\dot\psi$ は回さない", MUTED, 10)
    ax.set_xlim(-0.70, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_cmd_frames")


def fig_two_points() -> Path:
    fig, ax = _new(w=11.6, h=5.15)
    pose0 = pose_points(p_b=np.array([0.0, 0.0, 0.0]))
    pose1 = pose_points(p_b=np.array([0.50, 0.0, 0.0]))
    _ground(ax, _proj_iso, x0=-0.45, x1=1.15, yspan=0.22)
    _draw_dog(ax, pose0, fade=1.0)
    _draw_dog(ax, pose1, fade=0.45, show_joints=False)
    c0 = _proj_iso(pose0["com"])
    c1 = _proj_iso(pose1["com"])
    ax.plot([c0[0], c1[0]], [c0[1], c1[1]], color=GOLD, lw=1.6, ls="--", zorder=1)
    _arrow(ax, c0, c0 + np.array([0.22, 0]), TEAL, 1.8)
    _arrow(ax, c1, c1 + np.array([0.22, 0]), TEAL, 1.8)
    _label(ax, c0 + np.array([-0.08, 0.16]), "今  点0", NAVY, 12, weight="bold")
    _label(ax, c1 + np.array([-0.02, 0.16]), "今+1 s  点1", TEAL, 12, weight="bold")
    _label(ax, (c0 + c1) / 2 + np.array([0.0, 0.10]), "0.5 m（直線）", GOLD, 10)
    _label(ax, np.array([0.70, 0.44]), "参照は 2 点だけ", NAVY, 13, weight="bold")
    _label(ax, np.array([0.70, 0.37]), "速さは両端とも 0.5 m/s", MUTED, 10)
    _label(ax, np.array([0.70, 0.31]), "足軌道はここには無い", MUTED, 10)
    ax.set_xlim(-0.70, 1.40)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_two_points")


def fig_kalman() -> Path:
    fig, ax = _new()
    pose = pose_points()
    _ground(ax, _proj_iso)
    _draw_dog(ax, pose)
    com = _proj_iso(pose["com"])
    # IMU
    imu = com + np.array([0.0, 0.06])
    ax.add_patch(Rectangle((imu[0] - 0.03, imu[1] - 0.02), 0.06, 0.04, facecolor=NAVY, zorder=6))
    _label(ax, imu + np.array([0.08, 0.01]), "IMU  向き・加速度", NAVY, 10, weight="bold")
    _arrow(ax, com, com + np.array([0.0, -0.12]), GOLD, 1.4)
    _label(ax, com + np.array([0.04, -0.10]), r"$a_W$", GOLD, 10)
    for n in ("LF", "RH"):
        f = _proj_iso(pose["chains"][n]["foot"])
        _arrow(ax, com, f, TEAL, 1.1)
    _label(ax, _proj_iso(pose["chains"]["LF"]["foot"]) + np.array([0.03, 0.10]), r"$p_b-p_f\approx p_s$", TEAL, 11, weight="bold")
    _label(ax, np.array([0.42, 0.44]), "測るもの / 推定するもの", NAVY, 13, weight="bold")
    _label(ax, np.array([0.42, 0.37]), "向き・関節はセンサのまま", MUTED, 10)
    _label(ax, np.array([0.42, 0.31]), "地図の位置・速さだけ Kalman", MUTED, 10)
    _label(ax, np.array([0.42, 0.25]), "空中の足は信用しない", TERR, 10)
    ax.set_xlim(-0.75, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_kalman")


def fig_gait_types() -> Path:
    """stance / trot / flying_trot as dogs. flying の列は本repo未照合。"""
    fig, axes = plt.subplots(1, 3, figsize=(11.6, 5.15), dpi=150)
    fig.patch.set_facecolor(PAPER)
    panels = (
        ({n: True for n in ORDER}, 0.0, "stance", "四脚接地。初期。", TEAL),
        ({"LF": True, "RH": True, "RF": False, "LH": False}, 0.0, "trot", "対角が組。0.6 s で交代。", TERR),
        ({n: False for n in ORDER}, 0.08, "flying_trot", "空中区間がある名前。列は gait.info。", GOLD),
    )
    for ax, (stance, clearance, title, sub, col) in zip(axes, panels):
        ax.set_facecolor(PAPER)
        ax.set_aspect("equal")
        ax.axis("off")
        pose = pose_points(clearance=clearance)
        _ground(ax, _proj_iso)
        _draw_dog(ax, pose, stance=stance)
        _label(ax, np.array([-0.18, 0.46]), title, col, 14, weight="bold")
        _label(ax, np.array([-0.18, 0.39]), sub, MUTED, 9)
        ax.set_xlim(-0.65, 0.65)
        ax.set_ylim(-0.12, 0.52)
    fig.text(
        0.5,
        0.03,
        "人が端末で名前を選ぶ。速さ指令は歩容を切り替えない。NMPC も WBC も種類を選ばない。",
        ha="center",
        fontproperties=_fp(11),
        color=MUTED,
    )
    return _save(fig, "fig_gait_types")


def fig_gait_binds() -> Path:
    """Same contact flags: gait name → NMPC forces → WBC torque. A scene, not boxes."""
    fig, ax = _new(w=11.6, h=5.15)
    stance = {"LF": True, "RH": True, "RF": False, "LH": False}
    pose = pose_points()
    _ground(ax, _proj_iso, x0=-0.35, x1=0.55)
    _draw_dog(ax, pose, stance=stance)
    com = _proj_iso(pose["com"])
    _label(ax, np.array([-0.72, 0.46]), "人の2つ", TEAL, 12, weight="bold")
    _label(ax, np.array([-0.72, 0.38]), "速さ  「前へ 0.5」", MUTED, 10)
    _label(ax, np.array([-0.72, 0.32]), "歩容  「trot」", TERR, 10)
    _arrow(ax, np.array([-0.48, 0.28]), np.array([-0.16, 0.20]), TEAL, 1.4)
    for n, on in stance.items():
        f = _proj_iso(pose["chains"][n]["foot"])
        if on:
            _arrow(ax, f, f + np.array([0.0, 0.14]), GOLD, 1.6)
            _label(ax, f + np.array([0.02, 0.16]), r"$c=1$", GOLD, 9)
        else:
            _label(ax, f + np.array([0.02, 0.07]), r"$c=0$", MUTED, 9)
    _label(ax, np.array([0.40, 0.46]), "NMPC", NAVY, 12, weight="bold")
    _label(ax, np.array([0.40, 0.39]), "立脚だけ力を許す", MUTED, 10)
    _label(ax, np.array([0.40, 0.33]), "遊脚は $f_c=0$", MUTED, 10)
    _label(ax, np.array([0.40, 0.16]), "WBC", TERR, 12, weight="bold")
    _label(ax, np.array([0.40, 0.09]), "同じ $c$ で押す足を切る", MUTED, 10)
    _label(ax, np.array([0.40, 0.03]), "出口は $\\tau$ 12", TERR, 10)
    k = _proj_iso(pose["chains"]["LF"]["knee"])
    ax.add_patch(Circle(k, 0.020, facecolor=TERR, alpha=0.85, zorder=6))
    _label(ax, k + np.array([0.04, 0.00]), r"$\tau$", TERR, 11, weight="bold")
    _label(ax, np.array([-0.12, 0.48]), "同じ接地旗 $c$ が、計画と実行に入る", NAVY, 12, weight="bold")
    ax.set_xlim(-0.85, 0.95)
    ax.set_ylim(-0.12, 0.54)
    return _save(fig, "fig_gait_binds")


def fig_gait_trot() -> Path:
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 5.15), dpi=150)
    fig.patch.set_facecolor(PAPER)
    phases = (
        ({"LF": True, "RH": True, "RF": False, "LH": False}, "0.0–0.3 s   LF + RH", TEAL),
        ({"RF": True, "LH": True, "LF": False, "RH": False}, "0.3–0.6 s   RF + LH", TERR),
    )
    for ax, (stance, title, col) in zip(axes, phases):
        ax.set_facecolor(PAPER)
        ax.set_aspect("equal")
        ax.axis("off")
        pose = pose_points()
        _ground(ax, _proj_iso)
        _draw_dog(ax, pose, stance=stance)
        _label(ax, np.array([-0.15, 0.44]), title, col, 13, weight="bold")
        _label(ax, np.array([-0.15, 0.38]), "接地は力あり / 空中は力 0", MUTED, 10)
        ax.set_xlim(-0.70, 0.70)
        ax.set_ylim(-0.12, 0.50)
    fig.text(0.5, 0.04, "trot。NMPC はこの時間割を選ばない。人が gait 名で与える。", ha="center", fontproperties=_fp(11), color=MUTED)
    return _save(fig, "fig_gait_trot")


def fig_nmpc_horizon() -> Path:
    fig, ax = _new()
    xs = [0.0, 0.17, 0.33, 0.50]
    for i, x in enumerate(xs):
        fade = 1.0 if i == 0 else 0.28 + 0.12 * i
        pose = pose_points(p_b=np.array([x, 0.0, 0.0]))
        if i == 0:
            _ground(ax, _proj_iso, x0=-0.4, x1=1.05, yspan=0.20)
        _draw_dog(ax, pose, fade=fade, show_joints=(i == 0), lw=2.2 if i == 0 else 1.4)
    c0 = _proj_iso(pose_points()["com"])
    c1 = _proj_iso(pose_points(p_b=np.array([0.50, 0, 0]))["com"])
    ax.plot([c0[0], c1[0]], [c0[1] + 0.12, c1[1] + 0.12], color=NAVY, lw=1.2, ls=":", zorder=1)
    _label(ax, c0 + np.array([-0.06, 0.20]), "今  → WBC へ渡す 1 点", TERR, 11, weight="bold")
    _label(ax, c1 + np.array([-0.10, 0.20]), "未来 1.0 s（約 67 点）", NAVY, 11, weight="bold")
    _label(ax, np.array([0.62, 0.44]), "NMPC は未来を持つ", NAVY, 13, weight="bold")
    _label(ax, np.array([0.62, 0.37]), "WBC は今の切り口だけ", TERR, 10)
    _label(ax, np.array([0.62, 0.31]), "トルクはここでは決めない", MUTED, 10)
    ax.set_xlim(-0.65, 1.35)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_nmpc_horizon")


def fig_wbc_eom() -> Path:
    fig, ax = _new()
    stance = {"LF": True, "RH": True, "RF": False, "LH": False}
    pose = pose_points()
    _ground(ax, _proj_iso)
    _draw_dog(ax, pose, stance=stance)
    com = _proj_iso(pose["com"])
    _arrow(ax, com, com + np.array([0.0, -0.14]), MUTED, 1.6)
    _label(ax, com + np.array([0.04, -0.12]), r"$nle$（重力など）", MUTED, 10)
    for n in ("LF", "RH"):
        f = _proj_iso(pose["chains"][n]["foot"])
        _arrow(ax, f, f + np.array([0.0, 0.15]), GOLD, 1.8)
        _label(ax, f + np.array([0.03, 0.16]), r"$F_c$", GOLD, 10)
    k = _proj_iso(pose["chains"]["LF"]["knee"])
    ax.add_patch(Circle(k, 0.022, facecolor=TERR, alpha=0.85, zorder=6))
    _label(ax, k + np.array([0.05, 0.00]), r"$\tau$", TERR, 12, weight="bold")
    sw = _proj_iso(pose["chains"]["RF"]["foot"])
    _label(ax, sw + np.array([0.03, 0.08]), "遊脚 PD", TEAL, 10)
    _label(ax, np.array([0.40, 0.44]), "今の姿勢で釣り合わせる", NAVY, 13, weight="bold")
    _label(ax, np.array([0.40, 0.37]), r"$M\ddot q-J^\top F_c-S^\top\tau+nle=0$", NAVY, 11)
    _label(ax, np.array([0.40, 0.30]), "出口は τ 12 だけ", TERR, 10)
    ax.set_xlim(-0.75, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_wbc_eom")


def fig_joints_tau() -> Path:
    fig, ax = _new()
    pose = pose_points()
    _ground(ax, _proj_iso)
    _draw_dog(ax, pose)
    i = 0
    for name in ORDER:
        ch = pose["chains"][name]
        for k in ("hip2", "knee"):
            j = _proj_iso(ch[k])
            ax.add_patch(Circle(j, 0.016, facecolor=TERR, edgecolor=WHITE, lw=0.6, zorder=6))
        # calf mid as KFE already knee; add a third mark near hip for HAA
        h = _proj_iso(ch["hip"])
        ax.add_patch(Circle(h, 0.016, facecolor=TERR, edgecolor=WHITE, lw=0.6, zorder=6))
        i += 3
    _label(ax, np.array([0.38, 0.44]), "12 モータは同じ式", NAVY, 13, weight="bold")
    _label(ax, np.array([0.38, 0.37]), r"$\tau_{\mathrm{cmd}}=\tau_{\mathrm{WBC}}+3(\dot q^*-\dot q)$", NAVY, 11)
    _label(ax, np.array([0.38, 0.30]), "Kp = 0。位置ばねは使わない", TERR, 10)
    _label(ax, np.array([0.38, 0.24]), "上限 33.5 N·m（a1）", MUTED, 10)
    ax.set_xlim(-0.75, 1.15)
    ax.set_ylim(-0.12, 0.52)
    return _save(fig, "fig_joints_tau")


def _box(ax, x, y, w, h, title, lines, accent=TEAL, fs=10):
    ax.add_patch(
        FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.03", facecolor=WHITE, edgecolor=LINE, lw=1.0, mutation_aspect=0.4)
    )
    ax.add_patch(Rectangle((x, y + h - 0.012), w, 0.012, facecolor=accent, lw=0))
    ax.text(x + 0.03, y + h - 0.07, title, fontproperties=_fp(11, "bold"), color=accent, va="top")
    ax.text(x + 0.03, y + h - 0.16, lines, fontproperties=_fp(fs), color=INK, va="top")


def fig_architecture() -> Path:
    """Software / thread architecture with the plant on the right."""
    _setup_font()
    fig, ax = plt.subplots(figsize=(11.6, 5.20), dpi=150)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.2)
    ax.axis("off")
    ax.text(0.25, 4.95, "起動するプロセスと、ロボット", fontproperties=_fp(14, "bold"), color=NAVY)

    _box(ax, 0.25, 3.55, 2.55, 1.20, "legged_robot_target", "/cmd_vel  速さ 4\nまたは goal  位置 6\n→ 目標 24 × 2 点", TEAL, 9)
    _box(ax, 3.00, 3.55, 2.55, 1.20, "gait_command", "端末の gait 名\nModeSchedule\n胴体指令とは別", TERR, 9)
    _box(ax, 5.75, 3.55, 2.70, 1.20, "controller_manager", "LeggedController\nプラグイン\n500 Hz の update", NAVY, 9)

    _box(ax, 0.25, 1.85, 4.05, 1.40, "計画スレッド  100 Hz", "SqpMpc。未来 1.0 s を SQP 1 回。\n状態 24・入力 24。トルクは無い。\npolicy を共有メモリへ書く。", NAVY, 9)
    _box(ax, 4.50, 1.85, 3.95, 1.40, "実行ループ  500 Hz", "推定 → policy の今 → WBC → モータ\nNMPC が遅れても前回の束を使う。\n出すのは τ 12。", TERR, 9)

    # plant panel
    ax.add_patch(FancyBboxPatch((8.70, 0.25), 3.10, 4.70, boxstyle="round,pad=0.02,rounding_size=0.04", facecolor=WHITE, edgecolor=LINE, lw=1.0))
    ax.text(8.88, 4.70, "Plant  A1", fontproperties=_fp(12, "bold"), color=NAVY)
    ax.text(8.88, 4.40, "12 モータ\nIMU\n足力 4", fontproperties=_fp(10), color=INK)
    # mini dog in plant
    inset = ax.inset_axes([0.745, 0.08, 0.23, 0.52])
    inset.set_facecolor(WHITE)
    inset.set_aspect("equal")
    inset.axis("off")
    pose = pose_points()
    _draw_dog(inset, pose, show_joints=False, lw=1.8)
    inset.set_xlim(-0.55, 0.55)
    inset.set_ylim(-0.05, 0.42)

    ax.annotate("", xy=(2.20, 3.25), xytext=(1.52, 3.55), arrowprops=dict(arrowstyle="-|>", color=TEAL, lw=1.2))
    ax.annotate("", xy=(2.40, 3.25), xytext=(4.28, 3.55), arrowprops=dict(arrowstyle="-|>", color=TERR, lw=1.2))
    ax.annotate("", xy=(6.50, 3.25), xytext=(7.10, 3.55), arrowprops=dict(arrowstyle="-|>", color=NAVY, lw=1.2))
    ax.annotate("", xy=(8.70, 2.55), xytext=(8.45, 2.55), arrowprops=dict(arrowstyle="-|>", color=GOLD, lw=1.4))
    ax.text(7.55, 2.68, "τ 12", fontproperties=_fp(10, "bold"), color=GOLD)
    ax.annotate(
        "",
        xy=(7.40, 1.85),
        xytext=(9.40, 1.05),
        arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.1, connectionstyle="arc3,rad=0.18"),
    )
    ax.text(7.55, 1.48, "関節・IMU・接地", fontproperties=_fp(9), color=MUTED)

    ax.text(0.25, 0.55, "層は 3 つ。指令（上）→ 計画（100 Hz）→ 実行（500 Hz）。推定は層ではなく、実行が毎回読む「今」。", fontproperties=_fp(10), color=MUTED)
    ax.text(0.25, 0.22, "ロボットは一番右。ソフトの出口は 12 トルク。センサが推定へ戻る。", fontproperties=_fp(10), color=MUTED)
    return _save(fig, "fig_architecture")


def fig_clocks() -> Path:
    _setup_font()
    fig, ax = plt.subplots(figsize=(11.6, 5.15), dpi=150)
    fig.patch.set_facecolor(PAPER)
    ax.set_facecolor(PAPER)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.2)
    ax.axis("off")
    ax.text(0.3, 4.90, "同じ 20 ms を、2 つの時計で見る", fontproperties=_fp(14, "bold"), color=NAVY)

    # time axis
    ax.plot([0.6, 11.4], [2.55, 2.55], color=LINE, lw=1.2)
    for i in range(11):
        x = 0.6 + i * 1.08
        ax.plot([x, x], [2.48, 2.62], color=MUTED, lw=1.0)
        ax.text(x, 2.28, f"{i*2}", ha="center", fontproperties=_fp(8), color=MUTED)
    ax.text(11.55, 2.48, "ms", fontproperties=_fp(8), color=MUTED)

    # 500 Hz ticks
    ax.text(0.3, 3.85, "実行  500 Hz", fontproperties=_fp(12, "bold"), color=TERR)
    for i in range(10):
        x = 0.6 + i * 1.08
        ax.add_patch(FancyBboxPatch((x - 0.38, 3.15), 0.76, 0.55, boxstyle="round,pad=0.01,rounding_size=0.03", facecolor="#C45A2018", edgecolor=TERR, lw=0.8))
        ax.text(x, 3.42, "推定\nWBC τ", ha="center", va="center", fontproperties=_fp(7), color=TERR)
    ax.text(0.3, 1.85, "計画  100 Hz", fontproperties=_fp(12, "bold"), color=NAVY)
    for i in range(2):
        x0 = 0.6 + i * 5.4
        ax.add_patch(FancyBboxPatch((x0 - 0.38, 1.05), 5.0, 0.70, boxstyle="round,pad=0.01,rounding_size=0.03", facecolor="#1B3A4B14", edgecolor=NAVY, lw=1.0))
        ax.text(x0 + 2.1, 1.40, "未来 1 s を SQP 1 回  →  policy を更新", ha="center", fontproperties=_fp(10), color=NAVY)

    ax.text(0.3, 0.45, "速い側は束の全部を使わない。evaluatePolicy が「今」だけを切る。遅れても前回の束。", fontproperties=_fp(11), color=MUTED)
    return _save(fig, "fig_clocks")


def fig_closed_loop() -> Path:
    """Dog in the middle, sensors in, torques out — the physical loop."""
    fig, ax = _new(w=11.6, h=5.15)
    pose = pose_points()
    _ground(ax, _proj_iso, x0=-0.35, x1=0.55)
    _draw_dog(ax, pose)
    com = _proj_iso(pose["com"])
    _label(ax, np.array([-0.70, 0.44]), "人", TEAL, 12, weight="bold")
    _label(ax, np.array([-0.70, 0.37]), "速さ 4\n歩容名", MUTED, 10)
    _arrow(ax, np.array([-0.48, 0.32]), np.array([-0.18, 0.22]), TEAL, 1.5)
    _label(ax, np.array([0.42, 0.44]), "計画 1 s", NAVY, 12, weight="bold")
    _label(ax, np.array([0.42, 0.37]), "x*, u*\n今の 1 点", MUTED, 10)
    _arrow(ax, np.array([0.38, 0.30]), com + np.array([0.18, 0.06]), NAVY, 1.5)
    _label(ax, np.array([0.42, 0.08]), "τ 12", TERR, 12, weight="bold")
    _arrow(ax, com + np.array([0.16, -0.02]), np.array([0.40, 0.12]), TERR, 1.5)
    _label(ax, np.array([-0.70, 0.08]), "IMU・関節・足", GOLD, 11, weight="bold")
    _arrow(ax, _proj_iso(pose["chains"]["LH"]["foot"]), np.array([-0.42, 0.10]), GOLD, 1.3)
    _label(ax, np.array([-0.05, 0.46]), "閉ループの本体は犬", NAVY, 13, weight="bold")
    ax.set_xlim(-0.85, 0.85)
    ax.set_ylim(-0.12, 0.54)
    return _save(fig, "fig_closed_loop")


def build_all() -> list[Path]:
    fns = [
        fig_dog_model,
        fig_state_x,
        fig_input_u,
        fig_cmd_frames,
        fig_two_points,
        fig_kalman,
        fig_gait_types,
        fig_gait_binds,
        fig_gait_trot,
        fig_nmpc_horizon,
        fig_wbc_eom,
        fig_joints_tau,
        fig_architecture,
        fig_clocks,
        fig_closed_loop,
    ]
    return [fn() for fn in fns]


if __name__ == "__main__":
    for p in build_all():
        print(p)
