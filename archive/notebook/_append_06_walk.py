"""Append 06 walk-criteria cells. Delete after use."""
from __future__ import annotations

from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "06_gait_modes.ipynb"

OLD_SUCCESS = """### 成功条件（数値。ここが合否）

[README §3.8](../docs/block-curriculum/00_README.md)。**各モードで連続 5.0 秒。** \\(T=7.0\\,\\mathrm{s}\\)。

| 量 | 全モード |
|---|---|
| \\(\\lvert\\mathrm{roll}\\rvert,\\lvert\\mathrm{pitch}\\rvert\\) | \\(<0.35\\,\\mathrm{rad}\\) |
| \\(\\lVert xy-xy_0\\rVert\\) | \\(<0.30\\,\\mathrm{m}\\) |
| \\(z\\) | \\(>0.20\\,\\mathrm{m}\\) |
| `qpos` | 有限 |
| hold | \\(\\ge 5.0\\,\\mathrm{s}\\) |

加えて:

| モード | 追加 |
|---|---|
| `full_stance` | 実測空中 \\(<0.05\\,\\mathrm{s}/\\)脚（足が上がらない確認） |
| `trot` | 実測空中 \\(\\ge 0.15\\,\\mathrm{s}/\\)脚。対角ペア指令が両方 |
| `crawl` / `pace` / `bound` | 指令 \\(c(t)\\) が offset どおり（4 本線の図）。空中は duty が高いので短くてよい |

PyMPC 既定の低い duty のまま pace すると転ぶ。その失敗 GIF は残す。満たすまで 07 へ進まない。
"""

NEW_SUCCESS = r"""### 成功条件（数値。ここが合否）

[README §3.8](../docs/block-curriculum/00_README.md)。制御は **05 試行 8 の瞬間 wrench** に揃える（EqualShare の duty 上げは直立になる）。変えるのは offset / duty / freq だけ。`if gait == "trot"` は書かない。

**合否はこの表である。** 試行 1–2 は空中 \(0.15\,\mathrm{s}\) と「短くてよい」で 5 モードを通した。リフトは数 mm。旧数字は消さないが、合格には使わない。

\(T=7.0\,\mathrm{s}\)。胴体は 05 と同じ。

| 量 | 条件 |
|---|---|
| \(\lvert\mathrm{roll}\rvert,\lvert\mathrm{pitch}\rvert\) | \(<0.35\,\mathrm{rad}\)、連続 \(\ge 5.0\,\mathrm{s}\) |
| \(\lVert xy-xy_0\rVert\) | \(<0.30\,\mathrm{m}\) |
| ベース \(z\) | \(>0.18\,\mathrm{m}\) |
| `qpos` | 有限 |

モードごとの追加:

| モード | 追加（新判定） |
|---|---|
| `full_stance` | 各脚空中 \(<0.05\,\mathrm{s}\)。足が上がらないこと |
| `trot` / `crawl` | 各脚リフト \(\ge 0.020\,\mathrm{m}\)、各脚空中 \(\ge 0.40\,\mathrm{s}\)。hold \(\ge 5.0\,\mathrm{s}\) |
| `pace` / `bound` | 同じ wrench・実遊脚で測る。hold とリフトを同時に 5 秒満たせなければ **不合格のまま残す**。duty を \(0.99\) にして直立にした記録は合格にしない |

指令 \(c(t)\) の 4 本線を残す。失敗 GIF は消さない。`trot` と `crawl` が実歩で 5 秒、`full_stance` が静止、`pace`/`bound` の実遊脚失敗が残ってから 07 へ進む。
"""

OLD_CTRL = """05 の制御器（EqualShare + 高さ P + 立脚 \\(h-J^{\\top}F\\) + 遊脚 \\(+J^{\\top}\\) PD）は**変えない**。この段の新しい現象は、**位相オフセットと duty / 周波数だけ**で運びが変わることである（[S4](../docs/block-curriculum/02_Stage_Ladder.md)、[06](../docs/block-curriculum/06_Gait_Modes.md)）。`if gait == "trot"` は書かない。

offset の数字は上流 PyMPC `PeriodicGaitGenerator` から写す。EqualShare は 2 脚の左右（pace）や前後（bound）に弱いので、duty を上げて overlap を残す。duty もゲイトパラメータである。
"""

NEW_CTRL = r"""05 試行 8 の制御器（瞬間 wrench + 高さ P + 立脚 \(h-J^{\top}F\) + 遊脚 \(+J^{\top}\) PD）の **\(\tau\) の式は変えない**。この段の新しい現象は、**位相オフセットと duty / 周波数だけ**で運びが変わることである（[S4](../docs/block-curriculum/02_Stage_Ladder.md)、[06](../docs/block-curriculum/06_Gait_Modes.md)）。`if gait == "trot"` は書かない。

offset は trot \([0.5,1.0,1.0,0.5]\)、crawl \([0.0,0.5,0.75,0.25]\)、pace \([0.5,0.0,0.5,0.0]\)、bound \([0.5,0.5,0.0,0.0]\)（FL,FR,RL,RR）。EqualShare のまま duty を \(0.96\) 以上にすると映像は直立になる。
"""

OLD_HYP = """1. PyMPC 既定 duty（pace 0.70 など）では EqualShare のまま転ぶ
2. duty を上げ、offset だけモードどおりにすれば、5 秒持てる
"""

NEW_HYP = r"""1. PyMPC 既定 duty（pace 0.70 など）では EqualShare のまま転ぶ
2. duty を \(0.96\) 以上にすれば旧判定は 5 モードとも持つ。リフトは数 mm で足踏みではない
3. 同じ wrench で trot（duty \(0.75\)）と crawl（duty \(0.92\), \(0.4\,\mathrm{Hz}\)）は実歩で 5 秒持つ
4. 同じ wrench で pace / bound に実遊脚を付けると、左右または前後の 2 脚支持が持たず転ぶ
"""


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("新判定（試行 3 以降）" in "".join(c.source) or "試行 3 — 旧 5 モードを新判定" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    c0 = "".join(nb.cells[0].source)
    if OLD_SUCCESS not in c0:
        raise SystemExit("success block not found")
    c0 = c0.replace(OLD_SUCCESS, NEW_SUCCESS, 1)
    if OLD_CTRL not in c0:
        raise SystemExit("ctrl block not found")
    c0 = c0.replace(OLD_CTRL, NEW_CTRL, 1)
    if OLD_HYP not in c0:
        raise SystemExit("hyp block not found")
    c0 = c0.replace(OLD_HYP, NEW_HYP, 1)
    nb.cells[0].source = c0

    nb.cells[8].source = r"""## 途中分析（試行 1–2）

- 試行 1: ペース offset でも duty が低いと EqualShare は左右に倒れる。GIF `06a`
- 試行 2: duty \(0.96\)–\(0.995\) なら 5 モードとも旧判定で 5 秒。図 `06_gait_contact.png`、GIF `06b`。空中は \(0.03\)–\(0.26\,\mathrm{s}\)。直立に見える

次のセルから 05 の実歩判定と wrench で測り直す。
"""

    extra = [
        new_markdown_cell(
            r"""## 試行 3 — 旧 5 モードを新判定で測る（仮説 2）

制御は試行 2 のまま（EqualShare、高い duty、`step_h=2\,\mathrm{cm}`）。空中 \(0.40\,\mathrm{s}\) で落とす。セルと GIF `06b` は消さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 2 の GAITS を、05 と同じ空中 0.40s で採点する。旧 rollout に swing_dz は無い。

AIR_WALK = 0.40  # 内容: 05 新判定の各脚空中 [s]
AIR_STANCE = 0.05  # 内容: full_stance はこれ未満


def walks_air(r, air_need=AIR_WALK):
    """内容: 旧 rollout はリフトを返さないので、空中だけで新判定の足踏み側を見る。"""
    return float(r["meas_air"].min()) >= air_need


print("trial3  old EqualShare GAITS vs new air floor")
old_new = {}
for name, g in GAITS.items():
    r = rollout(g["freq"], g["duty"], g["off"])  # 内容: 試行 2 と同じ関数
    ok_old = r["hold"] >= HOLD_S
    if name == "full_stance":
        ok_new = r["hold"] >= HOLD_S and float(r["meas_air"].max()) < AIR_STANCE
    else:
        ok_new = r["hold"] >= HOLD_S and walks_air(r)
    old_new[name] = (ok_old, ok_new, r["hold"], r["meas_air"].round(3))
    print(name, "hold", r["hold"], "air", r["meas_air"].round(3), "old", ok_old, "new", ok_new)

assert old_new["full_stance"][1], "full_stance should still be a stand"
assert not any(old_new[n][1] for n in ("trot", "crawl", "pace", "bound"))
print("06 trial3: stepping modes FAIL new air floor (expected). stance still a stand.")
'''
        ),
        new_markdown_cell(
            r"""## 試行 4 — 同じ wrench、offset / duty / freq だけ変える（仮説 3–4）

\(\tau\) は 05 試行 8。trot と crawl は実歩で 5 秒。pace と bound は実遊脚のまま倒れることを残す。`full_stance` は duty \(1\)。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 05 の wrench 立脚を、ゲイト数字だけ変えて 5 モード走らせる。

LIFT_NEED = 0.020
Z_MIN_WALK = 0.18
KP_XY, KD_XY = 80.0, 20.0
KP_ATT, KD_ATT = 40.0, 4.0

GAITS_WALK = {
    "full_stance": {"freq": 1.0, "duty": 1.00, "off": np.array([0.0, 0.0, 0.0, 0.0]), "step_h": 0.05},
    "trot": {"freq": 1.35, "duty": 0.75, "off": np.array([0.5, 1.0, 1.0, 0.5]), "step_h": 0.05},
    "crawl": {"freq": 0.40, "duty": 0.92, "off": np.array([0.0, 0.5, 0.75, 0.25]), "step_h": 0.05},
    "pace": {"freq": 1.35, "duty": 0.75, "off": np.array([0.5, 0.0, 0.5, 0.0]), "step_h": 0.05},
    "bound": {"freq": 1.20, "duty": 0.80, "off": np.array([0.5, 0.5, 0.0, 0.0]), "step_h": 0.05},
}


def wrench_grf(feet, com, c, F_des, M_des):
    """内容: 05 と同じ瞬間最小二乗。ホライズンなし。"""
    idx = [i for i in range(4) if c[i] > 0.5]
    grf = np.zeros((4, 3))
    if not idx:
        return grf
    blocks = []
    for i in idx:
        r = feet[i] - com
        Ai = np.zeros((6, 3))
        Ai[0:3, :] = np.eye(3)
        Ai[3:6, :] = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
        blocks.append(Ai)
    A = np.hstack(blocks)
    b = np.concatenate([F_des, M_des])
    f, *_ = np.linalg.lstsq(A, b, rcond=None)
    for k, i in enumerate(idx):
        grf[i] = f[3 * k : 3 * k + 3]
        if grf[i, 2] < 0.0:
            grf[i, 2] = 0.0
    return grf


def rollout_wrench(freq, duty, off, step_h, T=T_RUN):
    """意図: 05 試行 8 と同じ tau。戻りに hold / 空中 / リフト / c(t)。"""
    p = MujocoGo2(scene="flat", seed=0)
    dt = p.sim_dt
    n = int(round(T / dt))
    geoms = p.foot_geom_ids()
    z0 = float(p.base_pos()[2])
    xy0 = p.base_pos()[:2].copy()
    mg = p.mass_kg * 9.81
    phi = np.asarray(off, dtype=np.float64).copy()
    lift = p.feet_pos_world().copy()
    prev = np.ones(4)
    ok, cs, meas_air = [], [], np.zeros(4)
    swing_dz = np.zeros(4)
    zmin, maxr, maxxy = z0, 0.0, 0.0
    for _k in range(n):
        phi = (phi + dt * freq) % 1.0
        c = (phi < duty).astype(float)
        pos = p.base_pos()
        velb = p.base_lin_vel_world()
        rpy = p.base_rpy()
        wbody = np.asarray(p.data.qvel[3:6], dtype=np.float64)
        F_des = np.array(
            [
                -KP_XY * (pos[0] - xy0[0]) - KD_XY * velb[0],
                -KP_XY * (pos[1] - xy0[1]) - KD_XY * velb[1],
                mg + KP_Z * (z0 - pos[2]) - KD_Z * velb[2],
            ]
        )
        M_des = np.array([-KP_ATT * rpy[0] - KD_ATT * wbody[0], -KP_ATT * rpy[1] - KD_ATT * wbody[1], 0.0])
        feet = p.feet_pos_world()
        fvel = p.feet_vel_world()
        h = np.asarray(p.data.qfrc_bias[6:], dtype=np.float64)
        grf = wrench_grf(feet, p.com_world(), c, F_des, M_des)
        tau = np.zeros(12)
        for i in range(4):
            if prev[i] > 0.5 and c[i] < 0.5:
                lift[i] = feet[i].copy()
            Jleg = jac_leg(p.model, p.data, geoms, i)
            sl = slice(3 * i, 3 * i + 3)
            if c[i] > 0.5:
                tau[sl] = h[sl] - Jleg.T @ grf[i]
            else:
                s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
                wsw = freq / max(1e-6, 1.0 - duty)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + step_h * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
                F_ee = 350.0 * (p_d - feet[i]) + 18.0 * (v_d - fvel[i])
                tau[sl] = Jleg.T @ F_ee
                swing_dz[i] = max(swing_dz[i], float(feet[i, 2] - lift[i, 2]))
        p.step(clip_torque(tau, p.model.actuator_ctrlrange))
        prev = c
        cs.append(c.copy())
        rpy = p.base_rpy()
        xy = p.base_pos()[:2]
        z = float(p.base_pos()[2])
        maxr = max(maxr, abs(rpy[0]), abs(rpy[1]))
        maxxy = max(maxxy, float(np.linalg.norm(xy - xy0)))
        zmin = min(zmin, z)
        ok.append(
            abs(rpy[0]) < ATT_TOL
            and abs(rpy[1]) < ATT_TOL
            and float(np.linalg.norm(xy - xy0)) < XY_TOL
            and z > Z_MIN_WALK
            and bool(np.isfinite(p.data.qpos).all())
        )
        contact = p.contact_on()
        for i in range(4):
            if contact[i] < 0.5:
                meas_air[i] += dt
    return {
        "hold": longest_true_seconds(ok, dt),
        "zmin": zmin,
        "maxr": maxr,
        "maxxy": maxxy,
        "meas_air": meas_air,
        "swing_dz": swing_dz,
        "c": np.asarray(cs),
        "dt": dt,
    }


def walks(r):
    """内容: 4 脚ともリフト 2cm と空中 0.40s。"""
    return float(r["swing_dz"].min()) >= LIFT_NEED and float(r["meas_air"].min()) >= AIR_WALK


walk_results = {}
for name, g in GAITS_WALK.items():
    r = rollout_wrench(g["freq"], g["duty"], g["off"], g["step_h"])
    walk_results[name] = r
    print(
        name,
        "hold",
        r["hold"],
        "walk",
        walks(r),
        "zmin",
        r["zmin"],
        "rpy",
        r["maxr"],
        "xy",
        r["maxxy"],
        "air",
        r["meas_air"].round(3),
        "dz",
        r["swing_dz"].round(3),
    )

assert walk_results["full_stance"]["hold"] >= HOLD_S
assert float(walk_results["full_stance"]["meas_air"].max()) < AIR_STANCE
assert walk_results["trot"]["hold"] >= HOLD_S and walks(walk_results["trot"])
assert walk_results["crawl"]["hold"] >= HOLD_S and walks(walk_results["crawl"])
assert walk_results["pace"]["hold"] < HOLD_S
assert walk_results["bound"]["hold"] < HOLD_S
print("06 trial4: stance/trot/crawl meet the new table; pace/bound real-swing FAIL (kept)")
'''
        ),
        new_markdown_cell(
            r"""## 接地の 4 本線と GIF

指令 \(c(t)\) が offset どおりか。crawl の成功と pace の転倒を残す。trot の実歩 GIF は 05 の `05d`。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 指令接地の 4 本線。crawl 成功 GIF と pace 転倒 GIF。

import matplotlib.pyplot as plt

fig, axes = plt.subplots(len(GAITS_WALK), 1, figsize=(8, 10), sharex=True)
for ax, name in zip(axes, GAITS_WALK):
    r = walk_results[name]
    t = np.arange(r["c"].shape[0]) * r["dt"]
    for i, lab in enumerate(("FL", "FR", "RL", "RR")):
        ax.plot(t, r["c"][:, i] + 1.15 * i, lw=0.8, label=lab)  # 内容: 脚ごとに縦にずらす
    ax.set_ylabel(name)
    ax.set_ylim(-0.2, 4.8)
    ax.grid(True, alpha=0.3)
axes[0].legend(loc="upper right", ncol=4, fontsize=8)
axes[-1].set_xlabel("t [s]")
fig.suptitle("06 command c(t)  wrench gaits  1=stance")
fig.tight_layout()
cpath = Path("assets/06_gait_contact_walk.png")
fig.savefig(cpath, dpi=110)
plt.close(fig)
print(cpath.resolve(), cpath.stat().st_size, "bytes")

# --- crawl 成功 GIF ---
st_c = {"phi": GAITS_WALK["crawl"]["off"].copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4), "xy0": None}


def tau_crawl(pl):
    """内容: 試行 4 の crawl。白矢印は wrench GRF。"""
    dt = pl.sim_dt
    g = GAITS_WALK["crawl"]
    if st_c["lift"] is None:
        st_c["lift"] = pl.feet_pos_world().copy()
        st_c["z0"] = float(pl.base_pos()[2])
        st_c["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    phi = (st_c["phi"] + dt * g["freq"]) % 1.0
    c = (phi < g["duty"]).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -KP_XY * (pos[0] - st_c["xy0"][0]) - KD_XY * velb[0],
            -KP_XY * (pos[1] - st_c["xy0"][1]) - KD_XY * velb[1],
            mg + KP_Z * (st_c["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-KP_ATT * rpy[0] - KD_ATT * wbody[0], -KP_ATT * rpy[1] - KD_ATT * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf(feet, pl.com_world(), c, F_des, M_des)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st_c["prev"][i] > 0.5 and c[i] < 0.5:
            st_c["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - g["duty"]) / max(1e-6, 1.0 - g["duty"]), 0.0, 1.0))
            wsw = g["freq"] / max(1e-6, 1.0 - g["duty"])
            p_d = st_c["lift"][i].copy()
            p_d[2] = st_c["lift"][i, 2] + g["step_h"] * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = g["step_h"] * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (350.0 * (p_d - feet[i]) + 18.0 * (v_d - fvel[i]))
    st_c["phi"], st_c["prev"], st_c["c"], st_c["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant_c = MujocoGo2(scene="flat", seed=0)
path_c = render_rollout_gif(
    plant_c,
    Path("assets/06c_crawl_lift.gif"),
    n_steps=int(round(T_RUN / plant_c.sim_dt)),
    capture_every=80,
    tau_fn=tau_crawl,
    command_grf=lambda pl: st_c["cmd"],
    extra_lines=lambda pl: [
        f"c={st_c['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="06c crawl wrench  lift>=2cm  white=command GRF",
)
print(path_c.resolve(), path_c.stat().st_size, "bytes")
display(Image(filename=str(path_c)))

# --- pace 転倒 GIF ---
st_p = {"phi": GAITS_WALK["pace"]["off"].copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4), "xy0": None}


def tau_pace(pl):
    """内容: 試行 4 の pace。同側 2 脚支持で倒れる。"""
    dt = pl.sim_dt
    g = GAITS_WALK["pace"]
    if st_p["lift"] is None:
        st_p["lift"] = pl.feet_pos_world().copy()
        st_p["z0"] = float(pl.base_pos()[2])
        st_p["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    phi = (st_p["phi"] + dt * g["freq"]) % 1.0
    c = (phi < g["duty"]).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -KP_XY * (pos[0] - st_p["xy0"][0]) - KD_XY * velb[0],
            -KP_XY * (pos[1] - st_p["xy0"][1]) - KD_XY * velb[1],
            mg + KP_Z * (st_p["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-KP_ATT * rpy[0] - KD_ATT * wbody[0], -KP_ATT * rpy[1] - KD_ATT * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf(feet, pl.com_world(), c, F_des, M_des)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st_p["prev"][i] > 0.5 and c[i] < 0.5:
            st_p["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - g["duty"]) / max(1e-6, 1.0 - g["duty"]), 0.0, 1.0))
            wsw = g["freq"] / max(1e-6, 1.0 - g["duty"])
            p_d = st_p["lift"][i].copy()
            p_d[2] = st_p["lift"][i, 2] + g["step_h"] * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = g["step_h"] * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (350.0 * (p_d - feet[i]) + 18.0 * (v_d - fvel[i]))
    st_p["phi"], st_p["prev"], st_p["c"], st_p["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant_p = MujocoGo2(scene="flat", seed=0)
path_p = render_rollout_gif(
    plant_p,
    Path("assets/06d_pace_real_swing_flip.gif"),
    n_steps=int(round(T_RUN / plant_p.sim_dt)),
    capture_every=80,
    tau_fn=tau_pace,
    command_grf=lambda pl: st_p["cmd"],
    extra_lines=lambda pl: [
        f"c={st_p['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="06d pace wrench + real swing  flip  white=command GRF",
)
print(path_p.resolve(), path_p.stat().st_size, "bytes")
display(Image(filename=str(path_p)))
print("06 PASS under new table: stance stand, trot+crawl walk, pace/bound real-swing fails kept")
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析

- 試行 1: 低い duty の pace は EqualShare で倒れる。GIF `06a`
- 試行 2: 高い duty なら 5 モードとも旧判定 PASS。GIF `06b`。足はほぼ上がらない
- 試行 3: 同じ走行を空中 \(0.40\,\mathrm{s}\) で落とす。`full_stance` だけが静止として残る
- 試行 4: 05 の wrench のまま offset を変える。`trot` / `crawl` はリフト \(2\,\mathrm{cm}\) 以上で 5 秒。`pace` / `bound` は同側または前後の 2 脚ではモーメントが足りず転ぶ。GIF `06c`（crawl）、`06d`（pace）

運びの違いは予測最適化ではない。offset である。同じ立脚分配では、対角（trot）と 3 脚（crawl）は足踏みでき、左右（pace）と前後（bound）はできない。

## 次の仮説

ごく小さい \(v_x\) を、05 の trot wrench に足す。着地 xy はまだ離地位置のまま。旧 07 の \(F_x=0.6\,\mathrm{N}\) 直立ずらしは、新判定では不合格である。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
