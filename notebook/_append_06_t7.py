"""Append 06 trial 7: slower vx under 20s/10m. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "06_gait_modes.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 7 — より遅い" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    last = "".join(nb.cells[-1].source)
    nb.cells[-1].source = last.replace(
        "06 は 10 s かつ 10 m では未成功のままである。07 へ進まない。着地点の先送りはフェーズ 2 だが、まず同じ式で 10 m を持たせるか、持てないことを切り分ける。",
        "06 の歩くモードは 20 s かつ 10 m。`full_stance` は 10 s。次のセルで \\(v_x\\) だけ落とす。",
    )
    extra = [
        new_markdown_cell(
            r"""## 試行 7 — より遅い \(v_x\)（20 s / 10 m）

05 は 10 s その場で完了。試行 6 の \(v_x=0.35\,\mathrm{m/s}\) は約 1 s で倒れた。仮説: 参照だけ \(0.15\,\mathrm{m/s}\) に落とすと、同じ 05-11 wrench で 20 s 持つ。着地点は離地 xy のまま。10 m は hold 中の水平。旧 GIF は消さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 6 と同じ円錐 wrench。vx だけ 0.15。T=25s。20s かつ hold 中 10m。GIF 06f。

HOLD_CTRL = 20.0  # 内容: 制御して歩く段の連続時間 [s]
DIST_M = 10.0     # 内容: hold 中の水平下限 [m]
T_FWD = 25.0
VX7 = 0.15        # 内容: 試行 6 の 0.35 から一つ下げる
TROT_OFF = np.array([0.5, 1.0, 1.0, 0.5])
FREQ7, DUTY7, H7 = 1.35, 0.75, 0.05


def gait_in_window(freq, duty, step_h):
    """内容: 窓判定。duty 0.80-0.74 の二進誤差を吸う。"""
    return abs(freq - 1.35) <= 0.15 + 1e-9 and abs(duty - 0.74) <= 0.06 + 1e-9 and 0.045 <= step_h <= 0.090


def wrench_grf_mu(feet, com, c, F_des, M_des, mu=0.8):
    """内容: 05 試行 11 と同じ瞬間 LS + 摩擦円錐。"""
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
    f, *_ = np.linalg.lstsq(A, np.concatenate([F_des, M_des]), rcond=None)
    for k, i in enumerate(idx):
        grf[i] = f[3 * k : 3 * k + 3]
        if grf[i, 2] < 0.0:
            grf[i, 2] = 0.0
        ft = float(np.hypot(grf[i, 0], grf[i, 1]))
        lim = mu * float(grf[i, 2])
        if ft > lim and ft > 1e-9:
            grf[i, :2] *= lim / ft
    return grf


def rollout_fwd7(vx=VX7, T=T_FWD):
    """意図: 05-11 + 遅い vx。dist は longest hold の終端 xy。"""
    p = MujocoGo2(scene="flat", seed=0)
    dt = p.sim_dt
    n = int(round(T / dt))
    geoms = p.foot_geom_ids()
    z0 = float(p.base_pos()[2])
    xy0 = p.base_pos()[:2].copy()
    mg = p.mass_kg * 9.81
    phi = TROT_OFF.copy()
    lift = p.feet_pos_world().copy()
    prev = np.ones(4)
    ok, meas_air, swing_dz = [], np.zeros(4), np.zeros(4)
    zmin, maxr = z0, 0.0
    first = None
    xs = []
    for k in range(n):
        t = k * dt
        phi = (phi + dt * FREQ7) % 1.0
        c = (phi < DUTY7).astype(float)
        pos = p.base_pos()
        velb = p.base_lin_vel_world()
        rpy = p.base_rpy()
        wbody = np.asarray(p.data.qvel[3:6], dtype=np.float64)
        xref = xy0[0] + vx * t
        F_des = np.array(
            [
                -80.0 * (pos[0] - xref) - 20.0 * (velb[0] - vx),
                -80.0 * (pos[1] - xy0[1]) - 20.0 * velb[1],
                mg + KP_Z * (z0 - pos[2]) - KD_Z * velb[2],
            ]
        )
        M_des = np.array([-40.0 * rpy[0] - 4.0 * wbody[0], -40.0 * rpy[1] - 4.0 * wbody[1], 0.0])
        feet = p.feet_pos_world()
        fvel = p.feet_vel_world()
        h = np.asarray(p.data.qfrc_bias[6:], dtype=np.float64)
        grf = wrench_grf_mu(feet, p.com_world(), c, F_des, M_des, mu=0.8)
        tau = np.zeros(12)
        for i in range(4):
            if prev[i] > 0.5 and c[i] < 0.5:
                lift[i] = feet[i].copy()
            Jleg = jac_leg(p.model, p.data, geoms, i)
            sl = slice(3 * i, 3 * i + 3)
            if c[i] > 0.5:
                tau[sl] = h[sl] - Jleg.T @ grf[i]
            else:
                s = float(np.clip((phi[i] - DUTY7) / max(1e-6, 1.0 - DUTY7), 0.0, 1.0))
                wsw = FREQ7 / max(1e-6, 1.0 - DUTY7)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + H7 * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = H7 * np.pi * np.cos(np.pi * s) * wsw
                tau[sl] = Jleg.T @ (220.0 * (p_d - feet[i]) + 14.0 * (v_d - fvel[i]))
                swing_dz[i] = max(swing_dz[i], float(feet[i, 2] - lift[i, 2]))
        p.step(clip_torque(tau, p.model.actuator_ctrlrange))
        prev = c
        rpy = p.base_rpy()
        xy = p.base_pos()[:2]
        z = float(p.base_pos()[2])
        maxr = max(maxr, abs(rpy[0]), abs(rpy[1]))
        zmin = min(zmin, z)
        y_ok = abs(float(xy[1] - xy0[1])) < 0.25
        reasons = []
        if abs(rpy[0]) >= ATT_TOL:
            reasons.append("roll")
        if abs(rpy[1]) >= ATT_TOL:
            reasons.append("pitch")
        if not y_ok:
            reasons.append("y")
        if z <= 0.18:
            reasons.append("z")
        good = not reasons and bool(np.isfinite(p.data.qpos).all())
        ok.append(good)
        xs.append(float(xy[0] - xy0[0]))
        if (not good) and first is None:
            first = (t, reasons, float(rpy[0]), float(rpy[1]), float(xy[0] - xy0[0]), z)
        for i in range(4):
            if p.contact_on()[i] < 0.5:
                meas_air[i] += dt
    hold = longest_true_seconds(ok, dt)
    # hold 中の最長 True 区間の終端で水平を読む
    best = cur = start = 0
    best_s = 0
    for i, f in enumerate(ok):
        if f:
            if cur == 0:
                start = i
            cur += 1
            if cur > best:
                best, best_s = cur, start
        else:
            cur = 0
    end = best_s + best - 1 if best else 0
    dist_hold = abs(xs[end]) if ok else 0.0
    walk = float(swing_dz.min()) >= 0.020 and float(meas_air.min()) >= 1.80
    return {
        "hold": hold,
        "dist_hold": dist_hold,
        "zmin": zmin,
        "maxr": maxr,
        "meas_air": meas_air,
        "swing_dz": swing_dz,
        "first": first,
        "walk": walk,
    }


assert gait_in_window(FREQ7, DUTY7, H7)
r7 = rollout_fwd7()
ok7 = r7["hold"] >= HOLD_CTRL and r7["dist_hold"] >= DIST_M and r7["walk"]
print("trial7  05-11 + vx0.15  hold", r7["hold"], "dist_hold", r7["dist_hold"], "walk", r7["walk"], "PASS", ok7)
print("  zmin", r7["zmin"], "rpy", r7["maxr"])
print("  meas_air", r7["meas_air"].round(3), "swing_dz", r7["swing_dz"].round(4))
print("  first_fail", r7["first"])
print("06 trial7", "PASS" if ok7 else "FAIL", "GIF 06f. stay on 06." if not ok7 else "20s/10m.")

st7 = {"phi": TROT_OFF.copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4)}


def tau_t7(pl):
    """内容: 試行 7。白矢印は指令 GRF。"""
    dt = pl.sim_dt
    if st7["lift"] is None:
        st7["lift"] = pl.feet_pos_world().copy()
        st7["z0"] = float(pl.base_pos()[2])
        st7["xy0"] = pl.base_pos()[:2].copy()
        st7["t"] = 0.0
    geoms = pl.foot_geom_ids()
    phi = (st7["phi"] + dt * FREQ7) % 1.0
    c = (phi < DUTY7).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    xref = st7["xy0"][0] + VX7 * st7["t"]
    F_des = np.array(
        [
            -80.0 * (pos[0] - xref) - 20.0 * (velb[0] - VX7),
            -80.0 * (pos[1] - st7["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st7["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-40.0 * rpy[0] - 4.0 * wbody[0], -40.0 * rpy[1] - 4.0 * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf_mu(feet, pl.com_world(), c, F_des, M_des, mu=0.8)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st7["prev"][i] > 0.5 and c[i] < 0.5:
            st7["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - DUTY7) / max(1e-6, 1.0 - DUTY7), 0.0, 1.0))
            wsw = FREQ7 / max(1e-6, 1.0 - DUTY7)
            p_d = st7["lift"][i].copy()
            p_d[2] = st7["lift"][i, 2] + H7 * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = H7 * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (220.0 * (p_d - feet[i]) + 14.0 * (v_d - fvel[i]))
    st7["phi"], st7["prev"], st7["c"], st7["cmd"] = phi, c, c, cmd
    st7["t"] += dt
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant7 = MujocoGo2(scene="flat", seed=0)
path7 = render_rollout_gif(
    plant7,
    Path("assets/06f_trot_vx015.gif"),
    n_steps=int(round(T_FWD / plant7.sim_dt)),
    capture_every=80,
    tau_fn=tau_t7,
    command_grf=lambda pl: st7["cmd"],
    extra_lines=lambda pl: [
        f"c={st7['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="06f trot + vx=0.15  20s/10m  white=command GRF",
)
print(path7.resolve(), path7.stat().st_size, "bytes", "t", float(plant7.data.time))
display(Image(filename=str(path7)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（試行 7）

数値と first_fail は上のセル。20 s かつ hold 中 10 m 未満なら 06 は未成功。GIF `06f`。旧 GIF は消さない。07 へ進まない。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
