"""Append 05 trial 15: trial 13 + higher attitude D. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 15 — 姿勢 D だけ上げる" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    extra = [
        new_markdown_cell(
            r"""## 試行 15 — 姿勢 D だけ上げる

yaw は外す。ゲイトは試行 13（\(1.22\,\mathrm{Hz}/0.80/5\,\mathrm{cm}\)、\(K_p^{sw}=180\)）。変えるのは \(k_d^{\mathrm{att}}:4\to 6\) だけ。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 13 と同じ。姿勢 D だけ 6。first_fail を残す。GIF 05i。

FREQ15, DUTY15, H15 = 1.22, 0.80, 0.05
KP_S15, KD_S15 = 180.0, 12.0
KD_ATT15 = 6.0  # 内容: 試行 13 の 4 から一つ上げる


def rollout_t15(T=T_20):
    """意図: 試行 13 + 姿勢 D。Mz=0。"""
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
    ok, meas_air = [], np.zeros(4)
    swing_dz = np.zeros(4)
    pair_a = pair_b = 0
    zmin, maxr, maxxy = z0, 0.0, 0.0
    first = None
    for k in range(n):
        phi = (phi + dt * FREQ15) % 1.0
        c = (phi < DUTY15).astype(float)
        pos = p.base_pos()
        velb = p.base_lin_vel_world()
        rpy = p.base_rpy()
        wbody = np.asarray(p.data.qvel[3:6], dtype=np.float64)
        F_des = np.array(
            [
                -80.0 * (pos[0] - xy0[0]) - 20.0 * velb[0],
                -80.0 * (pos[1] - xy0[1]) - 20.0 * velb[1],
                mg + KP_Z * (z0 - pos[2]) - KD_Z * velb[2],
            ]
        )
        M_des = np.array([-40.0 * rpy[0] - KD_ATT15 * wbody[0], -40.0 * rpy[1] - KD_ATT15 * wbody[1], 0.0])
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
                s = float(np.clip((phi[i] - DUTY15) / max(1e-6, 1.0 - DUTY15), 0.0, 1.0))
                wsw = FREQ15 / max(1e-6, 1.0 - DUTY15)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + H15 * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = H15 * np.pi * np.cos(np.pi * s) * wsw
                tau[sl] = Jleg.T @ (KP_S15 * (p_d - feet[i]) + KD_S15 * (v_d - fvel[i]))
                swing_dz[i] = max(swing_dz[i], float(feet[i, 2] - lift[i, 2]))
        p.step(clip_torque(tau, p.model.actuator_ctrlrange))
        prev = c
        rpy = p.base_rpy()
        xy = p.base_pos()[:2]
        z = float(p.base_pos()[2])
        dxy = float(np.linalg.norm(xy - xy0))
        maxr = max(maxr, abs(rpy[0]), abs(rpy[1]))
        maxxy = max(maxxy, dxy)
        zmin = min(zmin, z)
        reasons = []
        if abs(rpy[0]) >= ATT_TOL:
            reasons.append("roll")
        if abs(rpy[1]) >= ATT_TOL:
            reasons.append("pitch")
        if dxy >= XY_TOL:
            reasons.append("xy")
        if z <= 0.18:
            reasons.append("z")
        good = not reasons and bool(np.isfinite(p.data.qpos).all())
        ok.append(good)
        if (not good) and first is None:
            first = (k * dt, reasons, float(rpy[0]), float(rpy[1]), dxy, z)
        for i in range(4):
            if p.contact_on()[i] < 0.5:
                meas_air[i] += dt
        if c[0] > 0.5 and c[3] > 0.5 and c[1] < 0.5 and c[2] < 0.5:
            pair_a += 1
        if c[1] > 0.5 and c[2] > 0.5 and c[0] < 0.5 and c[3] < 0.5:
            pair_b += 1
    return {
        "hold": longest_true_seconds(ok, dt),
        "zmin": zmin,
        "maxr": maxr,
        "maxxy": maxxy,
        "meas_air": meas_air,
        "swing_dz": swing_dz,
        "pair_a_s": pair_a * dt,
        "pair_b_s": pair_b * dt,
        "first": first,
    }


r15 = rollout_t15()
ok15 = r15["hold"] >= HOLD_S20 and walks20(r15) and gait_in_window(FREQ15, DUTY15, H15)
print("trial15  t13+kdatt6  hold", r15["hold"], "walk", walks20(r15), "PASS_20s", ok15)
print("  zmin", r15["zmin"], "rpy", r15["maxr"], "xy", r15["maxxy"])
print("  meas_air", r15["meas_air"].round(3), "swing_dz", r15["swing_dz"].round(4))
print("  first_fail", r15["first"])
if not ok15:
    print("05 trial15 FAIL under 20s. GIF 05i. stay on 05.")
else:
    print("05 trial15 PASS under 20s + gait window.")

st15 = {"phi": TROT_OFF.copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4)}


def tau_t15(pl):
    """内容: 試行 15。白矢印は指令 GRF。"""
    dt = pl.sim_dt
    if st15["lift"] is None:
        st15["lift"] = pl.feet_pos_world().copy()
        st15["z0"] = float(pl.base_pos()[2])
        st15["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    phi = (st15["phi"] + dt * FREQ15) % 1.0
    c = (phi < DUTY15).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st15["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st15["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st15["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-40.0 * rpy[0] - KD_ATT15 * wbody[0], -40.0 * rpy[1] - KD_ATT15 * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf_mu(feet, pl.com_world(), c, F_des, M_des, mu=0.8)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st15["prev"][i] > 0.5 and c[i] < 0.5:
            st15["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - DUTY15) / max(1e-6, 1.0 - DUTY15), 0.0, 1.0))
            wsw = FREQ15 / max(1e-6, 1.0 - DUTY15)
            p_d = st15["lift"][i].copy()
            p_d[2] = st15["lift"][i, 2] + H15 * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = H15 * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (KP_S15 * (p_d - feet[i]) + KD_S15 * (v_d - fvel[i]))
    st15["phi"], st15["prev"], st15["c"], st15["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant15 = MujocoGo2(scene="flat", seed=0)
path15 = render_rollout_gif(
    plant15,
    Path("assets/05i_inplace_trot_20s_kdatt.gif"),
    n_steps=int(round(T_20 / plant15.sim_dt)),
    capture_every=80,
    tau_fn=tau_t15,
    command_grf=lambda pl: st15["cmd"],
    extra_lines=lambda pl: [
        f"c={st15['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05i  1.22Hz/0.80  kd_att=6  white=command GRF",
)
print(path15.resolve(), path15.stat().st_size, "bytes", "t", float(plant15.data.time))
display(Image(filename=str(path15)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（試行 15）

数値と first_fail は上のセル。20 s 未満なら 05 は未成功。GIF `05i`。旧 GIF は消さない。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
