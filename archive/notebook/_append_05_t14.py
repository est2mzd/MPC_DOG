"""Append 05 trial 14: same gait as 13 plus yaw P. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 14 — yaw の P だけ足す" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    extra = [
        new_markdown_cell(
            r"""## 試行 14 — yaw の P だけ足す

試行 13 は 17.7 s で転倒した。仮説: 見出し角が積もり、支持脚の水平力が壊れる。ゲイトは試行 13 のまま。変えるのは \(M_z=-20(\psi-\psi_0)-3\omega_z\) だけ。着地点は離地 xy のまま。NMPC は足さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 13 と同じ f/d/h/遊脚。wrench の Mz だけ yaw P。T=22s。GIF 05h。

FREQ14, DUTY14, H14 = 1.22, 0.80, 0.05
KP_S14, KD_S14 = 180.0, 12.0
KP_YAW, KD_YAW = 20.0, 3.0  # 内容: 見出し角を初期値へ戻す [N·m/rad], [N·m·s/rad]


def rollout_t14(T=T_20):
    """意図: 試行 13 + Mz。first_fail も返す。"""
    p = MujocoGo2(scene="flat", seed=0)
    dt = p.sim_dt
    n = int(round(T / dt))
    geoms = p.foot_geom_ids()
    z0 = float(p.base_pos()[2])
    xy0 = p.base_pos()[:2].copy()
    yaw0 = float(p.base_rpy()[2])
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
        phi = (phi + dt * FREQ14) % 1.0
        c = (phi < DUTY14).astype(float)
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
        M_des = np.array(
            [
                -40.0 * rpy[0] - 4.0 * wbody[0],
                -40.0 * rpy[1] - 4.0 * wbody[1],
                -KP_YAW * (rpy[2] - yaw0) - KD_YAW * wbody[2],
            ]
        )
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
                s = float(np.clip((phi[i] - DUTY14) / max(1e-6, 1.0 - DUTY14), 0.0, 1.0))
                wsw = FREQ14 / max(1e-6, 1.0 - DUTY14)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + H14 * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = H14 * np.pi * np.cos(np.pi * s) * wsw
                tau[sl] = Jleg.T @ (KP_S14 * (p_d - feet[i]) + KD_S14 * (v_d - fvel[i]))
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
        contact = p.contact_on()
        for i in range(4):
            if contact[i] < 0.5:
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


r14 = rollout_t14()
ok14 = r14["hold"] >= HOLD_S20 and walks20(r14) and gait_in_window(FREQ14, DUTY14, H14)
print("trial14  t13+yaw  hold", r14["hold"], "walk", walks20(r14), "PASS_20s", ok14)
print("  zmin", r14["zmin"], "rpy", r14["maxr"], "xy", r14["maxxy"])
print("  meas_air", r14["meas_air"].round(3), "swing_dz", r14["swing_dz"].round(4))
print("  pair_a", r14["pair_a_s"], "pair_b", r14["pair_b_s"])
print("  first_fail", r14["first"])
if not ok14:
    print("05 trial14 FAIL under 20s. GIF 05h. stay on 05.")
else:
    print("05 trial14 PASS under 20s + gait window.")

st14 = {
    "phi": TROT_OFF.copy(),
    "prev": np.ones(4),
    "lift": None,
    "cmd": np.zeros((4, 3)),
    "c": np.ones(4),
    "xy0": None,
}


def tau_t14(pl):
    """内容: 試行 14。白矢印は円錐付き指令 GRF。"""
    dt = pl.sim_dt
    if st14["lift"] is None:
        st14["lift"] = pl.feet_pos_world().copy()
        st14["z0"] = float(pl.base_pos()[2])
        st14["xy0"] = pl.base_pos()[:2].copy()
        st14["yaw0"] = float(pl.base_rpy()[2])
    geoms = pl.foot_geom_ids()
    phi = (st14["phi"] + dt * FREQ14) % 1.0
    c = (phi < DUTY14).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st14["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st14["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st14["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array(
        [
            -40.0 * rpy[0] - 4.0 * wbody[0],
            -40.0 * rpy[1] - 4.0 * wbody[1],
            -KP_YAW * (rpy[2] - st14["yaw0"]) - KD_YAW * wbody[2],
        ]
    )
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf_mu(feet, pl.com_world(), c, F_des, M_des, mu=0.8)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st14["prev"][i] > 0.5 and c[i] < 0.5:
            st14["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - DUTY14) / max(1e-6, 1.0 - DUTY14), 0.0, 1.0))
            wsw = FREQ14 / max(1e-6, 1.0 - DUTY14)
            p_d = st14["lift"][i].copy()
            p_d[2] = st14["lift"][i, 2] + H14 * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = H14 * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (KP_S14 * (p_d - feet[i]) + KD_S14 * (v_d - fvel[i]))
    st14["phi"], st14["prev"], st14["c"], st14["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant14 = MujocoGo2(scene="flat", seed=0)
path14 = render_rollout_gif(
    plant14,
    Path("assets/05h_inplace_trot_20s_yaw.gif"),
    n_steps=int(round(T_20 / plant14.sim_dt)),
    capture_every=80,
    tau_fn=tau_t14,
    command_grf=lambda pl: st14["cmd"],
    extra_lines=lambda pl: [
        f"c={st14['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05h  1.22Hz/0.80 + yaw P  white=command GRF",
)
print(path14.resolve(), path14.stat().st_size, "bytes", "t", float(plant14.data.time))
display(Image(filename=str(path14)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（試行 14）

数値と first_fail は上のセル。hold が 20 s 未満なら 05 は未成功。GIF `05h`。旧 GIF は消さない。06 へ進まない。

## 次の仮説

first_fail が pitch なら姿勢 D を一つ上げる。xy なら水平ゲインを一つ変える。z なら高さ D を一つ変える。一度に一つ。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
