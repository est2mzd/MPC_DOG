"""Append 05 trial 11: 10s pass with mu clip + milder swing. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 11 — 摩擦円錐と弱い遊脚 PD" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    extra = [
        new_markdown_cell(
            r"""## 試行 11 — 摩擦円錐と弱い遊脚 PD（10 s）

試行 8–10 の wrench は unconstrained 最小二乗で、遊脚 \(K_p=350\) が胴体を引っ張る。仮説: 立脚 GRF を \(\mu=0.8\) の円錐に切り、遊脚を \(K_p=220\) に落とすと、同じゲイト窓（\(1.35\,\mathrm{Hz}/0.75/5\,\mathrm{cm}\)）で 10 s 持つ。NMPC は足さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 8 の wrench に摩擦円錐を足し、遊脚 Kp を 220 にする。T=12s、10s 判定。

def wrench_grf_mu(feet, com, c, F_des, M_des, mu=0.8):
    """内容: 05 の瞬間 LS のあと、Fz>=0 と |Fxy|<=mu Fz に切る。ホライズンなし。"""
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
        ft = float(np.hypot(grf[i, 0], grf[i, 1]))
        lim = mu * float(grf[i, 2])
        if ft > lim and ft > 1e-9:
            grf[i, :2] *= lim / ft  # 内容: 摩擦円錐の内側へ縮小
    return grf


def rollout_wrench_mu(duty, step_h, freq, kp_s=220.0, kd_s=14.0, mu=0.8, T=T_LONG):
    """意図: 試行 8 と同じ式 + 円錐切。戻りは 10s 判定用。"""
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
    for _k in range(n):
        phi = (phi + dt * freq) % 1.0
        c = (phi < duty).astype(float)
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
        M_des = np.array([-40.0 * rpy[0] - 4.0 * wbody[0], -40.0 * rpy[1] - 4.0 * wbody[1], 0.0])
        feet = p.feet_pos_world()
        fvel = p.feet_vel_world()
        h = np.asarray(p.data.qfrc_bias[6:], dtype=np.float64)
        grf = wrench_grf_mu(feet, p.com_world(), c, F_des, M_des, mu=mu)
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
                tau[sl] = Jleg.T @ (kp_s * (p_d - feet[i]) + kd_s * (v_d - fvel[i]))
                swing_dz[i] = max(swing_dz[i], float(feet[i, 2] - lift[i, 2]))
        p.step(clip_torque(tau, p.model.actuator_ctrlrange))
        prev = c
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
    }


r11 = rollout_wrench_mu(duty=0.75, step_h=0.05, freq=1.35)
print("trial11  mu0.8 ks220  hold", r11["hold"], "walk", walks(r11))
print("  zmin", r11["zmin"], "rpy", r11["maxr"], "xy", r11["maxxy"])
print("  meas_air", r11["meas_air"].round(3), "swing_dz", r11["swing_dz"].round(4))
print("  pair_a", r11["pair_a_s"], "pair_b", r11["pair_b_s"], "gait_ok", gait_in_window(1.35, 0.75, 0.05))
assert gait_in_window(1.35, 0.75, 0.05)
assert r11["hold"] >= HOLD_S10, f"hold {r11['hold']:.3f}s"
assert walks(r11) and float(r11["meas_air"].min()) >= AIR_12
assert r11["pair_a_s"] > 0.0 and r11["pair_b_s"] > 0.0
print("05 PASS under 10s + gait window: hold", r11["hold"], "s  dz", r11["swing_dz"].round(3))

st11 = {
    "phi": TROT_OFF.copy(),
    "prev": np.ones(4),
    "lift": None,
    "cmd": np.zeros((4, 3)),
    "c": np.ones(4),
    "xy0": None,
}


def tau_ok_10s(pl):
    """内容: 試行 11 と同じ。白矢印は円錐付き指令 GRF。"""
    dt = pl.sim_dt
    if st11["lift"] is None:
        st11["lift"] = pl.feet_pos_world().copy()
        st11["z0"] = float(pl.base_pos()[2])
        st11["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    duty, freq, step_h = 0.75, 1.35, 0.05
    phi = (st11["phi"] + dt * freq) % 1.0
    c = (phi < duty).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st11["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st11["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st11["z0"] - pos[2]) - KD_Z * velb[2],
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
        if st11["prev"][i] > 0.5 and c[i] < 0.5:
            st11["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
            wsw = freq / max(1e-6, 1.0 - duty)
            p_d = st11["lift"][i].copy()
            p_d[2] = st11["lift"][i, 2] + step_h * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (220.0 * (p_d - feet[i]) + 14.0 * (v_d - fvel[i]))
    st11["phi"], st11["prev"], st11["c"], st11["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant11 = MujocoGo2(scene="flat", seed=0)
path11 = render_rollout_gif(
    plant11,
    Path("assets/05e_inplace_trot_10s.gif"),
    n_steps=int(round(T_LONG / plant11.sim_dt)),
    capture_every=80,
    tau_fn=tau_ok_10s,
    command_grf=lambda pl: st11["cmd"],
    extra_lines=lambda pl: [
        f"c={st11['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05e in-place trot  10s  1.35Hz/0.75/5cm  white=command GRF",
)
print(path11.resolve(), path11.stat().st_size, "bytes", "t", float(plant11.data.time))
display(Image(filename=str(path11)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（10 s 合格）

- 試行 8: 5 s 判定では通った。10 s では持たない（試行 9）
- 試行 10: 上流どおり \(0.74/5.6\,\mathrm{cm}\) と LC \(8\,\mathrm{cm}\) は 2–3 s
- 試行 11: 同じ窓の \(1.35\,\mathrm{Hz}/0.75/5\,\mathrm{cm}\) に \(\mu=0.8\) 切と遊脚 \(K_p=220\) を足すと hold \(\ge 10\,\mathrm{s}\)。GIF `05e`

摩擦円錐は予測のホライズンではない。NMPC はまだ呼ばない。

## 次の仮説

同じ \(\tau\) で offset だけ変える（06）。歩くモードは 10 s かつ 10 m。ゲイト数字は上流表の窓。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
