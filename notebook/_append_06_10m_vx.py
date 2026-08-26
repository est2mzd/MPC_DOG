"""Append 06 trial 6: 05-11 controller + vx, new fail GIF. Keep old GIFs."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "06_gait_modes.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 6 — 05 の 10 s 組に \(v_x\)" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    last = "".join(nb.cells[-1].source)
    if last.startswith("## 結果と分析（10 s / 10 m）"):
        nb.cells[-1].source = last.replace(
            "## 結果と分析（10 s / 10 m）",
            "## 途中分析（試行 5）",
            1,
        )
    extra = [
        new_markdown_cell(
            r"""## 試行 6 — 05 の 10 s 組に \(v_x\) を足す

05 試行 11（\(\mu=0.8\)、遊脚 \(K_p=220\)、ゲイト窓内）はその場なら 10 s 持つ。歩く判定は 10 m なので、参照 \(v_x=0.35\,\mathrm{m/s}\) を足す。着地点はまだ離地 xy。新しい GIF を残し、旧 GIF は消さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 05 試行 11 と同じ円錐 wrench。trot 窓のまま vx を足し、10s かつ 10m かを見る。

HOLD_S10 = 10.0
DIST_M = 10.0
T_FWD = 12.0
VX = 0.35  # 内容: 10m に必要な速さの目安。着地点先送りはしない


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


def rollout_fwd(freq, duty, off, step_h, vx, T=T_FWD, kp_s=220.0, kd_s=14.0):
    """意図: その場 10s 組に vx 参照だけ足す。着地 xy は離地位置。"""
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
    ok, meas_air, swing_dz = [], np.zeros(4), np.zeros(4)
    zmin, maxr, maxxy = z0, 0.0, 0.0
    t = 0.0
    for _k in range(n):
        phi = (phi + dt * freq) % 1.0
        c = (phi < duty).astype(float)
        pos = p.base_pos()
        velb = p.base_lin_vel_world()
        rpy = p.base_rpy()
        wbody = np.asarray(p.data.qvel[3:6], dtype=np.float64)
        xref = xy0[0] + vx * t  # 内容: 前進参照。Raibert 項は足さない
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
        grf = wrench_grf_mu(feet, p.com_world(), c, F_des, M_des)
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
        t += dt
        rpy = p.base_rpy()
        xy = p.base_pos()[:2]
        z = float(p.base_pos()[2])
        maxr = max(maxr, abs(rpy[0]), abs(rpy[1]))
        maxxy = max(maxxy, float(np.linalg.norm(xy - xy0)))
        zmin = min(zmin, z)
        ok.append(
            abs(rpy[0]) < ATT_TOL
            and abs(rpy[1]) < ATT_TOL
            and abs(float(xy[1] - xy0[1])) < 0.25
            and z > 0.18
            and bool(np.isfinite(p.data.qpos).all())
        )
        contact = p.contact_on()
        for i in range(4):
            if contact[i] < 0.5:
                meas_air[i] += dt
    dx = float(p.base_pos()[0] - xy0[0])
    return {
        "hold": longest_true_seconds(ok, dt),
        "zmin": zmin,
        "maxr": maxr,
        "maxxy": maxxy,
        "dx": dx,
        "meas_air": meas_air,
        "swing_dz": swing_dz,
    }


off_trot = np.array([0.5, 1.0, 1.0, 0.5])
r6 = rollout_fwd(1.35, 0.75, off_trot, 0.05, VX)
walk6 = float(r6["swing_dz"].min()) >= 0.020 and float(r6["meas_air"].min()) >= 1.00
ok6 = r6["hold"] >= HOLD_S10 and walk6 and abs(r6["dx"]) >= DIST_M
print("trial6  05-11 + vx  hold", r6["hold"], "walk", walk6, "dx", r6["dx"], "xy", r6["maxxy"])
print("  air", r6["meas_air"].round(3), "dz", r6["swing_dz"].round(3), "PASS", ok6)
assert not ok6
print("06 trial6 FAIL expected: 10s and 10m do not hold together. GIF 06e is new; 06a-d kept.")

st6 = {"phi": off_trot.copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4), "xy0": None, "t": 0.0}


def tau_fwd(pl):
    """内容: 試行 6 と同じ。白矢印は指令 GRF。旧 GIF は上書きしない。"""
    dt = pl.sim_dt
    if st6["lift"] is None:
        st6["lift"] = pl.feet_pos_world().copy()
        st6["z0"] = float(pl.base_pos()[2])
        st6["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    duty, freq, step_h = 0.75, 1.35, 0.05
    phi = (st6["phi"] + dt * freq) % 1.0
    c = (phi < duty).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    xref = st6["xy0"][0] + VX * st6["t"]
    F_des = np.array(
        [
            -80.0 * (pos[0] - xref) - 20.0 * (velb[0] - VX),
            -80.0 * (pos[1] - st6["xy0"][1]) - 20.0 * velb[1],
            pl.mass_kg * 9.81 + KP_Z * (st6["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-40.0 * rpy[0] - 4.0 * wbody[0], -40.0 * rpy[1] - 4.0 * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf_mu(feet, pl.com_world(), c, F_des, M_des)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st6["prev"][i] > 0.5 and c[i] < 0.5:
            st6["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
            wsw = freq / max(1e-6, 1.0 - duty)
            p_d = st6["lift"][i].copy()
            p_d[2] = st6["lift"][i, 2] + step_h * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (220.0 * (p_d - feet[i]) + 14.0 * (v_d - fvel[i]))
    st6["phi"], st6["prev"], st6["c"], st6["cmd"] = phi, c, c, cmd
    st6["t"] += dt
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant6 = MujocoGo2(scene="flat", seed=0)
path6 = render_rollout_gif(
    plant6,
    Path("assets/06e_trot_vx_flip.gif"),
    n_steps=int(round(T_FWD / plant6.sim_dt)),
    capture_every=80,
    tau_fn=tau_fwd,
    command_grf=lambda pl: st6["cmd"],
    extra_lines=lambda pl: [
        f"c={st6['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  x={pl.base_pos()[0]:.2f}"
    ],
    title="06e trot + vx  10s/10m fail  white=command GRF",
)
print(path6.resolve(), path6.stat().st_size, "bytes")
display(Image(filename=str(path6)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析

- 試行 1–5: 残す。旧 GIF `06a`–`06d` は消さない
- 試行 6: 05 で 10 s 持った組に \(v_x\) を足すと約 1 s で倒れる。着地点が離地 xy のままなので、胴体が足の前に出てピッチが壊れる。新しい GIF `06e`

06 は 10 s かつ 10 m では未成功のままである。07 へ進まない。着地点の先送りはフェーズ 2 だが、まず同じ式で 10 m を持たせるか、持てないことを切り分ける。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
