"""Append 05 trial 12: re-score trial 11 under 20 s. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 12 — 20 秒判定" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    last = "".join(nb.cells[-1].source)
    nb.cells[-1].source = last.replace(
        "同じ \\(\\tau\\) で offset だけ変える（06）。歩くモードは 10 s かつ 10 m。ゲイト数字は上流表の窓。",
        "10 s 合格は 20 s 判定の合格ではない。次のセルで試行 11 を \\(T=22\\,\\mathrm{s}\\) で測る。",
    )
    extra = [
        new_markdown_cell(
            r"""## 試行 12 — 20 秒判定で試行 11 を測る

背景の連続時間を 10 s から 20 s に伸ばした。制御は試行 11 のまま（\(\mu=0.8\)、遊脚 \(K_p=220\)、\(1.35\,\mathrm{Hz}/0.75/5\,\mathrm{cm}\)）。\(T=22\,\mathrm{s}\)。hold が 20 s 未満なら不合格のまま残す。旧 GIF `05e` は消さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 11 と同じ wrench を T=22s で走らせ、背景の 20s 判定で落とす。GIF は 05f。

HOLD_S20 = 20.0  # 内容: 05 以降の連続時間 [s]
T_20 = 22.0      # 内容: 20s 判定のあと余白
AIR_22 = 1.80    # 内容: T=22s での各脚空中下限 [s]


def gait_in_window(freq, duty, step_h):
    """内容: PyMPC/LC の足踏みから大きく外れていないか。"""
    return abs(freq - 1.35) <= 0.15 and abs(duty - 0.74) <= 0.06 and 0.045 <= step_h <= 0.090


def rollout_wrench_mu(duty, step_h, freq, kp_s=220.0, kd_s=14.0, mu=0.8, T=T_20):
    """意図: 試行 11 と同じ式。戻りは 20s 判定用。"""
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
            and z > 0.18
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


def walks20(r):
    """内容: リフト 2cm と T=22s の空中下限。"""
    return float(r["swing_dz"].min()) >= 0.020 and float(r["meas_air"].min()) >= AIR_22


def wrench_grf_mu(feet, com, c, F_des, M_des, mu=0.8):
    """内容: 瞬間 LS のあと Fz>=0 と |Fxy|<=mu Fz。ホライズンなし。"""
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
            grf[i, :2] *= lim / ft
    return grf


r12 = rollout_wrench_mu(duty=0.75, step_h=0.05, freq=1.35)
ok20 = r12["hold"] >= HOLD_S20 and walks20(r12) and gait_in_window(1.35, 0.75, 0.05)
print("trial12  trial11-ctrl T=22  hold", r12["hold"], "walk", walks20(r12), "gait_ok", gait_in_window(1.35, 0.75, 0.05))
print("  zmin", r12["zmin"], "rpy", r12["maxr"], "xy", r12["maxxy"])
print("  meas_air", r12["meas_air"].round(3), "swing_dz", r12["swing_dz"].round(4))
print("  pair_a", r12["pair_a_s"], "pair_b", r12["pair_b_s"], "PASS_20s", ok20)
assert gait_in_window(1.35, 0.75, 0.05)
assert r12["hold"] < HOLD_S20, "if this holds 20s, keep it as the new pass"
print("05 trial12 FAIL under 20s hold (expected). GIF 05e kept. new GIF 05f.")

st12 = {
    "phi": TROT_OFF.copy(),
    "prev": np.ones(4),
    "lift": None,
    "cmd": np.zeros((4, 3)),
    "c": np.ones(4),
    "xy0": None,
}


def tau_t12(pl):
    """内容: 試行 11 と同じ。白矢印は円錐付き指令 GRF。"""
    dt = pl.sim_dt
    if st12["lift"] is None:
        st12["lift"] = pl.feet_pos_world().copy()
        st12["z0"] = float(pl.base_pos()[2])
        st12["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    duty, freq, step_h = 0.75, 1.35, 0.05
    phi = (st12["phi"] + dt * freq) % 1.0
    c = (phi < duty).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st12["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st12["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st12["z0"] - pos[2]) - KD_Z * velb[2],
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
        if st12["prev"][i] > 0.5 and c[i] < 0.5:
            st12["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
            wsw = freq / max(1e-6, 1.0 - duty)
            p_d = st12["lift"][i].copy()
            p_d[2] = st12["lift"][i, 2] + step_h * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (220.0 * (p_d - feet[i]) + 14.0 * (v_d - fvel[i]))
    st12["phi"], st12["prev"], st12["c"], st12["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant12 = MujocoGo2(scene="flat", seed=0)
path12 = render_rollout_gif(
    plant12,
    Path("assets/05f_inplace_trot_20s_t11.gif"),
    n_steps=int(round(T_20 / plant12.sim_dt)),
    capture_every=80,
    tau_fn=tau_t12,
    command_grf=lambda pl: st12["cmd"],
    extra_lines=lambda pl: [
        f"c={st12['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05f trial11 ctrl  20s judge  1.35Hz/0.75/5cm  white=command GRF",
)
print(path12.resolve(), path12.stat().st_size, "bytes", "t", float(plant12.data.time))
display(Image(filename=str(path12)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（試行 12）

- 試行 11: 10 s 判定では通った（GIF `05e`）。消さない
- 試行 12: 同じ制御を \(T=22\,\mathrm{s}\) で測ると hold は約 10.2 s で切れる。20 s 不合格。GIF `05f`

05 は 20 s 判定では未成功のままである。06 へ進まない。失敗を消さない。

## 次の仮説

試行 11 の \(1.35\,\mathrm{Hz}\) は遊脚が長く、10 s のあとに姿勢が壊れる。ゲイト窓の下限寄り（\(f\approx 1.22\,\mathrm{Hz}\)、duty \(0.80\)、遊脚 \(K_p=180\)）なら、同じ wrench で 20 s に届くかを次の試行で見る。NMPC は足さない。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
