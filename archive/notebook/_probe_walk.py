"""Find a controller that LIFTS feet (>=2cm) and holds 5s. Not imported by notebooks."""
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mpc_dog.plant.mujoco_go2 import MujocoGo2

OFF = np.array([0.5, 1.0, 1.0, 0.5])


def jac_leg(model, data, geoms, i):
    J = np.zeros((3, model.nv))
    mujoco.mj_jacGeom(model, data, J, None, int(geoms[i]))
    return J[:, 6 + 3 * i : 9 + 3 * i]


def longest(flags, dt):
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return best * dt


def wrench_grf(feet, com, c, F_des, M_des, reg=0.0, mu=0.0):
    """Least-squares GRF for stance legs. F_des (3,), M_des (3,), returns (4,3)."""
    idx = [i for i in range(4) if c[i] > 0.5]
    grf = np.zeros((4, 3))
    if not idx:
        return grf
    A = []
    for i in idx:
        r = feet[i] - com
        Ai = np.zeros((6, 3))
        Ai[0:3, :] = np.eye(3)
        Ai[3:6, :] = np.array(
            [[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]]
        )
        A.append(Ai)
    A = np.hstack(A)
    b = np.concatenate([F_des, M_des])
    n_f = 3 * len(idx)
    if reg > 0.0:
        A = np.vstack([A, np.sqrt(reg) * np.eye(n_f)])
        b = np.concatenate([b, np.zeros(n_f)])
    f, *_ = np.linalg.lstsq(A, b, rcond=None)
    for k, i in enumerate(idx):
        grf[i] = f[3 * k : 3 * k + 3]
        if grf[i, 2] < 0:
            grf[i, 2] = 0.0
        if mu > 0.0:
            ft = float(np.hypot(grf[i, 0], grf[i, 1]))
            lim = mu * float(grf[i, 2])
            if ft > lim and ft > 1e-9:
                grf[i, 0:2] *= lim / ft
    return grf


def run(name, **kw):
    freq = kw.get("freq", 1.2)
    duty = kw.get("duty", 0.80)
    step_h = kw.get("step_h", 0.045)
    kp_s = kw.get("kp_s", 350.0)
    kd_s = kw.get("kd_s", 18.0)
    kp_z = kw.get("kp_z", 8000.0)
    kd_z = kw.get("kd_z", 400.0)
    kp_xy = kw.get("kp_xy", 80.0)
    kd_xy = kw.get("kd_xy", 20.0)
    kp_att = kw.get("kp_att", 40.0)
    kd_att = kw.get("kd_att", 4.0)
    use_wrench = kw.get("use_wrench", True)
    add_h_swing = kw.get("add_h_swing", False)
    joint_retract = kw.get("joint_retract", 0.0)
    off = np.asarray(kw.get("off", OFF), float)
    T = kw.get("T", 22.0)
    att = 0.35
    zmin_ok = 0.18
    lift_need = 0.020
    air_need = 0.40
    hold_need = kw.get("hold_need", 20.0)

    p = MujocoGo2(scene="flat", seed=0)
    dt = p.sim_dt
    n = int(T / dt)
    geoms = p.foot_geom_ids()
    z0 = float(p.base_pos()[2])
    xy0 = p.base_pos()[:2].copy()
    q0 = np.asarray(p.data.qpos[7:], float).copy()
    mg = p.mass_kg * 9.81
    m = p.mass_kg
    phi = off.copy()
    lift = p.feet_pos_world().copy()
    prev = np.ones(4)
    ok = []
    meas_air = np.zeros(4)
    swing_dz = np.zeros(4)
    zmin, maxr, maxxy = z0, 0.0, 0.0
    for _ in range(n):
        phi = (phi + dt * freq) % 1.0
        c = (phi < duty).astype(float)
        ns = max(float(c.sum()), 1.0)
        pos = p.base_pos()
        velb = p.base_lin_vel_world()
        rpy = p.base_rpy()
        # world angular vel from qvel[3:6] is in body? mujoco qvel 3:6 is rotational, typically local
        w = np.asarray(p.data.qvel[3:6], float)
        Fz = mg + kp_z * (z0 - pos[2]) - kd_z * velb[2]
        fx_cmd = kw.get("fx_cmd", None)
        vx_ref = kw.get("vx_ref", None)
        if vx_ref is not None:
            xref = xy0[0] + float(vx_ref) * (_ * dt)
            Fx = -kp_xy * (pos[0] - xref) - kd_xy * (velb[0] - float(vx_ref))
        elif fx_cmd is None:
            Fx = -kp_xy * (pos[0] - xy0[0]) - kd_xy * velb[0]
        else:
            Fx = float(fx_cmd) - kw.get("kd_x", kd_xy) * velb[0]
        Fy = -kp_xy * (pos[1] - xy0[1]) - kd_xy * velb[1]
        Mx = -kp_att * rpy[0] - kd_att * w[0]
        My = -kp_att * rpy[1] - kd_att * w[1]
        feet = p.feet_pos_world()
        fvel = p.feet_vel_world()
        h = np.asarray(p.data.qfrc_bias[6:], float)
        q = np.asarray(p.data.qpos[7:], float)
        qd = np.asarray(p.data.qvel[6:], float)
        if use_wrench:
            grf = wrench_grf(
                feet,
                p.com_world(),
                c,
                np.array([Fx, Fy, Fz]),
                np.array([Mx, My, 0.0]),
                reg=kw.get("reg", 0.0),
                mu=kw.get("mu", 0.0),
            )
        else:
            grf = np.zeros((4, 3))
            grf[c > 0.5, 2] = Fz / ns
        tau = np.zeros(12)
        for i in range(4):
            if prev[i] > 0.5 and c[i] < 0.5:
                lift[i] = feet[i].copy()
            Jleg = jac_leg(p.model, p.data, geoms, i)
            sl = slice(3 * i, 3 * i + 3)
            if c[i] > 0.5:
                tau[sl] = h[sl] - Jleg.T @ grf[i]
            else:
                s = float(np.clip((phi[i] - duty) / max(1e-6, 1 - duty), 0, 1))
                wsw = freq / max(1e-6, 1 - duty)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + step_h * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
                F_ee = kp_s * (p_d - feet[i]) + kd_s * (v_d - fvel[i])
                tau[sl] = Jleg.T @ F_ee
                if add_h_swing:
                    tau[sl] += h[sl]
                if joint_retract:
                    qdes = q0.copy()
                    qdes[3 * i + 1] += 0.35 * np.sin(np.pi * s)  # thigh flex
                    qdes[3 * i + 2] += 0.45 * np.sin(np.pi * s)  # calf flex
                    tau[sl] += joint_retract * (qdes[sl] - q[sl]) - 0.15 * joint_retract * qd[sl]
                swing_dz[i] = max(swing_dz[i], float(feet[i, 2] - lift[i, 2]))
        p.step(np.clip(tau, p.model.actuator_ctrlrange[:, 0], p.model.actuator_ctrlrange[:, 1]))
        prev = c
        rpy = p.base_rpy()
        xy = p.base_pos()[:2]
        z = float(p.base_pos()[2])
        maxr = max(maxr, abs(rpy[0]), abs(rpy[1]))
        maxxy = max(maxxy, float(np.linalg.norm(xy - xy0)))
        zmin = min(zmin, z)
        if fx_cmd is None and vx_ref is None:
            xy_ok = float(np.linalg.norm(xy - xy0)) < 0.30
        else:
            xy_ok = abs(float(xy[1] - xy0[1])) < 0.25
        ok.append(
            abs(rpy[0]) < att
            and abs(rpy[1]) < att
            and xy_ok
            and z > zmin_ok
            and np.isfinite(p.data.qpos).all()
        )
        contact = p.contact_on()
        for i in range(4):
            if contact[i] < 0.5:
                meas_air[i] += dt
    hold = longest(ok, dt)
    walks = float(swing_dz.min()) >= lift_need and float(meas_air.min()) >= air_need
    dx = float(p.base_pos()[0] - xy0[0])
    print(
        f"{name:48s} hold={hold:.3f} walk={walks} zmin={zmin:.3f} rpy={maxr:.3f} xy={maxxy:.3f} "
        f"dx={dx:+.3f} air={meas_air.round(2)} dz={swing_dz.round(3)} "
        f"PASS={hold >= hold_need and walks}"
    )


if __name__ == "__main__":
    print("=== old 05 'pass' against new walk criteria ===")
    run("old05 equal duty0.96 no wrench", use_wrench=False, duty=0.96, step_h=0.02, kp_s=300, kd_s=20, freq=1.0, kp_xy=0, kp_att=0)
    print("=== wrench + real swing ===")
    run("wrench d0.80 h0.045")
    run("wrench d0.75 h0.05 f1.35", duty=0.75, step_h=0.05, freq=1.35)
    run("wrench d0.82 h0.04 f1.0", duty=0.82, step_h=0.04, freq=1.0)
    run("wrench+retract d0.80", joint_retract=40)
    run("wrench+retract d0.78 h0.05", duty=0.78, step_h=0.05, joint_retract=50, kp_s=280)
    run("wrench att80 d0.80", kp_att=80, kd_att=8, duty=0.80)
    run("equal+retract d0.80 no wrench", use_wrench=False, joint_retract=50, duty=0.80, step_h=0.045)
    run("wrench d0.85 h0.04 retract30", duty=0.85, step_h=0.04, joint_retract=30)
    run("wrench weak att d0.80", kp_att=20, kd_att=3, kp_xy=40, duty=0.80)
