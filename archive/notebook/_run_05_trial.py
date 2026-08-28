"""Append + exec one 05 trial. Not imported by the notebook."""
from __future__ import annotations

import argparse
import base64
import io
import os
import sys
import traceback
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell, new_output

os.environ.setdefault("MUJOCO_GL", "egl")
NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"


class Image:
    def __init__(self, filename=None, **_k):
        self.filename = filename


def display(obj):
    print("[display]", getattr(obj, "filename", obj))


def append_trial(n: int, title: str, hypo: str, gif: str, marker: str, body: str, analysis: str) -> None:
    nb = nbformat.read(NB, as_version=4)
    if any(title in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    extra = [
        new_markdown_cell(f"## 試行 {n} — {title}\n\n{hypo}\n"),
        new_code_cell(body),
        new_markdown_cell(analysis),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


def exec_trial(marker: str, gif_name: str, exec_n: int) -> None:
    gif = NB.parent / "assets" / gif_name
    nb = nbformat.read(NB, as_version=4)
    ns = {"__name__": "__main__", "Image": Image, "display": display}

    def gait_in_window(freq, duty, step_h):
        return (
            abs(freq - 1.35) <= 0.15 + 1e-9
            and abs(duty - 0.74) <= 0.06 + 1e-9
            and 0.045 <= step_h <= 0.090
        )

    ns["gait_in_window"] = gait_in_window
    idxs = [1]
    tnew = None
    for i, c in enumerate(nb.cells):
        src = "".join(c.source)
        if "trial12  trial11-ctrl" in src:
            idxs.append(i)
        if marker in src:
            tnew = i
    if tnew is None:
        raise SystemExit(f"{marker} not found")
    idxs.append(tnew)
    os.chdir(NB.parent)
    for idx in idxs:
        src = "".join(nb.cells[idx].source)
        if "trial12  trial11-ctrl" in src:
            src = src.split("st12 = {")[0]
        buf_out, buf_err = io.StringIO(), io.StringIO()
        print(f"=== exec cell {idx} ===", flush=True)
        try:
            with redirect_stdout(buf_out), redirect_stderr(buf_err):
                exec(compile(src, f"05c{idx}", "exec"), ns, ns)
            ok, err = True, ""
        except Exception:
            ok, err = False, traceback.format_exc()
            print(err, file=sys.stderr)
        text, errc = buf_out.getvalue(), buf_err.getvalue()
        print(text, end="")
        if idx != tnew:
            if not ok:
                raise SystemExit(f"cell {idx} failed")
            continue
        outputs = []
        if text:
            outputs.append(new_output("stream", name="stdout", text=text))
        if errc:
            outputs.append(new_output("stream", name="stderr", text=errc))
        if not ok:
            outputs.append(
                new_output(
                    "error",
                    ename="ExecutionError",
                    evalue=err.splitlines()[-1],
                    traceback=err.splitlines(),
                )
            )
        if gif.is_file():
            outputs.append(
                new_output(
                    "display_data",
                    data={
                        "image/gif": base64.b64encode(gif.read_bytes()).decode("ascii"),
                        "text/plain": ["<IPython.core.display.Image object>"],
                    },
                    metadata={},
                )
            )
        nb.cells[idx].outputs = outputs
        nb.cells[idx].execution_count = exec_n
        if not ok:
            nbformat.write(nb, NB)
            raise SystemExit(f"cell {idx} failed")
    nbformat.write(nb, NB)
    print("wrote", NB)


T16_BODY = r'''# --- このセルの意図 ---
# 試行 13 と同じゲイト。姿勢 P だけ 30。GIF 05j。

FREQ16, DUTY16, H16 = 1.22, 0.80, 0.05
KP_S16, KD_S16 = 180.0, 12.0
KP_ATT16 = 30.0  # 内容: 試行 13 の 40 から一つ下げる


def rollout_t16(T=T_20):
    """意図: 試行 13 + 弱い姿勢 P。Mz=0。"""
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
    zmin, maxr, maxxy = z0, 0.0, 0.0
    first = None
    for k in range(n):
        phi = (phi + dt * FREQ16) % 1.0
        c = (phi < DUTY16).astype(float)
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
        M_des = np.array([-KP_ATT16 * rpy[0] - 4.0 * wbody[0], -KP_ATT16 * rpy[1] - 4.0 * wbody[1], 0.0])
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
                s = float(np.clip((phi[i] - DUTY16) / max(1e-6, 1.0 - DUTY16), 0.0, 1.0))
                wsw = FREQ16 / max(1e-6, 1.0 - DUTY16)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + H16 * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = H16 * np.pi * np.cos(np.pi * s) * wsw
                tau[sl] = Jleg.T @ (KP_S16 * (p_d - feet[i]) + KD_S16 * (v_d - fvel[i]))
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
    return {
        "hold": longest_true_seconds(ok, dt),
        "zmin": zmin,
        "maxr": maxr,
        "maxxy": maxxy,
        "meas_air": meas_air,
        "swing_dz": swing_dz,
        "first": first,
    }


r16 = rollout_t16()
ok16 = r16["hold"] >= HOLD_S20 and walks20(r16) and gait_in_window(FREQ16, DUTY16, H16)
print("trial16  t13+kpatt30  hold", r16["hold"], "walk", walks20(r16), "PASS_20s", ok16)
print("  zmin", r16["zmin"], "rpy", r16["maxr"], "xy", r16["maxxy"])
print("  meas_air", r16["meas_air"].round(3), "swing_dz", r16["swing_dz"].round(4))
print("  first_fail", r16["first"])
print("05 trial16", "PASS" if ok16 else "FAIL", "GIF 05j. stay on 05." if not ok16 else "20s window.")

st16 = {"phi": TROT_OFF.copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4)}


def tau_t16(pl):
    """内容: 試行 16。白矢印は指令 GRF。"""
    dt = pl.sim_dt
    if st16["lift"] is None:
        st16["lift"] = pl.feet_pos_world().copy()
        st16["z0"] = float(pl.base_pos()[2])
        st16["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    phi = (st16["phi"] + dt * FREQ16) % 1.0
    c = (phi < DUTY16).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st16["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st16["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st16["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-KP_ATT16 * rpy[0] - 4.0 * wbody[0], -KP_ATT16 * rpy[1] - 4.0 * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf_mu(feet, pl.com_world(), c, F_des, M_des, mu=0.8)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st16["prev"][i] > 0.5 and c[i] < 0.5:
            st16["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - DUTY16) / max(1e-6, 1.0 - DUTY16), 0.0, 1.0))
            wsw = FREQ16 / max(1e-6, 1.0 - DUTY16)
            p_d = st16["lift"][i].copy()
            p_d[2] = st16["lift"][i, 2] + H16 * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = H16 * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (KP_S16 * (p_d - feet[i]) + KD_S16 * (v_d - fvel[i]))
    st16["phi"], st16["prev"], st16["c"], st16["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant16 = MujocoGo2(scene="flat", seed=0)
path16 = render_rollout_gif(
    plant16,
    Path("assets/05j_inplace_trot_20s_kpatt.gif"),
    n_steps=int(round(T_20 / plant16.sim_dt)),
    capture_every=80,
    tau_fn=tau_t16,
    command_grf=lambda pl: st16["cmd"],
    extra_lines=lambda pl: [
        f"c={st16['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05j  1.22Hz/0.80  kp_att=30  white=command GRF",
)
print(path16.resolve(), path16.stat().st_size, "bytes", "t", float(plant16.data.time))
display(Image(filename=str(path16)))
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", choices=["append16", "exec16"])
    args = parser.parse_args()
    if args.step == "append16":
        append_trial(
            16,
            "姿勢 P だけ下げる",
            r"試行 13 のゲイトのまま。変えるのは \(k_p^{\mathrm{att}}:40\to 30\) だけ。D は 4。",
            "05j_inplace_trot_20s_kpatt.gif",
            "trial16  t13+kpatt30",
            T16_BODY,
            r"""## 結果と分析（試行 16）

数値と first_fail は上のセル。20 s 未満なら 05 は未成功。GIF `05j`。旧 GIF は消さない。
""",
        )
    else:
        exec_trial("trial16  t13+kpatt30", "05j_inplace_trot_20s_kpatt.gif", 16)


if __name__ == "__main__":
    main()
