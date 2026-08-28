"""Append 05 trial 13: slower trot in the gait window. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 13 — 窓の下限寄りの周波数" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    extra = [
        new_markdown_cell(
            r"""## 試行 13 — 窓の下限寄りの周波数

仮説: 試行 11 の \(1.35\,\mathrm{Hz}\) は遊脚が長く、10 s のあとに倒れる。窓の下限寄り \(f=1.22\,\mathrm{Hz}\)（\(\lvert 1.22-1.35\rvert=0.13\le 0.15\)）、duty \(0.80\)（\(\lvert 0.80-0.74\rvert=0.06\)）、遊脚 \(K_p=180\) なら、同じ瞬間 wrench で 20 s に届くかを見る。着地点の先送りと NMPC は足さない。旧 GIF は消さない。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# ゲイト窓の下限寄り。試行 11 と同じ wrench / 円錐。T=22s、20s 判定。GIF 05g。

FREQ13, DUTY13, H13 = 1.22, 0.80, 0.05  # 内容: 窓内の f, duty, 遊脚高さ
KP_S13, KD_S13 = 180.0, 12.0            # 内容: 試行 11 より弱い遊脚 PD

assert gait_in_window(FREQ13, DUTY13, H13)
r13 = rollout_wrench_mu(duty=DUTY13, step_h=H13, freq=FREQ13, kp_s=KP_S13, kd_s=KD_S13, mu=0.8, T=T_20)
ok13 = r13["hold"] >= HOLD_S20 and walks20(r13)
print("trial13  f1.22 d0.80 ks180  hold", r13["hold"], "walk", walks20(r13))
print("  zmin", r13["zmin"], "rpy", r13["maxr"], "xy", r13["maxxy"])
print("  meas_air", r13["meas_air"].round(3), "swing_dz", r13["swing_dz"].round(4))
print("  pair_a", r13["pair_a_s"], "pair_b", r13["pair_b_s"], "PASS_20s", ok13)
if not ok13:
    print("05 trial13 FAIL under 20s. GIF 05g. stay on 05.")
else:
    print("05 trial13 PASS under 20s + gait window.")

st13 = {
    "phi": TROT_OFF.copy(),
    "prev": np.ones(4),
    "lift": None,
    "cmd": np.zeros((4, 3)),
    "c": np.ones(4),
    "xy0": None,
}


def tau_t13(pl):
    """内容: 試行 13。白矢印は円錐付き指令 GRF。"""
    dt = pl.sim_dt
    if st13["lift"] is None:
        st13["lift"] = pl.feet_pos_world().copy()
        st13["z0"] = float(pl.base_pos()[2])
        st13["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    duty, freq, step_h = DUTY13, FREQ13, H13
    phi = (st13["phi"] + dt * freq) % 1.0
    c = (phi < duty).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st13["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st13["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st13["z0"] - pos[2]) - KD_Z * velb[2],
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
        if st13["prev"][i] > 0.5 and c[i] < 0.5:
            st13["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
            wsw = freq / max(1e-6, 1.0 - duty)
            p_d = st13["lift"][i].copy()
            p_d[2] = st13["lift"][i, 2] + step_h * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
            tau[sl] = Jleg.T @ (KP_S13 * (p_d - feet[i]) + KD_S13 * (v_d - fvel[i]))
    st13["phi"], st13["prev"], st13["c"], st13["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant13 = MujocoGo2(scene="flat", seed=0)
path13 = render_rollout_gif(
    plant13,
    Path("assets/05g_inplace_trot_20s_f122.gif"),
    n_steps=int(round(T_20 / plant13.sim_dt)),
    capture_every=80,
    tau_fn=tau_t13,
    command_grf=lambda pl: st13["cmd"],
    extra_lines=lambda pl: [
        f"c={st13['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05g  1.22Hz/0.80/5cm  ks180  white=command GRF",
)
print(path13.resolve(), path13.stat().st_size, "bytes", "t", float(plant13.data.time))
display(Image(filename=str(path13)))
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（試行 13）

数値は上のセル。hold が 20 s 未満なら 05 は未成功のまま。GIF `05g`。旧 GIF は消さない。06 へは進まない。

## 次の仮説

倒れる時刻の直前で roll / pitch / xy のどれが先に閾値を超えるかを見て、同じ wrench のゲインだけを一つ変える。NMPC と着地点の先送りは足さない。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
