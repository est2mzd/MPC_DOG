"""Append 05 walk-criteria cells. Do not import from notebooks. Delete after use."""
from __future__ import annotations

import json
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"

OLD_SUCCESS = """### 成功条件（数値。ここが合否）

[README §3.8](../docs/block-curriculum/00_README.md)。**連続 5.0 秒以上、全ステップ。** 一瞬の接地や「数歩」は不合格。符号を逆にして足が上がらない静止も不合格である。

\\(T=7.0\\,\\mathrm{s}\\)。\\(xy_0,z_0\\) は走行開始時。

| 量 | 条件 |
|---|---|
| \\(\\lvert\\mathrm{roll}\\rvert,\\lvert\\mathrm{pitch}\\rvert\\) | \\(<0.35\\,\\mathrm{rad}\\)、連続 \\(\\ge 5.0\\,\\mathrm{s}\\) |
| \\(\\lVert xy-xy_0\\rVert\\) | \\(<0.30\\,\\mathrm{m}\\) |
| ベース \\(z\\) | \\(>0.20\\,\\mathrm{m}\\) |
| `qpos` | 有限 |
| 各脚の実測空中（`contact_on=0`） | \\(\\ge 0.15\\,\\mathrm{s}\\)（指令どおり足が床を離れる） |
| 対角ペア指令 | FL+RR のみ、FR+RL のみ、の両方が出現 |
| 結果 | GIF 1 本以上。指令 GRF（白）と実接触。失敗走行の GIF は消さない |

トロット offset は上流 PyMPC と同じ \\([0.5,1.0,1.0,0.5]\\)（FL, FR, RL, RR）。周波数・duty は EqualShare が 2 脚支持で倒れない範囲で選ぶ。満たすまで 06 へ進まない。
"""

NEW_SUCCESS = r"""### 成功条件（数値。ここが合否）

[README §3.8](../docs/block-curriculum/00_README.md)。**連続 5.0 秒以上、全ステップ。** 一瞬の接地や「数歩」は不合格。

**合否はこの表である。** 試行 1–4 は空中 \(0.15\,\mathrm{s}\) だけで足踏みと書いた。実測リフトは数 mm で、映像は直立静止に見える。旧数字は消さないが、合格には使わない。

\(T=7.0\,\mathrm{s}\)。\(xy_0,z_0\) は走行開始時。

| 量 | 条件 |
|---|---|
| \(\lvert\mathrm{roll}\rvert,\lvert\mathrm{pitch}\rvert\) | \(<0.35\,\mathrm{rad}\)、連続 \(\ge 5.0\,\mathrm{s}\) |
| \(\lVert xy-xy_0\rVert\) | \(<0.30\,\mathrm{m}\) |
| ベース \(z\) | \(>0.18\,\mathrm{m}\) |
| `qpos` | 有限 |
| 各脚の遊脚リフト `swing_dz` | \(\ge 0.020\,\mathrm{m}\)（指令遊脚中の足先 \(z\) − 離地 \(z\)） |
| 各脚の実測空中（`contact_on=0`） | \(\ge 0.40\,\mathrm{s}\)（\(T=7\,\mathrm{s}\) の合計） |
| 対角ペア指令 | FL+RR のみ、FR+RL のみ、の両方が出現 |
| 結果 | GIF。指令 GRF（白）と実接触。足が床を離れるのが分かること。失敗 GIF は消さない |

duty を \(0.96\) まで上げて空中 \(0.15\,\mathrm{s}\) だけ稼ぐのは、新判定では不合格である。満たすまで 06 へ進まない。

### 旧判定（試行 1–4 が使った数字。合格には使わない）

| 量 | 旧条件 |
|---|---|
| 各脚の実測空中 | \(\ge 0.15\,\mathrm{s}\) |
| ベース \(z\) | \(>0.20\,\mathrm{m}\) |
| 遊脚リフト | 測っていなかった |

トロット offset は上流 PyMPC と同じ \([0.5,1.0,1.0,0.5]\)（FL, FR, RL, RR）。
"""

OLD_THINK = """2 脚の対角支持は静力学的に細い。duty を高くして overlap を残し、遊脚時間を短くする。\\(h\\) を遊脚に足すと脚が伸びたままになり、接触が切れない。逆符号 \\( -J^{\\top}F_{ee}\\) も同様に「立ったまま」になる。
"""

NEW_THINK = r"""2 脚の対角支持は静力学的に細い。duty を \(0.96\) まで上げると hold は 5 秒を超えるが、遊脚は数十 ms の瞬きになり、足先は数 mm しか上がらない。映像は直立静止である。足踏みにするなら duty を下げ、2 脚のロール・ピッチを立脚の力分配で抑える。\(h\) を遊脚に足すと脚が伸びたまま接触が切れない。逆符号 \(-J^{\top}F_{ee}\) も同様に立ったままになる。
"""

OLD_HYP = """1. 遊脚を \\(h\\) だけ（直交 PD なし）にすると、足は軌道を追わず転ぶ
2. \\(\\tau=-J^{\\top}F_{ee}\\) だと空中時間がほぼ 0 で、5 秒立っても足踏みではない
3. \\(+J^{\\top}\\) でも \\(h\\) 付き・高ゲインだと足が飛びすぎて倒れる
4. \\(+J^{\\top}\\)、遊脚に \\(h\\) を足さない、duty \\(0.96\\)、\\(k_p^{sw}=300\\) なら接触が切れ、かつ 5 秒持つ
"""

NEW_HYP = r"""1. 遊脚を \(h\) だけ（直交 PD なし）にすると、足は軌道を追わず転ぶ
2. \(\tau=-J^{\top}F_{ee}\) だと空中時間がほぼ 0 で、5 秒立っても足踏みではない
3. \(+J^{\top}\) でも \(h\) 付き・高ゲインだと足が飛びすぎて倒れる
4. \(+J^{\top}\)、遊脚に \(h\) を足さない、duty \(0.96\) なら旧判定（空中 \(0.15\,\mathrm{s}\)）は通る。リフトは数 mm のままである
5. 試行 4 を新判定で測ると、リフトと空中時間で落ちる
6. EqualShare のまま duty \(0.75\)・\(h_{\mathrm{step}}=5\,\mathrm{cm}\) にすると、2 脚支持が持たず転ぶ
7. 立脚を瞬間 wrench 最小二乗（\(F_z\) と小さい \(F_{xy}\)、\(M_x,M_y\)）にすると、同じ遊脚で hold \(\ge 5\,\mathrm{s}\) かつ各脚リフト \(\ge 2\,\mathrm{cm}\)
8. これはホライズン NMPC ではない。今接地している脚だけに力を分配する
"""

OLD_MATH_END = """仮想仕事の符号は立脚の \\(-J^{\\top}F\\)（床から足への力）と、遊脚の \\(+J^{\\top}F_{ee}\\)（足を目標へ加速する力）で逆である。
"""

NEW_MATH_END = r"""仮想仕事の符号は立脚の \(-J^{\top}F\)（床から足への力）と、遊脚の \(+J^{\top}F_{ee}\)（足を目標へ加速する力）で逆である。

試行 6 以降の立脚は EqualShare ではなく、今の立脚集合に対する瞬間最小二乗である（ホライズンなし）。

$$
A_i=\begin{bmatrix}I_3\\ [r_i]_{\times}\end{bmatrix},\quad
r_i=p_i-p_{\mathrm{com}},\quad
\min_f\Bigl\lVert \bigl[A_{i\in\mathcal{S}}\bigr] f - \begin{bmatrix}F^{\mathrm{des}}\\ M^{\mathrm{des}}\end{bmatrix}\Bigr\rVert
$$

$$
F^{\mathrm{des}}=\begin{bmatrix}-k_{xy}(x-x_0)-d_{xy}\dot x\\ -k_{xy}(y-y_0)-d_{xy}\dot y\\ mg+k_z(z_0-z)-d_z\dot z\end{bmatrix},\qquad
M^{\mathrm{des}}=\begin{bmatrix}-k_a\,\mathrm{roll}-d_a\omega_x\\ -k_a\,\mathrm{pitch}-d_a\omega_y\\ 0\end{bmatrix}
$$

\(f\) は立脚 GRF を縦に積んだベクトル。\(F_{z,i}<0\) なら 0 に切る。遊脚の式は試行 4 と同じ（\(h\) を足さない \(+J^{\top}\)）。
"""


def md_replace(src: str, old: str, new: str, label: str) -> str:
    if old not in src:
        raise SystemExit(f"pattern not found: {label}")
    return src.replace(old, new, 1)


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("新判定（試行 5 以降）" in "".join(c.source) for c in nb.cells):
        print("already patched, skip structure")
        return

    c0 = "".join(nb.cells[0].source)
    c0 = md_replace(c0, OLD_SUCCESS, NEW_SUCCESS, "success")
    c0 = md_replace(c0, OLD_THINK, NEW_THINK, "think")
    c0 = md_replace(c0, OLD_HYP, NEW_HYP, "hyp")
    c0 = md_replace(c0, OLD_MATH_END, NEW_MATH_END, "math")
    nb.cells[0].source = c0

    c8 = "".join(nb.cells[8].source)
    if "旧判定" not in c8:
        nb.cells[8].source = (
            c8.rstrip()
            + "\n\n"
            + "当時の合格条件は空中 \(0.15\\,\\mathrm{s}\\) だった。リフトは見ていない。試行 5 で新判定すると落ちる。セルと GIF `05b` は消さない。\n"
        )

    nb.cells[10].source = r"""## 途中分析（試行 1–4）

- 試行 1: 遊脚 PD が無いと軌道を追えない
- 試行 2: \(-J^{\top}\) は空中時間がほぼ 0。hold が長くても足踏みではない
- 試行 3: 符号は合っていても \(h\) と大きい \(K_p\) で転倒。GIF `05a` を残す
- 試行 4: 旧判定（空中 \(0.15\,\mathrm{s}\)、duty \(0.96\)）は hold \(6.712\,\mathrm{s}\) で通った。GIF `05b`。`swing_dz` は数 mm なので、新判定では不合格である

次のセルから新判定で測り直す。
"""

    extra = [
        new_markdown_cell(
            r"""## 試行 5 — 試行 4 を新判定で測る（仮説 5）

制御は試行 4 のまま（EqualShare、duty \(0.96\)、`step_h=2\,\mathrm{cm}`）。変えるのは合否だけ。失敗を消さず、旧 PASS を新 FAIL として残す。
"""
        ),
        new_code_cell(
            r"""# --- このセルの意図 ---
# 試行 4 と同じ rollout を、背景の新判定（リフト 2cm、空中 0.40s）で採点する。

# --- 新判定。旧 AIR_S=0.15 は試行 4 の assert 用に残してある ---
LIFT_NEED = 0.020  # 内容: 各脚、指令遊脚中の足先 z 上昇の下限 [m]
AIR_WALK = 0.40    # 内容: 各脚の実測空中の下限 [s]。T=7s の合計
Z_MIN_WALK = 0.18  # 内容: ベース高さの下限 [m]。旧 0.20 より遊脚中の沈みを認める


def walks(r, lift_need=LIFT_NEED, air_need=AIR_WALK):
    '''内容: 4脚ともリフトと空中が下限以上なら True。胴体 hold とは別。'''
    return float(r["swing_dz"].min()) >= lift_need and float(r["meas_air"].min()) >= air_need


r5 = rollout(swing_sign=+1.0, add_h_swing=False, kp_s=KP_SW, kd_s=KD_SW, duty=DUTY_OK, step_h=STEP_H)
print("trial5  (same as trial4) hold", r5["hold"], "walk", walks(r5))
print("  meas_air", r5["meas_air"].round(3), "swing_dz", r5["swing_dz"].round(4))
print("  pair_a", r5["pair_a_s"], "pair_b", r5["pair_b_s"])
old_pass = r5["hold"] >= HOLD_S and float(r5["meas_air"].min()) >= AIR_S
new_pass = r5["hold"] >= HOLD_S and walks(r5)
print("  old_criteria PASS", old_pass, " new_criteria PASS", new_pass)
assert old_pass, "trial4 controller should still pass the old air/hold numbers"
assert not new_pass, "mm-scale lift must fail the new walk criteria"
print("05 trial5 FAIL under new walk criteria (expected). GIF 05b kept.")
"""
        ),
        new_markdown_cell(
            r"""## 試行 6 — EqualShare のまま実遊脚（仮説 6）

duty \(0.75\)、\(h_{\mathrm{step}}=5\,\mathrm{cm}\)、\(f=1.35\,\mathrm{Hz}\)（PyMPC trot に近い）。立脚はまだ \(F_z=F_z^{\mathrm{tot}}/n_s\)、横力もモーメントも 0。2 脚支持の細い静力学を、短い遊脚で隠さない。
"""
        ),
        new_code_cell(
            r"""# --- このセルの意図 ---
# EqualShare + 実遊脚。hold が 5s 未満、または転倒することを残す。

r6 = rollout(
    swing_sign=+1.0,
    add_h_swing=False,
    kp_s=350.0,
    kd_s=18.0,
    duty=0.75,
    step_h=0.05,
    freq=1.35,
)
print("trial6  equal+real-swing  hold", r6["hold"], "walk", walks(r6))
print("  zmin", r6["zmin"], "rpy", r6["maxr"], "xy", r6["maxxy"])
print("  meas_air", r6["meas_air"].round(3), "swing_dz", r6["swing_dz"].round(4))
assert r6["hold"] < HOLD_S or not walks(r6)
print("05 trial6 FAIL expected: EqualShare cannot hold a real trot swing")

# --- 失敗 GIF。直立に見えない転倒を残す ---
st6 = {"phi": TROT_OFF.copy(), "prev": np.ones(4), "lift": None, "cmd": np.zeros((4, 3)), "c": np.ones(4)}


def tau_equal_real(pl):
    '''内容: 試行 6 と同じ EqualShare + sine 遊脚。白矢印は指令 Fz。'''
    dt = pl.sim_dt
    if st6["lift"] is None:
        st6["lift"] = pl.feet_pos_world().copy()
        st6["z0"] = float(pl.base_pos()[2])
    geoms = pl.foot_geom_ids()
    duty, freq, step_h = 0.75, 1.35, 0.05
    phi = (st6["phi"] + dt * freq) % 1.0
    c = (phi < duty).astype(float)
    ns = max(float(c.sum()), 1.0)
    z = float(pl.base_pos()[2])
    vz = float(pl.base_lin_vel_world()[2])
    fz_tot = pl.mass_kg * 9.81 + KP_Z * (st6["z0"] - z) - KD_Z * vz
    feet = pl.feet_pos_world()
    vel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st6["prev"][i] > 0.5 and c[i] < 0.5:
            st6["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            F = np.array([0.0, 0.0, fz_tot / ns])  # 内容: EqualShare。Mx,My を持たない
            cmd[i] = F
            tau[sl] = h[sl] - Jleg.T @ F
        else:
            s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
            wsw = freq / max(1e-6, 1.0 - duty)
            p_d = st6["lift"][i].copy()
            p_d[2] = st6["lift"][i, 2] + step_h * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
            F_ee = 350.0 * (p_d - feet[i]) + 18.0 * (v_d - vel[i])
            tau[sl] = Jleg.T @ F_ee
    st6["phi"], st6["prev"], st6["c"], st6["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant6 = MujocoGo2(scene="flat", seed=0)
path6 = render_rollout_gif(
    plant6,
    Path("assets/05c_equalshare_real_swing_flip.gif"),
    n_steps=int(round(T_RUN / plant6.sim_dt)),
    capture_every=80,
    tau_fn=tau_equal_real,
    command_grf=lambda pl: st6["cmd"],
    extra_lines=lambda pl: [
        f"c={st6['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05c EqualShare + real swing  flip  white=command GRF",
)
print(path6.resolve(), path6.stat().st_size, "bytes")
display(Image(filename=str(path6)))
"""
        ),
        new_markdown_cell(
            r"""## 試行 7 — 瞬間 wrench、まだ足りない遊脚（仮説 7 の手前）

立脚を最小二乗にする。duty \(0.80\)、`step_h=4.5\,\mathrm{cm}`。hold は 5 秒近くても、どれかの脚のリフトが \(2\,\mathrm{cm}\) 未満なら不合格のままにする。
"""
        ),
        new_code_cell(
            r"""# --- このセルの意図 ---
# 立脚 wrench を足す。遊脚が短い組はリフト不足で落とす。

def wrench_grf(feet, com, c, F_des, M_des):
    '''内容: 立脚 i の 6x3 ブロックを横に並べ、[F;M] を最小二乗で脚力にする。NMPC ではない。'''
    idx = [i for i in range(4) if c[i] > 0.5]
    grf = np.zeros((4, 3))
    if not idx:
        return grf
    blocks = []
    for i in idx:
        r = feet[i] - com  # 内容: CoM から足への位置ベクトル [m]
        Ai = np.zeros((6, 3))
        Ai[0:3, :] = np.eye(3)
        Ai[3:6, :] = np.array(
            [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]]
        )
        blocks.append(Ai)
    A = np.hstack(blocks)
    b = np.concatenate([F_des, M_des])
    f, *_ = np.linalg.lstsq(A, b, rcond=None)
    for k, i in enumerate(idx):
        grf[i] = f[3 * k : 3 * k + 3]
        if grf[i, 2] < 0.0:
            grf[i, 2] = 0.0  # 内容: 足は床を引けない
    return grf


def rollout_wrench(duty, step_h, freq, kp_s=350.0, kd_s=18.0, kp_xy=80.0, kd_xy=20.0, kp_att=40.0, kd_att=4.0, T=T_RUN):
    '''意図: 立脚は wrench LS、遊脚は +J^T PD。新判定用の辞書を返す。'''
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
        wbody = np.asarray(p.data.qvel[3:6], dtype=np.float64)  # 内容: ベース角速度。MuJoCo qvel[3:6]
        F_des = np.array(
            [
                -kp_xy * (pos[0] - xy0[0]) - kd_xy * velb[0],
                -kp_xy * (pos[1] - xy0[1]) - kd_xy * velb[1],
                mg + KP_Z * (z0 - pos[2]) - KD_Z * velb[2],
            ]
        )
        M_des = np.array(
            [
                -kp_att * rpy[0] - kd_att * wbody[0],
                -kp_att * rpy[1] - kd_att * wbody[1],
                0.0,
            ]
        )
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
                tau[sl] = h[sl] - Jleg.T @ grf[i]  # 内容: 02 と同じ MapJT。F だけ wrench
            else:
                s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
                wsw = freq / max(1e-6, 1.0 - duty)
                p_d = lift[i].copy()
                p_d[2] = lift[i, 2] + step_h * np.sin(np.pi * s)
                v_d = np.zeros(3)
                v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
                F_ee = kp_s * (p_d - feet[i]) + kd_s * (v_d - fvel[i])
                tau[sl] = Jleg.T @ F_ee  # 内容: 遊脚に h を足さない
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


r7 = rollout_wrench(duty=0.80, step_h=0.045, freq=1.2)
print("trial7  wrench d0.80 h0.045  hold", r7["hold"], "walk", walks(r7))
print("  zmin", r7["zmin"], "rpy", r7["maxr"], "xy", r7["maxxy"])
print("  meas_air", r7["meas_air"].round(3), "swing_dz", r7["swing_dz"].round(4))
assert not (r7["hold"] >= HOLD_S and walks(r7))
print("05 trial7 FAIL expected: hold or lift still short of the new table")
"""
        ),
        new_markdown_cell(
            r"""## 試行 8 — 瞬間 wrench + 実トロット遊脚（仮説 7）

duty \(0.75\)、\(h_{\mathrm{step}}=5\,\mathrm{cm}\)、\(f=1.35\,\mathrm{Hz}\)。立脚は試行 7 と同じ wrench。遊脚は \(+J^{\top}\) のみ。これが新判定の合格組である。
"""
        ),
        new_code_cell(
            r"""# --- このセルの意図 ---
# 新判定の合格組。hold>=5 かつ各脚リフト>=2cm・空中>=0.40s。GIF 05d を残す。

r8 = rollout_wrench(duty=0.75, step_h=0.05, freq=1.35)
print("trial8  wrench d0.75 h0.05 f1.35  hold", r8["hold"], "walk", walks(r8))
print("  zmin", r8["zmin"], "rpy", r8["maxr"], "xy", r8["maxxy"])
print("  meas_air", r8["meas_air"].round(3), "swing_dz", r8["swing_dz"].round(4))
print("  pair_a", r8["pair_a_s"], "pair_b", r8["pair_b_s"])
assert r8["hold"] >= HOLD_S, f"hold {r8['hold']:.3f}s"
assert walks(r8), f"air {r8['meas_air']} dz {r8['swing_dz']}"
assert r8["pair_a_s"] > 0.0 and r8["pair_b_s"] > 0.0
print("05 PASS under new walk criteria: hold", r8["hold"], "s  dz", r8["swing_dz"].round(3))

st8 = {
    "phi": TROT_OFF.copy(),
    "prev": np.ones(4),
    "lift": None,
    "cmd": np.zeros((4, 3)),
    "c": np.ones(4),
    "xy0": None,
}


def tau_wrench_ok(pl):
    '''内容: 試行 8 と同じ。白矢印は wrench が決めた立脚 GRF。'''
    dt = pl.sim_dt
    if st8["lift"] is None:
        st8["lift"] = pl.feet_pos_world().copy()
        st8["z0"] = float(pl.base_pos()[2])
        st8["xy0"] = pl.base_pos()[:2].copy()
    geoms = pl.foot_geom_ids()
    duty, freq, step_h = 0.75, 1.35, 0.05
    phi = (st8["phi"] + dt * freq) % 1.0
    c = (phi < duty).astype(float)
    pos = pl.base_pos()
    velb = pl.base_lin_vel_world()
    rpy = pl.base_rpy()
    wbody = np.asarray(pl.data.qvel[3:6], dtype=np.float64)
    mg = pl.mass_kg * 9.81
    F_des = np.array(
        [
            -80.0 * (pos[0] - st8["xy0"][0]) - 20.0 * velb[0],
            -80.0 * (pos[1] - st8["xy0"][1]) - 20.0 * velb[1],
            mg + KP_Z * (st8["z0"] - pos[2]) - KD_Z * velb[2],
        ]
    )
    M_des = np.array([-40.0 * rpy[0] - 4.0 * wbody[0], -40.0 * rpy[1] - 4.0 * wbody[1], 0.0])
    feet = pl.feet_pos_world()
    fvel = pl.feet_vel_world()
    h = np.asarray(pl.data.qfrc_bias[6:], dtype=np.float64)
    grf = wrench_grf(feet, pl.com_world(), c, F_des, M_des)
    tau = np.zeros(12)
    cmd = np.zeros((4, 3))
    for i in range(4):
        if st8["prev"][i] > 0.5 and c[i] < 0.5:
            st8["lift"][i] = feet[i].copy()
        Jleg = jac_leg(pl.model, pl.data, geoms, i)
        sl = slice(3 * i, 3 * i + 3)
        if c[i] > 0.5:
            cmd[i] = grf[i]
            tau[sl] = h[sl] - Jleg.T @ grf[i]
        else:
            s = float(np.clip((phi[i] - duty) / max(1e-6, 1.0 - duty), 0.0, 1.0))
            wsw = freq / max(1e-6, 1.0 - duty)
            p_d = st8["lift"][i].copy()
            p_d[2] = st8["lift"][i, 2] + step_h * np.sin(np.pi * s)
            v_d = np.zeros(3)
            v_d[2] = step_h * np.pi * np.cos(np.pi * s) * wsw
            F_ee = 350.0 * (p_d - feet[i]) + 18.0 * (v_d - fvel[i])
            tau[sl] = Jleg.T @ F_ee
    st8["phi"], st8["prev"], st8["c"], st8["cmd"] = phi, c, c, cmd
    return clip_torque(tau, pl.model.actuator_ctrlrange)


plant8 = MujocoGo2(scene="flat", seed=0)
path8 = render_rollout_gif(
    plant8,
    Path("assets/05d_inplace_trot_lift.gif"),
    n_steps=int(round(T_RUN / plant8.sim_dt)),
    capture_every=80,
    tau_fn=tau_wrench_ok,
    command_grf=lambda pl: st8["cmd"],
    extra_lines=lambda pl: [
        f"c={st8['c'].astype(int).tolist()}  z={pl.base_pos()[2]:.3f}  contact={pl.contact_on().astype(int).tolist()}"
    ],
    title="05d real in-place trot  lift>=2cm  white=command GRF",
)
print(path8.resolve(), path8.stat().st_size, "bytes", "t", float(plant8.data.time))
display(Image(filename=str(path8)))
"""
        ),
        new_markdown_cell(
            r"""## 結果と分析

- 試行 1–3: 符号と \(h\) の失敗。GIF `05a` は残す
- 試行 4: 旧判定 PASS。duty \(0.96\) で空中 \(0.25\,\mathrm{s}\)、リフト \(4\,\mathrm{mm}\)。映像は直立。GIF `05b`
- 試行 5: 同じ制御を新判定で FAIL。空中下限を \(0.40\,\mathrm{s}\)、リフト下限を \(2\,\mathrm{cm}\) にしたため
- 試行 6: EqualShare + 実遊脚は転ぶ。GIF `05c`。2 脚の \(F_z\) 等分だけではロール・ピッチが持てない
- 試行 7: wrench でも遊脚が短いとリフトが \(2\,\mathrm{cm}\) に届かない
- 試行 8: wrench + duty \(0.75\) + \(5\,\mathrm{cm}\) で hold \(\ge 5\,\mathrm{s}\) かつ各脚が床を離れる。GIF `05d`

EqualShare のまま duty を上げて「足踏み」と書くのは、この段の現象ではない。瞬間 wrench は予測のホライズンを持たない。NMPC はまだ呼ばない。

## 次の仮説

同じ wrench 立脚のまま、offset / freq / duty だけ変えるとペースやクロールになる。それが S4（06）である。06 以降の旧セルは、05 試行 4 と同じ直立ループなので、新判定では不合格のまま残し、続きを書く。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
