#!/usr/bin/env python3
"""Step 17 ジャンプの CSV を計測サマリにする。

入力: quadsdk_step17_jump.py が書く state_log.csv
出力: 標準出力に、ジャンプ 1 回のタイムラインと計測値・簡易判定。

「その場ジャンプ」の要件で見るもの:
  - 四脚が実際に地面から離れたか(足先 z で判定)、その時間 >= 30 ms
  - PRELOAD/REAR_PUSH 中に後脚が荷重を受けたか(NMPC GRF_z)
  - 前脚が後脚より先に離地したか
  - 胴体重心の上昇量(ホップ高さ)
  - 後脚 BL/BR の前進距離(踏切前 x → 着地安定後 x)
  - 着地後 2 s 転倒しない(base_z>=0.15, |roll|,|pitch|<0.6)
"""
from __future__ import annotations
import csv
import sys
from pathlib import Path

LEGS = ["FL", "BL", "FR", "BR"]
FRONT, REAR = ["FL", "FR"], ["BL", "BR"]
FOOT_AIR_Z = 0.06     # 足先 z がこれを超えたら離地とみなす
FALL_Z = 0.15
TILT = 0.6
MIN_FLIGHT_S = 0.030


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main() -> None:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "state_log.csv")
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print(f"no rows in {path}")
        return
    t = [f(r["sim_time_s"]) for r in rows]
    n = len(rows)
    bz = [f(r["base_pos_z_m"]) for r in rows]
    roll = [f(r["base_roll_rad"]) for r in rows]
    pitch = [f(r["base_pitch_rad"]) for r in rows]
    phase = [r.get("jump_phase") or "" for r in rows]
    footz = {l: [f(r[f"foot_{l}_pos_z_m"]) for r in rows] for l in LEGS}
    footx = {l: [f(r[f"foot_{l}_pos_x_m"]) for r in rows] for l in LEGS}
    grfz = {l: [f(r.get(f"grf_{l}_z_N")) or 0.0 for r in rows] for l in LEGS}
    grfx = {l: [f(r.get(f"grf_{l}_x_N")) or 0.0 for r in rows] for l in LEGS}

    print(f"file: {path}\nrows: {n}  span: {t[0]:.2f}..{t[-1]:.2f} s")

    # --- jump window: first PRELOAD/REAR_PUSH/leap to first index the plan is
    #     back to a settle/connect phase for good ---
    jstart = next((i for i in range(n) if phase[i] in
                   ("preload", "rear_push", "leap_stance")), None)
    if jstart is None:
        print("\n[NG] plan never entered a jump phase (preload/rear_push).")
        return
    print(f"\njump plan starts @ {t[jstart]:.2f} s (phase={phase[jstart]})")
    seen = {}
    for i in range(jstart, n):
        p = phase[i]
        if p and p not in seen:
            seen[p] = t[i]
    for p, ts in seen.items():
        print(f"  {p:12s} first @ {ts:.2f} s")

    def airborne(i):
        return all((footz[l][i] is not None and footz[l][i] > FOOT_AIR_Z)
                   for l in LEGS)

    # longest all-4-airborne window after jstart
    best, cs = 0.0, None
    for i in range(jstart, n):
        if airborne(i) and cs is None:
            cs = t[i]
        elif not airborne(i) and cs is not None:
            best = max(best, t[i] - cs)
            cs = None
    if cs is not None:
        best = max(best, t[-1] - cs)
    ok = best >= MIN_FLIGHT_S
    print(f"\n[{'OK' if ok else 'NG'}] longest all-4-feet-airborne: "
          f"{best*1000:.0f} ms  (foot z > {FOOT_AIR_Z} m; need >= 30)")

    # front vs rear liftoff order
    def pair_air_time(pair):
        for i in range(jstart, n):
            if all(footz[l][i] is not None and footz[l][i] > FOOT_AIR_Z
                   for l in pair):
                return t[i]
        return None
    ft, rt = pair_air_time(FRONT), pair_air_time(REAR)
    if ft is not None and rt is not None:
        print(f"[{'OK' if ft <= rt + 1e-6 else 'NG'}] front feet leave @ "
              f"{ft:.2f}s, rear feet leave @ {rt:.2f}s")
    else:
        print(f"[NG] a foot pair never left the ground  front={ft} rear={rt}")

    # hop height
    z_pre = bz[jstart]
    z_apex = max(v for v in bz[jstart:] if v is not None)
    apex_i = bz.index(z_apex)
    print(f"\nbody z: {z_pre:.3f} -> apex {z_apex:.3f} m @ {t[apex_i]:.2f}s "
          f"(rise {z_apex - z_pre:+.3f} m)")

    # rear GRF during preload+rear_push (NMPC output)
    push_idx = [i for i in range(jstart, n)
                if phase[i] in ("preload", "rear_push", "leap_stance")]
    if push_idx:
        rp = max(grfz["BL"][i] + grfz["BR"][i] for i in push_idx)
        fp = max(grfz["FL"][i] + grfz["FR"][i] for i in push_idx)
        rx = max(grfx["BL"][i] + grfx["BR"][i] for i in push_idx)
        print(f"push-phase GRF_z peak: rear(BL+BR) {rp:.0f} N, "
              f"front(FL+FR) {fp:.0f} N; rear GRF_x peak {rx:+.0f} N")
        rpush = [i for i in push_idx if phase[i] == "rear_push"]
        if rpush:
            rr = max(grfz["BL"][i] + grfz["BR"][i] for i in rpush)
            fr = max(grfz["FL"][i] + grfz["FR"][i] for i in rpush)
            print(f"  during REAR_PUSH only: rear {rr:.0f} N vs front {fr:.0f} N "
                  f"[{'OK rear-dominant' if rr > fr else 'NG'}]")

    # rear foot forward travel: mean(BL,BR) x pre-jump vs settled-after
    def mean_rear_x(i):
        vs = [footx[l][i] for l in REAR if footx[l][i] is not None]
        return sum(vs) / len(vs) if vs else None
    rx_pre = mean_rear_x(jstart)
    # settled: last row where |roll|,|pitch| small and not falling
    settle_i = None
    for i in range(apex_i, n):
        if (bz[i] is not None and bz[i] > 0.20 and
                abs(roll[i] or 9) < 0.3 and abs(pitch[i] or 9) < 0.3):
            settle_i = i
    rx_post = mean_rear_x(settle_i) if settle_i is not None else None
    if rx_pre is not None and rx_post is not None:
        print(f"\nREAR foot mean x: {rx_pre:.3f} m (pre) -> {rx_post:.3f} m "
              f"(settled @ {t[settle_i]:.2f}s)  travel {rx_post - rx_pre:+.3f} m")
    else:
        print(f"\nREAR foot travel: cannot measure "
              f"(pre={rx_pre}, settled row found={settle_i is not None})")

    # fall / tilt after jump
    fall_t = next((t[i] for i in range(jstart, n)
                   if bz[i] is not None and bz[i] < FALL_Z), None)
    tilt_t = next((t[i] for i in range(jstart, n)
                   if abs(roll[i] or 0) > TILT or abs(pitch[i] or 0) > TILT),
                  None)
    print(f"[{'NG' if fall_t else 'OK'}] base_z < {FALL_Z}: "
          + (f"@ {fall_t:.2f}s" if fall_t else "never"))
    print(f"[{'NG' if tilt_t else 'OK'}] |roll|/|pitch| > {TILT}: "
          + (f"@ {tilt_t:.2f}s" if tilt_t else "never"))
    mr = max(abs(v) for v in roll[jstart:] if v is not None)
    mp = max(abs(v) for v in pitch[jstart:] if v is not None)
    print(f"     max |roll| {mr:.2f}, max |pitch| {mp:.2f} rad (after jump start)")

    # torque / joint speed envelope
    taus, jvel = [], []
    for r in rows:
        for j in range(12):
            v = f(r.get(f"joint_{j}_cmd_torque_Nm"))
            if v is not None:
                taus.append(abs(v))
            w = f(r.get(f"joint_{j}_vel_radps"))
            if w is not None:
                jvel.append(abs(w))
    if taus:
        taus.sort(); jvel.sort()
        print(f"\ncmd torque |Nm| max {taus[-1]:.1f} p99 {taus[int(len(taus)*.99)]:.1f}"
              f" | joint speed |rad/s| max {jvel[-1]:.1f} p99 {jvel[int(len(jvel)*.99)]:.1f}")


if __name__ == "__main__":
    main()
