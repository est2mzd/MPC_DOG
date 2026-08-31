"""Step 04: 前進方向に 2 m 間隔・幅 5 m・長さ 30 cm の穴(トレンチ)を並べた
平面マップで、穴に落ちずに前進できるかを記録するハーネス。

マップ: `src/trial/assets/scene_gaps_1p5.xml`
  (生成: `src/trial/assets/gen_scene_gaps.py <depth> 1.5 scene_gaps_1p5.xml`)
  - y ∈ [-2.5, 2.5] の 5 m 幅の通路。x 方向に 1.2 m の凸条(box, 上面 z=0)を並べ、
    間に 0.30 m 幅・深さ 5 cm のトレンチ(= 穴)を **1.5 m 間隔**で作る
    (Step 03 は 2.0 m 間隔・凸条 1.7 m。Step 04 は間隔を詰めて安全地帯を狭くした)。
    トレンチ底は連続スラブ(z = -0.05)なので「底なしの穴」ではなく
    「5 cm 深・30 cm 長・5 m 幅の轍(わだち)」。
  - 穴の中心は x = 0.75, 2.25, 3.75, ...(凸条の中心は x = 0, 1.5, 3.0, ...)。
  - ロボットは y=0(穴の横中央)を +x 方向に歩く。
  - 底なしの深い穴は blind の Quadruped-PyMPC では最初の穴で転落する
    (足場回避をしないため。Step 03 参照)。浅い轍にすると踏み越えて前進できる。
実行時に gym_quadruped の robot_model/scene_gaps_1p5.xml へこの XML をコピーし、
`QuadrupedEnv(scene="gaps_1p5", ...)` で読み込ませる(external/ は変更しない)。

制御は Step 02 と同じく `compute_actions()` の引数・順序を一切変えず呼び出す。
Quadruped-PyMPC はデフォルト `visual_foothold_adaptation="blind"`(地形非考慮)で、
穴回避の足場修正は行わない。したがって「穴を跨いで着地する」かどうかは
歩容周波数と前進速度で決まる歩幅に依存する(Step 02 の知見の延長)。

環境変数: STEP04_VEL / STEP04_FREQ / STEP04_SECONDS で上書き可。
実行: `bash scripts/trial/run_step_04.sh`
"""

from __future__ import annotations

import copy
import csv
import json
import os
import shutil
import time
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import gym_quadruped
import mujoco
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as qpympc_cfg
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper
from gym_quadruped.quadruped_env import QuadrupedEnv


# ============================================================================
# 記録パラメータ(MPC_DOG側の設定。external/ 側の値は一切変更しない)
# ============================================================================
NUM_SECONDS = float(os.environ.get("STEP04_SECONDS", "20"))     # 記録するシミュレーション実時間[s]
INITIAL_FORWARD_VEL_MPS = float(os.environ.get("STEP04_VEL", "0.3"))   # 前進速度指令[m/s]
GAIT_STEP_FREQ_HZ = float(os.environ.get("STEP04_FREQ", "2.0"))  # 歩容の周波数[Hz]。trot既定1.4を上書きする

GAP_SPACING_M = 1.5     # 穴の中心間隔[m](マップ生成 gen_scene_gaps.py の spacing と一致させること)
GAP_LENGTH_M = 0.30     # 穴の x 方向長さ[m]
FIRST_GAP_X_M = 0.75    # 最初の穴の中心 x 座標[m](= spacing/2)

GIF_FPS = 10
GIF_MAX_WIDTH = 480
GIF_MAX_HEIGHT = 270
OVERLAY_FONT = ImageFont.load_default(size=22)
OVERLAY_COLOR = (0, 0, 0)

# 転倒判定(reset はしない。最初に閾値を割った sim 時刻だけ記録する)。
FALL_HEIGHT_THRESHOLD_M = 0.12   # base 高さがこれを割ったら「穴に落ちた/転倒」
FALL_TILT_THRESHOLD_RAD = 0.8    # |roll| または |pitch| がこれを超えたら転倒

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SCENE_SRC = REPO_ROOT / "src" / "trial" / "assets" / "scene_gaps_1p5.xml"
LOG_DIR = REPO_ROOT / "artifacts" / "logs" / "step_04"
GIF_DIR = REPO_ROOT / "artifacts" / "gifs"
SUMMARY_CSV_PATH = LOG_DIR / "trials_summary.csv"


def _stage_scene() -> None:
    """src/trial/assets/scene_gaps_1p5.xml を gym_quadruped の robot_model/ へコピーする。

    QuadrupedEnv は robot_model/scene_<name>.xml が存在すればそれをそのまま読む
    (utils/mujoco/terrain.py: generate_terrain の base_scene_env_path.exists() 分岐)。
    external/ ではなく site-packages への配置なので、毎回上書きコピーして揃える。
    """
    dst = Path(gym_quadruped.__file__).parent / "robot_model" / "scene_gaps_1p5.xml"
    shutil.copyfile(SCENE_SRC, dst)


def _next_trial_id(summary_path: Path) -> int:
    if not summary_path.exists():
        return 1
    with open(summary_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return 1
    return max(int(row["id"]) for row in rows) + 1


def _gaps_crossed(x_start: float, x_now: float) -> int:
    """x_start から x_now までの間に中心を通過した穴の数。"""
    n = 0
    gx = FIRST_GAP_X_M
    while gx <= x_now + 1e-9:
        if gx > x_start:
            n += 1
        gx += GAP_SPACING_M
    return n


def _foot_over_gap(x: float) -> bool:
    """足先 x 座標が穴の x レンジ(±長さ/2)に入っているか。"""
    # 最寄りの穴中心との距離
    k = round((x - FIRST_GAP_X_M) / GAP_SPACING_M)
    gx = FIRST_GAP_X_M + k * GAP_SPACING_M
    return abs(x - gx) <= GAP_LENGTH_M / 2.0


def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    GIF_DIR.mkdir(parents=True, exist_ok=True)
    _stage_scene()

    trial_id = _next_trial_id(SUMMARY_CSV_PATH)
    trial_id_str = f"{trial_id:02d}"
    gif_path = GIF_DIR / f"step_04_{trial_id_str}.gif"

    # Step 02 と同じ生成手順。scene だけ "gaps" にする。
    env = QuadrupedEnv(
        robot=qpympc_cfg.robot,
        scene="gaps_1p5",
        sim_dt=qpympc_cfg.simulation_params["dt"],
        ground_friction_coeff=1.0,  # 穴マップは XML 側で friction 固定。範囲指定はしない
        base_vel_command_type="forward",
        ref_base_lin_vel=INITIAL_FORWARD_VEL_MPS,
    )
    env.mjModel.opt.gravity[2] = -qpympc_cfg.gravity_constant
    env.reset(random=False)

    legs_order = ["FL", "FR", "RL", "RR"]
    heightmaps = None  # blind(Step 02 と同じ。VFA の 'vfa' は非公開・'height' は z のみ)

    qpympc_cfg.simulation_params["gait_params"][qpympc_cfg.simulation_params["gait"]]["step_freq"] = GAIT_STEP_FREQ_HZ

    quadrupedpympc_wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=legs_order,
    )

    tau_soft_limits_scalar = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft_limits_scalar,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft_limits_scalar,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft_limits_scalar,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft_limits_scalar,
    )
    tau = LegsAttr(*[np.zeros(env.mjModel.nu // 4) for _ in range(4)])

    simulation_dt = qpympc_cfg.simulation_params["dt"]
    n_steps = int(NUM_SECONDS // simulation_dt)

    renderer = mujoco.Renderer(env.mjModel, height=360, width=640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(env.mjModel, cam)
    cam.distance = 2.6
    cam.elevation = -12   # ほぼ真横から見て、穴とロボットの上下動が見えるように
    cam.azimuth = 90

    frame_stride = max(1, int(round((1.0 / GIF_FPS) / simulation_dt)))

    frames: list[np.ndarray] = []
    log_rows: list[dict] = []
    fall_time_s = None
    x0 = float(env.base_pos[0])
    min_foot_z = 0.0
    max_gaps = 0

    print(
        f"Step 04: gap crossing  |  v={INITIAL_FORWARD_VEL_MPS} m/s  "
        f"step_freq={GAIT_STEP_FREQ_HZ} Hz  |  gaps: {GAP_LENGTH_M*100:.0f} cm long, "
        f"{GAP_SPACING_M} m apart  |  recording {NUM_SECONDS:.0f}s ({n_steps} steps) ..."
    )
    t_wall_start = time.time()

    for step in range(n_steps):
        feet_pos = env.feet_pos(frame="world")
        feet_vel = env.feet_vel(frame="world")
        hip_pos = env.hip_positions(frame="world")
        base_lin_vel = env.base_lin_vel(frame="world")
        base_ang_vel = env.base_ang_vel(frame="base")
        base_ori_euler_xyz = env.base_ori_euler_xyz
        base_pos = copy.deepcopy(env.base_pos)
        com_pos = copy.deepcopy(env.com)

        ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()

        if qpympc_cfg.simulation_params["use_inertia_recomputation"]:
            inertia = env.get_base_inertia().flatten()
        else:
            inertia = qpympc_cfg.inertia.flatten()

        qpos, qvel = env.mjData.qpos, env.mjData.qvel
        legs_qvel_idx = env.legs_qvel_idx
        legs_qpos_idx = env.legs_qpos_idx
        joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)

        legs_mass_matrix = env.legs_mass_matrix
        legs_qfrc_bias = env.legs_qfrc_bias
        legs_qfrc_passive = env.legs_qfrc_passive

        feet_jac = env.feet_jacobians(frame="world", return_rot_jac=False)
        feet_jac_dot = env.feet_jacobians_dot(frame="world", return_rot_jac=False)

        t0 = time.perf_counter()
        tau = quadrupedpympc_wrapper.compute_actions(
            com_pos, base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel,
            feet_pos, hip_pos, joints_pos, heightmaps, legs_order, simulation_dt,
            ref_base_lin_vel, ref_base_ang_vel, env.step_num, qpos, qvel,
            feet_jac, feet_jac_dot, feet_vel, legs_qfrc_passive, legs_qfrc_bias,
            legs_mass_matrix, legs_qpos_idx, legs_qvel_idx, tau, inertia,
            env.mjData.contact,
        )
        compute_actions_time = time.perf_counter() - t0

        for leg in legs_order:
            tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
            tau[leg] = np.clip(tau[leg], tau_min, tau_max)

        action = np.zeros(env.mjModel.nu)
        action[env.legs_tau_idx.FL] = tau.FL
        action[env.legs_tau_idx.FR] = tau.FR
        action[env.legs_tau_idx.RL] = tau.RL
        action[env.legs_tau_idx.RR] = tau.RR

        state, reward, is_terminated, is_truncated, info = env.step(action=action)

        ctrl_state = quadrupedpympc_wrapper.get_obs()
        contact_bool, _, feet_grf_actual = env.feet_contact_state(ground_reaction_forces=True)

        fp = env.feet_pos(frame="world")
        foot_z = {leg: float(fp[leg][2]) for leg in legs_order}
        foot_x = {leg: float(fp[leg][0]) for leg in legs_order}
        min_foot_z = min(min_foot_z, *foot_z.values())
        max_gaps = max(max_gaps, _gaps_crossed(x0, float(base_pos[0])))

        log_rows.append(
            {
                "step": step,
                "sim_time_s": env.simulation_time,
                "base_pos_x_m": base_pos[0],
                "base_pos_y_m": base_pos[1],
                "base_pos_z_m": base_pos[2],
                "base_roll_rad": base_ori_euler_xyz[0],
                "base_pitch_rad": base_ori_euler_xyz[1],
                "base_yaw_rad": base_ori_euler_xyz[2],
                "base_lin_vel_x_mps": base_lin_vel[0],
                "base_lin_vel_y_mps": base_lin_vel[1],
                "base_lin_vel_z_mps": base_lin_vel[2],
                "base_ang_vel_x_radps": base_ang_vel[0],
                "base_ang_vel_y_radps": base_ang_vel[1],
                "base_ang_vel_z_radps": base_ang_vel[2],
                "ref_lin_vel_x_mps": ref_base_lin_vel[0],
                "gait_step_freq_hz": GAIT_STEP_FREQ_HZ,
                "gaps_crossed": _gaps_crossed(x0, float(base_pos[0])),
                **{f"contact_{leg}": bool(contact_bool[leg]) for leg in legs_order},
                **{f"foot_z_{leg}_m": foot_z[leg] for leg in legs_order},
                **{f"foot_over_gap_{leg}": _foot_over_gap(foot_x[leg]) for leg in legs_order},
                **{f"grf_mpc_{leg}_{ax}_N": float(ctrl_state["nmpc_GRFs"][leg][j])
                   for leg in legs_order for j, ax in enumerate("xyz")},
                **{f"tau_{leg}_{j}_Nm": float(tau[leg][j]) for leg in legs_order for j in range(3)},
                "compute_actions_time_s": compute_actions_time,
            }
        )

        if fall_time_s is None and (
            base_pos[2] < FALL_HEIGHT_THRESHOLD_M
            or abs(base_ori_euler_xyz[0]) > FALL_TILT_THRESHOLD_RAD
            or abs(base_ori_euler_xyz[1]) > FALL_TILT_THRESHOLD_RAD
        ):
            fall_time_s = env.simulation_time

        if step % frame_stride == 0:
            cam.lookat[:] = [base_pos[0], base_pos[1], 0.0]
            renderer.update_scene(env.mjData, camera=cam)
            img = renderer.render()
            pil_img = Image.fromarray(img)
            draw = ImageDraw.Draw(pil_img)
            overlay = (
                f"Step 04: gap crossing (unmodified Quadruped-PyMPC, blind)\n"
                f"t={env.simulation_time:5.2f}s  v_ref={INITIAL_FORWARD_VEL_MPS:.1f}  "
                f"step_freq={GAIT_STEP_FREQ_HZ:.2f}Hz\n"
                f"x={base_pos[0]:5.2f}m  gaps crossed={_gaps_crossed(x0, float(base_pos[0]))}  "
                f"(30cm holes @ 1.5m)"
            )
            draw.multiline_text((8, 8), overlay, font=OVERLAY_FONT, fill=OVERLAY_COLOR)
            frames.append(np.asarray(pil_img))

    wall_elapsed = time.time() - t_wall_start
    print(f"Done: {n_steps} steps in {wall_elapsed:.1f}s wall-clock")
    env.close()

    walk_dist_x = log_rows[-1]["base_pos_x_m"] - log_rows[0]["base_pos_x_m"]
    walk_dist_y = log_rows[-1]["base_pos_y_m"] - log_rows[0]["base_pos_y_m"]
    gaps_crossed = _gaps_crossed(x0, log_rows[-1]["base_pos_x_m"])
    # 成功 = 転倒せず、穴を3つ以上越え、横ズレが通路の半幅(2.5m)内
    success = (fall_time_s is None) and (gaps_crossed >= 3) and (abs(walk_dist_y) < 2.0)
    verdict = "PASS" if success else "FAIL"

    csv_path = LOG_DIR / "state_log.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
        writer.writeheader()
        writer.writerows(log_rows)
    print(f"Wrote {len(log_rows)} rows to {csv_path}")

    scale = min(GIF_MAX_WIDTH / frames[0].shape[1], GIF_MAX_HEIGHT / frames[0].shape[0], 1.0)
    if scale < 1.0:
        new_size = (int(frames[0].shape[1] * scale), int(frames[0].shape[0] * scale))
        frames = [np.asarray(Image.fromarray(f).resize(new_size)) for f in frames]
    imageio.mimsave(gif_path, frames, duration=1.0 / GIF_FPS, loop=0, optimize=True)
    print(f"Wrote GIF: {gif_path}")

    gif_reader = imageio.get_reader(gif_path)
    n_frames_actual = gif_reader.get_length()
    frame_shape = gif_reader.get_data(0).shape
    gif_reader.close()

    meta = {
        "step": "03_gap_crossing",
        "forward_vel_mps": INITIAL_FORWARD_VEL_MPS,
        "gait_step_freq_hz": GAIT_STEP_FREQ_HZ,
        "num_seconds_recorded": NUM_SECONDS,
        "n_sim_steps": n_steps,
        "wall_clock_seconds": wall_elapsed,
        "gap_length_m": GAP_LENGTH_M,
        "gap_spacing_m": GAP_SPACING_M,
        "walk_dist_x_m": walk_dist_x,
        "walk_dist_y_m": walk_dist_y,
        "gaps_crossed": gaps_crossed,
        "min_foot_z_m": min_foot_z,
        "fall_time_s": fall_time_s,
        "verdict": verdict,
        "gif_path": str(gif_path),
        "gif_n_frames": n_frames_actual,
        "gif_resolution_wh": [frame_shape[1], frame_shape[0]],
        "gif_size_mb": round(gif_path.stat().st_size / (1024 * 1024), 2),
    }
    with open(LOG_DIR / "gif_meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    print(json.dumps(meta, indent=2))

    summary_row = {
        "id": trial_id_str,
        "velocity_mps": INITIAL_FORWARD_VEL_MPS,
        "gait_step_freq_hz": GAIT_STEP_FREQ_HZ,
        "sim_time_s": log_rows[-1]["sim_time_s"] - log_rows[0]["sim_time_s"],
        "walk_dist_x_m": walk_dist_x,
        "walk_dist_y_m": walk_dist_y,
        "gaps_crossed": gaps_crossed,
        "fall_time_s": fall_time_s,
        "verdict": verdict,
    }
    write_header = not SUMMARY_CSV_PATH.exists()
    with open(SUMMARY_CSV_PATH, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(summary_row)
    print(f"Appended trial {trial_id_str} to {SUMMARY_CSV_PATH}")

    print(
        f"\n[{verdict}] v={INITIAL_FORWARD_VEL_MPS} m/s  freq={GAIT_STEP_FREQ_HZ} Hz  "
        f"gaps_crossed={gaps_crossed}  walk_dist_x={walk_dist_x:.2f} m  "
        f"y_drift={walk_dist_y:+.2f} m  fall_time_s={fall_time_s}"
    )
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
