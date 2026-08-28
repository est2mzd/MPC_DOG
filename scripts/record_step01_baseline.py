"""Step 01: Quadruped-PyMPC 公式サンプルを実行し、基準ログとGIFを記録する。

## このスクリプトの位置づけ(rule 3.1 / rule 3.4 に対応)

`external/Quadruped-PyMPC/simulation/simulation.py` の `run_simulation()` は、
- 制御ロジックの計算自体は `quadrupedpympc_wrapper.compute_actions(...)`(未変更のまま呼び出す)
  にすべて委譲している。
- しかし、レンダリングは対話的な `mujoco.viewer`(GUIウィンドウ)前提であり、
  フレームをオフスクリーンで取得してGIFに保存する仕組みは持たない。
- GRF・接触状態・MPC計算時間をCSVへ構造化して残す仕組みも持たない
  (`recording_path` はMuJoCoの生の観測(state)しかHDF5へ保存しない)。

このスクリプトは、`run_simulation()` の**制御ロジックそのものは一切書き換えず**、
その内側のループ(`simulation.py` 169〜327行目)を、記録(オフスクリーン
レンダリング・ログ書き出し)のためだけに**同じ順序・同じ引数でそのまま呼び出す**
薄いハーネスである。PyMPC自体の計算式・アルゴリズムはこのファイルには
一切含まれていない(すべて `quadrupedpympc_wrapper.compute_actions()` の
内部、`external/Quadruped-PyMPC` 側の変更していないコードが担う)。

各ブロックの直前コメントに、対応する `simulation.py` の行番号を明記した
(commit `cc145a2d353db4c39df4b49e6624959acc4b87b0` 時点)。
"""

from __future__ import annotations

import copy
import csv
import json
import time
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

import mujoco
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as qpympc_cfg
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

# gym_quadruped の QuadrupedEnv は simulation.py と同じ import 元
from gym_quadruped.quadruped_env import QuadrupedEnv


# ============================================================================
# 記録パラメータ(MPC_DOG側の設定。external/ 側の値は一切変更しない)
# ============================================================================
NUM_SECONDS = 25.0        # 記録するシミュレーション実時間(秒)。要件(20秒以上)に余裕を持たせる
GIF_FPS = 10               # GIFの再生フレームレート(要件: 10〜15fps程度)
GIF_MAX_WIDTH = 640        # GIF解像度(要件: 960x540以下。ファイルサイズを抑えるため小さめに設定)
GIF_MAX_HEIGHT = 360
LOG_DIR = Path(__file__).resolve().parent.parent / "artifacts" / "logs" / "step_01"
GIF_DIR = Path(__file__).resolve().parent.parent / "artifacts" / "gifs"
GIF_PATH = GIF_DIR / "step_01_reference_baseline.gif"


def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    GIF_DIR.mkdir(parents=True, exist_ok=True)

    # --- simulation.py 60〜75行目付近に相当: QuadrupedEnv の構築(未変更の呼び出し) ---
    # ロボット種別・地形・摩擦係数等は config.py の既定値をそのまま使う(この
    # スクリプトからは一切上書きしない)。
    env = QuadrupedEnv(
        robot=qpympc_cfg.robot,
        scene=qpympc_cfg.simulation_params["scene"],
        sim_dt=qpympc_cfg.simulation_params["dt"],
        ground_friction_coeff=qpympc_cfg.simulation_params.get("ground_friction_coeff", (0.5, 1.0)),
        base_vel_command_type=qpympc_cfg.simulation_params["mode"],
    )
    env.mjModel.opt.gravity[2] = -qpympc_cfg.gravity_constant
    env.reset(random=False)

    legs_order = ["FL", "FR", "RL", "RR"]
    heightmaps = None  # simulation.py 95〜113行目相当: visual_foothold_adaptation="blind" が既定のため None

    # --- simulation.py 134行目相当: コントローララッパーの構築(未変更) ---
    quadrupedpympc_wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=legs_order,
    )

    # --- simulation.py 付近: 関節トルク上限(90%セーフティマージン) ---
    tau_soft_limits_scalar = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft_limits_scalar,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft_limits_scalar,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft_limits_scalar,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft_limits_scalar,
    )
    tau = LegsAttr(*[np.zeros(env.mjModel.nu // 4) for _ in range(4)])

    simulation_dt = qpympc_cfg.simulation_params["dt"]
    n_steps = int(NUM_SECONDS // simulation_dt)  # simulation.py 165行目と同じ式

    # --- オフスクリーンレンダラ(MPC_DOG側の追加、external/は関与しない) ---
    # ロボット全体と床面が入るよう、base位置を追従するfree cameraを使う。
    # MuJoCoモデルの既定オフスクリーンフレームバッファ(640x480)に収まる
    # 解像度を使う(モデルXMLのvisual/global/offwidth等は external/ 側の
    # アセットなので変更しない)。
    renderer = mujoco.Renderer(env.mjModel, height=360, width=640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(env.mjModel, cam)
    cam.distance = 2.2
    cam.elevation = -20
    cam.azimuth = 120

    frame_stride = max(1, int(round((1.0 / GIF_FPS) / simulation_dt)))

    frames: list[np.ndarray] = []
    log_rows: list[dict] = []

    print(f"Recording {NUM_SECONDS}s ({n_steps} steps at dt={simulation_dt}s) ...")
    t_wall_start = time.time()

    for step in range(n_steps):
        # ==== simulation.py 172〜205行目相当:状態取得(引数を1つずつ、順序も同一) ====
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

        # ==== simulation.py 208〜236行目相当:PyMPCコントローラ本体(未変更の呼び出し) ====
        # ここで実際にMPC(acados)・WBCが計算される。この呼び出し自体の壁時計
        # 時間を「MPC計算時間」としてログする(内部で毎ステップ解くわけでは
        # なく、config.py の mpc_frequency に従い間引かれるため、値は
        # ステップごとに大小がある。これは compute_actions() 側の既存の挙動で
        # あり、ここでは変更していない)。
        t0 = time.perf_counter()
        tau = quadrupedpympc_wrapper.compute_actions(
            com_pos,
            base_pos,
            base_lin_vel,
            base_ori_euler_xyz,
            base_ang_vel,
            feet_pos,
            hip_pos,
            joints_pos,
            heightmaps,
            legs_order,
            simulation_dt,
            ref_base_lin_vel,
            ref_base_ang_vel,
            env.step_num,
            qpos,
            qvel,
            feet_jac,
            feet_jac_dot,
            feet_vel,
            legs_qfrc_passive,
            legs_qfrc_bias,
            legs_mass_matrix,
            legs_qpos_idx,
            legs_qvel_idx,
            tau,
            inertia,
            env.mjData.contact,
        )
        compute_actions_time = time.perf_counter() - t0

        # ==== simulation.py 238〜251行目相当:トルク制限とMuJoCoへの入力 ====
        for leg in legs_order:
            tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
            tau[leg] = np.clip(tau[leg], tau_min, tau_max)

        action = np.zeros(env.mjModel.nu)
        action[env.legs_tau_idx.FL] = tau.FL
        action[env.legs_tau_idx.FR] = tau.FR
        action[env.legs_tau_idx.RL] = tau.RL
        action[env.legs_tau_idx.RR] = tau.RR

        state, reward, is_terminated, is_truncated, info = env.step(action=action)

        # ==== simulation.py 254行目相当:コントローラの観測値(GRF等)を取得 ====
        ctrl_state = quadrupedpympc_wrapper.get_obs()
        contact_bool, _, feet_grf_actual = env.feet_contact_state(ground_reaction_forces=True)

        # ------------------------------------------------------------------
        # ここから記録処理(MPC_DOG側、external/には存在しない付加機能)。
        # 単位・座標系はすべて simulation.py と同じ(world座標系、m, m/s, rad,
        # rad/s, N, N・m)。脚順序は FL, FR, RL, RR で固定。
        # ------------------------------------------------------------------
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
                "ref_lin_vel_y_mps": ref_base_lin_vel[1],
                "ref_ang_vel_z_radps": ref_base_ang_vel[2],
                **{f"contact_{leg}": bool(contact_bool[leg]) for leg in legs_order},
                **{f"grf_mpc_{leg}_{ax}_N": float(ctrl_state["nmpc_GRFs"][leg][j])
                   for leg in legs_order for j, ax in enumerate("xyz")},
                **{f"grf_actual_{leg}_{ax}_N": float(feet_grf_actual[leg][j])
                   for leg in legs_order for j, ax in enumerate("xyz")},
                **{f"tau_{leg}_{j}_Nm": float(tau[leg][j]) for leg in legs_order for j in range(3)},
                "compute_actions_time_s": compute_actions_time,
            }
        )

        # ==== オフスクリーン描画(GIF用フレーム、frame_strideごとに間引く) ====
        if step % frame_stride == 0:
            cam.lookat[:] = [base_pos[0], base_pos[1], 0.0]
            renderer.update_scene(env.mjData, camera=cam)
            img = renderer.render()
            pil_img = Image.fromarray(img)
            draw = ImageDraw.Draw(pil_img)
            overlay = (
                f"Step 01: reference baseline (unmodified Quadruped-PyMPC)\n"
                f"t = {env.simulation_time:5.2f} s\n"
                f"ref v=({ref_base_lin_vel[0]:+.2f},{ref_base_lin_vel[1]:+.2f}) m/s  "
                f"actual v=({base_lin_vel[0]:+.2f},{base_lin_vel[1]:+.2f}) m/s"
            )
            draw.multiline_text((8, 8), overlay, fill=(255, 255, 0))
            frames.append(np.asarray(pil_img))

        # ==== simulation.py 319〜327行目相当:エピソード終了時のリセット ====
        # (num_seconds分を1エピソードで記録し切る設計のため、通常はこの分岐
        #  には到達しない。万一 is_terminated 等が発生した場合のみ、公式実装
        #  と同じくリセットして記録は継続する。)
        if is_terminated or is_truncated:
            env.reset(random=True)
            quadrupedpympc_wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))

    wall_elapsed = time.time() - t_wall_start
    print(f"Done: {n_steps} steps in {wall_elapsed:.1f}s wall-clock "
          f"({n_steps / wall_elapsed:.1f} steps/s)")

    env.close()

    # ------------------------------------------------------------------
    # ログをCSVへ保存
    # ------------------------------------------------------------------
    csv_path = LOG_DIR / "state_log.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
        writer.writeheader()
        writer.writerows(log_rows)
    print(f"Wrote {len(log_rows)} rows to {csv_path}")

    # ------------------------------------------------------------------
    # GIFを保存(無限ループ、fps=GIF_FPS)
    # ------------------------------------------------------------------
    # 要件の解像度上限(960x540)以下になるよう縮小する。
    scale = min(GIF_MAX_WIDTH / frames[0].shape[1], GIF_MAX_HEIGHT / frames[0].shape[0], 1.0)
    if scale < 1.0:
        new_size = (int(frames[0].shape[1] * scale), int(frames[0].shape[0] * scale))
        frames = [np.asarray(Image.fromarray(f).resize(new_size)) for f in frames]

    imageio.mimsave(GIF_PATH, frames, duration=1.0 / GIF_FPS, loop=0)
    print(f"Wrote GIF: {GIF_PATH}")

    # ------------------------------------------------------------------
    # メタデータ(実時間・解像度・フレーム数・ファイルサイズ)を検証して保存
    # ------------------------------------------------------------------
    gif_reader = imageio.get_reader(GIF_PATH)
    n_frames_actual = gif_reader.get_length()
    frame_shape = gif_reader.get_data(0).shape
    gif_reader.close()
    gif_real_time_s = n_frames_actual / GIF_FPS
    gif_size_bytes = GIF_PATH.stat().st_size

    meta = {
        "num_seconds_recorded": NUM_SECONDS,
        "n_sim_steps": n_steps,
        "wall_clock_seconds": wall_elapsed,
        "gif_path": str(GIF_PATH),
        "gif_n_frames": n_frames_actual,
        "gif_fps": GIF_FPS,
        "gif_real_time_seconds": gif_real_time_s,
        "gif_resolution_wh": [frame_shape[1], frame_shape[0]],
        "gif_size_bytes": gif_size_bytes,
        "gif_size_mb": round(gif_size_bytes / (1024 * 1024), 2),
    }
    with open(LOG_DIR / "gif_meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
