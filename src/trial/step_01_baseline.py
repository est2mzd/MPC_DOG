"""Step 01: 基準ログとGIFの記録ハーネス。

背景: simulation.py の run_simulation() はGIF用フレーム取得やGRF/接触/
MPC計算時間のCSV記録の仕組みを持たないため、記録用に作成した。
目的: run_simulation() の内側ループ(simulation.py 169-327行目、
commit cc145a2)を制御ロジックは変更せず同じ順序で呼び出し、ログとGIFを生成する。
"""

from __future__ import annotations

import copy
import csv
import json
import time
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import mujoco
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as qpympc_cfg
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

# gym_quadruped の QuadrupedEnv は simulation.py と同じ import 元
from gym_quadruped.quadruped_env import QuadrupedEnv


# ============================================================================
# 記録パラメータ(MPC_DOG側の設定。external/ 側の値は一切変更しない)
# ============================================================================
NUM_SECONDS = 10  # 記録するシミュレーション実時間(秒)。要件は20秒以上(現在2秒はテスト用の暫定値)
INITIAL_FORWARD_VEL_MPS = 1.1  # 犬の前進初期速度[m/s]
GIF_FPS = 10               # GIFの再生フレームレート(要件: 10〜15fps程度)
GIF_MAX_WIDTH = 480        # GIF解像度(要件: 960x540以下。ファイルサイズを抑えるため小さめに設定)
GIF_MAX_HEIGHT = 270
OVERLAY_FONT_SIZE = 24     # GIF内オーバーレイ文字のフォントサイズ(px)
OVERLAY_FONT = ImageFont.load_default(size=OVERLAY_FONT_SIZE)
OVERLAY_COLOR = (0, 0, 0)  # オーバーレイ文字の色(黒)
REPO_ROOT = Path(__file__).resolve().parent.parent.parent  # src/trial/ の2階層上
LOG_DIR = REPO_ROOT / "artifacts" / "logs" / "step_01"
GIF_DIR = REPO_ROOT / "artifacts" / "gifs"
SUMMARY_CSV_PATH = LOG_DIR / "trials_summary.csv"  # 試行ごとの結果一覧(id, 速度, sim時間, 歩行距離, コケた時間)


def _next_trial_id(summary_path: Path) -> int:
    """SUMMARY_CSV_PATHの既存行から次の連番idを決める。

    Args:
        summary_path: trials_summary.csvのパス。

    Returns:
        既存最大id+1。ファイルが無い/行が無い場合は1。
    """
    if not summary_path.exists():
        return 1
    with open(summary_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return 1
    return max(int(row["id"]) for row in rows) + 1


def main() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    GIF_DIR.mkdir(parents=True, exist_ok=True)

    trial_id = _next_trial_id(SUMMARY_CSV_PATH)  # このハーネスの1回の実行を識別する連番
    trial_id_str = f"{trial_id:02d}"  # 2桁ゼロ埋め表記(99を超える場合は桁が増える)
    gif_path = GIF_DIR / f"step_01_{trial_id_str}.gif"

    # simulation.py 55〜76行目相当(commit cc145a2)。ref_base_ang_vel/state_obs_names は
    # このハーネスでは不要なため渡していない。base_vel_command_type/ref_base_lin_velは
    # qpympc_cfgの既定("human"、キー入力待ちで速度0のまま)ではなく、非対話実行でも
    # 前進速度を指定できる"forward"に変更している(公式コードからの変更点)。
    env = QuadrupedEnv(
        robot=qpympc_cfg.robot,
        scene=qpympc_cfg.simulation_params["scene"],
        sim_dt=qpympc_cfg.simulation_params["dt"],
        ground_friction_coeff=qpympc_cfg.simulation_params.get("ground_friction_coeff", (0.5, 1.0)),
        base_vel_command_type="forward",  # "human"/"forward"/"random"のうち、非対話実行でも速度指令が入る"forward"を選択
        ref_base_lin_vel=INITIAL_FORWARD_VEL_MPS,  # 前進速度[m/s]の固定値(floatは(v,v)の縮退レンジとして扱われる)
    )
    env.mjModel.opt.gravity[2] = -qpympc_cfg.gravity_constant  # 重力をz軸下向き(負)に設定。simulation.py 66行目と同じ
    env.reset(random=False)  # simulation.py 72行目と同じ。初期姿勢を固定する

    legs_order = ["FL", "FR", "RL", "RR"]
    # simulation.py 95〜117行目のelse節(visual_foothold_adaptation="blind"が既定)に相当。
    # if節(VFA有効時のHeightMap構築)はこのハーネスでは使わないため実装していない。
    heightmaps = None

    # simulation.py 134〜139行目相当。feet_geom_id・quadrupedpympc_observables_names
    # 引数は省略(このハーネスはnmpc_GRFs等の既定observableしか使わないため不要)。
    quadrupedpympc_wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=legs_order,
    )

    # simulation.py 82〜89行目相当。関節トルク上限を90%に抑える安全マージン。
    tau_soft_limits_scalar = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft_limits_scalar,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft_limits_scalar,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft_limits_scalar,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft_limits_scalar,
    )
    tau = LegsAttr(*[np.zeros(env.mjModel.nu // 4) for _ in range(4)])  # 各脚3関節分の初期値(ゼロ)。初回compute_actions()で上書きされる

    simulation_dt = qpympc_cfg.simulation_params["dt"]  # シミュレーション周期(秒)
    n_steps = int(NUM_SECONDS // simulation_dt)  # simulation.py 165行目と同じ式

    # オフスクリーンレンダラ(MPC_DOG側、external/は関与しない)。
    # 解像度はMuJoCo既定のオフスクリーンフレームバッファ(640x480)に収まる値。
    renderer = mujoco.Renderer(env.mjModel, height=360, width=640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(env.mjModel, cam)
    cam.distance = 2.2   # ロボット全体と床面が画角に入る距離(実測して調整した値)
    cam.elevation = -20  # 見下ろし角度(度)。低いと足元が隠れ、高いと平面的になる
    cam.azimuth = 120    # 水平方向の向き(度)。脚の動きが見えやすい角度

    frame_stride = max(1, int(round((1.0 / GIF_FPS) / simulation_dt)))  # 何ステップに1回GIFフレームを記録するか

    frames: list[np.ndarray] = []
    log_rows: list[dict] = []
    fall_time_s = None  # is_terminated/is_truncatedが最初に発生したsim_time[s](発生しなければNoneのまま)

    print(f"Recording {NUM_SECONDS}s ({n_steps} steps at dt={simulation_dt}s) ...")
    t_wall_start = time.time()

    for step in range(n_steps):
        # ==== simulation.py 172〜205行目相当:状態取得(引数を1つずつ、順序も同一) ====
        feet_pos = env.feet_pos(frame="world")  # LegsAttr、各脚(3,) 位置[m]、world座標系
        feet_vel = env.feet_vel(frame="world")  # LegsAttr、各脚(3,) 速度[m/s]、world座標系
        hip_pos = env.hip_positions(frame="world")  # LegsAttr、各脚(3,) 位置[m]、world座標系
        base_lin_vel = env.base_lin_vel(frame="world")  # (3,) [m/s]、world座標系
        base_ang_vel = env.base_ang_vel(frame="base")  # (3,) [rad/s]、base座標系(base_lin_velはworldなので座標系が異なる)
        base_ori_euler_xyz = env.base_ori_euler_xyz  # (3,) roll,pitch,yaw[rad]、world座標系
        base_pos = copy.deepcopy(env.base_pos)  # (3,) [m]、world座標系
        com_pos = copy.deepcopy(env.com)  # (3,) 全身重心位置[m]、world座標系

        ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()  # (3,)+(3,) [m/s]+[rad/s]、world座標系。速度指令が未設定なら零ベクトル

        # Baseの回転慣性行列[kg・m^2]。(3,3)をflattenして(9,)にする(qpympc_cfg.inertiaと同じ形式)
        if qpympc_cfg.simulation_params["use_inertia_recomputation"]:
            inertia = env.get_base_inertia().flatten()
        else:
            inertia = qpympc_cfg.inertia.flatten()

        qpos, qvel = env.mjData.qpos, env.mjData.qvel  # MuJoCo生の状態。qposは(nq,)(base四元数7+関節数)、qvelは(nv,)(base6+関節数)
        legs_qvel_idx = env.legs_qvel_idx  # 脚ごとのqvel内インデックス(LegsAttr)
        legs_qpos_idx = env.legs_qpos_idx  # 脚ごとのqpos内インデックス(LegsAttr)
        joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)  # simulation.py 196行目と同じくqvel_idxを使う(qpos_idxではない。公式実装のまま)

        legs_mass_matrix = env.legs_mass_matrix  # 脚ごとの質量行列(LegsAttr、各脚(3,3))
        legs_qfrc_bias = env.legs_qfrc_bias  # 脚ごとのコリオリ・遠心力・重力項(LegsAttr、各脚(3,))
        legs_qfrc_passive = env.legs_qfrc_passive  # 脚ごとの受動力・摩擦項(LegsAttr、各脚(3,))

        feet_jac = env.feet_jacobians(frame="world", return_rot_jac=False)  # LegsAttr、各脚(3,nv) 並進ヤコビアン、world座標系
        feet_jac_dot = env.feet_jacobians_dot(frame="world", return_rot_jac=False)  # LegsAttr、各脚(3,nv) ヤコビアンの時間微分、world座標系

        # ==== simulation.py 208〜236行目相当:PyMPCコントローラ本体(未変更の呼び出し) ====
        t0 = time.perf_counter()  # MPC+WBC計算時間の計測開始

        # 状態(com_pos以下)からMPC(acados)でGRFを求め、ヤコビアン転置とswing軌道追従制御でtauに変換する。
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
        compute_actions_time = time.perf_counter() - t0  # compute_actions()の実測所要時間[s]

        # ==== simulation.py 238〜251行目相当:トルク制限とMuJoCoへの入力 ====
        for leg in legs_order:
            tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]  # 各(3,)。tau_limits[leg]は(3,2)=[関節, (min,max)][N・m]
            tau[leg] = np.clip(tau[leg], tau_min, tau_max)

        action = np.zeros(env.mjModel.nu)  # (12,) 全アクチュエータ分のトルク指令[N・m]初期値
        action[env.legs_tau_idx.FL] = tau.FL
        action[env.legs_tau_idx.FR] = tau.FR
        action[env.legs_tau_idx.RL] = tau.RL
        action[env.legs_tau_idx.RR] = tau.RR

        # actionのトルク[N・m]をロボットに適用しdt分シミュレーションを進める。
        # reward/infoは未使用。is_terminated/is_truncatedのみ後段のリセット判定に使う。
        state, reward, is_terminated, is_truncated, info = env.step(action=action)

        # ==== simulation.py 254行目相当:コントローラの観測値(GRF等)を取得 ====
        ctrl_state = quadrupedpympc_wrapper.get_obs()  # dict。"nmpc_GRFs"はLegsAttr、各脚(3,) MPC計算GRF[N]
        contact_bool, _, feet_grf_actual = env.feet_contact_state(ground_reaction_forces=True)  # LegsAttr(bool)、LegsAttr(未使用)、LegsAttr((3,)[N])、world座標系

        # ==== 記録処理(MPC_DOG側の追加。external/には存在しない) ====
        # 単位・座標系は simulation.py と同じ(world座標系、m, m/s, rad, rad/s, N, N・m)。
        # 脚順序は FL, FR, RL, RR で固定。
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
            cam.lookat[:] = [base_pos[0], base_pos[1], 0.0]  # xyはロボット追従、zは地面レベルで固定
            renderer.update_scene(env.mjData, camera=cam)  # 現在のmjDataとカメラ設定をrenderer内部バッファへ反映(戻り値なし)
            img = renderer.render()  # (360,640,3) uint8 RGB画像
            pil_img = Image.fromarray(img)
            draw = ImageDraw.Draw(pil_img)
            overlay = (
                f"Step 01: reference baseline (unmodified Quadruped-PyMPC)\n"
                f"t = {env.simulation_time:5.2f} s\n"
                f"ref v=({ref_base_lin_vel[0]:+.2f},{ref_base_lin_vel[1]:+.2f}) m/s  "
                f"actual v=({base_lin_vel[0]:+.2f},{base_lin_vel[1]:+.2f}) m/s"
            )
            draw.multiline_text((8, 8), overlay, font=OVERLAY_FONT, fill=OVERLAY_COLOR)
            frames.append(np.asarray(pil_img))

        # 転倒したら、リセットする処理
        # if is_terminated or is_truncated:
        #     if fall_time_s is None:
        #         fall_time_s = log_rows[-1]["sim_time_s"]  # 最初の発生時刻のみ記録(2回目以降のリセットは対象外)
        #     env.reset(random=True)  # ロボットをランダム化した初期姿勢へ戻す(戻り値なし)。65行目のrandom=Falseと対比
        #     quadrupedpympc_wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))  # MPC/WBC内部状態(歩容位相・着地目標など)をリセット(戻り値なし)

    wall_elapsed = time.time() - t_wall_start
    print(f"Done: {n_steps} steps in {wall_elapsed:.1f}s wall-clock "
          f"({n_steps / wall_elapsed:.1f} steps/s)")

    env.close()

    # ==== ログをCSVへ保存 ====
    csv_path = LOG_DIR / "state_log.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
        writer.writeheader()
        writer.writerows(log_rows)
    print(f"Wrote {len(log_rows)} rows to {csv_path}")

    # ==== GIFを保存(無限ループ、fps=GIF_FPS) ====
    scale = min(GIF_MAX_WIDTH / frames[0].shape[1], GIF_MAX_HEIGHT / frames[0].shape[0], 1.0)  # 解像度上限(960x540)以下に縮小する倍率
    if scale < 1.0:
        new_size = (int(frames[0].shape[1] * scale), int(frames[0].shape[0] * scale))
        frames = [np.asarray(Image.fromarray(f).resize(new_size)) for f in frames]

    # framesをGIFへ書き出す(戻り値なし)。duration=1フレームあたりの表示時間[s]、loop=0は無限ループ再生、
    # optimize=Trueはファイルサイズを抑えるパレット最適化(画質・解像度は変えない)。
    imageio.mimsave(gif_path, frames, duration=1.0 / GIF_FPS, loop=0, optimize=True)
    print(f"Wrote GIF: {gif_path}")

    # ==== メタデータ(実時間・解像度・フレーム数・ファイルサイズ)を検証して保存 ====
    gif_reader = imageio.get_reader(gif_path)
    n_frames_actual = gif_reader.get_length()
    frame_shape = gif_reader.get_data(0).shape
    gif_reader.close()
    gif_real_time_s = n_frames_actual / GIF_FPS
    gif_size_bytes = gif_path.stat().st_size

    meta = {
        "num_seconds_recorded": NUM_SECONDS,
        "n_sim_steps": n_steps,
        "wall_clock_seconds": wall_elapsed,
        "gif_path": str(gif_path),
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

    # ==== 試行サマリをSUMMARY_CSV_PATHへ追記(初回のみヘッダーを書く) ====
    summary_row = {
        "id": trial_id_str,
        "velocity_mps": INITIAL_FORWARD_VEL_MPS,
        "sim_time_s": n_steps * simulation_dt,
        "walk_dist_x_m": log_rows[-1]["base_pos_x_m"] - log_rows[0]["base_pos_x_m"],
        "walk_dist_y_m": log_rows[-1]["base_pos_y_m"] - log_rows[0]["base_pos_y_m"],
        "fall_time_s": fall_time_s,
    }
    write_header = not SUMMARY_CSV_PATH.exists()
    with open(SUMMARY_CSV_PATH, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(summary_row)
    print(f"Appended trial {trial_id_str} to {SUMMARY_CSV_PATH}")


if __name__ == "__main__":
    main()
