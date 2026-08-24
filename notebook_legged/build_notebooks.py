"""Generate the legged_control study notebooks.

The notebooks deliberately use only NumPy/SciPy/Matplotlib for executable
experiments.  ROS, OCS2, Pinocchio, qpOASES, and Unitree hardware are explained
from the inspected upstream implementation but are not required to run them.
"""

from __future__ import annotations

import json
import hashlib
from pathlib import Path
from textwrap import dedent


HERE = Path(__file__).resolve().parent
UPSTREAM_COMMIT = "a7f381c0367e98e31c01336e678eef47e304d40d"
UPSTREAM = f"https://github.com/qiayuanliao/legged_control/tree/{UPSTREAM_COMMIT}"


def md(text: str) -> dict:
    source = dedent(text).strip() + "\n"
    return {
        "cell_type": "markdown",
        "id": hashlib.sha1(source.encode()).hexdigest()[:8],
        "metadata": {},
        "source": source,
    }


def code(text: str) -> dict:
    source = dedent(text).strip() + "\n"
    return {
        "cell_type": "code",
        "id": hashlib.sha1(source.encode()).hexdigest()[:8],
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": source,
    }


def notebook(cells: list[dict]) -> dict:
    return {
        "cells": cells,
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3 (ipykernel)",
                "language": "python",
                "name": "python3",
            },
            "language_info": {"name": "python", "version": "3.11"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


COMMON = """
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path.cwd()
for candidate in [ROOT, *ROOT.parents]:
    if (candidate / "pyproject.toml").exists():
        ROOT = candidate
        break

np.set_printoptions(precision=4, suppress=True)
plt.rcParams.update({"figure.figsize": (9, 4), "axes.grid": True})
print("repository:", ROOT)
"""


NOTEBOOKS: dict[str, list[dict]] = {}


NOTEBOOKS["00_learning_map.ipynb"] = [
    md(
        f"""
        # 00 — legged_control の背景・目的・結論と全データフロー

        ## 1. リポジトリの背景
        四足ロボットは、浮動胴体6自由度と12関節、接地/遊脚が切り替わるhybrid systemである。
        関節PDだけでは、どの足で体重を支え、どの方向へ地面を蹴り、将来の接地切替に備えるかを
        一貫して決めにくい。`qiayuanliao/legged_control` はこの問題を、

        - OCS2のcentroidal NMPCによる **未来1秒の状態・GRF計画**
        - qpOASESのWeighted WBCによる **現在瞬間の全身力学整合**
        - 線形Kalman filterによる **浮動base並進推定**
        - ros-controlによる **Gazebo/Unitree実機の共通I/O**

        に分けた、A1/Go1/Aliengo向けのmodel-based locomotion baselineである。
        公開元は2025年時点で開発終了を明記し、知覚統合の後継として `legged_perceptive` を案内している。

        照合対象: [`legged_control` commit `{UPSTREAM_COMMIT[:10]}`]({UPSTREAM})

        > 重要: 上流C++は `external/legged_control/` に上記commitでclone済みだが、gitignore対象である。
        > 主要経路はC++と `docs/legged_control/` の照合結果を使う。OCS2本体はこのworkspaceに無いため、
        > OCS2内部の完全なODEは公開APIから分かる範囲、と区別する。

        ## 2. リポジトリの目的
        上流READMEが掲げる目的は、NMPC・WBC・状態推定・sim2realを一つのROS制御stackとして提供し、
        Unitree A1へ展開可能な高性能baselineにすることである。このNotebook系列の目的は別である。
        完成stackをblack boxとして起動するのではなく、各境界を
        **入力 → 数式 → C++処理 → 出力 → 次block** の順に読み直し、最終的にQ/R、摩擦、
        WBC重み、周期、関節gainや制約式を変更できるようにする。

        ## 3. 先に結論
        1. 速度/goalとGaitは別入力で、NMPCはGaitを選ばない。
        2. NMPC状態・入力は各24次元。入力はGRF 12 + 関節速度12で、torqueではない。
        3. NMPCは100 Hzで未来policyを作り、500 Hz側は現在時刻の1点だけを読む。
        4. 既定WBCは階層QPではなくWeightedWbc単一QP。42変数の末尾torque 12だけを送る。
        5. 関節指令はWBC torque feedforward + `Kp=0, Kd=3`。
        6. 上流commit `{UPSTREAM_COMMIT[:10]}` はROS1/OCS2の原実装である。一方、このprojectが所有する
           `src/legged_control_mujoco/` はMuJoCo実行adapterで、OCS2 SQPを実装していない。
        7. Notebook 13は4秒の **equation-level proxy benchmark**、Notebook 14はadapterを実際に
           20秒以上動かすA1 MuJoCo benchmarkであり、どちらも上流ROS1/OCS2そのものの性能ではない。

        ## 4. 読む順序
        1. 全体像とパッケージ
        2. 状態・入力・座標系
        3. 指令・参照・Gait
        4. 状態推定
        5. centroidal力学
        6. NMPC
        7. 接触制約
        8. WBC
        9. 関節ハイブリッド制御
        10. 100 Hz / 500 Hz の統合
        11. チューニングと数式変更
        12. 実C++コードの端から端までのwalkthrough
        13. equation-level proxy benchmark（上流性能ではない）
        14. A1 MuJoCo adapterの30シナリオ実行証拠
        """
    ),
    code(COMMON),
    md(
        r"""
        ## 5. 一本の閉ループ — データの流れ

        ```text
        人間/上位planner
          │
          ├─ /cmd_vel: [vx,vy,vz,yaw_rate] ──→ TargetTrajectoriesPublisher
          │                                      │ time(2), x_ref(2,24), u_ref(2,24)
          └─ gait名: stance/trot/... ─────────→ GaitReceiver
                                                 │ ModeSchedule
                                                 ▼
        ┌─────────────────────────────────────────────────────────────┐
        │ OCS2 SQP-NMPC thread                                   100 Hz│
        │ x0(24) + reference + contact schedule                         │
        │ → horizon 1.0 s の x*(t,24), u*(t,24), mode(t) policy         │
        └──────────────────────────┬──────────────────────────────────┘
                                   │ shared policy
        IMU(orientation,ω,a)       ▼
        joint q,dq ──→ Linear KF / rbd conversion ──→ x_meas(24)
        contact(4)          │ rbdState(36)                 │
                            └──────────────┬────────────────┘
                                           ▼
        ┌─────────────────────────────────────────────────────────────┐
        │ LeggedController::update                               500 Hz│
        │ evaluatePolicy(now,x) → x*(24),u*(24),mode                  │
        │ WeightedWbc → [qdd(18), Fc(12), tau(12)]                    │
        │ SafetyChecker → setCommand(q*,dq*,Kp=0,Kd=3,ff=tau)         │
        └──────────────────────────┬──────────────────────────────────┘
                                   ▼
                         Gazebo / Unitree motors
                                   │ q,dq,IMU,contact
                                   └──────────────→ 推定へ戻る
        ```

        ここで最も大切な分離は次の3つ。

        - **GaitはNMPCが選ばない。** 接地時間割として外から与える。
        - **NMPCは関節トルクを決めない。** 未来の状態・GRF・関節速度を決める。
        - **WBCは未来を解かない。** 現在瞬間の全身力学を満たすトルクをQPで決める。
        """
    ),
    code(
        """
        # 各境界の次元を、接続できる「型」として確認する。
        dimensions = {
            "command_twist_used": 4,
            "nmpc_state": 24,
            "nmpc_input": 24,
            "rbd_state": 36,
            "wbc_decision": 42,
            "joint_torque": 12,
            "hybrid_command_scalars": 12 * 5,
        }
        assert dimensions["wbc_decision"] == 18 + 12 + 12
        assert dimensions["nmpc_input"] == 12 + 12
        dimensions
        """
    ),
    md(
        """
        ## 6. blockごとの契約

        | Block | 入力 | 主な式/処理 | 出力 |
        |---|---|---|---|
        | 参照 | cmd + 現在姿勢 | $p^+=p+RvT$ | 2点 $x^{{ref}}$ |
        | Gait | gait名 + 時刻 | mode schedule | 接地flag |
        | 推定 | IMU,q,dq,接地 | linear KF | rbd 36 → x 24 |
        | NMPC | x0,ref,mode | centroidal OCP/SQP | policy x*,u*,mode |
        | WBC | x*,u*,rbd,mode | constrained QP | qdd,Fc,tau |
        | 関節 | q*,dq*,tau | torque FF + low-gain PD | motor command |
        | Plant | motor command | rigid-body/contact dynamics | sensors |

        ## 7. このNotebookの到達確認
        自分の言葉で答える。

        1. NMPCの24入力のうち、前半12と後半12は何か。
        2. WBCの42変数のうち、モータへ送るのはどこか。
        3. 100 HzのNMPC解を500 Hz側はどう使うか。
        4. Gaitと胴体速度指令を分ける理由は何か。

        次: `01_packages_and_loop.ipynb`
        """
    ),
]


NOTEBOOKS["01_packages_and_loop.ipynb"] = [
    md(
        f"""
        # 01 — パッケージと制御ループ

        ## 背景・目的
        ROSパッケージ名ではなく、**誰が、何Hzで、どのデータを更新するか**を読む。
        コード変更時に「計画」「推定」「実行」「I/O」の責務を混ぜないための章である。

        実装対応:
        - [`legged_controllers/src/LeggedController.cpp`]({UPSTREAM}/legged_controllers/src/LeggedController.cpp)
        - [`legged_hw/src/LeggedHWLoop.cpp`]({UPSTREAM}/legged_hw/src/LeggedHWLoop.cpp)
        - `legged_controllers/config/a1/task.info`
        """
    ),
    code(COMMON),
    md(
        """
        ## パッケージの責務

        - `legged_controllers`: controller plugin、参照軌道publisher、500 Hz update
        - `legged_interface`: OCS2のOCPへコスト・制約・参照を組み込む
        - `legged_estimation`: IMU・関節・接地から並進位置/速度を推定
        - `legged_wbc`: 現在時刻の全身QP
        - `legged_common`: hybrid joint interface
        - `legged_hw`, `legged_unitree_hw`, `legged_gazebo`: read/update/write境界
        - OCS2（外部）: SQP、centroidal dynamics、GaitScheduleの基盤

        `LeggedController::update()` の順序:
        1. 状態推定
        2. 観測をMPCへ共有
        3. 最新policy取得・現在時刻で評価
        4. WBC
        5. safety check
        6. 12関節command
        """
    ),
    code(
        """
        # 20 msの間に、500 Hzループと100 Hzループがいつ動くか可視化する。
        dt_fast, dt_slow, duration = 1/500, 1/100, 0.020
        t_fast = np.arange(0, duration + 1e-12, dt_fast)
        t_slow = np.arange(0, duration + 1e-12, dt_slow)

        fig, ax = plt.subplots()
        ax.eventplot([t_fast * 1e3, t_slow * 1e3],
                     lineoffsets=[1, 0], linelengths=0.6,
                     colors=["tab:orange", "tab:blue"])
        ax.set_yticks([0, 1], ["NMPC 100 Hz", "推定/WBC 500 Hz"])
        ax.set_xlabel("time [ms]")
        ax.set_title("2つの周期は同期した1本のfor-loopではない")
        plt.show()

        print("fast updates:", len(t_fast), "slow updates:", len(t_slow))
        """
    ),
    md(
        """
        ## コードを読む順序
        `setupMpc()` → `setupMrt()` → `starting()` → `update()` の順で読む。
        `update()`だけを読むと、別threadの `advanceMpc()` と初期policy待ちを見落とす。

        実装値は horizon 1.0 s、`sqp.dt=0.015` s、SQP反復1、MPC 100 Hz、
        hardware/WBC 500 Hz。予測点数は概算で約67だが、イベント時刻で区間が分割されるため、
        常に固定67変数と決めつけない。

        ### 変更課題
        MPCを50 Hzへ下げる前に確認するもの:
        solver時間、policy age、500 Hz側の補間、接触切替の遅れ、閉ループ安定性。
        """
    ),
]


NOTEBOOKS["02_state_input_frames.ipynb"] = [
    md(
        f"""
        # 02 — 状態・入力・座標系

        ## 目的
        次元が合うだけの誤接続を防ぐ。`x(24)`, `u(24)`, `rbdState(36)` は同じ24/36個の
        「数」ではなく、順序・単位・frameを含む契約である。

        実装対応:
        - `legged_controllers/config/a1/task.info`
        - `ocs2_legged_robot` の centroidal model helpers
        - [`StateEstimateBase.cpp`]({UPSTREAM}/legged_estimation/src/StateEstimateBase.cpp)
        """
    ),
    code(COMMON),
    md(
        r"""
        ## 主要ベクトル

        \[
        x=[v_{\mathrm{com}}(3),\,L/m(3),\,p_b(3),\,(\psi,\theta,\phi)(3),\,q_j(12)]
        \]
        \[
        u=[f_c(12),\,v_j(12)]
        \]

        `rbdState(36)`:
        ZYX(3), base position(3), joint angles(12), world angular velocity(3),
        world linear velocity(3), joint velocities(12)。

        - world = odom側、base = 胴体固定側
        - 姿勢は ZYX の **格納順 yaw, pitch, roll**
        - 関節順と接触脚順はコメント上で差がある箇所があるため、名前を正本にする
        """
    ),
    code(
        """
        # 明示的なsliceで契約をコード化する。
        x = np.zeros(24)
        blocks_x = {
            "v_com": slice(0, 3), "L_over_m": slice(3, 6),
            "base_position": slice(6, 9), "zyx": slice(9, 12),
            "joint_angles": slice(12, 24),
        }
        x[blocks_x["base_position"]] = [1.0, 2.0, 0.30]
        x[blocks_x["zyx"]] = [np.deg2rad(30), 0.0, 0.0]
        assert all(x[s].shape == (3,) for k, s in blocks_x.items() if k != "joint_angles")
        assert x[blocks_x["joint_angles"]].shape == (12,)
        x
        """
    ),
    code(
        """
        # body前方速度をworldへ回す。yaw=30 degならx/y双方に成分が出る。
        def Rz(yaw):
            c, s = np.cos(yaw), np.sin(yaw)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        v_body = np.array([0.5, 0.0, 0.0])
        yaw = x[9]
        v_world = Rz(yaw) @ v_body
        print("v_body [m/s]:", v_body)
        print("v_world [m/s]:", v_world)
        assert np.allclose(np.linalg.norm(v_world), np.linalg.norm(v_body))
        """
    ),
    md(
        """
        ## よくある誤り
        - `x[:3]`をbase位置だと思う（実際は正規化並進運動量/CoM速度）。
        - `u[12:]`をトルクだと思う（実際は関節速度）。
        - local IMU角速度とworld角速度を混ぜる。
        - `LF,LH,RF,RH` と接触名の順を無検証で同じとみなす。

        ### 演習
        すべての境界に `shape`, `unit`, `frame`, `leg order`, `rate` の5項目を書く。
        これが書けない配列は、数式変更より先に調査する。
        """
    ),
]


NOTEBOOKS["03_command_reference_gait.ipynb"] = [
    md(
        f"""
        # 03 — ユーザー指令・2点参照・Gait

        ## 目的
        人の速度指令をNMPC参照へ変換する過程と、接地scheduleが別経路である理由を理解する。

        実装対応:
        - [`TargetTrajectoriesPublisher.cpp`]({UPSTREAM}/legged_controllers/src/TargetTrajectoriesPublisher.cpp)
        - `legged_controllers/config/a1/reference.info`
        - `legged_controllers/config/a1/gait.info`
        """
    ),
    code(COMMON),
    md(
        r"""
        `/cmd_vel`で使うのは \([v_x,v_y,v_z,\dot\psi]\) の4成分。
        並進速度を現在姿勢でworldへ回し、\(T=1.0\) s積分する。

        \[
        p_{xy}^{+}=p_{xy}+v_{W,xy}T,\qquad
        \psi^+=\psi+\dot\psi T
        \]

        高さは `comHeight=0.3 m`、pitch/rollは0、関節参照はdefaultへ固定。
        参照軌道は現在と1秒先の **2点だけ**で、区間内は線形補間される。
        """
    ),
    code(
        """
        def Rz(yaw):
            c, s = np.cos(yaw), np.sin(yaw)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def make_two_point_reference(pose, cmd, horizon=1.0, com_height=0.30):
            # pose = [x,y,z,yaw,pitch,roll], cmd = [vx,vy,vz,yaw_rate]
            yaw = pose[3]
            v_world = Rz(yaw) @ np.asarray(cmd[:3])
            target = np.asarray(pose, dtype=float).copy()
            target[:2] += v_world[:2] * horizon
            target[2] = com_height
            target[3] += cmd[3] * horizon
            target[4:] = 0.0
            return np.array([0.0, horizon]), np.vstack([pose, target]), v_world

        pose = np.array([0, 0, 0.30, np.deg2rad(30), 0, 0], dtype=float)
        times, poses, v_world = make_two_point_reference(pose, [0.5, 0, 0, 0.2])
        print("times:", times)
        print("world velocity:", v_world)
        print("pose endpoints:\\n", poses)
        """
    ),
    code(
        """
        # trot: 0.0–0.3 sはLF/RH、0.3–0.6 sはRF/LH。
        leg_names = np.array(["LF", "RF", "LH", "RH"])
        def trot_contact(t):
            phase = np.mod(t, 0.6)
            return np.array([1, 0, 0, 1], dtype=bool) if phase < 0.3 else np.array([0, 1, 1, 0], dtype=bool)

        ts = np.linspace(0, 1.2, 241)
        contacts = np.array([trot_contact(t) for t in ts]).T
        fig, ax = plt.subplots()
        ax.imshow(contacts, aspect="auto", interpolation="nearest",
                  extent=[ts[0], ts[-1], 3.5, -0.5], cmap="Blues")
        ax.set_yticks(range(4), leg_names)
        ax.set_xlabel("time [s]")
        ax.set_title("Gaitは速度参照とは独立した接地時間割")
        plt.show()
        """
    ),
    md(
        """
        ## コード上の注意
        `latestObservation_.time == 0` の間は指令を捨てる。初期観測なしに参照を作らない。
        `goal`経路は距離と上限速度から到達時刻を決め、速度経路と違って運動量参照をゼロにする。

        ### 変更課題
        - horizonを2秒にするだけでは、solver負荷と参照到達時間が同時に変わる。
        - 速度に応じて自動でtrotへ切替える機能は現行標準経路にはない。
        - 地形に応じた高さ参照も現行にはない。追加時はframeと地形基準を明示する。
        """
    ),
]


NOTEBOOKS["04_state_estimation.ipynb"] = [
    md(
        f"""
        # 04 — 線形Kalman状態推定

        ## 背景・目的
        浮動baseのworld位置には直接encoderがない。IMU加速度を積分しつつ、接地足がworldに
        固定されるという観測でdriftを抑える。姿勢と関節までKFが推定する、という誤解を解く。

        実装対応:
        - [`LinearKalmanFilter.cpp`]({UPSTREAM}/legged_estimation/src/LinearKalmanFilter.cpp)
        - [`StateEstimateBase.cpp`]({UPSTREAM}/legged_estimation/src/StateEstimateBase.cpp)
        """
    ),
    code(COMMON),
    md(
        r"""
        実装のKF状態は
        \[
        \hat x=[p_b(3),v_b(3),p_{f,1}(3),\ldots,p_{f,4}(3)]\in\mathbb R^{18}.
        \]
        姿勢はIMU quaternion、関節角/速度はencoderを使う。KF対象はbase並進と足world位置。

        予測:
        \[
        p^+=p+\Delta t\,v+\tfrac12\Delta t^2 a_W,\quad v^+=v+\Delta t a_W.
        \]
        接地足では \(p_b-p_f\) とbase速度を強く観測し、遊脚では足process noiseを100倍して
        world固定という仮定を弱める。
        """
    ),
    code(
        """
        # 教育用1次元KF: [base位置, base速度, 足位置]。
        # 実装はこれを3軸・4足へ拡張した18状態である。
        dt = 1/500
        A = np.array([[1, dt, 0], [0, 1, 0], [0, 0, 1]], float)
        B = np.array([0.5*dt**2, dt, 0.0])
        # 相対位置 base-foot と base速度を観測
        C = np.array([[1, 0, -1], [0, 1, 0]], float)

        xhat = np.array([0.0, 0.0, 0.0])
        P = np.eye(3) * 0.1
        rng = np.random.default_rng(4)
        history = []
        true_x = np.array([0.0, 0.2, 0.0])

        for k in range(1000):
            true_x[0] += dt * true_x[1]
            a_meas = rng.normal(0, 0.15)
            contact = k < 600
            Q = np.diag([1e-8, 2e-5, 1e-7 if contact else 1e-2])
            R = np.diag([2e-4 if contact else 2e-1, 5e-3 if contact else 2e-1])
            xhat = A @ xhat + B * a_meas
            P = A @ P @ A.T + Q
            y = C @ true_x + rng.multivariate_normal(np.zeros(2), R)
            S = C @ P @ C.T + R
            K = np.linalg.solve(S, C @ P).T
            xhat += K @ (y - C @ xhat)
            P = (np.eye(3) - K @ C) @ P
            history.append([k*dt, true_x[0], xhat[0], contact])

        h = np.asarray(history)
        plt.plot(h[:,0], h[:,1], label="true base position")
        plt.plot(h[:,0], h[:,2], label="KF estimate")
        plt.axvline(600*dt, color="k", ls="--", label="foot becomes swing")
        plt.xlabel("time [s]"); plt.ylabel("position [m]"); plt.legend(); plt.show()
        """
    ),
    md(
        """
        ## 読み方と限界
        - IMU値はworldへ回して重力 `[0,0,-9.81]` を加える。符号ミスは静止時加速度で検査する。
        - 接地はA1実機で `footForce > 40` のbool。確率的接触ではない。
        - visual odometry topicが来た場合だけ位置を上書きする枝がある。
        - `CheaterController` はground truthを使うsim専用で、実機性能の証拠にはならない。

        ### チューニング順
        センサ静止bias → frame/重力 → 接触判定 → measurement noise → process noise。
        noiseを先に触り、frame誤りを「滑らかに隠す」のは避ける。
        """
    ),
]


NOTEBOOKS["05_centroidal_dynamics.ipynb"] = [
    md(
        f"""
        # 05 — Centroidal力学

        ## 目的
        NMPCが予測する「胴体と関節の縮約力学」を、接触力の合力・合momentから理解する。
        本実装の既定は単一剛体だけではなく `FullCentroidalDynamics` であり、関節運動の影響を残す。

        実装対応:
        - `legged_interface/src/LeggedInterface.cpp`
        - OCS2 `LeggedRobotDynamicsAD`（外部。完全な成分ODEはこのworkspaceでは未照合）
        """
    ),
    code(COMMON),
    md(
        r"""
        接触力 \(f_i\) とCoMから足へのlever arm \(r_i\) に対して
        \[
        m\dot v=\sum_i f_i+m g,\qquad
        \dot L=\sum_i r_i\times f_i.
        \]
        静止なら \(\sum f_{i,z}=mg\)。4脚等分は初期guessにはなるが、加速・姿勢moment・接触数が
        変われば一般には等分でない。
        """
    ),
    code(
        """
        mass = 12.5
        g = np.array([0.0, 0.0, -9.81])
        feet = np.array([
            [ 0.25,  0.15, -0.30],  # LF
            [ 0.25, -0.15, -0.30],  # RF
            [-0.25,  0.15, -0.30],  # LH
            [-0.25, -0.15, -0.30],  # RH
        ])
        forces = np.tile(np.array([0, 0, mass*9.81/4]), (4, 1))

        net_force = forces.sum(axis=0) + mass*g
        net_moment = np.cross(feet, forces).sum(axis=0)
        print("net force incl. gravity [N]:", net_force)
        print("net moment about CoM [N m]:", net_moment)
        assert np.allclose(net_force, 0)
        assert np.allclose(net_moment, 0)
        """
    ),
    code(
        """
        # 対角2脚支持で前進加速度0.5 m/s^2を作る例。
        contact = np.array([1, 0, 0, 1], dtype=bool)
        forces_trot = np.zeros((4, 3))
        forces_trot[contact, 0] = mass * 0.5 / contact.sum()
        forces_trot[contact, 2] = mass * 9.81 / contact.sum()
        a_com = forces_trot.sum(axis=0) / mass + g
        moment = np.cross(feet, forces_trot).sum(axis=0)
        print("forces [N]:\\n", forces_trot)
        print("CoM acceleration [m/s^2]:", a_com)
        print("moment [N m]:", moment)
        """
    ),
    md(
        """
        ## Full centroidal と SRBD
        `centroidalModelType=0` はfull centroidal。状態は正規化centroidal momentumと
        base pose、joint anglesで、入力後半のjoint velocityを通して形状変化も予測へ入る。
        SRBDへ切替える設定値はあるが、A1既定ではない。

        ### 数式変更前の検査
        1. 静止4脚で重力が相殺される。
        2. 左右対称力でroll momentが0。
        3. 接触していない脚の力が0。
        4. 単位をN、N m、kg、m、sへ統一。
        """
    ),
]


NOTEBOOKS["06_nmpc_ocp_and_tuning.ipynb"] = [
    md(
        f"""
        # 06 — NMPCの最適制御問題と重み

        ## 目的
        OCS2 SQP-NMPCの役割を、解ける小さな有限ホライズン問題へ縮約して体験する。
        実装の24/24問題そのものをPythonで再実装する章ではない。

        実装対応:
        - `LeggedInterface::setupOptimalControlProblem()`
        - `LeggedRobotQuadraticTrackingCost`
        - `LeggedController::setupMpc()` の `SqpMpc`
        """
    ),
    code(COMMON),
    md(
        r"""
        \[
        \min_{u(\cdot)}\phi(x(T))+\int_0^T
        \|x-x^\mathrm{ref}\|_Q^2+\|u-u^\mathrm{wc}\|_R^2\,dt
        \]
        subject to centroidal dynamics、接触等式、摩擦・自己衝突制約。

        実装値: horizon 1.0 s、shooting dt 0.015 s、SQP iteration 1。
        入力参照はゼロ力ではなく、接地脚へ体重を配る `weightCompensatingInput`。
        """
    ),
    code(
        """
        # 1D double integratorで「位置/速度追従 vs 力の節約」を見る。
        # x=[position, velocity], u=acceleration。有限ホライズンLQRを後退Riccatiで解く。
        def finite_horizon_lqr(q_pos=20.0, q_vel=2.0, r_u=0.2, dt=0.05, N=20):
            A = np.array([[1, dt], [0, 1]])
            B = np.array([[0.5*dt**2], [dt]])
            Q = np.diag([q_pos, q_vel])
            R = np.array([[r_u]])
            P = Q.copy()
            gains = []
            for _ in range(N):
                K = np.linalg.solve(R + B.T@P@B, B.T@P@A)
                gains.append(K)
                P = Q + A.T@P@(A-B@K)
            gains.reverse()
            x = np.array([-0.5, 0.0])
            xs, us = [x.copy()], []
            for K in gains:
                u = float((-K @ x).item())
                x = A@x + B[:,0]*u
                xs.append(x.copy()); us.append(u)
            return np.asarray(xs), np.asarray(us)

        configs = [(5, 0.2, "soft position"), (50, 0.2, "hard position"), (50, 2.0, "expensive input")]
        fig, axes = plt.subplots(1, 2, figsize=(11,4))
        for q, r, label in configs:
            xs, us = finite_horizon_lqr(q_pos=q, r_u=r)
            axes[0].plot(xs[:,0], label=label)
            axes[1].plot(us, label=label)
        axes[0].set_ylabel("position error [m]"); axes[1].set_ylabel("u [m/s²]")
        for ax in axes: ax.set_xlabel("step"); ax.legend()
        plt.show()
        """
    ),
    md(
        """
        ## 実装のQ/Rを読む
        A1で大きい状態重みは水平位置1000、高さ1500、yaw 100、roll/pitch 300。
        入力R後半は設定ファイル上「足速度」だが、名目姿勢のJacobianで
        \(R_{v_j}=J_b^\top R_\mathrm{task}J_b\) と関節速度blockへ写される。

        ### チューニングの因果
        - Qを上げる: 誤差を早く戻すが、GRF・関節速度・制約余裕を使う。
        - Rを上げる: 入力は滑らか/小さくなるが、追従が遅れる。
        - horizonを伸ばす: 先を見るが、計算量とモデル誤差の影響が増える。
        - SQP反復を増やす: 収束改善の可能性と締切超過を同時に評価する。

        一度に1群だけ変更し、solver time、KKT/constraint violation、姿勢誤差、
        torque saturation、slipを同じrunで記録する。
        """
    ),
]


NOTEBOOKS["07_contact_constraints_and_swing.ipynb"] = [
    md(
        f"""
        # 07 — 接触制約・摩擦・遊脚

        ## 目的
        Gaitのboolが、NMPC内で「立脚の制約」と「遊脚の制約」を切替える仕組みを理解する。

        実装対応:
        - `legged_interface/constraint/`
        - `SwingTrajectoryPlanner.cpp`
        - `SwitchedModelReferenceManager.cpp`
        """
    ),
    code(COMMON),
    md(
        r"""
        脚 \(i\) ごとの主要条件:

        - 遊脚: \(f_i=0\)
        - 立脚: 足速度0
        - 遊脚: plannerが与える法線方向速度
        - 立脚摩擦（NMPC soft constraint）:
          \[
          \mu(F_z+F_g)-\sqrt{F_x^2+F_y^2+\varepsilon}\ge0
          \]
        - 自己衝突距離 \(\ge 0.05\) m

        A1既定 \(\mu=0.3\)。地形法線のsetterは現行実装で未実装例外、
        reference managerは地形高さ0を渡すため、知覚不整地NMPCではない。
        """
    ),
    code(
        """
        # 摩擦円錐の余裕 margin = mu*Fz - ||Ft|| を計算する。
        def friction_margin(force, mu=0.3):
            force = np.asarray(force)
            return mu*force[...,2] - np.linalg.norm(force[...,:2], axis=-1)

        Fz = 60.0
        fx = np.linspace(-30, 30, 161)
        fy = np.linspace(-30, 30, 161)
        FX, FY = np.meshgrid(fx, fy)
        M = 0.3*Fz - np.sqrt(FX**2 + FY**2)
        filled = plt.contourf(FX, FY, M, levels=30, cmap="RdYlBu")
        plt.contour(FX, FY, M, levels=[0], colors="k", linewidths=2)
        plt.colorbar(filled, label="friction margin [N]")
        plt.xlabel("Fx [N]"); plt.ylabel("Fy [N]")
        plt.title("円の内側だけが実行可能 (mu=0.3, Fz=60 N)")
        plt.axis("equal"); plt.show()
        """
    ),
    code(
        """
        # 境界速度を指定した三次Hermite遊脚高さ。実装はspline CPGを使う。
        def hermite_z(s, z0=0.0, z1=0.0, v0=0.05, v1=-0.10, duration=0.3):
            h00 = 2*s**3 - 3*s**2 + 1
            h10 = s**3 - 2*s**2 + s
            h01 = -2*s**3 + 3*s**2
            h11 = s**3 - s**2
            return h00*z0 + h10*duration*v0 + h01*z1 + h11*duration*v1

        s = np.linspace(0, 1, 200)
        z_base = hermite_z(s)
        # 中間liftを加える教育用bump
        z = z_base + 0.08 * 4*s*(1-s)
        plt.plot(s, z)
        plt.xlabel("normalized swing phase"); plt.ylabel("foot height [m]")
        plt.title("swingHeight=0.08 m、lift/touchdown速度の意味")
        plt.show()
        """
    ),
    md(
        """
        ## NMPCとWBCで摩擦形状が違う
        NMPCは正則化した円錐のsoft constraint。WBCは
        \(|F_x|\le\mu F_z, |F_y|\le\mu F_z, F_z\ge0\) の線形pyramid。
        同じ \(\mu=0.3\) でも実行可能集合は一致しない。この差を無視して
        「NMPCでfeasibleだからWBCもfeasible」とは言えない。

        ### 変更課題
        \(\mu\) を上げて歩けた結果は、摩擦推定が改善した証拠ではない。
        実床の摩擦より大きくすると、存在しない横力を計画する。推定値、slip率、
        friction margin、WBC再配分を一緒に確認する。
        """
    ),
]


NOTEBOOKS["08_weighted_wbc.ipynb"] = [
    md(
        f"""
        # 08 — Weighted Whole-Body Control

        ## 目的
        NMPCの現在目標から、全身運動方程式・接触・摩擦・トルク上限を満たす
        関節トルクを解く瞬間QPを理解する。

        実装対応:
        - [`WbcBase.cpp`]({UPSTREAM}/legged_wbc/src/WbcBase.cpp)
        - [`WeightedWbc.cpp`]({UPSTREAM}/legged_wbc/src/WeightedWbc.cpp)
        - `HierarchicalWbc.cpp`（実装あり、既定controllerには未配線）
        """
    ),
    code(COMMON),
    md(
        r"""
        決定変数:
        \[
        z=[\ddot q(18),F_c(12),\tau(12)]\in\mathbb R^{42}.
        \]
        硬い運動方程式:
        \[
        [M,-J^\top,-S^\top]z=-nle.
        \]
        さらに torque box、立脚足加速度0、遊脚力0、摩擦pyramidをhard constraintにする。

        soft taskは遊脚加速度、base加速度、NMPC接触力追従。
        A1既定重みは swing 100、base accel 1、contact force 0.01。
        したがってNMPCのGRFは命令ではなく、WBCがずらせる弱い目標である。
        """
    ),
    code(
        """
        # 小さな等式制約付きweighted least squares。
        # z=[base acceleration a, contact force F, actuator torque tau]
        # hard: m*a - F - tau = -m*g
        def solve_toy_wbc(w_a=1.0, w_f=0.01, a_des=0.0, f_des=100.0,
                          mass=12.5, g=9.81):
            Aeq = np.array([[mass, -1.0, -1.0]])
            beq = np.array([-mass*g])
            C = np.array([[w_a, 0, 0], [0, w_f, 0]], float)
            d = np.array([w_a*a_des, w_f*f_des])
            # KKT: [C'C A'; A 0] [z,lambda] = [C'd,b]
            H = C.T@C + 1e-9*np.eye(3)
            KKT = np.block([[H, Aeq.T], [Aeq, np.zeros((1,1))]])
            rhs = np.r_[C.T@d, beq]
            return np.linalg.solve(KKT, rhs)[:3]

        for wf in [0.001, 0.01, 0.1, 1.0]:
            z = solve_toy_wbc(w_f=wf)
            residual = 12.5*z[0] - z[1] - z[2] + 12.5*9.81
            print(f"w_force={wf:5.3f} -> [a,F,tau]={z}, EoM residual={residual:.2e}")
        """
    ),
    code(
        """
        # 実装の行列shapeを組み立てて、転置・符号の契約を確認する。
        nq, nf, ntau = 18, 12, 12
        M = np.eye(nq)
        J = np.zeros((nf, nq))
        S = np.zeros((ntau, nq)); S[:, 6:] = np.eye(ntau)
        A_eom = np.c_[M, -J.T, -S.T]
        assert A_eom.shape == (18, 42)
        print("EoM matrix shape:", A_eom.shape)
        print("decision slices: qdd=0:18, force=18:30, torque=30:42")
        """
    ),
    md(
        """
        ## Weighted と Hierarchical
        `WeightedWbc`はhard constraints + soft taskの加重和をqpOASESで解く単一QP。
        `HierarchicalWbc`はnull-space階層を実装するが、`LeggedController::init` は
        `WeightedWbc`を生成する。READMEの階層説明を、動いている既定経路と混同しない。

        ### チューニング
        weight変更だけでなく、hard residual、task residual、active constraints、
        torque saturation、qp status、solve timeを記録する。実装はQP失敗時fallbackが
        明示されていないため、運用では前回安全torqueや停止方針も設計対象になる。
        """
    ),
]


NOTEBOOKS["09_hybrid_joint_hardware.ipynb"] = [
    md(
        f"""
        # 09 — 関節ハイブリッド指令とHardware

        ## 目的
        WBC出力をモータへ渡す最後の式と、Gazebo/Unitreeの共通interfaceを理解する。

        実装対応:
        - `legged_common/.../HybridJointInterface.h`
        - [`UnitreeHW.cpp`]({UPSTREAM}/legged_unitree_hw/src/UnitreeHW.cpp)
        - [`LeggedHWSim.cpp`]({UPSTREAM}/legged_gazebo/src/LeggedHWSim.cpp)
        """
    ),
    code(COMMON),
    md(
        r"""
        12関節それぞれへ
        \[
        \tau_\mathrm{cmd}=\tau_\mathrm{WBC}
        +K_p(q^*-q)+K_d(\dot q^*-\dot q).
        \]
        既定は \(K_p=0,\ K_d=3\) なので
        \[
        \tau_\mathrm{cmd}=\tau_\mathrm{WBC}+3(\dot q^*-\dot q).
        \]

        `setCommand`引数順は `(q*, dq*, Kp, Kd, ff)`。
        実機はUnitree LowCmdへ、Gazeboはplugin内で同じ式を計算してeffortへ渡す。
        """
    ),
    code(
        """
        def hybrid_torque(q, dq, q_des, dq_des, tau_ff, kp=0.0, kd=3.0):
            return tau_ff + kp*(q_des-q) + kd*(dq_des-dq)

        q = np.array([0.2, 0.7, -1.4])
        dq = np.array([0.1, -0.2, 0.3])
        q_des = np.array([0.0, 0.8, -1.5])
        dq_des = np.zeros(3)
        tau_ff = np.array([2.0, -5.0, 8.0])
        print("Kp=0:", hybrid_torque(q,dq,q_des,dq_des,tau_ff))
        print("Kp=20:", hybrid_torque(q,dq,q_des,dq_des,tau_ff,kp=20))
        print("位置誤差はKp=0なら直接torqueへ入らない")
        """
    ),
    code(
        """
        # 1関節の簡易慣性モデルでKdの効果を見る（実機同定モデルではない）。
        def simulate_joint(kd, dt=0.002, duration=1.0):
            inertia, damping = 0.08, 0.05
            q, dq, dq_des, tau_ff = 0.0, 2.0, 0.0, 0.0
            out = []
            for k in range(int(duration/dt)):
                tau = tau_ff + kd*(dq_des-dq)
                tau = np.clip(tau, -33.5, 33.5)
                ddq = (tau-damping*dq)/inertia
                dq += dt*ddq; q += dt*dq
                out.append((k*dt, q, dq, tau))
            return np.asarray(out)

        for kd in [0.0, 1.0, 3.0, 8.0]:
            h = simulate_joint(kd)
            plt.plot(h[:,0], h[:,2], label=f"Kd={kd}")
        plt.xlabel("time [s]"); plt.ylabel("joint velocity [rad/s]")
        plt.legend(); plt.show()
        """
    ),
    md(
        """
        ## 安全境界
        - A1 torque limitは各関節33.5 N m。
        - Unitree側はposition limitとpower protectionを適用する。
        - controller未接続時、実機readはffと目標速度を0、Kd=3へ戻してdampingを残す。
        - `SafetyChecker`は実装上rollが±π/2を超えるかを見るが、pitch・torque・temperatureを
          網羅する安全監視ではない。

        ### 変更課題
        Kpを追加するとWBCのforce controlと位置springが競合しうる。
        実機変更はsimの見た目ではなく、passivity/遅延、torque/current limit、接触衝撃、
        joint別inertia、停止時挙動を先に評価する。
        """
    ),
]


NOTEBOOKS["10_multirate_integration.ipynb"] = [
    md(
        f"""
        # 10 — Multi-rate統合とpolicy

        ## 目的
        100 Hzの未来policyと500 Hzの現在実行を、zero-order holdと誤解せず理解する。

        実装対応:
        - `LeggedController::setupMrt()`
        - `LeggedController::starting()`
        - `LeggedController::update()`
        """
    ),
    code(COMMON),
    md(
        """
        NMPC threadは `advanceMpc()` でpolicyを更新する。500 Hz側は毎周期:

        1. 新しい観測を共有
        2. `updatePolicy()`
        3. `evaluatePolicy(t, x, optimizedState, optimizedInput, plannedMode)`
        4. その1点をWBCへ

        `useFeedbackPolicy=false` なので、実装設定ではstate feedback gainより軌道補間が中心。
        NMPC更新間にも時刻は進むため、同じ入力値を5回固定する、と単純化しない。
        """
    ),
    code(
        """
        # 10 msごとのpolicy knotを、2 msの実行時刻で線形評価する教育例。
        t_policy = np.array([0.000, 0.010, 0.020, 0.030])
        u_policy = np.array([0.0, 10.0, -5.0, 0.0])
        t_fast = np.arange(0, 0.0301, 0.002)
        u_interp = np.interp(t_fast, t_policy, u_policy)
        u_hold = u_policy[np.minimum(np.searchsorted(t_policy, t_fast, side="right")-1,
                                     len(u_policy)-1)]

        plt.step(t_fast*1e3, u_hold, where="post", label="naive sample-and-hold")
        plt.plot(t_fast*1e3, u_interp, "o-", label="time interpolation image")
        plt.plot(t_policy*1e3, u_policy, "ks", label="policy knots")
        plt.xlabel("time [ms]"); plt.ylabel("planned force component [N]")
        plt.legend(); plt.show()
        """
    ),
    code(
        """
        # policy ageを監視する最小ロジック。実システムではclockとthread-safe timestampを使う。
        fast_times = np.arange(0, 0.050, 0.002)
        update_times = np.array([0.000, 0.010, 0.020, 0.041])  # 30 ms更新が遅れた例
        ages = []
        for t in fast_times:
            available = update_times[update_times <= t]
            ages.append(t - available[-1])
        ages = np.asarray(ages)
        plt.plot(fast_times*1e3, ages*1e3)
        plt.axhline(15, color="r", ls="--", label="example watchdog 15 ms")
        plt.xlabel("time [ms]"); plt.ylabel("policy age [ms]"); plt.legend(); plt.show()
        print("max policy age [ms]:", ages.max()*1e3)
        """
    ),
    md(
        """
        ## 統合時に測るもの
        solver wall timeの平均だけでは不十分。

        - worst-case / percentile solve time
        - policy age
        - missed deadline連続回数
        - 接触mode切替とpolicy時刻のずれ
        - WBC solve time・status
        - sensor timestampと制御時刻

        `starting()` は初期policyを受け取るまでMPCを進める。初期化を飛ばすと、
        未定義policyをWBCへ渡す危険がある。
        """
    ),
]


NOTEBOOKS["11_tuning_and_equation_changes.ipynb"] = [
    md(
        f"""
        # 11 — チューニングと数式変更の実践手順

        ## 最終到達目標
        現象を「どの層の、どの式の、どの残差か」へ戻し、一度に1仮説だけ変更する。
        この章は魔法の推奨値ではなく、再現可能な変更手順を作る。

        照合対象: [`legged_control` `{UPSTREAM_COMMIT[:10]}`]({UPSTREAM})
        """
    ),
    code(COMMON),
    md(
        """
        ## 変更の順序
        1. baseline commit/config、robot、gait、指令、床、seedを固定
        2. 失敗を層へ分類: sensor/frame → estimator → reference/gait → NMPC → WBC → joint/HW
        3. 数式の単位・符号・shapeを手計算と静的testで確認
        4. 変更parameterは1群、式変更は1項だけ
        5. constraint residual、solver status/time、tracking、torque、slipを保存
        6. 改善と副作用を比較し、戻せる差分にする

        ### 症状から最初に見る場所
        - 静止でbase位置drift: IMU重力/frame、接触flag、KF noise
        - 横滑り: 実摩擦、NMPC円錐margin、WBC pyramid、接触誤判定
        - 足先が遅れる: swing task residual、WBC weight、torque saturation
        - 姿勢追従が弱い: reference、Q、feasibility、base task weight
        - torque振動: policy age、接触切替、WBC active set、Kd、delay
        """
    ),
    code(
        """
        # 変更記録を機械的に比較するための最小schema。
        baseline = {
            "mu": 0.3, "horizon_s": 1.0, "mpc_hz": 100,
            "wbc_weight_swing": 100.0,
            "wbc_weight_base": 1.0,
            "wbc_weight_force": 0.01,
            "joint_kp": 0.0, "joint_kd": 3.0,
        }
        trial = baseline | {"wbc_weight_force": 0.03}
        changed = {k: (baseline[k], trial[k]) for k in baseline if baseline[k] != trial[k]}
        assert len(changed) == 1, "一度に1仮説の規則に反している"
        changed
        """
    ),
    md(
        r"""
        ## 数式変更テンプレート

        例: 摩擦を等方円錐から異方性ellipseへ変えるなら
        \[
        \sqrt{(F_x/\mu_x)^2+(F_y/\mu_y)^2}\le F_z
        \]
        と書き、次を同時に定義する。

        - \(\mu_x,\mu_y\) の物理的意味と同定法
        - \(F_z<0\) を許さない条件
        - smooth化epsilonとgradient
        - NMPC側soft constraintとWBC側linear approximationの整合
        - flat floorで元式へ戻る回帰test

        「式を変える」はC++1行の変更ではなく、model・constraint・solver微分・下位実行・
        testの契約変更である。
        """
    ),
    code(
        """
        # 等方円錐と異方性ellipseのmarginを比較する。
        def isotropic_margin(fx, fy, fz, mu):
            return mu*fz - np.hypot(fx, fy)

        def anisotropic_margin(fx, fy, fz, mux, muy):
            return fz - np.sqrt((fx/mux)**2 + (fy/muy)**2)

        test_forces = np.array([[10, 0, 50], [0, 10, 50], [12, 8, 50], [20, 0, 50]])
        for f in test_forces:
            old = isotropic_margin(*f, mu=0.3)
            new = anisotropic_margin(*f, mux=0.4, muy=0.2)
            print(f"F={f}: isotropic={old:7.3f}, anisotropic={new:7.3f}")
        """
    ),
    md(
        """
        ## 最終演習（順番を守る）
        1. `task.info`のQ/Rを全24状態・24入力へ対応付ける。
        2. 静止4脚でweight-compensating inputと力学残差を計算する。
        3. trot 2脚支持でNMPC円錐とWBC pyramid双方のmarginを出す。
        4. WBCの各task residualをlogできる設計を作る。
        5. policy age watchdogとQP failure fallbackを設計する。
        6. Qを1群だけ変え、追従・constraint・torque・solve timeを比較する。
        7. 最後に、摩擦または遊脚軌道の式を1つ変更し、元式へ戻る回帰testを書く。

        ## 修了判定
        次を説明できれば、コード変更へ進める。

        - x/u/rbd/WBC変数の中身、単位、frame
        - Gait、NMPC、WBC、hybrid jointの責務境界
        - KFが推定するもの/しないもの
        - NMPC円錐とWBC pyramidの差
        - weighted WBCでGRF目標がずれる理由
        - 100/500 Hz間でpolicyが古くなる危険
        - parameter変更と数式変更の検証項目
        """
    ),
]


NOTEBOOKS["12_repository_code_walkthrough.ipynb"] = [
    md(
        f"""
        # 12 — 実C++コードを端から端まで追う

        ## 背景
        ここまでのNotebookは式を小さなPythonへ写した。この章では上流commit
        `{UPSTREAM_COMMIT[:10]}` の実コードを、閉ループの呼出順に戻して読む。

        ## 目的
        各blockについて、入力、C++シンボル、数式、出力、次の呼出先を一行で説明できるようにする。

        ## 結論
        制御の中心は `LeggedController::update()` の短い配線である。複雑さは各block内部にあり、
        500 Hzの配線は **推定 → policyの現在値 → WeightedWbc → hybrid joint** の順を崩さない。

        ## 実行境界
        このcall graphは `external/legged_control/` のcommit `{UPSTREAM_COMMIT}` にある
        **ROS1 + OCS2原実装**を説明する。`src/legged_control_mujoco/adapter.py` はproject所有の
        ROS-free実行境界であり、gait/state/input/WBC/hybrid-commandの契約を移植する一方、
        OCS2 SQPを瞬時force plannerとMuJoCo acceleration-level WBCへ置換する。
        したがってadapter結果を「OCS2 SQPを実行した結果」と呼ばない。

        主な正本:
        - `external/legged_control/legged_controllers/src/LeggedController.cpp`
        - `external/legged_control/legged_controllers/src/TargetTrajectoriesPublisher.cpp`
        - `external/legged_control/legged_estimation/src/LinearKalmanFilter.cpp`
        - `external/legged_control/legged_interface/src/LeggedInterface.cpp`
        - `external/legged_control/legged_wbc/src/WbcBase.cpp`
        - `external/legged_control/legged_wbc/src/WeightedWbc.cpp`
        """
    ),
    code(COMMON),
    md(
        """
        ## 実行時call graph

        ```text
        LeggedController::init
          ├─ setupLeggedInterface
          │    └─ LeggedInterface::setupOptimalControlProblem
          │         ├─ dynamics: LeggedRobotDynamicsAD
          │         ├─ cost: LeggedRobotQuadraticTrackingCost
          │         └─ constraints: zero force/velocity, friction, collision
          ├─ setupMpc
          │    ├─ SqpMpc
          │    ├─ GaitReceiver
          │    └─ RosReferenceManager
          ├─ setupMrt
          │    └─ thread: advanceMpc() @ 100 Hz
          ├─ setupStateEstimate
          │    └─ KalmanFilterEstimate
          └─ WeightedWbc

        LeggedHWLoop / gazebo_ros_control @ 500 Hz
          └─ LeggedController::update
               ├─ updateStateEstimation
               ├─ setCurrentObservation
               ├─ updatePolicy + evaluatePolicy(now)
               ├─ WeightedWbc::update
               ├─ SafetyChecker::check
               └─ HybridJointHandle::setCommand
        ```
        """
    ),
    code(
        r'''
# --- Block 1: 上流の500 Hz配線を、コメント付きでそのまま読む ---
# この文字列は実行用Pythonではなく、照合commitのC++抜粋を教材として表示する。
cpp_update = r"""
void LeggedController::update(time, period) {
  updateStateEstimation(time, period);
  // IMU・q・dq・接地 → rbdState(36) → centroidal observation x(24)

  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  // 現在x(24)を100 Hz NMPC threadへ共有する

  mpcMrtInterface_->updatePolicy();
  // NMPC threadが完成させた最新policyを500 Hz側へ取り込む

  evaluatePolicy(now, x, optimizedState, optimizedInput, plannedMode);
  // 未来列全部ではなく「今」の x*(24), u*(24), mode だけを切り出す

  vector_t z = wbc_->update(xStar, uStar, rbdState, mode, period);
  vector_t torque = z.tail(12);
  // WBC決定変数 z=[qdd(18), Fc(12), tau(12)] の末尾だけを使う

  setCommand(qStar, dqStar, 0, 3, torque);
  // tau_cmd = tau + 0*(q*-q) + 3*(dq*-dq)
}
"""
print(cpp_update)
        '''
    ),
    code(
        """
# --- Block 2: 同じ配線をshape付きPython契約にする ---
# 意図: C++の型に隠れた次元をassertし、block間の誤接続を検出する。
def controller_update_contract(rbd_state, observation_x, policy_x, policy_u, wbc_solution):
    assert rbd_state.shape == (36,)       # KF / rigid-body state
    assert observation_x.shape == (24,)   # NMPC initial state
    assert policy_x.shape == (24,)        # optimized state at now
    assert policy_u.shape == (24,)        # GRF12 + joint velocity12 at now
    assert wbc_solution.shape == (42,)    # qdd18 + force12 + torque12

    q_des = policy_x[12:24]               # centroidal stateのjoint angle block
    dq_des = policy_u[12:24]              # NMPC input後半はjoint velocity
    tau_ff = wbc_solution[30:42]           # WBC末尾12だけがmotor feedforward
    hybrid = np.c_[q_des, dq_des,
                   np.zeros(12),           # Kp=0
                   np.full(12, 3.0),       # Kd=3
                   tau_ff]
    return hybrid                          # 12 joints × [q*,dq*,Kp,Kd,ff]

hybrid = controller_update_contract(
    np.zeros(36), np.zeros(24), np.zeros(24), np.zeros(24), np.zeros(42)
)
print("hybrid command shape:", hybrid.shape)
        """
    ),
    md(
        r"""
        ## Block 3 — 参照生成: C++と式

        `cmdVelToTargetTrajectories()` は
        ```cpp
        cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);
        target.x = current.x + cmdVelRot.x * TIME_TO_TARGET;
        target.y = current.y + cmdVelRot.y * TIME_TO_TARGET;
        target.yaw = current.yaw + cmdVel(3) * TIME_TO_TARGET;
        trajectories.stateTrajectory[0].head(3) = cmdVelRot;
        trajectories.stateTrajectory[1].head(3) = cmdVelRot;
        ```
        を行う。対応式は
        \[
        v_W=R_{ZYX}v_{cmd},\quad p_{xy}^+=p_{xy}+v_{W,xy}T,\quad
        \psi^+=\psi+\dot\psi T.
        \]
        出力は2時刻、状態2×24、入力2×24。入力trajectoryは次元合わせのzeroで、
        tracking cost側はweight-compensating inputを使う。
        """
    ),
    md(
        r"""
        ## Block 4 — 推定: C++と式

        `KalmanFilterEstimate` constructorは
        `numState = 6 + 3*numContacts = 18`,
        `numObserve = 2*3*numContacts + numContacts = 28` を作る。
        `update()` は
        \[
        A_{p,v}=\Delta tI,\quad B_p=\frac12\Delta t^2I,\quad B_v=\Delta tI
        \]
        を毎周期更新する。`StateEstimateBase`から姿勢と関節を受け、
        KFでbase位置・速度と足world位置を更新する。

        ```text
        q,dq ─→ Pinocchio FK ─→ 足のbase相対位置/速度 ─┐
        IMU orientation,a ─→ world acceleration ───────┼→ KF xHat(18)
        contact ─→ Q/Rを接地/遊脚で切替 ───────────────┘
        ```
        """
    ),
    code(
        """
# --- Block 5: WBCの実C++行列をshapeと式へ戻す ---
# C++:
#   a << data.M, -j_.transpose(), -s.transpose();
#   b = -data.nle;
# 数式:
#   [M, -J^T, -S^T] [qdd,Fc,tau]^T = -nle
nq, nf, ntau = 18, 12, 12
M = np.eye(nq)
J = np.zeros((nf, nq))
S = np.c_[np.zeros((ntau, 6)), np.eye(ntau)]
A_eom = np.c_[M, -J.T, -S.T]
assert A_eom.shape == (18, 42)

# C++ frictionPyramic:
# [ 0, 0,-1]F <= 0       -> Fz >= 0
# [±1, 0,-mu]F <= 0      -> |Fx| <= mu Fz
# [ 0,±1,-mu]F <= 0      -> |Fy| <= mu Fz
mu = 0.3
D_friction = np.array([
    [0,0,-1], [1,0,-mu], [-1,0,-mu], [0,1,-mu], [0,-1,-mu]
])
test_force = np.array([5.0, 3.0, 30.0])
print("EoM A shape:", A_eom.shape)
print("friction inequalities D F:", D_friction @ test_force, "<= 0 is feasible")
assert np.all(D_friction @ test_force <= 0)
        """
    ),
    md(
        r"""
        ## Block 6 — WeightedWbcの解法

        実コードはhard constraintsを
        `formulateFloatingBaseEomTask + torqueLimits + frictionCone + noContactMotion`
        で作り、soft taskを
        `swingLeg*100 + baseAccel*1 + contactForce*0.01`
        で作る。

        \[
        H=A_{soft}^TA_{soft},\qquad g=-A_{soft}^Tb_{soft}
        \]
        をqpOASESへ渡し、`nWsr=20`で解く。solver return codeを分岐せず
        `getPrimalSolution`するため、failure fallbackは実装改善候補である。

        ## 全体理解の確認
        1. `optimizedInput[:12]` と `[12:]` は何か。
        2. 推定modeと`plannedMode`のどちらがWBC接触flagになるか。
        3. frictionがNMPCではsoft円錐、WBCではhard pyramidである影響は何か。
        4. `HierarchicalWbc`がincludeされても既定で動かない根拠はどこか。
        5. `setCommand(...,0,3,tau)` の位置項が消えることを式で示せるか。

        次のNotebookで、この契約を30 scenarioへ流す。
        """
    ),
]


NOTEBOOKS["13_model_benchmark_30_scenarios.ipynb"] = [
    md(
        """
        # 13 — equation-level proxy benchmark（30シナリオ）

        ## 背景
        パラメータを説明するだけでは、制御モデルがどこまで要求wrenchを実現できるか分からない。
        そこで、`legged_control` の中心式であるcentroidal dynamics、接触schedule、摩擦制約、
        WBCのtorque limit、100 Hz計画を一つの小さな閉ループへまとめ、難易度別30条件を同じ指標で流す。
        これは4秒の **equation-level pre-benchmark** であり、repository performanceではない。

        ## 重要な測定範囲
        これは `external/legged_control` のROS node、OCS2 SQP、Pinocchio WBC、Gazebo A1を起動した
        end-to-end benchmarkではない。この環境には `roscore`, `roslaunch`, `catkin` が無いためである。
        ここで測るのは **上流と同じ物理契約を持つ教育用model-level benchmark**:

        - centroidal合力・合moment
        - stance/trot接触schedule
        - 摩擦pyramid投影
        - 33.5 N m torque proxy limit
        - 状態noise、policy delay、外乱

        したがって、結果は実機歩行性能ではなく、要求wrenchの実現性・tracking・計算時間の比較である。

        ## 目的
        1. 30条件を同一コードで再実行する。
        2. tracking、姿勢、高さ、wrench residual、torque飽和、計算時間を保存する。
        3. 難しくなるほど、どの制約が先に支配的になるかを分析する。

        ## 結論の読み方
        `pass` はこの縮約modelの閾値を満たした意味だけを持つ。上流repositoryの性能を断定するには、
        ROS Noetic + OCS2 + Gazebo環境で同じscenario定義を移植し直す必要がある。
        """
    ),
    code(
        COMMON
        + """
import json
import time
import pandas as pd

OUT = ROOT / "notebook_legged" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
"""
    ),
    md(
        r"""
        ## データの流れ

        ```text
        Scenario
          │ vx_ref, slope, mu, gait, noise, delay, disturbance
          ▼
        reference + noisy measured state
          │
          ▼ 100 Hz
        centroidal feedback
          │ desired wrench w*=[Fx,Fy,Fz,Mx,My,Mz]
          ▼
        contact force allocation
          │ min ||A(contact)F - w*||² + λ||F||²
          ▼
        friction/normal-force projection
          │ Fz>=0, |Fx|<=mu Fz, |Fy|<=mu Fz
          ▼
        WBC torque proxy
          │ tau_i = J_i^T F_i, |tau|<=33.5 N m
          ▼
        delayed plant integration
          │ m vdot = ΣF + mg + disturbance
          │ I ωdot = Σ(r_i×F_i) + disturbance moment
          └──────────── state ───────────→ feedback
        ```

        上流との対応:

        - desired wrench / centroidal model → `LeggedRobotDynamicsAD`
        - contact schedule → `GaitSchedule`
        - force constraint → NMPC `FrictionConeConstraint`
        - force allocation → NMPCの現在入力 $u^*[0:12]$ の縮約
        - torque proxy → WBCの $M\ddot q-J^TF-S^T\tau+nle=0$ の縮約
        """
    ),
    md(
        r"""
        ## 数式

        controller:
        \[
        F_x^*=m k_v(v_x^{ref}-\hat v_x),\qquad
        F_z^*=m[g+k_z(z^{ref}-\hat z)-d_z\hat v_z]
        \]
        \[
        M_{x,y}^*=k_R(R^{ref}_{x,y}-\hat R_{x,y})-d_R\hat\omega_{x,y}
        \]

        active contactの力を積んだ $F$ に対し、
        \[
        \min_F \|AF-w^*\|_2^2+\lambda\|F\|_2^2
        \]
        を解いた後、各足を摩擦pyramidへ投影する。

        plant:
        \[
        \dot v=\frac{1}{m}\left(\sum_iF_i+F_{ext}\right)+g,\qquad
        \dot\omega=I^{-1}\left(\sum_i r_i\times F_i+M_{ext}\right).
        \]

        C++の完全なSQP/WBCを置き換えるものではないが、入力・制約・残差の意味は同じである。
        """
    ),
    code(
        """
# --- Block A: scenario contract ---
# 内容: 30条件で変える物理量を一つの辞書形式へ揃える。
# 意図: 難易度ごとに別コードを書かず、controllerの性能差だけを比較する。
BASE = dict(
    duration=4.0, dt=0.01, gait="stance", gait_hz=1.35,
    vx_ref=0.0, slope_deg=0.0, mu=0.40, mass_scale=1.0,
    noise_pos=0.0, noise_vel=0.0, delay_ms=0.0,
    force_x=0.0, force_y=0.0, moment_roll=0.0,
)

def scenario(name, level, **changes):
    cfg = BASE | changes
    return {"name": name, "level": level, **cfg}

SCENARIOS = [
    # 簡易: 四脚接地を中心に、単独の小変更だけを加える。
    scenario("E01_static_nominal", "easy"),
    scenario("E02_static_mu050", "easy", mu=0.50),
    scenario("E03_static_mu030", "easy", mu=0.30),
    scenario("E04_forward_010", "easy", vx_ref=0.10),
    scenario("E05_forward_020", "easy", vx_ref=0.20),
    scenario("E06_slope_plus3", "easy", slope_deg=3.0),
    scenario("E07_slope_minus3", "easy", slope_deg=-3.0),
    scenario("E08_mass_plus05", "easy", mass_scale=1.05),
    scenario("E09_noise_small", "easy", noise_pos=0.002, noise_vel=0.01),
    scenario("E10_push_5N", "easy", force_x=5.0),

    # 普通: trot、速度、斜面、noise、delayを現実的範囲で一つまたは二つ組み合わせる。
    scenario("N01_trot_static", "normal", gait="trot"),
    scenario("N02_trot_v020", "normal", gait="trot", vx_ref=0.20),
    scenario("N03_trot_v040", "normal", gait="trot", vx_ref=0.40),
    scenario("N04_trot_slope5", "normal", gait="trot", slope_deg=5.0),
    scenario("N05_trot_mu030", "normal", gait="trot", mu=0.30),
    scenario("N06_trot_mass115", "normal", gait="trot", mass_scale=1.15),
    scenario("N07_trot_noise", "normal", gait="trot", noise_pos=0.005, noise_vel=0.03),
    scenario("N08_trot_delay10", "normal", gait="trot", delay_ms=10.0),
    scenario("N09_trot_push15", "normal", gait="trot", force_y=15.0),
    scenario("N10_v040_slope5", "normal", gait="trot", vx_ref=0.40, slope_deg=5.0),

    # 高難度: 低摩擦・大速度・大外乱・大delayを組み合わせ、制約支配を観察する。
    scenario("H01_v080", "hard", gait="trot", vx_ref=0.80),
    scenario("H02_mu018", "hard", gait="trot", vx_ref=0.40, mu=0.18),
    scenario("H03_slope12", "hard", gait="trot", vx_ref=0.30, slope_deg=12.0),
    scenario("H04_mass140", "hard", gait="trot", mass_scale=1.40),
    scenario("H05_delay30", "hard", gait="trot", vx_ref=0.50, delay_ms=30.0),
    scenario("H06_noise_large", "hard", gait="trot", vx_ref=0.40, noise_pos=0.02, noise_vel=0.12),
    scenario("H07_push40", "hard", gait="trot", vx_ref=0.30, force_y=40.0),
    scenario("H08_roll12Nm", "hard", gait="trot", moment_roll=12.0),
    scenario("H09_combo_lowmu_slope", "hard", gait="trot", vx_ref=0.60, mu=0.20, slope_deg=10.0),
    scenario("H10_combo_all", "hard", gait="trot", vx_ref=0.70, mu=0.18, slope_deg=12.0,
             mass_scale=1.25, delay_ms=30.0, noise_pos=0.02, noise_vel=0.12,
             force_y=30.0, moment_roll=8.0),
]

assert len(SCENARIOS) == 30
pd.DataFrame(SCENARIOS).groupby("level").size()
        """
    ),
    code(
        """
# --- Block B: gait / contact schedule ---
# 数式: trotは対角2脚が半周期ごとに交代する。
# C++対応: GaitScheduleが返すcontactFlags(t)。NMPCがmodeを最適化するのではない。
LEG_NAMES = ("LF", "RF", "LH", "RH")
FOOT_POS = np.array([
    [ 0.25,  0.15, -0.30],
    [ 0.25, -0.15, -0.30],
    [-0.25,  0.15, -0.30],
    [-0.25, -0.15, -0.30],
])

def contact_flags(t, gait, gait_hz):
    if gait == "stance":
        return np.ones(4, dtype=bool)
    phase = (t * gait_hz) % 1.0
    return np.array([1, 0, 0, 1], dtype=bool) if phase < 0.5 else np.array([0, 1, 1, 0], dtype=bool)

# --- Block C: wrench matrix A ---
# 各足力Fiは合力へI3、CoM momentへskew(ri)で寄与する: [ΣFi; Σri×Fi] = A F。
def skew(r):
    x, y, z = r
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

def wrench_matrix(active):
    blocks_force = [np.eye(3) for _ in active]
    blocks_moment = [skew(FOOT_POS[i]) for i in active]
    return np.vstack([np.hstack(blocks_force), np.hstack(blocks_moment)])

print("all stance A:", wrench_matrix(np.arange(4)).shape)
print("trot A:", wrench_matrix(np.array([0, 3])).shape)
        """
    ),
    code(
        """
# --- Block D: force allocator + friction projection ---
# 数式: min ||AF-w||² + ridge||F||²。
# コメント: pseudoinverseでunconstrained解を得てから、各足をWBC型摩擦pyramidへ投影する。
# 注意: 上流NMPCのsoft円錐とqpOASESの厳密QPを再現するsolverではない。
def project_foot_force(f, mu, fz_max):
    fx, fy, fz = f
    fz = np.clip(fz, 0.0, fz_max)
    fx = np.clip(fx, -mu*fz, mu*fz)
    fy = np.clip(fy, -mu*fz, mu*fz)
    return np.array([fx, fy, fz])

def allocate_forces(wrench_des, contact, mu, mass):
    active = np.flatnonzero(contact)
    A = wrench_matrix(active)
    ridge = 1e-5
    # 正規方程式ではなくlstsqで解き、rank不足のtrotでも最小norm解を得る。
    A_aug = np.vstack([A, np.sqrt(ridge)*np.eye(3*len(active))])
    b_aug = np.r_[wrench_des, np.zeros(3*len(active))]
    f_active = np.linalg.lstsq(A_aug, b_aug, rcond=None)[0].reshape(-1, 3)
    fz_max = 3.0 * mass * 9.81 / max(len(active), 1)
    f_active = np.array([project_foot_force(f, mu, fz_max) for f in f_active])
    forces = np.zeros((4, 3))
    forces[active] = f_active
    realized = A @ f_active.ravel()
    residual = np.linalg.norm(realized - wrench_des) / max(np.linalg.norm(wrench_des), 1.0)
    margin = np.min(mu*f_active[:,2] - np.max(np.abs(f_active[:,:2]), axis=1))
    return forces, realized, residual, margin

# --- Block E: WBC torque proxy ---
# 数式: tau≈J^T F。A1の脚長に近いlever armで3関節torqueを近似する。
# 上流WBCは42変数QPなので、このproxyはtorque飽和傾向だけを見る。
J_PROXY = np.array([[0.08, 0.00, 0.00],
                    [0.00, 0.20, 0.10],
                    [0.00, 0.10, 0.20]])
TAU_LIMIT = 33.5

def torque_proxy(forces):
    tau = np.array([J_PROXY.T @ f for f in forces])
    saturation = np.mean(np.abs(tau) >= TAU_LIMIT)
    return np.clip(tau, -TAU_LIMIT, TAU_LIMIT), saturation
        """
    ),
    code(
        """
# --- Block F: closed-loop simulation ---
# 処理順: noisy state→100 Hz controller→force allocation→delay→plant integration。
# 出力: tracking、姿勢、高さ、制約、torque、計算時間をscenarioごとに集約する。
def run_scenario(cfg, seed=42):
    rng = np.random.default_rng(seed)
    dt = cfg["dt"]
    steps = int(cfg["duration"] / dt)
    mass = 12.5 * cfg["mass_scale"]
    inertia = np.array([0.24, 0.55, 0.65]) * cfg["mass_scale"]
    slope = np.deg2rad(cfg["slope_deg"])
    delay_steps = int(round(cfg["delay_ms"] / 1000 / dt))

    # state = [z, vx, vy, vz, roll, pitch, wx, wy]
    state = np.array([0.30, 0.0, 0.0, 0.0, 0.0, slope, 0.0, 0.0])
    force_queue = [np.zeros((4, 3)) for _ in range(delay_steps + 1)]
    command_forces = np.zeros((4, 3))
    controller_times, logs = [], []
    residuals, margins, tau_sats = [], [], []

    for k in range(steps):
        t = k * dt
        contact = contact_flags(t, cfg["gait"], cfg["gait_hz"])
        measured = state.copy()
        measured[0] += rng.normal(0, cfg["noise_pos"])
        measured[1:4] += rng.normal(0, cfg["noise_vel"], 3)

        # 上流と同じ100 Hzで計画を更新し、間のplant stepでは直前commandを使う。
        if k % max(1, int(round(0.01/dt))) == 0:
            tic = time.perf_counter()
            z, vx, vy, vz, roll, pitch, wx, wy = measured
            ax_des = 2.0 * (cfg["vx_ref"] - vx)
            ay_des = -2.0 * vy
            az_des = 35.0 * (0.30 - z) - 9.0 * vz
            mx_des = 18.0 * (0.0 - roll) - 3.0 * wx
            my_des = 18.0 * (slope - pitch) - 3.0 * wy
            wrench_des = np.array([
                mass*ax_des, mass*ay_des, mass*(9.81 + az_des),
                mx_des, my_des, 0.0,
            ])
            command_forces, realized, residual, margin = allocate_forces(
                wrench_des, contact, cfg["mu"], mass
            )
            _, tau_sat = torque_proxy(command_forces)
            controller_times.append(time.perf_counter() - tic)
            residuals.append(residual); margins.append(margin); tau_sats.append(tau_sat)

        force_queue.append(command_forces.copy())
        applied = force_queue.pop(0)
        # slope座標の縮約: 重力を斜面接線(-x)と法線(-z)へ分解する。
        # 完全な3D接触frame回転ではないが、slopeを無意味なラベルにしない。
        gravity_on_slope = np.array([-mass*9.81*np.sin(slope), 0.0, -mass*9.81*np.cos(slope)])
        net_force = applied.sum(axis=0) + gravity_on_slope + np.array([cfg["force_x"], cfg["force_y"], 0.0])
        net_moment = np.cross(FOOT_POS, applied).sum(axis=0) + np.array([cfg["moment_roll"], 0.0, 0.0])

        acc = net_force / mass
        alpha = net_moment[:2] / inertia[:2]
        state[1:4] += dt * acc
        state[0] += dt * state[3]
        state[6:8] += dt * alpha
        state[4:6] += dt * state[6:8]
        logs.append(state.copy())

    h = np.asarray(logs)
    z_rmse = float(np.sqrt(np.mean((h[:,0]-0.30)**2)))
    vx_rmse = float(np.sqrt(np.mean((h[:,1]-cfg["vx_ref"])**2)))
    angle_rmse = float(np.sqrt(np.mean(h[:,4]**2 + (h[:,5]-slope)**2)))
    fallen = bool(np.any(h[:,0] < 0.16) or np.any(np.abs(h[:,4:6]) > 0.75))
    mean_residual = float(np.mean(residuals))
    sat_rate = float(np.mean(tau_sats))
    solve_ms = np.asarray(controller_times) * 1e3
    score = np.clip(100 - 180*z_rmse - 35*vx_rmse - 55*angle_rmse
                    - 45*mean_residual - 100*sat_rate - 100*fallen, 0, 100)
    passed = (not fallen and z_rmse < 0.10 and vx_rmse < 0.40
              and angle_rmse < 0.30 and mean_residual < 0.55 and sat_rate < 0.25)
    return {
        "name": cfg["name"], "level": cfg["level"], "pass": bool(passed),
        "score": float(score), "z_rmse_m": z_rmse, "vx_rmse_mps": vx_rmse,
        "angle_rmse_rad": angle_rmse, "wrench_residual": mean_residual,
        "min_friction_margin_N": float(np.min(margins)),
        "torque_sat_rate": sat_rate, "fallen": fallen,
        "controller_mean_ms": float(np.mean(solve_ms)),
        "controller_p95_ms": float(np.percentile(solve_ms, 95)),
        "realtime_margin_at_100Hz": float(10.0 / max(np.percentile(solve_ms,95), 1e-9)),
    }
        """
    ),
    code(
        """
# --- Block G: 30 scenariosを実行して保存 ---
# 意図: 同じseed・同じmetricで難易度間を比較し、都合の良いrunだけを選ばない。
started = time.perf_counter()
results = [run_scenario(cfg, seed=1000+i) for i, cfg in enumerate(SCENARIOS)]
elapsed = time.perf_counter() - started
df = pd.DataFrame(results)

csv_path = OUT / "legged_model_benchmark_30.csv"
json_path = OUT / "legged_model_benchmark_30.json"
df.to_csv(csv_path, index=False)
json_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")

print(f"30 scenarios wall time: {elapsed:.3f} s")
display(df)
display(df.groupby("level").agg(
    scenarios=("name","count"),
    passed=("pass","sum"),
    mean_score=("score","mean"),
    mean_vx_rmse=("vx_rmse_mps","mean"),
    mean_wrench_residual=("wrench_residual","mean"),
    p95_controller_ms=("controller_p95_ms","max"),
))
        """
    ),
    code(
        """
# --- Block H: 性能図 ---
# 上: score。下: 主要制約metric。色は難易度、×はfail。
colors = {"easy":"tab:green", "normal":"tab:blue", "hard":"tab:red"}
fig, axes = plt.subplots(2, 1, figsize=(14, 9), constrained_layout=True)
x = np.arange(len(df))
bar_colors = [colors[v] for v in df["level"]]
axes[0].bar(x, df["score"], color=bar_colors, alpha=0.85)
axes[0].scatter(x[~df["pass"]], df.loc[~df["pass"],"score"], marker="x", s=80, color="black", label="fail")
axes[0].set_ylabel("model-level score [0-100]")
axes[0].set_title("30 scenario model benchmark — not end-to-end ROS/Gazebo performance")
axes[0].legend()

axes[1].plot(x, df["vx_rmse_mps"], "o-", label="vx RMSE [m/s]")
axes[1].plot(x, df["wrench_residual"], "s-", label="normalized wrench residual")
axes[1].plot(x, df["torque_sat_rate"], "^-", label="torque saturation rate")
axes[1].set_ylabel("metric")
axes[1].legend(ncol=3)
axes[1].set_xticks(x, df["name"], rotation=75, ha="right")
fig_path = OUT / "legged_model_benchmark_30.png"
fig.savefig(fig_path, dpi=150)
plt.show()
print(fig_path)
        """
    ),
    md(
        """
        ## 結果の分析方法
        次の順で失敗原因を読む。

        1. `fallen`: 縮約plantが高さ/姿勢限界を超えた。
        2. `wrench_residual`: 接触幾何と摩擦で要求wrenchを作れない。
        3. `min_friction_margin_N`: 0付近は摩擦境界がactive。
        4. `torque_sat_rate`: forceは作れても関節torque proxyで飽和。
        5. `controller_p95_ms`: 100 Hzの10 ms deadlineに対する計算余裕。

        ## この先の本当のrepository benchmark
        同じ30 scenario schemaを次のROS/Gazebo項目へ移植する。

        - `task.info`, `reference.info`, `gait.info` をscenarioごとに生成
        - `/cmd_vel` とgait commandをtimestamp付きpublish
        - `/legged_robot_mpc_observation`, odometry, joint state, WBC/solver統計をrosbag保存
        - 同じRMSE、転倒、constraint、torque、deadline指標を計算
        - A1 URDF、Gazebo contact、OCS2/qpOASESを含む結果だけを上流end-to-end性能と呼ぶ

        現環境ではROS/OCS2が無いため、そこを実行済みと偽らない。
        実際のA1 MuJoCo plant、20秒以上のGIF、物理閾値を使う実行証拠は
        `14_a1_mujoco_benchmark_30_scenarios.ipynb` へ進む。
        """
    ),
]


CHAPTER_ENRICHMENTS = {
    "01_packages_and_loop.ipynb": {
        "background": "ROS callback、100 Hz solver thread、500 Hz hardware loopが別々に進むため、周期と所有dataを知らずに局所コードだけ読むとraceと遅延を見落とす。",
        "purpose": "初期化からmotor commandまでをthread境界付きで追い、各周期の責務を変更前に説明できるようにする。",
        "conclusion": "`LeggedController::update` は500 Hzの配線、`MPC_MRT_Interface::advanceMpc` は100 Hzの計画更新である。両者を一つの同期loopとは扱わない。",
        "flow": "LeggedHWLoop::update(500 Hz) -> estimate -> evaluate latest policy -> WBC -> write\n                         ^                 |\n                         | shared observation/policy\n                 advanceMpc(100 Hz) <-----+",
        "mapping": """// external/legged_control/legged_controllers/src/LeggedController.cpp
setupMpc();                 // SqpMpcを構築: OCP policy生成器
setupMrt();                 // advanceMpc() threadを開始: 100 Hz
updateStateEstimation(...); // sensor -> x_k
setCurrentObservation(x_k); // 数式の初期条件 x(0)=x_k
evaluatePolicy(t, x_k,...); // (x*(t),u*(t),mode(t))を現在時刻で評価
wbc_->update(...);          // 現在点を42変数QPへ渡す
// external/legged_control/legged_hw/src/LeggedHWLoop.cpp
controllerManager_->update(...); // read -> controller -> write の500 Hz境界""",
    },
    "02_state_input_frames.ipynb": {
        "background": "同じ長さのvectorでも、順序・frame・単位が違えば物理的には別の型である。",
        "purpose": "24状態、24入力、36 rigid-body stateのsliceを、変換symbolと数式に結び付ける。",
        "conclusion": "`x[:6]` は正規化centroidal momentum、`u[:12]` はworld contact force、`u[12:]` はjoint velocityであり、torqueはWBC後まで現れない。",
        "flow": "rbdState(36: ZYX,p,q,omega,v,dq) -> CentroidalModelRbdConversions\n                                            -> x(24: h/m,pose,q)\nu(24: four world forces,dq*) -----------------> centroidal dynamics",
        "mapping": """// external/legged_control/legged_estimation/src/StateEstimateBase.cpp
rbdState.segment<3>(0) = zyx;       // [yaw,pitch,roll]
rbdState.segment<3>(3) = position;  // p_b^W
rbdState.segment<12>(6) = q;        // q_j
rbdState.segment<3>(18) = omegaW;   // omega_b^W
rbdState.segment<3>(21) = vW;       // v_b^W
rbdState.segment<12>(24) = dq;      // dq_j
// ocs2 centroidal helper contract from config/a1/task.info
x = [h_linear/m, h_angular/m, p_b, ZYX, q_j];
u = [f_LF^W,f_RF^W,f_LH^W,f_RH^W,dq_j];""",
    },
    "03_command_reference_gait.ipynb": {
        "background": "速度目標と接地modeは異なる意思決定であり、一緒に自動生成されるとは限らない。",
        "purpose": "`cmd_vel`から2点TargetTrajectoriesを作る式と、独立GaitSchedule経路を追う。",
        "conclusion": "参照publisherは1秒先のposeを作るがgaitを選ばない。GaitReceiverが別topicからModeScheduleを更新する。",
        "flow": "/cmd_vel -> cmdVelToTargetTrajectories -> [t0,t1],[x0,x1]\ngait name -> GaitReceiver -> ModeSchedule ------------------+-> OCP reference manager",
        "mapping": """// external/legged_control/legged_controllers/src/TargetTrajectoriesPublisher.cpp
cmdVelRot = R_zyx * cmdVel.head(3);       // v_W = R_WB v_B
target.x += cmdVelRot.x() * T;            // x^+ = x + v_Wx T
target.y += cmdVelRot.y() * T;            // y^+ = y + v_Wy T
target.yaw += cmdVel(3) * T;              // psi^+ = psi + yawRate T
stateTrajectory[0].head(3)=cmdVelRot;     // h_linear/m reference
stateTrajectory[1].head(3)=cmdVelRot;
// GaitReceiver: gait command -> GaitSchedule::insertModeSequenceTemplate(...)""",
    },
    "04_state_estimation.ipynb": {
        "background": "free-floating base位置はencoderで直接測れず、IMU積分だけではbiasがdriftになる。",
        "purpose": "接地足の相対運動学を観測に使う18状態linear KFを予測・観測・noise切替の順で読む。",
        "conclusion": "KFが推定するのはbase並進と4足world位置である。姿勢はIMU、関節状態はencoder由来で、遊脚ではworld固定仮定をnoiseで弱める。",
        "flow": "IMU q,a -> rotate+gravity -> KF predict [p,v,pf]\nq,dq -> Pinocchio foot kinematics -> relative observation -> KF update\ncontact ---------------------------> Q/R scaling",
        "mapping": """// external/legged_control/legged_estimation/src/LinearKalmanFilter.cpp
a_.block<3,3>(0,3)=dt*I;       // p^+=p+dt*v
b_.block<3,3>(0,0)=dt^2/2*I;  // +0.5*dt^2*a_W
b_.block<3,3>(3,0)=dt*I;      // v^+=v+dt*a_W
xhat_=a_*xhat_+b_*accel;      // x^- = A x + B a
S=C_*P_*C_.transpose()+R;      // innovation covariance
xhat_ += K*(y-C_*xhat_);      // x^+ = x^- + K residual
// swing foot: process/measurement noiseを大きくしp_f^W固定仮定を弱める""",
    },
    "05_centroidal_dynamics.ipynb": {
        "background": "NMPCは18自由度の全運動をそのまま積分せず、全身の運動量と形状へ縮約して未来を予測する。",
        "purpose": "接触力からlinear/angular momentum rateが生じる式と、A1既定full-centroidal選択を確認する。",
        "conclusion": "合力が並進、CoMまわりの合momentが角運動量を変える。A1既定はSRBDではなくfull centroidalである。",
        "flow": "x=[h/m,base pose,q], u=[four forces,dq]\n        -> LeggedRobotDynamicsAD / centroidal model\n        -> xdot=[sum(F)/m+g, sum(r x F)/m, pose rate, dq]",
        "mapping": """// external/legged_control/legged_interface/src/LeggedInterface.cpp
centroidalModelInfo = createCentroidalModelInfo(...); // task.info modelType=0
dynamicsPtr.reset(new LeggedRobotDynamicsAD(...));     // xdot=f(x,u)
// faithful equation map (OCS2内部成分実装はこのworkspaceで未照合)
hDot_linear = sum_i(f_i) + m*g;       // m*vdot = Σf + mg
hDot_angular = sum_i(r_i.cross(f_i)); // Ldot = Σ(r_i×f_i)
qDot_joint = u.tail(12);              // 入力後半は関節速度""",
    },
    "06_nmpc_ocp_and_tuning.ipynb": {
        "background": "良い瞬間入力だけでは接地切替後を準備できないため、有限horizonで状態・入力列を同時評価する。",
        "purpose": "OCPのcost、dynamics、constraint、SQP設定をsetup symbolへ対応付け、Q/R変更の因果を読めるようにする。",
        "conclusion": "Q/Rは単なるgainではなく、制約下で有限な資源をどの誤差へ配るかを決める。adapterの瞬時plannerはこのSQP horizonを持たない。",
        "flow": "x0 + target + ModeSchedule -> setupOptimalControlProblem\n -> cost(Q,R)+dynamics+constraints -> SqpMpc(horizon 1 s)\n -> policy {x*(t),u*(t),mode(t)}",
        "mapping": """// external/legged_control/legged_interface/src/LeggedInterface.cpp
problem.dynamicsPtr = LeggedRobotDynamicsAD(...);         // xdot=f(x,u)
problem.costPtr->add("quadraticTracking", trackingCost);   // ||x-xr||_Q^2+||u-uwc||_R^2
problem.equalityConstraintPtr->add(...);                   // contact-mode等式
problem.inequalityConstraintPtr->add(...);                 // friction/collision
// external/legged_control/legged_controllers/src/LeggedController.cpp
mpcPtr_.reset(new SqpMpc(mpcSettings,sqpSettings,problem)); // receding-horizon SQP
// adapter limitation: src/legged_control_mujoco はSqpMpcを呼ばない""",
    },
    "07_contact_constraints_and_swing.ipynb": {
        "background": "同じ足でも立脚中は地面拘束、遊脚中は力ゼロと軌道追従へ役割が切り替わる。",
        "purpose": "ModeScheduleから各足の等式・摩擦不等式・swing targetがactivateされる経路を読む。",
        "conclusion": "NMPCのsoft円錐とWBCのhard pyramidは異なる集合である。地形法線setter未実装の上流既定を知覚歩行と誤認しない。",
        "flow": "ModeSchedule -> contact flag -> stance: foot velocity=0, friction\n                            \\-> swing: force=0, spline velocity/height\nterrain height(既定0) -> SwingTrajectoryPlanner",
        "mapping": """// external/legged_control/legged_interface constraint activation
isActive(t) = !contactFlag[i]; // ZeroForceConstraint: swingなら f_i=0
isActive(t) =  contactFlag[i]; // ZeroVelocityConstraint: stanceなら v_foot=0
// FrictionConeConstraint
h = mu*(Fz+Fg)-sqrt(Fx*Fx+Fy*Fy+eps); // h>=0
// SwingTrajectoryPlanner.cpp
swingHeight = 0.08;                   // task.info
z(t)=spline(position, liftOffVelocity, touchDownVelocity); // 遊脚法線軌道""",
    },
    "08_weighted_wbc.ipynb": {
        "background": "NMPCの縮約入力だけでは12 motor torqueを直接得られず、全身運動方程式と接触を現在瞬間で満たす必要がある。",
        "purpose": "42変数、hard constraints、soft tasks、qpOASES入力行列を式とC++ symbolへ一対一対応させる。",
        "conclusion": "既定は単一WeightedWbc QPであり、GRF追従はweight 0.01のsoft task。解の末尾12 torqueだけがmotorへ進む。",
        "flow": "x*,u*,rbd,mode -> WbcBase task matrices\n -> hard: EoM/torque/contact/friction + soft: swing/base/force\n -> WeightedWbc::update -> z=[qdd,F,tau] -> tau",
        "mapping": """// external/legged_control/legged_wbc/src/WbcBase.cpp
a << data.M, -j_.transpose(), -s.transpose(); // [M,-J^T,-S^T]
b = -data.nle;                               // A z = -nle
// external/legged_control/legged_wbc/src/WeightedWbc.cpp
weightedTask = swing*100.0 + baseAccel*1.0 + contactForce*0.01;
H = Asoft.transpose()*Asoft;                 // min 1/2 z^T H z + g^Tz
g = -Asoft.transpose()*bsoft;
qp.init(H,g,Ahard,lb,ub,lbA,ubA,nWsr);       // hard bounds remain constraints""",
    },
    "09_hybrid_joint_hardware.ipynb": {
        "background": "WBC torqueは最後に位置・速度feedbackと合成され、simulationと実機のhardware interfaceへ渡る。",
        "purpose": "setCommandの5値と実際のtorque式、limit/protection、Gazebo/Unitree分岐を追う。",
        "conclusion": "既定Kp=0,Kd=3なのでposition errorは直接torqueへ入らない。adapterも同式と33.5 N m clipを使うがUnitree通信ではない。",
        "flow": "tau_WBC,q*,dq* -> HybridJointHandle::setCommand\n -> tau_ff+Kp(q*-q)+Kd(dq*-dq) -> limit/protection\n -> Gazebo effort OR Unitree LowCmd OR project MuJoCo ctrl",
        "mapping": """// external/legged_control/legged_controllers/src/LeggedController.cpp
handle.setCommand(qDes,dqDes,0.0,3.0,tauWbc); // [q*,dq*,Kp,Kd,ff]
// external/legged_control/legged_gazebo/src/LeggedHWSim.cpp
tau = ff + kp*(qDes-q) + kd*(dqDes-dq);       // hybrid equation
// external/legged_control/legged_unitree_hw/src/UnitreeHW.cpp
lowCmd.motorCmd[i] = command;                 // 実機transport/protection境界
// src/legged_control_mujoco/adapter.py::hybrid_command
return clip(tau, -33.5, 33.5);                // project adapter境界""",
    },
    "10_multirate_integration.ipynb": {
        "background": "solver deadlineを満たしても、古いpolicyを実行すれば閉ループ性能は崩れる。",
        "purpose": "observation共有、policy swap、時刻評価、WBC実行を100/500 Hz間の時系列として理解する。",
        "conclusion": "500 Hz側は最新policyを毎回現在時刻で評価する。平均solver時間だけでなくpolicy ageとworst-case deadlineを測る。",
        "flow": "100 Hz: observation -> advanceMpc -> atomic policy\n                                  |\n500 Hz: estimate -> updatePolicy -> evaluatePolicy(now) -> WBC -> motor",
        "mapping": """// external/legged_control/legged_controllers/src/LeggedController.cpp
mpcMrtInterface_->setCurrentObservation(obs); // x_kをsolver側へpublish
mpcMrtInterface_->updatePolicy();             // 完成policyをconsumerへswap
evaluatePolicy(time,x,xOpt,uOpt,mode);         // π(t,x)の現在点
wbc_->update(xOpt,uOpt,rbd,mode,period);       // 2 ms周期の瞬間QP
// setupMrt thread
while (...) { mpcMrtInterface_->advanceMpc(); rate.sleep(); } // 100 Hz""",
    },
    "11_tuning_and_equation_changes.ipynb": {
        "background": "複数層のparameterを同時変更すると、改善原因も副作用も同定できない。",
        "purpose": "症状を残差へ戻し、config変更と式変更を回帰可能な実験として設計する。",
        "conclusion": "baseline固定、1仮説、物理残差、solver/time、安全指標を揃えて初めて調整になる。式変更はNMPCとWBC双方の整合まで含む。",
        "flow": "symptom -> identify block/residual -> freeze baseline -> one change\n -> unit/shape/gradient tests -> 30-scenario evidence -> accept or revert",
        "mapping": """// external/legged_control/legged_controllers/config/a1/task.info
Q(state)=...; R(input)=...; mu=0.3; // parameter変更: cost/feasible set
// LeggedInterface::setupOptimalControlProblem
constraint = FrictionConeConstraint(...);     // NMPC式を変更する場所
// WbcBase::formulateFrictionConeTask
D_i * f_i <= 0;                               // WBC近似も同時に整合
// 検査式
old_margin=mu*Fz-hypot(Fx,Fy);                // baseline残差
new_margin=Fz-sqrt((Fx/mux)^2+(Fy/muy)^2);    // 新式と単位を明示""",
    },
}

for chapter_name, enrichment in CHAPTER_ENRICHMENTS.items():
    NOTEBOOKS[chapter_name].extend(
        [
            md(
                f"""
                ## 章固有の背景
                {enrichment["background"]}

                ## 章固有の目的
                {enrichment["purpose"]}

                ## この章のASCIIデータフロー
                ```text
                {enrichment["flow"]}
                ```

                ## 上流C++ / faithful pseudocode と数式の行対応
                ```cpp
                {enrichment["mapping"]}
                ```

                **事実のラベル**: `external/legged_control/` の記述はcommit
                `{UPSTREAM_COMMIT}` の上流実装事実。数式展開はそのinterfaceを説明する理論。
                `src/legged_control_mujoco/` に言及した行はproject所有adapterの実装であり、
                ROS1/OCS2 SQP原実装とは同一ではない。

                ## 章固有の結論
                {enrichment["conclusion"]}
                """
            )
        ]
    )


A1_SCENARIOS = (
    ("easy", "E01_stance_baseline"), ("easy", "E02_stance_low"),
    ("easy", "E03_stance_high"), ("easy", "E04_walk_005"),
    ("easy", "E05_walk_008"), ("easy", "E06_walk_lateral"),
    ("easy", "E07_stance_payload"), ("easy", "E08_stance_gentle_push"),
    ("easy", "E09_walk_turn"), ("easy", "E10_walk_012"),
    ("normal", "N01_walk_016"), ("normal", "N02_walk_diagonal"),
    ("normal", "N03_walk_turn"), ("normal", "N04_dynamic_walk"),
    ("normal", "N05_standing_trot"), ("normal", "N06_trot"),
    ("normal", "N07_walk_payload"), ("normal", "N08_walk_push"),
    ("normal", "N09_walk_mu045"), ("normal", "N10_walk_low_turn_push"),
    ("hard", "H01_walk_025"), ("hard", "H02_walk_strafe"),
    ("hard", "H03_walk_fast_turn"), ("hard", "H04_trot_fast"),
    ("hard", "H05_flying_trot"), ("hard", "H06_pace"),
    ("hard", "H07_low_friction"), ("hard", "H08_heavy_payload"),
    ("hard", "H09_strong_push"), ("hard", "H10_compound"),
)


def measured_benchmark_markdown() -> str:
    result_path = HERE / "assets" / "scenarios" / "scenario_results.json"
    if not result_path.is_file():
        return (
            "## 生成時の測定結果\n"
            "**PENDING**: `scenario_results.json` はまだ無い。benchmark完了後にgeneratorを再実行するか、"
            "下のcode cellを実行して結果を読み込む。未実行をpassとして扱わない。"
        )
    try:
        payload = json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return f"## 生成時の測定結果\n**PENDING**: 集約JSONを安全に読めなかった: `{type(exc).__name__}`"
    records = payload.get("results", [])
    by_name = {item.get("config", {}).get("name"): item for item in records}
    lines = [
        "## 生成時の測定結果",
        f"集約済み `{len(records)}/30`、pass `{payload.get('passed_count', 0)}`、"
        f"fail `{payload.get('failed_count', 0)}`。閾値判定はbenchmark scriptの保存値を表示する。",
        "",
    ]
    for level, name in A1_SCENARIOS:
        record = by_name.get(name)
        if record is None:
            lines.append(f"- `{name}` ({level}): **PENDING**")
            continue
        metrics = record.get("metrics", {})
        status = "PASS" if metrics.get("passed") else "FAIL"
        reasons = ", ".join(metrics.get("failure_reasons", [])) or "all thresholds met"
        sim_s = metrics.get("simulated_duration_s")
        gif_s = metrics.get("gif_playback_duration_s")
        sim_text = f"{sim_s:.3f}" if isinstance(sim_s, (int, float)) else "n/a"
        gif_text = f"{gif_s:.3f}" if isinstance(gif_s, (int, float)) else "n/a"
        lines.append(
            f"- `{name}` ({level}): **{status}**; sim "
            f"{sim_text} s; GIF {gif_text} s; {reasons}"
        )
    return "\n".join(lines)


def scenario_gallery_cells() -> list[dict]:
    cells: list[dict] = []
    labels = {"easy": "Easy 10", "normal": "Normal 10", "hard": "Hard 10"}
    for level in ("easy", "normal", "hard"):
        cells.append(md(f"## {labels[level]}"))
        for _, name in (item for item in A1_SCENARIOS if item[0] == level):
            cells.append(
                md(
                    f"""
                    ### `{name}`
                    固有scenario定義・command・外乱・seedは
                    `scripts/run_legged_control_benchmark.py::SCENARIOS` を正本とする。

                    ![{name}](assets/scenarios/gifs/{name}.gif)
                    """
                )
            )
    return cells


NOTEBOOKS["14_a1_mujoco_benchmark_30_scenarios.ipynb"] = [
    md(
        f"""
        # 14 — A1 MuJoCo adapter benchmark: 10 easy + 10 normal + 10 hard

        ## 背景
        Notebook 13は4秒のequation-level proxyで、robot modelを動かす証拠ではない。
        最終評価ではUnitree A1 MuJoCo plantを各scenario 20秒以上、転倒後も途中resetせず実行し、
        定量metricとGIF playback時間を保存する。

        ## 目的
        1. `src/legged_control_mujoco` のA1 adapterを30条件で実行した証拠を読む。
        2. easy/normal/hardが各10件、simulation/GIFが各20秒以上か機械検証する。
        3. pass/failだけでなく、失敗理由と物理metricを追って調整箇所へ戻る。

        ## 厳密な実装境界
        - 上流正本: `external/legged_control/` commit `{UPSTREAM_COMMIT}`。ROS1、OCS2 SQP-NMPC、
          Pinocchio/qpOASES WBC、Gazebo/Unitree I/Oの原実装。
        - 実行対象: project所有 `src/legged_control_mujoco/adapter.py` と `models/a1.xml`。
        - adapterはgait template、24D state/input contract、WBC task構造、hybrid torque式を対応させるが、
          **OCS2 SQPではない**。有限horizon policyを、瞬時friction-constrained force plannerと
          MuJoCo acceleration-level inverse dynamicsで置換する。
        - したがって結果は「A1 MuJoCo adapter性能」であり、上流ROS1/OCS2 repository性能ではない。
        - この経路は **Quadruped-PyMPCを一切使用しない**。

        ## 結論
        保存されたmetricとGIFが揃ったscenarioだけを実行済みとみなす。閾値passはadapterについての
        再現可能な判定であり、OCS2 SQPや実機A1の性能主張へ外挿しない。
        """
    ),
    code(
        COMMON
        + """
import csv
import json
from PIL import Image

SCENARIO_ROOT = ROOT / "notebook_legged" / "assets" / "scenarios"
JSON_PATH = SCENARIO_ROOT / "scenario_results.json"
CSV_PATH = SCENARIO_ROOT / "scenario_results.csv"
"""
    ),
    md(
        r"""
        ## ASCIIデータフロー
        ```text
        scenario(name,gait,command,friction,payload,push,seed)
          -> A1 MuJoCo model (q,v,contacts,M,b,J)
          -> instantaneous force planner
               min ||W(J^T f - (M qdd* + b))||² + lambda||f-f_nom||²
               fz>=0, |fx|<=mu*fz, |fy|<=mu*fz
          -> acceleration WBC
               stance/swing Cartesian qdd* + posture regularization
               M qdd + b - J^T f - S^T tau = 0
          -> hybrid torque
               tau_cmd=clip(tau+0(q*-q)+3(dq*-dq), +/-33.5)
          -> MuJoCo plant -- q,v,measured contact --> next control sample
          -> metrics + 20 s GIF -> JSON/CSV/gallery
        ```

        ## source / symbol / equation mapping
        ```cpp
        // upstream: external/legged_control/legged_controllers/config/a1/gait.info
        ModeSchedule::from_gait             // adapter.py: GAIT_TEMPLATES, mode_phase
        // upstream: centroidal input first 12 entries
        A1HeadlessAdapter::_optimize_contact_forces
          demand=(M*qdd+b)[0:6]             // floating-base wrench equation
          minimize ||W(J^T f-demand)||^2    // instantaneous; NOT OCS2 SQP
        // upstream: WbcBase::formulateNoContactMotionTask/formulateSwingLegTask
        A1HeadlessAdapter::_desired_qacc
          J*qdd = a_foot^* - Jdot*qdot      // stance/swing acceleration task
        // upstream: WbcBase.cpp floating-base EoM and torque extraction
        A1HeadlessAdapter::solve_wbc
          tau=(M*qdd+b-J^T*f)[actuated]      // then +/-33.5 N m
        // upstream: LeggedController.cpp setCommand(...,0,3,tau)
        adapter.py::hybrid_command           // ff + Kp error + Kd error
        // project runner
        scripts/run_legged_control_benchmark.py::run_scenario/write_aggregates
        ```
        """
    ),
    md(
        """
        ## 判定閾値
        benchmark scriptの `Thresholds` が正本:

        - simulation duration ≥ 20 s、GIF playback ≥ 20 s
        - fallなし、minimum base height ≥ 0.18 m
        - height RMSE ≤ 0.10 m、max |roll/pitch| ≤ 0.60 rad
        - planar velocity RMSE ≤ 0.35 m/s、yaw-rate RMSE ≤ 0.60 rad/s
        - max torque ≤ 33.5 N m、saturation fraction ≤ 0.10
        - post-saturation dynamics residual ≤ 5.0
        - planned/measured contact agreement ≥ 0.55

        転倒検出自体はheight < 0.18 mまたは|roll/pitch| > 0.9 rad。判定姿勢閾値0.60 radの方が厳しい。
        """
    ),
    code(
        f"""
# --- Block 1: JSON/CSVを同時にloadし、30/10/10と名前集合を検証 ---
EXPECTED = {list(A1_SCENARIOS)!r}
expected_names = [name for _, name in EXPECTED]
expected_levels = {{level: sum(item[0] == level for item in EXPECTED)
                   for level in ("easy", "normal", "hard")}}
assert len(EXPECTED) == 30 and expected_levels == {{"easy": 10, "normal": 10, "hard": 10}}

if not JSON_PATH.is_file() or not CSV_PATH.is_file():
    print("PENDING: run uv run python scripts/run_legged_control_benchmark.py --all")
    records, csv_rows = [], []
else:
    aggregate = json.loads(JSON_PATH.read_text(encoding="utf-8"))
    records = aggregate["results"]
    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        csv_rows = list(csv.DictReader(handle))
    json_names = [record["config"]["name"] for record in records]
    csv_names = [row["name"] for row in csv_rows]
    assert len(records) == len(csv_rows) == 30
    assert set(json_names) == set(csv_names) == set(expected_names)
    counts = {{level: sum(r["config"]["difficulty"] == level for r in records)
              for level in ("easy", "normal", "hard")}}
    assert counts == {{"easy": 10, "normal": 10, "hard": 10}}
    print("validated scenario counts:", counts)
"""
    ),
    code(
        """
# --- Block 2: Pillowで全30 GIFのframe timingをdecodeして20秒以上を検証 ---
def gif_playback_seconds(path):
    total_ms = 0
    with Image.open(path) as image:
        frame_count = image.n_frames
        for index in range(frame_count):
            image.seek(index)
            total_ms += int(image.info.get("duration", 0))
    return frame_count, total_ms / 1000.0

gif_checks = []
for level, name in EXPECTED:
    path = SCENARIO_ROOT / "gifs" / f"{name}.gif"
    assert path.is_file(), f"missing GIF: {path}"
    frames, playback_s = gif_playback_seconds(path)
    assert frames > 0, f"empty GIF: {name}"
    assert playback_s + 1e-9 >= 20.0, f"{name}: GIF playback {playback_s:.3f} s < 20 s"
    gif_checks.append((level, name, frames, playback_s))
print(f"validated {len(gif_checks)} GIFs; minimum playback:",
      min(item[3] for item in gif_checks), "s")
"""
    ),
    code(
        """
# --- Block 3: 保存metricを再表示し、失敗理由を隠さない ---
if records:
    summary = []
    for record in records:
        cfg, metric = record["config"], record["metrics"]
        summary.append({
            "name": cfg["name"], "difficulty": cfg["difficulty"],
            "passed": metric["passed"],
            "sim_s": metric["simulated_duration_s"],
            "gif_s": metric["gif_playback_duration_s"],
            "height_rmse_m": metric["height_error_rmse_m"],
            "velocity_rmse_mps": metric["velocity_tracking_rmse"]["planar_mps"],
            "max_rp_rad": metric["maximum_abs_roll_pitch_rad"],
            "max_tau_nm": metric["maximum_abs_torque_nm"],
            "dyn_residual": metric["maximum_dynamics_residual"],
            "contact_agreement": metric["planned_vs_measured_contact_agreement"],
            "failure_reasons": "; ".join(metric["failure_reasons"]) or "none",
        })
    import pandas as pd
    result_df = pd.DataFrame(summary).sort_values("name")
    display(result_df)
    display(result_df.groupby("difficulty").agg(
        scenarios=("name", "count"), passed=("passed", "sum"),
        mean_velocity_rmse=("velocity_rmse_mps", "mean"),
        worst_dynamics_residual=("dyn_residual", "max"),
    ))
else:
    print("PENDING: aggregate files are not complete yet")
"""
    ),
    md(measured_benchmark_markdown()),
] + scenario_gallery_cells()


README = f"""# legged_control 理論・コード学習Notebook

大学院の初心者が `qiayuanliao/legged_control` を、完成したROSシステムとして眺めるのではなく、
数式・データ契約・小さな数値実験・C++対応箇所へ分解して学ぶ教材です。

照合した上流: commit `{UPSTREAM_COMMIT}`（2025-02-13、master）

## 特徴

- `00` から `14` まで順番に読む
- NumPy / SciPy / Matplotlibだけで理論実験を再実行可能
- ROS / OCS2 / Gazebo / Unitreeが必要な実装事実と、教育用縮約実験を明確に区別
- 背景・目的・結論、ASCIIデータフロー、数式、コメント付きblock codeを接続
- `13` は4秒のequation-level proxy、`14` はA1 MuJoCo adapterの20秒以上×30 scenario
- 各章末にチューニング・変更時の観測項目を記載
- 詳細な実装監査は `../docs/legged_control/` を正本として参照

## 起動

```bash
uv sync --extra workshop
uv run jupyter lab notebook_legged/
```

最終benchmarkの厳密な再現command:

```bash
uv run python scripts/run_legged_control_benchmark.py --all
```

30本の20秒以上GIFに加えてJSON/CSV/summaryを保存するため、`notebook_legged/assets/scenarios/`
には数百MB規模の空き容量を見込む。既存の有効なscenarioは既定で再利用され、`--overwrite` を
明示しない限り一致する出力を置換しない。

## 章

| No. | Notebook | 到達点 |
|---:|---|---|
| 00 | `00_learning_map.ipynb` | 閉ループ全体と責務境界 |
| 01 | `01_packages_and_loop.ipynb` | package・thread・周期 |
| 02 | `02_state_input_frames.ipynb` | shape・単位・frame |
| 03 | `03_command_reference_gait.ipynb` | 2点参照と独立Gait |
| 04 | `04_state_estimation.ipynb` | 接地足を使う並進KF |
| 05 | `05_centroidal_dynamics.ipynb` | 合力・moment・centroidal |
| 06 | `06_nmpc_ocp_and_tuning.ipynb` | OCP・Q/R・horizon |
| 07 | `07_contact_constraints_and_swing.ipynb` | 接触・摩擦・遊脚 |
| 08 | `08_weighted_wbc.ipynb` | 42変数の瞬間QP |
| 09 | `09_hybrid_joint_hardware.ipynb` | torque FF + low-gain PD |
| 10 | `10_multirate_integration.ipynb` | policyと100/500 Hz統合 |
| 11 | `11_tuning_and_equation_changes.ipynb` | 再現可能な調整・式変更 |
| 12 | `12_repository_code_walkthrough.ipynb` | 実C++の端から端までのcall graph |
| 13 | `13_model_benchmark_30_scenarios.ipynb` | 4秒のequation-level proxy benchmark |
| 14 | `14_a1_mujoco_benchmark_30_scenarios.ipynb` | A1 MuJoCo adapterの30条件・20秒GIF・物理metric |

## 事実の境界

上流C++は `external/legged_control/` に上記commitでclone済みですが、gitignore対象です。
Notebookの実装説明は主要C++経路と `docs/legged_control/` の照合結果に基づきます。
OCS2本体はworkspaceに無いため、`LeggedRobotDynamicsAD` の完全な成分式など、
未照合の箇所は断定していません。

上流commitはROS1/OCS2原実装です。project所有の `src/legged_control_mujoco/` は
gait/state/input/WBC/hybrid-command契約をMuJoCoへ接続するadapterですが、
**OCS2 SQPではありません**。瞬時force plannerとacceleration-level WBCへ置換した実行境界であり、
Notebook 14の結果は上流ROS1/OCS2や実機A1の性能主張ではありません。
このcurriculumとbenchmarkはQuadruped-PyMPCを使用しません。

`build_notebooks.py` は教材の再生成用です。Notebookを直接編集した後に実行すると上書きするため、
生成元を更新してから実行してください。
"""


def main() -> None:
    HERE.mkdir(parents=True, exist_ok=True)
    (HERE / "README.md").write_text(README, encoding="utf-8")
    for name, cells in NOTEBOOKS.items():
        path = HERE / name
        path.write_text(
            json.dumps(notebook(cells), ensure_ascii=False, indent=1) + "\n",
            encoding="utf-8",
        )
        print(path.relative_to(HERE.parent))


if __name__ == "__main__":
    main()
