#!/usr/bin/env python3
"""Quad-SDK Step 01: 基準ログの記録ハーネス(ROS2ノード)。

背景: Quad-SDKはROS2ノード群(シミュレータ+planner+controller)を組み合わせて動く
ため、Quadruped-PyMPC版(step_01_baseline.py)のような単一プロセスのPythonループ
では記録できない。
目的: state/ground_truthとcontrol/grfsを購読し、Quadruped-PyMPC版のstate_log.csv
と近い形式でCSVに記録する。GIF/動画は quad_mujoco.py の recording:=true 機能
(mp4出力)を使う(このスクリプトでは扱わない)。

実行方法: ROS2(source /opt/ros/jazzy/setup.bash)とsystem python3が必要。
このプロジェクトの.venv(uv管理)にはrclpyが無いため、`uv run`では実行できない。
scripts/trial/run_quadsdk_step01_baseline.sh から呼ばれる想定。
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from quad_msgs.msg import GRFArray, RobotPlan, RobotState

# quad_msgs/MultiFootState.msgのコメントに明記された脚順(0=FL,1=BL,2=FR,3=BR)。
# Quadruped-PyMPC版(FL,FR,RL,RR)とは並びが異なる点に注意。
# GRFArray.vectors/points/contact_statesも同じ4要素配列の並びに従う前提(未確認、要検証)。
LEG_ORDER = ["FL", "BL", "FR", "BR"]

# go2の立位base高さは約0.3m(state_log.csvの実測値より)。転倒判定の閾値として
# その半分程度(0.15m)を下回ったら「転倒」とみなす。公式のis_terminated相当の
# 仕組みはQuad-SDKに見つからなかったため、Quadruped-PyMPC版とは異なる簡易的な代用。
FALL_HEIGHT_THRESHOLD_M = 0.15


def _next_trial_id(summary_path: Path) -> int:
    """summary_pathの既存行から次の連番idを決める(Quadruped-PyMPC版と同じ方式)。

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


class Step01Recorder(Node):
    """state/ground_truthとcontrol/grfsを購読し、指定秒数分をCSVへ記録するノード。

    Args:
        robot_ns: ロボットの名前空間(例: "robot_1")。
        duration_s: 記録する実時間[s]。
        csv_path: 出力CSVのパス。
        summary_csv_path: 試行サマリCSVのパス(id, velocity_mps, sim_time_s,
            walk_dist_x/y_m, fall_time_sを1行追記する)。
        velocity_mps: 指令した前進速度[m/s](サマリ記録用。この値自体はROS2から
            読み取れないため呼び出し側から渡す)。

    入力: `/{robot_ns}/state/ground_truth`(quad_msgs/RobotState)、
        `/{robot_ns}/control/grfs`(quad_msgs/GRFArray)、
        `/{robot_ns}/local_plan`(quad_msgs/RobotPlan、NMPC計算時間・反復数の診断用)
        を購読する。
    出力: なし(csv_path/summary_csv_pathへの書き込みは_on_timeout内で行う)。
    """

    def __init__(
        self, robot_ns: str, duration_s: float, csv_path: Path,
        summary_csv_path: Path, velocity_mps: float,
    ) -> None:
        super().__init__("quadsdk_step01_recorder")
        self._csv_path = csv_path  # 出力CSVのパス
        self._summary_csv_path = summary_csv_path  # 試行サマリCSVのパス(Quadruped-PyMPC版のtrials_summary.csv相当)
        self._velocity_mps = velocity_mps  # 指令した前進速度[m/s](サマリ記録用)
        self._duration_s = duration_s  # 記録する実時間[s]
        self._rows: list[dict] = []  # 1行=1回のstate/ground_truth受信
        self._latest_grf: GRFArray | None = None  # 直近受信したGRFArray(state受信時に付加する)
        self._latest_plan: RobotPlan | None = None  # 直近受信したlocal_plan(state受信時に付加する)
        self._fall_time_s: float | None = None  # base_pos_zが最初に閾値を下回った時刻[s](無ければNone)
        self._t0_msg_s: float | None = None  # 最初に受信したstateメッセージのheader時刻[s](sim_time_s計算の基準点)

        # control/grfs、local_planは受信するたびに最新値をキャッシュするだけ
        # (state/ground_truthより低頻度、かつlocal_planはNMPC解が成功した時しか
        # publishされない=欠落自体が失敗の間接証拠になる)。
        self.create_subscription(GRFArray, f"/{robot_ns}/control/grfs", self._on_grf, 10)
        self.create_subscription(RobotPlan, f"/{robot_ns}/local_plan", self._on_plan, 10)
        self.create_subscription(RobotState, f"/{robot_ns}/state/ground_truth", self._on_state, 10)

        # duration_s(壁時計秒)経過後に記録を打ち切ってCSVを書き出すタイマー(1回だけ発火)。
        # ノード自身の時計(use_sim_time)には合わせない: quad_mujoco.pyの/clock配信タイミング次第で
        # get_clock().now()が0から実際のsim時刻へ不連続にジャンプし、このタイマーも巻き込まれて
        # 直後に発火してしまう不具合があったため(1回だけしか記録されない事象で発覚)。
        # 各行のsim_time_sはstate/ground_truthのheader.stamp(下記_on_state参照)から計算する。
        self._timer = self.create_timer(duration_s, self._on_timeout)

    def _on_grf(self, msg: GRFArray) -> None:
        """control/grfs受信コールバック。最新値をキャッシュするだけ(戻り値なし)。"""
        self._latest_grf = msg

    def _on_plan(self, msg: RobotPlan) -> None:
        """local_plan受信コールバック。最新値をキャッシュするだけ(戻り値なし)。

        quad_msgs/RobotPlan.compute_time[ms]とdiagnostics.iterations(IPOPT反復数、
        nmpc_controller/src/quad_nlp.cpp:1338のip_data->iter_count()由来)を記録用に
        保持する。NMPC解が失敗した回はpublishLocalPlan()自体が呼ばれない
        (local_planner.cpp:657)ため、このメッセージが来ないこと自体が失敗の
        間接的なシグナルになる。
        """
        self._latest_plan = msg

    def _on_state(self, msg: RobotState) -> None:
        """state/ground_truth受信コールバック。1行分の記録をself._rowsへ追加する(戻り値なし)。"""
        # msg.header.stamp(builtin_interfaces/Time、sec+nanosec)はpublisher側が付与した
        # シミュレーション時刻。自ノードの時計より信頼できるためこちらを使う。
        msg_time_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self._t0_msg_s is None:
            self._t0_msg_s = msg_time_s  # 最初の受信を基準(sim_time_s=0)にする
        sim_time_s = msg_time_s - self._t0_msg_s  # [s]、記録開始からの経過時間

        # quad_msgs/RobotState: body(BodyState)がpose(位置+姿勢)とtwist(速度)を持つ。
        # 全てworld座標系(quad_msgs/BodyState.msgのコメント通り)。
        pos = msg.body.pose.position  # geometry_msgs/Point、(x,y,z)[m]
        quat = msg.body.pose.orientation  # geometry_msgs/Quaternion、(x,y,z,w)
        lin = msg.body.twist.linear  # geometry_msgs/Vector3、(x,y,z)[m/s]
        ang = msg.body.twist.angular  # geometry_msgs/Vector3、(x,y,z)[rad/s]
        # クォータニオン(x,y,z,w)からroll,pitch,yaw[rad]を計算(Quadruped-PyMPC版と同じscipy規約)
        roll, pitch, yaw = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")

        # 単位・座標系はstate_log.csv(Quadruped-PyMPC版)に合わせる: world座標系、
        # m, m/s, rad, rad/s, N。脚順序はLEG_ORDER(FL,BL,FR,BR)で固定。
        row = {
            "step": len(self._rows),
            "sim_time_s": sim_time_s,
            "base_pos_x_m": pos.x,
            "base_pos_y_m": pos.y,
            "base_pos_z_m": pos.z,
            "base_roll_rad": roll,
            "base_pitch_rad": pitch,
            "base_yaw_rad": yaw,
            "base_lin_vel_x_mps": lin.x,
            "base_lin_vel_y_mps": lin.y,
            "base_lin_vel_z_mps": lin.z,
            "base_ang_vel_x_radps": ang.x,
            "base_ang_vel_y_radps": ang.y,
            "base_ang_vel_z_radps": ang.z,
        }
        # local_planは失敗回はpublishされないため、直近成功分をキャッシュしたまま
        # 使い回すことになる。plan_age_sで「そのキャッシュがどれだけ古いか」を示す
        # (値が伸び続けている区間 = local_planが更新されていない = 失敗が続いている
        # 可能性を示す間接指標)。RobotPlan.compute_time[ms]はlocal_planner全体の
        # 計算時間、diagnostics.iterationsはIPOPTの反復回数(quad_nlp.cpp:1338)。
        if self._latest_plan is not None:
            plan_msg_time_s = (
                self._latest_plan.header.stamp.sec
                + self._latest_plan.header.stamp.nanosec / 1e9
            )
            row["plan_age_s"] = sim_time_s - (plan_msg_time_s - self._t0_msg_s)
            row["plan_compute_time_ms"] = self._latest_plan.compute_time
            row["plan_nmpc_iterations"] = self._latest_plan.diagnostics.iterations
            row["plan_nmpc_cost"] = self._latest_plan.diagnostics.cost
        else:
            row["plan_age_s"] = None
            row["plan_compute_time_ms"] = None
            row["plan_nmpc_iterations"] = None
            row["plan_nmpc_cost"] = None

        # contact/grf列は行ごとに有無が変わるとcsv.DictWriterがfieldnames不一致で
        # 例外を出すため、GRFArrayを未受信でも既定値(False/0.0)で必ず全列を埋める。
        for i, leg in enumerate(LEG_ORDER):
            if self._latest_grf is not None:
                row[f"contact_{leg}"] = bool(self._latest_grf.contact_states[i])
                v = self._latest_grf.vectors[i]  # geometry_msgs/Vector3、(x,y,z)[N]、world座標系
                row[f"grf_{leg}_x_N"] = v.x
                row[f"grf_{leg}_y_N"] = v.y
                row[f"grf_{leg}_z_N"] = v.z
            else:
                row[f"contact_{leg}"] = False
                row[f"grf_{leg}_x_N"] = 0.0
                row[f"grf_{leg}_y_N"] = 0.0
                row[f"grf_{leg}_z_N"] = 0.0
        if self._fall_time_s is None and pos.z < FALL_HEIGHT_THRESHOLD_M:
            self._fall_time_s = sim_time_s  # 最初に閾値を下回った時刻のみ記録

        self._rows.append(row)

    def _on_timeout(self) -> None:
        """duration_s経過時に1回だけ呼ばれる。CSV+試行サマリを書き出してノードを終了する(戻り値なし)。"""
        self._timer.cancel()
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        if self._rows:
            with open(self._csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(self._rows[0].keys()))
                writer.writeheader()
                writer.writerows(self._rows)
        self.get_logger().info(f"Wrote {len(self._rows)} rows to {self._csv_path}")

        if self._rows:
            trial_id = _next_trial_id(self._summary_csv_path)
            summary_row = {
                "id": f"{trial_id:02d}",  # 2桁ゼロ埋め表記(Quadruped-PyMPC版と同じ)
                "velocity_mps": self._velocity_mps,
                "sim_time_s": self._rows[-1]["sim_time_s"],
                "walk_dist_x_m": self._rows[-1]["base_pos_x_m"] - self._rows[0]["base_pos_x_m"],
                "walk_dist_y_m": self._rows[-1]["base_pos_y_m"] - self._rows[0]["base_pos_y_m"],
                "fall_time_s": self._fall_time_s,
            }
            write_header = not self._summary_csv_path.exists()
            with open(self._summary_csv_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(summary_row.keys()))
                if write_header:
                    writer.writeheader()
                writer.writerow(summary_row)
            self.get_logger().info(f"Appended trial {summary_row['id']} to {self._summary_csv_path}")

        rclpy.shutdown()


def main() -> None:
    """コマンドライン引数を読み、Step01Recorderをduration_s秒間spinする(戻り値なし)。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ns", default="robot_1")  # quad_mujoco.pyのrobot_configs既定値と合わせる
    parser.add_argument("--duration-s", type=float, default=10.0)  # 記録する実時間[s]
    parser.add_argument("--csv-path", type=Path, required=True)  # 出力CSVのパス
    parser.add_argument("--summary-csv-path", type=Path, required=True)  # 試行サマリCSVのパス
    parser.add_argument("--velocity-mps", type=float, required=True)  # 指令した前進速度[m/s](サマリ記録用)
    args = parser.parse_args()

    rclpy.init()
    node = Step01Recorder(
        args.robot_ns, args.duration_s, args.csv_path,
        args.summary_csv_path, args.velocity_mps,
    )
    rclpy.spin(node)  # _on_timeoutがrclpy.shutdown()を呼ぶまでブロックする


if __name__ == "__main__":
    main()
