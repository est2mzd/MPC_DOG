#!/usr/bin/env python3
"""Step 17 前方ジャンプ用の計測ハーネス(ROS2ノード)。

背景: step01 の quadsdk_step01_baseline.py は body pose/vel と control/grfs
(NMPC が出した計画 GRF・計画接触) と NMPC 診断しか記録しない。Step 17 の
「後脚踏切・四脚実離地・穴越え・安定着地を計測値で確認する」判定には、
課題 §10 が要求する次が足りない:
  - 足先位置(実測) / 実測接触(シミュレータ) / 実測 normal force
  - 関節 position / velocity(実測) / 指令トルク
  - primitive_id / jump_phase

このノードはそれらを 1 本の CSV に足す。step01 側は他ステップが使うため触らない。

購読:
  /{ns}/state/ground_truth  quad_msgs/RobotState  … 行の駆動。body / joints / feet
  /{ns}/control/grfs         quad_msgs/GRFArray    … 計画接触 + 計画 GRF(NMPC)
  /{ns}/state/grfs           quad_msgs/GRFArray    … 実測 GRF(あれば)
  /{ns}/local_plan           quad_msgs/RobotPlan   … primitive_ids[0] + 診断
  /{ns}/control/joint_command quad_msgs/LegCommandArray … 指令トルク

実行: system python3 + source /opt/ros/jazzy/setup.bash + ws の setup.bash。
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from quad_msgs.msg import GRFArray, LegCommandArray, RobotPlan, RobotState

# quad_msgs/MultiFootState.msg と GRFArray の脚順: 0=FL 1=BL 2=FR 3=BR。
# 課題の "FL/RL/FR/RR" は RL=BL(左後), RR=BR(右後) と読み替える。
LEG_ORDER = ["FL", "BL", "FR", "BR"]

# primitive_id -> jump_phase ラベル。quad_utils/primitive_ids.hpp の PrimitiveId と一致。
PHASE_LABEL = {
    0: "connect",
    1: "leap_stance",
    2: "flight",
    3: "land_stance",
    4: "preload",
    5: "rear_push",
    6: "front_land",
    7: "settle",
}

FALL_HEIGHT_THRESHOLD_M = 0.15


def _next_trial_id(summary_path: Path) -> int:
    if not summary_path.exists():
        return 1
    with open(summary_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return 1
    return max(int(row["id"]) for row in rows) + 1


class Step17Recorder(Node):
    def __init__(
        self, robot_ns: str, duration_s: float, csv_path: Path,
        summary_csv_path: Path, velocity_mps: float,
    ) -> None:
        super().__init__("quadsdk_step17_recorder")
        self._csv_path = csv_path
        self._summary_csv_path = summary_csv_path
        self._velocity_mps = velocity_mps
        self._duration_s = duration_s
        self._rows: list[dict] = []
        self._grf_planned: GRFArray | None = None
        self._grf_measured: GRFArray | None = None
        self._plan: RobotPlan | None = None
        self._cmd: LegCommandArray | None = None
        self._fall_time_s: float | None = None
        self._first_flight_s: float | None = None
        self._t0_msg_s: float | None = None

        self.create_subscription(GRFArray, f"/{robot_ns}/control/grfs", self._on_grf_planned, 10)
        self.create_subscription(GRFArray, f"/{robot_ns}/state/grfs", self._on_grf_measured, 10)
        self.create_subscription(RobotPlan, f"/{robot_ns}/local_plan", self._on_plan, 10)
        self.create_subscription(LegCommandArray, f"/{robot_ns}/control/joint_command", self._on_cmd, 10)
        self.create_subscription(RobotState, f"/{robot_ns}/state/ground_truth", self._on_state, 10)

        self._timer = self.create_timer(duration_s, self._on_timeout)

    def _on_grf_planned(self, msg: GRFArray) -> None:
        self._grf_planned = msg

    def _on_grf_measured(self, msg: GRFArray) -> None:
        self._grf_measured = msg

    def _on_plan(self, msg: RobotPlan) -> None:
        self._plan = msg

    def _on_cmd(self, msg: LegCommandArray) -> None:
        self._cmd = msg

    def _on_state(self, msg: RobotState) -> None:
        msg_time_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self._t0_msg_s is None:
            self._t0_msg_s = msg_time_s
        sim_time_s = msg_time_s - self._t0_msg_s

        pos = msg.body.pose.position
        quat = msg.body.pose.orientation
        lin = msg.body.twist.linear
        ang = msg.body.twist.angular
        roll, pitch, yaw = Rotation.from_quat(
            [quat.x, quat.y, quat.z, quat.w]).as_euler("xyz")

        row: dict = {
            "step": len(self._rows),
            "sim_time_s": sim_time_s,
            "base_pos_x_m": pos.x,
            "base_pos_y_m": pos.y,
            "base_pos_z_m": pos.z,
            "base_lin_vel_x_mps": lin.x,
            "base_lin_vel_y_mps": lin.y,
            "base_lin_vel_z_mps": lin.z,
            "base_roll_rad": roll,
            "base_pitch_rad": pitch,
            "base_yaw_rad": yaw,
        }

        # primitive_id / jump_phase: local_plan の先頭 primitive を「今実行中」とみなす。
        primitive_id = None
        if self._plan is not None and len(self._plan.primitive_ids) > 0:
            primitive_id = int(self._plan.primitive_ids[0])
        row["primitive_id"] = primitive_id if primitive_id is not None else ""
        row["jump_phase"] = PHASE_LABEL.get(primitive_id, "") if primitive_id is not None else ""

        # NMPC 診断
        if self._plan is not None:
            plan_msg_time_s = (
                self._plan.header.stamp.sec + self._plan.header.stamp.nanosec / 1e9)
            row["plan_age_s"] = sim_time_s - (plan_msg_time_s - self._t0_msg_s)
            row["plan_compute_time_ms"] = self._plan.compute_time
            row["plan_nmpc_iterations"] = self._plan.diagnostics.iterations
            row["plan_nmpc_cost"] = self._plan.diagnostics.cost
        else:
            row["plan_age_s"] = None
            row["plan_compute_time_ms"] = None
            row["plan_nmpc_iterations"] = None
            row["plan_nmpc_cost"] = None

        # 足先位置(実測) + 実測接触(シミュレータ) + 計画接触(control/grfs)
        n_feet = len(msg.feet.feet)
        n_meas_contact_true = 0
        for i, leg in enumerate(LEG_ORDER):
            if i < n_feet:
                fp = msg.feet.feet[i].position
                row[f"foot_{leg}_pos_x_m"] = fp.x
                row[f"foot_{leg}_pos_y_m"] = fp.y
                row[f"foot_{leg}_pos_z_m"] = fp.z
                mc = bool(msg.feet.feet[i].contact)
                row[f"measured_contact_{leg}"] = int(mc)
                n_meas_contact_true += int(mc)
            else:
                row[f"foot_{leg}_pos_x_m"] = None
                row[f"foot_{leg}_pos_y_m"] = None
                row[f"foot_{leg}_pos_z_m"] = None
                row[f"measured_contact_{leg}"] = None

            if self._grf_planned is not None and i < len(self._grf_planned.contact_states):
                row[f"planned_contact_{leg}"] = int(bool(self._grf_planned.contact_states[i]))
                gv = self._grf_planned.vectors[i]
                row[f"grf_{leg}_x_N"] = gv.x
                row[f"grf_{leg}_y_N"] = gv.y
                row[f"grf_{leg}_z_N"] = gv.z
            else:
                row[f"planned_contact_{leg}"] = None
                row[f"grf_{leg}_x_N"] = 0.0
                row[f"grf_{leg}_y_N"] = 0.0
                row[f"grf_{leg}_z_N"] = 0.0

            # 実測 normal force: state/grfs があればその z、無ければ NMPC GRF の z で代用。
            if self._grf_measured is not None and i < len(self._grf_measured.vectors):
                row[f"normal_force_{leg}_N"] = self._grf_measured.vectors[i].z
            else:
                row[f"normal_force_{leg}_N"] = row[f"grf_{leg}_z_N"]

        # 実飛翔: 4脚とも実測接触なしの最初の時刻を記録。
        if (self._first_flight_s is None and n_feet == 4
                and n_meas_contact_true == 0 and sim_time_s > 1.0):
            self._first_flight_s = sim_time_s

        # 関節: 実測 pos/vel(state/ground_truth.joints) + 指令トルク(joint_command)。
        jpos = list(msg.joints.position)
        jvel = list(msg.joints.velocity)
        # joint_command の脚順は FL,BL,FR,BR、各脚 motor_commands は Abd,Hip,Knee。
        cmd_tau: list[float | None] = [None] * 12
        if self._cmd is not None and len(self._cmd.leg_commands) == 4:
            for li in range(4):
                mc = self._cmd.leg_commands[li].motor_commands
                for ji in range(min(3, len(mc))):
                    val = mc[ji].effort if mc[ji].effort != 0.0 else mc[ji].torque_ff
                    cmd_tau[li * 3 + ji] = val
        for j in range(12):
            row[f"joint_{j}_pos_rad"] = jpos[j] if j < len(jpos) else None
            row[f"joint_{j}_vel_radps"] = jvel[j] if j < len(jvel) else None
            row[f"joint_{j}_cmd_torque_Nm"] = cmd_tau[j]

        if self._fall_time_s is None and pos.z < FALL_HEIGHT_THRESHOLD_M:
            self._fall_time_s = sim_time_s

        self._rows.append(row)

    def _on_timeout(self) -> None:
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
                "id": f"{trial_id:02d}",
                "velocity_mps": self._velocity_mps,
                "sim_time_s": self._rows[-1]["sim_time_s"],
                "walk_dist_x_m": self._rows[-1]["base_pos_x_m"] - self._rows[0]["base_pos_x_m"],
                "walk_dist_y_m": self._rows[-1]["base_pos_y_m"] - self._rows[0]["base_pos_y_m"],
                "first_flight_s": self._first_flight_s,
                "fall_time_s": self._fall_time_s,
            }
            write_header = not self._summary_csv_path.exists()
            with open(self._summary_csv_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(summary_row.keys()))
                if write_header:
                    writer.writeheader()
                writer.writerow(summary_row)
            self.get_logger().info(
                f"Appended trial {summary_row['id']} to {self._summary_csv_path}")

        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ns", default="robot_1")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--csv-path", type=Path, required=True)
    parser.add_argument("--summary-csv-path", type=Path, required=True)
    parser.add_argument("--velocity-mps", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Step17Recorder(
        args.robot_ns, args.duration_s, args.csv_path,
        args.summary_csv_path, args.velocity_mps,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
