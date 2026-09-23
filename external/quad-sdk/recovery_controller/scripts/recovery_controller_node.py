#!/usr/bin/env python3

import csv
import json
import math
import os
import sys
from copy import deepcopy

import yaml


def body_up_value(qx, qy):
    return 1.0 - 2.0 * (qx * qx + qy * qy)


def body_left_up_value(qx, qy, qz, qw):
    return 2.0 * (qy * qz + qw * qx)


def classify_pose(qx, qy, qz, qw, height, cfg):
    up = body_up_value(qx, qy)
    if up <= cfg["supine_up_max"]:
        return "supine"
    if abs(up) <= cfg["side_abs_up_max"]:
        return "left_side" if body_left_up_value(qx, qy, qz, qw) < 0.0 else "right_side"
    if up >= cfg["upright_up_min"] and height >= cfg["upright_height_min"]:
        return "upright"
    if up > 0.0 and height < cfg["upright_height_min"]:
        return "prone"
    return "unknown"


def interpolate(start, goal, fraction):
    """Move most of the way at the start so the next frame keeps the swing.

    A smoothstep stops at both ends, so each keyframe begins from rest.
    This curve has a high initial rate and only settles as it arrives.
    """
    fraction = min(1.0, max(0.0, fraction))
    weight = 1.0 - (1.0 - fraction) ** 3
    return [a + weight * (b - a) for a, b in zip(start, goal)]


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def mirror_left_right(joints):
    """Swap left and right legs and reverse abad so the opposite side pushes."""
    swapped = joints[6:9] + joints[9:12] + joints[0:3] + joints[3:6]
    for index in (0, 3, 6, 9):
        swapped[index] *= -1.0
    return swapped


def recovery_frame_action(frame_name, pose, push_reversed, up=None):
    """Choose the next recovery step after the current frame duration elapses.

    Supine recovery stays on the roll frames until the back leaves the ground.
    A push that has already lifted the back keeps the same legs instead of
    swapping sides. The stand frames are used only after the classified pose
    has changed.
    """
    if frame_name == "side_push" and pose in ("left_side", "right_side"):
        return "hold"
    if pose == "supine" and frame_name == "side_push":
        if up is not None and up > -0.75:
            return "hold"
        if not push_reversed:
            return "reverse_push"
        return "retry"
    if pose == "supine" and frame_name in ("low_support", "stand"):
        return "retry"
    return "advance"


def should_switch_sequence(operation, pose):
    """Keep the supine roll on its own frames until the body is standing.

    A left-side roll that has reached the back continues into the supine
    push while the body is still rotating. Switching into the side tuck
    while a supine push has only reached the side drops the back down.
    """
    if operation in ("left_side", "right_side") and pose == "supine":
        return True
    if operation in ("supine", "left_side", "right_side", "prone"):
        return False
    if pose == operation or pose in ("unknown", "upright"):
        return False
    return True


def supine_chain_start(operation):
    """Keep the legs that are already extended when a side roll meets the back.

    The left-side roll extends the right legs. Replaying open_gap pulls both
    hips toward 2.60 rad and stops that roll. A right-side roll continues
    with the default left-leg push.
    """
    if operation == "left_side":
        return "side_push", True
    if operation == "right_side":
        return "side_push", False
    return None, False


def sequence_name_for_pose(pose, up):
    """Pick the recovery motion for the face that is toward the ground.

    A side pose with the back already rising is the edge of prone, so the
    stand frames are used. The left-side tuck remains for a back-down start.
    """
    if pose in ("left_side", "right_side") and up > 0.0:
        return "prone"
    if pose in ("supine", "left_side", "right_side", "prone"):
        return pose
    return "prone"


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    required = ("detection", "control", "sequences")
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError(f"missing recovery config sections: {missing}")
    for name, frames in config["sequences"].items():
        if not frames:
            raise ValueError(f"sequence {name} is empty")
        for index, frame in enumerate(frames):
            if len(frame["joints"]) != 12:
                raise ValueError(f"sequence {name} frame {index} must have 12 joints")
            if float(frame["duration"]) <= 0.0:
                raise ValueError(f"sequence {name} frame {index} duration must be positive")
    return config


def main():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Twist
    from quad_msgs.msg import LegCommand, LegCommandArray, MotorCommand, RobotPlan, RobotState
    from rclpy.node import Node
    from std_msgs.msg import String, UInt8

    class RecoveryController(Node):
        def __init__(self):
            super().__init__("recovery_controller")
            default_config = os.path.join(
                get_package_share_directory("recovery_controller"),
                "config",
                "go2_recovery.yaml",
            )
            self.declare_parameter("config_path", default_config)
            self.declare_parameter("log_path", "")
            self.declare_parameter("automatic_recovery", True)
            self.config = load_config(self.get_parameter("config_path").value)
            self.det = self.config["detection"]
            self.ctrl = self.config["control"]

            self.state = None
            self.walking_command = None
            self.raw_twist = Twist()
            self.raw_twist_time = None
            self.latest_plan_stamp = -1.0
            self.mode = "MONITOR"
            self.operation = "pass_through"
            self.pose = "unknown"
            self.fall_pending_since = None
            self.recovery_started = None
            self.recovery_completed = None
            self.upright_since = None
            self.frame_started = None
            self.frame_index = 0
            self.frame_start_joints = [0.0] * 12
            self.sequence = []
            self.push_reversed = False
            self.hold_count = 0
            self.resume_fall_since = None
            self.reaching = False
            self.reach_played = False
            self.pose_stable_since = None
            self.pose_stable_name = ""
            self.switched_at = -1.0
            self.attempt = 0
            self.failure_reason = ""
            self.last_status_text = ""

            self.command_pub = self.create_publisher(
                LegCommandArray, "control/joint_command", 1
            )
            self.mode_request_pub = self.create_publisher(UInt8, "control/mode", 1)
            self.status_pub = self.create_publisher(String, "recovery/status", 10)
            self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
            self.create_subscription(
                RobotState, "state/ground_truth", self.on_state, 10
            )
            self.create_subscription(
                LegCommandArray,
                "control/walking_joint_command",
                self.on_walking_command,
                1,
            )
            self.create_subscription(Twist, "recovery/cmd_vel_input", self.on_twist, 10)
            self.create_subscription(RobotPlan, "local_plan", self.on_plan, 1)
            self.timer = self.create_timer(
                1.0 / float(self.ctrl["publish_rate"]), self.on_timer
            )

            self.log_file = None
            self.log_writer = None
            log_path = self.get_parameter("log_path").value
            if log_path:
                os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
                self.log_file = open(log_path, "w", newline="", encoding="utf-8")
                self.log_writer = csv.writer(self.log_file)
                self.log_writer.writerow(
                    [
                        "time_s",
                        "mode",
                        "operation",
                        "pose",
                        "owner",
                        "attempt",
                        "keyframe",
                        "u",
                        "left_up",
                        "height_m",
                        "roll_rad",
                        "pitch_rad",
                        "angular_speed_rad_s",
                        "plan_stamp_s",
                        "failure_reason",
                    ]
                    + [f"joint_pos_{index}" for index in range(12)]
                    + [f"joint_cmd_{index}" for index in range(12)]
                )
            self.get_logger().info("Recovery controller is ready")

        def destroy_node(self):
            if self.log_file is not None:
                self.log_file.flush()
                self.log_file.close()
            super().destroy_node()

        def on_state(self, msg):
            self.state = msg

        def on_walking_command(self, msg):
            self.walking_command = msg

        def on_twist(self, msg):
            self.raw_twist = msg
            self.raw_twist_time = self.get_clock().now()

        def on_plan(self, msg):
            self.latest_plan_stamp = stamp_seconds(msg.header.stamp)

        def now_seconds(self):
            return self.get_clock().now().nanoseconds * 1.0e-9

        def pose_values(self):
            orientation = self.state.body.pose.orientation
            qx = orientation.x
            qy = orientation.y
            qz = orientation.z
            qw = orientation.w
            up = body_up_value(qx, qy)
            left_up = body_left_up_value(qx, qy, qz, qw)
            height = self.state.body.pose.position.z
            pose = classify_pose(qx, qy, qz, qw, height, self.det)
            wx = self.state.body.twist.angular.x
            wy = self.state.body.twist.angular.y
            wz = self.state.body.twist.angular.z
            angular_speed = math.sqrt(wx * wx + wy * wy + wz * wz)
            roll = math.atan2(
                2.0 * (qw * qx + qy * qz),
                1.0 - 2.0 * (qx * qx + qy * qy),
            )
            pitch = math.asin(
                min(1.0, max(-1.0, 2.0 * (qw * qy - qz * qx)))
            )
            return pose, up, left_up, height, angular_speed, roll, pitch

        def publish_mode_request(self, value):
            msg = UInt8()
            msg.data = value
            self.mode_request_pub.publish(msg)

        def start_recovery(self, pose, now):
            if self.attempt >= int(self.ctrl["max_attempts"]):
                self.enter_safety("attempt_limit")
                return
            _, up, _, _, _, _, _ = self.pose_values()
            sequence_name = sequence_name_for_pose(pose, up)
            self.sequence = self.copy_sequence(sequence_name)
            self.mode = "RECOVERY"
            self.operation = sequence_name
            self.pose = pose
            self.attempt += 1
            self.recovery_started = now
            self.frame_started = now
            self.frame_index = 0
            self.upright_since = None
            self.frame_start_joints = list(self.state.joints.position[:12])
            self.push_reversed = False
            self.hold_count = 0
            self.resume_fall_since = None
            self.reaching = False
            self.reach_played = False
            self.publish_mode_request(1)
            self.get_logger().warn(
                f"RECOVERY start pose={pose} attempt={self.attempt}"
            )

        def copy_sequence(self, pose):
            return [
                {**frame, "joints": list(frame["joints"])}
                for frame in self.config["sequences"][pose]
            ]

        def switch_sequence(self, pose, now, start_name=None, mirror=False):
            self.sequence = self.copy_sequence(pose)
            self.operation = pose
            self.frame_index = 0
            self.push_reversed = False
            if mirror:
                for frame in self.sequence:
                    frame["joints"] = mirror_left_right(list(frame["joints"]))
                self.push_reversed = True
            if start_name:
                for index, frame in enumerate(self.sequence):
                    if frame["name"] == start_name:
                        self.frame_index = index
                        break
            self.frame_started = now
            self.frame_start_joints = list(self.state.joints.position[:12])
            self.switched_at = now
            self.get_logger().info(f"RECOVERY pose changed to {pose}")

        def update_pose_sequence(self, pose, now):
            if self.operation in ("left_side", "right_side") and pose == "supine":
                start_name, mirror = supine_chain_start(self.operation)
                self.switch_sequence(
                    pose, now, start_name=start_name, mirror=mirror
                )
                return
            if not should_switch_sequence(self.operation, pose):
                self.pose_stable_name = pose
                self.pose_stable_since = None
                return
            if now - self.switched_at < 0.5:
                return
            if pose != self.pose_stable_name:
                self.pose_stable_name = pose
                self.pose_stable_since = now
                return
            if now - self.pose_stable_since >= 0.2:
                self.switch_sequence(pose, now)

        def reverse_push(self, now):
            frame = self.sequence[self.frame_index]
            frame["joints"] = mirror_left_right(list(frame["joints"]))
            self.frame_started = now
            self.frame_start_joints = list(self.state.joints.position[:12])
            self.push_reversed = True
            self.get_logger().warn("RECOVERY reverses the supine push side")

        def enter_safety(self, reason):
            self.mode = "SAFETY"
            self.operation = "damping"
            self.failure_reason = reason
            self.publish_mode_request(4)
            self.get_logger().error(f"RECOVERY failed reason={reason}")

        def recovery_command(self, now):
            if now - self.recovery_started > float(self.ctrl["recovery_timeout"]):
                self.enter_safety("timeout")
                return self.damping_command()
            pose, up, _, height, _, _, _ = self.pose_values()
            self.update_pose_sequence(pose, now)
            if self.reaching:
                return self.reach_command(now)
            if self.frame_index >= len(self.sequence):
                pose, up, _, height, angular_speed, _, _ = self.pose_values()
                upright = (
                    up >= float(self.det["upright_up_min"])
                    and height >= float(self.det["upright_height_min"])
                    and angular_speed <= float(self.det["settled_angular_speed_max"])
                )
                if upright:
                    if self.upright_since is None:
                        self.upright_since = now
                    if now - self.upright_since >= float(self.det["upright_hold_time"]):
                        self.mode = "RESUME"
                        self.operation = "wait_new_plan"
                        self.recovery_completed = now
                        self.publish_mode_request(1)
                        self.get_logger().info("RECOVERY upright confirmed")
                else:
                    self.upright_since = None
                    if now - self.frame_started > float(self.ctrl["final_hold_time"]):
                        next_pose = pose if pose != "upright" else "prone"
                        self.start_recovery(next_pose, now)
                return self.make_command(
                    self.sequence[-1]["joints"],
                    self.sequence[-1],
                )

            frame = self.sequence[self.frame_index]
            duration = float(frame["duration"])
            elapsed = now - self.frame_started
            target = interpolate(
                self.frame_start_joints, frame["joints"], elapsed / duration
            )
            if elapsed >= duration:
                target = list(frame["joints"])
                action = recovery_frame_action(
                    frame["name"], pose, self.push_reversed, up
                )
                if (
                    frame["name"] == "stand"
                    and action == "advance"
                    and not self.reach_played
                    and height < float(self.det["upright_height_min"])
                    and up > 0.0
                ):
                    self.reach_played = True
                    self.reaching = True
                    self.frame_started = now
                    self.frame_start_joints = list(self.state.joints.position[:12])
                    return self.make_command(self.frame_start_joints, frame)
                if action == "hold" and height >= float(self.det["upright_height_min"]):
                    action = "advance"
                if action == "hold":
                    self.hold_count += 1
                    self.frame_started = now
                    if self.hold_count == 2:
                        recoiled = list(frame["joints"])
                        for index in (1, 4, 7, 10):
                            recoiled[index] = min(float(recoiled[index]), 2.2)
                        self.frame_start_joints = recoiled
                        return self.make_command(recoiled, frame)
                    self.frame_start_joints = list(frame["joints"])
                    return self.make_command(list(frame["joints"]), frame)
                if action == "reverse_push":
                    self.reverse_push(now)
                    return self.make_command(target, frame)
                if action == "retry":
                    self.start_recovery(pose, now)
                    return self.make_command(target, frame)
                self.frame_index += 1
                self.hold_count = 0
                self.frame_started = now
                self.frame_start_joints = target
            return self.make_command(target, frame)

        def walking_command_ready(self):
            command = self.walking_command
            if command is None or len(command.leg_commands) < 4:
                return False
            count = 0
            for leg in command.leg_commands[:4]:
                if len(leg.motor_commands) < 3:
                    return False
                for motor in leg.motor_commands[:3]:
                    if not math.isfinite(float(motor.pos_setpoint)):
                        return False
                    count += 1
            return count == 12

        def resume_command(self, now):
            zero = Twist()
            pose, up, _, height, angular_speed, _, _ = self.pose_values()
            fallen = (
                up <= float(self.det["fall_up_max"])
                or height <= float(self.det["fall_height_max"])
            )
            if fallen and angular_speed <= float(self.det["settled_angular_speed_max"]):
                if self.resume_fall_since is None:
                    self.resume_fall_since = now
                elif (
                    now - self.resume_fall_since
                    >= float(self.det["fall_confirm_time"])
                ):
                    self.resume_fall_since = None
                    self.start_recovery(pose, now)
                    if self.mode != "RECOVERY":
                        return self.damping_command()
                    return self.recovery_command(now)
            else:
                self.resume_fall_since = None
            elapsed = now - self.recovery_completed
            plan_is_new = self.latest_plan_stamp > self.recovery_completed
            if not plan_is_new or not self.walking_command_ready():
                self.cmd_vel_pub.publish(zero)
                return self.stand_command()

            self.publish_mode_request(1)
            ramp = min(1.0, elapsed / float(self.ctrl["resume_ramp_time"]))
            output = deepcopy(self.raw_twist)
            output.linear.x *= ramp
            output.linear.y *= ramp
            output.linear.z *= ramp
            output.angular.x *= ramp
            output.angular.y *= ramp
            output.angular.z *= ramp
            self.cmd_vel_pub.publish(output)
            self.operation = "speed_ramp" if ramp < 1.0 else "walking"
            upright = (
                up >= float(self.det["upright_up_min"])
                and height >= float(self.det["upright_height_min"])
            )
            if ramp >= 1.0 and upright:
                self.mode = "MONITOR"
                self.operation = "pass_through"
                self.fall_pending_since = None
                self.get_logger().info("RESUME walking command restored")
            return self.walking_command

        def reach_command(self, now):
            duration = float(self.ctrl["reach_duration"])
            elapsed = now - self.frame_started
            target = interpolate(
                self.frame_start_joints,
                self.ctrl["reach_joints"],
                elapsed / duration,
            )
            frame = self.sequence[min(self.frame_index, len(self.sequence) - 1)]
            if elapsed >= duration:
                self.reaching = False
                self.frame_started = now
                self.frame_start_joints = list(self.ctrl["reach_joints"])
                for index, item in enumerate(self.sequence):
                    if item["name"] == "stand":
                        self.frame_index = index
                        break
                target = list(self.ctrl["reach_joints"])
            return self.make_command(target, frame)

        def stand_command(self):
            frame = {
                "kp": self.ctrl["stand_kp"],
                "kd": self.ctrl["stand_kd"],
                "torque_limit": self.ctrl["torque_limit"],
            }
            return self.make_command(self.ctrl["stand_joints"] * 4, frame)

        def damping_command(self):
            joints = (
                list(self.state.joints.position[:12])
                if self.state is not None
                else [0.0] * 12
            )
            frame = {"kp": 0.0, "kd": self.ctrl["safety_kd"], "torque_limit": 0.0}
            return self.make_command(joints, frame)

        def make_command(self, joints, frame):
            array = LegCommandArray()
            array.header.stamp = self.get_clock().now().to_msg()
            kp = frame["kp"]
            kd = frame["kd"]
            kp_values = kp if isinstance(kp, list) else [kp] * 3
            kd_values = kd if isinstance(kd, list) else [kd] * 3
            for leg_index in range(4):
                leg = LegCommand()
                leg.header = array.header
                for joint_index in range(3):
                    motor = MotorCommand()
                    motor.header = array.header
                    index = leg_index * 3 + joint_index
                    motor.pos_setpoint = float(joints[index])
                    motor.vel_setpoint = 0.0
                    motor.kp = float(kp_values[joint_index])
                    motor.kd = float(kd_values[joint_index])
                    motor.torque_ff = 0.0
                    leg.motor_commands.append(motor)
                array.leg_commands.append(leg)
            return array

        def command_positions(self, command):
            if command is None:
                return [float("nan")] * 12
            return [
                motor.pos_setpoint
                for leg in command.leg_commands
                for motor in leg.motor_commands[:3]
            ][:12]

        def publish_status(self, values, owner):
            pose, up, left_up, height, angular_speed, roll, pitch = values
            status = {
                "mode": self.mode,
                "operation": self.operation,
                "pose": pose,
                "owner": owner,
                "attempt": self.attempt,
                "keyframe": self.frame_index,
                "u": round(up, 5),
                "height": round(height, 5),
                "failure_reason": self.failure_reason,
            }
            text = json.dumps(status, sort_keys=True)
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)
            if text != self.last_status_text:
                self.last_status_text = text

        def write_log(self, values, owner, command, now):
            if self.log_writer is None:
                return
            pose, up, left_up, height, angular_speed, roll, pitch = values
            joints = list(self.state.joints.position[:12])
            joints += [float("nan")] * (12 - len(joints))
            self.log_writer.writerow(
                [
                    f"{now:.9f}",
                    self.mode,
                    self.operation,
                    pose,
                    owner,
                    self.attempt,
                    self.frame_index,
                    f"{up:.6f}",
                    f"{left_up:.6f}",
                    f"{height:.6f}",
                    f"{roll:.6f}",
                    f"{pitch:.6f}",
                    f"{angular_speed:.6f}",
                    f"{self.latest_plan_stamp:.9f}",
                    self.failure_reason,
                ]
                + joints
                + self.command_positions(command)
            )
            self.log_file.flush()

        def on_timer(self):
            if self.state is None:
                return
            now = self.now_seconds()
            values = self.pose_values()
            pose, up, _, height, angular_speed, _, _ = values
            self.pose = pose
            fallen = (
                up <= float(self.det["fall_up_max"])
                or height <= float(self.det["fall_height_max"])
            )

            command = None
            owner = "walking"
            if self.mode == "MONITOR":
                if fallen and angular_speed <= float(
                    self.det["settled_angular_speed_max"]
                ):
                    if self.fall_pending_since is None:
                        self.fall_pending_since = now
                    elif (
                        self.get_parameter("automatic_recovery").value
                        and now - self.fall_pending_since
                        >= float(self.det["fall_confirm_time"])
                    ):
                        self.start_recovery(pose, now)
                else:
                    self.fall_pending_since = None
                if self.walking_command_ready():
                    command = self.walking_command
                elif not bool(self.get_parameter("automatic_recovery").value):
                    command = self.stand_command()
                else:
                    command = self.walking_command or self.damping_command()
            if self.mode == "RECOVERY":
                owner = "recovery"
                command = self.recovery_command(now)
                self.cmd_vel_pub.publish(Twist())
            elif self.mode == "RESUME":
                owner = "recovery"
                command = self.resume_command(now)
            elif self.mode == "SAFETY":
                owner = "safety"
                command = self.damping_command()
                self.cmd_vel_pub.publish(Twist())
            elif self.mode == "MONITOR":
                if self.raw_twist_time is not None:
                    age = (self.get_clock().now() - self.raw_twist_time).nanoseconds * 1.0e-9
                    self.cmd_vel_pub.publish(
                        self.raw_twist
                        if age <= float(self.ctrl["cmd_vel_timeout"])
                        else Twist()
                    )

            if command is not None:
                self.command_pub.publish(command)
            self.publish_status(values, owner)
            self.write_log(values, owner, command, now)

    rclpy.init(args=sys.argv)
    node = RecoveryController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
