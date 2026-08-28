#!/usr/bin/env python3
"""Run the ROS-independent qiayuanl/legged_control A1 adapter headlessly."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from legged_control_mujoco import (
    A1HeadlessAdapter,
    MotionCommand,
    ScenarioConfig,
    save_gif,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Headless MuJoCo execution adapter for legged_control A1 (not Quadruped-PyMPC)."
    )
    parser.add_argument("--duration", type=float, default=0.2, help="simulation duration in seconds")
    parser.add_argument("--gait", default="stance", help="upstream gait.info template name")
    parser.add_argument(
        "--render",
        choices=("none", "rgb"),
        default="none",
        help="offscreen RGB rendering; no GUI is created",
    )
    parser.add_argument("--gif", type=Path, help="optional GIF output (implies --render rgb)")
    parser.add_argument("--seed", type=int, default=0, help="deterministic NumPy seed")
    parser.add_argument("--vx", type=float, default=0.0, help="commanded world x velocity [m/s]")
    parser.add_argument("--vy", type=float, default=0.0, help="commanded world y velocity [m/s]")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="commanded yaw rate [rad/s]")
    parser.add_argument("--base-height", type=float, default=0.32, help="commanded base height [m]")
    parser.add_argument("--friction", type=float, default=0.6, help="floor/foot sliding friction")
    parser.add_argument("--payload-mass", type=float, default=0.0, help="additional trunk mass [kg]")
    parser.add_argument(
        "--push-force",
        type=float,
        nargs=3,
        metavar=("FX", "FY", "FZ"),
        default=(0.0, 0.0, 0.0),
        help="world-frame external trunk force [N]",
    )
    parser.add_argument("--push-start", type=float, default=0.0)
    parser.add_argument("--push-duration", type=float, default=0.0)
    parser.add_argument("--fps", type=int, default=30, help="GIF/render frame rate")
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--height", type=int, default=360)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    adapter = A1HeadlessAdapter(
        gait=args.gait,
        seed=args.seed,
        command=MotionCommand(args.vx, args.vy, args.yaw_rate, args.base_height),
        scenario=ScenarioConfig(
            friction=args.friction,
            payload_mass=args.payload_mass,
            push_force=tuple(args.push_force),
            push_start=args.push_start,
            push_duration=args.push_duration,
        ),
    )
    result = adapter.run(
        duration=args.duration,
        render=args.render == "rgb" or args.gif is not None,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    if args.gif is not None:
        save_gif(result.frames, args.gif, args.fps)
    summary = {
        "duration": args.duration,
        "steps_control": len(result.times),
        "frames": len(result.frames),
        "gait": args.gait,
        "modes_seen": sorted(set(result.modes)),
        "final_base_xyz": result.qpos[-1, :3].round(8).tolist(),
        "final_base_rpy": result.metrics.base_rpy[-1].round(8).tolist(),
        "distance_xy": result.distance_xy,
        "velocity_tracking_rmse": result.tracking_rmse.round(8).tolist(),
        "max_abs_torque_nm": float(abs(result.torques).max()),
        "torque_saturation_fraction": result.torque_saturation_fraction,
        "max_post_saturation_dynamics_residual": result.max_dynamics_residual,
        "planned_contact_fraction": result.metrics.planned_contacts.mean(axis=0).tolist(),
        "measured_contact_fraction": result.metrics.measured_contacts.mean(axis=0).tolist(),
        "fallen": result.fallen,
        "fall_time": result.fall_time,
        "gif": str(args.gif) if args.gif is not None else None,
        "implementation": "friction-constrained centroidal wrench plan + MuJoCo acceleration WBC; not OCS2 SQP",
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
