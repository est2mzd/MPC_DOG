"""Minimal config.py excerpt for preset apply tests (from upstream Quadruped-PyMPC)."""
robot = 'go2'
mass = 15.019
gravity_constant = 9.81
hip_height = 0.3

mpc_params = {
 'type': 'nominal',
 'verbose': False,
 'horizon': 12,
 'dt': 0.02,
 "grf_max": mass * gravity_constant,
 "grf_min": 0,
 'mu': 0.5,
 'use_foothold_optimization': True,
 'solver_mode': 'balance',
 }

simulation_params = {
 'swing_position_gain_fb': 500,
 'swing_velocity_gain_fb': 10,
 'impedence_joint_position_gain': 10.0,
 'impedence_joint_velocity_gain': 2.0,
 'step_height': 0.2 * hip_height,
 "visual_foothold_adaptation": 'blind',
 'dt': 0.002,
 'gait': 'trot',
 'gait_params': {'trot': {'step_freq': 1.4, 'duty_factor': 0.65, 'type': 0},
 'crawl': {'step_freq': 0.5, 'duty_factor': 0.8, 'type': 1},
 },
 'mode': 'human',
 'ref_z': hip_height,
 'mpc_frequency': 100,
 'scene': 'flat',
 }
