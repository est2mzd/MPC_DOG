# AGENTS.md — Quadruped-PyMPC (`external/Quadruped-PyMPC`)

This file orients an AI coding agent (or a new contributor) working on the
vendored **Quadruped-PyMPC** controller at `external/Quadruped-PyMPC`. It
describes what is canonical, where execution actually flows, and what to
check before changing anything. It is a map, not a tutorial — for a guided,
line-by-line walkthrough of the same code, see `notebook_pympc/`
(Japanese-language curriculum, notebooks 01–16) and `docs/pympc_2day/`. This
file should stay short; put deep explanations there instead of expanding
this one.

All paths below are relative to this repo's root (`mpc_dog/`) unless noted.

## What Quadruped-PyMPC is

A model-predictive controller for quadruped locomotion based on the
**single rigid body model** (Centroidal / SRBD dynamics), in two flavors:

- **gradient-based**: CasADi model + acados NLP solver (fast, ~5ms/step on CPU)
- **sampling-based**: JAX, thousands of parallel rollouts on GPU (MPPI/CEM-MPPI/random)

It is vendored at `external/Quadruped-PyMPC` (not a git submodule itself —
only the nested `external/Quadruped-PyMPC/quadruped_pympc/acados` directory
is a submodule, see `external/Quadruped-PyMPC/.gitmodules`). The directory
is otherwise excluded by `.gitignore` except for this file (see the
`external/*` exceptions there). Upstream: https://github.com/iit-DLSLab/Quadruped-PyMPC

## Canonical execution path ("nominal" flavor)

Read these files in this order to understand one full control cycle. This is
the path used when `config.py` has `mpc_params['type'] = 'nominal'`
(the default) — read this path fully before comparing other branches
(`sampling`, `input_rates`, `kinodynamic`, `lyapunov`, `collaborative`).

| # | Step | File |
|---|------|------|
| 1 | Plant observation (state, contacts, Jacobians) | `external/Quadruped-PyMPC/simulation/simulation.py::run_simulation` |
| 2 | Velocity command | `simulation_params['mode']` in `external/Quadruped-PyMPC/quadruped_pympc/config.py` |
| 3 | Gait / contact schedule | `external/Quadruped-PyMPC/quadruped_pympc/helpers/periodic_gait_generator.py::PeriodicGaitGenerator` |
| 4 | Foothold reference | `external/Quadruped-PyMPC/quadruped_pympc/helpers/foothold_reference_generator.py::FootholdReferenceGenerator` |
| 5 | State/reference assembly | `external/Quadruped-PyMPC/quadruped_pympc/interfaces/wb_interface.py::WBInterface` |
| 6 | SRBD model | `external/Quadruped-PyMPC/quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py::Centroidal_Model_Nominal` |
| 7 | OCP (acados NLP) | `external/Quadruped-PyMPC/quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py::Acados_NMPC_Nominal` |
| 8 | Receding horizon (take stage-0 GRF/foothold only) | `external/Quadruped-PyMPC/quadruped_pympc/interfaces/srbd_controller_interface.py::SRBDControllerInterface` |
| 9 | GRF → joint torque (stance: `τ = -Jᵀf`; swing: Cartesian PD + feedback linearization) | `wb_interface.py::WBInterface` (same file as step 5) |
| 10 | Actuate plant | `simulation/simulation.py` → `env.step(action)` (same file as step 1) |

Top-level integration point:
`external/Quadruped-PyMPC/quadruped_pympc/quadruped_pympc_wrapper.py::QuadrupedPyMPC_Wrapper.compute_actions` —
this is what an external caller (e.g. a real-robot ROS 2 node, or another sim)
calls each control tick; it wraps steps 3–9 above.

**Non-obvious gotcha**: `simulation_params['mpc_frequency']` (Hz) is not used
directly — it's converted to a step counter:
`step_num % round(1/(mpc_frequency*simulation_dt))`. With `dt=0.002` and
`mpc_frequency=100`, the MPC runs every 5 sim steps. Don't assume a config
name means what it sounds like — trace it to its use site.

## Module map (`external/Quadruped-PyMPC/quadruped_pympc/`)

- `config.py` — **the** place to change robot, MPC type, and tuning. Two dicts:
  `mpc_params` (controller behavior/type/horizon/constraints/solver mode) and
  `simulation_params` (gait, swing gains, scene, mpc_frequency, sim dt).
  Also holds per-robot mass/inertia (`robot = 'go1'|'go2'|'aliengo'|'b2'|...`).
- `quadruped_pympc_wrapper.py` — `QuadrupedPyMPC_Wrapper`: single entry point
  gluing gait, foothold, MPC interface, and whole-body interface together.
  `compute_actions(...)`, `get_obs()`, `reset(...)`.
- `interfaces/`
  - `srbd_controller_interface.py::SRBDControllerInterface` — dispatches on
    `mpc_params['type']` to pick the concrete controller class (see table
    below), runs the OCP, extracts stage-0 solution.
  - `srbd_batched_controller_interface.py::SRBDBatchedControllerInterface` —
    batched variant used for step-frequency optimization (multiple OCPs in parallel).
  - `wb_interface.py::WBInterface` — whole-body layer: builds MPC state/reference
    dicts, converts GRF solution to joint torques (stance PD+dynamics, swing
    Cartesian control), talks to `SwingTrajectoryController`/`TerrainEstimator`.
- `controllers/gradient/<variant>/` — one dynamics-model + acados-OCP pair per
  MPC variant. `nominal` is the reference implementation; the others are diffs
  against it:
  - `nominal/` — `Centroidal_Model_Nominal`, `Acados_NMPC_Nominal`, plus
    `Acados_NMPC_GaitAdaptive` (used when `optimize_step_freq=True`, any type).
  - `input_rates/` — optimizes Δ(GRF) instead of GRF directly.
  - `lyapunov/` — adds a Lyapunov-based stability constraint.
  - `kinodynamic/` — SRBD + joint-level kinematics (experimental).
  - `collaborative/` — adds a passive-arm disturbance model.
- `controllers/sampling/` — JAX sampling-based MPC (`centroidal_nmpc_jax.py`,
  `..._gait_adaptive.py`), rollouts of `centroidal_model_jax.py`. Selected via
  `mpc_params['type'] = 'sampling'`; method via `sampling_method`
  (`random_sampling`/`mppi`/`cem_mppi`) and `control_parametrization`.
  On the CPU-only path use the `no-cuda` pixi env (see below).
- `helpers/` — `PeriodicGaitGenerator`/`PeriodicGaitGeneratorJax` (contact
  schedule from gait phase/duty factor), `FootholdReferenceGenerator` (Raibert
  heuristic), `SwingTrajectoryController` (swing foot trajectory + PD),
  `TerrainEstimator`, `VisualFootholdAdaptation`, `VelocityModulator`,
  `EarlyStanceDetector`, `quadruped_utils.py` (`GaitType`, `LegsAttr`-adjacent
  helpers — the actual `LegsAttr` type comes from the `gym_quadruped` package).
- `acados/` — vendored acados source (its own git submodule); build artifacts
  live under `acados/build`, generated OCP C code under
  `controllers/gradient/<variant>/c_generated_code/`. Don't hand-edit
  generated code — it's regenerated from the CasADi model on next run.

### Controller-type dispatch (`mpc_params['type']`)

`SRBDControllerInterface.__init__`
(`external/Quadruped-PyMPC/quadruped_pympc/interfaces/srbd_controller_interface.py`)
picks the class with `if/elif` on `self.type` / `cfg.mpc_params['type']`:
`'nominal'` → `Acados_NMPC_Nominal` (or `Acados_NMPC_GaitAdaptive` if
`optimize_step_freq`), `'input_rates'`, `'lyapunov'`, `'kinodynamic'`,
`'sampling'` → their respective classes. When adding a new MPC variant, add a
branch here plus a new `controllers/gradient/<variant>/` (or `sampling/`) pair.

## Entry points to run things

- `external/Quadruped-PyMPC/simulation/simulation.py::run_simulation` —
  self-contained MuJoCo sim driving the wrapper above; the standard way to
  try a change end-to-end.
- `external/Quadruped-PyMPC/simulation/batched_simulations.py`,
  `external/Quadruped-PyMPC/simulation/generate_dataset.py` — batch
  evaluation / dataset generation variants.
- `external/Quadruped-PyMPC/ros2/run_controller.py`,
  `external/Quadruped-PyMPC/ros2/run_simulator.py` — ROS 2 real/sim glue
  (uses `dls2_interface` msgs under `ros2/msgs_ws`).

## Running it

This repo's workshop env sets up acados' shared libs and headless MuJoCo:

```bash
source .env.workshop && uv run --extra workshop jupyter lab   # notebooks
source .env.workshop && uv run python <script>                 # scripts
```
`.env.workshop` sets `ACADOS_SOURCE_DIR`, `LD_LIBRARY_PATH` (acados `lib/`),
`MUJOCO_GL=egl`. These must be set via shell `source`, not `os.environ` inside
an already-running process (acados' shared-library search happens at import).

If installing/building Quadruped-PyMPC standalone (fresh env, not needed for
this repo's existing `.venv`): see `external/Quadruped-PyMPC/README_install.md` —
Pixi or Conda, then build acados (`cmake .. && make install`) and
`pip install -e quadruped_pympc/acados/interfaces/acados_template`. Its
`pixi.lock` pins a `no-cuda` (default) and `cuda` environment for the JAX
sampling MPC.

Related tests: `tests/test_apply_pympc_preset.py`,
`tests/test_legged_control_benchmark.py`, `tests/test_legged_control_mujoco.py`,
fixture at `tests/fixtures/pympc_config_sample.py`.

## Before changing MPC behavior

1. Identify which `mpc_params['type']` you're affecting — a change in
   `nominal/` does not propagate to `input_rates/`, `lyapunov/`, etc.; they
   are separate model+OCP files, not subclasses that share logic.
2. Changing anything in a `centroidal_model_*.py` (CasADi symbolic model)
   requires the corresponding `centroidal_nmpc_*.py` to regenerate its acados
   OCP solver on next run (deletes/rewrites `c_generated_code/`) — expect the
   first run after a model change to be slow (solver codegen + compile).
3. Config changes belong in `external/Quadruped-PyMPC/quadruped_pympc/config.py`,
   not hardcoded in controllers — check whether a knob you want already
   exists there first (it's a large, fairly complete dict; skim it before
   adding a new field).
4. Validate with `simulation/simulation.py::run_simulation` before/after —
   compare GRF/torque plots, not just "it runs".

## Work logs (`agent_reports/`)

Log what you did to `agent_reports/` at the repo root:

- One subfolder per work group (a task/topic grouping, not one folder per
  session) — e.g. `agent_reports/<group-name>/`.
- Inside a group's folder, name files with a zero-padded sequence number
  plus a short slug: `01_xxx.md`, `02_yyy.md`, ... — number continues
  across sessions within the same group, so check the highest existing
  number in that folder before adding the next entry.
