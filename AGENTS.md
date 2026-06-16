# AGENTS

Compact guide for coding agents working in this repository.

## Scope
- ROS package `sets` for the SETS planner (Spectral Expansion Tree Search).
- Main upstream docs: [README.md](README.md), [package.xml](package.xml).
- Current implementation specs: [docs/spec-drone-planner-stanley-controller.md](docs/spec-drone-planner-stanley-controller.md), [docs/spec-drone-planner-test-requirements.md](docs/spec-drone-planner-test-requirements.md).

## Read First
1. [README.md](README.md) — paper citation, install, high-level build.
2. [src/bindings.cpp](src/bindings.cpp) — pybind entrypoint.
3. [scripts/drone_planner_node.py](scripts/drone_planner_node.py) — online ROS planner node.
4. [scripts/stanley_controller_node.py](scripts/stanley_controller_node.py) — trajectory tracking controller.
5. [launch/drone_planner.launch](launch/drone_planner.launch) — runtime wiring and default params.

## Repository Map
- `src/` — C++ planner/solver source, pybind module, and Python helpers (`plotter.py`, `util/`).
- `scripts/` — experiment scripts and ROS nodes.
- `configs/` — YAML configs; current active config is `configs/sixdofaircraft/policy_convergence_drone.yaml`.
- `launch/` — ROS launch files.
- `tests/` — ROS mission test runner (`run_drone_planner_test.py`).
- `docs/` — design and runbook docs.
- Generated artifacts: `plots/`, `data/` (kept by `.gitkeep`, contents ignored).

## Environment and Build

README install step:
```bash
conda env create --file environment.yml
conda activate sets
```

Caveat: `environment.yml` pins Python 3.7 and has a stale `prefix` line. The current workspace is actually running Python 3.12 from base conda, and `src/build/` contains bindings for both Python 3.7 and 3.12. Whichever interpreter you use, rebuild bindings so the generated `.so` matches it.

Build from `src/`:
```bash
cd src
mkdir -p build
cd build
cmake -DPYTHON_EXECUTABLE=$(which python) -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

The compiled module is imported as `build.bindings` after adding `src/` to `sys.path`. Rebuild after changes to `src/bindings.cpp`, `src/mdps/`, or `src/solvers/`.

## Primary Workflows

Offline paper-style plots (run from `scripts/`):
```bash
cd scripts
PYTHONPATH=../src python value_convergence.py
PYTHONPATH=../src python policy_convergence.py
```

ROS planner + Stanley controller:
```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
roslaunch sets drone_planner.launch
```

One-click two-waypoint regression test (start `roscore` first, then launch, then run):
```bash
source /opt/ros/noetic/setup.bash
export SETS_ROOT=/home/tyang/sets
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
python3 tests/run_drone_planner_test.py \
  --use-ros-test-node --sim-mode rollout \
  --wp0=-20,0,-120 --wp1=20,0,-120 \
  --reach-dist 8.0 --wp-timeout 300 \
  --config-name policy_convergence_drone \
  --out-prefix /home/tyang/sets/plots/drone_two_wp_test
```

Acceptance: `status=success` and `phase_min_dist.wp0`/`wp1` <= `reach_dist`.

## Conventions and Pitfalls

- **Import path inconsistency**: `policy_convergence.py`, `rollout.py`, and the ROS nodes add `../src` to `sys.path`, but `value_convergence.py` does not. Run it with `PYTHONPATH=../src` or after `conda develop`ing `src/`, otherwise `plotter`/`util`/`build` imports fail.
- **Config resolution is by basename**: `util.get_config_path()` searches `configs/` recursively for `<name>.yaml`. Keep config basenames unique (e.g. `policy_convergence_drone.yaml` vs `policy_convergence.yaml`).
- **ROS package discovery**: `roslaunch sets ...` requires `ROS_PACKAGE_PATH` to include the parent of this directory (`/home/tyang`), not the package directory itself.
- **Test runner path**: the mission test is `tests/run_drone_planner_test.py`; old docs may reference `test/` which no longer exists.
- **Virtual ROS mission mode**: prefer `--sim-mode rollout`; `cmd_dynamics` is less stable for regression checks.
- **Vertical control**: `launch/drone_planner.launch` sets `enable_vertical_control=false`; `Twist.linear.z` stays 0 in current runs.
- **Bindings compatibility**: some builds omit `Trajectory.vz_cmds`; use `getattr(traj, "vz_cmds", None)` fallbacks.
- **Frame convention**: planner uses NED z-down internally; launch defaults `mocap_frame=ENU` flip the z sign from the external pose source.

## Fast Validation

Bindings import smoke check:
```bash
cd scripts
python -c "import os,sys; sys.path.insert(0, os.path.join(os.getcwd(), '..', 'src')); from build.bindings import get_mdp; print('bindings ok')"
```

Python syntax smoke check:
```bash
cd scripts
python -m py_compile \
  rollout.py value_convergence.py policy_convergence.py \
  drone_planner_node.py stanley_controller_node.py
```

## Linked Docs
- ROS node architecture: [docs/spec-drone-planner-stanley-controller.md](docs/spec-drone-planner-stanley-controller.md)
- Mission test workflow: [docs/RUN_DRONE_OBS_TEST_SKILL.md](docs/RUN_DRONE_OBS_TEST_SKILL.md)
- Gazebo/real test flow: [docs/DESIGN_TELLO_GAZEBO_REAL_TEST_FLOW.md](docs/DESIGN_TELLO_GAZEBO_REAL_TEST_FLOW.md)
