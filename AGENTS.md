# AGENTS

This file gives coding agents the fastest reliable path to build, run, and validate changes in this repository.

## Scope
- Root project: [sets](.)
- Main upstream context and baseline commands: [README.md](README.md)
- ROS package metadata: [package.xml](package.xml)

## Read First
1. [README.md](README.md)
2. [src/bindings.cpp](src/bindings.cpp)
3. [scripts/policy_convergence.py](scripts/policy_convergence.py)
4. [scripts/drone_planner_node.py](scripts/drone_planner_node.py)
5. [docs/DESIGN_TELLO_ROS_NODE.md](docs/DESIGN_TELLO_ROS_NODE.md)

## Repository Map
- Core planner and pybind module: [src](src)
- Experiment and ROS runtime scripts: [scripts](scripts)
- Configs: [configs](configs)
- ROS launch files: [launch](launch)
- Validation entry points: [test](test), [tests](tests)
- Generated artifacts: [plots](plots), [src/build](src/build)

## Environment And Build
Create and activate env:
```bash
conda env create --file environment.yml
conda activate sets
```

Build bindings from [src](src):
```bash
cd src
mkdir -p build
cd build
cmake -DPYTHON_EXECUTABLE=$(which python) -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

Rebuild bindings after changes under [src](src), especially [src/bindings.cpp](src/bindings.cpp), [src/mdps](src/mdps), and [src/solvers](src/solvers).

## Primary Workflows
Offline paper-style plots:
```bash
cd scripts
python value_convergence.py
python policy_convergence.py
```

ROS planner/controller integration:
```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
roslaunch sets drone_planner.launch
```

One-click two-waypoint validation:
```bash
python3 test/run_drone_planner_test.py --use-ros-test-node --sim-mode rollout
```

## Conventions And Pitfalls
- Most Python entrypoints rely on adding [src](src) to `sys.path` and importing `build.bindings`; preserve this pattern unless you update all callers.
- Run plot/analysis scripts from [scripts](scripts) so relative `../plots` and `../data` style paths resolve correctly.
- Config resolution uses config basename matching in [src/util/util.py](src/util/util.py#L150); keep config names unique across [configs](configs).
- ROS launch in this workspace requires `ROS_PACKAGE_PATH` to include `/home/tyang` (parent of the package directory).
- For virtual ROS mission tests, prefer `--use-ros-test-node --sim-mode rollout`; `cmd_dynamics` mode is less stable for regression checks.
- In [scripts/tello_planner_node.py](scripts/tello_planner_node.py), parse `ground_mdp_U` as row-major `[u_min(8), u_max(8)]`; do not apply Fortran-order reshape.
- `Trajectory.vz_cmds` may be absent in some bindings builds; use `getattr(traj, "vz_cmds", None)` style fallback patterns.

## Fast Validation
Python syntax smoke check:
```bash
cd scripts
python -m py_compile rollout.py value_convergence.py policy_convergence.py drone_planner_node.py stanley_controller_node.py
```

Bindings import smoke check:
```bash
cd scripts
python -c "import os,sys; sys.path.insert(0, os.path.join(os.getcwd(), '..', 'src')); from build.bindings import get_mdp; print('bindings ok')"
```

ROS regression example:
```bash
python3 tests/test_lateral_regression.py
```

## Linked Design And Test Docs
- ROS node architecture: [docs/DESIGN_TELLO_ROS_NODE.md](docs/DESIGN_TELLO_ROS_NODE.md)
- Gazebo/real test flow: [docs/DESIGN_TELLO_GAZEBO_REAL_TEST_FLOW.md](docs/DESIGN_TELLO_GAZEBO_REAL_TEST_FLOW.md)
- Mission test workflow: [docs/RUN_DRONE_PLANNER_TEST_SKILL.md](docs/RUN_DRONE_PLANNER_TEST_SKILL.md)
- Planner test requirements: [docs/spec-drone-planner-test-requirements.md](docs/spec-drone-planner-test-requirements.md)
