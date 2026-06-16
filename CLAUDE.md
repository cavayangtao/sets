# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

SETS (Spectral Expansion Tree Search) planner — ROS 1 Noetic package for online drone trajectory planning via UCT-MPC + Stanley controller. Upstream: `aerorobotics/sets`; fork: `cavayangtao/sets`.

See [AGENTS.md](AGENTS.md) for detailed build/test/run commands and conventions. This file covers architecture patterns and design decisions that span multiple files.

## Architecture

```
roscore
├── drone_planner_node.py          (UCT-MPC planner, 1 Hz)
│   in:  /pose (PoseStamped)
│   out: /planner/trajectory (Path), /planner/vz_cmd (Float64)
│
└── stanley_controller_node.py    (Stanley tracker, 30 Hz)
    in:  /pose, /planner/trajectory
    out: /cmd_vel (Twist)
```

Planner uses C++ pybind11 bindings (`src/build/` ← `src/bindings.cpp`). Controller is pure Python with an optional cubic spline interpolator (`cubic_spline_planner.py`). See `docs/spec-drone-planner-stanley-controller.md` for full topic/param specification.

## Environment (required before any ROS command)

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
```

`SETS_ROOT=/home/tyang/sets` is also commonly exported.

## Controller Path-Following Parameters

The controller has two major configurable behaviors not present in the upstream version:

| Param (`~` prefix) | Type | Code default | Launch default | Effect |
|---|---|---|---|---|
| `use_spline_interpolation` | bool | `true` | `false` | When false, track raw planner waypoints directly (no cubic spline). Path direction computed via `atan2(dy, dx)` between consecutive waypoints. |
| `monotonic_target_index` | bool | `true` | `false` | When false, allows the target index along the path to move backward. Disabling eliminates end-of-path heading oscillation ("hesitation"). |

Both are set in `launch/drone_planner.launch`. The `cmd_smoothing_alpha` parameter and the first-order low-pass filter on `/cmd_vel` were intentionally removed (commented-out code was dead; it added latency without measurable benefit).

## Real-Time Visualization Module

`scripts/stanley_visualizer.py` — optional live matplotlib animation window, toggled by `~enable_visualization` (default `false`). Key design points:

- **Thread model**: All matplotlib GUI operations (import, figure creation, `FuncAnimation`, `plt.show()`) run in a single daemon thread (`_gui_main`). The controller main thread writes into a shared dict protected by `threading.Lock`.
- **Backend**: `TkAgg` (explicit). `blit=False` is required because the adaptive view lock calls `set_xlim`/`set_ylim` 1–2 times during a mission; with `blit=True` + TkAgg + `ax.axis("equal")`, matplotlib does not reliably re-capture the background on axes changes, causing static patches (obstacles) to disappear.
- **View locking**: The view locks once at mission start (based on start + target + obstacle extents) and expands only if the target moves outside the current viewport. The full trajectory history is excluded from view computation so a distant start position does not shrink the flight corridor to unreadable scale.
- **Obstacle loading**: Loads from planner YAML config via `util.get_obstacles(config, 0)`. Drawn as semi-transparent red `Rectangle` patches.
- **Path rendering**: Raw waypoints = red dashed (`r--`); spline points = darkred solid. When `use_spline_interpolation=false`, the controller clears `spline_x`/`spline_y` in the shared buffer so only the dashed line is visible.
- **Initial default target filter**: The view lock defers until `|tgt - cur| < 200 m` — filters out the planner's distant default target `(120, 120)` from the launch file.

Full spec: `docs/spec-stanley-controller-viz.md`.

## Test Framework Limitation

`tests/run_drone_planner_test.py --sim-mode rollout` uses the test node's **internal UCT planner** to drive the drone pose — not the controller's `/cmd_vel`. The controller and planner nodes do run and their outputs are recorded, but the controller's velocity commands do not directly move the simulated drone in rollout mode.

The alternative `--sim-mode cmd_dynamics` applies `/cmd_vel` through MDP dynamics, **but it is incompatible** with the current controller: the controller outputs velocity `Twist(linear.x=0.2)`, while `SixDOFAircraft.mdp.F(state, u)` expects `u = [thrust_z, tau_x, tau_y, tau_z]`. The format mismatch causes `is_state_valid()` to reject every state update, freezing the drone at its initial position.

To validate controller behavior end-to-end, run on Gazebo or real hardware rather than the internal test node.

## Frame Convention

Planner uses NED (z-down). Launch defaults `mocap_frame=ENU` with `mocap_to_planner_rz_deg=0` and unit scale, so `x_NED = x_ENU`, `y_NED = y_ENU`, `z_NED = -z_ENU`. Target waypoints are specified in planner (NED) frame: a target at z=-120 in NED corresponds to z=+120 in ENU/Gazebo.

## Cleanup

Between test runs, kill stale processes:
```bash
pkill -f "run_drone_planner_test.py|drone_planner_node.py|stanley_controller_node.py|roslaunch|roscore|rosmaster|rosout" || true
```

## Key Documentation

- [docs/spec-drone-planner-stanley-controller.md](docs/spec-drone-planner-stanley-controller.md) — node/param spec
- [docs/spec-stanley-controller-viz.md](docs/spec-stanley-controller-viz.md) — visualization spec
- [docs/RUN_DRONE_OBS_TEST_SKILL.md](docs/RUN_DRONE_OBS_TEST_SKILL.md) — test runbook
- [docs/DESIGN_TELLO_GAZEBO_REAL_TEST_FLOW.md](docs/DESIGN_TELLO_GAZEBO_REAL_TEST_FLOW.md) — Gazebo test design
