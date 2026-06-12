# SPEC: Drone Planner + Stanley Controller (Current Implementation)

Last aligned: 2026-06-12

## 1. Scope

This document is aligned to the current code and launch defaults in:

1. `scripts/drone_planner_node.py`
2. `scripts/stanley_controller_node.py`
3. `launch/drone_planner.launch`

It describes implemented behavior only.

## 2. Runtime Graph

Input:

1. `/pose` (`geometry_msgs/PoseStamped`)

Planner outputs:

1. `/planner/trajectory` (`nav_msgs/Path`)
2. `/planner/control_seq` (`std_msgs/Float64MultiArray`)
3. `/planner/vz_cmd` (`std_msgs/Float64`)
4. `/planner/vz_cmd_seq` (`std_msgs/Float64MultiArray`)
5. `/drone_planner/status` (`std_msgs/String`)

Controller output:

1. `/cmd_vel` (`geometry_msgs/Twist`)

## 3. Planner Node (`drone_planner`)

## 3.1 Core flow

1. Load config (`~config_name`, default `policy_convergence_drone`).
2. Build MDP/DOTS/UCT/RNG once.
3. Subscribe pose, estimate velocity by finite difference + LPF.
4. Transform ENU mocap state to planner frame (default ENU -> NED).
5. Build state vector and validate with `is_state_valid`.
6. Set planner target directly from `/drone_planner/target_pos`.
7. Run `run_uct2(...)` and publish trajectory/control sequence.

## 3.2 Obstacle bias status

1. Planner-side obstacle target bias has been removed.
2. No online target y-shift is applied anymore.
3. Obstacle avoidance relies on planner dynamics/costs and obstacle constraints.

## 3.3 Key parameters

Code defaults:

1. `~planner_rate=2.0`
2. `~pose_timeout=4.0`
3. `~auto_arm=true`
4. `~use_estimated_velocity=true`
5. `~vel_lpf_tau=0.15`
6. `~vel_diff_max_dt=0.5`
7. `~trajectory_max_points=200`
8. `~seed=0`

Launch overrides currently in use:

1. `planner_rate=1.0`
2. `pose_timeout=4.0`
3. `vel_lpf_tau=0.15`
4. `vel_diff_max_dt=0.8`
5. `trajectory_max_points=200`
6. `seed=0`

## 4. Controller Node (`stanley_controller`)

## 4.1 Core flow

1. Consume trajectory + pose.
2. Build spline (`cx`, `cy`, `cyaw`, `cz`) from `Path`.
3. Compute lateral command with Stanley law.
4. Compute vertical command only when vertical control is enabled.
5. Publish `/cmd_vel`.

## 4.2 Timeout behavior

Implemented timeout handling is not immediate stop:

1. `dt <= trajectory_timeout`: normal track.
2. `trajectory_timeout < dt <= trajectory_timeout + timeout_hold_max`: keep previous command and exponentially decay.
3. Beyond hold window: stop command.

## 4.3 Obstacle bias and smoothing status

1. Controller-side obstacle yaw bias mechanism has been removed.
2. The previous first-order output LPF block remains commented out (not active).
3. Yaw deadband remains active (`yaw_rate_deadband`) to suppress tiny oscillation.

## 4.4 Vertical channel status

1. Vertical logic exists in code.
2. Current launch default sets `enable_vertical_control=false`.
3. Therefore `Twist.linear.z` is held at 0.0 in current runs.

## 4.5 Key parameters

Code defaults:

1. `~k=0.5`, `~L=0.2`, `~target_velocity=0.2`
2. `~trajectory_timeout=0.5`
3. `~control_rate=30.0`
4. `~enable_vertical_control=true`
5. `~goal_slowdown_distance=0.0`
6. `~goal_stop_distance=0.6`
7. `~min_forward_speed=0.03`
8. `~timeout_hold_max=2.0`
9. `~timeout_hold_decay=0.985`
10. `~min_waypoint_separation=0.05`

Launch overrides currently in use:

1. `control_rate=30.0`
2. `trajectory_timeout=3.0`
3. `enable_vertical_control=false`
4. `goal_slowdown_distance=8.0`
5. `goal_stop_distance=1.0`
6. `min_forward_speed=0.04`
7. `timeout_hold_max=2.0`
8. `timeout_hold_decay=0.985`
9. `min_waypoint_separation=0.01`

## 5. Launch Baseline (`drone_planner.launch`)

Current launch baseline:

Planner:

1. `config_name=policy_convergence_drone`
2. `uct_N=500`, `uct_max_depth=12`, `uct_c=3.0`, `uct_wct=1200.0`
3. `planner_rate=1.0`

Controller:

1. `target_velocity=0.2`
2. `trajectory_timeout=3.0`
3. `enable_vertical_control=false`

Obstacle bias:

1. No planner-side bias params.
2. No controller-side bias params.

## 6. Known Runtime Notes

1. When bindings miss `traj.vz_cmds`, planner logs fallback warning and uses available channels.
2. If pose stream ages beyond `pose_timeout`, planner skips planning cycles.
3. If trajectory stream stalls long enough, controller transitions to stop after hold-decay window.