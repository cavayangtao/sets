# SPEC: Drone Planner + Stanley Controller (Implementation-Aligned)

> Source of truth: current code in `scripts/drone_planner_node.py`, `scripts/stanley_controller_node.py`, and `launch/drone_planner.launch`.
> Last aligned: 2026-06-09

## 1. Scope

This document describes the behavior that is actually implemented now, not the earlier target design.

- Node A: `drone_planner` (UCT-MPC planner)
- Node B: `stanley_controller` (Stanley lateral + decoupled vertical control)
- Launch entry: `launch/drone_planner.launch`

## 2. Runtime Architecture

- Input from simulation:
  - `/pose` (`geometry_msgs/PoseStamped`)
- Planner outputs:
  - `/planner/trajectory` (`nav_msgs/Path`)
  - `/planner/control_seq` (`std_msgs/Float64MultiArray`)
  - `/planner/vz_cmd` (`std_msgs/Float64`)
  - `/planner/vz_cmd_seq` (`std_msgs/Float64MultiArray`)
  - `/drone_planner/status` (`std_msgs/String`)
- Controller output:
  - `/cmd_vel` (`geometry_msgs/Twist`)

## 3. Planner Node (`drone_planner`)

### 3.1 Core Flow

1. Load config by `~config_name` (default `policy_convergence_drone`).
2. Initialize MDP, DOTS, UCT, RNG once at startup.
3. Subscribe `/pose`, estimate velocity by finite difference + LPF.
4. Convert pose/velocity from mocap frame to planner frame (default ENU -> NED).
5. Build 13D planner state and validate by `is_state_valid`.
6. Optionally apply online obstacle side-bias to target (runtime target only).
7. Run `run_uct2(...)`.
8. On success:
   - publish `Path` from planned states
   - publish active controls (indices 3..6)
   - publish vertical command scalar and sequence

### 3.2 State & Control Mapping

State (`13D`):

- `0..2`: position (planner frame)
- `3..5`: velocity (estimated, planner frame)
- `6..8`: roll/pitch/yaw
- `9..11`: angular rates (currently set to 0)
- `12`: timestep counter

Control (`8D` from planner):

- Published active channels are `us[:, 3:7]`
- Semantic order: `thrust_z, tau_x, tau_y, tau_z`

### 3.3 Path Conversion

For each planned state row `xs[i]`:

- `x = xs[i,0]`
- `y = xs[i,1]`
- `z = -xs[i,2]` (NED -> ENU)
- yaw quaternion from `xs[i,8]`

Trajectory is uniformly downsampled to at most `~trajectory_max_points`.

### 3.4 Vertical Command Publishing

- `~vz_cmd_topic` (`/planner/vz_cmd`): first vertical command of sequence
- `~vz_cmd_seq_topic` (`/planner/vz_cmd_seq`): full sequence
- If trajectory has no `vz_cmds` field in bindings, fallback uses `us[:,2]`

### 3.5 State Machine and Services

State machine:

- `INIT -> READY` when first pose arrives
- `READY -> RUNNING` on first successful plan when `~auto_arm=true`

Services:

- `~arm` (`std_srvs/SetBool`): arm/disarm planner state
- `~estop` (`std_srvs/Trigger`): force `INIT`
- `~update_target` (`std_srvs/Trigger`): reload `~target_pos`

### 3.6 Planner Parameters (actual defaults in code)

- `~config_name`: `policy_convergence_drone`
- `~planner_rate`: `2.0`
- `~pose_timeout`: `4.0`
- `~auto_arm`: `true`
- `~use_estimated_velocity`: `true`
- `~vel_lpf_tau`: `0.15`
- `~vel_diff_max_dt`: `0.5`
- `~trajectory_frame`: `world`
- `~trajectory_max_points`: `200`
- `~seed`: `0`
- `~mocap_frame`: `ENU`
- `~mocap_to_planner_rz_deg`: `0.0`
- `~mocap_to_planner_scale_xyz`: `[1.0, 1.0, 1.0]`
- `~target_pos`: from config `ground_mdp_xd` unless overridden

UCT / DOTS params loaded from config and may be overridden:

- `~uct_N` (launch default 500)
- `~uct_max_depth` (launch default 12)
- `~uct_c` (launch default 3.0)
- `~uct_wct` (launch default 1200.0)
- `~uct_mpc_depth`
- `~uct_dt` (launch sets 0.01)
- `~dots_decision_making_horizon`
- `~dots_dynamics_horizon`

Obstacle side-bias params in planner:

- `~enable_obstacle_side_bias` (code default true, launch sets false)
- `~obstacle_side_bias` (default 5.0, launch 2.0)
- `~obstacle_bias_sign` (default 1.0)
- `~obstacle_bias_trigger_margin` (default 10.0)

## 4. Stanley Controller Node (`stanley_controller`)

### 4.1 Core Flow

1. Subscribe planned path and pose.
2. Convert path waypoints into spline (`cx, cy, cyaw`) and interpolated `cz`.
3. At fixed timer rate, compute lateral command by Stanley law.
4. Compute vertical command with decoupled altitude controller + optional feedforward.
5. Apply first-order output smoothing and publish `/cmd_vel`.

### 4.2 Trajectory Handling

- Requires at least 2 points to build/refresh spline.
- Near-duplicate waypoints are removed by `~min_waypoint_separation`.
- If a short invalid path arrives while an old spline exists, old spline is kept.

### 4.3 Timeout Behavior (implemented)

Not immediate hard-stop by default:

1. If no spline exists: publish zero command.
2. If `dt <= trajectory_timeout`: normal tracking.
3. If `trajectory_timeout < dt <= trajectory_timeout + timeout_hold_max`:
   - hold previous command and exponentially decay by `timeout_hold_decay`.
4. If beyond hold window: hard-stop (zero command).

### 4.4 Lateral Control

- Stanley steering:
  - front-axle target index
  - heading error + cross-track term
- Saturation:
  - steer clipped by `~max_steer`
  - yaw rate clipped by `~max_w`
- Optional obstacle yaw bias is added before clip.
- Goal behavior:
  - optional slowdown by distance to final waypoint
  - near-goal stop zone by `~goal_stop_distance`

### 4.5 Vertical Control

- Optional feedforward from `/planner/vz_cmd` (with timeout check).
- Feedback: `vz = kp_z * (z_ref - z_meas)` with deadband.
- Bounds and saturation:
  - validity check vs `~z_bounds`
  - command clip by `~vz_limit`
- Altitude hold on invalid input:
  - if enabled, keep last valid vertical command until input recovers.

### 4.6 Output Smoothing

Published command uses first-order smoothing:

- `cmd = (1-a) * cmd_prev + a * cmd_raw`
- applied to `linear.x`, `angular.z`, `linear.z`
- `a = ~cmd_smoothing_alpha`

### 4.7 Controller Parameters (actual defaults in code)

Base Stanley:

- `~k`: `0.5`
- `~Kp`: `1.0` (reserved)
- `~L`: `0.2`
- `~max_steer`: `0.5236` rad
- `~target_velocity`: `0.2`
- `~max_w`: `0.5`
- `~factor_v`: `1.0`
- `~factor_w`: `1.0`
- `~spline_ds`: `0.01`
- `~control_rate`: `30.0`
- `~trajectory_timeout`: `0.5` (launch sets `3.0`)

Vertical control:

- `~enable_lateral_control`: `true`
- `~enable_vertical_control`: `true`
- `~enable_altitude_hold`: `true`
- `~kp_z`: `1.0`
- `~z_deadband`: `0.05`
- `~vz_limit`: `[-2.0, 2.0]`
- `~z_bounds`: `[-500.0, 500.0]`
- `~use_vz_ff_from_topic`: `true`
- `~vz_cmd_topic`: `/planner/vz_cmd`
- `~vz_cmd_timeout`: `1.0`
- `~vz_estimation_alpha`: `0.25`

Stability/smoothing:

- `~goal_slowdown_distance`: `0.0` (launch `8.0`)
- `~goal_stop_distance`: `0.6` (launch `1.0`)
- `~min_forward_speed`: `0.03` (launch `0.04`)
- `~cmd_smoothing_alpha`: `0.35`
- `~yaw_rate_deadband`: `0.01`
- `~timeout_hold_max`: `2.0`
- `~timeout_hold_decay`: `0.985`
- `~min_waypoint_separation`: `0.05` (launch `0.01`)

Obstacle side-bias (controller):

- `~enable_obstacle_side_bias`: `true`
- `~obstacle_side_preference`: `1.0`
- `~obstacle_influence_margin`: `6.0`
- `~obstacle_repulsion_gain`: `0.10` (launch `0.06`)
- `~obstacle_side_bias_gain`: `0.08` (launch `0.04`)
- `~obstacle_max_bias_w`: `0.20` (launch `0.12`)
- `~obstacle_release_margin`: `8.0`
- `~obstacle_center_epsilon`: `0.20`

## 5. Launch Defaults (drone_planner.launch)

Implemented launch file sets:

- Planner:
  - `planner_rate=2.0`
  - `uct_N=500`
  - `trajectory_topic=/planner/trajectory`
  - `control_seq_topic=/planner/control_seq`
  - `vz_cmd_topic=/planner/vz_cmd`
  - `vz_cmd_seq_topic=/planner/vz_cmd_seq`
  - planner obstacle side-bias disabled (`enable_obstacle_side_bias=false`)

- Controller:
  - `control_rate=30.0`
  - `trajectory_timeout=3.0`
  - vertical control enabled
  - obstacle side-bias enabled

## 6. Known Behavior Notes

- Planner and controller both include obstacle-side bias mechanisms; planner side-bias is disabled by launch default, controller side-bias is enabled by launch default.
- Controller timeout policy is soft-hold + decay before hard-stop, which differs from "timeout then immediate zero" behavior in the old spec.
- Vertical channel is fully integrated in controller (`Twist.linear.z`) and planner publishes matching feedforward topics.
