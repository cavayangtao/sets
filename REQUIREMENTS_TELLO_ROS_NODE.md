# Tello + MoCap Online Control Requirements (ROS Node)

## 1. Purpose
Define the requirement baseline aligned with the **current** ROS1 implementation in `scripts/tello_planner_node.py`.

System objective:
1. Subscribe to MoCap pose input.
2. Run UCT-MPC online planning.
3. Publish bounded velocity commands via `cmd_vel` (`geometry_msgs/Twist`).

## 2. Scope and Status
This document describes implemented requirements and current operational constraints.

Implemented runtime architecture:
1. Single ROS node with asynchronous subscriber/publisher/timers.
2. Dual-rate scheduling (planner timer + command publish timer).
3. Safety state machine and watchdog.
4. Runtime services: `~arm`, `~estop`, `~update_target`.

## 3. Assumptions
1. ROS distribution is ROS1 (`rospy`).
2. Tello driver consumes `cmd_vel` as body-frame velocity and yaw rate command.
3. Pose input is `geometry_msgs/PoseStamped`.
4. MoCap stream is sufficiently frequent for stable finite-difference velocity estimation.

## 4. Functional Requirements
### FR-1 Node Runtime
The node shall continuously:
1. Subscribe to MoCap pose topic.
2. Maintain planner state.
3. Publish `cmd_vel` at fixed rate.

### FR-2 Input Interface
The node shall support:
1. Configurable input pose topic name.
2. Pose message type parameter `pose_msg_type`, with current supported value `PoseStamped`.
3. Mandatory extraction of position `(x, y, z)` and orientation `(quaternion -> rpy/yaw)`.

Velocity handling in current implementation:
1. Optional use of estimated velocity via `use_estimated_velocity`.
2. When enabled, velocity is derived from finite-difference position and passed through first-order LPF.
3. LPF/estimation parameters are runtime configurable: `vel_lpf_tau`, `vel_diff_max_dt`.

### FR-3 State Mapping
The node shall map transformed pose/velocity into planner state order consistent with:
1. `configs/sixdofaircraft/policy_convergence_tello_stage3.yaml`
2. `ground_mdp_state_labels`

### FR-4 Command Output
The node shall publish planner action to `geometry_msgs/Twist` as:
1. `linear.x <- v_bx_cmd`
2. `linear.y <- v_by_cmd`
3. `linear.z <- v_bz_cmd`
4. `angular.z <- yaw_rate_cmd`

### FR-5 Dual-Rate Control
The node shall provide:
1. Planner loop at configurable low rate (`planner_rate`).
2. Command publish loop at configurable higher rate (`cmd_pub_rate`) that republishes latest command.

### FR-6 Safety State Machine
The node shall implement states:
1. `INIT`: waiting first valid pose.
2. `READY`: pose available, waiting arm/auto planning transition.
3. `RUNNING`: planner active + nonzero command eligible.
4. `FAILSAFE`: publish zero command.

### FR-7 Timeout Handling
Current behavior:
1. Pose timeout is monitored in `RUNNING` state.
2. Command-hold timeout is monitored in `RUNNING` state.
3. On timeout, node transitions to `FAILSAFE` and outputs zero command.

### FR-8 Bounds and Saturation
Before publish, command shall be saturated using:
1. YAML-derived limits from `ground_mdp_U` (default source of truth), or
2. Runtime limit overrides (`vx_max`, `vy_max`, `vz_max`, `yaw_rate_max`).

### FR-9 Runtime Reconfiguration
The following shall be exposed via ROS params:
1. topics and rates
2. planner hyper-parameters
3. frame transform settings
4. limits and safety thresholds
5. velocity estimation/filter parameters

### FR-10 Logging and Diagnostics
The node shall provide:
1. planner latency and success/failure indicators
2. state transitions and timeout/error logs
3. status heartbeat on `status_topic`

## 5. Non-Functional Requirements
### NFR-1 Latency
Planner cycle latency shall be observable and logged.

### NFR-2 Determinism
Command publish loop shall remain decoupled from planner spikes by timer-driven republish.

### NFR-3 Robustness
Planner exceptions shall not crash the command publisher loop. If planner stops refreshing commands, watchdog timeout shall drive `FAILSAFE`.

### NFR-4 Maintainability
Offline planning scripts and pybind planner stack shall remain usable for reproducibility and debugging.

## 6. Interface Requirements
### I/O Topics
1. Input pose topic: configurable (`pose_topic`).
2. Output command topic: `cmd_vel` (`geometry_msgs/Twist`).
3. Status topic: configurable (`status_topic`).

### Services
1. `~arm` (`std_srvs/SetBool`): arm/disarm planner runtime state.
2. `~estop` (`std_srvs/Trigger`): emergency transition to `FAILSAFE`.
3. `~update_target` (`std_srvs/Trigger`): apply `~target_pos` to MDP target.

## 7. Validation Criteria
### AC-1 Smoke and Safety
Node should satisfy:
1. state transitions work as designed.
2. nonzero command appears in `RUNNING` when planning succeeds.
3. `estop` forces zero command stream.

### AC-2 Two-Waypoint Runtime Mission
Using `scripts/run_tello_two_waypoints_auto.py` with ROS test node mode:
1. waypoint 0 and waypoint 1 are reached under configured threshold.
2. mission exits success and emits plot/GIF/report artifacts.

### AC-3 Timeout Safety
Pose drop or stale command update in `RUNNING` shall transition to `FAILSAFE`.

## 8. Known Risks
1. Frame mismatch may invert control signs.
2. UCT compute load may reduce effective planner update frequency.
3. Velocity estimate quality depends on MoCap timestamp quality and LPF tuning.
4. Driver-specific `cmd_vel` semantics may vary by package version.

## 9. Future Enhancements (Out of Current Scope)
1. Additional pose message types beyond `PoseStamped`.
2. Direct velocity ingestion from external estimator/IMU fusion.
3. Wider timeout policy coverage outside `RUNNING`.
4. Rich typed status topic (instead of string payload).