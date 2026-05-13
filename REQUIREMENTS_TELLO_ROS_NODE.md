# Tello + MoCap Online Control Requirement Analysis (ROS Node)

## 1. Purpose
Analyze whether the current planning entry script can be transformed into a ROS node for real Tello flight with:
- Pose input from motion capture.
- Velocity command output through `cmd_vel` (`geometry_msgs/Twist`).

This document is analysis-only. No code rewrite is included.

## 2. Conclusion (Feasibility)
The conversion is feasible, but not as a direct drop-in of the current offline script.

Reason:
1. Current flow in [scripts/policy_convergence.py](scripts/policy_convergence.py) is experiment-oriented batch execution and plotting, not a continuous online ROS loop.
2. Core planner execution through [scripts/rollout.py](scripts/rollout.py) is simulation-oriented and blocking.
3. Real flight requires asynchronous ROS I/O, watchdog, command rate holding, and safety state machine.

Recommendation:
- Keep C++ planner bindings.
- Add a dedicated ROS runtime wrapper node (new script/module).
- Separate planner update rate from command publish rate.

## 3. Current-State Analysis
### 3.1 What exists today
1. Planner stack based on UCT-MPC with pybind bindings from `build.bindings`.
2. Stage config with control semantics already aligned to velocity command structure:
   - [configs/sixdofaircraft/policy_convergence_tello_stage3.yaml](configs/sixdofaircraft/policy_convergence_tello_stage3.yaml)
   - `ground_mdp_control_labels = [v_bx_cmd, v_by_cmd, v_bz_cmd, yaw_rate_cmd, ...]`
3. Trajectory export and plotting pipeline for offline analysis.

### 3.2 Gaps vs ROS online control
1. No ROS subscriber/publisher in current codebase.
2. No online state estimator bridge from MoCap pose to planner state vector.
3. No explicit frame convention contract (world/body, ENU/NED, yaw sign).
4. No runtime safety guards (timeout hover/stop/land).
5. Planner loop is blocking; command output requires deterministic periodic publication.

## 4. Assumptions
1. Tello ROS driver accepts `cmd_vel` as body-frame linear velocity + yaw rate.
2. MoCap provides at least position + orientation at stable rate (>= 30 Hz preferred).
3. ROS distribution can be ROS1 or ROS2; exact target must be fixed before implementation.

## 5. Functional Requirements
### FR-1 Node Runtime
The system shall run as one ROS node process (or coordinated two-node architecture) that continuously:
1. Subscribes to MoCap pose topic.
2. Maintains planner state.
3. Publishes `cmd_vel` at fixed control rate.

### FR-2 Input Interface
The node shall subscribe to pose topic with configurable message type and topic name.
Mandatory extracted fields:
1. Position (x, y, z).
2. Orientation (quaternion -> yaw at minimum).
Optional:
1. Velocity from MoCap.
2. If missing, derive velocity by filtered differentiation.

### FR-3 State Mapping
The node shall map MoCap/estimated state into planner state order consistent with:
- [configs/sixdofaircraft/policy_convergence_tello_stage3.yaml](configs/sixdofaircraft/policy_convergence_tello_stage3.yaml)
- `ground_mdp_state_labels`.

### FR-4 Command Output
The node shall publish `geometry_msgs/Twist` to `cmd_vel` using planner action components:
1. `linear.x <- v_bx_cmd`
2. `linear.y <- v_by_cmd`
3. `linear.z <- v_bz_cmd`
4. `angular.z <- yaw_rate_cmd`

### FR-5 Dual-Rate Control
The system shall support:
1. Planner update loop (e.g., 1-5 Hz depending on solve time).
2. Command publish loop (e.g., 20-50 Hz) that re-publishes latest valid command.

### FR-6 Safety State Machine
The node shall implement at least:
1. INIT (wait for valid pose stream).
2. READY (armed for planning).
3. RUNNING (publish planner commands).
4. FAILSAFE (publish zero/hover command on timeout/errors).

### FR-7 Timeout Handling
If pose timeout exceeds threshold (configurable, e.g., 0.2-0.5 s), node shall immediately enter FAILSAFE and stop planning.

### FR-8 Bounds and Saturation
Before publish, command shall be saturated by configured limits (`ground_mdp_U` or runtime limits).

### FR-9 Runtime Reconfiguration
Key parameters shall be externalized (YAML/ROS params):
1. topics
2. rates
3. frame settings
4. limits
5. fail-safe thresholds

### FR-10 Logging
The node shall log:
1. planner latency per cycle
2. command outputs
3. mode transitions
4. timeout and safety events

## 6. Non-Functional Requirements
### NFR-1 Latency
Planner cycle latency target should be monitored; if beyond threshold repeatedly, node shall degrade gracefully (hold last command / zero command strategy by policy).

### NFR-2 Determinism
Command publish loop jitter should be bounded and independent from planner spikes.

### NFR-3 Robustness
Any exception in planner step shall not crash command publisher without safety transition.

### NFR-4 Maintainability
Offline experiment script behavior in [scripts/policy_convergence.py](scripts/policy_convergence.py) should remain available for reproducibility.

## 7. Interface Requirements
### I/O Topics
1. Input pose topic: configurable.
2. Output command topic: `cmd_vel` (`geometry_msgs/Twist`).
3. Optional status topic: planner state and diagnostics.

### Service/Action (optional but recommended)
1. start/stop planner.
2. emergency stop.
3. set target waypoint.

## 8. Validation and Acceptance Criteria
### AC-1 Offline-in-the-loop
Using recorded MoCap data replay, node publishes bounded `cmd_vel` with no crashes for >= 10 minutes.

### AC-2 Online smoke test (propellers off / safe mode)
With live MoCap stream, mode transitions and timeout fail-safe trigger correctly.

### AC-3 Controlled flight test
In a netted/safe area:
1. stable command stream
2. no command spikes beyond limits
3. fail-safe verified by artificially dropping pose topic

### AC-4 Performance report
Provide measured:
1. planner cycle latency distribution
2. publish rate
3. timeout event count

## 9. Risks
1. Frame mismatch (MoCap world vs Tello body frame) may cause inverted control.
2. Planner compute time may exceed practical online budget.
3. Missing velocity estimate quality may destabilize behavior.
4. Driver-side `cmd_vel` semantics (units/frame) may differ by package version.

## 10. Suggested Implementation Phases (for next step)
1. Phase A: ROS skeleton + pose->state mapping + bounded zero/hover command.
2. Phase B: planner-in-loop single-step command generation.
3. Phase C: dual-rate scheduling + safety state machine.
4. Phase D: flight tuning and acceptance tests.