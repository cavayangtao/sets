# Drone Planner Test Requirements (Current Baseline)

Last aligned: 2026-06-12

## 1. Goal

Define the current, executable test baseline for `test/run_drone_planner_test.py`.

This baseline targets:

1. Reproducible two-waypoint mission execution.
2. Deterministic artifact generation (PNG/GIF/JSON).
3. Pass/fail decision by waypoint reach threshold.

## 2. Scope

In scope:

1. Internal ROS test node mode (`--use-ros-test-node --sim-mode rollout`).
2. Planner + controller launched by `roslaunch sets drone_planner.launch`.
3. Output validation from JSON report.

Out of scope:

1. Gazebo external mode.
2. Hardware-in-the-loop.
3. Legacy obstacle side-bias behavior (removed from current implementation).

## 3. Current Runtime Baseline

Current launch baseline (`launch/drone_planner.launch`):

1. Planner: `config_name=policy_convergence_drone`.
2. Planner: `planner_rate=1.0`, `uct_N=500`, `uct_max_depth=12`, `uct_c=3.0`, `uct_wct=1200.0`.
3. Controller: `control_rate=30.0`, `trajectory_timeout=3.0`.
4. Controller: `enable_vertical_control=false`.
5. Obstacle side-bias params: not present in launch and corresponding code path removed.

## 4. Canonical Test Command

```bash
python3 /home/tyang/sets/test/run_drone_planner_test.py \
  --use-ros-test-node \
  --sim-mode rollout \
  --wp0=-20,0,-120 \
  --wp1=20,0,-120 \
  --reach-dist 8.0 \
  --wp-timeout 300 \
  --pose-wait-timeout 20 \
  --config-name policy_convergence_drone \
  --out-prefix /home/tyang/sets/plots/drone_obs_test_<tag>
```

Required ROS environment:

```bash
source /opt/ros/noetic/setup.bash
export SETS_ROOT=/home/tyang/sets
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
```

## 5. Acceptance Criteria

A run is PASS when all conditions hold:

1. Report field `status` is `success`.
2. `phase_min_dist.wp0 <= reach_dist`.
3. `phase_min_dist.wp1 <= reach_dist`.
4. Artifact files exist:
   - `<out-prefix>.png`
   - `<out-prefix>.gif`
   - `<out-prefix>_report.json`

A run is FAIL when any condition is not met, including `status=failed` or timeout waiting waypoint.

## 6. Report Contract

JSON report must include at least:

1. `status`
2. `error`
3. `wp0`, `wp1`
4. `reach_dist`
5. `phase_min_dist`
6. `metrics`

Important metrics currently used for regression observation:

1. `final_dist_wp1`
2. `min_dist_wp1`
3. `trajectory_msg_count`
4. `trajectory_msg_ratio`
5. `cmd_nonzero_ratio`
6. `duration_s`

## 7. Recent Verified Reference Run

Reference run tag: `drone_obs_test_rerun_20260612_4_no_bias`

Observed result:

1. `status=success`
2. `phase_min_dist.wp0=7.560880828083984`
3. `phase_min_dist.wp1=7.9909610850927315`
4. `final_dist_wp1=7.578073590556946`
5. `reach_dist=8.0`

## 8. Failure Triage Priority

When failing, inspect in order:

1. ROS process health (`roscore`, `roslaunch`, test runner lifecycle).
2. Pose freshness (`pose_timeout` warnings in planner log).
3. Trajectory continuity (`trajectory_timeout` warnings in controller log).
4. Waypoint convergence (`phase_min_dist` and `final_dist_wp1` in report).