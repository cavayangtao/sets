---
name: run-drone-obs-test-current
description: Run the current two-waypoint drone planner test and generate PNG/GIF/JSON artifacts.
argument-hint: out_prefix=/home/tyang/sets/plots/<name>
user-invocable: true
---

# RUN_DRONE_OBS_TEST_SKILL

## Scope

This runbook executes the current validated flow:

1. Start ROS master.
2. Start `drone_planner` + `stanley_controller` via launch.
3. Run `test/run_drone_planner_test.py` in internal rollout mode.
4. Export PNG/GIF/JSON artifacts.

Out of scope:

1. Rebuilding C++ bindings.
2. Gazebo external mode.
3. Algorithm retuning.

## Current baseline assumptions

Current implementation/launch baseline:

1. Planner obstacle side-bias mechanism removed.
2. Controller obstacle yaw-bias mechanism removed.
3. `planner_rate=1.0` (launch default).
4. `enable_vertical_control=false` (launch default).
5. Test threshold settings: `reach-dist=8.0`, `wp-timeout=300`.

## Common environment

```bash
source /opt/ros/noetic/setup.bash
export SETS_ROOT=/home/tyang/sets
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
```

## Step 0: Clean stale processes

```bash
pkill -f "run_drone_planner_test.py|drone_planner_node.py|stanley_controller_node.py|roslaunch|roscore|rosmaster|rosout" || true
```

## Step 1: Start ROS master

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
roscore
```

## Step 2: Start planner + controller

```bash
source /opt/ros/noetic/setup.bash
export SETS_ROOT=/home/tyang/sets
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
roslaunch sets drone_planner.launch
```

## Step 3: Run mission test and generate artifacts

```bash
source /opt/ros/noetic/setup.bash
export SETS_ROOT=/home/tyang/sets
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

python3 /home/tyang/sets/test/run_drone_planner_test.py \
  --use-ros-test-node \
  --sim-mode rollout \
  --wp0=-20,0,-120 \
  --wp1=20,0,-120 \
  --reach-dist 8.0 \
  --wp-timeout 300 \
  --pose-wait-timeout 20 \
  --config-name policy_convergence_drone \
  --out-prefix /home/tyang/sets/plots/drone_obs_test_rerun_<tag>
```

Example verified prefix:

`/home/tyang/sets/plots/drone_obs_test_rerun_20260612_4_no_bias`

## Step 4: Validate outputs

```bash
ls -lh /home/tyang/sets/plots/drone_obs_test_rerun_<tag>.png \
       /home/tyang/sets/plots/drone_obs_test_rerun_<tag>.gif \
       /home/tyang/sets/plots/drone_obs_test_rerun_<tag>_report.json

cat /home/tyang/sets/plots/drone_obs_test_rerun_<tag>_report.json
```

Expected pass condition:

1. `status` is `success`.
2. `phase_min_dist.wp0 <= 8.0`.
3. `phase_min_dist.wp1 <= 8.0`.

## Cleanup

```bash
pkill -f "run_drone_planner_test.py|drone_planner_node.py|stanley_controller_node.py|roslaunch|roscore|rosmaster|rosout" || true
```