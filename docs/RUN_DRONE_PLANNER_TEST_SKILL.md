---
name: run-drone-planner-test
description: 'Test test/run_drone_planner_test.py. Use for two-waypoint mission validation of drone_planner_node + stanley_controller_node with external pose/cmd topics or internal ros-test-node mode.'
argument-hint: 'mode=external|internal pose_topic=... cmd_topic=... wp0=... wp1=...'
user-invocable: true
---

# run_drone_planner_test.py Test Skill

## Scope
This skill is only for testing:
1. test/run_drone_planner_test.py

The test validates the full data pipeline:
- drone_planner_node (UCT-MPC) → /planner/trajectory (Path)
- stanley_controller_node (Stanley) → /cmd_vel (Twist)

It does not cover:
1. policy_convergence tuning
2. planner algorithm development
3. mcts build/debug workflow
4. Gazebo simulation testing

## Prerequisites
1. ROS Noetic is available and sourced.
2. Conda sets environment is available.
3. ROS master is running.
4. Planner and controller nodes are running and services exist:
   - /drone_planner/arm
   - /drone_planner/update_target

## Common Environment Block
```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate sets
source /opt/ros/noetic/setup.bash
export SETS_ROOT="$(cd "$(dirname "$0")" && pwd)"
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
```

## Quick Service Check
```bash
rosservice list | grep /drone_planner/arm
rosservice list | grep /drone_planner/update_target
```

## Recommended Planner Startup (No Manual Tuning Args)
Start both drone_planner and stanley_controller with defaults:
```bash
roslaunch sets drone_planner.launch
```

Optional overrides for faster planning (default is 0.5 Hz):
```bash
roslaunch sets drone_planner.launch uct_N:=1000 planner_rate:=1.0 trajectory_timeout:=1.5
```

Optional seed override:
```bash
roslaunch sets drone_planner.launch uct_N:=1000 seed:=3
```

## Mode A: External Pose/Cmd Topics (Gazebo or Real)
Use this when pose and cmd topics are provided by simulator or hardware.

```bash
export POSE_TOPIC=/pose
export CMD_TOPIC=/cmd_vel
python3 "$SETS_ROOT/test/run_drone_planner_test.py" \
  --pose-topic "$POSE_TOPIC" \
  --cmd-topic "$CMD_TOPIC" \
  --wp0=-20,0,-120 \
  --wp1=20,0,-120 \
  --reach-dist 2.0 \
  --wp-timeout 300 \
  --pose-wait-timeout 20 \
  --out-prefix "$SETS_ROOT/plots/drone_two_wp_external_01"
```

## Mode B: Internal ROS Test Node (Recommended)
Use this for deterministic validation without Gazebo. The built-in
DroneRosTestPoseNode simulates the drone pose stream using UCT rollout states
from the same MDP model as the planner.

```bash
python3 "$SETS_ROOT/test/run_drone_planner_test.py" \
  --use-ros-test-node \
  --sim-mode rollout \
  --sim-rate 50 \
  --wp0=-20,0,-120 \
  --wp1=20,0,-120 \
  --reach-dist 2.0 \
  --wp-timeout 300 \
  --out-prefix "$SETS_ROOT/plots/drone_two_wp_internal_01"
```

Custom seed and UCT params:
```bash
# Start planner with custom seed
roslaunch sets drone_planner.launch uct_N:=1000 seed:=7

# Run test
python3 "$SETS_ROOT/test/run_drone_planner_test.py" \
  --use-ros-test-node \
  --sim-mode rollout \
  --sim-rate 50 \
  --wp0=-20,0,-120 \
  --wp1=20,0,-120 \
  --reach-dist 2.0 \
  --wp-timeout 300 \
  --out-prefix "$SETS_ROOT/plots/drone_two_wp_seed7"
```

## Expected Outputs
1. `<out-prefix>.png` — 2×2 subplot: top-down trajectory, altitude, x/y vs time, /cmd_vel history
2. `<out-prefix>.gif` — Trajectory animation with obstacle overlay
3. `<out-prefix>_report.json` — Metrics including:
   - final_pos, final_dist_wp0, final_dist_wp1
   - min_dist_wp0, min_dist_wp1
   - path_len, path_efficiency, total_turn_rad
   - cmd_delta_rms, cmd_delta_max
   - min_obstacle_clearance
   - trajectory_msg_count, cmd_nonzero_ratio
   - duration_s, num_samples

## Safety
Emergency stop:
```bash
rosservice call /drone_planner/estop
```

Reset to READY:
```bash
rosservice call /drone_planner/arm "data: false"
```

## Cleanup
```bash
pkill -f drone_planner_node.py || true
pkill -f stanley_controller_node.py || true
pkill -f "roscore -p 11311" || true
pkill -f rosout || true
```
