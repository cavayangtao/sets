---
name: run-tello-two-waypoints-test
description: 'Test scripts/run_tello_two_waypoints_auto.py only. Use for two-waypoint mission validation with external pose/cmd topics (gazebo or real) or internal ros-test-node mode.'
argument-hint: 'mode=external|internal pose_topic=... cmd_topic=... wp0=... wp1=...'
user-invocable: true
---

# run_tello_two_waypoints_auto.py Test Skill

## Scope
This skill is only for testing:
1. scripts/run_tello_two_waypoints_auto.py

It does not cover:
1. policy_convergence tuning
2. planner algorithm development
3. mcts build/debug workflow

## Prerequisites
1. ROS Noetic is available and sourced.
2. Conda sets environment is available.
3. ROS master is running.
4. Planner node is running and services exist:
   - /tello_planner/arm
   - /tello_planner/update_target

## Common Environment Block
source ~/anaconda3/etc/profile.d/conda.sh
conda activate sets
source /opt/ros/noetic/setup.bash
export SETS_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1

## Quick Service Check
rosservice list | grep /tello_planner/arm
rosservice list | grep /tello_planner/update_target

## Recommended S3 Planner Startup (No Manual Tuning Args)
The launch defaults are synchronized to the S3 smoothness set:
1. uct_N=10000
2. uct_max_depth=10
3. uct_c=0.7
4. uct_wct=2200.0
5. uct_mpc_depth=2
6. pose_timeout=8.0
7. max_cmd_hold=12.0
8. planner_rate=1.0
9. cmd_pub_rate=30.0

Start planner:
roslaunch sets tello_planner.launch

Optional seed override:
roslaunch sets tello_planner.launch seed:=3

## Mode A: External Pose/Cmd Topics (Gazebo or Real)
Use this when pose and cmd topics are provided by simulator or hardware.

export POSE_TOPIC=/pose
export CMD_TOPIC=/cmd_vel
python3 "$SETS_ROOT/scripts/run_tello_two_waypoints_auto.py" \
  --pose-topic "$POSE_TOPIC" \
  --cmd-topic "$CMD_TOPIC" \
  --wp0 2,0,-1.2 \
  --wp1 2,1,-1.2 \
  --reach-dist 0.5 \
  --wp-timeout 40 \
  --pose-wait-timeout 10 \
  --out-prefix "$SETS_ROOT/plots/tello_two_wp_external_01"

## Mode B: Internal ROS Test Node
Use this when you want deterministic virtual regression.

python3 "$SETS_ROOT/scripts/run_tello_two_waypoints_auto.py" \
  --use-ros-test-node \
  --sim-mode rollout \
  --sim-rate 50 \
  --wp0 20,0,-12 \
  --wp1 20,10,-12 \
  --reach-dist 2.0 \
  --wp-timeout 150 \
  --out-prefix "$SETS_ROOT/plots/tello_two_wp_s3_default"

## S3 Stability Benchmark (Multi-Seed)
Run 10-seed benchmark (seed 0..9) and auto print success rate + mean/variance:

bash "$SETS_ROOT/scripts/benchmark_stability.sh" 0 9

Outputs:
1. Per-seed reports: $SETS_ROOT/plots/tello_two_wp_s3_seed<seed>_report.json
2. Aggregated CSV:  $SETS_ROOT/plots/tello_two_wp_s3_seed_stats.csv

## Expected Outputs
1. <out-prefix>.png
2. <out-prefix>.gif
3. <out-prefix>_report.json

## Safety
Emergency stop:
rosservice call /tello_planner/estop

Reset to READY:
rosservice call /tello_planner/arm "data: false"

## Cleanup
pkill -f tello_planner_node.py || true
pkill -f "roscore -p 11312" || true
pkill -f "rosmaster --core -p 11312" || true
pkill -f rosout || true
