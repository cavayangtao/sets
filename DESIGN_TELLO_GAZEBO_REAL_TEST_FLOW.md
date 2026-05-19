# Tello ROS Test Flow Design (Gazebo-Only Validation, FLU/ENU Input)

## 1. Goal
Build a staged validation flow in Gazebo while reusing the same planner and mission runner:
1. scripts/tello_planner_node.py
2. scripts/run_tello_two_waypoints_auto.py

Primary objectives:
1. Validate topic contract and control loop safety in simulation first.
2. Keep command flow stable with explicit topic/frame conventions.
3. Keep mission artifacts (png, gif, report json) for traceability.

## 2. Scope and Assumptions
1. ROS1 Noetic environment is available.
2. This repository is built and runnable on target Linux host.
3. Gazebo drone receives control on `/cmd_vel`.
4. External frames (world + drone body) are right-handed and follow `x` forward, `y` left, `z` up.
5. Pose must be provided to planner as `geometry_msgs/PoseStamped`.

Out of scope:
1. Flight controller firmware tuning.
2. Real hardware testing workflow (deferred for this document).

## 3. Topic Contract
### 3.1 Gazebo Source Topics
1. Pose source topic is typically `/pose`.
2. Command topic: `/cmd_vel` (`geometry_msgs/Twist`).

### 3.2 Current Node Interface Constraint
1. `tello_planner_node.py` subscribes to `geometry_msgs/PoseStamped`.
2. `run_tello_two_waypoints_auto.py` also subscribes to `geometry_msgs/PoseStamped` for position tracking.
3. `tello_planner.launch` sets planner pose topic to `/mocap/pose` by default.

Therefore:
1. If Gazebo `/pose` is already `geometry_msgs/PoseStamped`, relay it to `/mocap/pose`.
2. If Gazebo `/pose` is another type (for example `nav_msgs/Odometry`), add an adapter node that republishes `PoseStamped` to `/mocap/pose`.

### 3.3 Frame Convention
1. External Gazebo convention (input): `x` forward, `y` left, `z` up.
2. Planner runtime keeps `_mocap_frame:=ENU`, `_mocap_to_planner_rz_deg:=0.0`, `_mocap_to_planner_scale_xyz:=[1,1,1]`.
3. Under `_mocap_frame:=ENU`, planner internally flips `z` sign (up -> down) and applies corresponding attitude sign mapping.
4. Mission waypoints in `run_tello_two_waypoints_auto.py` are interpreted in planner frame.
5. Practical rule: desired Gazebo altitude `z=+h` should be entered as waypoint `z=-h`.

## 4. Architecture
Closed loop:
1. Pose source (`/pose` or adapted PoseStamped topic) -> planner node
2. Planner node -> cmd_vel
3. Vehicle dynamics (Gazebo model) -> updated pose
4. Mission runner controls targets through /tello_planner/update_target and /tello_planner/arm

## 5. Staged Test Plan
## Stage A: Environment Precheck
Exit criteria:
1. ROS master starts.
2. Pose topic is alive and stable.
3. Planner services are available.

Commands:
source /opt/ros/noetic/setup.bash
export SETS_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
roscore -p 11312

In another terminal:
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
rostopic type /pose
rostopic hz /pose

## Stage B: Gazebo Functional Mission (Recommended First)
Purpose:
1. Verify end-to-end mission success in a safe closed loop.
2. Validate frame consistency and command sign before real flight.

Terminal 1:
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
roscore -p 11312

Terminal 2:
source /opt/ros/noetic/setup.bash
export SETS_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
export ROS_PACKAGE_PATH=/home/tyang:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
roslaunch sets tello_planner.launch

Terminal 2b (pose wiring when Gazebo publishes `/pose` PoseStamped):
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
rostopic type /pose
rosrun topic_tools relay /pose /mocap/pose

If `rostopic type /pose` is not `geometry_msgs/PoseStamped` (for example `nav_msgs/Odometry`),
run an adapter node that republishes `PoseStamped` to `/mocap/pose` before starting mission.

Terminal 3:
source /opt/ros/noetic/setup.bash
export SETS_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
python3 "$SETS_ROOT/scripts/run_tello_two_waypoints_auto.py" \
   --pose-topic /mocap/pose \
   --cmd-topic /cmd_vel \
  --wp0 2,0,-1.2 \
  --wp1 2,1,-1.2 \
  --reach-dist 0.5 \
  --wp-timeout 40 \
  --pose-wait-timeout 10 \
  --out-prefix "$SETS_ROOT/plots/tello_two_wp_gazebo_01"

Waypoint note:
1. `z=-1.2` in command corresponds to Gazebo altitude `z=+1.2`.

Success criteria:
1. Exit code 0.
2. Logs show Waypoint 0 reached and Waypoint 1 reached.
3. Artifact files generated:
   - tello_two_wp_gazebo_01.png
   - tello_two_wp_gazebo_01.gif
   - tello_two_wp_gazebo_01_report.json

## Stage C: Deferred (Real Hardware)
Real hardware flow is intentionally excluded in this revision. This document focuses on Gazebo-only validation.

## 6. Safety Operations
Immediate stop:
rosservice call /tello_planner/estop

Recover to READY:
rosservice call /tello_planner/arm "data: false"

## 7. Cleanup
pkill -f tello_planner_node.py || true
pkill -f "roscore -p 11312" || true
pkill -f "rosmaster --core -p 11312" || true
pkill -f rosout || true

## 8. Common Failure Cases
1. No pose stream:
   - Symptom: mission waits for first pose and times out.
   - Check: rostopic hz POSE_TOPIC.
2. Service not ready:
   - Symptom: cannot reach /tello_planner/arm.
   - Check: planner process and ROS_MASTER_URI consistency.
3. Message type mismatch on pose topic:
   - Symptom: planner node fails to subscribe or receives no valid pose.
   - Check: rostopic type /pose and ensure PoseStamped is provided to planner.
4. Wrong frame/sign:
   - Symptom: diverging motion or oscillation.
   - Check: keep `_mocap_frame:=ENU`, verify external frame is `x` forward/`y` left/`z` up, and confirm waypoint altitude uses negative planner `z`.
5. Timeout to FAILSAFE:
   - Symptom: command drops to zero unexpectedly.
   - Check: pose_timeout/max_cmd_hold against actual update rates.
