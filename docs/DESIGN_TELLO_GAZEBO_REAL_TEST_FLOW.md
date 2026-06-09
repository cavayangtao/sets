# Tello ROS Test Flow Design (Gazebo Launch-Only Single-Target Obstacle Avoidance)

## 1. Goal
Build a staged Gazebo validation flow that does not depend on the two-waypoint mission script.

Only use:
1. launch/tello_planner.launch
2. scripts/tello_planner_node.py

Primary objectives:
1. Validate launch-only closed-loop control in simulation.
2. Validate runtime single-target update through ROS param + service.
3. Validate obstacle-avoidance behavior using stage3 planner config.

## 2. Scope and Assumptions
1. ROS1 Noetic environment is available.
2. This repository is built and runnable on target Linux host.
3. Gazebo simulator is started from ~/Downloads/sim_ws using hector_quadrotor_demo sim.launch.
4. Gazebo drone receives command on /cmd_vel.
5. Planner pose input must be geometry_msgs/PoseStamped.
6. External frame convention is x forward, y left, z up.
7. Default launch config policy_convergence_tello_stage3 includes obstacles.

Out of scope:
1. scripts/run_tello_two_waypoints_auto.py test flow.
2. Real hardware workflow.
3. Flight controller firmware tuning.

## 3. Topic and Service Contract
### 3.1 Gazebo Source Topics
1. Pose source topic is typically /pose.
2. Command topic is /cmd_vel (geometry_msgs/Twist).

### 3.2 Planner Interface Constraints
1. tello_planner_node.py subscribes to geometry_msgs/PoseStamped only.
2. tello_planner.launch defaults planner pose topic to /mocap/pose.
3. Runtime target update is done by:
   - rosparam set /tello_planner/target_pos "[x,y,z,...]"
   - rosservice call /tello_planner/update_target
4. target_pos must match planner state dimension (length 13 for current config).

Therefore:
1. If Gazebo /pose is already geometry_msgs/PoseStamped, relay /pose -> /mocap/pose.
2. If Gazebo /pose is another type (for example nav_msgs/Odometry), run an adapter that republishes PoseStamped to /mocap/pose.

### 3.3 Frame Convention
1. Keep launch params as:
   - mocap_frame=ENU
   - mocap_to_planner_rz_deg=0.0
   - mocap_to_planner_scale_xyz=[1,1,1]
2. Under ENU mode, planner converts z-up to internal z-down sign.
3. Single target in target_pos is interpreted in planner frame.
4. Compatibility conclusion:
   - For Gazebo world frame x-forward/y-left/z-up and body frame x-forward/y-left/z-up, current implementation is compatible with default launch settings.
   - No extra x/y axis swap is required.
   - Command mapping keeps body-frame semantics: [v_bx, v_by, v_bz, yaw_rate] -> [cmd_vel.linear.x, .linear.y, .linear.z, .angular.z].
5. Practical altitude sign rule: desired Gazebo altitude z=+h should be entered as target z=-h.

## 4. Architecture
Closed loop:
1. Pose source (/pose or adapted PoseStamped) -> /mocap/pose
2. tello_planner node -> /cmd_vel
3. Gazebo dynamics -> updated pose
4. Operator sets single target via /tello_planner/target_pos + /tello_planner/update_target

## 5. Staged Test Plan
## Stage A: Environment Precheck
Exit criteria:
1. ROS master starts.
2. Pose topic is alive and stable.
3. Planner services become available.

Commands (run after Stage B Terminal 1 starts Gazebo):
source /opt/ros/noetic/setup.bash
rostopic type /pose
rostopic hz /pose

## Stage B: Gazebo Launch-Only Single-Target Obstacle-Avoidance Test
Purpose:
1. Start Gazebo simulation from ~/Downloads/sim_ws.
2. First trigger takeoff with positive z-axis linear velocity.
3. After takeoff, raise the drone to 12 m.
3. Start planner from launch file.
4. Push one target online and verify obstacle-avoidance behavior.

Terminal 1 (start Gazebo simulation in sim_ws):
cd ~/Downloads/sim_ws
conda deactivate
source devel_isolated/setup.bash
roslaunch hector_quadrotor_demo sim.launch

Terminal 1b (mandatory: takeoff first, then climb to 12 m before planner test):
source /opt/ros/noetic/setup.bash

# Step 1: takeoff trigger (z-axis linear velocity up). Keep publishing until the drone clearly leaves ground.
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 1.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Step 2: continue climbing with z-axis command until altitude reaches about 12 m.
# If needed, keep the same command running, or re-run with a smaller z speed for smoother approach:
# rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.8}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

When altitude reaches about 12 m in Gazebo, press Ctrl+C to stop publishing,
then send one zero command:
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Terminal 2 (start planner from launch):
source /opt/ros/noetic/setup.bash
export SETS_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
export ROS_PACKAGE_PATH=/home/ros/Downloads:${ROS_PACKAGE_PATH:-}
roslaunch sets tello_planner.launch

Terminal 2b (pose wiring):
source /opt/ros/noetic/setup.bash
rostopic type /pose
rostopic hz /pose
rosrun topic_tools relay /pose /mocap/pose

If rostopic type /pose is not geometry_msgs/PoseStamped,
run an adapter node to convert to PoseStamped before continuing.

Terminal 3 (single-target command and observation):
source /opt/ros/noetic/setup.bash

# Optional: inspect current pose before choosing target
rostopic echo -n 1 /mocap/pose

# Set one target in planner frame (example goal with Gazebo altitude +12 m -> z=-12)
rosparam set /tello_planner/target_pos "[20.0, 0.0, -12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
rosservice call /tello_planner/update_target

# Observe planner state and command stream
rostopic echo /tello_planner/status
rostopic hz /cmd_vel

# Run convergence monitor and export trajectory CSV/PNG/JSON to plots/
/usr/bin/python3 /home/ros/Downloads/sets/scripts/monitor_single_target_convergence.py \
   --pose-topic /pose \
   --target 20,0,-12 \
   --timeout 180 \
   --reach-dist 2.5 \
   --hold-sec 2.0 \
   --config-name policy_convergence_tello_stage3 \
   --out-prefix /home/ros/Downloads/sets/plots/tello_single_target_roundX

Success criteria:
1. Drone has clearly taken off (left ground) before 12 m climb.
2. Drone is manually stabilized around 12 m altitude before planner target update.
3. /tello_planner/update_target returns success=true.
4. /tello_planner/status shows state:RUNNING and periodic planning updates.
5. /cmd_vel is active during transit.
6. monitor script reports success=true and writes trajectory plot to sets/plots/.
7. Vehicle converges near the requested single target while maintaining collision-free motion in the stage3 obstacle field.

## Stage C: Deferred (Real Hardware)
Real hardware flow is intentionally excluded in this revision.

## 6. Safety Operations
Immediate stop:
rosservice call /tello_planner/estop

Recover to READY:
rosservice call /tello_planner/arm "data: false"

## 7. Cleanup
pkill -f tello_planner_node.py || true
pkill -f "hector_quadrotor_demo sim.launch" || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f "rosmaster --core" || true
pkill -f rosout || true

## 8. Common Failure Cases
1. No pose stream:
   - Symptom: state remains INIT/READY, no effective planning progress.
   - Check: rostopic hz /pose and rostopic hz /mocap/pose.
2. Pose type mismatch:
   - Symptom: planner subscribes but receives no valid pose.
   - Check: rostopic type /pose and ensure PoseStamped reaches /mocap/pose.
3. Service not ready:
   - Symptom: cannot call /tello_planner/update_target.
   - Check: planner process and ROS_MASTER_URI consistency.
4. target_pos dimension mismatch:
   - Symptom: update_target returns failure with shape/state_dim error.
   - Check: provide 13 values in target_pos.
5. Wrong frame/sign:
   - Symptom: vehicle diverges or moves opposite in altitude.
   - Check: keep mocap_frame=ENU and use negative planner z for positive Gazebo altitude.
6. Timeout to FAILSAFE:
   - Symptom: cmd_vel drops to zero unexpectedly.
   - Check: pose_timeout and max_cmd_hold against real topic update rates.
7. Planner starts before 12 m initialization:
   - Symptom: early trajectory quality is poor or vehicle behavior is unstable at test start.
   - Check: complete the manual climb-to-12m step before calling /tello_planner/update_target.
8. No takeoff before climb command:
   - Symptom: vehicle cannot move or cannot continue climbing toward 12 m.
   - Check: first apply positive z-axis linear velocity until clear lift-off, then continue climb.
