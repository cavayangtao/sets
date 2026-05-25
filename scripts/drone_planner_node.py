#!/usr/bin/env python3
"""
drone_planner_node.py — ROS node for online drone trajectory planning via UCT-MPC.

Uses policy_convergence_drone.yaml (2.0 kg quadrotor, 8D thrust+torque control).
Publishes planned trajectory as nav_msgs/Path for a downstream controller.
Designed for Gazebo simulation (no hardware failsafe).

Architecture:
  - Imports C++ bindings directly (get_mdp, get_dots_mdp, get_uct2, run_uct2).
  - Creates MDP/DOTS/UCT objects once at init; reuses them per cycle.
  - Single timer: planner runs MPC, publishes trajectory + control sequence.
  - State machine: INIT -> READY -> RUNNING (no FAILSAFE for simulation).

Input:  simulation pose     (geometry_msgs/PoseStamped)
Output: trajectory          (nav_msgs/Path)
        control sequence    (std_msgs/Float64MultiArray)
"""

import sys
import os
import time
import math
import ast
import threading
import traceback
from enum import Enum

import numpy as np

# -- project imports --
_project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(_project_root, "src"))
sys.path.insert(0, _project_root)
from util import util
from build.bindings import get_mdp, get_dots_mdp, get_uct2, RNG, run_uct2

# -- ROS imports (rospy / ROS 1) --
try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
    from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
except ImportError:
    rospy = None


class State(Enum):
    INIT    = "INIT"
    READY   = "READY"
    RUNNING = "RUNNING"


def quaternion_to_rpy(q):
    """Convert geometry_msgs/Quaternion to (roll, pitch, yaw) in radians."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    """Convert (roll, pitch, yaw) to (x, y, z, w) quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def transform_pose(px, py, pz, roll, pitch, yaw, frame_config):
    """Apply configurable frame transform (ENU -> NED for planner)."""
    scale = np.array(frame_config.get("mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]))
    rz_deg = frame_config.get("mocap_to_planner_rz_deg", 0.0)

    px, py, pz = px * scale[0], py * scale[1], pz * scale[2]

    if abs(rz_deg) > 1e-9:
        rad = math.radians(rz_deg)
        c, s = math.cos(rad), math.sin(rad)
        px, py = c * px - s * py, s * px + c * py
        yaw = yaw + rad

    mocap_frame = frame_config.get("mocap_frame", "ENU")
    if mocap_frame == "ENU":
        pz = -pz
        pitch = -pitch
        yaw = -yaw
    elif mocap_frame == "NED_ZY":
        px, py, pz = px, pz, py
        roll, pitch, yaw = roll, yaw, pitch

    return px, py, pz, roll, pitch, yaw


def transform_velocity(vx, vy, vz, frame_config):
    """Apply frame convention to velocity (ENU -> NED for planner)."""
    scale = np.array(frame_config.get("mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]))
    rz_deg = frame_config.get("mocap_to_planner_rz_deg", 0.0)

    vx, vy, vz = vx * scale[0], vy * scale[1], vz * scale[2]

    if abs(rz_deg) > 1e-9:
        rad = math.radians(rz_deg)
        c, s = math.cos(rad), math.sin(rad)
        vx, vy = c * vx - s * vy, s * vx + c * vy

    mocap_frame = frame_config.get("mocap_frame", "ENU")
    if mocap_frame == "ENU":
        vz = -vz
    elif mocap_frame == "NED_ZY":
        vx, vy, vz = vx, vz, vy

    return vx, vy, vz


class DronePlannerNode:
    def __init__(self):
        if rospy is None:
            raise ImportError("rospy is not available. Install ROS 1 (rospy).")
        rospy.init_node("drone_planner", anonymous=False)
        self._load_params()
        self._init_planner()

        self._lock = threading.Lock()
        self._state = State.INIT

        self._mocap_pos = None
        self._mocap_quat = None
        self._mocap_stamp = None

        self._prev_pos = None
        self._prev_stamp = None
        self._est_vel = np.zeros(3)

        self._planning_active = False
        self._plan_success = False
        self._plan_latency = 0.0
        self._plan_count = 0
        self._timestep = 0

        self._setup_ros()
        rospy.loginfo("DronePlannerNode initialized. State: %s", self._state.value)

    def _load_params(self):
        self._config_name = rospy.get_param("~config_name", "policy_convergence_drone")
        self._config_path = util.get_config_path(self._config_name)
        self._config = util.load_yaml(self._config_path)

        self._pose_topic = rospy.get_param("~pose_topic", "/pose")
        self._trajectory_topic = rospy.get_param("~trajectory_topic", "/planner/trajectory")
        self._control_seq_topic = rospy.get_param("~control_seq_topic", "/planner/control_seq")
        self._trajectory_frame = rospy.get_param("~trajectory_frame", "world")
        self._status_topic = rospy.get_param("~status_topic", "/drone_planner/status")

        self._planner_rate = rospy.get_param("~planner_rate", 2.0)

        self._uct_N = rospy.get_param("~uct_N", self._config.get("uct_N", 500))
        self._uct_max_depth = rospy.get_param("~uct_max_depth", self._config.get("uct_max_depth", 12))
        self._uct_c = rospy.get_param("~uct_c", self._config.get("uct_c", 3.0))
        self._uct_wct = rospy.get_param("~uct_wct", self._config.get("uct_wct", 1200.0))
        self._uct_mpc_depth = rospy.get_param("~uct_mpc_depth", self._config.get("uct_mpc_depth", 2))
        self._dots_decision_making_horizon = rospy.get_param(
            "~dots_decision_making_horizon", self._config.get("dots_decision_making_horizon", 60))
        self._dots_dynamics_horizon = rospy.get_param(
            "~dots_dynamics_horizon", self._config.get("dots_dynamics_horizon", 20))
        self._uct_dt = rospy.get_param("~uct_dt", self._config.get("uct_dt", 0.01))

        self._auto_arm = rospy.get_param("~auto_arm", True)
        self._pose_timeout = rospy.get_param("~pose_timeout", 4.0)
        self._use_estimated_velocity = rospy.get_param("~use_estimated_velocity", True)
        self._vel_lpf_tau = rospy.get_param("~vel_lpf_tau", 0.15)
        self._vel_diff_max_dt = rospy.get_param("~vel_diff_max_dt", 0.5)

        _scale_xyz = rospy.get_param("~mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0])
        if isinstance(_scale_xyz, str):
            try:
                parsed = ast.literal_eval(_scale_xyz)
                if isinstance(parsed, (list, tuple, np.ndarray)):
                    _scale_xyz = list(parsed)
                else:
                    raise ValueError("scale_xyz string did not parse to sequence")
            except Exception as e:
                rospy.logwarn("Failed to parse scale_xyz: %s", e)
                _scale_xyz = [1.0, 1.0, 1.0]
        self._frame_config = {
            "mocap_frame": rospy.get_param("~mocap_frame", "ENU"),
            "mocap_to_planner_rz_deg": rospy.get_param("~mocap_to_planner_rz_deg", 0.0),
            "mocap_to_planner_scale_xyz": _scale_xyz,
        }

        self._target_pos = rospy.get_param("~target_pos",
            self._config.get("ground_mdp_xd", [120.0, 120.0, -120.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        if isinstance(self._target_pos, str):
            try:
                parsed = ast.literal_eval(self._target_pos)
                if isinstance(parsed, (list, tuple, np.ndarray)):
                    self._target_pos = list(parsed)
            except Exception as e:
                rospy.logwarn("Failed to parse target_pos string: %s", e)

        self._seed = rospy.get_param("~seed", 0)

        self._trajectory_max_points = rospy.get_param("~trajectory_max_points", 200)

        rospy.loginfo("Params: config=%s, N=%d, planner_rate=%.1f Hz",
                      self._config_name, self._uct_N, self._planner_rate)

    def _init_planner(self):
        config = self._config
        config_path = self._config_path

        self._ground_mdp = get_mdp(config["ground_mdp_name"], config_path)
        self._ground_mdp.set_dt(self._uct_dt)

        self._dots_mdp = get_dots_mdp()
        self._dots_mdp.set_param(
            self._ground_mdp,
            config["dots_expansion_mode"],
            config["dots_initialize_mode"],
            config["dots_special_actions"],
            config["dots_num_branches"],
            self._dots_decision_making_horizon,
            self._dots_dynamics_horizon,
            config["dots_spectral_branches_mode"],
            config["dots_control_mode"],
            config["dots_scale_mode"],
            config["dots_modal_damping_mode"],
            config["dots_modal_damping_gains"],
            np.diag(np.array(config["dots_rho"])),
            config["dots_greedy_gain"],
            config["dots_greedy_rate"],
            config["dots_greedy_min_dist"],
            config["dots_baseline_mode_on"],
            config["dots_num_discrete_actions"],
            config["dots_verbose"],
        )

        self._uct = get_uct2()
        self._uct.set_param(
            self._uct_N,
            self._uct_max_depth,
            self._uct_wct,
            self._uct_c,
            False, False, False, False, False,
            config["uct_heuristic_mode"],
            config["uct_tree_exploration"],
            config["uct_downsample_traj_on"],
            False,
        )

        self._rng = RNG()
        self._rng.set_seed(self._seed)

        obstacles = util.get_obstacles(config, 0)
        self._ground_mdp.clear_obstacles()
        for obs in obstacles:
            self._ground_mdp.add_obstacle(obs)

        try:
            xd = np.array(self._target_pos, dtype=np.float64).reshape(-1, 1)
            if xd.shape[0] != self._ground_mdp.state_dim():
                raise ValueError(
                    f"target_pos length {xd.shape[0]} != state_dim {self._ground_mdp.state_dim()}")
            self._ground_mdp.set_xd(xd)
        except Exception as e:
            rospy.logwarn("Invalid startup target_pos, fallback to YAML: %s", e)
            fallback = np.array(config.get("ground_mdp_xd", []), dtype=np.float64).reshape(-1, 1)
            if fallback.shape[0] == self._ground_mdp.state_dim():
                self._ground_mdp.set_xd(fallback)
                self._target_pos = fallback.reshape(-1).tolist()

        rospy.loginfo("Planner objects created: MDP state_dim=%d, N=%d, obstacles=%d",
                      self._ground_mdp.state_dim(), self._uct_N, len(obstacles))

    def _setup_ros(self):
        self._pose_sub = rospy.Subscriber(
            self._pose_topic, PoseStamped, self._pose_callback, queue_size=1)

        self._traj_pub = rospy.Publisher(self._trajectory_topic, RosPath, queue_size=1)
        self._ctrl_pub = rospy.Publisher(self._control_seq_topic, Float64MultiArray, queue_size=1)
        self._status_pub = rospy.Publisher(self._status_topic, String, queue_size=10)

        self._planner_timer = rospy.Timer(
            rospy.Duration(1.0 / self._planner_rate), self._planner_callback)

        self._arm_srv = rospy.Service("~arm", SetBool, self._arm_callback)
        self._estop_srv = rospy.Service("~estop", Trigger, self._estop_callback)
        self._update_target_srv = rospy.Service("~update_target", Trigger, self._update_target_callback)

    def _pose_callback(self, msg):
        with self._lock:
            self._mocap_pos = (msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z)
            self._mocap_quat = msg.pose.orientation
            self._mocap_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

            now = self._mocap_stamp.to_sec()
            pos = np.array(self._mocap_pos)
            if self._prev_pos is not None and self._prev_stamp is not None:
                dt = now - self._prev_stamp
                if 1e-6 < dt <= self._vel_diff_max_dt:
                    raw_vel = (pos - self._prev_pos) / dt
                    if self._vel_lpf_tau > 1e-9:
                        alpha = float(np.clip(dt / (self._vel_lpf_tau + dt), 0.0, 1.0))
                        self._est_vel = self._est_vel + alpha * (raw_vel - self._est_vel)
                    else:
                        self._est_vel = raw_vel
                elif dt > self._vel_diff_max_dt:
                    rospy.logwarn_throttle(5.0,
                        "Pose dt too large (dt=%.3fs); reset est vel", dt)
                    self._est_vel = np.zeros(3)
            self._prev_pos = pos
            self._prev_stamp = now

            if self._state == State.INIT:
                self._state = State.READY
                rospy.loginfo("First pose received -> READY")

    def _planner_callback(self, event):
        if self._state not in (State.READY, State.RUNNING):
            return

        if self._state == State.READY and not self._auto_arm:
            return

        if self._planning_active:
            rospy.logwarn_throttle(5.0, "Planner cycle skipped: previous still running")
            return

        self._planning_active = True
        t_start = time.time()
        try:
            with self._lock:
                pos = self._mocap_pos
                quat = self._mocap_quat
                est_vel = self._est_vel.copy()

            if pos is None or quat is None:
                rospy.logwarn_throttle(5.0, "No mocap data available for planning")
                return

            r, p, y = quaternion_to_rpy(quat)
            px, py, pz, roll, pitch, yaw = transform_pose(
                pos[0], pos[1], pos[2], r, p, y, self._frame_config)
            if self._use_estimated_velocity:
                vx, vy, vz = transform_velocity(
                    est_vel[0], est_vel[1], est_vel[2], self._frame_config)
            else:
                vx, vy, vz = 0.0, 0.0, 0.0

            state = np.zeros(self._ground_mdp.state_dim(), dtype=np.float64)
            state[0]  = px
            state[1]  = py
            state[2]  = pz
            state[3]  = vx
            state[4]  = vy
            state[5]  = vz
            state[6]  = roll
            state[7]  = pitch
            state[8]  = yaw
            state[9]  = 0.0
            state[10] = 0.0
            state[11] = 0.0
            state[12] = float(self._timestep)

            if not self._ground_mdp.is_state_valid(state):
                rospy.logwarn("State invalid: %s", np.array2string(state, precision=2))
                return

            result = run_uct2(self._dots_mdp, self._uct, state, self._rng)
            t_elapsed = time.time() - t_start

            with self._lock:
                self._plan_latency = t_elapsed
                self._plan_count += 1

                if result.success:
                    if self._state not in (State.READY, State.RUNNING):
                        self._plan_success = False
                        return

                    xs = np.array(result.planned_traj.xs)
                    us = np.array(result.planned_traj.us)

                    self._publish_trajectory(xs)
                    self._publish_control_sequence(us)

                    self._plan_success = True
                    self._timestep += 1
                    if self._state == State.READY:
                        self._state = State.RUNNING
                        rospy.loginfo("First plan succeeded (auto-arm) -> RUNNING")
                else:
                    self._plan_success = False
                    rospy.logwarn("Planner returned success=False (plan #%d, %.3fs)",
                                  self._plan_count, t_elapsed)

        except Exception:
            rospy.logerr("Planner exception:\n%s", traceback.format_exc())
            with self._lock:
                self._plan_success = False
        finally:
            self._planning_active = False

    def _publish_trajectory(self, xs):
        """Convert state trajectory matrix (T x 13) to nav_msgs/Path and publish.

        Extracts position (indices 0,1,2) and yaw (index 8).
        Converts z from NED (z-down) back to ENU (z-up) for the output message.
        """
        path_msg = RosPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self._trajectory_frame

        num_steps = xs.shape[0]
        step = max(1, num_steps // self._trajectory_max_points)

        for i in range(0, num_steps, step):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = self._trajectory_frame
            pose.pose.position.x = xs[i, 0]
            pose.pose.position.y = xs[i, 1]
            pose.pose.position.z = -xs[i, 2]  # NED -> ENU
            qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, xs[i, 8])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self._traj_pub.publish(path_msg)

    def _publish_control_sequence(self, us):
        """Extract active controls (indices 3,4,5,6) from control trajectory and publish.

        Control layout: [unused_0, unused_1, unused_2, thrust_z, tau_x, tau_y, tau_z, unused_7]
        Only the 4 active controls (thrust_z, tau_x, tau_y, tau_z) are published.
        """
        num_steps = us.shape[0]
        active = us[:, 3:7]
        flat = active.reshape(-1).tolist()

        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="steps",  size=num_steps, stride=num_steps * 4))
        msg.layout.dim.append(MultiArrayDimension(label="controls", size=4,          stride=4))
        msg.layout.data_offset = 0
        msg.data = flat

        self._ctrl_pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = (
            f"state:{self._state.value}"
            f"|plan_count:{self._plan_count}"
            f"|plan_latency_ms:{self._plan_latency * 1000:.0f}"
            f"|plan_success:{self._plan_success}"
            f"|timestep:{self._timestep}"
        )
        self._status_pub.publish(msg)

    def _arm_callback(self, req):
        with self._lock:
            if req.data:
                if self._state == State.READY:
                    self._state = State.RUNNING
                    return SetBoolResponse(success=True, message="Armed -> RUNNING")
                else:
                    return SetBoolResponse(success=False,
                        message=f"Cannot arm from state {self._state.value}")
            else:
                self._state = State.READY
                self._timestep = 0
                return SetBoolResponse(success=True, message="Disarmed -> READY")

    def _estop_callback(self, req):
        with self._lock:
            self._state = State.INIT
            self._timestep = 0
        rospy.logwarn("ESTOP triggered -> INIT")
        return TriggerResponse(success=True, message="ESTOP: state -> INIT")

    def _update_target_callback(self, req):
        new_target = rospy.get_param("~target_pos", None)
        if new_target is None:
            return TriggerResponse(success=False,
                message="~target_pos param not set.")
        if isinstance(new_target, str):
            try:
                parsed = ast.literal_eval(new_target)
                if isinstance(parsed, (list, tuple, np.ndarray)):
                    new_target = list(parsed)
            except Exception as e:
                return TriggerResponse(success=False,
                    message=f"Failed to parse target_pos: {e}")
        try:
            xd = np.array(new_target, dtype=np.float64).reshape(-1, 1)
            with self._lock:
                self._ground_mdp.set_xd(xd)
                self._target_pos = new_target
            rospy.loginfo("Target updated to [%.1f, %.1f, %.1f]",
                          xd[0, 0], xd[1, 0], xd[2, 0])
            return TriggerResponse(success=True,
                message=f"Target set to {new_target[:3]}")
        except Exception as e:
            rospy.logerr("Failed to update target: %s", e)
            return TriggerResponse(success=False, message=str(e))

    def spin(self):
        rospy.loginfo("DronePlannerNode spinning. State: %s", self._state.value)
        rospy.spin()


if __name__ == "__main__":
    if rospy is not None:
        try:
            node = DronePlannerNode()
            node.spin()
        except rospy.ROSInterruptException:
            pass
    else:
        print("rospy not available. Install ROS 1 (rospy).")
        sys.exit(1)
