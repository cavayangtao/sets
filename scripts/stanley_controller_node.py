#!/usr/bin/env python3
"""
stanley_controller_node.py — Stanley lateral controller for trajectory tracking.

Receives planned trajectory (nav_msgs/Path) from drone_planner_node and
current pose (PoseStamped).  Uses cubic spline interpolation on the path
waypoints, then applies the Stanley control law to generate /cmd_vel.

Key features:
  - Cubic spline interpolation for smooth path following
  - Trajectory timeout: immediately stops if no new path received within
    trajectory_timeout seconds
  - All control parameters configurable via ROS param server
  - Timer-driven control loop at configurable rate (default 30 Hz)
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import os
import ast

# Import cubic spline planner from same directory
_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)
import cubic_spline_planner


class State(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0, v=0.0, vz=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.v = v
        self.vz = vz


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


def calc_target_index(state, cx, cy, L):
    """Compute the index of the closest point on the path to the front axle."""
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def stanley_control(state, cx, cy, cyaw, last_target_idx, k, L):
    """Stanley steering control law."""
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy, L)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    theta_d = np.arctan2(k * error_front_axle, state.v)
    delta = theta_e + theta_d

    return delta, current_target_idx


class StanleyControllerNode:
    def __init__(self):
        rospy.init_node("stanley_controller", anonymous=False)
        self._load_params()

        self._state = State()
        self._cx = None
        self._cy = None
        self._cz = None
        self._cyaw = None
        self._last_trajectory_time = rospy.Time(0)
        self._spline_ready = False
        self._last_pose_time = None
        self._last_pose_z = None
        self._last_target_idx = 0
        self._last_valid_vz_cmd = 0.0
        self._altitude_input_was_valid = True
        self._latest_vz_ff = 0.0
        self._latest_vz_ff_time = rospy.Time(0)
        self._last_cmd_linear_x = 0.0
        self._last_cmd_angular_z = 0.0
        self._last_cmd_linear_z = 0.0

        # Subscribers
        self._traj_sub = rospy.Subscriber(
            self._trajectory_topic, RosPath, self._trajectory_callback, queue_size=1)
        self._pose_sub = rospy.Subscriber(
            self._pose_topic, PoseStamped, self._pose_callback, queue_size=10)
        self._vz_cmd_sub = None
        if self._use_vz_ff_from_topic:
            self._vz_cmd_sub = rospy.Subscriber(
                self._vz_cmd_topic, Float64, self._vz_cmd_callback, queue_size=10)

        # Publisher
        self._cmd_pub = rospy.Publisher(self._cmd_vel_topic, Twist, queue_size=10)

        # Control timer
        self._control_timer = rospy.Timer(
            rospy.Duration(1.0 / self._control_rate), self._control_callback)

        rospy.loginfo("StanleyControllerNode initialized: k=%.2f, v=%.2f, rate=%.1f Hz",
                      self._k, self._target_velocity, self._control_rate)

    def _parse_list_param(self, name, default_value):
        raw = rospy.get_param(name, default_value)
        if isinstance(raw, str):
            try:
                raw = ast.literal_eval(raw)
            except Exception:
                rospy.logwarn("Param %s is not a valid list string: %s", name, raw)
                return list(default_value)
        if not isinstance(raw, (list, tuple)):
            rospy.logwarn("Param %s should be list/tuple, got %s", name, type(raw))
            return list(default_value)
        return list(raw)

    def _load_params(self):
        self._k = rospy.get_param("~k", 0.5)
        self._Kp = rospy.get_param("~Kp", 1.0)
        self._L = rospy.get_param("~L", 0.2)
        self._max_steer = rospy.get_param("~max_steer", np.radians(30.0))
        self._target_velocity = rospy.get_param("~target_velocity", 0.2)
        self._max_w = rospy.get_param("~max_w", 0.5)
        self._factor_v = rospy.get_param("~factor_v", 1.0)
        self._factor_w = rospy.get_param("~factor_w", 1.0)
        self._spline_ds = rospy.get_param("~spline_ds", 0.01)
        self._trajectory_timeout = rospy.get_param("~trajectory_timeout", 0.5)
        self._pose_topic = rospy.get_param("~pose_topic", "/pose")
        self._trajectory_topic = rospy.get_param("~trajectory_topic", "/planner/trajectory")
        self._cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self._control_rate = rospy.get_param("~control_rate", 30.0)

        # Decoupled vertical control params
        self._enable_lateral_control = rospy.get_param("~enable_lateral_control", True)
        self._enable_vertical_control = rospy.get_param("~enable_vertical_control", True)
        self._enable_altitude_hold = rospy.get_param("~enable_altitude_hold", True)
        self._kp_z = rospy.get_param("~kp_z", 1.0)
        self._z_deadband = rospy.get_param("~z_deadband", 0.05)

        _vz_limit = self._parse_list_param("~vz_limit", [-2.0, 2.0])
        if len(_vz_limit) == 2 and _vz_limit[0] < _vz_limit[1]:
            self._vz_limit = [float(_vz_limit[0]), float(_vz_limit[1])]
        else:
            rospy.logwarn("Invalid ~vz_limit=%s, fallback to [-2.0, 2.0]", _vz_limit)
            self._vz_limit = [-2.0, 2.0]

        _z_bounds = self._parse_list_param("~z_bounds", [-500.0, 500.0])
        if len(_z_bounds) == 2 and _z_bounds[0] < _z_bounds[1]:
            self._z_bounds = [float(_z_bounds[0]), float(_z_bounds[1])]
        else:
            rospy.logwarn("Invalid ~z_bounds=%s, fallback to [-500.0, 500.0]", _z_bounds)
            self._z_bounds = [-500.0, 500.0]

        self._use_vz_ff_from_topic = rospy.get_param("~use_vz_ff_from_topic", True)
        self._vz_cmd_topic = rospy.get_param("~vz_cmd_topic", "/planner/vz_cmd")
        self._vz_cmd_timeout = float(rospy.get_param("~vz_cmd_timeout", 1.0))
        self._vz_estimation_alpha = float(rospy.get_param("~vz_estimation_alpha", 0.25))

        # Near-goal stabilization and smoothing parameters.
        self._enable_goal_speed_taper = rospy.get_param("~enable_goal_speed_taper", False)
        self._goal_slowdown_distance = float(rospy.get_param("~goal_slowdown_distance", 0.0))
        self._goal_stop_distance = float(rospy.get_param("~goal_stop_distance", 0.6))
        self._min_forward_speed = float(rospy.get_param("~min_forward_speed", 0.03))
        self._cmd_smoothing_alpha = float(rospy.get_param("~cmd_smoothing_alpha", 0.35))
        self._yaw_rate_deadband = float(rospy.get_param("~yaw_rate_deadband", 0.01))

        # Smooth timeout handling: hold then softly decay instead of hard stop.
        self._timeout_hold_max = float(rospy.get_param("~timeout_hold_max", 2.0))
        self._timeout_hold_decay = float(rospy.get_param("~timeout_hold_decay", 0.985))

        # Spline de-duplication to avoid zero-length segments in cubic_spline_planner.
        self._min_waypoint_separation = float(rospy.get_param("~min_waypoint_separation", 0.05))

        # Normalize invalid values to deterministic defaults.
        if self._kp_z < 0.0:
            rospy.logwarn("~kp_z is negative (%.3f), clamp to 0.0", self._kp_z)
            self._kp_z = 0.0
        if self._z_deadband < 0.0:
            rospy.logwarn("~z_deadband is negative (%.3f), clamp to 0.0", self._z_deadband)
            self._z_deadband = 0.0
        self._vz_estimation_alpha = float(np.clip(self._vz_estimation_alpha, 0.0, 1.0))
        self._cmd_smoothing_alpha = float(np.clip(self._cmd_smoothing_alpha, 0.0, 1.0))
        self._goal_slowdown_distance = max(self._goal_slowdown_distance, 0.0)
        self._goal_stop_distance = max(self._goal_stop_distance, 0.0)
        self._min_forward_speed = max(self._min_forward_speed, 0.0)
        self._timeout_hold_max = max(self._timeout_hold_max, 0.0)
        self._timeout_hold_decay = float(np.clip(self._timeout_hold_decay, 0.0, 1.0))
        self._min_waypoint_separation = max(self._min_waypoint_separation, 0.0)

    def _pose_callback(self, msg):
        self._state.x = msg.pose.position.x
        self._state.y = msg.pose.position.y
        self._state.z = msg.pose.position.z
        quat = msg.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        self._state.yaw = euler_from_quaternion(quaternion)[2]
        self._state.v = self._target_velocity

        stamp = msg.header.stamp
        if stamp is None or stamp == rospy.Time():
            stamp = rospy.Time.now()

        if self._last_pose_time is not None and self._last_pose_z is not None:
            dt = (stamp - self._last_pose_time).to_sec()
            if 1e-6 < dt < 1.0:
                raw_vz = (self._state.z - self._last_pose_z) / dt
                alpha = self._vz_estimation_alpha
                self._state.vz = (1.0 - alpha) * self._state.vz + alpha * raw_vz

        self._last_pose_time = stamp
        self._last_pose_z = self._state.z

    def _vz_cmd_callback(self, msg):
        self._latest_vz_ff = float(msg.data)
        self._latest_vz_ff_time = rospy.Time.now()

    def _trajectory_callback(self, msg):
        """Receive planned trajectory, build cubic spline for tracking."""
        poses = msg.poses
        if len(poses) < 2:
            # Keep following the previous spline for short path dropouts to avoid stop-go jitter.
            if self._spline_ready:
                self._last_trajectory_time = rospy.Time.now()
                rospy.logwarn_throttle(
                    2.0,
                    "Received trajectory with < 2 poses, keep previous spline",
                )
            else:
                rospy.logwarn_throttle(2.0, "Received trajectory with < 2 poses, ignoring")
            return

        ax = [p.pose.position.x for p in poses]
        ay = [p.pose.position.y for p in poses]
        az = [p.pose.position.z for p in poses]

        # Remove near-duplicate consecutive points to avoid zero segment length (h=0).
        if self._min_waypoint_separation > 0.0:
            dedup = [(ax[0], ay[0], az[0])]
            for i in range(1, len(ax)):
                px, py, pz = ax[i], ay[i], az[i]
                dx = px - dedup[-1][0]
                dy = py - dedup[-1][1]
                if np.hypot(dx, dy) >= self._min_waypoint_separation:
                    dedup.append((px, py, pz))

            if len(dedup) < 2:
                rospy.logwarn_throttle(2.0, "Trajectory dedup leaves < 2 points, ignoring trajectory")
                return

            ax = [p[0] for p in dedup]
            ay = [p[1] for p in dedup]
            az = [p[2] for p in dedup]

        had_spline = self._spline_ready

        try:
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
                ax, ay, ds=self._spline_ds)
            if len(cx) < 2:
                rospy.logwarn_throttle(2.0, "Spline has < 2 points after dedup, ignoring trajectory")
                return
            self._cx = cx
            self._cy = cy
            self._cyaw = cyaw

            if len(az) >= 2:
                src = np.arange(len(az), dtype=np.float64)
                dst = np.linspace(0.0, len(az) - 1, num=len(cx), dtype=np.float64)
                self._cz = np.interp(dst, src, np.array(az, dtype=np.float64)).tolist()
            else:
                self._cz = [az[0]] * len(cx)

            self._spline_ready = True
            if not had_spline:
                self._last_target_idx = 0
            else:
                self._last_target_idx = int(np.clip(self._last_target_idx, 0, max(0, len(cx) - 2)))
            self._last_trajectory_time = rospy.Time.now()
            rospy.logdebug("Spline built: %d waypoints -> %d spline points",
                           len(poses), len(cx))
        except Exception as e:
            rospy.logerr("Failed to build spline from trajectory: %s", e)
            self._spline_ready = False
            self._cz = None
            self._last_target_idx = 0

    def _get_vz_feedforward(self):
        if not self._use_vz_ff_from_topic:
            return 0.0
        if self._latest_vz_ff_time == rospy.Time(0):
            return 0.0
        if self._vz_cmd_timeout > 0.0:
            age = (rospy.Time.now() - self._latest_vz_ff_time).to_sec()
            if age > self._vz_cmd_timeout:
                return 0.0
        return self._latest_vz_ff

    def _compute_vertical_velocity_command(self, z_ref, z_meas, vz_ff):
        if not self._enable_vertical_control:
            self._last_valid_vz_cmd = 0.0
            return 0.0

        input_invalid = False
        reason = ""
        if np.isnan(z_ref) or np.isnan(z_meas):
            input_invalid = True
            reason = "nan"
        elif z_ref < self._z_bounds[0] or z_ref > self._z_bounds[1] or z_meas < self._z_bounds[0] or z_meas > self._z_bounds[1]:
            input_invalid = True
            reason = "out_of_bounds"

        if input_invalid and self._enable_altitude_hold:
            if self._altitude_input_was_valid:
                rospy.logwarn("Vertical control hold: %s (z_ref=%.3f, z_meas=%.3f), keeping vz_cmd=%.3f",
                              reason, z_ref, z_meas, self._last_valid_vz_cmd)
                self._altitude_input_was_valid = False
            return self._last_valid_vz_cmd

        if not input_invalid and not self._altitude_input_was_valid:
            rospy.loginfo("Vertical control input recovered (z_ref=%.3f, z_meas=%.3f)", z_ref, z_meas)
            self._altitude_input_was_valid = True

        e_z = z_ref - z_meas
        vz_cmd = self._kp_z * e_z
        if abs(e_z) < self._z_deadband:
            vz_cmd = 0.0
        vz_cmd += vz_ff
        vz_cmd = float(np.clip(vz_cmd, self._vz_limit[0], self._vz_limit[1]))
        self._last_valid_vz_cmd = vz_cmd
        return vz_cmd

    def _control_callback(self, event):
        """Timer callback: compute and publish control command."""
        twist = Twist()

        # Check trajectory timeout
        dt = (rospy.Time.now() - self._last_trajectory_time).to_sec()
        if not self._spline_ready:
            self._cmd_pub.publish(twist)
            return

        if dt > self._trajectory_timeout:
            # Hold and smoothly decay the previous command to avoid stop-go chatter.
            hold_window = self._trajectory_timeout + self._timeout_hold_max
            if self._timeout_hold_max > 0.0 and dt <= hold_window:
                over = max(0.0, dt - self._trajectory_timeout)
                decay = self._timeout_hold_decay ** (over * self._control_rate)
                twist.linear.x = self._last_cmd_linear_x * decay
                twist.angular.z = self._last_cmd_angular_z * decay
                twist.linear.z = self._last_cmd_linear_z * decay
                self._cmd_pub.publish(twist)
                return

            rospy.logwarn_throttle(2.0,
                "Trajectory timeout (%.2fs > %.2fs), hold window passed -> stop", dt, self._trajectory_timeout)
            self._last_cmd_linear_x = 0.0
            self._last_cmd_angular_z = 0.0
            self._last_cmd_linear_z = 0.0
            self._cmd_pub.publish(twist)
            return

        state = self._state

        try:
            target_idx, _ = calc_target_index(state, self._cx, self._cy, self._L)
        except (IndexError, ValueError) as e:
            rospy.logwarn_throttle(2.0, "calc_target_index error: %s", e)
            self._cmd_pub.publish(twist)
            return

        # Keep target index monotonic across control ticks to reduce back-and-forth oscillation.
        n_path = len(self._cx)
        max_track_idx = max(0, n_path - 2)
        target_idx = int(np.clip(max(target_idx, self._last_target_idx), 0, max_track_idx))

        # Smoothly taper forward speed near final waypoint to avoid shaky stop behavior.
        goal_dx = self._cx[-1] - state.x
        goal_dy = self._cy[-1] - state.y
        dist_to_goal = float(np.hypot(goal_dx, goal_dy))
        if (not self._enable_goal_speed_taper) or self._goal_slowdown_distance <= 0.0:
            desired_v = self._target_velocity
        else:
            if dist_to_goal <= self._goal_stop_distance:
                desired_v = 0.0
            elif dist_to_goal < self._goal_slowdown_distance:
                speed_scale = dist_to_goal / self._goal_slowdown_distance
                desired_v = max(self._min_forward_speed, self._target_velocity * speed_scale)
            else:
                desired_v = self._target_velocity

        if self._enable_lateral_control and n_path > 1:
            di, target_idx = stanley_control(
                state, self._cx, self._cy, self._cyaw, target_idx,
                self._k, self._L)
            target_idx = int(np.clip(target_idx, 0, max_track_idx))
            di = np.clip(di, -self._max_steer, self._max_steer)
            w = desired_v / self._L * np.tan(di)
            w = np.clip(w, -self._max_w, self._max_w)
            if abs(w) < self._yaw_rate_deadband:
                w = 0.0

            twist.linear.x = desired_v * self._factor_v
            twist.angular.z = w * self._factor_w
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        if self._enable_vertical_control and self._cz is not None and len(self._cz) > 0:
            z_ref = self._cz[min(target_idx, len(self._cz) - 1)]
            vz_ff = self._get_vz_feedforward()
            twist.linear.z = self._compute_vertical_velocity_command(z_ref, state.z, vz_ff)
        else:
            twist.linear.z = 0.0

        # # First-order smoothing to suppress high-frequency jitter in ROS command stream.
        # a = self._cmd_smoothing_alpha
        # twist.linear.x = (1.0 - a) * self._last_cmd_linear_x + a * twist.linear.x
        # twist.angular.z = (1.0 - a) * self._last_cmd_angular_z + a * twist.angular.z
        # twist.linear.z = (1.0 - a) * self._last_cmd_linear_z + a * twist.linear.z

        self._last_cmd_linear_x = twist.linear.x
        self._last_cmd_angular_z = twist.angular.z
        self._last_cmd_linear_z = twist.linear.z
        self._last_target_idx = target_idx

        self._cmd_pub.publish(twist)


if __name__ == "__main__":
    try:
        node = StanleyControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
