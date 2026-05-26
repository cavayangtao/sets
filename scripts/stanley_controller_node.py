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
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import os

# Import cubic spline planner from same directory
_script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _script_dir)
import cubic_spline_planner


class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


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
        self._cyaw = None
        self._last_trajectory_time = rospy.Time(0)
        self._spline_ready = False

        # Subscribers
        self._traj_sub = rospy.Subscriber(
            self._trajectory_topic, RosPath, self._trajectory_callback, queue_size=1)
        self._pose_sub = rospy.Subscriber(
            self._pose_topic, PoseStamped, self._pose_callback, queue_size=10)

        # Publisher
        self._cmd_pub = rospy.Publisher(self._cmd_vel_topic, Twist, queue_size=10)

        # Control timer
        self._control_timer = rospy.Timer(
            rospy.Duration(1.0 / self._control_rate), self._control_callback)

        rospy.loginfo("StanleyControllerNode initialized: k=%.2f, v=%.2f, rate=%.1f Hz",
                      self._k, self._target_velocity, self._control_rate)

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

    def _pose_callback(self, msg):
        self._state.x = msg.pose.position.x
        self._state.y = msg.pose.position.y
        quat = msg.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        self._state.yaw = euler_from_quaternion(quaternion)[2]
        self._state.v = self._target_velocity

    def _trajectory_callback(self, msg):
        """Receive planned trajectory, build cubic spline for tracking."""
        poses = msg.poses
        if len(poses) < 2:
            rospy.logwarn("Received trajectory with < 2 poses, ignoring")
            return

        ax = [p.pose.position.x for p in poses]
        ay = [p.pose.position.y for p in poses]

        try:
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
                ax, ay, ds=self._spline_ds)
            self._cx = cx
            self._cy = cy
            self._cyaw = cyaw
            self._spline_ready = True
            self._last_trajectory_time = rospy.Time.now()
            rospy.logdebug("Spline built: %d waypoints -> %d spline points",
                           len(poses), len(cx))
        except Exception as e:
            rospy.logerr("Failed to build spline from trajectory: %s", e)
            self._spline_ready = False

    def _control_callback(self, event):
        """Timer callback: compute and publish control command."""
        twist = Twist()

        # Check trajectory timeout
        dt = (rospy.Time.now() - self._last_trajectory_time).to_sec()
        if not self._spline_ready or dt > self._trajectory_timeout:
            if self._spline_ready and dt > self._trajectory_timeout:
                rospy.logwarn_throttle(2.0,
                    "Trajectory timeout (%.2fs > %.2fs), stopping", dt, self._trajectory_timeout)
            self._cmd_pub.publish(twist)
            return

        state = self._state

        try:
            target_idx, _ = calc_target_index(state, self._cx, self._cy, self._L)
        except (IndexError, ValueError) as e:
            rospy.logwarn_throttle(2.0, "calc_target_index error: %s", e)
            self._cmd_pub.publish(twist)
            return

        if target_idx < len(self._cx) - 1:
            di, target_idx = stanley_control(
                state, self._cx, self._cy, self._cyaw, target_idx,
                self._k, self._L)
            di = np.clip(di, -self._max_steer, self._max_steer)
            w = self._target_velocity / self._L * np.tan(di)
            w = np.clip(w, -self._max_w, self._max_w)

            twist.linear.x = self._target_velocity * self._factor_v
            twist.angular.z = w * self._factor_w
        else:
            # Reached end of path
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self._cmd_pub.publish(twist)


if __name__ == "__main__":
    try:
        node = StanleyControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
