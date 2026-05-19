#!/usr/bin/env python3
"""
One-click ROS mission runner for two sequential waypoints.

Flow:
1) Wait for planner services and mocap pose stream
2) Arm planner
3) Set wp0 via /tello_planner/target_pos + /tello_planner/update_target
4) Wait until wp0 reached
5) Set wp1 and wait until wp1 reached
6) Save trajectory plot + GIF under sets/plots

Prerequisites:
- roscore is running
- tello_planner node is running (e.g., roslaunch sets tello_planner.launch)
"""

import os
import sys
import time
import math
import argparse
import threading
import json
import ast

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

_project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(_project_root, "src"))
sys.path.insert(0, _project_root)

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool, Trigger
from util import util


def _parse_scale_xyz(value):
    """Parse ROS param for scale xyz, accepting list or string literal."""
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except Exception:
            value = [1.0, 1.0, 1.0]
    try:
        arr = np.array(value, dtype=np.float64).reshape(-1)
    except Exception:
        arr = np.array([1.0, 1.0, 1.0], dtype=np.float64)
    if arr.size != 3:
        arr = np.array([1.0, 1.0, 1.0], dtype=np.float64)
    return arr


def transform_position(px, py, pz, frame_config):
    """Apply the same position transform convention as tello_planner_node."""
    scale = _parse_scale_xyz(frame_config.get("mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]))
    rz_deg = float(frame_config.get("mocap_to_planner_rz_deg", 0.0))

    px, py, pz = px * scale[0], py * scale[1], pz * scale[2]

    if abs(rz_deg) > 1e-9:
        rad = math.radians(rz_deg)
        c, s = math.cos(rad), math.sin(rad)
        px, py = c * px - s * py, s * px + c * py

    mocap_frame = frame_config.get("mocap_frame", "ENU")
    if mocap_frame == "ENU":
        pz = -pz
    elif mocap_frame == "NED_ZY":
        px, py, pz = px, pz, py

    return float(px), float(py), float(pz)


def planner_to_mocap_position(px, py, pz, frame_config):
    """Inverse of transform_position for position-only mapping."""
    scale = _parse_scale_xyz(frame_config.get("mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]))
    rz_deg = float(frame_config.get("mocap_to_planner_rz_deg", 0.0))
    mocap_frame = frame_config.get("mocap_frame", "ENU")

    # inverse hand-ness / axis permutation
    if mocap_frame == "ENU":
        px_h, py_h, pz_h = px, py, -pz
    elif mocap_frame == "NED_ZY":
        px_h, py_h, pz_h = px, pz, py
    else:  # NED
        px_h, py_h, pz_h = px, py, pz

    # inverse rotation around Z
    if abs(rz_deg) > 1e-9:
        rad = math.radians(rz_deg)
        c, s = math.cos(rad), math.sin(rad)
        px_r = c * px_h + s * py_h
        py_r = -s * px_h + c * py_h
    else:
        px_r, py_r = px_h, py_h

    # inverse scale
    sx = scale[0] if abs(scale[0]) > 1e-12 else 1.0
    sy = scale[1] if abs(scale[1]) > 1e-12 else 1.0
    sz = scale[2] if abs(scale[2]) > 1e-12 else 1.0
    return float(px_r / sx), float(py_r / sy), float(pz_h / sz)


def planner_yaw_to_mocap_yaw(planner_yaw, frame_config):
    """Approximate inverse yaw mapping used by transform_pose for ENU/NED."""
    rz_deg = float(frame_config.get("mocap_to_planner_rz_deg", 0.0))
    rad = math.radians(rz_deg)
    mocap_frame = frame_config.get("mocap_frame", "ENU")

    if mocap_frame == "ENU":
        return float(-planner_yaw - rad)
    # NED and NED_ZY share this approximation for yaw-only test publishing
    return float(planner_yaw - rad)


class RosTestPoseNode:
    """ROS test node for virtual pose publishing.

    Modes:
      - rollout (default): advance pose using UCT rollout states (model-consistent)
      - cmd_dynamics: advance pose by applying /cmd_vel through mdp.F
    """

    def __init__(self, pose_topic, cmd_topic, frame_config, config_name, sim_rate_hz, mode="rollout"):
        from build.bindings import get_mdp, get_dots_mdp, get_uct2, RNG, run_uct2

        self._pose_topic = pose_topic
        self._cmd_topic = cmd_topic
        self._frame_config = frame_config
        self._mode = mode
        self._sim_dt = 1.0 / max(1e-3, sim_rate_hz)
        self._lock = threading.Lock()
        self._u = np.zeros(4, dtype=np.float64)
        self._segment = []

        self._run_uct2 = run_uct2

        cfg_path = util.get_config_path(config_name)
        cfg = util.load_yaml(cfg_path)
        self._mdp = get_mdp("SixDOFAircraft", cfg_path)
        self._mdp.set_dt(float(cfg.get("uct_dt", 0.01)))
        self._state = np.array(self._mdp.initial_state(), dtype=np.float64)

        self._mdp.clear_obstacles()
        for obs in util.get_obstacles(cfg, 0):
            self._mdp.add_obstacle(obs)

        self._dots = get_dots_mdp()
        self._dots.set_param(
            self._mdp,
            cfg["dots_expansion_mode"],
            cfg["dots_initialize_mode"],
            cfg["dots_special_actions"],
            cfg["dots_num_branches"],
            int(cfg.get("dots_decision_making_horizon", 20)),
            int(cfg.get("dots_dynamics_horizon", 5)),
            cfg["dots_spectral_branches_mode"],
            cfg["dots_control_mode"],
            cfg["dots_scale_mode"],
            cfg["dots_modal_damping_mode"],
            cfg["dots_modal_damping_gains"],
            np.diag(np.array(cfg["dots_rho"])),
            cfg["dots_greedy_gain"],
            cfg["dots_greedy_rate"],
            cfg["dots_greedy_min_dist"],
            cfg["dots_baseline_mode_on"],
            cfg["dots_num_discrete_actions"],
            False,
        )

        self._uct = get_uct2()
        sim_uct_N = int(rospy.get_param("/tello_planner/uct_N", cfg.get("uct_N", 500)))
        sim_uct_max_depth = int(rospy.get_param("/tello_planner/uct_max_depth", cfg.get("uct_max_depth", 12)))
        sim_uct_wct = float(rospy.get_param("/tello_planner/uct_wct", cfg.get("uct_wct", 1200.0)))
        sim_uct_c = float(rospy.get_param("/tello_planner/uct_c", cfg.get("uct_c", 3.0)))
        sim_uct_heuristic = rospy.get_param("/tello_planner/uct_heuristic_mode", cfg.get("uct_heuristic_mode", "shuffled"))
        sim_uct_tree_exploration = rospy.get_param("/tello_planner/uct_tree_exploration", cfg.get("uct_tree_exploration", "uct"))
        sim_uct_downsample = bool(rospy.get_param("/tello_planner/uct_downsample_traj_on", cfg.get("uct_downsample_traj_on", True)))
        self._uct.set_param(
            sim_uct_N,
            sim_uct_max_depth,
            sim_uct_wct,
            sim_uct_c,
            False,
            False,
            False,
            False,
            False,
            sim_uct_heuristic,
            sim_uct_tree_exploration,
            sim_uct_downsample,
            False,
        )
        sim_seed = int(rospy.get_param("/tello_planner/seed", cfg.get("seed", 42)))
        self._rng = RNG()
        self._rng.set_seed(sim_seed)
        self._mpc_steps = int(rospy.get_param("/tello_planner/uct_mpc_depth", cfg.get("uct_mpc_depth", 2)))
        self._config_name = config_name

        self._cmd_sub = rospy.Subscriber(self._cmd_topic, Twist, self._cmd_cb, queue_size=20)
        self._pose_pub = rospy.Publisher(self._pose_topic, PoseStamped, queue_size=20)
        self._timer = rospy.Timer(rospy.Duration(self._sim_dt), self._tick)

        # Publish initial pose immediately so planner can leave INIT.
        self._publish_pose(rospy.Time.now())
        rospy.loginfo("RosTestPoseNode started: mode=%s pose=%s cmd=%s dt=%.3f seed=%d",
              self._mode, self._pose_topic, self._cmd_topic, self._sim_dt, sim_seed)

    def _cmd_cb(self, msg):
        with self._lock:
            self._u[:] = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]

    def _publish_pose(self, stamp):
        px_m, py_m, pz_m = planner_to_mocap_position(
            self._state[0], self._state[1], self._state[2], self._frame_config
        )
        yaw_m = planner_yaw_to_mocap_yaw(self._state[8], self._frame_config)

        msg = PoseStamped()
        msg.header.stamp = stamp if stamp is not None else rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = px_m
        msg.pose.position.y = py_m
        msg.pose.position.z = pz_m
        msg.pose.orientation.z = math.sin(yaw_m * 0.5)
        msg.pose.orientation.w = math.cos(yaw_m * 0.5)
        self._pose_pub.publish(msg)

    def _tick(self, event):
        try:
            if self._mode == "cmd_dynamics":
                with self._lock:
                    u = self._u.copy()
                nxt = np.array(self._mdp.F(self._state, u), dtype=np.float64)
                if self._mdp.is_state_valid(nxt):
                    self._state = nxt
                else:
                    rospy.logwarn_throttle(5.0, "RosTestPoseNode state invalid; holding previous state")
            else:
                # rollout mode: use model-consistent planned states as pose stream.
                if not self._segment:
                    self._sync_target_from_ros()
                    res = self._run_uct2(self._dots, self._uct, self._state, self._rng)
                    if getattr(res, "success", False):
                        xs = np.array(res.planned_traj.xs)
                        steps = min(self._mpc_steps, xs.shape[0] - 1)
                        if steps > 0:
                            self._segment = [xs[k, :].copy() for k in range(1, steps + 1)]
                    else:
                        rospy.logwarn_throttle(5.0, "RosTestPoseNode rollout planning failed")

                if self._segment:
                    self._state = np.array(self._segment.pop(0), dtype=np.float64)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "RosTestPoseNode step failed: %s", e)

        self._publish_pose(event.current_real if event is not None else rospy.Time.now())

    def _sync_target_from_ros(self):
        raw = rospy.get_param("/tello_planner/target_pos", None)
        if raw is None:
            return
        if isinstance(raw, str):
            try:
                raw = ast.literal_eval(raw)
            except Exception:
                return
        try:
            xd = np.array(raw, dtype=np.float64).reshape(-1, 1)
            if xd.shape[0] == self._mdp.state_dim():
                self._mdp.set_xd(xd)
        except Exception:
            pass

    def shutdown(self):
        try:
            self._timer.shutdown()
        except Exception:
            pass


class TwoWaypointRunner:
    def __init__(self, args):
        self.args = args

        self.pose_topic = args.pose_topic or rospy.get_param("/tello_planner/pose_topic", "/mocap/pose")
        self.cmd_topic = args.cmd_topic or rospy.get_param("/tello_planner/cmd_vel_topic", "/cmd_vel")

        self.frame_config = {
            "mocap_frame": rospy.get_param("/tello_planner/mocap_frame", "ENU"),
            "mocap_to_planner_rz_deg": rospy.get_param("/tello_planner/mocap_to_planner_rz_deg", 0.0),
            "mocap_to_planner_scale_xyz": rospy.get_param("/tello_planner/mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]),
        }

        self._pose_lock = threading.Lock()
        self._cmd_lock = threading.Lock()

        self.latest_pos = None
        self.latest_cmd = np.zeros(4, dtype=np.float64)

        self.samples = []
        self.phase_dmin = {}
        self.phase = "idle"
        self.start_wall = time.time()

        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=20)
        self.cmd_sub = rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb, queue_size=20)

        self._test_pose_node = None
        if self.args.use_ros_test_node:
            self._test_pose_node = RosTestPoseNode(
                pose_topic=self.pose_topic,
                cmd_topic=self.cmd_topic,
                frame_config=self.frame_config,
                config_name=self.args.sim_config_name,
                sim_rate_hz=self.args.sim_rate,
                mode=self.args.sim_mode,
            )

        rospy.wait_for_service("/tello_planner/arm", timeout=15.0)
        rospy.wait_for_service("/tello_planner/update_target", timeout=15.0)
        self.arm_srv = rospy.ServiceProxy("/tello_planner/arm", SetBool)
        self.update_target_srv = rospy.ServiceProxy("/tello_planner/update_target", Trigger)

    def _cmd_cb(self, msg):
        with self._cmd_lock:
            self.latest_cmd[:] = [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z,
            ]

    def _pose_cb(self, msg):
        p = msg.pose.position
        px, py, pz = transform_position(p.x, p.y, p.z, self.frame_config)

        with self._pose_lock:
            self.latest_pos = np.array([px, py, pz], dtype=np.float64)

        with self._cmd_lock:
            cmd = self.latest_cmd.copy()

        self.samples.append({
            "t": time.time() - self.start_wall,
            "x": px,
            "y": py,
            "z": pz,
            "vx": float(cmd[0]),
            "vy": float(cmd[1]),
            "vz": float(cmd[2]),
            "yr": float(cmd[3]),
            "phase": self.phase,
        })

    def _wait_first_pose(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            with self._pose_lock:
                if self.latest_pos is not None:
                    return True
            rospy.sleep(0.05)
        return False

    def _set_target(self, wp):
        target = [float(wp[0]), float(wp[1]), float(wp[2])] + [0.0] * 10
        rospy.set_param("/tello_planner/target_pos", target)
        resp = self.update_target_srv()
        if not resp.success:
            raise RuntimeError(f"update_target failed: {resp.message}")

    def _distance_to(self, wp):
        with self._pose_lock:
            p = None if self.latest_pos is None else self.latest_pos.copy()
        if p is None:
            return None
        return float(np.linalg.norm(p - np.array(wp, dtype=np.float64)))

    def _wait_reach(self, wp, reach_dist, timeout, phase_name):
        self.phase = phase_name
        t0 = time.time()
        last_log = 0.0
        dmin = float("inf")

        while not rospy.is_shutdown():
            dt = time.time() - t0
            if dt > timeout:
                self.phase_dmin[phase_name] = dmin if dmin < float("inf") else None
                return False, None

            dist = self._distance_to(wp)
            with self._pose_lock:
                p_now = None if self.latest_pos is None else self.latest_pos.copy()
            if dist is not None and dist < reach_dist:
                self.phase_dmin[phase_name] = min(dmin, dist)
                return True, dist

            if dist is not None:
                dmin = min(dmin, dist)

            if dt - last_log > 1.0 and dist is not None:
                if p_now is not None:
                    rospy.loginfo(
                        "[%s] dist=%.2fm pos=(%.2f,%.2f,%.2f), remaining %.1fs",
                        phase_name, dist, p_now[0], p_now[1], p_now[2], timeout - dt,
                    )
                else:
                    rospy.loginfo("[%s] dist=%.2fm, remaining %.1fs", phase_name, dist, timeout - dt)
                last_log = dt

            rospy.sleep(0.05)

        self.phase_dmin[phase_name] = dmin if dmin < float("inf") else None
        return False, None

    def run(self):
        wp0 = np.array(self.args.wp0, dtype=np.float64)
        wp1 = np.array(self.args.wp1, dtype=np.float64)

        rospy.loginfo("Waiting first pose on %s ...", self.pose_topic)
        if not self._wait_first_pose(timeout=self.args.pose_wait_timeout):
            raise RuntimeError("No pose received before timeout")

        rospy.loginfo("Set waypoint 0: (%.1f, %.1f, %.1f)", wp0[0], wp0[1], wp0[2])
        self._set_target(wp0)

        rospy.loginfo("Arming planner ...")
        arm_resp = self.arm_srv(data=True)
        if not arm_resp.success:
            # When auto_arm is enabled, planner may already be RUNNING.
            if "RUNNING" in arm_resp.message:
                rospy.logwarn("Planner already RUNNING; continue without explicit arm")
            else:
                raise RuntimeError(f"arm failed: {arm_resp.message}")

        ok0, d0 = self._wait_reach(wp0, self.args.reach_dist, self.args.wp_timeout, "wp0")
        if not ok0:
            raise RuntimeError("Timeout waiting waypoint 0")
        rospy.loginfo("Waypoint 0 reached (dist=%.2f m)", d0)

        rospy.loginfo("Switch to waypoint 1: (%.1f, %.1f, %.1f)", wp1[0], wp1[1], wp1[2])
        self._set_target(wp1)
        ok1, d1 = self._wait_reach(wp1, self.args.reach_dist, self.args.wp_timeout, "wp1")
        if not ok1:
            raise RuntimeError("Timeout waiting waypoint 1")
        rospy.loginfo("Waypoint 1 reached (dist=%.2f m)", d1)

        self.phase = "done"
        return {
            "wp0_dist": float(d0),
            "wp1_dist": float(d1),
        }

    def shutdown(self):
        if self._test_pose_node is not None:
            self._test_pose_node.shutdown()


def load_obstacle_boxes():
    config = util.load_yaml(util.get_config_path("policy_convergence_tello_stage3"))
    obstacles = util.get_obstacles(config, 0)
    boxes = []
    for obs in obstacles:
        boxes.append(((obs[0, 0], obs[0, 1]), (obs[1, 0], obs[1, 1]), (obs[2, 0], obs[2, 1])))
    return boxes


def save_plots(samples, wp0, wp1, out_prefix):
    if not samples:
        raise RuntimeError("No trajectory samples collected")

    os.makedirs(os.path.dirname(out_prefix), exist_ok=True)

    t = np.array([s["t"] for s in samples], dtype=np.float64)
    x = np.array([s["x"] for s in samples], dtype=np.float64)
    y = np.array([s["y"] for s in samples], dtype=np.float64)
    z = np.array([s["z"] for s in samples], dtype=np.float64)
    vx = np.array([s["vx"] for s in samples], dtype=np.float64)
    vy = np.array([s["vy"] for s in samples], dtype=np.float64)
    vz = np.array([s["vz"] for s in samples], dtype=np.float64)

    final_pos = np.array([x[-1], y[-1], z[-1]], dtype=np.float64)
    d_wp0 = np.linalg.norm(np.stack([x, y, z], axis=1) - np.array(wp0, dtype=np.float64), axis=1)
    d_wp1 = np.linalg.norm(np.stack([x, y, z], axis=1) - np.array(wp1, dtype=np.float64), axis=1)

    boxes = load_obstacle_boxes()

    png_path = out_prefix + ".png"
    gif_path = out_prefix + ".gif"

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    ax = axes[0, 0]
    ax.plot(x, y, color="royalblue", linewidth=1.2, label="trajectory")
    ax.plot(x[0], y[0], "go", markersize=8, label="start")
    ax.plot(x[-1], y[-1], "ko", markersize=7, label="final")
    ax.plot(wp0[0], wp0[1], marker="*", color="orange", markersize=14, label="wp0")
    ax.plot(wp1[0], wp1[1], marker="*", color="red", markersize=14, label="wp1")
    for xr, yr, _ in boxes:
        rect = Rectangle((xr[0], yr[0]), xr[1] - xr[0], yr[1] - yr[0],
                         alpha=0.25, facecolor="firebrick", edgecolor="darkred")
        ax.add_patch(rect)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down trajectory")
    ax.axis("equal")
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    ax.plot(t, z, color="seagreen", linewidth=1.0)
    ax.axhline(wp0[2], color="orange", linestyle="--", linewidth=0.9, alpha=0.7)
    ax.axhline(wp1[2], color="red", linestyle=":", linewidth=0.9, alpha=0.7)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("z (m)")
    ax.set_title("Altitude")

    ax = axes[1, 0]
    ax.plot(t, x, linewidth=0.9, label="x")
    ax.plot(t, y, linewidth=0.9, label="y")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("position (m)")
    ax.set_title("x/y vs time")
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    ax.plot(t, vx, linewidth=0.9, label="cmd vx")
    ax.plot(t, vy, linewidth=0.9, label="cmd vy")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("cmd (m/s)")
    ax.set_title("Command history")
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(7.2, 6.2))
    for xr, yr, _ in boxes:
        rect = Rectangle((xr[0], yr[0]), xr[1] - xr[0], yr[1] - yr[0],
                         alpha=0.25, facecolor="firebrick", edgecolor="darkred")
        ax2.add_patch(rect)
    ax2.plot(wp0[0], wp0[1], marker="*", color="orange", markersize=14)
    ax2.plot(wp1[0], wp1[1], marker="*", color="red", markersize=14)
    ax2.plot(x[0], y[0], "go", markersize=7)

    xmin, xmax = np.min(x), np.max(x)
    ymin, ymax = np.min(y), np.max(y)
    pad = 3.0
    ax2.set_xlim(min(xmin, wp0[0], wp1[0]) - pad, max(xmax, wp0[0], wp1[0]) + pad)
    ax2.set_ylim(min(ymin, wp0[1], wp1[1]) - pad, max(ymax, wp0[1], wp1[1]) + pad)
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_title("Trajectory animation")
    ax2.axis("equal")

    line, = ax2.plot([], [], color="royalblue", linewidth=1.2)
    dot, = ax2.plot([], [], "o", color="navy", markersize=7)

    n_frames = min(300, len(x))
    idxs = np.linspace(0, len(x) - 1, n_frames).astype(int)

    def _init():
        line.set_data([], [])
        dot.set_data([], [])
        return line, dot

    def _update(i):
        k = idxs[i]
        line.set_data(x[:k + 1], y[:k + 1])
        dot.set_data([x[k]], [y[k]])
        return line, dot

    ani = FuncAnimation(fig2, _update, init_func=_init, frames=len(idxs), interval=50, blit=True)
    ani.save(gif_path, writer="pillow", fps=20, dpi=90)
    plt.close(fig2)

    metrics = {
        "final_pos": [float(final_pos[0]), float(final_pos[1]), float(final_pos[2])],
        "final_dist_wp0": float(np.linalg.norm(final_pos - np.array(wp0, dtype=np.float64))),
        "final_dist_wp1": float(np.linalg.norm(final_pos - np.array(wp1, dtype=np.float64))),
        "min_dist_wp0": float(np.min(d_wp0)),
        "min_dist_wp1": float(np.min(d_wp1)),
        "duration_s": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
        "num_samples": int(len(samples)),
    }

    # Smoothness and safety indicators for parameter tuning.
    pos = np.stack([x, y, z], axis=1)
    diffs = np.diff(pos, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    path_len = float(np.sum(seg))
    straight_len = float(np.linalg.norm(np.array(wp1, dtype=np.float64) - pos[0]))
    metrics["path_len"] = path_len
    metrics["straight_len_start_to_wp1"] = straight_len
    metrics["path_efficiency"] = float(straight_len / (path_len + 1e-9))

    xy_step = np.diff(np.stack([x, y], axis=1), axis=0)
    if xy_step.shape[0] > 1:
        headings = np.arctan2(xy_step[:, 1], xy_step[:, 0])
        dhead = np.diff(np.unwrap(headings))
        metrics["total_turn_rad"] = float(np.sum(np.abs(dhead)))
    else:
        metrics["total_turn_rad"] = 0.0

    cmd = np.stack([vx, vy, vz], axis=1)
    if cmd.shape[0] > 1:
        dcmd = np.diff(cmd, axis=0)
        dcmd_norm = np.linalg.norm(dcmd, axis=1)
        metrics["cmd_delta_rms"] = float(np.sqrt(np.mean(dcmd_norm ** 2)))
        metrics["cmd_delta_max"] = float(np.max(dcmd_norm))
    else:
        metrics["cmd_delta_rms"] = 0.0
        metrics["cmd_delta_max"] = 0.0

    # 3D min distance from trajectory points to all axis-aligned obstacle boxes.
    min_clearance = float("inf")
    for xr, yr, zr in boxes:
        x_min, x_max = min(xr[0], xr[1]), max(xr[0], xr[1])
        y_min, y_max = min(yr[0], yr[1]), max(yr[0], yr[1])
        z_min, z_max = min(zr[0], zr[1]), max(zr[0], zr[1])
        dx = np.maximum(0.0, np.maximum(x_min - x, x - x_max))
        dy = np.maximum(0.0, np.maximum(y_min - y, y - y_max))
        dz = np.maximum(0.0, np.maximum(z_min - z, z - z_max))
        dist = np.sqrt(dx * dx + dy * dy + dz * dz)
        min_clearance = min(min_clearance, float(np.min(dist)))
    metrics["min_obstacle_clearance"] = float(min_clearance if np.isfinite(min_clearance) else 0.0)

    return png_path, gif_path, metrics


def parse_wp(text):
    vals = [float(v.strip()) for v in text.split(",") if v.strip()]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("Waypoint must be 'x,y,z'")
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wp0", type=parse_wp, default=[20.0, 0.0, -12.0],
                    help="Waypoint 0 in planner frame, format: x,y,z")
    ap.add_argument("--wp1", type=parse_wp, default=[20.0, 10.0, -12.0],
                    help="Waypoint 1 in planner frame, format: x,y,z")
    ap.add_argument("--reach-dist", type=float, default=2.0,
                    help="Arrival distance threshold in meters")
    ap.add_argument("--wp-timeout", type=float, default=90.0,
                    help="Timeout per waypoint in seconds")
    ap.add_argument("--pose-wait-timeout", type=float, default=20.0,
                    help="Wait timeout for first mocap pose in seconds")
    ap.add_argument("--out-prefix", type=str,
                    default="/home/tyang/sets/plots/tello_two_wp_auto",
                    help="Output prefix, without extension")
    ap.add_argument("--pose-topic", type=str, default="",
                    help="Pose topic to subscribe. Default: /tello_planner/pose_topic or /mocap/pose")
    ap.add_argument("--cmd-topic", type=str, default="",
                    help="Command topic to subscribe. Default: /tello_planner/cmd_vel_topic or /cmd_vel")
    ap.add_argument("--use-ros-test-node", action="store_true",
                    help="Start built-in ROS pose test node driven by MDP dynamics")
    ap.add_argument("--sim-rate", type=float, default=50.0,
                    help="Pose publish/simulation rate (Hz) for --use-ros-test-node")
    ap.add_argument("--sim-config-name", type=str, default="policy_convergence_tello_stage3",
                    help="Config used by ROS test pose node dynamics")
    ap.add_argument("--sim-mode", type=str, default="rollout", choices=["rollout", "cmd_dynamics"],
                    help="ROS test pose node mode: rollout (stable) or cmd_dynamics")
    args = ap.parse_args()

    rospy.init_node("tello_two_waypoints_auto_runner", anonymous=True)

    runner = TwoWaypointRunner(args)
    result = None
    code = 0
    mission_status = "success"
    error_message = ""
    try:
        result = runner.run()
        rospy.loginfo("Mission success: wp0_dist=%.2f, wp1_dist=%.2f", result["wp0_dist"], result["wp1_dist"])
    except Exception as e:
        code = 2
        mission_status = "failed"
        error_message = str(e)
        rospy.logerr("Mission failed: %s", e)
    finally:
        runner.shutdown()
        try:
            png_path, gif_path, metrics = save_plots(runner.samples, np.array(args.wp0), np.array(args.wp1), args.out_prefix)
            rospy.loginfo("Saved plot: %s", png_path)
            rospy.loginfo("Saved gif:  %s", gif_path)

            report = {
                "status": mission_status,
                "error": error_message,
                "wp0": [float(v) for v in args.wp0],
                "wp1": [float(v) for v in args.wp1],
                "reach_dist": float(args.reach_dist),
                "phase_min_dist": runner.phase_dmin,
                "metrics": metrics,
            }
            report_path = args.out_prefix + "_report.json"
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2)
            rospy.loginfo("Saved report: %s", report_path)
        except Exception as e:
            code = 3 if code == 0 else code
            rospy.logerr("Plotting failed: %s", e)

    sys.exit(code)


if __name__ == "__main__":
    main()
