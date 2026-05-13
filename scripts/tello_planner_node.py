#!/usr/bin/env python3
"""
tello_planner_node.py — ROS node for online Tello drone control via UCT-MPC planning.

Architecture (Plan C):
  - Imports C++ bindings directly (get_mdp, get_dots_mdp, get_uct2, run_uct2).
  - Does NOT modify rollout.py or policy_convergence.py.
  - Creates MDP/DOTS/UCT objects once at init; reuses them per cycle.
  - Dual-rate: planner timer (low Hz) + command republish timer (high Hz).
  - Safety state machine: INIT → READY → RUNNING → FAILSAFE.

Input:  motion capture pose  (geometry_msgs/PoseStamped or custom)
Output: cmd_vel              (geometry_msgs/Twist)
"""

import sys
import os
import time
import math
import threading
import traceback
from enum import Enum

import numpy as np

# ── project imports ──────────────────────────────────────────────────────────
_project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(_project_root, "src"))
sys.path.insert(0, _project_root)
from util import util
from build.bindings import get_mdp, get_dots_mdp, get_uct2, RNG, run_uct2

# ── ROS imports (rospy / ROS 1) ──────────────────────────────────────────────
try:
    import rospy
    from geometry_msgs.msg import Twist, PoseStamped
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
except ImportError:
    rospy = None


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  State Machine                                                              ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

class State(Enum):
    INIT     = "INIT"       # waiting for first valid mocap pose
    READY    = "READY"      # mocap streaming; waiting for arm
    RUNNING  = "RUNNING"    # planning + publishing commands
    FAILSAFE = "FAILSAFE"   # timeout / error → publish zero command


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Frame / Convention Helpers                                                 ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def quaternion_to_rpy(q):
    """Convert geometry_msgs/Quaternion to (roll, pitch, yaw) in radians."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def transform_pose(px, py, pz, roll, pitch, yaw, frame_config):
    """
    Apply configurable frame transform to bring mocap coordinates into
    the planner's world frame (NED-like: z positive down).

    frame_config keys:
        mocap_frame: "ENU" | "NED" | "NED_ZY"
        mocap_to_planner_rz_deg: rotation around Z in degrees (applied first)
        mocap_to_planner_scale_xyz: [sx, sy, sz] scaling factors
    """
    scale = np.array(frame_config.get("mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]))
    rz_deg = frame_config.get("mocap_to_planner_rz_deg", 0.0)

    # scale
    px, py, pz = px * scale[0], py * scale[1], pz * scale[2]

    # rotate around Z
    if abs(rz_deg) > 1e-9:
        rad = math.radians(rz_deg)
        c, s = math.cos(rad), math.sin(rad)
        px, py = c * px - s * py, s * px + c * py
        yaw = yaw + rad

    # frame hand-ness
    mocap_frame = frame_config.get("mocap_frame", "ENU")
    if mocap_frame == "ENU":
        pz = -pz
        pitch = -pitch
        yaw = -yaw
    elif mocap_frame == "NED_ZY":
        px, py, pz = px, pz, py
        roll, pitch, yaw = roll, yaw, pitch
    # "NED" → pass through

    return px, py, pz, roll, pitch, yaw


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Planner Node                                                               ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

class TelloPlannerNode:
    def __init__(self):
        # ── ROS node init ─────────────────────────────────────────────────
        if rospy is None:
            raise ImportError("rospy is not available. Install ROS 1 (rospy).")
        rospy.init_node("tello_planner", anonymous=False)
        self._load_params()

        # ── planner objects (created ONCE) ─────────────────────────────────
        self._init_planner()

        # ── runtime state ──────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._state = State.INIT

        # latest mocap data (populated by subscriber)
        self._mocap_pos = None
        self._mocap_quat = None
        self._mocap_stamp = None

        # velocity estimation (finite-difference fallback)
        self._prev_pos = None
        self._prev_stamp = None
        self._est_vel = np.zeros(3)

        # latest planned command (body-frame vx, vy, vz, yaw_rate)
        self._cmd = np.zeros(4)
        self._cmd_stamp = rospy.Time.now()
        self._planning_active = False
        self._plan_success = False
        self._plan_latency = 0.0
        self._plan_count = 0

        # timestep counter (maps to planner state dim -1)
        self._timestep = 0

        # ── ROS interfaces ────────────────────────────────────────────────
        self._setup_ros()

        rospy.loginfo("TelloPlannerNode initialized. State: %s", self._state.value)

    # ── parameter loading ─────────────────────────────────────────────────────

    def _load_params(self):
        """Load all runtime parameters from ROS param server."""
        # planner config
        self._config_name = rospy.get_param("~config_name", "policy_convergence_tello_stage3")
        self._config_path = util.get_config_path(self._config_name)
        self._config = util.load_yaml(self._config_path)

        # topics
        self._pose_topic = rospy.get_param("~pose_topic", "/mocap/pose")
        self._cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self._status_topic = rospy.get_param("~status_topic", "/tello_planner/status")

        # rates
        self._planner_rate = rospy.get_param("~planner_rate", 2.0)   # Hz
        self._cmd_pub_rate = rospy.get_param("~cmd_pub_rate", 30.0)  # Hz

        # planner overrides (reduce N for online use)
        self._uct_N = rospy.get_param("~uct_N", 500)
        self._uct_max_depth = rospy.get_param("~uct_max_depth", self._config.get("uct_max_depth", 12))
        self._uct_c = rospy.get_param("~uct_c", self._config.get("uct_c", 3.0))
        self._uct_wct = rospy.get_param("~uct_wct", self._config.get("uct_wct", 1200.0))
        self._uct_mpc_depth = rospy.get_param("~uct_mpc_depth", self._config.get("uct_mpc_depth", 2))
        self._dots_decision_making_horizon = rospy.get_param(
            "~dots_decision_making_horizon", self._config.get("dots_decision_making_horizon", 20))
        self._dots_dynamics_horizon = rospy.get_param(
            "~dots_dynamics_horizon", self._config.get("dots_dynamics_horizon", 5))
        self._uct_dt = rospy.get_param("~uct_dt", self._config.get("uct_dt", 0.01))

        # safety
        self._auto_arm = rospy.get_param("~auto_arm", True)              # if True, auto-transition to RUNNING on first pose
        self._pose_timeout = rospy.get_param("~pose_timeout", 2.0)       # seconds; should be > plan latency
        self._max_cmd_hold = rospy.get_param("~max_cmd_hold", 3.0)       # seconds; stop if no new plan
        self._clip_cmd = rospy.get_param("~clip_cmd", True)
        # cmd_vel limits: read from YAML ground_mdp_U as source of truth; ROS param can override
        _U = np.array(self._config.get("ground_mdp_U", [])).reshape(2, -1, order="F")
        _default_lim = _U[1, :4] if _U.size >= 8 else [7.0, 7.0, 2.5, 1.8]
        self._vx_max = rospy.get_param("~vx_max", float(_default_lim[0]))
        self._vy_max = rospy.get_param("~vy_max", float(_default_lim[1]))
        self._vz_max = rospy.get_param("~vz_max", float(_default_lim[2]))
        self._yaw_rate_max = rospy.get_param("~yaw_rate_max", float(_default_lim[3]))

        # frame transform
        self._frame_config = {
            "mocap_frame": rospy.get_param("~mocap_frame", "ENU"),
            "mocap_to_planner_rz_deg": rospy.get_param("~mocap_to_planner_rz_deg", 0.0),
            "mocap_to_planner_scale_xyz": rospy.get_param(
                "~mocap_to_planner_scale_xyz", [1.0, 1.0, 1.0]),
        }

        # pose message type
        self._pose_msg_type = rospy.get_param("~pose_msg_type", "PoseStamped")

        # target (can be overridden from config)
        self._target_pos = rospy.get_param("~target_pos",
            self._config.get("ground_mdp_xd", [20.0, 0.0, -12.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        # seed
        self._seed = rospy.get_param("~seed", 0)

        # log
        rospy.loginfo("Params: config=%s, N=%d, planner_rate=%.1f Hz, cmd_rate=%.1f Hz",
                      self._config_name, self._uct_N, self._planner_rate, self._cmd_pub_rate)

    # ── planner init (one-time) ───────────────────────────────────────────────

    def _init_planner(self):
        """Create MDP, DOTS, and UCT objects. Called once at startup."""
        config = self._config
        config_path = self._config_path

        self._ground_mdp = get_mdp(config["ground_mdp_name"], config_path)
        self._ground_mdp.set_dt(self._uct_dt)

        # dots
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

        # uct
        self._uct = get_uct2()
        self._uct.set_param(
            self._uct_N,
            self._uct_max_depth,
            self._uct_wct,
            self._uct_c,
            False,   # export_topology
            False,   # export_node_states
            False,   # export_trajs
            False,   # export_cbdsbds
            False,   # export_tree_statistics
            config["uct_heuristic_mode"],
            config["uct_tree_exploration"],
            config["uct_downsample_traj_on"],
            False,   # verbose
        )

        # rng
        self._rng = RNG()
        self._rng.set_seed(self._seed)

        # init obstacles from config
        obstacles = util.get_obstacles(config, 0)
        self._ground_mdp.clear_obstacles()
        for obs in obstacles:
            self._ground_mdp.add_obstacle(obs)

        rospy.loginfo("Planner objects created: MDP state_dim=%d, N=%d, obstacles=%d",
                      self._ground_mdp.state_dim(), self._uct_N, len(obstacles))

    # ── ROS setup ─────────────────────────────────────────────────────────────

    def _setup_ros(self):
        # subscriber
        if self._pose_msg_type == "PoseStamped":
            self._pose_sub = rospy.Subscriber(
                self._pose_topic, PoseStamped, self._pose_callback, queue_size=1)
        else:
            rospy.logerr("Unsupported pose_msg_type: %s", self._pose_msg_type)
            raise ValueError(f"Unsupported pose_msg_type: {self._pose_msg_type}")

        # publisher
        self._cmd_pub = rospy.Publisher(self._cmd_vel_topic, Twist, queue_size=1)
        self._status_pub = rospy.Publisher(self._status_topic, String, queue_size=10)

        # timers
        self._planner_timer = rospy.Timer(
            rospy.Duration(1.0 / self._planner_rate), self._planner_callback)
        self._cmd_timer = rospy.Timer(
            rospy.Duration(1.0 / self._cmd_pub_rate), self._cmd_publish_callback)
        self._watchdog_timer = rospy.Timer(
            rospy.Duration(0.1), self._watchdog_callback)

        # services
        self._arm_srv = rospy.Service("~arm", SetBool, self._arm_callback)
        self._estop_srv = rospy.Service("~estop", Trigger, self._estop_callback)
        self._update_target_srv = rospy.Service("~update_target", Trigger, self._update_target_callback)

    # ── mocap callback ────────────────────────────────────────────────────────

    def _pose_callback(self, msg):
        """Receive mocap pose, update latest state, run velocity estimation."""
        with self._lock:
            self._mocap_pos = (msg.pose.position.x,
                               msg.pose.position.y,
                               msg.pose.position.z)
            self._mocap_quat = msg.pose.orientation
            self._mocap_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

            # velocity estimation (if not provided by mocap)
            now = self._mocap_stamp.to_sec()
            pos = np.array(self._mocap_pos)
            if self._prev_pos is not None and self._prev_stamp is not None:
                dt = now - self._prev_stamp
                if dt > 1e-6:
                    self._est_vel = (pos - self._prev_pos) / dt
            self._prev_pos = pos
            self._prev_stamp = now

            # auto-transition INIT → READY on first pose
            if self._state == State.INIT:
                self._state = State.READY
                rospy.loginfo("First pose received → READY")

    # ── planner timer (low rate) ──────────────────────────────────────────────

    def _planner_callback(self, event):
        """Run one UCT planning cycle using latest mocap position as initial state."""
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

            # build planner state vector from mocap
            r, p, y = quaternion_to_rpy(quat)
            px, py, pz, roll, pitch, yaw = transform_pose(
                pos[0], pos[1], pos[2], r, p, y, self._frame_config)

            state = np.zeros(self._ground_mdp.state_dim(), dtype=np.float64)
            state[0]  = px
            state[1]  = py
            state[2]  = pz
            state[3]  = est_vel[0]
            state[4]  = est_vel[1]
            state[5]  = est_vel[2]
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

            # run planner
            result = run_uct2(self._dots_mdp, self._uct, state, self._rng)
            t_elapsed = time.time() - t_start

            with self._lock:
                self._plan_latency = t_elapsed
                self._plan_count += 1

                if result.success:
                    us = np.array(result.planned_traj.us)
                    self._cmd = self._saturate_command(us[0, :])
                    self._cmd_stamp = rospy.Time.now()
                    self._plan_success = True
                    self._timestep += 1
                    if self._state == State.READY:
                        self._state = State.RUNNING
                        rospy.loginfo("First plan succeeded (auto-arm) → RUNNING")
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

    # ── command publish timer (high rate) ─────────────────────────────────────

    def _cmd_publish_callback(self, event):
        """Publish current command at high rate. Holds last command during planning."""
        twist = Twist()

        if self._state == State.RUNNING:
            with self._lock:
                cmd = self._cmd.copy()
            twist.linear.x  = cmd[0]
            twist.linear.y  = cmd[1]
            twist.linear.z  = cmd[2]
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = cmd[3]

        elif self._state == State.FAILSAFE:
            # zero command
            pass

        else:
            # INIT / READY → publish zero
            pass

        self._cmd_pub.publish(twist)

    # ── watchdog timer ────────────────────────────────────────────────────────

    def _watchdog_callback(self, event):
        """Monitor pose freshness and command age; trigger FAILSAFE if stale."""
        now = rospy.Time.now()

        with self._lock:
            pose_stamp = self._mocap_stamp
            cmd_stamp = self._cmd_stamp
            state = self._state

        # pose timeout
        if pose_stamp is not None:
            dt = (now - pose_stamp).to_sec()
            if dt > self._pose_timeout and state == State.RUNNING:
                rospy.logerr("Pose timeout (%.3fs > %.3fs) → FAILSAFE", dt, self._pose_timeout)
                with self._lock:
                    self._state = State.FAILSAFE
                state = State.FAILSAFE

        # command timeout
        dt_cmd = (now - cmd_stamp).to_sec()
        if dt_cmd > self._max_cmd_hold and state == State.RUNNING:
            rospy.logerr("Command hold timeout (%.3fs > %.3fs) → FAILSAFE",
                         dt_cmd, self._max_cmd_hold)
            with self._lock:
                self._state = State.FAILSAFE
            state = State.FAILSAFE

        self._publish_status()

    # ── command saturation ────────────────────────────────────────────────────

    def _saturate_command(self, action):
        """Clip action to configured cmd_vel bounds."""
        if not self._clip_cmd:
            return np.array(action[:4], dtype=np.float64)
        return np.array([
            np.clip(action[0], -self._vx_max, self._vx_max),
            np.clip(action[1], -self._vy_max, self._vy_max),
            np.clip(action[2], -self._vz_max, self._vz_max),
            np.clip(action[3], -self._yaw_rate_max, self._yaw_rate_max),
        ], dtype=np.float64)

    # ── services ──────────────────────────────────────────────────────────────

    def _arm_callback(self, req):
        with self._lock:
            if req.data:
                if self._state == State.READY:
                    self._state = State.RUNNING
                    return SetBoolResponse(success=True, message="Armed → RUNNING")
                else:
                    return SetBoolResponse(success=False,
                        message=f"Cannot arm from state {self._state.value}")
            else:
                self._state = State.READY
                self._timestep = 0
                self._cmd = np.zeros(4)
                return SetBoolResponse(success=True, message="Disarmed → READY")

    def _estop_callback(self, req):
        with self._lock:
            self._state = State.FAILSAFE
            self._cmd = np.zeros(4)
        rospy.logwarn("ESTOP triggered → FAILSAFE")
        return TriggerResponse(success=True, message="ESTOP: state → FAILSAFE")

    def _update_target_callback(self, req):
        """Read ~target_pos from param server and update MDP target.
        Usage:
            rosparam set /tello_planner/target_pos "[x,y,z,...]"
            rosservice call /tello_planner/update_target
        Or in one step via command line (uses param + trigger):
            rosparam set ... && rosservice call ...
        """
        new_target = rospy.get_param("~target_pos", None)
        if new_target is None:
            return TriggerResponse(success=False,
                                   message="~target_pos param not set. Use: rosparam set /tello_planner/target_pos \"[...]\"")
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

    # ── status ────────────────────────────────────────────────────────────────

    def _publish_status(self):
        msg = String()
        msg.data = (
            f"state:{self._state.value}"
            f"|plan_count:{self._plan_count}"
            f"|plan_latency_ms:{self._plan_latency * 1000:.0f}"
            f"|plan_success:{self._plan_success}"
            f"|cmd:[{self._cmd[0]:.2f},{self._cmd[1]:.2f},{self._cmd[2]:.2f},{self._cmd[3]:.2f}]"
            f"|timestep:{self._timestep}"
        )
        self._status_pub.publish(msg)

    # ── run ───────────────────────────────────────────────────────────────────

    def spin(self):
        rospy.loginfo("TelloPlannerNode spinning. State: %s", self._state.value)
        rospy.spin()


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Command-line fallback (no-ROS test)                                        ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def dry_run_test():
    """Offline test: plan a few steps with simulated state updates."""
    print("=== Dry-run test (no ROS) ===")

    config_path = util.get_config_path("policy_convergence_tello_stage3")
    config = util.load_yaml(config_path)

    mdp = get_mdp("SixDOFAircraft", config_path)
    mdp.set_dt(0.01)
    dots = get_dots_mdp()
    dots.set_param(mdp, config["dots_expansion_mode"], config["dots_initialize_mode"],
        config["dots_special_actions"], config["dots_num_branches"], 20, 5,
        config["dots_spectral_branches_mode"], config["dots_control_mode"],
        config["dots_scale_mode"], config["dots_modal_damping_mode"],
        config["dots_modal_damping_gains"], np.diag(np.array(config["dots_rho"])),
        config["dots_greedy_gain"], config["dots_greedy_rate"],
        config["dots_greedy_min_dist"], config["dots_baseline_mode_on"],
        config["dots_num_discrete_actions"], False)
    uct = get_uct2()
    uct.set_param(500, 12, 1200.0, 3.0, False, False, False, False, False,
                  "shuffled", "uct", True, False)
    rng = RNG()
    rng.set_seed(0)

    x0 = np.array(mdp.initial_state())

    for step in range(5):
        t0 = time.time()
        result = run_uct2(dots, uct, x0, rng)
        elapsed = time.time() - t0
        if result.success:
            us = np.array(result.planned_traj.us)
            action = us[0, :]
            print(f"step {step}: latency={elapsed:.3f}s, "
                  f"vx={action[0]:.2f}, vy={action[1]:.2f}, vz={action[2]:.2f}, yaw_rate={action[3]:.2f}")
            # simulate motion: step the position forward by first action * dt
            xs = np.array(result.planned_traj.xs)
            x0 = xs[min(1, xs.shape[0] - 1), :].copy()
            x0[-1] = float(step + 1)
        else:
            print(f"step {step}: plan FAILED ({elapsed:.3f}s)")
            break

    print("=== Dry-run complete ===")


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Main                                                                       ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

if __name__ == "__main__":
    if "--dry-run" in sys.argv:
        dry_run_test()
    elif rospy is not None:
        try:
            node = TelloPlannerNode()
            node.spin()
        except rospy.ROSInterruptException:
            pass
    else:
        print("rospy not available. Use --dry-run for offline test, or install ROS.")
        sys.exit(1)
