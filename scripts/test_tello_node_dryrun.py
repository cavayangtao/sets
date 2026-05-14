#!/usr/bin/env python3
"""
Verify the ROS node produces correct cmd_vel by feeding it the same initial
states as the offline dry-run and comparing the output actions.

Requires roscore + tello_planner node running:

    terminal 1: roscore
    terminal 2: rosrun sets tello_planner_node.py \
                    _uct_N:=500 _planner_rate:=10.0 _mocap_frame:=NED
    terminal 3: python3 test_tello_node_dryrun.py
"""

import sys, os, time, math, argparse
import numpy as np

_project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(_project_root, "src"))
sys.path.insert(0, _project_root)

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_srvs.srv import SetBool


def _to_pose_observable_state(state):
    """Project full MDP state into fields observable from PoseStamped.

    Observable in this test path:
      - position: p_x, p_y, p_z
      - yaw: psi

    Other terms are set to zero to match node-side reconstruction from pose.
    """
    obs = np.zeros_like(state, dtype=np.float64)
    obs[0:3] = state[0:3]
    obs[8] = state[8]
    if obs.shape[0] > 12:
        obs[12] = 0.0
    return obs


def _make_uct(get_uct2, uct_n=500):
    """Create a fresh UCT object so each case starts from a clean search tree."""
    uct = get_uct2()
    uct.set_param(uct_n, 12, 1200.0, 3.0,
                  False, False, False, False, False,
                  "shuffled", "uct", True, False)
    return uct


def _make_pose(state, mocap_frame):
    """MDP state -> PoseStamped in configured mocap_frame (inverse mapping)."""
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()

    if mocap_frame == "ENU":
        px, py, pz = state[0], state[1], -state[2]
        yaw = float(-state[8])
    elif mocap_frame == "NED":
        px, py, pz = state[0], state[1], state[2]
        yaw = float(state[8])
    else:
        raise ValueError(f"Unsupported mocap_frame for this test: {mocap_frame}")

    p.pose.position = Point(px, py, pz)
    cy = math.cos(yaw / 2.0); sy = math.sin(yaw / 2.0)
    p.pose.orientation = Quaternion(0, 0, sy, cy)
    return p


def run_verification(n_test_states=8, min_pass_ratio=0.35):
    """
    Run the offline dry-run to get reference actions, then feed the same
    initial states into the ROS node and compare cmd_vel outputs.
    """
    # ── Part A: offline dry-run (no ROS) ───────────────────────────────────
    from util import util
    from build.bindings import get_mdp, get_dots_mdp, get_uct2, RNG, run_uct2

    config_path = util.get_config_path("policy_convergence_tello_stage3")
    config = util.load_yaml(config_path)

    mdp = get_mdp("SixDOFAircraft", config_path)
    mdp.set_dt(0.01)
    mdp.clear_obstacles()
    for obs in util.get_obstacles(config, 0):
        mdp.add_obstacle(obs)

    dots = get_dots_mdp()
    dots.set_param(mdp,
        config["dots_expansion_mode"], config["dots_initialize_mode"],
        config["dots_special_actions"], config["dots_num_branches"],
        20, 5,
        config["dots_spectral_branches_mode"], config["dots_control_mode"],
        config["dots_scale_mode"], config["dots_modal_damping_mode"],
        config["dots_modal_damping_gains"],
        np.diag(np.array(config["dots_rho"])),
        config["dots_greedy_gain"], config["dots_greedy_rate"],
        config["dots_greedy_min_dist"], config["dots_baseline_mode_on"],
        config["dots_num_discrete_actions"], False)
    # Use a dedicated rollout stream only to generate diverse anchor poses.
    rollout_uct = _make_uct(get_uct2, uct_n=500)
    rollout_rng = RNG(); rollout_rng.set_seed(42)

    # collect reference actions from observable-state single-sample planning
    state_cursor = np.array(mdp.initial_state(), dtype=np.float64)
    reference = []  # (state, action)

    print("=== Part A: offline dry-run (observable-state reference) ===")
    for step in range(n_test_states):
        obs_state = _to_pose_observable_state(state_cursor)

        # Reset search/RNG per case to reduce cross-case stochastic drift.
        case_uct = _make_uct(get_uct2, uct_n=500)
        case_rng = RNG(); case_rng.set_seed(0)
        case_result = run_uct2(dots, case_uct, obs_state, case_rng)
        if not case_result.success:
            print(f"  step {step}: FAILED")
            break
        us = np.array(case_result.planned_traj.us)
        action = us[0, :4]
        reference.append((obs_state.copy(), action.copy()))
        print(f"  step {step}: pos=({obs_state[0]:6.1f},{obs_state[1]:5.1f},{obs_state[2]:5.1f}) "
              f"yaw={obs_state[8]:.2f} → vx={action[0]:.2f} vy={action[1]:.2f} "
              f"vz={action[2]:.2f} yr={action[3]:.2f}")

        # Advance an internal rollout cursor to generate the next anchor state.
        rollout_result = run_uct2(dots, rollout_uct, state_cursor, rollout_rng)
        if not rollout_result.success:
            break
        rollout_xs = np.array(rollout_result.planned_traj.xs)
        state_cursor = rollout_xs[min(3, rollout_xs.shape[0] - 1), :].copy()

    # ── Part B: ROS node verification ─────────────────────────────────────
    rospy.init_node("verify_ros", anonymous=True)
    rospy.wait_for_service("/tello_planner/arm", timeout=10)
    arm = rospy.ServiceProxy("/tello_planner/arm", SetBool)
    mocap_frame = rospy.get_param("/tello_planner/mocap_frame", "ENU")
    rospy.loginfo("Detected /tello_planner/mocap_frame=%s", mocap_frame)

    pose_pub = rospy.Publisher("/mocap/pose", PoseStamped, queue_size=1)
    latest_cmd = np.zeros(4)
    cmd_seq = {"n": 0}

    def cmd_cb(msg):
        latest_cmd[:] = [msg.linear.x, msg.linear.y,
                         msg.linear.z, msg.angular.z]
        cmd_seq["n"] += 1
    cmd_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_cb, queue_size=10)

    rospy.sleep(1.0)
    rospy.loginfo("=== Part B: ROS node (feeding same states) ===")

    results = []
    for i, (ref_state, ref_action) in enumerate(reference):
        # force a clean start for this case
        try:
            arm(data=False)
            rospy.sleep(0.2)
        except:
            pass

        rospy.loginfo("  Test %d: feeding pos=(%.1f,%.1f,%.1f) yaw=%.2f",
                      i, ref_state[0], ref_state[1], ref_state[2], ref_state[8])

        # reset observation for this case to avoid stale command carry-over
        latest_cmd[:] = 0.0
        seq0 = cmd_seq["n"]

        # warm up pose while planner is READY
        warm_t0 = time.time()
        while time.time() - warm_t0 < 0.5:
            p = _make_pose(ref_state, mocap_frame)
            p.header.stamp = rospy.Time.now()
            pose_pub.publish(p)
            rospy.sleep(0.02)

        # explicitly arm and capture one fresh planning sample
        try:
            arm(data=True)
        except:
            pass

        # publish continuously until we get the first NEW non-zero output
        t0 = time.time()
        got_new_nonzero = False
        while time.time() - t0 < 10.0:
            p = _make_pose(ref_state, mocap_frame)
            p.header.stamp = rospy.Time.now()
            pose_pub.publish(p)

            if cmd_seq["n"] > seq0 and any(abs(v) > 1e-6 for v in latest_cmd):
                got_new_nonzero = True
                # Stop immediately to enforce single-sample behavior.
                try:
                    arm(data=False)
                except:
                    pass
                break
            rospy.sleep(0.02)

        ros_action = latest_cmd.copy()
        results.append((ref_state, ref_action, ros_action, got_new_nonzero))

        # print comparison
        # Due to RNG divergence between offline and ROS node, exact match is
        # impossible. Check that ROS produces NON-ZERO outputs and keeps the
        # same sign on the dominant reference axis.
        dominant_idx = int(np.argmax(np.abs(ref_action[:4])))
        ref_dom = ref_action[dominant_idx]
        ros_dom = ros_action[dominant_idx]
        ref_sign = 1 if ref_dom > 0.1 else (-1 if ref_dom < -0.1 else 0)
        ros_sign = 1 if ros_dom > 0.1 else (-1 if ros_dom < -0.1 else 0)
        ros_nonzero = got_new_nonzero and any(abs(v) > 1e-6 for v in ros_action)
        sign_ok = (ref_sign == 0) or (ros_sign == ref_sign)

        marker = "✓" if (ros_nonzero and sign_ok) else "✗"
        rospy.loginfo("    ref:  vx=%6.2f vy=%6.2f vz=%6.2f yr=%6.2f", *ref_action)
        rospy.loginfo("    ros:  vx=%6.2f vy=%6.2f vz=%6.2f yr=%6.2f  nonzero=%s sign_ok=%s dom_idx=%d %s",
                  *ros_action, ros_nonzero, sign_ok, dominant_idx, marker)

    # ── Summary ──
    failures = []
    nonzero_cases = 0
    for i, (_s, ref, ros, got_new_nonzero) in enumerate(results):
        dominant_idx = int(np.argmax(np.abs(ref[:4])))
        ref_dom = ref[dominant_idx]
        ros_dom = ros[dominant_idx]
        ref_sign = 1 if ref_dom > 0.1 else (-1 if ref_dom < -0.1 else 0)
        ros_sign = 1 if ros_dom > 0.1 else (-1 if ros_dom < -0.1 else 0)
        if not got_new_nonzero or not any(abs(v) > 1e-6 for v in ros):
            failures.append(f"case{i}:no_new_nonzero_cmd")
            continue
        nonzero_cases += 1
        if ref_sign != 0 and ros_sign != ref_sign:
            failures.append(
                f"case{i}:dominant_sign_mismatch(axis={dominant_idx},ref={ref_sign},ros={ros_sign})"
            )

    passed_cases = len(results) - len(failures)
    pass_ratio = (float(passed_cases) / float(len(results))) if results else 0.0

    rospy.loginfo("=== Result: %d/%d cases passed, ratio=%.2f (threshold=%.2f) ===",
                  passed_cases, len(results), pass_ratio, min_pass_ratio)
    if failures:
        if pass_ratio < min_pass_ratio:
            rospy.logerr("FAILURES: %s", ", ".join(failures))
        else:
            rospy.logwarn("Non-fatal mismatches: %s", ", ".join(failures))

    # Also verify state machine
    from std_msgs.msg import String as SM
    try:
        status = rospy.wait_for_message("/tello_planner/status", SM, timeout=2)
        rospy.loginfo("Status: %s", status.data)
    except:
        pass

    if len(results) == 0:
        rospy.logerr("FAIL: no test cases were executed.")
        return False
    if nonzero_cases == 0:
        rospy.logerr("FAIL: ROS node never produced new non-zero commands.")
        return False
    if pass_ratio < min_pass_ratio:
        rospy.logerr("FAIL: pass ratio %.2f below threshold %.2f.", pass_ratio, min_pass_ratio)
        return False
    return True


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-test-states", type=int, default=8)
    ap.add_argument("--min-pass-ratio", type=float, default=0.35)
    args = ap.parse_args()

    ok = run_verification(n_test_states=args.n_test_states,
                          min_pass_ratio=args.min_pass_ratio)
    sys.exit(0 if ok else 1)
