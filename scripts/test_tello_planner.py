#!/usr/bin/env python3
"""
Smoke test for tello_planner_node.py — no hardware required.

Publishes fake MoCap poses, watches /cmd_vel, exercises arm/estop services.
Run alongside the planner node:

    terminal 1: roscore
    terminal 2: rosrun sets tello_planner_node.py _uct_N:=200 _planner_rate:=10.0
    terminal 3: rosrun sets test_tello_planner.py
"""

import sys
import time
import math
import threading

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_srvs.srv import SetBool, Trigger


def _make_pose(x, y, z, yaw=0.0):
    """Build a PoseStamped. z is ENU altitude (positive up);
    the planner's transform_pose flips sign to NED internally."""
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "world"
    p.pose.position = Point(x, y, z)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    p.pose.orientation = Quaternion(0, 0, sy, cy)
    return p


def test_smoke():
    rospy.init_node("test_tello_planner", anonymous=True)

    # wait for planner to come up
    rospy.loginfo("Waiting for tello_planner services...")
    rospy.wait_for_service("/tello_planner/arm", timeout=10)
    rospy.wait_for_service("/tello_planner/estop", timeout=10)
    arm = rospy.ServiceProxy("/tello_planner/arm", SetBool)
    estop = rospy.ServiceProxy("/tello_planner/estop", Trigger)

    pose_pub = rospy.Publisher("/mocap/pose", PoseStamped, queue_size=10)

    # collect cmd_vel messages
    cmds = []
    cmd_lock = threading.Lock()

    def cmd_cb(msg):
        with cmd_lock:
            cmds.append((msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z))

    cmd_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_cb, queue_size=10)

    # collect status messages
    statuses = []
    def status_cb(msg):
        statuses.append(msg.data)

    from std_msgs.msg import String as StringMsg
    status_sub = rospy.Subscriber("/tello_planner/status", StringMsg, status_cb, queue_size=10)

    rospy.sleep(1.0)  # let connections establish

    # ── Phase 1: publish fake pose, verify transition INIT → READY → RUNNING ──
    rospy.loginfo("=== Phase 1: publishing fake poses ===")
    for step in range(10):
        p = _make_pose(-20.0 + step * 0.2, 0.0, 12.0, yaw=0.0)
        pose_pub.publish(p)
        rospy.sleep(0.2)

    # give planner time to run one cycle (N=200 should be ~0.3s)
    rospy.loginfo("Waiting for first plan (N=200, ~0.3s)...")
    rospy.sleep(5.0)

    with cmd_lock:
        nonzero = [c for c in cmds if any(abs(v) > 1e-6 for v in c)]
    rospy.loginfo("Total cmd_vel msgs: %d, nonzero: %d", len(cmds), len(nonzero))

    if len(statuses) >= 3:
        rospy.loginfo("Status samples: %s ... %s", statuses[0], statuses[-1])

    # ── Phase 2: test estop ──
    rospy.loginfo("=== Phase 2: testing estop ===")
    with cmd_lock:
        before_estop = len(cmds)
    resp = estop()
    rospy.loginfo("Estop response: %s", resp.message)
    rospy.sleep(2.0)
    with cmd_lock:
        after_estop = len(cmds)
        recent = cmds[before_estop:] if before_estop < len(cmds) else []
        all_zero = all(all(abs(v) < 1e-9 for v in c) for c in recent) if recent else True
    rospy.loginfo("Post-estop cmds all zero: %s (%d msgs)", all_zero, len(recent))

    # ── Phase 3: recover from FAILSAFE via disarm ──
    rospy.loginfo("=== Phase 3: disarm to recover ===")
    resp = arm(data=False)
    rospy.loginfo("Disarm response: %s", resp.message)
    rospy.sleep(0.5)
    resp = arm(data=True)
    rospy.loginfo("Re-arm response: %s", resp.message)
    rospy.sleep(5.0)

    # ── Summary ──
    rospy.loginfo("=== Summary ===")
    with cmd_lock:
        nonzero = [c for c in cmds if any(abs(v) > 1e-6 for v in c)]
        if nonzero:
            c0, cn = nonzero[0], nonzero[-1]
            rospy.loginfo("First nonzero cmd:  vx=%.2f vy=%.2f vz=%.2f yaw=%.2f", *c0)
            rospy.loginfo("Last  nonzero cmd:  vx=%.2f vy=%.2f vz=%.2f yaw=%.2f", *cn)
        rospy.loginfo("Cmd msgs total: %d, nonzero: %d", len(cmds), len(nonzero))

    passed = len(nonzero) > 0
    rospy.loginfo("SMOKE TEST %s", "PASSED" if passed else "FAILED")
    return passed


if __name__ == "__main__":
    ok = test_smoke()
    sys.exit(0 if ok else 1)
