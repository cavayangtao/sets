#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))
sys.path.insert(0, PROJECT_ROOT)

import rospy
from geometry_msgs.msg import PoseStamped
from util import util


def parse_target(text):
    parts = [float(x.strip()) for x in text.split(",")]
    if len(parts) != 3:
        raise ValueError("target must be 3 floats: x,y,z (planner frame)")
    return np.array(parts, dtype=np.float64)


def enu_to_planner(point_enu):
    # For mocap_frame=ENU in tello_planner_node: x,y pass-through, z sign flips.
    return np.array([point_enu[0], point_enu[1], -point_enu[2]], dtype=np.float64)


def point_to_aabb_distance(pt, box_min, box_max):
    d = np.maximum(0.0, np.maximum(box_min - pt, pt - box_max))
    return float(np.linalg.norm(d))


def compute_min_clearance(xs_planner, obstacles):
    if len(obstacles) == 0 or len(xs_planner) == 0:
        return float("inf")
    min_dist = float("inf")
    for p in xs_planner:
        for obs in obstacles:
            bmin = obs[0:3, 0]
            bmax = obs[0:3, 1]
            d = point_to_aabb_distance(p, bmin, bmax)
            min_dist = min(min_dist, d)
    return min_dist


def plot_xy(xs_planner, target_planner, obstacles, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 7))

    if len(xs_planner) > 0:
        arr = np.array(xs_planner)
        ax.plot(arr[:, 0], arr[:, 1], "b-", lw=2, label="trajectory")
        ax.scatter(arr[0, 0], arr[0, 1], c="g", s=60, label="start")
        ax.scatter(arr[-1, 0], arr[-1, 1], c="k", s=60, label="end")

    ax.scatter(target_planner[0], target_planner[1], c="r", s=90, marker="*", label="target")

    for i, obs in enumerate(obstacles):
        xmin, xmax = obs[0, 0], obs[0, 1]
        ymin, ymax = obs[1, 0], obs[1, 1]
        w = xmax - xmin
        h = ymax - ymin
        rect = plt.Rectangle((xmin, ymin), w, h, fill=False, edgecolor="m", linewidth=1.5)
        ax.add_patch(rect)
        if i == 0:
            rect.set_label("obstacle (xy projection)")

    ax.set_xlabel("x (planner frame)")
    ax.set_ylabel("y (planner frame)")
    ax.set_title("Single-Target Trajectory (Planner XY)")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Monitor single-target convergence and export trajectory plot/report")
    parser.add_argument("--pose-topic", default="/pose")
    parser.add_argument("--target", default="20,0,-12", help="planner frame target x,y,z")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--reach-dist", type=float, default=2.5)
    parser.add_argument("--hold-sec", type=float, default=2.0)
    parser.add_argument("--config-name", default="policy_convergence_tello_stage3")
    parser.add_argument("--out-prefix", required=True)
    args = parser.parse_args()

    target_planner = parse_target(args.target)

    cfg = util.load_yaml(util.get_config_path(args.config_name))
    obstacles = util.get_obstacles(cfg, 0)

    os.makedirs(os.path.dirname(args.out_prefix), exist_ok=True)
    out_csv = args.out_prefix + ".csv"
    out_png = args.out_prefix + ".png"
    out_json = args.out_prefix + "_report.json"

    rospy.init_node("monitor_single_target_convergence", anonymous=True)

    latest = {"pose": None, "stamp": None}

    def cb(msg):
        latest["pose"] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float64)
        latest["stamp"] = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

    sub = rospy.Subscriber(args.pose_topic, PoseStamped, cb, queue_size=20)

    t_wait = time.time() + 10.0
    while latest["pose"] is None and time.time() < t_wait and not rospy.is_shutdown():
        time.sleep(0.02)

    if latest["pose"] is None:
        print("No pose received on", args.pose_topic)
        return 2

    t0 = time.time()
    reached_since = None
    success = False
    rows = []
    xs_planner = []
    min_dist = float("inf")
    final_dist = None

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        now = time.time()
        elapsed = now - t0
        if elapsed > args.timeout:
            break

        p_enu = latest["pose"]
        if p_enu is None:
            rate.sleep()
            continue

        p_planner = enu_to_planner(p_enu)
        d = float(np.linalg.norm(p_planner - target_planner))
        min_dist = min(min_dist, d)
        final_dist = d

        rows.append([elapsed, p_enu[0], p_enu[1], p_enu[2], p_planner[0], p_planner[1], p_planner[2], d])
        xs_planner.append(p_planner.tolist())

        if d <= args.reach_dist:
            if reached_since is None:
                reached_since = now
            if now - reached_since >= args.hold_sec:
                success = True
                break
        else:
            reached_since = None

        rate.sleep()

    min_clearance = compute_min_clearance(xs_planner, obstacles)

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x_enu", "y_enu", "z_enu", "x_planner", "y_planner", "z_planner", "dist_to_target"]) 
        w.writerows(rows)

    plot_xy(xs_planner, target_planner, obstacles, out_png)

    report = {
        "success": success,
        "timeout_sec": args.timeout,
        "reach_dist": args.reach_dist,
        "hold_sec": args.hold_sec,
        "target_planner": target_planner.tolist(),
        "samples": len(rows),
        "min_dist": None if math.isinf(min_dist) else min_dist,
        "final_dist": final_dist,
        "min_obstacle_clearance": min_clearance,
        "csv": out_csv,
        "plot": out_png,
    }
    with open(out_json, "w") as f:
        json.dump(report, f, indent=2)

    print(json.dumps(report, ensure_ascii=True))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
