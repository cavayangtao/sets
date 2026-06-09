#!/usr/bin/env python3
"""
Offline trajectory test: runs the planner in dry-run mode (identical to
policy_convergence's UCT-MPC loop), records state evolution via the MDP's
OWN dynamics, and plots/animates the result with obstacles.

This guarantees 100% consistency with the planner's dynamics model.
No ROS required.

Usage:
    python3 test_tello_trajectory.py
"""

import sys, os, time, argparse, numpy as np

_project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(_project_root, "src"))
sys.path.insert(0, _project_root)
from util import util
from build.bindings import get_mdp, get_dots_mdp, get_uct2, RNG, run_uct2


def load_obstacles():
    config = util.load_yaml(util.get_config_path("policy_convergence_tello_stage3"))
    obstacles = util.get_obstacles(config, 0)
    boxes = []
    for obs in obstacles:
        boxes.append(((obs[0, 0], obs[0, 1]),
                      (obs[1, 0], obs[1, 1]),
                      (obs[2, 0], obs[2, 1])))
    return boxes


def run_planner_loop(n_steps=40, uct_n=2000, waypoints=None):
    """
    Run UCT-MPC loop with multi-waypoint support.
    After reaching each waypoint (< 2m), switches to the next via mdp.set_xd().
    Returns (traj, waypoints_reached) where traj column -1 is the active waypoint index.
    """
    if waypoints is None:
        waypoints = [np.array([20.0, 0.0, -12.0])]

    config_path = util.get_config_path("policy_convergence_tello_stage3")
    config = util.load_yaml(config_path)

    mdp = get_mdp("SixDOFAircraft", config_path)
    mdp.set_dt(0.01)

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

    uct = get_uct2()
    uct.set_param(uct_n, 12, 1200.0, 3.0,
                  False, False, False, False, False,
                  "shuffled", "uct", True, False)
    rng = RNG(); rng.set_seed(42)

    # obstacles
    mdp.clear_obstacles()
    for obs in util.get_obstacles(config, 0):
        mdp.add_obstacle(obs)

    state = np.array(mdp.initial_state(), dtype=np.float64)
    traj = []
    wp_idx = 0
    current_target = waypoints[wp_idx]
    mdp.set_xd(np.array(list(current_target) + [0]*10, dtype=np.float64).reshape(-1, 1))
    print(f"Waypoint 0: target=({current_target[0]:.0f},{current_target[1]:.0f},{current_target[2]:.0f})")

    for plan_idx in range(n_steps):
        t0 = time.time()
        result = run_uct2(dots, uct, state, rng)
        elapsed = time.time() - t0

        if not result.success:
            print(f"Plan {plan_idx}: FAILED ({elapsed:.1f}s)")
            break

        xs = np.array(result.planned_traj.xs)
        us = np.array(result.planned_traj.us)

        mpc_steps = min(3, xs.shape[0] - 1)
        for k in range(mpc_steps):
            traj.append((plan_idx, k,
                         xs[k, 0], xs[k, 1], xs[k, 2], xs[k, 8],
                         us[k, 0], us[k, 1], us[k, 2], us[k, 3], wp_idx))
        state = xs[mpc_steps, :].copy()

        dist = np.linalg.norm(state[:3] - current_target)
        print(f"Plan {plan_idx:3d} ({elapsed:.1f}s): pos=({state[0]:6.1f},{state[1]:5.1f},{state[2]:5.1f}) "
              f"wp{wp_idx} dist={dist:.1f}m act=(vx={us[0,0]:.1f},vy={us[0,1]:.1f})")

        if dist < 2.0:
            wp_idx += 1
            if wp_idx >= len(waypoints):
                print(f"ARRIVED at final waypoint after {plan_idx + 1} plans!")
                break
            current_target = waypoints[wp_idx]
            xd = np.array(list(current_target) + [0]*10, dtype=np.float64).reshape(-1, 1)
            mdp.set_xd(xd)
            print(f"  -> switching to waypoint {wp_idx}: ({current_target[0]:.0f},{current_target[1]:.0f},{current_target[2]:.0f})")

    return np.array(traj)


# ── plotting ─────────────────────────────────────────────────────────────────

def _draw_obstacle_boxes(ax, boxes):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    for (xr, yr, zr) in boxes:
        x0, x1 = xr; y0, y1 = yr; z0, z1 = zr
        verts = np.array([
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        ])
        faces = [
            [verts[0], verts[1], verts[2], verts[3]],
            [verts[4], verts[5], verts[6], verts[7]],
            [verts[0], verts[1], verts[5], verts[4]],
            [verts[2], verts[3], verts[7], verts[6]],
            [verts[1], verts[2], verts[6], verts[5]],
            [verts[0], verts[3], verts[7], verts[4]],
        ]
        poly = Poly3DCollection(faces, alpha=0.25, facecolor="red",
                                edgecolor="darkred", linewidth=0.5)
        ax.add_collection3d(poly)


def plot_trajectory(traj, obstacles, waypoints, save_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # traj columns: plan_idx, micro_step, px, py, pz, yaw, vx_cmd, vy_cmd, vz_cmd, yr_cmd, wp_idx
    x, y, z = traj[:, 2], traj[:, 3], traj[:, 4]
    yaw = traj[:, 5]
    vx, vy, vz, yr = traj[:, 6], traj[:, 7], traj[:, 8], traj[:, 9]
    wp = traj[:, 10].astype(int) if traj.shape[1] > 10 else np.zeros(len(traj), dtype=int)
    t = np.arange(len(traj)) * 0.01

    wp_colors = ["blue", "orange", "green", "purple"]
    n_wp = len(waypoints)

    # ── static ──
    fig1, axes = plt.subplots(2, 3, figsize=(14, 8))

    for wi in range(n_wp):
        mask = wp == wi
        if mask.any():
            c = wp_colors[wi % len(wp_colors)]
            axes[0, 0].plot(x[mask], y[mask], color=c, alpha=0.7, linewidth=0.8,
                            label=f"wp{wi}→({waypoints[wi][0]:.0f},{waypoints[wi][1]:.0f})")
            axes[0, 1].plot(t[mask], x[mask], color=c, alpha=0.7, linewidth=0.6)
            axes[0, 1].plot(t[mask], y[mask], color=c, alpha=0.7, linewidth=0.6, linestyle="--")

    axes[0, 0].plot(x[0], y[0], "go", markersize=8, label="start")
    for wi, wp_target in enumerate(waypoints):
        axes[0, 0].plot(wp_target[0], wp_target[1], marker="*",
                        color=wp_colors[wi % len(wp_colors)], markersize=14,
                        markeredgecolor="black", markeredgewidth=0.5,
                        label=f"goal{wi}")
    from matplotlib.patches import Rectangle
    for (xr, yr_r, _) in obstacles:
        rect = Rectangle((xr[0], yr_r[0]), xr[1] - xr[0], yr_r[1] - yr_r[0],
                          alpha=0.25, facecolor="red", edgecolor="darkred", linewidth=0.8)
        axes[0, 0].add_patch(rect)
    axes[0, 0].set_xlabel("x (m)"); axes[0, 0].set_ylabel("y (m)")
    axes[0, 0].set_title("Top-down trajectory (multi-waypoint)"); axes[0, 0].axis("equal")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    axes[0, 0].legend(dict(zip(labels, handles)).values(), dict(zip(labels, handles)).keys(), fontsize=7)

    axes[0, 2].plot(t, z, "g-", alpha=0.7)
    for wi, wp_target in enumerate(waypoints):
        axes[0, 2].axhline(wp_target[2], color=wp_colors[wi % len(wp_colors)],
                           linestyle=":", alpha=0.5, linewidth=0.8)
    axes[0, 2].set_xlabel("time (s)"); axes[0, 2].set_ylabel("z (m)")
    axes[0, 2].set_title("Altitude vs time")

    axes[1, 0].plot(t, vx, alpha=0.7, linewidth=0.5, label="v_bx"); axes[1, 0].plot(t, vy, alpha=0.7, linewidth=0.5, label="v_by")
    axes[1, 0].set_xlabel("time (s)"); axes[1, 0].set_title("Body velocity"); axes[1, 0].legend()

    axes[1, 1].plot(t, vz, alpha=0.7, linewidth=0.5)
    axes[1, 1].set_xlabel("time (s)"); axes[1, 1].set_title("Vertical velocity cmd")

    axes[1, 2].plot(t, yr, alpha=0.7, linewidth=0.5, label="yr"); axes[1, 2].plot(t, yaw, alpha=0.7, linewidth=0.5, label="yaw")
    axes[1, 2].set_xlabel("time (s)"); axes[1, 2].set_title("Yaw"); axes[1, 2].legend()

    # mark waypoint arrival times
    for wi in range(1, n_wp):
        first_idx = np.where(wp == wi)[0]
        if len(first_idx) > 0:
            for ax_row in axes:
                for ax in ax_row:
                    ax.axvline(t[first_idx[0]], color=wp_colors[wi % len(wp_colors)],
                              linestyle="--", alpha=0.4, linewidth=0.8)

    fig1.tight_layout()
    static_path = save_path.replace(".gif", "_static.png")
    fig1.savefig(static_path, dpi=100)
    plt.close(fig1)
    print(f"Static plot: {static_path}")

    # ── 3D animation ──
    n_frames = min(300, len(traj))
    stride = max(1, len(traj) // n_frames)
    idxs = np.arange(0, len(traj), stride)
    x_s, y_s, z_s = x[idxs], y[idxs], z[idxs]
    vx_s, vy_s, vz_s = vx[idxs], vy[idxs], vz[idxs]
    wp_s = wp[idxs]

    fig2 = plt.figure(figsize=(10, 8))
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    ax = fig2.add_subplot(111, projection="3d")

    obs_x = [v for b in obstacles for v in b[0]]
    obs_y = [v for b in obstacles for v in b[1]]
    obs_z = [v for b in obstacles for v in b[2]]
    x_all = np.concatenate([x, obs_x] + [[wp[0] for wp in waypoints]])
    y_all = np.concatenate([y, obs_y] + [[wp[1] for wp in waypoints]])
    z_all = np.concatenate([z, obs_z] + [[wp[2] for wp in waypoints]])
    pad = 3.0
    ax.set_xlim(x_all.min() - pad, x_all.max() + pad)
    ax.set_ylim(y_all.min() - pad, y_all.max() + pad)
    ax.set_zlim(z_all.min() - pad, z_all.max() + pad)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title("Drone Trajectory (multi-waypoint)")

    _draw_obstacle_boxes(ax, obstacles)
    ax.scatter(x[0], y[0], z[0], c="green", s=80, marker="o")
    for wi, wp_target in enumerate(waypoints):
        ax.scatter(wp_target[0], wp_target[1], wp_target[2],
                   c=wp_colors[wi % len(wp_colors)], s=100, marker="*",
                   edgecolors="black", linewidth=0.5)

    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    legend_items = [
        Patch(facecolor="red", alpha=0.25, edgecolor="darkred", linewidth=0.5, label="obstacles"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="green", markersize=8, label="start"),
    ]
    for wi, wp_target in enumerate(waypoints):
        legend_items.append(
            Line2D([0], [0], marker="*", color="w",
                   markerfacecolor=wp_colors[wi % len(wp_colors)],
                   markersize=10, label=f"wp{wi}"))
    ax.legend(handles=legend_items, loc="upper left")

    trail, = ax.plot([], [], [], "b-", alpha=0.4, linewidth=0.8)
    dot, = ax.plot([], [], [], "bo", markersize=8)
    arrow = [None]

    def init():
        trail.set_data([], []); trail.set_3d_properties([])
        dot.set_data([], []); dot.set_3d_properties([])
        return trail, dot

    def update(i):
        trail.set_data(x_s[:i + 1], y_s[:i + 1])
        trail.set_3d_properties(z_s[:i + 1])
        trail.set_color(wp_colors[wp_s[i] % len(wp_colors)])
        dot.set_data([x_s[i]], [y_s[i]]); dot.set_3d_properties([z_s[i]])
        dot.set_color(wp_colors[wp_s[i] % len(wp_colors)])
        if arrow[0] is not None:
            arrow[0].remove()
        s = 2.0
        arrow[0] = ax.quiver(x_s[i], y_s[i], z_s[i],
                             vx_s[i] * s, vy_s[i] * s, vz_s[i] * s,
                             color="orange", alpha=0.8, linewidth=1.5)
        return trail, dot, arrow[0]

    ani = FuncAnimation(fig2, update, frames=len(idxs),
                        init_func=init, blit=False, interval=60)
    ani.save(save_path, writer="pillow", fps=15, dpi=80)
    plt.close(fig2)
    print(f"Animation: {save_path}")


# ── entry ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=40)
    ap.add_argument("--n", type=int, default=2000, help="UCT simulations per plan")
    ap.add_argument("--output", type=str,
                    default="/home/tyang/sets/plots/tello_trajectory.gif")
    args = ap.parse_args()

    # waypoints: first matches launch target_pos, second is a new target set at runtime
    waypoints = [
        np.array([20.0,  0.0, -12.0]),   # wp0: launch default target
        np.array([20.0, 10.0, -12.0]),   # wp1: new target set after arrival
    ]

    print("Loading obstacles and running planner...")
    obstacles = load_obstacles()
    print(f"Loaded {len(obstacles)} obstacles. Waypoints: {len(waypoints)}")

    traj = run_planner_loop(n_steps=args.steps, uct_n=args.n, waypoints=waypoints)

    if len(traj) > 0:
        if len(traj) > 5000:
            stride = len(traj) // 5000 + 1
            traj = traj[::stride]
            print(f"Subsampled to {len(traj)} points for plotting.")
        plot_trajectory(traj, obstacles, waypoints, save_path=args.output)
        final = traj[-1]
        last_wp = waypoints[-1]
        dist = np.linalg.norm(final[2:5] - last_wp)
        print(f"Final: x={final[2]:.1f} y={final[3]:.1f} z={final[4]:.1f}  dist_to_last_goal={dist:.1f}m")
    else:
        print("No trajectory data!"); sys.exit(1)
