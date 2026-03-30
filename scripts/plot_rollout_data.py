"""
加载 data/ 目录下已有的 rollout_result_policy_convergence_*.pkl 文件，
可视化飞行器 3D 轨迹、状态量和控制输入。
无需编译 C++ 扩展即可运行。

用法:
    python plot_rollout_data.py                         # 绘制全部10组
    python plot_rollout_data.py --idx 0                 # 只绘制第0组
    python plot_rollout_data.py --idx 0 2 5             # 绘制第0、2、5组
    python plot_rollout_data.py --show                  # 弹窗显示而非保存PDF
"""

import sys, os, glob, argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from util import util


def plot_3d_cube(points, ax, color, alpha):
    """绘制3D立方体, points: (3, 2) 每行是 [min, max]"""
    xlims, ylims, zlims = points[0,:], points[1,:], points[2,:]
    xx, yy = np.meshgrid(xlims, ylims)
    for z in zlims:
        ax.plot_surface(xx, yy, z * np.ones((2,2)), alpha=alpha, color=color)
    xx, zz = np.meshgrid(xlims, zlims)
    for y in ylims:
        ax.plot_surface(xx, y * np.ones((2,2)), zz, alpha=alpha, color=color)
    yy, zz = np.meshgrid(ylims, zlims)
    for x in xlims:
        ax.plot_surface(x * np.ones((2,2)), yy, zz, alpha=alpha, color=color)


def plot_thermals_and_obstacles(config, ax):
    """在3D图上绘制热流区域和障碍物"""
    # 热流区域 (橙色半透明)
    if "Xs_thermal" in config:
        for xs_t in config["Xs_thermal"]:
            X_thermal = np.reshape(np.array(xs_t), (13, 2), order="F")
            bounds = X_thermal[0:3, :].copy()
            bounds[2, :] = -1 * bounds[2, :]  # z轴翻转
            if bounds[2, 0] > bounds[2, 1]:
                bounds[2, :] = bounds[2, ::-1]
            plot_3d_cube(bounds, ax, "orange", 0.1)

    # 障碍物 (黑色半透明)
    if "ground_mdp_obstacles" in config:
        for obs in config["ground_mdp_obstacles"]:
            obs_arr = np.reshape(np.array(obs), (13, 2), order="F")
            bounds = obs_arr[0:3, :].copy()
            bounds[2, :] = -1 * bounds[2, :]
            if bounds[2, 0] > bounds[2, 1]:
                bounds[2, :] = bounds[2, ::-1]
            plot_3d_cube(bounds, ax, "black", 0.1)


def load_results(data_dir, idxs=None):
    fns = sorted(glob.glob(os.path.join(data_dir, "rollout_result_policy_convergence_*.pkl")))
    if not fns:
        print("未找到 rollout_result_policy_convergence_*.pkl 文件")
        sys.exit(1)
    if idxs is not None:
        fns = [fns[i] for i in idxs if i < len(fns)]
    results = []
    for fn in fns:
        print(f"加载: {fn}")
        results.append((fn, util.load_pickle(fn)))
    return results


def plot_3d_trajectory(result, title, fig, ax):
    xs = np.array(result["rollout_xs"])
    config = result.get("config_dict", {})

    if xs.ndim < 2 or xs.shape[0] == 0:
        ax.set_title(f"{title} (no data)")
        return

    # 绘制热流区域和障碍物
    plot_thermals_and_obstacles(config, ax)

    ax.plot(xs[:, 0], xs[:, 1], -1 * xs[:, 2], alpha=0.8)
    ax.scatter(xs[0, 0], xs[0, 1], -1 * xs[0, 2], marker='o', c='green', s=50, label='start')
    ax.scatter(xs[-1, 0], xs[-1, 1], -1 * xs[-1, 2], marker='s', c='red', s=50, label='end')

    # 绘制目标点
    for t in config.get("targets", []):
        ax.scatter(t[0], t[1], -1 * t[2], marker='*', c='gold', s=120, zorder=5)

    # 设置状态空间边界（与原始代码一致）
    if "ground_mdp_X" in config:
        state_lims = np.reshape(np.array(config["ground_mdp_X"]), (13, 2), order="F")
        state_lims[2, :] = -1 * state_lims[2, :]
        if state_lims[2, 0] > state_lims[2, 1]:
            state_lims[2, :] = state_lims[2, ::-1]
        ax.set_xlim([state_lims[0, 0], state_lims[0, 1]])
        ax.set_ylim([state_lims[1, 0], state_lims[1, 1]])
        ax.set_zlim([state_lims[2, 0], state_lims[2, 1]])

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(title)
    ax.legend(fontsize=8)


def plot_states(result, title):
    xs = np.array(result["rollout_xs"])
    if xs.ndim < 2 or xs.shape[0] == 0:
        fig, _ = plt.subplots()
        plt.title(f"States - {title} (no data)")
        return fig
    config = result.get("config_dict", {})
    labels = config.get("ground_mdp_state_labels", [f"s{i}" for i in range(xs.shape[1])])
    state_lims = np.array(config.get("ground_mdp_X", []))

    n_states = min(12, xs.shape[1])  # 只画前12个状态
    fig, axes = plt.subplots(3, 4, figsize=(14, 8))
    fig.suptitle(f"States - {title}", fontsize=10)
    axes = axes.flatten()

    if state_lims.size > 0:
        state_lims = state_lims.reshape((-1, 2), order="F")

    for i in range(n_states):
        ax = axes[i]
        ax.plot(xs[:, i], linewidth=0.8)
        if i < len(labels):
            ax.set_title(labels[i], fontsize=8)
        if state_lims.size > 0 and i < state_lims.shape[0]:
            ax.axhline(state_lims[i, 0], color='gray', ls='--', alpha=0.5)
            ax.axhline(state_lims[i, 1], color='gray', ls='--', alpha=0.5)
        ax.tick_params(labelsize=6)

    for i in range(n_states, len(axes)):
        axes[i].set_visible(False)
    fig.tight_layout()
    return fig


def plot_controls(result, title):
    us = np.array(result["rollout_us"])
    if us.ndim < 2 or us.shape[0] == 0:
        fig, _ = plt.subplots()
        plt.title(f"Controls - {title} (no data)")
        return fig
    config = result.get("config_dict", {})
    control_labels = config.get("ground_mdp_control_labels", [f"u{i}" for i in range(us.shape[1])])
    control_lims = np.array(config.get("ground_mdp_U", []))

    n_controls = us.shape[1]
    ncols = min(4, n_controls)
    nrows = (n_controls + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3 * nrows))
    fig.suptitle(f"Controls - {title}", fontsize=10)
    if n_controls == 1:
        axes = [axes]
    else:
        axes = np.array(axes).flatten()

    if control_lims.size > 0:
        control_lims = control_lims.reshape((-1, 2), order="F")

    for i in range(n_controls):
        ax = axes[i]
        ax.plot(us[:, i], linewidth=0.8)
        if i < len(control_labels):
            ax.set_title(control_labels[i], fontsize=8)
        if control_lims.size > 0 and i < control_lims.shape[0]:
            ax.axhline(control_lims[i, 0], color='gray', ls='--', alpha=0.5)
            ax.axhline(control_lims[i, 1], color='gray', ls='--', alpha=0.5)
        ax.tick_params(labelsize=6)

    for i in range(n_controls, len(axes)):
        axes[i].set_visible(False)
    fig.tight_layout()
    return fig


def plot_energy(result, title):
    xs = np.array(result["rollout_xs"])
    if xs.ndim < 2 or xs.shape[0] == 0:
        fig, _ = plt.subplots()
        plt.title(f"Energy - {title} (no data)")
        return fig
    mass = 11.0
    gravity = 9.8
    KE = 0.5 * mass * np.sum(xs[:, 3:6] ** 2, axis=1)
    PE = mass * gravity * (-1 * xs[:, 2])
    total = KE + PE

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(KE, color="blue", label="Kinetic", linewidth=0.8)
    ax.plot(PE, color="red", label="Potential", linewidth=0.8)
    ax.plot(total, color="purple", label="Total", linewidth=0.8)
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Energy")
    ax.set_title(f"Energy - {title}")
    ax.legend()
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="可视化已有的 rollout 轨迹数据")
    parser.add_argument("--idx", type=int, nargs='*', default=None, help="要绘制的数据编号，如 0 1 2")
    parser.add_argument("--show", action="store_true", help="弹窗显示而非保存PDF")
    parser.add_argument("--out", type=str, default="../plots/rollout_trajectories.pdf", help="输出PDF路径")
    args = parser.parse_args()

    data_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
    results = load_results(data_dir, args.idx)

    all_figs = []

    # ---- 总览：所有轨迹在同一个 3D 图中 ----
    if len(results) > 1:
        fig_all = plt.figure(figsize=(10, 8))
        ax_all = fig_all.add_subplot(111, projection='3d')
        colors = plt.cm.tab10(np.linspace(0, 1, len(results)))
        for i, (fn, result) in enumerate(results):
            xs = np.array(result["rollout_xs"])
            if xs.ndim < 2 or xs.shape[0] == 0:
                continue
            label = os.path.basename(fn).replace("rollout_result_policy_convergence_", "run ").replace(".pkl", "")
            ax_all.plot(xs[:, 0], xs[:, 1], -1 * xs[:, 2], alpha=0.7, color=colors[i], label=label)
        ax_all.set_xlabel("x")
        ax_all.set_ylabel("y")
        ax_all.set_zlabel("z")
        ax_all.set_title("All Rollout Trajectories")
        ax_all.legend(fontsize=7)
        all_figs.append(fig_all)

    # ---- 每组数据单独绘制 ----
    for fn, result in results:
        tag = os.path.basename(fn).replace(".pkl", "")

        # 3D 轨迹
        fig3d = plt.figure(figsize=(8, 6))
        ax3d = fig3d.add_subplot(111, projection='3d')
        plot_3d_trajectory(result, f"3D Trajectory - {tag}", fig3d, ax3d)
        all_figs.append(fig3d)

        # 状态量
        all_figs.append(plot_states(result, tag))

        # 控制输入
        all_figs.append(plot_controls(result, tag))

        # 能量
        all_figs.append(plot_energy(result, tag))

    if args.show:
        plt.show()
    else:
        out_path = args.out
        out_dir = os.path.dirname(out_path)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        with PdfPages(out_path) as pp:
            for fig in all_figs:
                pp.savefig(fig)
        print(f"已保存到 {os.path.abspath(out_path)}  (共 {len(all_figs)} 页)")
        plt.close('all')


if __name__ == '__main__':
    main()
