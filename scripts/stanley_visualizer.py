#!/usr/bin/env python3
"""
stanley_visualizer.py — Real-time matplotlib visualization for Stanley controller.

Runs in an independent daemon thread so it never blocks the 30 Hz control loop.
All matplotlib GUI operations (figure creation, animation, event loop) are
confined to a single thread — required by TkAgg/QtAgg backends.

Renders a top-down (XY-plane) animation window showing:
  - Raw planned path (red dashed)
  - Spline-interpolated path (darkred solid)
  - Historical drone trajectory (blue solid)
  - Current position (navy dot)
  - Target waypoint (green star)
  - Obstacle bounding boxes (red semi-transparent rectangles)

Toggled via ROS param ~enable_visualization (default false).
All matplotlib imports are lazy — zero overhead when disabled.
"""

import threading
import sys
import os

# Ensure project src is on path for util imports
_project_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
_src_path = os.path.join(_project_root, "src")
if _src_path not in sys.path:
    sys.path.insert(0, _src_path)

# Lazy rospy reference — initialized on first use so this module can be
# imported without ROS running.
_rospy = None


def _get_rospy():
    global _rospy
    if _rospy is None:
        import rospy as _rp

        _rospy = _rp
    return _rospy


def _load_obstacle_boxes(config_name="policy_convergence_drone"):
    """Load obstacle bounding boxes from drone_planner config.

    Returns list of ((x0, x1), (y0, y1), (z0, z1)).
    Returns empty list if config is unavailable or obstacles section is missing.
    """
    try:
        from util import util

        config_path = util.get_config_path(config_name)
        config = util.load_yaml(config_path)
        obstacles = util.get_obstacles(config, 0)
        boxes = []
        for obs in obstacles:
            boxes.append(
                (
                    (obs[0, 0], obs[0, 1]),
                    (obs[1, 0], obs[1, 1]),
                    (obs[2, 0], obs[2, 1]),
                )
            )
        return boxes
    except Exception as e:
        try:
            rospy = _get_rospy()
            rospy.logwarn(
                "StanleyVisualizer: could not load obstacle boxes from "
                "config '%s': %s",
                config_name,
                e,
            )
        except Exception:
            pass
        return []


class StanleyVisualizer:
    """Real-time top-down visualization of Stanley controller tracking.

    All matplotlib GUI operations run in a single daemon thread to satisfy
    TkAgg / QtAgg thread-affinity requirements.  The controller main thread
    only writes into a shared dict protected by ``viz_lock``; the GUI thread
    reads from it each animation frame.
    """

    def __init__(
        self,
        viz_lock,
        viz_data,
        config_name="policy_convergence_drone",
        update_interval_ms=50.0,
    ):
        self._viz_lock = viz_lock
        self._viz_data = viz_data
        self._config_name = config_name
        self._update_interval_ms = update_interval_ms

        # Shared state between controller thread and GUI thread
        self._running = False
        self._ready = threading.Event()       # set when GUI is ready
        self._error = None                     # stores any startup exception
        self._thread = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """Launch the GUI in a daemon thread.  Returns immediately.

        Returns True if the GUI started successfully, False otherwise.
        """
        if self._running:
            return True

        self._ready.clear()
        self._error = None
        self._running = True
        self._thread = threading.Thread(
            target=self._gui_main, daemon=True, name="stanley-viz-gui"
        )
        self._thread.start()

        # Block until the GUI thread signals readiness or reports an error
        # (timeout prevents hanging forever if GUI fails silently).
        if not self._ready.wait(timeout=10.0):
            rospy = _get_rospy()
            rospy.logwarn("StanleyVisualizer: GUI startup timed out")
            self._running = False
            self._thread = None
            return False
        elif self._error is not None:
            rospy = _get_rospy()
            rospy.logwarn(
                "StanleyVisualizer: failed to start GUI — %s", self._error
            )
            self._running = False
            self._thread = None
            return False
        return True

    def shutdown(self):
        """Request the GUI thread to close and close the figure window."""
        self._running = False
        try:
            if self._fig is not None:
                import matplotlib.pyplot as plt
                plt.close(self._fig)
        except Exception:
            pass
        self._thread = None

    # ------------------------------------------------------------------
    # GUI main — runs entirely in the daemon thread
    # ------------------------------------------------------------------

    def _gui_main(self):
        """Thread target: import matplotlib, build figure, run event loop."""
        # If start() already timed out, abort early before creating any GUI.
        if not self._running:
            self._ready.set()
            return

        try:
            import matplotlib

            # Use interactive mode; let the system pick the best backend.
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
            from matplotlib.patches import Rectangle
        except Exception as e:
            self._error = "matplotlib import failed: %s" % e
            self._ready.set()
            return

        try:
            self._fig = None
            self._ax = None
            self._ani = None

            # --- figure & axes ---
            self._fig, self._ax = plt.subplots(figsize=(8.0, 7.0))
            try:
                self._fig.canvas.manager.set_window_title(
                    "Stanley Controller — Live Path Tracking"
                )
            except Exception:
                pass
            self._fig.canvas.mpl_connect("close_event", self._handle_close)

            self._ax.set_xlabel("x (m)")
            self._ax.set_ylabel("y (m)")
            self._ax.set_title("Stanley Controller — Live Path Tracking")
            self._ax.axis("equal")
            self._ax.grid(True, alpha=0.3)

            # --- static obstacle patches ---
            boxes = _load_obstacle_boxes(self._config_name)
            for bx, by, _bz in boxes:
                rect = Rectangle(
                    (bx[0], by[0]),
                    bx[1] - bx[0],
                    by[1] - by[0],
                    alpha=0.25,
                    facecolor="firebrick",
                    edgecolor="darkred",
                )
                self._ax.add_patch(rect)

            # --- placeholder artists ---
            self._raw_path_line, = self._ax.plot(
                [], [], "r--", linewidth=1.0, label="Planned path"
            )
            self._spline_line, = self._ax.plot(
                [], [], color="darkred", linewidth=1.2, label="Spline path"
            )
            self._traj_line, = self._ax.plot(
                [], [], color="royalblue", linewidth=1.0, label="Trajectory"
            )
            self._current_dot, = self._ax.plot(
                [], [], "o", color="navy", markersize=7
            )
            self._target_marker, = self._ax.plot(
                [], [], "*", color="lime", markersize=14, label="Target"
            )
            self._start_marker, = self._ax.plot(
                [], [], "go", markersize=8, label="Start"
            )
            self._ax.legend(loc="upper right", fontsize=8)
            self._fig.tight_layout()

            # Start with a reasonable default view so the obstacle patch
            # (the only thing with data initially) doesn't dominate the
            # figure before trajectory data arrives.
            self._ax.set_xlim(-30, 30)
            self._ax.set_ylim(-30, 30)

            # Scene-bounds accumulators for one-shot view lock.
            # Seed with obstacle extents so they are always visible.
            # Pre-seed scene bounds with obstacle extents so they
            # contribute to the one-shot view lock in _update().
            self._scene_x = []
            self._scene_y = []
            for bx, by, _bz in boxes:
                self._scene_x.extend([bx[0], bx[1]])
                self._scene_y.extend([by[0], by[1]])
            self._view_locked = False

            # Tracked state for lazy-update
            self._last_target = None
            self._start_set = False

            # --- animation ---
            # blit=False is required because the adaptive view lock
            # may call set_xlim/set_ylim 1-2 times during the mission.
            # With blit=True + TkAgg + ax.axis("equal"), matplotlib
            # does not reliably re-capture the background on axes
            # changes, causing static patches (obstacles) to disappear.
            self._ani = FuncAnimation(
                self._fig,
                self._update,
                interval=self._update_interval_ms,
                blit=False,
                cache_frame_data=False,
                save_count=0,
            )

            # Signal controller thread that GUI is ready
            self._ready.set()

            # --- block on the GUI event loop ---
            plt.show(block=True)

        except Exception as e:
            self._error = str(e)
            self._ready.set()

    # ------------------------------------------------------------------
    # Per-frame update (called by FuncAnimation on the GUI thread)
    # ------------------------------------------------------------------

    def _update(self, _frame):
        """Read latest data from shared buffer and update all artists."""
        rospy = _get_rospy()

        try:
            if rospy.is_shutdown() or not self._running:
                self._stop_animation()
                return []

            with self._viz_lock:
                raw_x = list(self._viz_data.get("raw_path_x", []) or [])
                raw_y = list(self._viz_data.get("raw_path_y", []) or [])
                spl_x = list(self._viz_data.get("spline_x", []) or [])
                spl_y = list(self._viz_data.get("spline_y", []) or [])
                traj_x = list(self._viz_data.get("traj_x", []) or [])
                traj_y = list(self._viz_data.get("traj_y", []) or [])
                cur_x = self._viz_data.get("current_x", 0.0)
                cur_y = self._viz_data.get("current_y", 0.0)
                tgt_x = self._viz_data.get("target_x", None)
                tgt_y = self._viz_data.get("target_y", None)
                start_x = self._viz_data.get("start_x", None)
                start_y = self._viz_data.get("start_y", None)

            # --- Update artists ---
            self._raw_path_line.set_data(raw_x, raw_y)
            self._spline_line.set_data(spl_x, spl_y)
            self._traj_line.set_data(traj_x, traj_y)
            self._current_dot.set_data([cur_x], [cur_y])

            # Target marker (only when coords change)
            if tgt_x is not None and tgt_y is not None:
                if (tgt_x, tgt_y) != self._last_target:
                    self._target_marker.set_data([tgt_x], [tgt_y])
                    self._last_target = (tgt_x, tgt_y)
            else:
                self._target_marker.set_data([], [])

            # Start marker (set once)
            if not self._start_set and start_x is not None:
                self._start_marker.set_data([start_x], [start_y])
                self._start_set = True

            # --- Adaptive view management ---
            # Initial lock: base the view on start + target + obstacle
            # so all three scene anchors are visible at once.  The
            # lock happens when start, target, and path are all known
            # and the target is not the planner's distant default.
            if not self._view_locked:
                if (tgt_x is not None and (spl_x or raw_x)
                        and start_x is not None):
                    if (abs(tgt_x - cur_x) > 200.0
                            or abs(tgt_y - cur_y) > 200.0):
                        pass  # planner's initial default (120,120), skip
                    else:
                        xs = [start_x, tgt_x, cur_x] + self._scene_x
                        ys = [start_y, tgt_y, cur_y] + self._scene_y
                        self._ax.set_xlim(min(xs) - 5.0, max(xs) + 5.0)
                        self._ax.set_ylim(min(ys) - 5.0, max(ys) + 5.0)
                        self._view_locked = True
            else:
                # After lock: if the target changes to a location
                # beyond the current viewport, expand just enough to
                # bring it into frame (one-shot expansion, not
                # continuous auto-scale).
                if tgt_x is not None and tgt_y is not None:
                    xl, yl = self._ax.get_xlim(), self._ax.get_ylim()
                    if (tgt_x < xl[0] or tgt_x > xl[1]
                            or tgt_y < yl[0] or tgt_y > yl[1]):
                        xs = [tgt_x, xl[0], xl[1]] + self._scene_x
                        ys = [tgt_y, yl[0], yl[1]] + self._scene_y
                        self._ax.set_xlim(min(xs) - 3.0, max(xs) + 3.0)
                        self._ax.set_ylim(min(ys) - 3.0, max(ys) + 3.0)

            return [
                self._raw_path_line,
                self._spline_line,
                self._traj_line,
                self._current_dot,
                self._target_marker,
                self._start_marker,
            ]
        except Exception as e:
            rospy.logwarn_throttle(
                10.0, "StanleyVisualizer update error: %s", e
            )
            return []

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _handle_close(self, _event):
        """Called when user closes the matplotlib window."""
        self._running = False

    def _stop_animation(self):
        """Stop the animation timer from inside the callback."""
        try:
            if self._ani is not None and self._ani.event_source is not None:
                self._ani.event_source.stop()
        except Exception:
            pass
