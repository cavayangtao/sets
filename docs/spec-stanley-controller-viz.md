# SPEC: Stanley Controller 实时可视化模块

> Technical specification derived from: [tasks/prd-stanley-controller-viz.md](tasks/prd-stanley-controller-viz.md)
> Generated: 2026-06-15 | Updated: 2026-06-17 | Target branch: main | Commit: 8e2fb15

## 1. Summary

### 1.1 What This SPEC Covers

This SPEC defines the implementation of a real-time matplotlib-based visualization module for `stanley_controller_node.py`. The module renders a live top-down (XY-plane) animation window showing: (a) the raw planned path from the planner, (b) the spline-interpolated path used for Stanley tracking, (c) the drone's historical trajectory, (d) the current target waypoint, and (e) obstacle bounding boxes. The visualization runs in an independent daemon thread so it never blocks the 30 Hz control loop. It is toggled by a single ROS param `~enable_visualization` (default `false`), with all matplotlib imports lazy-loaded.

### 1.2 PRD Reference

- Source: `tasks/prd-stanley-controller-viz.md`
- User Stories covered: US-001, US-002, US-003, US-004, US-005, US-006
- Functional Requirements covered: FR-1 through FR-9

### 1.3 Design Decisions Summary

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Visualization file location | New file `scripts/stanley_visualizer.py` | Keeps controller readable; separate module with single responsibility |
| Thread model | `threading.Thread(daemon=True)` | Daemon ensures auto-cleanup on controller exit; no explicit join needed |
| matplotlib backend | TkAgg (explicit) | `FuncAnimation` requires interactive backend; TkAgg works on all test platforms |
| matplotlib import | Lazy, only when `enable_visualization=True` | FR-1: zero overhead when disabled |
| Obstacle data source | YAML config file via `/drone_planner/config_name` param | PRD answer 2A; consistent with existing `load_obstacle_boxes` |
| Data sharing | Flat dict + `threading.Lock` | Simple, fast, sufficient for single-producer-single-consumer |
| Path display | Raw path (red dashed) + Path points (darkred solid) | PRD answer 3C; when spline off, both show same waypoints |
| Toggle mechanism | Launch params, no runtime toggle | PRD answer 4A |
| Trajectory storage | Sliding window: last N seconds, temporal downsampling | PRD answer 5C |
| View scaling | Adaptive one-shot lock | User requirement: fixed readable scale; locks on first path+target+start, expands if target moves outside |
| Spline interpolation | Configurable via `~use_spline_interpolation` | Default `false` in launch; enables raw-waypoint tracking |
| Monotonic target index | Configurable via `~monotonic_target_index` | Default `false` in launch; when false, eliminates target-approach oscillation |
| blit mode | `blit=False` | Required because adaptive view lock may call `set_xlim`/`set_ylim` 1-2 times; `blit=True` + TkAgg does not reliably re-capture background on axes changes, causing static patches to disappear |

---

## 2. Architecture

### 2.1 System Context

```
┌─────────────────────────────────────────────────────────────┐
│  drone_planner.launch                                       │
│                                                             │
│  ┌─────────────────────┐    ┌──────────────────────────────┐│
│  │ drone_planner_node   │    │ stanley_controller_node      ││
│  │                      │    │                              ││
│  │ /planner/trajectory ─┼───▶│ _trajectory_callback         ││
│  │ /planner/vz_cmd ─────┼───▶│ _vz_cmd_callback            ││
│  │                      │    │                              ││
│  │                      │    │ /pose ───▶ _pose_callback    ││
│  │                      │    │                              ││
│  │                      │    │ _control_callback (30 Hz)    ││
│  │                      │    │   ├─▶ /cmd_vel               ││
│  │                      │    │   └─▶ update _viz_data ─────┼──┐
│  │                      │    │       (thread-safe)          │  │
│  │                      │    │                              │  │
│  │                      │    │ StanleyVisualizer (thread)   │  │
│  │                      │    │   └─▶ FuncAnimation (20 FPS) │◀─┘
│  └─────────────────────┘    └──────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Component Design

**New component:** `StanleyVisualizer` class in `scripts/stanley_visualizer.py`

Responsibilities:
- Own the matplotlib `Figure`, `Axes`, and `FuncAnimation` lifecycle
- Periodically read shared data buffer (protected by `threading.Lock`)
- Render all visual elements on each animation frame
- Clean up on window close or ROS shutdown

**Modified component:** `StanleyControllerNode` in `scripts/stanley_controller_node.py`

Changes:
- Conditionally import and instantiate `StanleyVisualizer`
- Populate `self._viz_data` dict during `_trajectory_callback` and `_pose_callback`
- Pass `self._viz_lock` and config params to visualizer

### 2.3 Module Interactions

```
_init:
  if enable_visualization:
      import stanley_visualizer
      self._viz_lock = threading.Lock()
      self._viz_data = { ... empty buffers ... }
      self._visualizer = stanley_visualizer.StanleyVisualizer(
          viz_lock=self._viz_lock,
          viz_data=self._viz_data,
          config_name=...,
          history_window_s=...,
          max_history_points=...,
      )

_trajectory_callback:
  with self._viz_lock:
      self._viz_data["raw_path_x"] = ax  # original waypoints
      self._viz_data["raw_path_y"] = ay
      self._viz_data["spline_x"] = self._cx  # spline points
      self._viz_data["spline_y"] = self._cy

_pose_callback:
  with self._viz_lock:
      append (x, y, t) to trajectory buffer
      apply sliding window + downsampling

StanleyVisualizer._update(frame):
  with self._viz_lock:
      copy relevant data  # fast, shallow copies of lists
  redraw artists
```

### 2.4 File Structure

```
scripts/
├── stanley_controller_node.py   [MODIFY: add viz integration]
├── stanley_visualizer.py        [NEW: visualization module]
└── ... (unchanged)

launch/
└── drone_planner.launch         [MODIFY: add enable_visualization arg/param]
```

No new dependencies beyond what the project already uses (`matplotlib`, `numpy`, `threading`).

---

## 3. Data Model

### 3.1 Shared Data Structure

The visualization thread and main thread communicate through a shared dict protected by a single `threading.Lock`. No queue is needed because only the latest state matters — old frames are simply skipped.

```python
# Initialized in StanleyControllerNode.__init__
self._viz_data = {
    # -- written by _trajectory_callback --
    "raw_path_x": [],        # list[float]: original waypoint x coords
    "raw_path_y": [],        # list[float]: original waypoint y coords
    "spline_x": [],          # list[float]: spline-interpolated x coords
    "spline_y": [],          # list[float]: spline-interpolated y coords

    # -- written by _pose_callback --
    "traj_x": [],            # list[float]: history x (downsampled, sliding)
    "traj_y": [],            # list[float]: history y
    "traj_t": [],            # list[float]: timestamps (seconds since epoch)
    "current_x": 0.0,        # float
    "current_y": 0.0,        # float

    # -- written once at init --
    "obstacle_boxes": [],    # list[((x0,x1),(y0,y1),(z0,z1))]
    "start_x": None,         # float | None (set on first pose)
    "start_y": None,         # float | None

    # -- read periodically from ROS param by _control_callback --
    "target_x": None,        # float | None
    "target_y": None,        # float | None
}

self._viz_lock = threading.Lock()
```

### 3.2 Trajectory Buffer Management

Pseudocode for the sliding window logic executed in `_pose_callback`:

```python
def _append_trajectory_sample(self, x, y):
    now = rospy.Time.now().to_sec()
    window = self._history_window_s      # default 60.0
    max_points = self._max_history_points  # default 5000

    with self._viz_lock:
        data = self._viz_data
        data["traj_x"].append(x)
        data["traj_y"].append(y)
        data["traj_t"].append(now)

        # Set start position on first sample
        if data["start_x"] is None:
            data["start_x"] = x
            data["start_y"] = y

        # Slide window: remove points older than window
        cutoff = now - window
        while data["traj_t"] and data["traj_t"][0] < cutoff:
            data["traj_t"].pop(0)
            data["traj_x"].pop(0)
            data["traj_y"].pop(0)

        # Downsample if over max_points
        if len(data["traj_t"]) > max_points:
            step = len(data["traj_t"]) // max_points
            data["traj_x"] = data["traj_x"][::step]
            data["traj_y"] = data["traj_y"][::step]
            data["traj_t"] = data["traj_t"][::step]
```

### 3.3 Obstacle Data Loading

Reuse the exact logic from `run_drone_planner_test.py:load_obstacle_boxes()`:

```python
def _load_obstacle_boxes(config_name):
    config = util.load_yaml(util.get_config_path(config_name))
    obstacles = util.get_obstacles(config, 0)  # timestep=0
    boxes = []
    for obs in obstacles:
        boxes.append(((obs[0, 0], obs[0, 1]),
                      (obs[1, 0], obs[1, 1]),
                      (obs[2, 0], obs[2, 1])))
    return boxes
```

Called once at visualizer initialization. If config is unavailable, log a warning and continue with an empty obstacle list.

---

## 4. API / Interface Design

### 4.1 `StanleyVisualizer` Public Interface

```python
class StanleyVisualizer:
    def __init__(self,
                 viz_lock: threading.Lock,
                 viz_data: dict,
                 config_name: str = "policy_convergence_drone",
                 update_interval_ms: float = 50.0):
        """
        Parameters
        ----------
        viz_lock : threading.Lock
            Shared lock for viz_data access.
        viz_data : dict
            Shared data buffer (see Section 3.1).
        config_name : str
            Config name for obstacle loading (matches planner).
        update_interval_ms : float
            FuncAnimation interval in milliseconds (≈20 FPS).
        """

    def start(self) -> bool:
        """Launch the matplotlib animation in a daemon thread.

        Returns True if the GUI started successfully, False on timeout
        or error.  Blocks the caller for up to 10 s waiting for the
        GUI thread to signal readiness.
        """

    def shutdown(self):
        """Close the matplotlib figure window and stop the animation."""
```

### 4.2 New ROS Parameters

Added to the `stanley_controller` private namespace:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~enable_visualization` | bool | `false` | Master toggle |
| `~visualization_history_window` | float | `60.0` | Sliding window in seconds |
| `~visualization_max_history_points` | int | `5000` | Max trajectory points |
| `~use_spline_interpolation` | bool | `true` (code) / `false` (launch) | When false, track raw planner waypoints directly |
| `~monotonic_target_index` | bool | `true` (code) / `false` (launch) | When false, allow target index to move backward along path |

### 4.3 Launch File Changes

In `drone_planner.launch`:

```xml
<!-- Visualization args -->
<arg name="enable_visualization"         default="false"/>
<arg name="visualization_history_window" default="60.0"/>

<!-- Spline / path following args -->
<arg name="use_spline_interpolation" default="false"/>
<arg name="monotonic_target_index"   default="false"/>
```

In the `stanley_controller` node block:

```xml
<param name="use_spline_interpolation" value="$(arg use_spline_interpolation)"/>
<param name="monotonic_target_index"   value="$(arg monotonic_target_index)"/>
<param name="enable_visualization"           value="$(arg enable_visualization)"/>
<param name="visualization_history_window"   value="$(arg visualization_history_window)"/>
<param name="visualization_max_history_points" value="5000"/>
```

### 4.4 Controller Integration Points

In `StanleyControllerNode.__init__`, after existing subscriber/publisher setup:

```python
# --- Visualization (conditional) ---
self._enable_visualization = rospy.get_param("~enable_visualization", False)
self._visualizer = None
self._viz_lock = None
self._viz_data = None

if self._enable_visualization:
    self._load_viz_params()
    self._viz_lock = threading.Lock()
    self._viz_data = { ... }  # Section 3.1
    self._visualizer = StanleyVisualizer(
        viz_lock=self._viz_lock,
        viz_data=self._viz_data,
        config_name=self._config_name_for_viz,
        history_window_s=self._viz_history_window,
        max_history_points=self._viz_max_history_points,
    )
    self._visualizer.start()
    rospy.loginfo("Visualization enabled: window=%.1fs, max_points=%d",
                  self._viz_history_window, self._viz_max_history_points)
```

In `_trajectory_callback`, after spline is built:

```python
if self._enable_visualization:
    with self._viz_lock:
        self._viz_data["raw_path_x"] = ax   # from dedup'd waypoints
        self._viz_data["raw_path_y"] = ay
        self._viz_data["spline_x"] = list(self._cx)
        self._viz_data["spline_y"] = list(self._cy)
```

In `_pose_callback`, after updating state:

```python
if self._enable_visualization:
    self._append_trajectory_sample(self._state.x, self._state.y)
```

In `_control_callback` (or a slow timer), update target position:

```python
if self._enable_visualization:
    try:
        target = rospy.get_param("/drone_planner/target_pos", None)
        if target is not None:
            if isinstance(target, str):
                target = ast.literal_eval(target)
            with self._viz_lock:
                self._viz_data["target_x"] = float(target[0])
                self._viz_data["target_y"] = float(target[1])
    except Exception:
        pass  # target param may not exist yet
```

---

## 5. Business Logic

### 5.1 Visualization Rendering Algorithm

Executed by `FuncAnimation._update` at ~20 FPS in the visualization thread:

```
function update(frame):
    if rospy.is_shutdown() or not _running:
        _stop_animation()
        return []

    with viz_lock:
        raw_x  = copy(raw_path_x)
        raw_y  = copy(raw_path_y)
        spl_x  = copy(spline_x)
        spl_y  = copy(spline_y)
        traj_x = copy(traj_x)
        traj_y = copy(traj_y)
        cur_x  = current_x
        cur_y  = current_y
        tgt_x  = target_x
        tgt_y  = target_y
        start_x = start_x
        start_y = start_y

    # Update line artists (reuse for performance)
    _raw_path_line.set_data(raw_x, raw_y)       # red dashed
    _spline_line.set_data(spl_x, spl_y)          # darkred solid
    _traj_line.set_data(traj_x, traj_y)          # blue solid
    _current_dot.set_data([cur_x], [cur_y])      # navy dot

    # Target marker (only when coords change)
    if tgt_x is not None and (tgt_x, tgt_y) != _last_target:
        _target_marker.set_data([tgt_x], [tgt_y])

    # Start marker (set once on first pose)
    if not _start_set and start_x is not None:
        _start_marker.set_data([start_x], [start_y])

    # Adaptive view lock (see §5.3)

    return [_raw_path_line, _spline_line, _traj_line,
            _current_dot, _target_marker, _start_marker]
```

Animation uses `blit=False` — full redraw each frame.  This is required
because the adaptive view lock (§5.3) calls `set_xlim`/`set_ylim` 1–2
times during the mission; with `blit=True` + TkAgg + `ax.axis("equal")`,
matplotlib does not reliably re-capture the background on axes changes,
causing static patches (obstacles) to disappear.

### 5.2 Legend & Static Elements

Static elements (created once in `__init__`, never modified):

| Artist | Creation | Data |
|--------|----------|------|
| `_raw_path_line` | `ax.plot([], [], 'r--', lw=1.0, label='Planned path')` | Updated each frame |
| `_spline_line` | `ax.plot([], [], color='darkred', lw=1.2, label='Spline path')` | Updated each frame |
| `_traj_line` | `ax.plot([], [], color='royalblue', lw=1.0, label='Trajectory')` | Updated each frame |
| `_current_dot` | `ax.plot([], [], 'o', color='navy', ms=7)` | Updated each frame |
| `_target_marker` | `ax.plot([], [], '*', color='lime', ms=14, label='Target')` | Updated when target changes |
| `_start_marker` | `ax.plot([], [], 'go', ms=8, label='Start')` | Set once on first pose |
| Obstacle rects | `ax.add_patch(Rectangle(...))` per obstacle | Static |
| Legend | `ax.legend(loc='upper right', fontsize=8)` | Static |

### 5.3 Adaptive View Lock

The view is locked once at mission start and expanded only when
necessary — never continuously auto-scaled.  This keeps a fixed,
readable scale for the entire mission.

**Initial lock** (triggered when target, path, and start are all known,
and the target is not the planner's distant default value):

```python
xs = [start_x, tgt_x, cur_x] + obstacle_bounds_x
ys = [start_y, tgt_y, cur_y] + obstacle_bounds_y
ax.set_xlim(min(xs) - 5.0, max(xs) + 5.0)
ax.set_ylim(min(ys) - 5.0, max(ys) + 5.0)
```

**Post-lock expansion** (triggered when the target changes to a location
outside the current viewport, e.g., wp0 → wp1):

```python
if tgt_x < xl[0] or tgt_x > xl[1] or tgt_y < yl[0] or tgt_y > yl[1]:
    xs = [tgt_x, xl[0], xl[1]] + obstacle_bounds_x
    ys = [tgt_y, yl[0], yl[1]] + obstacle_bounds_y
    ax.set_xlim(min(xs) - 3.0, max(xs) + 3.0)
    ax.set_ylim(min(ys) - 3.0, max(ys) + 3.0)
```

The view is based on: start position, target position, and obstacle
extents.  Full trajectory history is deliberately excluded — a distant
start would otherwise shrink the relevant corridor to an unreadable
scale.  The trajectory line and start marker are still drawn; they may
extend beyond the locked viewport.

Default target filter: if `|tgt - cur| > 200 m`, the target is assumed
to be the planner's initial default (e.g., `[120, 120, …]` from the
launch file) and the lock is deferred until the test script sets the
real waypoint.

### 5.4 Edge Cases

| Scenario | Handling |
|----------|----------|
| No trajectory received yet | Raw/spline path lines remain empty (no data to plot) |
| First pose before any trajectory | Trajectory line shows single dot at start position |
| Trajectory timeout (no new path) | Keep last known path lines visible; no flickering |
| Target param not set | `target_x`/`target_y` remain `None`; target marker hidden |
| Config file not found | Log warning, continue with empty obstacle list |
| Window closed by user | `_handle_close` event sets a flag; animation loop exits gracefully |
| ROS shutdown during visualization | `rospy.is_shutdown()` check in `_update`; call `plt.close()` on True |
| Very long trajectory (>5000 points) | Sliding window + downsampling keeps buffer bounded |
| matplotlib import fails | Log error once, set `_enable_visualization = False`, continue |

---

## 6. Error Handling

### 6.1 Error Taxonomy

| Condition | Severity | Behavior |
|-----------|----------|----------|
| matplotlib not installed | WARN | Log warning, disable viz, continue controller |
| Config file not found | WARN | Log warning, empty obstacle list, continue |
| Target param parse failure | DEBUG | Skip target update for this cycle |
| Lock timeout (should never happen) | — | No timeout; `threading.Lock` blocks but all critical sections are O(1) |
| Window close event | INFO | Clean shutdown of animation thread |
| Exception in `_update` | ERROR | Log traceback, skip frame, animation continues |

### 6.2 Failure Modes

- **matplotlib import failure:** If `import matplotlib.pyplot` raises `ImportError`, catch it, log a one-time warning, set `self._enable_visualization = False`, and the controller runs normally without viz.
- **GUI backend unavailable (headless server):** If no `$DISPLAY` and no interactive backend, `plt.figure()` will raise. Catch this in `StanleyVisualizer.__init__`, log warning, and return early without starting the thread.

### 6.3 Graceful Shutdown

```python
def shutdown(self):
    """Called from controller cleanup or ROS shutdown hook."""
    self._running = False
    if self._fig is not None:
        plt.close(self._fig)
    # Daemon thread exits automatically when process ends
```

---

## 7. Performance

### 7.1 Expected Load

- **Animation rate:** 20 FPS (50 ms interval)
- **Data points per frame:** < 1000 trajectory points, < 500 spline points, < 50 raw waypoints
- **Lock hold time:** < 50 µs (shallow copy of lists)
- **matplotlib redraw time:** ~5-15 ms for simple 2D line plots
- **Total CPU overhead:** < 5% of one core (estimated)

### 7.2 Optimization Strategy

1. **Reuse artists** — Never call `ax.plot()` inside the animation loop; mutate existing `Line2D` objects via `.set_data()`
2. **Lazy target check** — Only call `.set_data()` on the target marker when coordinates actually change
3. **O(n) trajectory pruning** — Index-based cutoff + single slice replaces the original O(n²) `list.pop(0)` loop, keeping lock hold times short
4. **Lock granularity** — Critical section under `viz_lock` performs shallow copies of list data (~µs hold time); heavy work (axes updates) happens outside the lock
5. **`blit=False` + full redraw** — Simple 2D line plots redraw in <10 ms at 20 FPS; the reliability benefit (no disappearing obstacles) outweighs the marginal performance gain from `blit=True`

### 7.3 Control Loop Impact

The main thread writes trajectory points (O(1) append) under the lock. The visualization thread is the only reader. With a single lock and O(1) critical sections, the main thread should never block on visualization for more than a few microseconds.

---

## 8. Testing Strategy

### 8.1 Unit Tests

| Test | What it verifies |
|------|-----------------|
| `test_sliding_window` | Trajectory buffer correctly removes old points |
| `test_downsampling` | Buffer downsamples when exceeding max points |
| `test_lock_thread_safety` | Concurrent read/write doesn't corrupt data |
| `test_obstacle_loading` | `_load_obstacle_boxes` returns correct format |
| `test_viz_disabled_default` | Controller initializes without importing matplotlib when param is false |

### 8.2 Integration Tests

| Test | What it verifies |
|------|-----------------|
| `test_viz_thread_start_stop` | Visualizer starts and shuts down without exception |
| `test_trajectory_callback_updates_viz_data` | Path data correctly flows from callback to shared buffer |
| `test_pose_callback_appends_sample` | Pose samples correctly appended to trajectory buffer |

### 8.3 Manual Verification

| Test | How to verify |
|------|---------------|
| Window appears | `roslaunch sets drone_planner.launch enable_visualization:=true` |
| Planned path visible | Set a target, wait for planner to publish trajectory |
| Trajectory grows | Watch blue line extend as drone moves |
| Obstacles visible | Verify red rectangles match config file |
| Target marker | Verify green star at target position |
| No viz mode | `roslaunch sets drone_planner.launch` (default) — no window, controller runs normally |

### 8.4 Acceptance Criteria Mapping

| US | Verification |
|----|-------------|
| US-001 | Launch with/without `enable_visualization:=true`; verify window presence/absence |
| US-002 | Check controller log for "Visualization enabled" message; verify no control-loop jitter |
| US-003 | Inspect window: red dashed line = raw waypoints, darkred solid line = spline |
| US-004 | Inspect window: blue line grows over time; verify older points disappear after window |
| US-005 | Inspect window: green `*` at target; red rectangles for obstacles |
| US-006 | Check `drone_planner.launch` for new arg and param entries |

---

## 9. Implementation Plan

### 9.1 Phases

| Phase | Scope | Files |
|-------|-------|-------|
| **P1: Core module** | `StanleyVisualizer` class with matplotlib window, FuncAnimation, obstacle loading | `stanley_visualizer.py` (new) |
| **P2: Controller integration** | Shared buffer, lock, callback hooks, param loading | `stanley_controller_node.py` (modify) |
| **P3: Launch config** | New args and params in launch file | `drone_planner.launch` (modify) |
| **P4: Validation** | Manual run, edge case tests, performance check | — |

### 9.2 File Change Summary

| File | Action | Lines (est.) |
|------|--------|-------------|
| `scripts/stanley_visualizer.py` | **Create** | ~180 lines |
| `scripts/stanley_controller_node.py` | **Modify** | +~50 lines |
| `launch/drone_planner.launch` | **Modify** | +~7 lines |

### 9.3 Incremental Delivery

No feature flags needed beyond the existing `enable_visualization` param. The feature is entirely self-contained — when disabled, no code paths differ from the current version.

---

## 10. Open Questions & Risks

### 10.1 Unresolved Questions

- **What config name should the visualizer use for obstacle loading?**
  - Resolution: Read `rospy.get_param("/drone_planner/config_name", "policy_convergence_drone")` so it automatically matches the planner's config. No additional param needed.

- **Should the visualizer also render the drone's yaw orientation?**
  - Not requested in PRD. Could be a future enhancement (arrow or triangle indicating heading).

### 10.2 Technical Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| matplotlib GUI backend not available on headless systems | Viz silently disabled | Catch exception at `plt.figure()`, log warning, disable |
| `FuncAnimation` memory leak if window closed improperly | Minor memory growth over hours | `_handle_close` event + `rospy.is_shutdown()` guard |
| Lock contention under extreme load | Brief control loop jitter | Critical sections are O(1); lock held for microseconds |

### 10.3 Assumptions

- matplotlib is installed in the ROS environment (it is already used by `run_drone_planner_test.py` and `src/plotter.py`)
- A display server (X11/Wayland) is available when `enable_visualization=true`
- The controller node runs on a machine with a GUI (not a headless robot computer) when visualization is enabled
- The obstacle config file structure matches what `util.get_obstacles()` returns
