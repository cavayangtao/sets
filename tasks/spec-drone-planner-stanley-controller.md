# SPEC: Drone Trajectory Planner + Stanley Controller

> Technical specification derived from: [tasks/prd-drone-planner-stanley-controller.md](prd-drone-planner-stanley-controller.md)
> Generated: 2026-05-26 | Target branch: N/A (local) | Commit: N/A

## 1. Summary

### 1.1 What This SPEC Covers

本 SPEC 详细设计两个 ROS 1 节点：`drone_planner_node` 使用 `policy_convergence_drone.yaml`（2.0 kg 四旋翼，8 维推力+扭矩控制）运行在线 UCT-MPC，滚动发布 `nav_msgs/Path` 轨迹；`stanley_controller_node` 接收轨迹并通过三次样条+Stanley 控制律生成 `/cmd_vel`。两个节点通过 `drone_planner.launch` 统一启动。

### 1.2 PRD Reference

- Source: `tasks/prd-drone-planner-stanley-controller.md`
- User Stories covered: US-001, US-002, US-003, US-004, US-005, US-006
- Functional Requirements covered: FR-1 到 FR-9

### 1.3 Design Decisions Summary

| Decision | Choice | Rationale |
|----------|--------|-----------|
| 轨迹消息类型 | `nav_msgs/Path` | 标准 ROS 消息，Gazebo/RViz 原生支持可视化 |
| Path 内容 | 位置(xyz) + 偏航(yaw) | 提供朝向信息供 Stanley 前视投影使用 |
| 轨迹提取方式 | 从 `result.planned_traj.xs` 逐行提取 state[0,1,2,8] | 位置 indices 0-2, yaw index 8 |
| 控制序列格式 | `Float64MultiArray` 展平拼接，每步 4 个有效控制量 | `layout.dim` 标注 [steps, 4]，方便解析 |
| Stanley 跟踪方式 | 三次样条插值 Path 点后跟踪 | 与现有 `stanley.py` 算法一致，轨迹更平滑 |
| 控制器丢轨迹行为 | 立即发布零速度 | 安全优先，PRD 已确认 |
| 坐标系 | 仿真 `/pose` 使用 ENU (z-up)，planner 内部使用 NED (z-down) | 与现有 tello_planner 惯例一致 |

---

## 2. Architecture

### 2.1 System Context

```
┌──────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                       │
│  ┌─────────────┐                                         │
│  │ 无人机模型    │──► /pose (PoseStamped, ENU)             │
│  │ (2.0kg quad) │◄── /cmd_vel (Twist)                    │
│  └─────────────┘                                         │
└──────────────────────────────────────────────────────────┘
         │                              ▲
         │ /pose                         │ /cmd_vel
         ▼                              │
┌──────────────────┐    /planner/     ┌──────────────────────┐
│ drone_planner_node│─── trajectory ─►│ stanley_controller_  │
│                  │     (Path)       │        node          │
│  UCT-MPC @2Hz    │─── control_seq─►│  Stanley @30Hz       │
│                  │  (Float64Multi) │  cubic_spline + PID  │
└──────────────────┘                 └──────────────────────┘
```

### 2.2 Component Design

#### drone_planner_node

| 属性 | 值 |
|--------|-------|
| 源文件 | `scripts/drone_planner_node.py` (新) |
| 参考 | `scripts/tello_planner_node.py` |
| 配置 | `configs/sixdofaircraft/policy_convergence_drone.yaml` |
| 状态机 | INIT → READY → RUNNING（无 FAILSAFE，纯仿真） |

职责：
- 加载 `policy_convergence_drone.yaml` 并通过 pybind11 初始化 MDP/DOTS/UCT2
- 订阅 `/pose`，估计速度（有限差分+LPF），构建 13 维状态向量
- 以可配置频率（默认 2 Hz）运行 `run_uct2`，得到 planned trajectory
- 从 `result.planned_traj.xs` 提取位置+偏航，发布 `nav_msgs/Path`
- 从 `result.planned_traj.us` 提取控制量，发布 `Float64MultiArray`
- 提供服务：`~arm`、`~estop`、`~update_target`

#### stanley_controller_node

| 属性 | 值 |
|--------|-------|
| 源文件 | `scripts/stanley_controller_node.py` (新) |
| 参考 | `scripts/stanley.py` |
| 依赖 | `scripts/cubic_spline_planner.py`（复用，不修改） |

职责：
- 订阅 `/planner/trajectory` (Path) 和 `/pose` (PoseStamped)
- 将 Path 消息转换为 (cx, cy, cyaw) 数组，通过三次样条插值
- 使用 Stanley 控制律计算转向角，PID 控制速度
- 发布 `geometry_msgs/Twist` 到 `/cmd_vel`
- 未收到轨迹时立即发布零速度

### 2.3 Module Interactions

```
sequence
    participant Gazebo
    participant Planner as drone_planner_node
    participant Controller as stanley_controller_node

    loop 500 Hz (sim)
        Gazebo->>Planner: /pose (PoseStamped)
        Gazebo->>Controller: /pose (PoseStamped)
    end

    loop 2 Hz (planner rate)
        Planner->>Planner: build 13D state, run_uct2()
        Planner->>Controller: /planner/trajectory (Path)
        Planner-->>any: /planner/control_seq (Float64MultiArray)
    end

    loop 30 Hz (control rate)
        Controller->>Controller: cubic_spline(Path) + stanley_control()
        Controller->>Gazebo: /cmd_vel (Twist)
    end
```

### 2.4 File Structure

```
sets/
├── scripts/
│   ├── drone_planner_node.py          [NEW]  Planner node
│   ├── stanley_controller_node.py     [NEW]  Controller node
│   ├── cubic_spline_planner.py        [REUSE, no changes]
│   └── tello_planner_node.py          [REFERENCE only]
├── launch/
│   ├── drone_planner.launch           [NEW]  Launch file
│   └── tello_planner.launch           [REFERENCE only]
├── configs/sixdofaircraft/
│   └── policy_convergence_drone.yaml  [REUSE, no changes]
└── src/
    └── bindings.cpp                   [REUSE, no changes]
```

---

## 3. Data Model

### 3.1 State Vector Layout (13D)

根据 `policy_convergence_drone.yaml` 的 `ground_mdp_state_labels`：

| Index | Label | Meaning | Unit | Source |
|-------|-------|---------|------|--------|
| 0 | p_x | x position | m | /pose.position.x → ENU→NED |
| 1 | p_y | y position | m | /pose.position.y → ENU→NED |
| 2 | p_z | z position (down) | m | /pose.position.z → ENU→NED |
| 3 | v_x | x velocity | m/s | estimated (finite diff + LPF) |
| 4 | v_y | y velocity | m/s | estimated (finite diff + LPF) |
| 5 | v_z | z velocity | m/s | estimated (finite diff + LPF) |
| 6 | phi | roll | rad | /pose.orientation → euler_from_quaternion |
| 7 | theta | pitch | rad | /pose.orientation → euler_from_quaternion |
| 8 | psi | yaw | rad | /pose.orientation → euler_from_quaternion |
| 9 | p | roll rate | rad/s | set to 0.0（仿真环境简化） |
| 10 | q | pitch rate | rad/s | set to 0.0（仿真环境简化） |
| 11 | r | yaw rate | rad/s | set to 0.0（仿真环境简化） |
| 12 | time | timestep counter | — | self._timestep (incremented per plan) |

### 3.2 Control Vector Layout (8D)

根据 `ground_mdp_control_labels`：

| Index | Label | Meaning | Used? |
|-------|-------|---------|-------|
| 0 | unused_0 | — | No |
| 1 | unused_1 | — | No |
| 2 | unused_2 | — | No |
| **3** | **thrust_z** | Thrust (N) | **Yes** |
| **4** | **tau_x** | Roll torque (Nm) | **Yes** |
| **5** | **tau_y** | Pitch torque (Nm) | **Yes** |
| **6** | **tau_z** | Yaw torque (Nm) | **Yes** |
| 7 | unused_7 | — | No |

发布到 `/planner/control_seq` 时仅提取有效控制量（indices 3-6），每步 4 个值。

### 3.3 ROS Message Schemas

#### /planner/trajectory (`nav_msgs/Path`)

```yaml
header:
  seq: <auto>
  stamp: <plan_time>
  frame_id: "world"  # configurable via ~trajectory_frame
poses:
  - header:
      stamp: <plan_time>
      frame_id: "world"
    pose:
      position: {x: state[0], y: state[1], z: state[2]}
      orientation: <yaw-as-quaternion from state[8]>  # only yaw non-zero
  - ...  # one per trajectory step
```

注意事项：
- z 坐标需要从 NED (z-down) 转回 ENU (z-up)：`pose.position.z = -state[2]`
- 偏航：`state[8]` 是 NED 偏航（绕 z-down 的正方向），转 quaternion 时保持不变（xy 平面旋转）
- `frame_id` 通过参数 `~trajectory_frame` 配置，默认 `"world"`

#### /planner/control_seq (`std_msgs/Float64MultiArray`)

```yaml
layout:
  dim:
    - label: "steps"
      size: <num_steps>
      stride: <num_steps * 4>
    - label: "controls"
      size: 4
      stride: 4
  data_offset: 0
data: [thrust_z_0, tau_x_0, tau_y_0, tau_z_0, thrust_z_1, tau_x_1, tau_y_1, tau_z_1, ...]
```

#### /cmd_vel (`geometry_msgs/Twist`)

与现有 `stanley.py` 完全一致：
```yaml
linear:
  x: <target_velocity * factor_v>  # 前进速度
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: <angular_velocity>  # v/L * tan(delta), clipped
```

---

## 4. Node API Design

### 4.1 drone_planner_node

#### 话题

| Topic | Direction | Type | QoS |
|-------|-----------|------|-----|
| `/pose` | Sub | `geometry_msgs/PoseStamped` | queue_size=1 |
| `/planner/trajectory` | Pub | `nav_msgs/Path` | queue_size=1 |
| `/planner/control_seq` | Pub | `std_msgs/Float64MultiArray` | queue_size=1 |
| `/drone_planner/status` | Pub | `std_msgs/String` | queue_size=10 |

#### 服务

| Service | Type | Description |
|---------|------|-------------|
| `~arm` | `std_srvs/SetBool` | true=READY→RUNNING, false=RUNNING→READY |
| `~estop` | `std_srvs/Trigger` | 任何状态→INIT，命令清零 |
| `~update_target` | `std_srvs/Trigger` | 从 param server 读取 `~target_pos` 并更新 MDP |

#### 参数

| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `~config_name` | string | `"policy_convergence_drone"` | YAML 配置名 |
| `~pose_topic` | string | `"/pose"` | 位姿话题名 |
| `~trajectory_topic` | string | `"/planner/trajectory"` | 轨迹输出话题 |
| `~control_seq_topic` | string | `"/planner/control_seq"` | 控制序列输出话题 |
| `~trajectory_frame` | string | `"world"` | Path 消息的 frame_id |
| `~status_topic` | string | `"/drone_planner/status"` | 状态输出话题 |
| `~planner_rate` | float | `2.0` | 规划频率 (Hz) |
| `~uct_N` | int | `20000` | UCT 采样数（来自 YAML default） |
| `~uct_max_depth` | int | `12` | UCT 最大深度 |
| `~uct_c` | float | `3.0` | UCT 探索常数 |
| `~uct_wct` | float | `1200.0` | UCT 时间预算 (s) |
| `~uct_mpc_depth` | int | `2` | MPC 滚动深度 |
| `~uct_dt` | float | `0.01` | 离散时间步长 (s) |
| `~auto_arm` | bool | `true` | 收到首位姿后自动进入 RUNNING |
| `~pose_timeout` | float | `4.0` | 位姿超时 (s) |
| `~target_pos` | list | YAML `ground_mdp_xd` | 13 维目标状态 |
| `~mocap_frame` | string | `"ENU"` | 输入坐标系 |
| `~mocap_to_planner_rz_deg` | float | `0.0` | Z 轴旋转校正 |
| `~mocap_to_planner_scale_xyz` | list | `[1.0,1.0,1.0]` | 缩放因子 |
| `~seed` | int | `0` | 随机种子 |

### 4.2 stanley_controller_node

#### 话题

| Topic | Direction | Type | QoS |
|-------|-----------|------|-----|
| `/pose` | Sub | `geometry_msgs/PoseStamped` | queue_size=10 |
| `/planner/trajectory` | Sub | `nav_msgs/Path` | queue_size=1 |
| `/cmd_vel` | Pub | `geometry_msgs/Twist` | queue_size=10 |

#### 参数

| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `~k` | float | `0.5` | Stanley 控制增益 |
| `~Kp` | float | `1.0` | 速度 PID 比例增益 |
| `~L` | float | `0.2` | 轴距 (m) |
| `~max_steer` | float | `0.5236` (30°) | 最大转向角 (rad) |
| `~target_velocity` | float | `0.2` | 目标线速度 (m/s) |
| `~max_w` | float | `0.5` | 最大角速度 (rad/s) |
| `~factor_v` | float | `1.0` | 线速度缩放系数 |
| `~factor_w` | float | `1.0` | 角速度缩放系数 |
| `~spline_ds` | float | `0.01` | 样条插值间距 (m) |
| `~trajectory_timeout` | float | `0.5` | 轨迹超时后立即停止 (s) |
| `~pose_topic` | string | `"/pose"` | 位姿话题名 |
| `~trajectory_topic` | string | `"/planner/trajectory"` | 轨迹输入话题 |
| `~cmd_vel_topic` | string | `"/cmd_vel"` | 控制输出话题 |
| `~control_rate` | float | `30.0` | 控制发布频率 (Hz) |

---

## 5. Business Logic

### 5.1 Planner: State Estimation & Frame Conversion

```
Input: PoseStamped msg from /pose (ENU frame, z-up)

1. Extract position: (px_enu, py_enu, pz_enu) from msg.pose.position
2. Extract quaternion → euler: (roll, pitch, yaw) via quaternion_to_rpy()
3. Transform ENU → NED (planner frame):
   - pz_ned = -pz_enu
   - pitch_ned = -pitch_enu
   - yaw_ned = -yaw_enu
4. Estimate velocity (finite difference + first-order LPF):
   - raw_vel = (pos_current - pos_prev) / dt
   - est_vel = est_vel + alpha * (raw_vel - est_vel)
   - alpha = dt / (vel_lpf_tau + dt), clamped to [0, 1]
5. Apply same frame transform to velocity (ENU → NED)
6. Build state vector: see Section 3.1
7. Validate via mdp.is_state_valid(state)
```

### 5.2 Planner: UCT-MPC Cycle

```
Input: 13D state vector (in NED planner frame)

1. Call: result = run_uct2(self._dots_mdp, self._uct, state, self._rng)
2. If result.success:
   a. Extract state trajectory: xs = np.array(result.planned_traj.xs)  # shape (T, 13)
   b. Extract control trajectory: us = np.array(result.planned_traj.us)  # shape (T, 8)
   c. Build Path message:
      - For each row in xs:
        - position: (xs[i, 0], xs[i, 1], -xs[i, 2])  # NED→ENU for z
        - orientation: quaternion from yaw xs[i, 8]
      - Set header.stamp = now, header.frame_id = trajectory_frame
   d. Build Float64MultiArray:
      - Extract us[:, 3:7] (only active controls)
      - Set layout.dim = [{label:"steps", size:T, stride:T*4}, {label:"controls", size:4, stride:4}]
      - data = us[:, 3:7].flatten().tolist()
   e. Publish both messages
3. If not result.success:
   - Log warning, skip publish (controller will timeout and stop)
```

### 5.3 Controller: Stanley Tracking with Cubic Spline

```
Input: Path msg, current PoseStamped

1. On Path message received:
   a. Extract N waypoints: (x_i, y_i, yaw_i) for i = 0..N-1
   b. Build cubic spline: cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds)
   c. Store spline arrays; update last_idx = len(cx) - 1
   d. Update last_trajectory_time = now

2. On control timer (30 Hz):
   a. Check trajectory timeout: if now - last_trajectory_time > trajectory_timeout:
      - Publish zero Twist
      - return
   b. Extract current state from latest /pose:
      - state.x, state.y from position
      - state.yaw from orientation (euler_from_quaternion)
      - state.v = target_velocity (constant)
   c. Find target index: target_idx, error_front_axle = calc_target_index(state, cx, cy)
   d. Stanley control:
      - theta_e = normalize_angle(cyaw[target_idx] - state.yaw)
      - theta_d = arctan2(k * error_front_axle, state.v)
      - delta = theta_e + theta_d
      - delta = clip(delta, -max_steer, max_steer)
   e. Compute angular velocity:
      - w = state.v / L * tan(delta)
      - w = clip(w, -max_w, max_w)
   f. Speed control:
      - v_out = Kp * (target_velocity - state.v) + target_velocity  # simplified
      - v_out = clip(v_out, ...)
   g. Publish Twist:
      - linear.x = v_out * factor_v
      - angular.z = w * factor_w
   h. Update last_idx = target_idx (for forward-only tracking)
```

**关键差异 vs 现有 `stanley.py`：**
- 轨迹来源从硬编码 `ax/ay` 改为订阅 `nav_msgs/Path`
- 添加轨迹超时保护（0.5s 无轨迹 → 零速度）
- 控制循环从 `rospy.spin()` + callback 改为定时器驱动（保证固定频率）
- `State` 类中 `v` 字段改为可更新的实例变量

### 5.4 State Machine (Planner)

```
INIT ──(first pose received)──► READY ──(first plan success)──► RUNNING
  ▲                               ▲                                │
  │                               │                                │
  └──(estop)──────────────────────┴──(estop)───────────────────────┘
```

- **INIT**：等待首位姿消息。收到后自动转为 READY。
- **READY**：位姿流正常运行。若 `auto_arm=true`，首次规划成功后自动转 RUNNING；否则等待 `~arm` 服务。
- **RUNNING**：正常规划+发布。`~arm(false)` 可转回 READY。
- 无 FAILSAFE 状态（仿真环境，不需要硬件安全超时）

### 5.5 Controller: Trajectory Timeout FSM

```
IDLE ──(Path received)──► TRACKING ──(timer tick)──► publish cmd_vel
    ◄──(timeout 0.5s)───
    ◄──(estop-like? )─── (future)
```

- **IDLE**：无轨迹，发布零速度
- **TRACKING**：有有效轨迹，正常跟踪
- 轨迹超时（`now - last_trajectory_time > trajectory_timeout`）时立即切回 IDLE，发布零速度
- 新的 Path 消息到达时立即切回 TRACKING，重建样条路径

### 5.6 Edge Cases

| Case | Handling |
|------|----------|
| 首个 Path 消息到达前 | Controller 处于 IDLE，发布零 Twist |
| Path 包含 0 个或 1 个位姿 | Controller 跳过样条构建，保持 IDLE，log warning |
| Planner 规划失败 (`success=false`) | 不发布新轨迹/控制序列，Controller 超时后停止 |
| 位姿长时间不变（无人机卡住） | Controller 正常跟踪到最近点，输出小/零控制量 |
| Path 中 z 坐标为正值（ENU） | Planner 发布时已转为 ENU（z-up），Controller 不处理 z |
| Path frame_id 不匹配 | 不做 frame 转换，直接使用 x,y 坐标 |
| `~update_target` 在 INIT 状态调用 | 更新 MDP 目标但不规划（需先收到位姿进入 READY/RUNNING） |

---

## 6. Error Handling

### 6.1 Error Taxonomy

| Error | Severity | Handling |
|-------|----------|----------|
| C++ binding import failure | FATAL | 节点退出，log error |
| YAML config not found | FATAL | 节点退出，log error |
| MDP/DOTS/UCT init failure | FATAL | 节点退出，log error |
| `is_state_valid` returns false | WARN | 跳过本次规划，log warning throttle 5s |
| `run_uct2` throws exception | ERROR | 捕获并 log traceback，plan_success=false |
| `run_uct2` returns success=false | WARN | log warning，不发布新轨迹 |
| Pose timeout (>4s) | WARN | log warning，planner 暂停规划 |
| `/pose` message missing orientation | WARN | 跳过本次回调 |

### 6.2 Failure Modes

| Failure | Planner Behavior | Controller Behavior |
|---------|-----------------|---------------------|
| Gazebo 未启动（无 /pose） | 停留在 INIT | 停留在 IDLE（零速度） |
| Planner 崩溃 | — | 轨迹超时→零速度停止 |
| Controller 崩溃 | 继续发布轨迹（无影响） | — |
| UCT 超时（wall clock exceeded） | success=false，跳过本周期 | 使用最后有效轨迹继续跟踪直到超时 |

---

## 7. Performance

### 7.1 Timing Budget

| Component | Frequency | Period | Budget |
|-----------|-----------|--------|--------|
| Planner cycle | 2 Hz (default) | 500 ms | `run_uct2` 必须 < 500 ms |
| Controller cycle | 30 Hz (default) | 33 ms | Stanley 计算 < 1 ms |
| Pose callback | ~100-500 Hz | 2-10 ms | 位姿复制+滤波 < 0.1 ms |

### 7.2 Optimization Notes

- UCT 参数从 YAML 继承：`uct_N: 20000`, `uct_wct: 1200.0` — 这是离线参数。在线场景通过 launch arg `uct_N` 覆盖为较小值（如 500-2000）以确保实时性。
- `run_uct2` 是阻塞调用，在 planner timer callback 中同步执行。若上一周期未完成，跳过本周期（`_planning_active` 标志位）。
- Path 消息中位姿数量 = `uct_max_depth * dots_decision_making_horizon` ≈ 12 * 60 = 720 个位姿，消息大小约 720 × ~100 bytes ≈ 72 KB，在 2 Hz 下带宽可接受。
- 样条构建 O(N)，N 为 Path 位姿数。对于 ~720 点，样条构建 < 5 ms，仅在收到新 Path 时触发。

---

## 8. Testing Strategy

### 8.1 Unit Tests (Dry-Run / No ROS)

| Test | Method | Validates |
|------|--------|-----------|
| Planner init | 实例化节点类（无 ROS），验证 MDP/DOTS/UCT 创建成功 | US-001 |
| State vector build | 给定模拟 PoseStamped，验证 state 数组各索引值 | US-003 |
| Path message build | 给定模拟 xs matrix，验证 nav_msgs/Path 内容正确 | US-002 |
| Control seq format | 给定模拟 us matrix，验证 Float64MultiArray layout + data | US-002 |
| Stanley steering calc | 给定 state + 样条路径，验证 delta 计算值 | US-004 |
| Trajectory timeout | 模拟时钟前进 >0.5s，验证输出零 Twist | US-004 |

### 8.2 Integration Tests

| Test | Method | Validates |
|------|--------|-----------|
| Launch smoke test | `roslaunch sets drone_planner.launch`，验证两节点均启动 | US-005 |
| Topic connectivity | `rostopic list` 验证所有话题存在，`rostopic info` 验证类型 | US-005 |
| Trajectory publication | `rostopic echo /planner/trajectory` 验证收到有效 Path | US-002 |
| Control loop closed | `rostopic echo /cmd_vel` 验证收到非零 Twist | US-004 |
| Service availability | `rosservice list` 验证 `~arm`, `~estop`, `~update_target` | US-001 |

### 8.3 End-to-End Test (Gazebo)

| Test | Method | Validates |
|------|--------|-----------|
| Full pipeline | 启动 Gazebo + launch 文件，观察无人机运动 | US-006 |
| Obstacle avoidance | 验证无人机轨迹绕过 YAML 中定义的障碍物 | US-006 |
| Target update | 运行时调用 `~update_target`，验证轨迹方向改变 | US-006 |
| Trajectory loss stop | 杀掉 planner 节点，验证 controller 在 <1s 内停止 | US-006 |

### 8.4 Acceptance Criteria Mapping

| US/FR | Test Type | Description |
|-------|-----------|-------------|
| US-001 | Unit + Integration | Planner 节点加载配置、C++ 绑定、状态机转换、服务响应 |
| US-002 | Unit + Integration | MPC 周期运行、Path 消息正确性、control_seq 格式 |
| US-003 | Unit | 位姿→13D 状态转换、速度估计、坐标系变换 |
| US-004 | Unit + Integration | Stanley 控制律、轨迹超时零速度、参数可配 |
| US-005 | Integration | Launch 启动双节点、topic 连通性 |
| US-006 | E2E (Gazebo) | 完整闭环、障碍物规避、动态目标切换 |

---

## 9. Launch File Design

### 9.1 `launch/drone_planner.launch`

```xml
<launch>
  <arg name="config_name"     default="policy_convergence_drone"/>
  <arg name="uct_N"           default="500"/>
  <arg name="uct_max_depth"   default="12"/>
  <arg name="uct_c"           default="3.0"/>
  <arg name="uct_wct"         default="1200.0"/>
  <arg name="planner_rate"    default="2.0"/>
  <arg name="control_rate"    default="30.0"/>
  <arg name="k"               default="0.5"/>
  <arg name="target_velocity" default="0.2"/>
  <arg name="trajectory_frame" default="world"/>

  <!-- Planner Node -->
  <node name="drone_planner" pkg="sets" type="drone_planner_node.py"
        output="screen" respawn="false">
    <param name="config_name"       value="$(arg config_name)"/>
    <param name="pose_topic"        value="/pose"/>
    <param name="trajectory_topic"  value="/planner/trajectory"/>
    <param name="control_seq_topic" value="/planner/control_seq"/>
    <param name="trajectory_frame"  value="$(arg trajectory_frame)"/>
    <param name="status_topic"      value="/drone_planner/status"/>
    <param name="planner_rate"      value="$(arg planner_rate)"/>
    <param name="uct_N"             value="$(arg uct_N)"/>
    <param name="uct_max_depth"     value="$(arg uct_max_depth)"/>
    <param name="uct_c"             value="$(arg uct_c)"/>
    <param name="uct_wct"           value="$(arg uct_wct)"/>
    <param name="uct_dt"            value="0.01"/>
    <param name="auto_arm"          value="true"/>
    <param name="pose_timeout"      value="4.0"/>
    <param name="mocap_frame"       value="ENU"/>
    <param name="mocap_to_planner_rz_deg" value="0.0"/>
    <param name="mocap_to_planner_scale_xyz" value="[1.0, 1.0, 1.0]"/>
    <param name="seed"              value="0"/>
  </node>

  <!-- Stanley Controller Node -->
  <node name="stanley_controller" pkg="sets" type="stanley_controller_node.py"
        output="screen" respawn="false">
    <param name="pose_topic"          value="/pose"/>
    <param name="trajectory_topic"    value="/planner/trajectory"/>
    <param name="cmd_vel_topic"       value="/cmd_vel"/>
    <param name="k"                   value="$(arg k)"/>
    <param name="target_velocity"     value="$(arg target_velocity)"/>
    <param name="control_rate"        value="$(arg control_rate)"/>
    <param name="trajectory_timeout"  value="0.5"/>
    <param name="L"                   value="0.2"/>
    <param name="max_steer"           value="0.5236"/>
    <param name="max_w"               value="0.5"/>
    <param name="factor_v"            value="1.0"/>
    <param name="factor_w"            value="1.0"/>
    <param name="spline_ds"           value="0.01"/>
  </node>
</launch>
```

---

## 10. Implementation Plan

### 10.1 Phases

| Phase | Files | Depends On | Description |
|-------|-------|------------|-------------|
| 1 | `drone_planner_node.py` | — | Planner 核心框架：类结构、参数加载、C++ 绑定初始化、状态机 |
| 2 | `drone_planner_node.py` | Phase 1 | 位姿订阅、状态估计、坐标变换 |
| 3 | `drone_planner_node.py` | Phase 2 | MPC 定时循环、轨迹发布（Path + control_seq） |
| 4 | `stanley_controller_node.py` | — | Controller 核心：Path 订阅、样条构建、Stanley 控制律 |
| 5 | `stanley_controller_node.py` | Phase 4 | 控制定时器、轨迹超时保护 |
| 6 | `drone_planner.launch` | Phase 3, 5 | Launch 文件 |
| 7 | — | Phase 6 | 集成测试 + E2E 验证 |

### 10.2 Issue Mapping

| US | Phases | Priority | Description |
|----|--------|----------|-------------|
| US-001 | 1, 2 | high | Planner 节点核心框架 |
| US-002 | 3 | high | 规划器定时循环与轨迹发布 |
| US-003 | 2 | high | 位姿订阅与状态估计 |
| US-004 | 4, 5 | high | Stanley 控制器节点 |
| US-005 | 6 | high | Launch 文件 |
| US-006 | 7 | medium | 端到端集成测试 |

### 10.3 Incremental Delivery

各 Phase 可独立测试：
1. Phase 1-2 完成后：启动 Planner，验证参数加载和位姿订阅（日志输出）
2. Phase 3 完成后：`rostopic echo /planner/trajectory` 验证轨迹发布
3. Phase 4-5 完成后：手动发布 Path 消息，验证 Controller 的 `/cmd_vel` 输出
4. Phase 6 完成后：一键启动全系统
5. Phase 7：Gazebo 闭环验证

---

## 11. Open Questions & Risks

### 11.1 Resolved Questions

| Question | Decision |
|----------|----------|
| 轨迹从 13D 状态如何提取？ | 位置 (indices 0,1,2) + 偏航 (index 8) |
| Stanley 跟踪方式？ | 三次样条插值 Path 点后跟踪 |
| 控制序列格式？ | Float64MultiArray 展平，每步 4 个有效控制量 |
| `~update_target` 支持？ | 需要支持 |
| 轨迹丢失行为？ | 控制器立即发布零速度 |

### 11.2 Technical Risks

| Risk | Impact | Mitigation |
|------|--------|-----------|
| `uct_N=20000` 在线规划超时（>500ms @2Hz） | Planner 跳周期，轨迹更新不及时 | 默认 launch arg 覆盖为 500；提供参数调优指南 |
| 8D→4D 控制量信息丢失 | Planner 输出的推力+扭矩无法直接用于 Gazebo 速度控制接口 | 已确认：Stanley 不依赖 planner 控制量，独立计算 Twist。规划器输出的 control_seq 仅用于监控/日志 |
| Path 消息过大（>720 点） | 带宽压力，Controller 处理延迟 | 可通过下采样参数控制（`uct_downsample_traj_on: True` 已在 YAML 中启用） |
| NED↔ENU z 轴符号混淆 | 轨迹在错误高度飞行 | 明确文档化：Planner 内部 NED，Path 发布时转 ENU，Controller 直接用 Path 中的 ENU 坐标 |

### 11.3 Assumptions

- Gazebo 仿真环境已在 `/pose` 话题发布 `PoseStamped`（ENU 帧，z-up）
- C++ 绑定 `.so` 文件已编译且可用（`build/bindings.cpython-312-x86_64-linux-gnu.so`）
- `policy_convergence_drone.yaml` 中的障碍物、目标点、奖励参数不需要修改
- Controller 仅使用 2D 平面控制（x, y, yaw），不处理 z 轴高度控制
- 角速度状态 (p, q, r) 在仿真中因缺乏 IMU 数据而设为零，假设这对 MPC 规划精度影响可接受
