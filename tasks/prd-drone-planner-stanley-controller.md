# PRD: Drone Trajectory Planner + Stanley Controller

## Introduction

构建两个 ROS 1 节点组成仿真控制系统：Drone Planner 节点使用 `policy_convergence_drone.yaml` 配置运行在线 MPC，滚动输出飞行轨迹；Stanley Controller 节点接收轨迹并计算 `/cmd_vel` 控制量。两个节点通过 launch 文件统一启动，用于 Gazebo 纯仿真环境。

## Goals

- 实现基于 `policy_convergence_drone.yaml` 的在线 MPC 规划器节点，周期性滚动输出未来轨迹
- 实现接收 `nav_msgs/Path` 轨迹的 Stanley 控制器节点，输出 `geometry_msgs/Twist` 到 `/cmd_vel`
- 通过 launch 文件一键启动两个节点，参数可通过 launch args 覆盖
- 规划器和控制器解耦，可独立替换或测试

## User Stories

### US-001: 创建 Drone Planner 节点核心框架
**Description:** As a 开发者，我需要一个基于 `policy_convergence_drone.yaml` 配置的 ROS 规划器节点，能够加载 C++ 绑定、初始化 MDP/求解器，并具备状态机管理（INIT → READY → RUNNING）。

**Acceptance Criteria:**
- [ ] 节点从 `configs/sixdofaircraft/policy_convergence_drone.yaml` 加载配置
- [ ] 通过 pybind11 加载 C++ 绑定（`get_mdp`, `get_uct2`, `run_uct2`）
- [ ] 实现状态机：INIT → READY → RUNNING（仿真模式无需 FAILSAFE）
- [ ] 提供 `~arm` (SetBool)、`~estop` (Trigger) 和 `~update_target` (Trigger) 服务
- [ ] 所有参数通过 ROS 参数服务器可配置，YAML 为默认值来源
- [ ] Typecheck/lint 通过

### US-002: 实现规划器定时循环与轨迹发布
**Description:** As a 开发者，我需要规划器节点以可配置频率运行 MPC，计算未来轨迹并将轨迹以 `nav_msgs/Path` 发布到话题。

**Acceptance Criteria:**
- [ ] 定时器以可配置频率（默认 2 Hz）触发规划循环
- [ ] 每次规划使用当前估计状态作为初始状态，调用 `run_uct2` 得到动作序列
- [ ] 通过前向模拟将动作序列展开为状态轨迹（位置序列）
- [ ] 将轨迹发布到 `/planner/trajectory` 话题（`nav_msgs/Path`）
- [ ] 将控制序列发布到 `/planner/control_seq` 话题（`std_msgs/Float64MultiArray`）
- [ ] 规划器状态和轨迹信息通过 ROS 日志输出（INFO 级别）
- [ ] Typecheck/lint 通过

### US-003: 实现位姿订阅与状态估计
**Description:** As a 开发者，我需要规划器节点订阅仿真环境中的无人机位姿（`/pose`），并估计速度等不可直接观测的状态量，作为 MPC 的初始状态。

**Acceptance Criteria:**
- [ ] 订阅 `/pose`（`geometry_msgs/PoseStamped`）获取当前位姿
- [ ] 通过位姿差分 + 低通滤波估计线速度和角速度
- [ ] 构建完整的 13 维状态向量（位置、姿态四元数、速度、角速度）
- [ ] 支持 ENU/NED 坐标系转换（通过参数配置）
- [ ] 位姿超时检测（可配置超时时间，默认 4.0s）
- [ ] Typecheck/lint 通过

### US-004: 创建 Stanley 控制器节点
**Description:** As a 开发者，我需要一个 ROS 控制器节点，订阅规划器发布的轨迹和当前位姿，使用 Stanley 控制律计算并发布 `/cmd_vel`。

**Acceptance Criteria:**
- [ ] 订阅 `/planner/trajectory`（`nav_msgs/Path`）接收目标轨迹
- [ ] 订阅 `/pose`（`geometry_msgs/PoseStamped`）获取当前位姿
- [ ] 实现 Stanley 横向控制律：`delta = theta_e + arctan(k * cross_track_error / v)`
- [ ] 发布 `geometry_msgs/Twist` 到 `/cmd_vel`
- [ ] 控制器参数可通过 ROS 参数服务器配置：`k`, `Kp`, `L`（轴距）, `max_steer`, `target_velocity`
- [ ] 当未收到轨迹或轨迹超时时，立即发布零速度命令（安全停止，不保持最后轨迹）
- [ ] Typecheck/lint 通过

### US-005: 创建 Launch 文件
**Description:** As a 开发者，我需要一个 launch 文件同时启动 planner 和 controller 两个节点，并支持关键参数通过 launch args 覆盖。

**Acceptance Criteria:**
- [ ] 创建 `launch/drone_planner.launch`（XML 格式，ROS 1）
- [ ] 同时启动 `drone_planner_node.py` 和 `stanley_controller_node.py`
- [ ] 支持 launch args：`config_name`, `uct_N`, `uct_max_depth`, `uct_c`, `planner_rate`, `cmd_pub_rate`, `k`, `target_velocity`
- [ ] 可通过 `roslaunch sets drone_planner.launch` 一键启动
- [ ] 启动后两个节点均正常运行，无崩溃或连接错误

### US-006: 端到端集成测试
**Description:** As a 开发者，我需要验证 planner → trajectory → controller → /cmd_vel 的完整数据流在仿真环境中正常工作。

**Acceptance Criteria:**
- [ ] 在 Gazebo 仿真中启动 launch 文件，无人机模型能正常接收 `/cmd_vel`
- [ ] 规划器周期性发布轨迹，控制器周期性发布控制指令
- [ ] 无人机在仿真中沿规划轨迹移动，不出现失控或震荡
- [ ] `rostopic echo /planner/trajectory` 能看到有效的 Path 消息
- [ ] `rostopic echo /cmd_vel` 能看到非零的 Twist 消息

## Functional Requirements

- FR-1: 系统必须通过 `roslaunch sets drone_planner.launch` 一键启动所有节点
- FR-2: Drone Planner 节点必须从 `policy_convergence_drone.yaml` 加载 MDP 配置（2.0 kg 四旋翼，8 维推力+扭矩控制）
- FR-3: Drone Planner 节点必须以可配置频率周期性运行 UCT-MPC，输出未来 N 步轨迹
- FR-4: Drone Planner 节点必须将轨迹以 `nav_msgs/Path` 发布到 `/planner/trajectory`
- FR-5: Drone Planner 节点必须将控制序列以 `Float64MultiArray` 发布到 `/planner/control_seq`
- FR-6: Stanley Controller 节点必须订阅 `/planner/trajectory` 并使用 Stanley 控制律生成 `/cmd_vel`
- FR-7: Stanley Controller 节点必须在未收到轨迹或轨迹超时时立即发布零速度（立即停止，不维持最后轨迹继续跟踪）
- FR-8: 系统必须支持通过 ROS 参数服务器覆盖关键参数
- FR-9: Drone Planner 节点必须提供 `~update_target` 服务，支持运行时动态切换飞行目标

## Non-Goals (Out of Scope)

- 不支持真实硬件（无 MoCap 集成，无 Tello 驱动）
- 不支持硬件安全超时或 FAILSAFE 状态（纯仿真环境）
- 不实现自定义 ROS 消息类型（仅使用标准消息）
- 不处理 8 维控制量到 Twist 的映射（Stanley 独立计算，不依赖 planner 的控制输出）
- 不实现动态障碍物规避（使用 YAML 中的静态障碍物）
- 不实现多机协同规划

## Design Considerations

### 架构概览

```
仿真环境 (Gazebo)
     │
     ├── /pose (PoseStamped)
     │        │
     │        ├──► drone_planner_node ──► /planner/trajectory (Path)
     │        │                                  │
     │        │                                  ├──► stanley_controller_node
     │        │                                  │         │
     │        └──────────────────────────────────┘         │
     │                                                     │
     │                                              /cmd_vel (Twist)
     │                                                     │
     └─────────────────────────────────────────────────────┘
                                                    ▼
                                              无人机模型
```

### 节点职责

| 组件 | 订阅 | 发布 | 参数 |
|-------|-------------|----------|--------|
| `drone_planner_node` | `/pose` | `/planner/trajectory`, `/planner/control_seq` | `config_name`, `uct_N`, `uct_max_depth`, `uct_c`, `uct_wct`, `planner_rate` |
| `stanley_controller_node` | `/planner/trajectory`, `/pose` | `/cmd_vel` | `k`, `Kp`, `L`, `max_steer`, `target_velocity` |

### 代码复用策略

- `drone_planner_node.py` 参考 `tello_planner_node.py` 的骨架：C++ 绑定加载、状态机框架、定时器调度、参数加载。移除：FAILSAFE 状态、cmd_vel 直接输出、MoCap 坐标系逻辑（简化为仿真环境）。
- `stanley_controller_node.py` 参考 `stanley.py` 的 Stanley 控制律核心算法。修改：输入从硬编码路径改为订阅 `/planner/trajectory`。
- 启动文件参考 `launch/tello_planner.launch` 的多参数格式。

## Technical Considerations

- **ROS 版本**：ROS 1 (rospy / roscpp)
- **C++ 绑定**：依赖 `build/bindings.cpython-312-x86_64-linux-gnu.so`（pybind11）
- **配置**：`policy_convergence_drone.yaml` 使用 `mode: "quadrotor"`（不是 `quadrotor_vbody_yaw`），控制为 `[thrust_z, tau_x, tau_y, tau_z]` 共 8 维
- **状态向量**：13 维（位置 3、四元数 4、线速度 3、角速度 3）
- **工作空间**：(-120, -120, -120) 到 (120, 120, -120)，远大于 Tello 场景
- **消息类型**：`nav_msgs/Path`, `geometry_msgs/Twist`, `geometry_msgs/PoseStamped`, `std_msgs/Float64MultiArray`

## Success Metrics

- `roslaunch sets drone_planner.launch` 能成功启动两个节点
- `/planner/trajectory` 话题能收到周期性的 Path 消息（规划频率可验证）
- `/cmd_vel` 话题能收到非零 Twist 消息（控制器正常工作）
- Gazebo 仿真中无人机沿规划轨迹移动，不出现发散或震荡
- 调节 Stanley 参数（k、target_velocity）后跟踪行为有明显变化

## Resolved Questions

- **`~update_target` 服务**：需要支持，允许运行时动态切换飞行目标
- **轨迹丢失行为**：控制器立即停止（发布零速度），不维持最后轨迹
