# Tello Planner ROS Node — 设计文档

## 1. 架构概览

```
MoCap (30+ Hz)                   Tello 真机
    │                                │
    ▼                                ▼
┌──────────────────────────────────────────────┐
│              tello_planner_node.py            │
│                                              │
│  ┌──────────┐  ┌──────────┐  ┌────────────┐  │
│  │ pose_cb  │  │ planner  │  │ cmd_pub    │  │
│  │ (回调)   │  │ timer    │  │ timer      │  │
│  │          │  │ 2 Hz     │  │ 30 Hz      │  │
│  │ 位姿→    │  │          │  │            │  │
│  │ 状态向量  │──▶ UCT规划  │──▶ 饱和限幅   │──▶ /cmd_vel
│  │ 速度估计  │  │ N=500    │  │            │  │
│  └──────────┘  └──────────┘  └────────────┘  │
│                                              │
│  ┌──────────────────────────────────────┐    │
│  │  watchdog timer (10 Hz)              │    │
│  │  位姿超时/指令超时 → FAILSAFE        │    │
│  └──────────────────────────────────────┘    │
│                                              │
│  服务: ~arm  ~estop  ~update_target          │
└──────────────────────────────────────────────┘
    │                                │
    ▼                                ▼
C++ bindings (build/bindings.cpython-*.so)
    │
    ├── get_mdp()        MDP 模型（动力学、障碍物、目标）
    ├── get_dots_mdp()   DOTS 谱展开动力学
    ├── get_uct2()       UCT 蒙特卡洛树搜索
    └── run_uct2()       单次规划调用

YAML 配置 (policy_convergence_tello_stage3.yaml)
    │
    └── 飞行器参数、障碍物、约束、算法超参
```

## 2. 数据流

### 2.1 输入路径（MoCap → 规划器状态）

```
PoseStamped (世界系位姿)
  │ position (x,y,z)    → 坐标系变换 (ENU→NED 等)
  │ orientation (四元数) → quaternion_to_rpy() → (φ,θ,ψ)
  │
  ▼
13 维状态向量:
  [p_x, p_y, p_z, v_x, v_y, v_z, φ, θ, ψ, p, q, r, time]
    世界位置      世界速度(差分+LPF) 姿态     角速度(暂为0) 时间步

说明：位置与速度必须使用同一坐标约定。`mocap_frame`、`mocap_to_planner_rz_deg`、`mocap_to_planner_scale_xyz`
对位置与速度同时生效，避免出现“位置已转换、速度未转换”的状态不一致。

速度估计实现（当 `use_estimated_velocity=true`）：
- 原始差分：`v_raw = (p_k - p_{k-1}) / dt`
- 一阶低通：`v_k = v_{k-1} + α (v_raw - v_{k-1})`
- 其中 `α = dt / (vel_lpf_tau + dt)`，并对过大 `dt`（`dt > vel_diff_max_dt`）执行保护重置。
```

坐标系变换 `transform_pose()` 支持三种模式：

| mocap_frame | 变换 |
|-------------|------|
| `ENU` (默认) | z→-z, pitch→-pitch, yaw→-yaw |
| `NED` | 直通 |
| `NED_ZY` | 轴交换 |

### 2.2 输出路径（规划器动作 → cmd_vel）

```
UCT 规划 → planned_traj.us[0,:] = [v_bx_cmd, v_by_cmd, v_bz_cmd, yaw_rate_cmd]
                                          │         │         │          │
                                          ▼         ▼         ▼          ▼
                              Twist.linear.x  .linear.y  .linear.z  .angular.z
```

输出前经过 `_saturate_command()` 限幅，默认限幅值从 YAML `ground_mdp_U` 读取。

## 3. 参数分层

```
┌──────────────────────────────────────────┐
│  YAML (policy_convergence_tello_stage3)  │  ← 唯一数据源
│  动力学、障碍物、起点/目标、算法参数      │
├──────────────────────────────────────────┤
│  ROS param (launch 文件 / 命令行)        │  ← 运行时覆盖
│  话题名、频率、安全阈值、坐标系           │
└──────────────────────────────────────────┘
```

### 3.1 YAML 控制（需重启生效）

| 参数 | 说明 |
|------|------|
| `ground_mdp_name` | MDP 模型类型（SixDOFAircraft） |
| `mass, Ixx, Iyy, Izz` | 飞行器质量/惯性 |
| `ground_mdp_X` | 状态边界 |
| `ground_mdp_U` | 控制量边界（指令限幅默认值） |
| `ground_mdp_x0 / xd` | 起点 / 目标点（13 维） |
| `ground_mdp_obstacles` | 静态障碍物 |
| `dots_*` | DOTS 谱展开参数 |
| `uct_*` | UCT 搜索参数 |
| `flight_mode` | 飞行模式（quadrotor_vbody_yaw） |
| `reward_parameters` | 奖励函数权重 |

### 3.2 ROS param（launch 设定，可运行时覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_name` | `policy_convergence_tello_stage3` | YAML 配置名 |
| **话题** | | |
| `pose_topic` | `/mocap/pose` | MoCap 位姿输入 |
| `cmd_vel_topic` | `/cmd_vel` | 速度指令输出 |
| `status_topic` | `/tello_planner/status` | 状态诊断输出 |
| **频率** | | |
| `planner_rate` | 2.0 Hz | 规划触发频率 |
| `cmd_pub_rate` | 30.0 Hz | 指令发布频率 |
| **规划器覆盖** | | |
| `uct_N` | 500 | 每次规划模拟次数 |
| `uct_max_depth` | 12 | 搜索深度 |
| `uct_c` | 3.0 | UCT 探索系数 |
| `uct_wct` | 1200.0 | 壁钟时间限制 |
| `uct_dt` | 0.01 | 离散时间步长 |
| `target_pos` | YAML `ground_mdp_xd` | 目标点（13 维列表） |
| **安全** | | |
| `auto_arm` | true | 收到位姿后自动开始规划 |
| `pose_timeout` | 2.0 s | 位姿超时阈值 |
| `max_cmd_hold` | 3.0 s | 指令保持超时 |
| `clip_cmd` | true | 是否启用指令限幅 |
| `use_estimated_velocity` | true | 是否使用位姿差分速度 |
| `vel_lpf_tau` | 0.15 s | 速度估计一阶低通时间常数（<=0 关闭滤波） |
| `vel_diff_max_dt` | 0.5 s | 差分速度允许的最大时间间隔 |
| **坐标系** | | |
| `mocap_frame` | ENU | MoCap 坐标系 |
| `mocap_to_planner_rz_deg` | 0.0 | Z 轴旋转补偿 |
| `mocap_to_planner_scale_xyz` | [1,1,1] | 缩放因子 |
| **其他** | | |
| `pose_msg_type` | PoseStamped | 位姿消息类型（当前仅支持 PoseStamped） |
| `seed` | 0 | 随机种子 |

补充：节点启动时会立即将 `target_pos` 写入 MDP（`set_xd`），因此 launch 中配置的
`target_pos` 在首轮规划前即生效，无需先调用 `~update_target`。

## 4. 状态机

```
                    ┌─────────┐
                    │  INIT   │  等待首帧 MoCap 位姿
                    └────┬────┘
                         │ 收到位姿 (自动)
                         ▼
                    ┌─────────┐
         arm(false)  │  READY  │  arm(true)
        ◄────────────│         │────────────┐
                    └─────────┘            │
                         ▲                 ▼
                         │           ┌──────────┐
                         │           │ RUNNING  │  规划 + 发布非零指令
                         │           └────┬─────┘
                         │                │
                         │        pose_timeout 或
                         │       max_cmd_hold 或
                         │          estop()
                         │                ▼
                         │           ┌──────────┐
                         └───────────│ FAILSAFE │  发布零指令
                           arm(false) └──────────┘
```

**流转规则**：
- INIT → READY：首帧有效位姿到达（自动）
- READY → RUNNING：首次规划成功（`auto_arm=true` 时自动）或手动 `arm(true)`
- RUNNING → FAILSAFE：位姿超时 / 指令超时 / `estop()`
- FAILSAFE → READY：`arm(false)` 手动解锁
- 任意状态 → FAILSAFE：`estop()` 急停

## 5. 双速率调度

```
规划 Timer（2 Hz, 每 500ms 触发）
    │
    ├─ 触发 → run_uct2()（~0.8s @ N=500）→ 更新 self._cmd
    ├─ 触发 → 跳过（上一轮仍在运行）
    └─ 触发 → 下一轮规划开始

指令发布 Timer（30 Hz, 每 33ms）
    │
    └─ 读取 self._cmd → 发布 Twist（规划间隙重复发送）
```

- 有效规划频率受 UCT 耗时限制：N=500 时约 1.2 Hz，N=300 时约 1.0 Hz
- 指令发布速率固定 30 Hz，不受规划耗时影响

## 6. 线程安全

所有共享状态通过 `threading.Lock` 保护：

| 变量 | 写者 | 读者 | 锁保护 |
|------|------|------|:------:|
| `_mocap_pos/quat/stamp` | pose_cb | planner_cb, watchdog | ✅ |
| `_cmd, _cmd_stamp` | planner_cb, arm_cb, estop_cb | cmd_pub, watchdog, status | ✅ |
| `_state` | planner_cb, watchdog, arm_cb, estop_cb | 所有回调 | ✅ |
| `_est_vel` | pose_cb | planner_cb | ✅ |
| `_plan_latency/count/success` | planner_cb | status | ✅ |

## 7. 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `~arm` | `std_srvs/SetBool` | `true`=解锁, `false`=锁定到 READY |
| `~estop` | `std_srvs/Trigger` | 任意状态 → FAILSAFE |
| `~update_target` | `std_srvs/Trigger` | 读取 `~target_pos` 参数并调用 `mdp.set_xd()` |

### 运行时切换目标点

```bash
# 1. 设置新目标
rosparam set /tello_planner/target_pos "[10.0, 5.0, -12.0, 0,0,0,0,0,0,0,0,0,0]"

# 2. 触发更新
rosservice call /tello_planner/update_target
```

第一步修改参数服务器，第二步让 MDP 读到新目标。下一轮 UCT 规划立即生效。

## 8. 诊断输出

`/tello_planner/status` 话题（`std_msgs/String`，10 Hz）：

```
state:RUNNING|plan_count:15|plan_latency_ms:850|plan_success:True|cmd:[-6.50,2.10,0.05,0.80]|timestep:15
```

## 9. 验证结果

### 9.1 离线规划验证（test_tello_trajectory.py）

| 测试项 | 结果 |
|--------|:----:|
| 从 (-20,0,-12) 到达 wp0 (20,0,-12) | ✅ 13 步, dist=0.8m |
| 运行时切换到 wp1 (20,10,-12) | ✅ 10 步, dist=1.5m |
| 绕过 x≈-8 和 x≈+8 处障碍物 | ✅ 南侧绕行 (y≈-4) |
| 总计 22 次规划，两目标均到达 | ✅ |

### 9.2 ROS 节点验证（test_tello_planner.py / test_tello_node_dryrun.py）

| 测试项 | 结果 |
|--------|:----:|
| 状态机 INIT→READY→RUNNING→FAILSAFE | ✅ |
| cmd_vel 在规划阶段产生非零输出（严格断言） | ✅ |
| 与离线规划器主方向一致（严格断言） | ✅ |
| estop 触发零指令 | ✅ |
| arm/disarm 状态切换 | ✅ |

CI 判定口径：
- `test_tello_planner.py` 必须同时满足：出现非零控制、`estop` 后连续零控制、包含 FAILSAFE 状态。
- `test_tello_node_dryrun.py` 必须同时满足：有新非零命令输出，且“主方向符号一致”通过率不低于阈值（默认 35%，可参数化）。
- 任一断言失败即返回非零退出码。

## 10. 使用方式

```bash
# 编译 C++ bindings（一次性）
cd /home/tyang/sets/build && cmake .. && make -j$(nproc)

# 启动（三终端）
# 终端 1: roscore
# 终端 2: rosrun sets tello_planner_node.py
# 终端 3: 可选调参
rosrun sets tello_planner_node.py _uct_N:=300 _mocap_frame:=NED

# 飞行中操作
rosservice call /tello_planner/arm "data: true"     # 解锁
rosservice call /tello_planner/estop                 # 急停
rosparam set /tello_planner/target_pos "[x,y,z,...]" # 设新目标
rosservice call /tello_planner/update_target          # 激活新目标
```

## 11. 已知限制

| 限制 | 影响 | 建议 |
|------|------|------|
| 角速度 p/q/r 硬编码为 0 | 姿态机动时规划精度下降 | 从 MoCap 读取或 IMU 估计 |
| 当前仅支持 PoseStamped 输入，不直接读取外部速度 | 速度完全依赖位姿质量与时间戳 | 如需更稳可接入外部速度估计/IMU 融合 |
| 差分速度滤波参数需按场景调参（`vel_lpf_tau`、`vel_diff_max_dt`） | 参数不当会导致响应迟滞或噪声放大 | 结合 MoCap 频率与噪声水平整定 |
| `pose_timeout` 仅检测 RUNNING 状态 | READY 状态下位姿丢失不报警 | 如需可扩展到所有状态 |
| 不支持 cbds UCT 模式 | 只能用 no_cbds | 按需扩展 |
| FAILSAFE 后需手动 arm(false) 恢复 | 不能自动恢复 | 设计如此（安全考虑） |

## 12. 文件清单

| 文件 | 角色 |
|------|------|
| `scripts/tello_planner_node.py` | ROS 节点主程序 |
| `launch/tello_planner.launch` | ROS 启动文件 |
| `configs/.../policy_convergence_tello_stage3.yaml` | 飞行器+场景配置 |
| `build/bindings.cpython-*.so` | C++ pybind 规划器 |
| `scripts/test_tello_trajectory.py` | 离线规划+多航点测试+绘图 |
| `scripts/test_tello_planner.py` | ROS 烟雾测试 |
| `scripts/test_tello_node_dryrun.py` | ROS 节点 vs 离线一致性验证 |
| `package.xml` | ROS 包清单 |
