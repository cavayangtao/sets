# 实现默认 P 纵向控制与 vz_cmd 限幅

## Description
实现初版默认 P 控制器，把 z 误差转换为 vz_cmd，并执行上限和下限约束。

## Acceptance Criteria
- [x] 计算 e_z = z_ref - z_meas。
- [x] 默认策略为 P 控制（可配置 kp_z）。
- [x] 输出 vz_cmd 并应用 vz_limit 上下限饱和。
- [x] e_z 接近 0 时，vz_cmd 收敛到 0（含死区策略）。
- [x] 构建与节点级基础测试通过。

## Dependencies
Issue #1

## Type
backend

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
