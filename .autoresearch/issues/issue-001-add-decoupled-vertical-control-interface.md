# 新增解耦纵向控制接口骨架

## Description
建立与横向 Stanley 分离的纵向控制入口与数据流，保证纵向链路不读取横向误差。

## Acceptance Criteria
- [x] 新增独立纵向控制模块或函数入口（命名清晰可检索）。
- [x] 纵向输入仅使用 z_ref、z_meas（可选 vz_ref、vz_meas）。
- [x] 纵向控制路径不读取横向误差变量。
- [x] 控制路径有清晰注释或命名标识横纵解耦。
- [x] 构建通过。

## Dependencies
None

## Type
backend

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
