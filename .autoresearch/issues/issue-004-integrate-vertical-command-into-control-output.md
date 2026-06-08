# 控制命令聚合中接入纵向链路

## Description
将纵向输出接入节点最终控制命令，确保启停逻辑与现有横向控制共存。

## Acceptance Criteria
- [x] 最终控制命令组装中包含 vz_cmd。
- [x] 仅启用纵向时，横向输出保持基线一致。
- [x] 仅启用横向时，纵向按约定值（0 或保持）输出。
- [x] 横向计算路径未被纵向变量侵入。

## Dependencies
Issue #2, Issue #3

## Type
backend

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
