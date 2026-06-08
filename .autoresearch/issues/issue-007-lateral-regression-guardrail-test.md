# 横向回归守护测试（劣化不超过 5%）

## Description
建立回归测试确保新增纵向后横向性能劣化不超过 5%。

## Acceptance Criteria
- [x] 固定基线轨迹与评估口径（横向误差指标定义明确）。
- [x] 改造前后同条件回放并输出对比结果。
- [x] 横向误差劣化不超过 5%。
- [x] 结果可在 CI 或脚本中复现。

## Dependencies
Issue #4, Issue #5

## Type
infra

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
