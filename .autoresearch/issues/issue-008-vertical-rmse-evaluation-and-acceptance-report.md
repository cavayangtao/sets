# 纵向 RMSE 评估与验收报告（小于 0.15 m）

## Description
建立 z 轴 RMSE 评估流程并产出验收报告。

## Acceptance Criteria
- [x] 对代表性轨迹集计算 z 轴 RMSE。
- [x] RMSE 阈值固定为小于 0.15 m。
- [x] 报告包含测试场景、时长、采样与结果明细。
- [x] 构建与测试流水线无新增回归失败。

## Dependencies
Issue #2, Issue #3, Issue #4, Issue #6

## Type
infra

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
