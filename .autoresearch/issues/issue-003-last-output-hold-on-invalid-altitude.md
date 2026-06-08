# 异常高度输入的保持策略与日志

## Description
对高度数据缺失或非法场景实现保持上一时刻输出的安全退化，避免突变控制。

## Acceptance Criteria
- [x] 识别高度输入异常（缺失、NaN、超界等至少一种可配置规则）。
- [x] 异常时输出保持 last_vz_cmd，而非重置为 0。
- [x] 策略触发时记录可检索日志（含触发原因）。
- [x] 恢复有效输入后可回到正常控制路径。

## Dependencies
Issue #2

## Type
backend

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
