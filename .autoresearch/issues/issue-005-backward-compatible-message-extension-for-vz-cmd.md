# 无破坏性扩展话题和消息定义

## Description
以兼容方式扩展消息字段承载纵向命令，保证旧订阅方可继续运行。

## Acceptance Criteria
- [x] 消息或话题定义新增 vz_cmd（或等价字段）。
- [x] 扩展方式不破坏现有消费者（旧订阅端可继续运行）。
- [x] 发布者与订阅者编译和联调通过，无接口不匹配错误。
- [x] 联调可观测纵向字段持续更新。

## Dependencies
Issue #4

## Type
backend

## Priority
high

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
