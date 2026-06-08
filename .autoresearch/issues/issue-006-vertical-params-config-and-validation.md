# 参数配置与加载校验（YAML 和 launch）

## Description
补齐纵向参数项与加载校验，使控制器可调并具备非法参数保护。

## Acceptance Criteria
- [x] 配置新增 kp_z、vz_limit、z_deadband 等参数。
- [x] launch 或参数加载流程能读取并生效。
- [x] 参数缺失或非法时触发确定性行为（回退默认或启动失败并报错）。
- [x] 参数修改后按项目现状支持重启生效。

## Dependencies
Issue #2

## Type
infra

## Priority
medium

## Verification
- [x] Implemented in current branch.
- [x] Python syntax check passed for updated ROS scripts.
- [x] Relevant automated tests passed (Issue #007 and Issue #008).
