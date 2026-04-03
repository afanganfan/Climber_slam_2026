# 2026 赛季 MPPI 调参与 Wrapper 方案

## 核心经验（实战版）

- 本赛季局部控制器采用 MPPI，避障效果优于此前配置。
- 调参时不要只盯着采样标准差（`vx_std / vy_std / wz_std`），优先关注：
  - `temperature`
  - `gamma`
- `PathFollowCritic` 与 `PathAlignCritic` 建议约束：
  - `cost_weight <= 10`
  - `threshold_to_consider <= 1.0`
- 想提升速度时，可参考官方建议（原文中的 `image.png`）。
- 实战“邪修”方案：提升 `GoalCritic.cost_weight` 与 `GoalCritic.threshold_to_consider`，在不破坏避障的前提下更愿意收敛到目标。

## 典型顽疾：目标点附近圆周运动

在目标点附近，MPPI 可能出现反复绕圈或小半径圆周运动。

### 解决思路

新增一个轻量 `wrapper` 控制器，实际主控制器仍是 MPPI：

- 远离目标：透传 MPPI 输出速度。
- 接近目标（`near_goal_distance` 内）：
  - 强制固定线速度（`fixed_linear_speed`）
  - 仅用航向误差 `Kp` 输出角速度
- 极近目标（`stop_distance` 内）：直接置零速度。

该方案在比赛环境中可稳定抑制目标点附近绕圈。

## 当前仓库落地位置

- MPPI 参数：`nav2_params.yaml` 中 `controller_server.ros__parameters.FollowPath`
- Wrapper 参数：`nav2_params.yaml` 中 `mppi_goal_wrapper.ros__parameters`
- Wrapper 节点：`scripts/mppi_goal_wrapper.py`
- 启动接线：`launch/navigation_launch.py`
  - `controller_server -> cmd_vel_nav`
  - `mppi_goal_wrapper: cmd_vel_nav -> cmd_vel_nav_wrapped`
  - `velocity_smoother <- cmd_vel_nav_wrapped`

## Wrapper 可调参数建议

- `near_goal_distance`: 0.6 ~ 1.0
- `stop_distance`: 0.04 ~ 0.10
- `fixed_linear_speed`: 0.20 ~ 0.45
- `heading_kp`: 1.8 ~ 3.2
- `max_angular_speed`: 0.8 ~ 1.5

从保守值开始，逐项单参数扫描，不要同时改多项。
