# nav2_params.yaml 参数修改记录（待验证）

## 解决问题

### 1. 避障功能：

#### 修改参数：

local_costmap(本地代价地图)

决定机器人实时运动过程中扫描的地图。通过增大扫描体积可以较快检测到障碍物。

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 20.0                     # 代价地图更新频率 (Hz)
      publish_frequency: 10.0                    # 代价地图发布频率 (Hz)
      global_frame: map                          # 全局参考坐标系
      robot_base_frame: base_link                # 机器人基座坐标系
      use_sim_time: False                        # 是否使用仿真时间
      rolling_window: true                       # 是否使用滚动窗口
      width: 5                                   # 代价地图宽度 (m)
      height: 5                                  # 代价地图高度 (m)
      resolution: 0.05                           # 代价地图分辨率 (m/cell)
      robot_radius: 0.2                          # 机器人半径 (m)
      plugins: ["voxel2d_layer", "voxel3d_layer", "inflation_layer"]  # 图层插件列表
```

inflation_layer（膨胀层）

决定障碍物的膨胀半径

cost_scaling_factor 越大表示越不允许接近膨胀层

```yaml
inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 8.0                 # 成本缩放因子
        inflation_radius: 0.7
```

global_costmap（全局代价地图）

用于躲避已知障碍

FollowPath（DWB局部规划器）

    BaseObstacle.scale机器人对障碍物的敏感程度

    sim_time预测未来多长时间的路径

    acc_lim_x / decel_lim_x决定机器人刹车

Recoveries Server（补救规划器）

    用的不多

#### 总结

- **怕撞 / 离得太近：** 调大 `inflation_layer` 的 `inflation_radius` 或 `BaseObstacle.scale`。

* **不钻窄缝：** 调大 `inflation_radius` 或 `robot_radius`。
* **发现障碍物反应太慢：** 检查 `local_costmap` 的 `update_frequency` (当前 20Hz) 是否足够。
* **死机/卡住不动：** 检查 `recoveries_server` 的配置。

### 2.走起来歪歪扭扭的：

#### 修改参数：

vx_samples、vy_samples、vtheta_samples增加采样频率

sim_time 增长预测时间

velocity_smoother（速度平滑器）大概率是这个问题

```yaml
max_accel: [5.0, 5.0, 15.0]
max_decel: [-5.0, -5.0, -15.0]  
```

调整加速和减速的加速度，减少鬼畜

```yaml
smoothing_frequency: 5.0
```

平滑频率

```yaml
 GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5                             # 规划容差 (m)
      use_astar: false                           # 是否使用A*算法 (false=Dijkstra)
      allow_unknown: true                        # 是否允许规划经过未知区域

```

use_astar: false 使用A*算法

plugin:"nav2_navfn_planner/NavfnPlanner"更换插件为nav2_smac_planner/SmacPlanner2D或SmacPlannerLattice

控制频率

```yaml
controller_frequency: 5.0
```

可以更快的修正轨道，防止需要大幅度修正轨道

### 3.没有膨胀层

```yaml
inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True                            # 是否启用膨胀层
        cost_scaling_factor: 8.0                 # 成本缩放因子
        inflation_radius: 0.7                    # 膨胀半径 (m)
  
      	always_send_full_costmap: True             # 总是发送完整代价地图
```

大概率是enabled没有开
