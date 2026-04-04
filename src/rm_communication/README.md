# rm_communication 区域模式切换（地图画区）

新增功能：在 `pgm` 地图上画区域，不同区域映射不同模式（`attack/defense/sensitive`），机器人进入区域后自动切换模式字节。

## 1. 画区工具

脚本：`scripts/zone_map_editor.py`

依赖：

```bash
pip install opencv-python
```

示例（在工作区根目录执行）：

```bash
python3 src/rm_communication/scripts/zone_map_editor.py \
  --pgm "assets/7v7 1.pgm" \
  --map-yaml "assets/7v7.yaml" \
  --out "src/rm_communication/config/zone_modes.csv"
```

说明：
- `assets/7v7 1.pgm` 用于可视化画区。
- `assets/7v7.yaml` 提供 `resolution/origin`，用于像素坐标到地图坐标换算。
- `assets/7v7.pcd` 是点云地图文件，通常用于定位/建图链路，本工具不直接读取该文件。

操作方式：
- `1` 切换到 `attack`（红色）
- `2` 切换到 `defense`（蓝色）
- `3` 切换到 `sensitive`（绿色）
- 鼠标左键拖拽：创建矩形区域
- 鼠标右键：撤销最后一个区域
- `s`：保存 CSV
- `q`：退出

## 2. 运行时接入

`talker` 节点增加参数：
- `zone_config_path`（默认：`src/rm_communication/config/zone_modes.csv`）

启动时可覆盖：

```bash
ros2 run rm_communication talker --ros-args -p zone_config_path:="src/rm_communication/config/zone_modes.csv"
```

## 3. 模式优先级

- 机器人位姿落在任意画区内：优先采用区域模式（`attack/defense/sensitive`），并停止发送自动导航目标。
- 不在任何画区内：沿用原有血量/时间驱动策略。
- 当落入多个重叠区域时，按 `priority` 字段从大到小选择。
- 若 `priority` 相同：优先选择面积更小的区域（更精细的局部策略）。
- 若面积也相同：后定义（CSV中更靠后）的区域覆盖前定义区域。

`zone_modes.csv` 支持 6 列格式（推荐）：

```csv
mode,x_min,x_max,y_min,y_max,priority
attack,1.0,2.0,-1.0,0.5,100
defense,1.2,1.8,-0.6,0.2,300
sensitive,1.4,1.6,-0.3,0.1,200
```

兼容旧格式（5列无 `priority`），代码会使用默认优先级：
- `defense=300`
- `sensitive=200`
- `attack=100`
