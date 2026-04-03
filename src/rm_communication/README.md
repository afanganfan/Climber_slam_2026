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
python src/rm_communication/scripts/zone_map_editor.py \
  --pgm "assets/7v7 1.pgm" \
  --map-yaml "assets/7v7.yaml" \
  --out "src/rm_communication/config/zone_modes.csv"
```

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
