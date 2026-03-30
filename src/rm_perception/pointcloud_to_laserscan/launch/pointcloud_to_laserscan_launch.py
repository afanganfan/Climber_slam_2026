from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'  # 启动参数：用于给示例话题添加命名空间
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # 话题重映射：将输入点云和输出激光话题映射到项目实际使用的话题名
            remappings=[('cloud_in',  ['/segmentation/obstacle']),
                        ('scan',  ['/scan'])],
            parameters=[{
                'target_frame': 'livox_frame',      # 目标坐标系：输出 LaserScan 所在的 TF 坐标系
                'transform_tolerance': 0.01,        # TF 变换容忍时间（秒），用于等待/插值坐标变换
                'min_height': -1.0,                 # 点云高度下限（米），低于该值的点会被过滤
                'max_height': 0.25,                 # 点云高度上限（米），高于该值的点会被过滤
                'angle_min': -3.14159,              # 扫描起始角（弧度）
                'angle_max': 3.14159,               # 扫描结束角（弧度）
                'angle_increment': 0.0043,          # 相邻激光束角分辨率（弧度）
                'scan_time': 0.3333,                # 一帧扫描周期（秒），对应输出 LaserScan 的时间基准
                'range_min': 0.45,                  # 最小测距（米），小于该值视为无效
                'range_max': 6.0,                   # 最大测距（米），大于该值视为无效
                'use_inf': True,                    # 是否使用 inf 表示超量程/无回波
                'inf_epsilon': 1.0                  # 当不使用 inf 时，超量程值会写成 range_max + inf_epsilon
            }],
            name='pointcloud_to_laserscan'          # 节点名
        )
    ])
