from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ballistic_angle_solver',
            executable='ballistic_angle_node',
            name='ballistic_angle_node',
            output='screen',
            parameters=[
                {'v0': 15.0},
                {'target_x': 7.0},
                {'target_y': 4.0},
                {'target_z': 0.5}
            ]
        ),
        Node(
            package='ballistic_angle_solver',
            executable='key_event_publisher',
            name='key_event_publisher',
            output='screen'
        )
    ])
