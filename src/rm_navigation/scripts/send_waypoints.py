#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus  # 引入标准状态定义

def yaw_to_quaternion(yaw: float):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class WaypointSender(Node):
    def __init__(self, waypoints, frame_id='map'):
        super().__init__('waypoint_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = waypoints
        self.frame_id = frame_id

    def send_all(self):
        # 等待服务器连接
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available')
            return 1

        for i, (x, y, yaw) in enumerate(self.waypoints, start=1):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            self.get_logger().info(f'Sending waypoint {i}: x={x}, y={y}')
            
            # 发送目标
            send_goal_future = self._client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            
            if not send_goal_future.done():
                self.get_logger().error(f'Failed to send goal {i} (timeout)')
                continue
                
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Goal {i} was rejected')
                continue
            
            # 等待导航结果
            self.get_logger().info(f'Goal {i} accepted, navigating...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            
            if not result_future.done():
                self.get_logger().warning(f'Waypoint {i} timed out, cancelling...')
                goal_handle.cancel_goal_async()
                continue
                
            # 获取结果并检查状态
            result_response = result_future.result()
            status = result_response.status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Waypoint {i} reached successfully!')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn(f'Waypoint {i} was canceled.')
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error(f'Waypoint {i} aborted by server.')
            else:
                self.get_logger().warn(f'Waypoint {i} finished with status code: {status}')

        return 0

def main(argv=None):
    rclpy.init(args=argv)
    waypoints = [
        (6.87, -0.0265, -0.00143),
        (6.9, 1.28, -0.00143),
        (-0.74, 0.584, 0.00647),
        (-0.983, 2.3, -0.00137),
        (6.78, 1.5, -0.00137),
        (6.94, 3.4, -0.00137),
        (-0.94, 2.55, -0.00143),
        (-0.625, 4.0, -0.00137),
        (5.83, 4.3, -0.00143),
    ]

    node = WaypointSender(waypoints)
    try:
        node.send_all()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()