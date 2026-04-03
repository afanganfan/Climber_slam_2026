#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MppiGoalWrapper(Node):
    def __init__(self) -> None:
        super().__init__('mppi_goal_wrapper')

        self.declare_parameter('enable_wrapper', True)
        self.declare_parameter('input_cmd_vel_topic', 'cmd_vel_nav')
        self.declare_parameter('output_cmd_vel_topic', 'cmd_vel_nav_wrapped')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('near_goal_distance', 0.8)
        self.declare_parameter('stop_distance', 0.06)
        self.declare_parameter('fixed_linear_speed', 0.35)
        self.declare_parameter('heading_kp', 2.6)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('transform_timeout_sec', 0.08)

        self.enable_wrapper = bool(self.get_parameter('enable_wrapper').value)
        self.input_cmd_vel_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        self.output_cmd_vel_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        self.plan_topic = str(self.get_parameter('plan_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.near_goal_distance = float(self.get_parameter('near_goal_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.fixed_linear_speed = float(self.get_parameter('fixed_linear_speed').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.transform_timeout = Duration(seconds=float(self.get_parameter('transform_timeout_sec').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_goal_xy: Optional[Tuple[float, float]] = None
        self.plan_frame: str = 'map'

        self.cmd_sub = self.create_subscription(Twist, self.input_cmd_vel_topic, self.on_cmd_vel, 20)
        self.plan_sub = self.create_subscription(Path, self.plan_topic, self.on_plan, 10)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_vel_topic, 20)

        self.get_logger().info(
            f'MPPI goal wrapper started: in={self.input_cmd_vel_topic}, out={self.output_cmd_vel_topic}, plan={self.plan_topic}'
        )

    def on_plan(self, msg: Path) -> None:
        # 始终取全局路径最后一个点作为当前导航目标点
        if not msg.poses:
            return
        last = msg.poses[-1].pose.position
        self.current_goal_xy = (last.x, last.y)
        self.plan_frame = msg.header.frame_id or self.plan_frame

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        # 将机器人位姿统一转换到路径坐标系中，便于直接计算目标距离和航向误差
        try:
            tf = self.tf_buffer.lookup_transform(
                self.plan_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=self.transform_timeout,
            )
        except TransformException:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return t.x, t.y, yaw

    def on_cmd_vel(self, cmd_in: Twist) -> None:
        # 1) wrapper关闭或尚未拿到目标点：直接透传MPPI输出
        if not self.enable_wrapper or self.current_goal_xy is None:
            self.cmd_pub.publish(cmd_in)
            return

        robot_pose = self.get_robot_pose()
        # TF不可用时不做接管，避免因位姿缺失导致异常控制
        if robot_pose is None:
            self.cmd_pub.publish(cmd_in)
            return

        rx, ry, ryaw = robot_pose
        gx, gy = self.current_goal_xy
        dx = gx - rx
        dy = gy - ry
        distance = math.hypot(dx, dy)

        # 2) 未进入接管半径：继续透传MPPI，保持原有避障与轨迹跟踪能力
        if distance > self.near_goal_distance:
            self.cmd_pub.publish(cmd_in)
            return

        cmd_out = Twist()

        # 3) 已非常接近目标：直接停车，防止在目标附近绕圈
        if distance <= self.stop_distance:
            self.cmd_pub.publish(cmd_out)
            return

        # 4) 进入接管区：固定线速度 + 仅Kp航向控制（角速度限幅）
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - ryaw)

        cmd_out.linear.x = min(self.fixed_linear_speed, distance)
        cmd_out.linear.y = 0.0
        cmd_out.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, self.heading_kp * yaw_error))
        self.cmd_pub.publish(cmd_out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MppiGoalWrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
