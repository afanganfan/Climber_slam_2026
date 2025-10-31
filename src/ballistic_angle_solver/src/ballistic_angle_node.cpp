#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>

struct Point3D // 三维点结构体
{
    double x, y, z;
};

// 计算发射仰角（角度制），返回-1表示不可达
double calculate_launch_angle(
    const Point3D &start,
    const Point3D &target,
    double v0,
    const rclcpp::Logger &node_logger,
    double g = 9.81)
{
    double dx = target.x - start.x;
    double dy = target.y - start.y;
    double dz = target.z - start.z;
    double d = sqrt(dx * dx + dy * dy); // 水平距离
    if (d < 0.01)
    {
        RCLCPP_WARN(node_logger, "Target horizontal distance is too small!");
        return -1;
    }
    // 理想抛体解析解
    double v0_2 = v0 * v0;
    double g_d2 = g * d * d;
    double under_sqrt = v0_2 * v0_2 - g * (g_d2 + 2 * dz * v0_2);
    if (under_sqrt < 0)
    {
        RCLCPP_WARN(node_logger, "Target unreachable: no real solution for launch angle.");
        return -1;
    }
    // 两个解，取较小仰角（更平的弹道）
    double sqrt_val = sqrt(under_sqrt);
    double theta1 = atan((v0_2 + sqrt_val) / (g * d));
    double theta2 = atan((v0_2 - sqrt_val) / (g * d));
    double theta = std::min(theta1, theta2);
    double angle_deg = theta * 180.0 / M_PI;
    RCLCPP_DEBUG(node_logger, "Analytical launch angle: %.2f deg", angle_deg);
    return angle_deg;
}

class BallisticAngleNode : public rclcpp::Node
{
public:
    BallisticAngleNode() : Node("ballistic_angle_node"), last_angle_(-1.0),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(*tf_buffer_)
    {
        // 保留原参数配置（可根据实际需求调整v0、target等）
        this->declare_parameter<double>("v0", 25.0);
        this->declare_parameter<double>("g", 9.81);
        this->declare_parameter<double>("target_x", 20.45);
        this->declare_parameter<double>("target_y", -0.12);
        this->declare_parameter<double>("target_z", 1.343);

        // 里程计订阅（保留原QoS配置）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&BallisticAngleNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Ballistic angle node started (fixed wind & velocity)");
        RCLCPP_INFO(this->get_logger(), "Default params: v0=%.1f m/s, target=(%.2f,%.2f,%.2f)",
                    this->get_parameter("v0").as_double(),
                    this->get_parameter("target_x").as_double(),
                    this->get_parameter("target_y").as_double(),
                    this->get_parameter("target_z").as_double());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 保留原逻辑：读取机器人位置、目标参数
        Point3D robot;
        robot.x = msg->pose.pose.position.x;
        robot.y = msg->pose.pose.position.y;
        robot.z = msg->pose.pose.position.z;

        Point3D target;
        target.x = this->get_parameter("target_x").as_double();
        target.y = this->get_parameter("target_y").as_double();
        target.z = this->get_parameter("target_z").as_double();
        double v0 = this->get_parameter("v0").as_double();
        double g = this->get_parameter("g").as_double();

        // 计算发射角度
        double angle = calculate_launch_angle(
            robot, target, v0, this->get_logger(), g);

        // 输出日志（保留原逻辑）
        double horizontal_dist = sqrt(
            pow(target.x - robot.x, 2) + pow(target.y - robot.y, 2));
        RCLCPP_INFO(this->get_logger(), "=== Odom Callback ===");
        RCLCPP_INFO(this->get_logger(), "Robot pos: (%.2f, %.2f, %.2f)", robot.x, robot.y, robot.z);
        RCLCPP_INFO(this->get_logger(), "Target pos: (%.2f, %.2f, %.2f) (horiz dist: %.2f m)",
                    target.x, target.y, target.z, horizontal_dist);

        if (angle < 0)
        {
            RCLCPP_WARN(this->get_logger(), "❌ Target out of range (angle: %.2f)", angle);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "✅ Launch angle: %.2f deg", angle);
        }

        // 保存最近一次坐标和角度
        last_robot_ = robot;
        last_angle_ = angle;

        // 新增：计算 livox_frame y轴与目标方向夹角（全部用 map 坐标系）
        try {
            auto tf = tf_buffer_->lookupTransform("map", "livox_frame", tf2::TimePointZero);
            // livox_frame y轴在自身坐标系下为 (0,1,0)
            Eigen::Vector3d y_local(0, 1, 0);
            Eigen::Quaterniond q(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z);
            Eigen::Vector3d y_map = q * y_local;
            // livox_frame 在 map 下的位置
            Eigen::Vector3d livox_pos(
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z);
            // 目标方向（map坐标系）
            Eigen::Vector3d target_pos(target.x, target.y, target.z);
            Eigen::Vector3d target_dir = (target_pos - livox_pos).normalized();
            // 计算夹角
            double cos_angle = y_map.normalized().dot(target_dir);
            double angle_rad = acos(std::min(1.0, std::max(-1.0, cos_angle)));
            double angle_deg = angle_rad * 180.0 / M_PI;
            RCLCPP_INFO(this->get_logger(), "livox_frame y轴与目标方向夹角(map): %.2f deg", angle_deg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", e.what());
        }
    }

    Point3D last_robot_;
    double last_angle_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // 移除 key_sub_
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BallisticAngleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
