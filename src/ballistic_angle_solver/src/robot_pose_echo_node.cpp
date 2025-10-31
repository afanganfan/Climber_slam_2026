#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class RobotPoseEchoNode : public rclcpp::Node {
public:
    RobotPoseEchoNode() : Node("robot_pose_echo_node") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            rclcpp::QoS(10),
            std::bind(&RobotPoseEchoNode::odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "robot_pose_echo_node started. Listening to /Odometry...");
    }
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Robot pose: (%.3f, %.3f, %.3f)",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPoseEchoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
