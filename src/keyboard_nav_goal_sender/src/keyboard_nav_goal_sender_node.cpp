#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#define _LINUX_TERMIOS_H 1
#include <linux/termios.h>

class KeyboardNavGoalSender : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    KeyboardNavGoalSender() : Node("keyboard_nav_goal_sender_node") {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        RCLCPP_INFO(this->get_logger(), "Press 'b' to send goal (-3, -4.75, 0)");
        set_terminal_mode();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardNavGoalSender::check_key, this));
    }
    ~KeyboardNavGoalSender() {
        reset_terminal_mode();
    }
private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios orig_termios_;

    void set_terminal_mode() {
        tcgetattr(0, &orig_termios_);
        struct termios new_termios = orig_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &new_termios);
    }
    void reset_terminal_mode() {
        tcsetattr(0, TCSANOW, &orig_termios_);
    }
    void check_key() {
        int bytes = 0;
        ioctl(0, FIONREAD, &bytes);
        if (bytes > 0) {
            char c = getchar();
            if (c == 'b' || c == 'B') {
                send_goal();
            }
        }
    }
    void send_goal() {
        if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not available");
            return;
        }
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.pose.position.x = -3.0;
        goal_msg.pose.pose.position.y = -4.75;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        RCLCPP_INFO(this->get_logger(), "Sending goal: (-3, -4.75, 0)");
        client_->async_send_goal(goal_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNavGoalSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
