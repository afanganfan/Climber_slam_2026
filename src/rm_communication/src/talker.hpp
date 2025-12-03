#ifndef TALKER_HPP
#define TALKER_HPP

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;
using namespace std::chrono_literals;

// --- 常量/宏定义 ---
#define FRAME_TAIL 0xDD
#define DATA_TYPE_SEVEN "seven"
#define DATA_TYPE_THREE "three"

// --- 3V3 结构体定义 (24 bytes) ---
#pragma pack(1)
struct DownlinkDataThree
{
    uint32_t match_time;
    int16_t score_diff;
    uint16_t our_hero_blood;
    uint8_t our_hero_level;
    uint16_t our_infantry_blood;
    uint8_t our_infantry_level;
    uint16_t our_sentry_blood;
    uint16_t enemy_hero_blood;
    uint8_t enemy_hero_level;
    uint16_t enemy_infantry_blood;
    uint8_t enemy_infantry_level;
    uint16_t enemy_sentry_blood;
    uint8_t checksum;
    uint8_t frame_tail;
};
#pragma pack()
#define LEN_THREE sizeof(DownlinkDataThree)

// --- 7V7 结构体定义 (20 bytes) ---
#pragma pack(1)
struct DownlinkDataSeven
{
    uint32_t time;
    uint8_t enemy_outpost_alive;
    int16_t base_hp_diff;
    uint8_t my_alive_num;
    uint8_t enemy_alive_num;
    uint16_t hp;
    uint16_t bullet_num;
    uint8_t shooter_on;
    uint8_t checksum;
    uint8_t frame_tail;
};
#pragma pack()
#define LEN_SEVEN sizeof(DownlinkDataSeven)

// --- 类声明 ---
class ReceiveNode : public rclcpp::Node
{
public:
    ReceiveNode();
    ~ReceiveNode();

private:
    // ROS2 成员
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // 串口通信成员
    serial::Serial serial_port_;
    std::string port_name_;
    int baud_rate_;
    std::string data_type_;
    bool is_serial_open_;
    std::deque<uint8_t> buffer_;

    // 决策辅助成员
    uint16_t old_seven_hp_{0};
    uint16_t old_three_our_sentry_{0};

    // --- 方法声明 ---

    // 回调函数
    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // 解析逻辑
    void parse_three();
    void parse_seven();

    // 辅助函数
    uint8_t calc_checksum(const std::vector<uint8_t> &packet);
    void send_navigation_goal(double x, double y);

    // action goal 回调 (内部使用，可简化为 lambda)
    // 尽管您在 cpp 文件中使用 lambda，但在 hpp 中列出关键方法有助于概览
    // ... (action 回调的声明可以省略或保留)
};

#endif 