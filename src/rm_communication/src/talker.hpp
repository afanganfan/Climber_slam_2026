#ifndef TALKER_HPP
#define TALKER_HPP

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rm_communication/zone_mode_switcher.hpp"

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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;

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
    bool has_nav_goal_active_{false};

    // 发送到底盘的最新速度缓存（无导航目标时按10Hz发送）
    int16_t cached_vx_{0};
    int16_t cached_vy_{0};
    int16_t cached_vz_{0};
    rclcpp::Time last_idle_cmd_send_time_;

    // 任务事件去重与节流状态
    bool has_last_mission_event_{false};
    std::string last_mission_event_;
    rclcpp::Time last_mission_event_time_;
    
    // 区域模式切换器和当前决策参数
    std::shared_ptr<rm_communication::ZoneModeSwitcher> zone_switcher_;
    rm_communication::ModeDecisionParams mode_params_;

    // --- 方法声明 ---

    // 回调函数
    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // 解析逻辑
    void parse_three();
    void parse_seven();

    // 辅助函数
    uint8_t calc_checksum(const std::vector<uint8_t> &packet);
    void send_cmd_packet(int16_t vx, int16_t vy, int16_t vz);
    void execute_mode_navigation(const rm_communication::DecisionResult &decision);
};

#endif 