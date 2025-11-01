#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <iostream>
#include <string>
#include <arpa/inet.h> // 字节序转换函数
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using std::placeholders::_1;

// 宏定义：帧头、帧尾、超时时间等
#define FRAME_HEADER 0xff
#define FRAME_TAIL 0xdd
#define UART_TIMEOUT 100 // 串口超时时间(ms)
#define CMD_BUFF_LEN 10  // 发送指令缓冲区长度

// 新结构体：比赛核心数据（1字节对齐）
#pragma pack(1)
struct DownlinkData
{
    // 1. 全局比赛信息
    uint32_t match_time; // 比赛时间（单位：秒，0~4294秒）- 4字节
    int16_t score_diff;  // 比分差（我方得分-敌方得分，支持负数）- 2字节

    // 2. 我方机器人数据（Hero/Infantry/Sentry）
    uint16_t our_hero_blood;     // 我方Hero血量（0~10000）- 2字节
    uint8_t our_hero_level;      // 我方Hero等级（0~100）- 1字节
    uint16_t our_infantry_blood; // 我方Infantry血量（0~10000）- 2字节
    uint8_t our_infantry_level;  // 我方Infantry等级（0~100）- 1字节
    uint16_t our_sentry_blood;   // 我方Sentry血量（0~10000）- 2字节（无等级）

    // 3. 敌方机器人数据（Hero/Infantry/Sentry）
    uint16_t enemy_hero_blood;     // 敌方Hero血量（0~10000）- 2字节
    uint8_t enemy_hero_level;      // 敌方Hero等级（0~100）- 1字节
    uint16_t enemy_infantry_blood; // 敌方Infantry血量（0~10000）- 2字节
    uint8_t enemy_infantry_level;  // 敌方Infantry等级（0~100）- 1字节
    uint16_t enemy_sentry_blood;   // 敌方Sentry血量（0~10000）- 2字节（无等级）

    // 4. 校验与帧尾（固定位置）
    uint8_t checksum;   // 校验和（data[0]~data[22]异或，覆盖上述所有数据字段）- 1字节
    uint8_t frame_tail; // 帧尾（固定0xdd）- 1字节
};
#pragma pack() // 恢复默认对齐
// 新结构体总长度：4+2 + (2+1+2+1+2) + (2+1+2+1+2) +1+1 = 24字节
#define DOWNLINK_DATA_LEN sizeof(DownlinkData)

class SerialNode : public rclcpp::Node
{
public:
    // 保留核心ROS组件（速度订阅、裁判数据发布、串口定时器）
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr judge_pub_;
    rclcpp::TimerBase::SharedPtr serial_timer_;

    // 串口相关（保留）
    serial::Serial serial_port_;
    string serial_port_name_;
    int serial_baudrate_;
    bool is_serial_open_;

    SerialNode() : Node("serial_port"),
                   serial_port_name_("/dev/ttySLAM"),
                   serial_baudrate_(115200),
                   is_serial_open_(false)
    {
        // 初始化串口
        serial_init();

        // 初始化核心ROS组件（速度指令订阅、裁判数据发布）
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 10, bind(&SerialNode::cmd_vel_callback, this, _1));
        judge_pub_ = this->create_publisher<std_msgs::msg::String>("/judge", 10);

        // 定时器：100ms读取一次串口
        serial_timer_ = this->create_wall_timer(
            100ms, bind(&SerialNode::read_serial_data, this));

        RCLCPP_INFO(this->get_logger(), "Serial node initialized (match data version)");
    }

    ~SerialNode()
    {
        if (serial_port_.isOpen())
        {
            serial_port_.close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
    }

private:
    // 串口初始化（保留，无修改）
    void serial_init()
    {
        try
        {
            serial_port_.setPort(serial_port_name_);
            serial_port_.setBaudrate(serial_baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(UART_TIMEOUT);
            serial_port_.setTimeout(to);
            serial_port_.open();

            if (serial_port_.isOpen())
            {
                is_serial_open_ = true;
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud",
                            serial_port_name_.c_str(), serial_baudrate_);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
            is_serial_open_ = false;
        }
    }

    // 速度指令回调（保留，无修改，发送逻辑与接收结构体无关）
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!is_serial_open_)
        {
            RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send cmd_vel");
            return;
        }

        uint8_t send_buff[CMD_BUFF_LEN] = {FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, FRAME_TAIL};
        int16_t linear_x = static_cast<int16_t>(msg->linear.x * 10000);
        int16_t linear_y = static_cast<int16_t>(msg->linear.y * 10000);
        int16_t angular_z = static_cast<int16_t>(msg->angular.z * 10000);

        linear_x = htons(linear_x);
        linear_y = htons(linear_y);
        angular_z = htons(angular_z);

        send_buff[1] = (linear_x >> 8) & 0xFF;
        send_buff[2] = linear_x & 0xFF;
        send_buff[3] = (linear_y >> 8) & 0xFF;
        send_buff[4] = linear_y & 0xFF;
        send_buff[5] = (angular_z >> 8) & 0xFF;
        send_buff[6] = angular_z & 0xFF;
        send_buff[7] = 0; // 预留位（运动模式）

        try
        {
            serial_port_.write(send_buff, CMD_BUFF_LEN);
            RCLCPP_INFO(this->get_logger(), "Sent: linear(%.2f, %.2f), angular(%.2f)",
                        msg->linear.x, msg->linear.y, msg->angular.z);
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Send failed: %s", e.what());
        }
    }

    // 读取串口数据（保留，无修改，仅依赖DOWNLINK_DATA_LEN）
    void read_serial_data()
    {
        if (!is_serial_open_)
        {
            RCLCPP_WARN(this->get_logger(), "Serial port not open, trying to reconnect...");
            serial_init();
            return;
        }

        try
        {
            size_t available = serial_port_.available();
            if (available >= DOWNLINK_DATA_LEN)
            {
                vector<uint8_t> recv_data(DOWNLINK_DATA_LEN);
                size_t bytes_read = serial_port_.read(recv_data.data(), DOWNLINK_DATA_LEN);

                if (bytes_read == DOWNLINK_DATA_LEN)
                {
                    parse_downlink_data(recv_data.data()); // 核心修改：适配新结构体解析
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Incomplete frame: got %zu/%zu bytes",
                                bytes_read, DOWNLINK_DATA_LEN);
                }
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
            is_serial_open_ = false;
        }
    }

    // 核心修改：解析新的比赛数据结构体
    void parse_downlink_data(const uint8_t *data)
    {
        const DownlinkData *pkg = reinterpret_cast<const DownlinkData *>(data);

        // 1. 校验帧尾（固定逻辑）
        if (pkg->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid frame tail: 0x%02x", pkg->frame_tail);
            return;
        }

        // 2. 校验和（自动适配新结构体长度，无需手动改范围）
        uint8_t checksum = 0;
        for (int i = 0; i < DOWNLINK_DATA_LEN - 2; ++i) // 排除checksum和frame_tail
        {
            checksum ^= data[i];
        }
        if (checksum != pkg->checksum)
        {
            RCLCPP_WARN(this->get_logger(), "Checksum error: calc=0x%02x, recv=0x%02x", checksum, pkg->checksum);
            return;
        }

        // 3. 解析数据（多字节字段需转字节序，单字节无需转）
        // 3.1 全局比赛信息
        uint32_t match_time = ntohl(pkg->match_time); // 4字节转序
        int16_t score_diff = ntohs(pkg->score_diff);  // 2字节转序

        // 3.2 我方机器人数据
        uint16_t our_hero_blood = ntohs(pkg->our_hero_blood);
        uint8_t our_hero_level = pkg->our_hero_level;
        uint16_t our_infantry_blood = ntohs(pkg->our_infantry_blood);
        uint8_t our_infantry_level = pkg->our_infantry_level;
        uint16_t our_sentry_blood = ntohs(pkg->our_sentry_blood);

        // 3.3 敌方机器人数据
        uint16_t enemy_hero_blood = ntohs(pkg->enemy_hero_blood);
        uint8_t enemy_hero_level = pkg->enemy_hero_level;
        uint16_t enemy_infantry_blood = ntohs(pkg->enemy_infantry_blood);
        uint8_t enemy_infantry_level = pkg->enemy_infantry_level;
        uint16_t enemy_sentry_blood = ntohs(pkg->enemy_sentry_blood);

        // 4. 发布裁判数据（按“全局→我方→敌方”顺序，用|分隔，便于后续解析）
        std_msgs::msg::String msg;
        msg.data = to_string(match_time) + "|" +
                   to_string(score_diff) + "|" +
                   to_string(our_hero_blood) + "|" + to_string(our_hero_level) + "|" +
                   to_string(our_infantry_blood) + "|" + to_string(our_infantry_level) + "|" +
                   to_string(our_sentry_blood) + "|" +
                   to_string(enemy_hero_blood) + "|" + to_string(enemy_hero_level) + "|" +
                   to_string(enemy_infantry_blood) + "|" + to_string(enemy_infantry_level) + "|" +
                   to_string(enemy_sentry_blood);
        judge_pub_->publish(msg);

        // 5. 打印解析结果（格式清晰，便于调试）
        RCLCPP_INFO(this->get_logger(),
                    "Match Data Parsed: "
                    "Time=%us, ScoreDiff=%d | "
                    "Our[Hero(B=%u,L=%u), Infantry(B=%u,L=%u), Sentry(B=%u)] | "
                    "Enemy[Hero(B=%u,L=%u), Infantry(B=%u,L=%u), Sentry(B=%u)]",
                    match_time, score_diff,
                    our_hero_blood, our_hero_level, our_infantry_blood, our_infantry_level, our_sentry_blood,
                    enemy_hero_blood, enemy_hero_level, enemy_infantry_blood, enemy_infantry_level, enemy_sentry_blood);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}