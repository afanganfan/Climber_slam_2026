#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <string>
#include <arpa/inet.h>
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std;
using namespace std::chrono_literals;

#define FRAME_HEADER 0xFF
#define FRAME_TAIL 0xDD
#define UART_TIMEOUT 100
#define DATA_TYPE_SEVEN "seven"
#define DATA_TYPE_THREE "three"

// 7V7数据结构体
#pragma pack(1)
struct DownlinkDataSeven
{
    uint32_t time;               // 比赛时间
    uint8_t enemy_outpost_alive; // 敌方前哨战状态（0 or 1）
    int16_t base_hp_diff;        // 基地血量差（我-敌）
    uint8_t my_alive_num;        // 我方存活数量
    uint8_t enemy_alive_num;     // 敌方存活数量
    uint16_t hp;                 // 自身血量（核心对比数据）
    uint16_t bullet_num;         // 发弹量
    uint8_t shooter_on;          // 发射机构状态（0 or 1）
    uint8_t checksum;
    uint8_t frame_tail;
};
#pragma pack()
#define LEN_SEVEN sizeof(DownlinkDataSeven)

// 3V3数据结构体（仅保留我方哨兵血量相关字段）
#pragma pack(1)
struct DownlinkDataThree
{
    uint32_t match_time;           // 比赛时间
    int16_t score_diff;            // 分数差（我方-敌方）
    uint16_t our_hero_blood;       // 我方英雄血量
    uint8_t our_hero_level;        // 我方英雄等级
    uint16_t our_infantry_blood;   // 我方步兵血量
    uint8_t our_infantry_level;    // 我方步兵等级
    uint16_t our_sentry_blood;     // 我方哨兵血量（核心对比数据）
    uint16_t enemy_hero_blood;     // 敌方英雄血量
    uint8_t enemy_hero_level;      // 敌方英雄等级
    uint16_t enemy_infantry_blood; // 敌方步兵血量
    uint8_t enemy_infantry_level;  // 敌方步兵等级
    uint16_t enemy_sentry_blood;   // （已弃用，仅为结构体兼容保留）
    uint8_t checksum;
    uint8_t frame_tail; // 帧尾
};
#pragma pack()
#define LEN_THREE sizeof(DownlinkDataThree)

// 指令结构体
#pragma pack(1)
struct Command
{
    uint8_t header;
    uint8_t cmd_type; // 指令类型
    uint16_t param;   // 指令参数
    uint8_t checksum;
    uint8_t tail;
};
#pragma pack()

class ReceiveNode : public rclcpp::Node
{
public:
    ReceiveNode() : Node("receive_node"),
                    serial_port_("/dev/ttySLAM", 115200, serial::Timeout::simpleTimeout(UART_TIMEOUT)),
                    is_serial_open_(false), data_type_(DATA_TYPE_SEVEN)
    {
        // 初始化旧数据（仅保留必要字段）
        old_seven_hp_ = 0;
        old_three_our_sentry_ = 0;

        // 读取模式参数（7V7/3V3）
        declare_parameter("data_type", DATA_TYPE_SEVEN);
        get_parameter("data_type", data_type_);
        if (data_type_ != DATA_TYPE_SEVEN && data_type_ != DATA_TYPE_THREE)
        {
            RCLCPP_ERROR(get_logger(), "Invalid data_type! Use 'seven' or 'three'");
            data_type_ = DATA_TYPE_SEVEN;
        }
        RCLCPP_INFO(get_logger(), "Data type: %s", data_type_.c_str());

        // 初始化串口
        is_serial_open_ = serial_port_.isOpen();
        if (!is_serial_open_)
            RCLCPP_ERROR(get_logger(), "Serial open failed");

        // 初始化定时器与发布者
        timer_ = create_wall_timer(100ms, bind(&ReceiveNode::timer_callback, this));
        pub_ = create_publisher<std_msgs::msg::String>("/judge", 10);
        cmd_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/control_command", 10);
    }

private:
    // 串口与ROS基础成员
    serial::Serial serial_port_;
    bool is_serial_open_;
    string data_type_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;

    // 存储旧数据
    uint16_t old_seven_hp_;         // 7V7：旧自身血量
    uint16_t old_three_our_sentry_; // 3V3：旧我方哨兵血量

    // 异或校验和计算
    uint8_t calc_checksum(const vector<uint8_t> &data)
    {
        uint8_t sum = 0;
        for (auto b : data)
            sum ^= b;
        return sum;
    }

    // 读取指定长度串口数据
    bool read_data(vector<uint8_t> &buf, size_t len)
    {
        if (!is_serial_open_ || serial_port_.available() < len)
            return false;
        return serial_port_.read(buf, len) == len;
    }

    Command make_decision(const string &data_type,
                          const uint16_t new_hp,
                          const uint16_t new_our_sentry,
                          const DownlinkDataThree *three_data = nullptr) 
    {
        Command cmd = {FRAME_HEADER, 0x00, 0x0000, 0x00, FRAME_TAIL};

        // 7V7模式
        if (data_type == DATA_TYPE_SEVEN)
        {
            // 条件1：hp少于100（紧急低血量）
            if (new_hp < 100)
            {
                cmd.cmd_type = 0x01;
                cmd.param = htons(0x0001); // 回家
            }
            // 条件2：100 < hp < 200，且hp下降（缓慢掉血）
            else if (new_hp > 100 && new_hp < 200 && new_hp < old_seven_hp_)
            {
                cmd.cmd_type = 0x02;
                cmd.param = htons(0x0002); // 去回防点
            }
        }
        // 3V3模式
        else if (data_type == DATA_TYPE_THREE && three_data != nullptr) // 增加three_data非空判断
        {

            uint32_t time = ntohl(three_data->match_time);
            int16_t score_diff = ntohs(three_data->score_diff);

            // 条件1：我方哨兵血量<100（紧急低血量）
            if (new_our_sentry < 100)
            {
                cmd.cmd_type = 0x01;
                cmd.param = htons(0x0001); // 回家
            }
            // 条件2：我方哨兵100<血量<200，且正在下降（缓慢掉血）
            else if (new_our_sentry > 100 && new_our_sentry < 200 && new_our_sentry < old_three_our_sentry_)
            {
                cmd.cmd_type = 0x02;
                cmd.param = htons(0x0002); // 去回防点
            }

            else if (time < 60 && ((score_diff > 0 && score_diff < 60) || (score_diff < 0 && score_diff > -60)))
            {
                cmd.cmd_type = 0x03;
                cmd.param = htons(0x0003); // 去增益点
            }
        }

        vector<uint8_t> cmd_data = {
            cmd.header, cmd.cmd_type,
            static_cast<uint8_t>((cmd.param >> 8) & 0xFF),
            static_cast<uint8_t>(cmd.param & 0xFF)};
        cmd.checksum = calc_checksum(cmd_data);

        return cmd;
    }

    // 发送指令（串口+ROS发布）
    void send_command(const Command &cmd)
    {
        if (!is_serial_open_)
            return;

        // 封装指令字节流
        vector<uint8_t> cmd_buf = {
            cmd.header, cmd.cmd_type,
            static_cast<uint8_t>((cmd.param >> 8) & 0xFF),
            static_cast<uint8_t>(cmd.param & 0xFF),
            cmd.checksum, cmd.tail};

        // 串口下发
        serial_port_.write(cmd_buf);

        auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
        msg->data = cmd_buf;
        cmd_pub_->publish(std::move(msg));

        RCLCPP_INFO(get_logger(), "Sent Command: Type=0x%02X, Param=0x%04X",
                    cmd.cmd_type, ntohs(cmd.param));
    }

    void parse_seven()
    {
        vector<uint8_t> buf;
        if (!read_data(buf, LEN_SEVEN))
            return;

        auto *data = reinterpret_cast<DownlinkDataSeven *>(buf.data());

        // 帧尾校验
        if (data->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(get_logger(), "Seven Frame Tail Error");
            return;
        }

        // 校验和校验
        vector<uint8_t> check_data(buf.begin(), buf.end() - 2);
        if (calc_checksum(check_data) != data->checksum)
        {
            RCLCPP_WARN(get_logger(), "Seven Checksum Error");
            return;
        }

        uint16_t new_hp = ntohs(data->hp);

        Command cmd = make_decision(DATA_TYPE_SEVEN, new_hp, 0, nullptr);
        send_command(cmd);

        old_seven_hp_ = new_hp;

        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Seven|" + to_string(ntohl(data->time)) + "|" +
                    to_string(data->enemy_outpost_alive) + "|" + to_string(ntohs(data->base_hp_diff)) + "|" +
                    to_string(data->my_alive_num) + "|" + to_string(data->enemy_alive_num) + "|" +
                    to_string(new_hp) + "|" + to_string(ntohs(data->bullet_num)) + "|" +
                    to_string(data->shooter_on);
        pub_->publish(std::move(msg));
    }

    void parse_three()
    {
        vector<uint8_t> buf;
        if (!read_data(buf, LEN_THREE))
            return;

        auto *data = reinterpret_cast<DownlinkDataThree *>(buf.data());

        // 帧尾校验
        if (data->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(get_logger(), "Three Frame Tail Error");
            return;
        }

        // 校验和校验
        vector<uint8_t> check_data(buf.begin(), buf.end() - 2);
        if (calc_checksum(check_data) != data->checksum)
        {
            RCLCPP_WARN(get_logger(), "Three Checksum Error");
            return;
        }

        // 提取我方哨兵新血量（网络字节序转主机字节序）
        uint16_t new_our_sentry = ntohs(data->our_sentry_blood);

        Command cmd = make_decision(DATA_TYPE_THREE, 0, new_our_sentry, data);
        send_command(cmd);

        // 更新我方哨兵旧血量
        old_three_our_sentry_ = new_our_sentry;

        // 发布解析数据（已移除敌方哨兵血量相关内容）
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Three|" + to_string(ntohl(data->match_time)) + "|" + to_string(ntohs(data->score_diff)) + "|" +
                    to_string(ntohs(data->our_hero_blood)) + "|" + to_string(data->our_hero_level) + "|" +
                    to_string(ntohs(data->our_infantry_blood)) + "|" + to_string(data->our_infantry_level) + "|" +
                    to_string(new_our_sentry) + "|" + to_string(ntohs(data->enemy_hero_blood)) + "|" +
                    to_string(data->enemy_hero_level) + "|" + to_string(ntohs(data->enemy_infantry_blood)) + "|" +
                    to_string(data->enemy_infantry_level);
        pub_->publish(std::move(msg));
    }

    // 定时器回调
    void timer_callback()
    {
        // 串口重连逻辑
        if (!is_serial_open_)
        {
            serial_port_.open();
            is_serial_open_ = serial_port_.isOpen();
            RCLCPP_INFO(get_logger(), is_serial_open_ ? "Serial Reconnected" : "Serial Reconnect Failed");
            return;
        }

        // 按模式解析数据
        if (data_type_ == DATA_TYPE_SEVEN)
            parse_seven();
        else
            parse_three();
    }
};

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ReceiveNode>());
    rclcpp::shutdown();
    return 0;
}