#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <string>
#include <arpa/inet.h>
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp" // 用于发布指令

using namespace std;
using namespace std::chrono_literals;

#define FRAME_HEADER 0xFF
#define FRAME_TAIL 0xDD
#define UART_TIMEOUT 100
#define DATA_TYPE_SEVEN "seven"
#define DATA_TYPE_THREE "three"

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

// 指令结构体（示例）
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
        declare_parameter("data_type", DATA_TYPE_SEVEN);
        get_parameter("data_type", data_type_);
        if (data_type_ != DATA_TYPE_SEVEN && data_type_ != DATA_TYPE_THREE)
        {
            RCLCPP_ERROR(get_logger(), "Invalid data_type! Use 'seven' or 'three'");
            data_type_ = DATA_TYPE_SEVEN;
        }
        RCLCPP_INFO(get_logger(), "Data type: %s", data_type_.c_str());

        is_serial_open_ = serial_port_.isOpen();
        if (!is_serial_open_)
            RCLCPP_ERROR(get_logger(), "Serial open failed");

        timer_ = create_wall_timer(100ms, bind(&ReceiveNode::timer_callback, this));
        pub_ = create_publisher<std_msgs::msg::String>("/judge", 10);
        cmd_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/control_command", 10);
    }

private:
    serial::Serial serial_port_;
    bool is_serial_open_;
    string data_type_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr cmd_pub_;

    uint8_t calc_checksum(const vector<uint8_t> &data)
    {
        uint8_t sum = 0;
        for (auto b : data)
            sum ^= b;
        return sum;
    }

    bool read_data(vector<uint8_t> &buf, size_t len)
    {
        if (!is_serial_open_ || serial_port_.available() < len)
            return false;
        return serial_port_.read(buf, len) == len;
    }

    // 决策函数：根据不同模式的结构体数据生成指令
    Command make_decision(const string &data_type,
                          const DownlinkDataSeven &seven_data,
                          const DownlinkDataThree &three_data)
    {
        Command cmd = {FRAME_HEADER, 0x00, 0x0000, 0x00, FRAME_TAIL};

        if (data_type == DATA_TYPE_SEVEN)
        {
            // 移除未使用的time变量，或根据实际需求使用它
            uint8_t enemy_alive = seven_data.enemy_alive_num;
            int16_t base_hp = ntohs(seven_data.base_hp_diff);

            if (enemy_alive > 3)
            {
                cmd.cmd_type = 0x01;
                cmd.param = htons(0x0001);
            }
            else if (base_hp < 0)
            {
                cmd.cmd_type = 0x02;
                cmd.param = htons(0x0002);
            }
        }
        else if (data_type == DATA_TYPE_THREE)
        {
            int16_t score_diff = ntohs(three_data.score_diff);
            uint16_t our_hero_blood = ntohs(three_data.our_hero_blood);

            if (score_diff < 0)
            {
                cmd.cmd_type = 0x03;
                cmd.param = htons(0x0003);
            }
            else if (our_hero_blood < 30)
            {
                cmd.cmd_type = 0x04;
                cmd.param = htons(0x0004);
            }
        }

        // 计算指令校验和
        vector<uint8_t> cmd_data;
        cmd_data.push_back(cmd.header);
        cmd_data.push_back(cmd.cmd_type);
        cmd_data.push_back((cmd.param >> 8) & 0xFF);
        cmd_data.push_back(cmd.param & 0xFF);
        cmd.checksum = calc_checksum(cmd_data);

        return cmd;
    }

    // 发送指令到下位机
    void send_command(const Command &cmd)
    {
        if (!is_serial_open_)
            return;

        vector<uint8_t> cmd_buf;
        cmd_buf.push_back(cmd.header);
        cmd_buf.push_back(cmd.cmd_type);
        cmd_buf.push_back((cmd.param >> 8) & 0xFF);
        cmd_buf.push_back(cmd.param & 0xFF);
        cmd_buf.push_back(cmd.checksum);
        cmd_buf.push_back(cmd.tail);

        serial_port_.write(cmd_buf);

        // 修正：使用std::make_unique而非类内的make_unique
        auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
        msg->data = cmd_buf;
        cmd_pub_->publish(std::move(msg));
        RCLCPP_INFO(get_logger(), "Sent command: type=0x%02X, param=0x%04X", cmd.cmd_type, ntohs(cmd.param));
    }

    void parse_seven()
    {
        vector<uint8_t> buf;
        if (!read_data(buf, LEN_SEVEN))
            return;
        auto *pkg = reinterpret_cast<DownlinkDataSeven *>(buf.data());

        if (pkg->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(get_logger(), "Seven tail err");
            return;
        }
        vector<uint8_t> check(buf.begin(), buf.end() - 2);
        if (calc_checksum(check) != pkg->checksum)
        {
            RCLCPP_WARN(get_logger(), "Seven checksum err");
            return;
        }

        // 生成决策指令
        Command cmd = make_decision(DATA_TYPE_SEVEN, *pkg, DownlinkDataThree());
        send_command(cmd);

        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Seven|" + to_string(ntohl(pkg->time)) + "|" +
                    to_string(pkg->enemy_outpost_alive) + "|" + to_string(ntohs(pkg->base_hp_diff)) + "|" +
                    to_string(pkg->my_alive_num) + "|" + to_string(pkg->enemy_alive_num) + "|" +
                    to_string(ntohs(pkg->hp)) + "|" + to_string(ntohs(pkg->bullet_num)) + "|" +
                    to_string(pkg->shooter_on);
        pub_->publish(std::move(msg));
    }

    void parse_three()
    {
        vector<uint8_t> buf;
        if (!read_data(buf, LEN_THREE))
            return;
        auto *pkg = reinterpret_cast<DownlinkDataThree *>(buf.data());

        if (pkg->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(get_logger(), "Three tail err");
            return;
        }
        vector<uint8_t> check(buf.begin(), buf.end() - 2);
        if (calc_checksum(check) != pkg->checksum)
        {
            RCLCPP_WARN(get_logger(), "Three checksum err");
            return;
        }

        // 生成决策指令
        Command cmd = make_decision(DATA_TYPE_THREE, DownlinkDataSeven(), *pkg);
        send_command(cmd);

        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Three|" + to_string(ntohl(pkg->match_time)) + "|" + to_string(ntohs(pkg->score_diff)) + "|" +
                    to_string(ntohs(pkg->our_hero_blood)) + "|" + to_string(pkg->our_hero_level) + "|" +
                    to_string(ntohs(pkg->our_infantry_blood)) + "|" + to_string(pkg->our_infantry_level) + "|" +
                    to_string(ntohs(pkg->our_sentry_blood)) + "|" + to_string(ntohs(pkg->enemy_hero_blood)) + "|" +
                    to_string(pkg->enemy_hero_level) + "|" + to_string(ntohs(pkg->enemy_infantry_blood)) + "|" +
                    to_string(pkg->enemy_infantry_level) + "|" + to_string(ntohs(pkg->enemy_sentry_blood));
        pub_->publish(std::move(msg));
    }

    void timer_callback()
    {
        if (!is_serial_open_)
        {
            serial_port_.open();
            is_serial_open_ = serial_port_.isOpen();
            RCLCPP_INFO(get_logger(), is_serial_open_ ? "Serial reconnected" : "Reconnect failed");
            return;
        }
        if (data_type_ == DATA_TYPE_SEVEN)
            parse_seven();
        else
            parse_three();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ReceiveNode>());
    rclcpp::shutdown();
    return 0;
}