#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <arpa/inet.h> // for ntohs, ntohl (endianness conversion)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"

using namespace std;
using namespace std::chrono_literals;

#define FRAME_TAIL 0xDD
#define DATA_TYPE_SEVEN "seven"
#define DATA_TYPE_THREE "three"

// ---------------- 3V3 结构体定义 (24 bytes) ----------------
#pragma pack(1)
struct DownlinkDataThree
{
    uint32_t match_time;
    int16_t score_diff; // 有符号数，需要特殊处理
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

// ---------------- 7V7 结构体定义 (20 bytes, 示例长度) ----------------
#pragma pack(1)
struct DownlinkDataSeven
{
    uint32_t time;
    uint8_t enemy_outpost_alive;
    int16_t base_hp_diff; // 有符号数，需要特殊处理
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

class ReceiveNode : public rclcpp::Node
{
public:
    ReceiveNode() : Node("receive_node"), is_serial_open_(false)
    {
        // 1. 声明与获取参数
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("data_type", DATA_TYPE_THREE);

        port_name_ = this->get_parameter("port_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        data_type_ = this->get_parameter("data_type").as_string();

        // 2. 配置串口
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);

        // 编译修复: 创建 timeout 变量后传入
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_port_.setTimeout(timeout);

        pub_ = this->create_publisher<std_msgs::msg::String>("communication_data", 10);

        // 3. 定时器 10ms (100Hz)
        timer_ = this->create_wall_timer(
            10ms, std::bind(&ReceiveNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Port: %s, Mode: %s",
                    port_name_.c_str(), data_type_.c_str());
    }

    ~ReceiveNode()
    {
        if (is_serial_open_)
            serial_port_.close();
    }

private:
    uint8_t calc_checksum(const std::vector<uint8_t> &packet)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < packet.size() - 2; ++i)
        {
            sum ^= packet[i];
        }
        return sum;
    }

    void timer_callback()
    {
        if (!is_serial_open_)
        {
            try
            {
                serial_port_.open();
                is_serial_open_ = true;
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            }
            catch (const serial::IOException &e)
            {
                return;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Unhandled exception during open: %s", e.what());
                return;
            }
        }

        // 2. 读取与分发 (IO 异常保护)
        try
        {
            size_t available = serial_port_.available();
            if (available > 0)
            {
                std::vector<uint8_t> temp(available);
                serial_port_.read(temp.data(), available);
                buffer_.insert(buffer_.end(), temp.begin(), temp.end());
            }

            // 根据参数选择解析模式
            if (data_type_ == DATA_TYPE_SEVEN)
            {
                parse_seven();
            }
            else
            {
                parse_three();
            }
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_WARN(this->get_logger(), "Serial disconnected: %s", e.what());
            serial_port_.close();
            is_serial_open_ = false;
            buffer_.clear(); // 清空残留数据
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "System error: %s", e.what());
            serial_port_.close();
            is_serial_open_ = false;
        }
    }

    // ---------------- 3V3 解析逻辑 (输出全部 12 个字段) ----------------
    void parse_three()
    {
        if (buffer_.size() > 1024)
            buffer_.erase(buffer_.begin(), buffer_.end() - LEN_THREE * 2);

        while (buffer_.size() >= LEN_THREE)
        {
            auto it = std::find(buffer_.begin(), buffer_.end(), FRAME_TAIL);
            if (it == buffer_.end())
                return;

            long tail_idx = std::distance(buffer_.begin(), it);
            long head_idx = tail_idx - (LEN_THREE - 1);

            if (head_idx < 0)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                continue;
            }

            std::vector<uint8_t> packet(buffer_.begin() + head_idx, it + 1);
            auto *data = reinterpret_cast<DownlinkDataThree *>(packet.data());

            if (calc_checksum(packet) != data->checksum)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                RCLCPP_WARN(this->get_logger(), "3V3 Checksum mismatch! Drop packet.");
                continue;
            }

            // 校验成功：移除缓冲区中的这帧数据
            buffer_.erase(buffer_.begin(), it + 1);

            // --- 提取并转换全部 12 个数据字段 ---
            int16_t score_diff_val = (int16_t)ntohs(data->score_diff); // 修复符号数bug
            uint32_t match_time_val = ntohl(data->match_time);
            uint16_t our_hero_blood_val = ntohs(data->our_hero_blood);
            uint16_t our_infantry_blood_val = ntohs(data->our_infantry_blood);
            uint16_t our_sentry_blood_val = ntohs(data->our_sentry_blood);
            uint16_t enemy_hero_blood_val = ntohs(data->enemy_hero_blood);
            uint16_t enemy_infantry_blood_val = ntohs(data->enemy_infantry_blood);
            uint16_t enemy_sentry_blood_val = ntohs(data->enemy_sentry_blood);

            uint8_t our_hero_level_val = data->our_hero_level;
            uint8_t our_infantry_level_val = data->our_infantry_level;
            uint8_t enemy_hero_level_val = data->enemy_hero_level;
            uint8_t enemy_infantry_level_val = data->enemy_infantry_level;

            // 业务逻辑处理 (发布全部消息)
            auto msg = std::make_unique<std_msgs::msg::String>();

            // 拼接全部 12 个字段
            msg->data = "Three|" + to_string(match_time_val) + "|" +
                        to_string(score_diff_val) + "|" +
                        to_string(our_hero_blood_val) + "|" +
                        to_string(our_hero_level_val) + "|" +
                        to_string(our_infantry_blood_val) + "|" +
                        to_string(our_infantry_level_val) + "|" +
                        to_string(our_sentry_blood_val) + "|" +
                        to_string(enemy_hero_blood_val) + "|" +
                        to_string(enemy_hero_level_val) + "|" +
                        to_string(enemy_infantry_blood_val) + "|" +
                        to_string(enemy_infantry_level_val) + "|" +
                        to_string(enemy_sentry_blood_val);

            pub_->publish(std::move(msg));
        }
    }

    // ---------------- 7V7 解析逻辑 (输出全部 8 个字段) ----------------
    void parse_seven()
    {
        if (buffer_.size() > 1024)
            buffer_.erase(buffer_.begin(), buffer_.end() - LEN_SEVEN * 2);

        while (buffer_.size() >= LEN_SEVEN)
        {
            auto it = std::find(buffer_.begin(), buffer_.end(), FRAME_TAIL);
            if (it == buffer_.end())
                return;

            long tail_idx = std::distance(buffer_.begin(), it);
            long head_idx = tail_idx - (LEN_SEVEN - 1);

            if (head_idx < 0)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                continue;
            }

            std::vector<uint8_t> packet(buffer_.begin() + head_idx, it + 1);
            auto *data = reinterpret_cast<DownlinkDataSeven *>(packet.data());

            if (calc_checksum(packet) != data->checksum)
            {
                buffer_.erase(buffer_.begin(), it + 1);
                RCLCPP_WARN(this->get_logger(), "7V7 Checksum mismatch! Drop packet.");
                continue;
            }

            // 校验成功
            buffer_.erase(buffer_.begin(), it + 1);

            // --- 提取并转换全部 8 个数据字段 ---
            int16_t base_hp_diff_val = (int16_t)ntohs(data->base_hp_diff); // 修复符号数bug
            uint32_t time_val = ntohl(data->time);
            uint16_t hp_val = ntohs(data->hp);
            uint16_t bullet_num_val = ntohs(data->bullet_num);

            uint8_t enemy_outpost_alive_val = data->enemy_outpost_alive;
            uint8_t my_alive_num_val = data->my_alive_num;
            uint8_t enemy_alive_num_val = data->enemy_alive_num;
            uint8_t shooter_on_val = data->shooter_on;

            // 业务逻辑处理 (发布全部消息)
            auto msg = std::make_unique<std_msgs::msg::String>();

            // 拼接全部 8 个字段
            msg->data = "Seven|" + to_string(time_val) + "|" +
                        to_string(enemy_outpost_alive_val) + "|" +
                        to_string(base_hp_diff_val) + "|" +
                        to_string(my_alive_num_val) + "|" +
                        to_string(enemy_alive_num_val) + "|" +
                        to_string(hp_val) + "|" +
                        to_string(bullet_num_val) + "|" +
                        to_string(shooter_on_val);

            pub_->publish(std::move(msg));
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    serial::Serial serial_port_;
    std::string port_name_;
    int baud_rate_;
    std::string data_type_;
    bool is_serial_open_;

    std::deque<uint8_t> buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReceiveNode>());
    rclcpp::shutdown();
    return 0;
}