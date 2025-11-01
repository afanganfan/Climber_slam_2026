#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <iostream>
#include <string>
#include <arpa/inet.h> // 字节序转换函数
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using std::placeholders::_1;

// 宏定义：帧头、帧尾、超时时间等（删除原DOWNLINK_DATA_LEN宏，后续直接计算）
#define FRAME_HEADER 0xff
#define FRAME_TAIL 0xdd
#define UART_TIMEOUT 100 // 串口超时时间(ms)
#define CMD_BUFF_LEN 10  // 发送指令缓冲区长度

// 精简后的下位机数据结构（删除decision_x/y，1字节对齐）
#pragma pack(1)
struct DownlinkData
{
    uint32_t time;               // 时间戳（4字节）
    uint8_t enemy_outpost_alive; // 敌方前哨站是否存活（0或1，1字节）
    int16_t base_hp_diff;       // 基地血量差（0~9999，2字节，大端序）
    uint8_t my_alive_num;        // 我方存活数量（<10，1字节）
    uint8_t enemy_alive_num;     // 敌方存活数量（<10，1字节）
    uint16_t hp;                 // 血量（2字节，大端序）
    uint16_t bullet_num;         // 发弹量（2字节，大端序）
    uint8_t shooter_on;          // 发射机构（0或1，1字节）
    uint8_t checksum;            // 校验和（data[0]~data[11]异或，1字节）
    uint8_t frame_tail;          // 帧尾：固定为0xdd (1字节)
};
#pragma pack() // 恢复默认对齐
// 重新定义接收数据包长度（精简后结构体大小：4+1+2+1+1+2+2+1+1+1=16字节）
#define DOWNLINK_DATA_LEN sizeof(DownlinkData)

class SerialNode : public rclcpp::Node
{
public:
    // 保留核心ROS组件（删除导航客户端、键盘订阅）
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

        // 初始化核心ROS组件
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 10, bind(&SerialNode::cmd_vel_callback, this, _1));
        judge_pub_ = this->create_publisher<std_msgs::msg::String>("/judge", 10);

        // 定时器：100ms读取一次串口（保留）
        serial_timer_ = this->create_wall_timer(
            100ms, bind(&SerialNode::read_serial_data, this));

        RCLCPP_INFO(this->get_logger(), "Serial node initialized (core functions only)");
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
    // 串口初始化（保留）
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

    // 速度指令回调：发送到下位机（保留）
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!is_serial_open_)
        {
            RCLCPP_WARN(this->get_logger(), "Serial port not open, cannot send cmd_vel");
            return;
        }

        // 构造发送缓冲区（帧头+数据+帧尾）
        uint8_t send_buff[CMD_BUFF_LEN] = {FRAME_HEADER, 0, 0, 0, 0, 0, 0, 0, 0, FRAME_TAIL};
        // 速度值放大10000倍转为int16_t（保留精度）
        int16_t linear_x = static_cast<int16_t>(msg->linear.x * 10000);
        int16_t linear_y = static_cast<int16_t>(msg->linear.y * 10000);
        int16_t angular_z = static_cast<int16_t>(msg->angular.z * 10000);
        // 转换为网络字节序（大端）后拆分字节
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

        // 发送数据
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

    // 读取串口数据并解析（保留，内部逻辑已精简）
    void read_serial_data()
    {
        if (!is_serial_open_)
        {
            RCLCPP_WARN(this->get_logger(), "Serial port not open, trying to reconnect...");
            serial_init(); // 尝试重连
            return;
        }

        try
        {
            size_t available = serial_port_.available();
            if (available >= DOWNLINK_DATA_LEN)
            {
                // 读取完整数据包
                vector<uint8_t> recv_data(DOWNLINK_DATA_LEN);
                size_t bytes_read = serial_port_.read(recv_data.data(), DOWNLINK_DATA_LEN);

                if (bytes_read == DOWNLINK_DATA_LEN)
                {
                    parse_downlink_data(recv_data.data()); // 解析数据（已精简）
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
            is_serial_open_ = false; // 标记为关闭，触发重连
        }
    }

    // 解析下位机数据
    void parse_downlink_data(const uint8_t *data)
    {
        const DownlinkData *pkg = reinterpret_cast<const DownlinkData *>(data);

        // 校验帧尾（保留）
        if (pkg->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid frame tail: 0x%02x", pkg->frame_tail);
            return;
        }

        // 校验和（因结构体精简，范围自动调整为DOWNLINK_DATA_LEN-2）
        uint8_t checksum = 0;
        for (int i = 0; i < DOWNLINK_DATA_LEN - 2; ++i)
        { // 不含checksum和frame_tail
            checksum ^= data[i];
        }
        if (checksum != pkg->checksum)
        {
            RCLCPP_WARN(this->get_logger(), "Checksum error: calc=0x%02x, recv=0x%02x", checksum, pkg->checksum);
            return;
        }

        // 解析原有字段
        uint32_t time = ntohl(pkg->time);
        uint8_t enemy_outpost_alive = pkg->enemy_outpost_alive;
        int16_t base_hp_diff = ntohs(pkg->base_hp_diff);
        uint8_t my_alive_num = pkg->my_alive_num;
        uint8_t enemy_alive_num = pkg->enemy_alive_num;
        uint16_t hp = ntohs(pkg->hp);
        uint16_t bullet_num = ntohs(pkg->bullet_num);
        uint8_t shooter_on = pkg->shooter_on;

        // 发布解析后的数据
        std_msgs::msg::String msg;
        msg.data = std::to_string(time) + "|" +
                   std::to_string(enemy_outpost_alive) + "|" +
                   std::to_string(base_hp_diff) + "|" +
                   std::to_string(my_alive_num) + "|" +
                   std::to_string(enemy_alive_num) + "|" +
                   std::to_string(hp) + "|" +
                   std::to_string(bullet_num) + "|" +
                   std::to_string(shooter_on);
        judge_pub_->publish(msg);

        // 打印完整解析结果
        RCLCPP_INFO(this->get_logger(),
                    "Parsed: time=%u, outpost=%u, diff=%d, my=%u, enemy=%u, hp=%u, bullet=%u, shooter=%u",
                    time, enemy_outpost_alive, base_hp_diff, my_alive_num, enemy_alive_num,
                    hp, bullet_num, shooter_on);
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
