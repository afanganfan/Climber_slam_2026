#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <iostream>
#include <string>
#include <arpa/inet.h> // 字节序转换函数
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

// 宏定义：帧头、帧尾、超时时间等
#define FRAME_HEADER 0xff
#define FRAME_TAIL 0xdd
#define UART_TIMEOUT 100                       // 串口超时时间(ms)
#define CMD_BUFF_LEN 10                        // 发送指令缓冲区长度
#define DOWNLINK_DATA_LEN sizeof(DownlinkData) // 接收数据包长度（自动匹配结构体）

// 扩展后的下位机数据结构（含决策目标点，1字节对齐）
#pragma pack(1)
struct DownlinkData
{
    uint32_t time;               // 时间戳（4字节）
    uint8_t enemy_outpost_alive; // 敌方前哨站是否存活（0或1，1字节）
    uint16_t base_hp_diff;       // 基地血量差（0~9999，2字节，大端序）
    uint8_t my_alive_num;        // 我方存活数量（<10，1字节）
    uint8_t enemy_alive_num;     // 敌方存活数量（<10，1字节）
    uint16_t hp;                 // 血量（2字节，大端序）
    uint16_t bullet_num;         // 发弹量（2字节，大端序）
    uint8_t shooter_on;          // 发射机构（0或1，1字节）
    float decision_x;            // 新增：下位机决策目标点x（4字节，大端序）
    float decision_y;            // 新增：下位机决策目标点y（4字节，大端序）
    uint8_t checksum;            // 校验和（data[0]~data[13+8]异或，1字节）
    uint8_t frame_tail;          // 帧尾：固定为0xdd (1字节)
};
#pragma pack() // 恢复默认对齐

class SerialNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // ROS组件
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr judge_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub_; // 新增：键盘订阅

    // 串口相关
    serial::Serial serial_port_;
    string serial_port_name_;
    int serial_baudrate_;
    bool is_serial_open_;

    // 新增：下位机决策使能标志（true=启用，false=禁用）
    bool enable_downlink_decision_;

    SerialNode() : Node("serial_port"),
                   serial_port_name_("/dev/ttySLAM"),
                   serial_baudrate_(115200),
                   is_serial_open_(false),
                   enable_downlink_decision_(true) // 默认启用决策
    {
        // 初始化串口
        serial_init();

        // 初始化ROS组件
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 10, bind(&SerialNode::cmd_vel_callback, this, _1));
        judge_pub_ = this->create_publisher<std_msgs::msg::String>("/judge", 10);
        navigate_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        // 新增：订阅键盘输入（话题名：/keyboard_input，用于接收按键"K"）
        keyboard_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_input", 10,
            bind(&SerialNode::keyboard_callback, this, _1));
        // 定时器：100ms读取一次串口
        serial_timer_ = this->create_wall_timer(
            100ms, bind(&SerialNode::read_serial_data, this));

        RCLCPP_INFO(this->get_logger(), "Serial node initialized (downlink decision: %s)",
                    enable_downlink_decision_ ? "ON" : "OFF");
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
    // 串口初始化
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

    // 速度指令回调：发送到下位机
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

    // 读取串口数据并解析
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
                    parse_downlink_data(recv_data.data()); // 解析数据
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

    // 解析下位机数据（含决策目标点解析）
    void parse_downlink_data(const uint8_t *data)
    {
        const DownlinkData *pkg = reinterpret_cast<const DownlinkData *>(data);

        // 校验帧尾
        if (pkg->frame_tail != FRAME_TAIL)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid frame tail: 0x%02x", pkg->frame_tail);
            return;
        }

        // 校验和（包含新增的decision_x和decision_y字段）
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

        // 解析原有字段（字节序转换）
        uint32_t time = ntohl(pkg->time);
        uint8_t enemy_outpost_alive = pkg->enemy_outpost_alive;
        uint16_t base_hp_diff = ntohs(pkg->base_hp_diff);
        uint8_t my_alive_num = pkg->my_alive_num;
        uint8_t enemy_alive_num = pkg->enemy_alive_num;
        uint16_t hp = ntohs(pkg->hp);
        uint16_t bullet_num = ntohs(pkg->bullet_num);
        uint8_t shooter_on = pkg->shooter_on;

        // 新增：解析决策目标点（处理浮点型字节序）
        // 步骤：将float的4字节视为uint32_t，转换字节序后转回float
        uint32_t x_raw = ntohl(*reinterpret_cast<const uint32_t *>(&pkg->decision_x));
        float decision_x = *reinterpret_cast<float *>(&x_raw);
        uint32_t y_raw = ntohl(*reinterpret_cast<const uint32_t *>(&pkg->decision_y));
        float decision_y = *reinterpret_cast<float *>(&y_raw);

        // 新增：根据决策开关状态触发导航
        if (enable_downlink_decision_)
        {
            RCLCPP_INFO(this->get_logger(), "Downlink decision: navigating to (%.2f, %.2f)",
                        decision_x, decision_y);
            send_nav_goal(decision_x, decision_y); // 调用导航函数
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Downlink decision disabled: target (%.2f, %.2f) ignored",
                        decision_x, decision_y);
        }

        // 发布解析后的数据（含新增目标点）
        std_msgs::msg::String msg;
        msg.data = std::to_string(time) + "|" +
                   std::to_string(enemy_outpost_alive) + "|" +
                   std::to_string(base_hp_diff) + "|" +
                   std::to_string(my_alive_num) + "|" +
                   std::to_string(enemy_alive_num) + "|" +
                   std::to_string(hp) + "|" +
                   std::to_string(bullet_num) + "|" +
                   std::to_string(shooter_on) + "|" +
                   std::to_string(decision_x) + "|" +
                   std::to_string(decision_y);
        judge_pub_->publish(msg);

        // 打印完整解析结果
        RCLCPP_INFO(this->get_logger(),
                    "Parsed: time=%u, outpost=%u, diff=%u, my=%u, enemy=%u, hp=%u, bullet=%u, shooter=%u, target=(%.2f,%.2f)",
                    time, enemy_outpost_alive, base_hp_diff, my_alive_num, enemy_alive_num,
                    hp, bullet_num, shooter_on, decision_x, decision_y);
    }

    // 新增：键盘按键回调（处理"K"键切换决策开关）
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "K" || msg->data == "k")
        {                                                           // 不区分大小写
            enable_downlink_decision_ = !enable_downlink_decision_; // 切换状态
            RCLCPP_INFO(this->get_logger(), "Downlink decision %s",
                        enable_downlink_decision_ ? "ENABLED" : "DISABLED");
        }
    }

    // 导航目标发送函数
    void send_nav_goal(float x, float y)
    {
        if (!navigate_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Navigation server not available");
            return;
        }

        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.w = 1.0; // 无旋转

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback = bind(&SerialNode::goal_response, this, _1);
        options.feedback_callback = bind(&SerialNode::goal_feedback, this, _1, _2);
        options.result_callback = bind(&SerialNode::goal_result, this, _1);

        navigate_client_->async_send_goal(goal, options);
    }

    // 导航回调函数（目标响应）
    void goal_response(const GoalHandleNavigateToPose::SharedPtr &handle)
    {
        if (!handle)
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        else
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }

    // 导航回调函数（过程反馈）
    void goal_feedback(GoalHandleNavigateToPose::SharedPtr,
                       const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Nav feedback: remaining=%.2fm",
                    feedback->distance_remaining);
    }

    // 导航回调函数（最终结果）
    void goal_result(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Nav succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Nav aborted");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Nav failed");
        }
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