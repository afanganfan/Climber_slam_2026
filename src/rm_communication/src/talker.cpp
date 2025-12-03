#include "talker.hpp"

// 使用命名空间别名，避免在整个文件中写 `nav2_msgs::action::NavigateToPose`
using Action = nav2_msgs::action::NavigateToPose;

// --- 构造函数实现 ---
ReceiveNode::ReceiveNode() : Node("receive_node"), is_serial_open_(false)
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
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_.setTimeout(timeout);

    // 3. 创建发布者、订阅者、Action 客户端和定时器
    pub_ = this->create_publisher<std_msgs::msg::String>("communication_data", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_nav", 10, std::bind(&ReceiveNode::cmd_vel_callback, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<Action>(this, "navigate_to_pose");
    timer_ = this->create_wall_timer(
        10ms, std::bind(&ReceiveNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node initialized. Port: %s, Mode: %s",
                port_name_.c_str(), data_type_.c_str());
}

// --- 析构函数实现 ---
ReceiveNode::~ReceiveNode()
{
    if (is_serial_open_)
        serial_port_.close();
}

// --- calc_checksum 实现 ---
uint8_t ReceiveNode::calc_checksum(const std::vector<uint8_t> &packet)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < packet.size() - 2; ++i)
    {
        sum ^= packet[i];
    }
    return sum;
}

// --- cmd_vel_callback 实现 (发送速度到下位机) ---
void ReceiveNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // ... (保持原有的实现)
    if (!is_serial_open_)
        return;
    const int CMD_LEN = 10;
    uint8_t send_buff[CMD_LEN] = {0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0xdd};
    int16_t vx = static_cast<int16_t>(msg->linear.x * 10000);
    int16_t vy = static_cast<int16_t>(msg->linear.y * 10000);
    int16_t vz = static_cast<int16_t>(msg->angular.z * 10000);
    vx = htons(vx);
    vy = htons(vy);
    vz = htons(vz);
    send_buff[1] = (vx >> 8) & 0xFF;
    send_buff[2] = vx & 0xFF;
    send_buff[3] = (vy >> 8) & 0xFF;
    send_buff[4] = vy & 0xFF;
    send_buff[5] = (vz >> 8) & 0xFF;
    send_buff[6] = vz & 0xFF;
    send_buff[7] = 0; // mode/reserve
    try
    {
        serial_port_.write(send_buff, CMD_LEN);
    }
    catch (const serial::IOException &e)
    {
        RCLCPP_WARN(this->get_logger(), "Serial write failed: %s", e.what());
    }
}

// --- send_navigation_goal 实现 (发送导航目标) ---
void ReceiveNode::send_navigation_goal(double x, double y)
{
    // ... (保持原有的实现，注意 Action 类型别名)
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Navigate action server not available");
        return;
    }
    auto goal_msg = Action::Goal();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    goal_msg.pose = pose;

    auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
    // ... (省略 Action 回调的详细实现，保持原样)
    send_goal_options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    };
    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
               const std::shared_ptr<const Action::Feedback> feedback)
    {
        (void)feedback;
        RCLCPP_DEBUG(this->get_logger(), "Navigation feedback");
    };
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<Action>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Navigation canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Navigation unknown result");
            break;
        }
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

// --- timer_callback 实现 (串口 IO 循环) ---
void ReceiveNode::timer_callback()
{
    // ... (保持原有的实现)
    // 1. 自动重连机制 (USB 拔插保护)
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

// --- parse_three 实现 ---
void ReceiveNode::parse_three()
{
    // ... (保持原有的实现)
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

        // 导航决策（3V3）
        // 条件1：我方哨兵血量<100 -> 回家
        if (our_sentry_blood_val < 100)
        {
            send_navigation_goal(-0.8, 2.0); // 回家坐标
        }
        // 条件2：100 < 血量 < 200 且血量下降 -> 去回防点
        else if (our_sentry_blood_val > 100 && our_sentry_blood_val < 200 && our_sentry_blood_val < old_three_our_sentry_)
        {
            send_navigation_goal(1.0, 2.0); // 回防点
        }
        // 条件3：比赛时间 < 60 且得分差在 -60..60 -> 去增益点
        else if (match_time_val < 60 && (score_diff_val > -60 && score_diff_val < 60))
        {
            send_navigation_goal(-1.8, -1.0); // 增益点
        }

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
        // 更新历史值
        old_three_our_sentry_ = our_sentry_blood_val;
    }
}

// --- parse_seven 实现 ---
void ReceiveNode::parse_seven()
{
    // ... (保持原有的实现)
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

        // 导航决策（7V7）
        // 条件1：hp少于100 -> 回家
        if (hp_val < 100)
        {
            send_navigation_goal(-0.8, 2.0);
        }
        // 条件2：100 < hp < 200 且 hp 下降 -> 去回防点
        else if (hp_val > 100 && hp_val < 200 && hp_val < old_seven_hp_)
        {
            send_navigation_goal(1.0, 2.0);
        }

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
        // 更新历史值
        old_seven_hp_ = hp_val;
    }
}

// --- main 函数 (保持不变) ---
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 引用 ReceiveNode 类，而不是内联定义
    rclcpp::spin(std::make_shared<ReceiveNode>());
    rclcpp::shutdown();
    return 0;
}