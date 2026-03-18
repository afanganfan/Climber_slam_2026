#include "talker.hpp"

using Action = nav2_msgs::action::NavigateToPose;

ReceiveNode::ReceiveNode() : Node("talker"), is_serial_open_(false)
{
    this->declare_parameter("port_name", "/dev/ttySLAM");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("data_type", DATA_TYPE_THREE);

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    data_type_ = this->get_parameter("data_type").as_string();

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

    // 初始化区域模式切换器
    zone_switcher_ = std::make_shared<rm_communication::ZoneModeSwitcher>(this);
    mode_params_ = rm_communication::ModeDecisionParams();
    last_idle_cmd_send_time_ = this->now();
    last_nav_goal_send_time_ = this->now();

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

void ReceiveNode::send_cmd_packet(int16_t vx, int16_t vy, int16_t vz)
{
    // 11 字节：帧头 + 3轴速度(6B) + 模式 + 预留2B + 帧尾
    const int CMD_LEN = 10;
    uint8_t send_buff[CMD_LEN] = {0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0xdd};

    send_buff[1] = (uint8_t)((vx >> 8) & 0xFF);
    send_buff[2] = (uint8_t)(vx & 0xFF);
    send_buff[3] = (uint8_t)((vy >> 8) & 0xFF);
    send_buff[4] = (uint8_t)(vy & 0xFF);
    send_buff[5] = (uint8_t)((vz >> 8) & 0xFF);
    send_buff[6] = (uint8_t)(vz & 0xFF);
    const uint8_t mode_byte = zone_switcher_->current_mode_byte();
    send_buff[7] = mode_byte; // 模式字节

    try
    {
        RCLCPP_INFO(this->get_logger(), "Send mode: %u", static_cast<unsigned int>(mode_byte));
        serial_port_.write(send_buff, CMD_LEN);
    }
    catch (const std::exception &e)
    {
        (void)e;
    }
}

// --- cmd_vel_callback 实现 (发送速度到下位机) ---
void ReceiveNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cached_vx_ = static_cast<int16_t>(msg->linear.x * 6000);
    cached_vy_ = static_cast<int16_t>(msg->linear.y * 6000);
    cached_vz_ = static_cast<int16_t>(msg->angular.z * 6000);

    if (!is_serial_open_)
        return;

    // 只有有导航目标时，才按 cmd_vel 到达频率发送
    if (!has_nav_goal_active_)
        return;

    send_cmd_packet(cached_vx_, cached_vy_, cached_vz_);
}

// --- send_navigation_goal 实现 (发送导航目标) ---
void ReceiveNode::send_navigation_goal(double x, double y)
{
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

    nav_goal_inflight_ = true;
    has_last_nav_goal_ = true;
    last_nav_goal_x_ = x;
    last_nav_goal_y_ = y;
    last_nav_goal_send_time_ = this->now();

    auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<Action>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            nav_goal_inflight_ = false;
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
            has_nav_goal_active_ = false;
            nav_goal_inflight_ = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
            has_nav_goal_active_ = false;
            nav_goal_inflight_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Navigation canceled");
            has_nav_goal_active_ = false;
            nav_goal_inflight_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Navigation unknown result");
            has_nav_goal_active_ = false;
            nav_goal_inflight_ = false;
            break;
        }
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void ReceiveNode::execute_mode_navigation(const rm_communication::DecisionResult &decision)
{
    has_nav_goal_active_ = decision.has_nav_goal;

    // 导航与模式是同级决策：有目标就发，不再依赖“模式是否切换”
    if (!decision.has_nav_goal)
    {
        nav_goal_inflight_ = false;
        return;
    }

    const bool same_goal = has_last_nav_goal_ &&
                           (std::fabs(decision.goal_x - last_nav_goal_x_) < 0.05) &&
                           (std::fabs(decision.goal_y - last_nav_goal_y_) < 0.05);

    // 同一目标且仍在执行中：不重复发送
    if (same_goal && nav_goal_inflight_)
    {
        return;
    }

    // 同一目标刚发送过：做最小重发间隔，避免高频触发 Nav2 中止
    const auto now = this->now();
    if (same_goal && (now - last_nav_goal_send_time_).seconds() < 1.0)
    {
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "策略执行：mode=%d, 发送导航目标(%.2f, %.2f)",
                static_cast<int>(decision.mode), decision.goal_x, decision.goal_y);
    send_navigation_goal(decision.goal_x, decision.goal_y);
}

void ReceiveNode::timer_callback()
{

    if (!is_serial_open_)
    {
        try
        {
            serial_port_.open();
            is_serial_open_ = true;
            RCLCPP_INFO(this->get_logger(), "串口成功打开");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_WARN(this->get_logger(), "串口打开失败(%s): %s", port_name_.c_str(), e.what());
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unhandled exception during open: %s", e.what());
            return;
        }
    }

    try
    {
        size_t available = serial_port_.available();
        if (available > 0)
        {
            std::vector<uint8_t> temp(available);
            serial_port_.read(temp.data(), available);
            buffer_.insert(buffer_.end(), temp.begin(), temp.end());
        }

        if (data_type_ == DATA_TYPE_SEVEN)
        {
            parse_seven();
        }
        else
        {
            parse_three();
        }

        // 没有导航目标时，按 10Hz 周期发送最新速度缓存
        const auto now = this->now();
        if (!has_nav_goal_active_ && (now - last_idle_cmd_send_time_).nanoseconds() >= 100000000)
        {
            send_cmd_packet(cached_vx_, cached_vy_, cached_vz_);
            last_idle_cmd_send_time_ = now;
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
void ReceiveNode::parse_three()
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

        std::vector<uint8_t> packet;
        DownlinkDataThree *data = nullptr;
        bool checksum_ok = false;
        auto erase_end_it = it + 1;

        // 格式A（原定义）：[payload][checksum][tail]
        if (head_idx >= 0)
        {
            packet.assign(buffer_.begin() + head_idx, buffer_.begin() + head_idx + LEN_THREE);
            data = reinterpret_cast<DownlinkDataThree *>(packet.data());
            checksum_ok = (calc_checksum(packet) == data->checksum);
        }

        // 格式B（兼容下位机常见实现）：[payload][tail][checksum]
        if (!checksum_ok)
        {
            long head_idx_b = tail_idx - (LEN_THREE - 2);
            if (head_idx_b >= 0 && (tail_idx + 1) < static_cast<long>(buffer_.size()))
            {
                packet.assign(buffer_.begin() + head_idx_b, buffer_.begin() + head_idx_b + LEN_THREE);
                data = reinterpret_cast<DownlinkDataThree *>(packet.data());
                uint8_t checksum_b = packet[LEN_THREE - 1];
                checksum_ok = (packet[LEN_THREE - 2] == FRAME_TAIL) && (calc_checksum(packet) == checksum_b);
                if (checksum_ok)
                {
                    erase_end_it = it + 2; // 额外吞掉 tail 后面的 checksum
                }
            }
        }

        if (!checksum_ok)
        {
            buffer_.erase(buffer_.begin(), it + 1);
            RCLCPP_WARN(this->get_logger(), "3V3 Checksum mismatch! Drop packet.");
            continue;
        }

        // 校验成功：移除缓冲区中的这帧数据
        buffer_.erase(buffer_.begin(), erase_end_it);

        // --- 提取并转换全部 12 个数据字段 ---
        int16_t score_diff_val = (int16_t)ntohs(data->score_diff); 
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

        // --- 更新模式决策（3V3） ---
        double robot_x = 0.0, robot_y = 0.0;
    if (zone_switcher_->get_robot_pose(robot_x, robot_y))
    {
        mode_params_.robot_x = robot_x;
        mode_params_.robot_y = robot_y;
        mode_params_.our_sentry_blood = our_sentry_blood_val; // 哨兵血量
        mode_params_.enemy_sentry_blood = enemy_sentry_blood_val; // 敌方哨兵血量
        mode_params_.match_time = match_time_val;
        
        // 同级决策：一次 update 同时得到“模式 + 导航目标”
        auto decision = zone_switcher_->update(mode_params_);
        execute_mode_navigation(decision);
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

        std::vector<uint8_t> packet;
        DownlinkDataSeven *data = nullptr;
        bool checksum_ok = false;
        auto erase_end_it = it + 1;

        // 格式A（原定义）：[payload][checksum][tail]
        if (head_idx >= 0)
        {
            packet.assign(buffer_.begin() + head_idx, buffer_.begin() + head_idx + LEN_SEVEN);
            data = reinterpret_cast<DownlinkDataSeven *>(packet.data());
            checksum_ok = (calc_checksum(packet) == data->checksum);
        }

        // 格式B（兼容）：[payload][tail][checksum]
        if (!checksum_ok)
        {
            long head_idx_b = tail_idx - (LEN_SEVEN - 2);
            if (head_idx_b >= 0 && (tail_idx + 1) < static_cast<long>(buffer_.size()))
            {
                packet.assign(buffer_.begin() + head_idx_b, buffer_.begin() + head_idx_b + LEN_SEVEN);
                data = reinterpret_cast<DownlinkDataSeven *>(packet.data());
                uint8_t checksum_b = packet[LEN_SEVEN - 1];
                checksum_ok = (packet[LEN_SEVEN - 2] == FRAME_TAIL) && (calc_checksum(packet) == checksum_b);
                if (checksum_ok)
                {
                    erase_end_it = it + 2;
                }
            }
        }

        if (!checksum_ok)
        {
            buffer_.erase(buffer_.begin(), it + 1);
            RCLCPP_WARN(this->get_logger(), "7V7 Checksum mismatch! Drop packet.");
            continue;
        }

        // 校验成功
        buffer_.erase(buffer_.begin(), erase_end_it);

        // --- 提取并转换全部 8 个数据字段 ---
        int16_t base_hp_diff_val = (int16_t)ntohs(data->base_hp_diff); // 修复符号数bug
        uint32_t time_val = ntohl(data->time);
        uint16_t hp_val = ntohs(data->hp);
        uint16_t bullet_num_val = ntohs(data->bullet_num);

        uint8_t enemy_outpost_alive_val = data->enemy_outpost_alive;
        uint8_t my_alive_num_val = data->my_alive_num;
        uint8_t enemy_alive_num_val = data->enemy_alive_num;
        uint8_t shooter_on_val = data->shooter_on;

        // --- 更新模式决策（7V7） ---
        double robot_x = 0.0, robot_y = 0.0;
        if (zone_switcher_->get_robot_pose(robot_x, robot_y))
        {
            mode_params_.robot_x = robot_x;
            mode_params_.robot_y = robot_y;
            mode_params_.our_sentry_blood = hp_val; // 自身血量
            mode_params_.match_time = time_val;
        
            auto decision = zone_switcher_->update(mode_params_);
            execute_mode_navigation(decision);
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

// --- main 函数 ---
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // 引用 ReceiveNode 类，而不是内联定义
    rclcpp::spin(std::make_shared<ReceiveNode>());
    rclcpp::shutdown();          
    return 0;
}
