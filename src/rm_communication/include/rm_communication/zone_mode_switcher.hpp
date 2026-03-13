#ifndef ZONE_MODE_SWITCHER_HPP
#define ZONE_MODE_SWITCHER_HPP

#include <cstdint>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/time.h>

namespace rm_communication
{

enum class ZoneMode : uint8_t { DEFENSE = 0, ATTACK = 1, MOVE = 2 };

struct RectZone
{
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    bool contains(double x, double y) const
    {
        return x > x_min && x < x_max && y > y_min && y < y_max;
    }
};

struct ModeDecisionParams
{
    double robot_x{0.0};
    double robot_y{0.0};
    uint16_t our_sentry_blood{400};
    uint32_t match_time{300}; 
};

class ZoneModeSwitcher
{
public:
    ZoneModeSwitcher(rclcpp::Node *node,
                     RectZone attack = RectZone{5.0, 10.0, -2.0, 2.0},
                     RectZone defense = RectZone{-2.0, 2.0, -3.0, 3.0})
        : node_(node), 
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
          tf_listener_(*tf_buffer_), 
          attack_zone_(attack), 
          defense_zone_(defense),
          last_blood_(400), // 初始血量，根据实际车型调整
          last_match_time_(300)
    {
        current_mode_ = ZoneMode::MOVE;
        RCLCPP_INFO(node_->get_logger(), "ZoneModeSwitcher initialized");
    }

    ZoneMode update(const ModeDecisionParams &params)
    {
        ZoneMode new_mode = current_mode_;
        
        // --- 1. 计算血量变化率 ---
        double blood_drop_rate = 0.0;
        if (params.match_time > last_match_time_) {
            // 计算每秒掉血量 (delta_blood / delta_time)
            int32_t delta_blood = static_cast<int32_t>(last_blood_) - static_cast<int32_t>(params.our_sentry_blood);
            uint32_t delta_time = params.match_time - last_match_time_;
            
            // 只有在掉血时才计算变化率（回血不计入）
            if (delta_blood > 0) {
                blood_drop_rate = static_cast<double>(delta_blood) / delta_time;
            }
        }

        // --- 2. 核心逻辑决策（按优先级排列） ---

        // 策略 A: 比赛前10秒，开启移动模式
        if (params.match_time <= 10)
        {
            new_mode = ZoneMode::MOVE;
        }
        // 策略 B: 血量低于100，紧急防御（回家）
        else if (params.our_sentry_blood < 100)
        {
            new_mode = ZoneMode::DEFENSE;
        }
        // 策略 C: 比赛最后阶段（假设 match_time 增加，且总长大于60s，此处逻辑为时间 > 比赛总长-60）
        // 如果你的 params.match_time 是倒计时，请改为 params.match_time < 60
        else if (params.match_time >= 360) // 假设420秒比赛，最后60秒进攻
        {
            new_mode = ZoneMode::ATTACK;
        }
        // 策略 D: 根据血量下降速度
        else if (blood_drop_rate > 50.0) // 阈值示例：每秒掉血超过50点算“极快”
        {
            new_mode = ZoneMode::DEFENSE;
        }
        else if (blood_drop_rate > 5.0 && blood_drop_rate <= 50.0) // 掉血慢（有损血但尚可承受）
        {
            new_mode = ZoneMode::ATTACK;
        }
        // 策略 E: 默认区域逻辑
        else 
        {
            if (attack_zone_.contains(params.robot_x, params.robot_y))
                new_mode = ZoneMode::ATTACK;
            else if (defense_zone_.contains(params.robot_x, params.robot_y))
                new_mode = ZoneMode::DEFENSE;
            else
                new_mode = ZoneMode::MOVE;
        }

        // 更新状态记录
        last_blood_ = params.our_sentry_blood;
        last_match_time_ = params.match_time;

        // 状态变更日志
        if (new_mode != current_mode_)
        {
            RCLCPP_INFO(node_->get_logger(), 
                       "Mode Changed: %d -> %d | Blood: %u | Rate: %.1f",
                       static_cast<int>(current_mode_), static_cast<int>(new_mode), 
                       params.our_sentry_blood, blood_drop_rate);
            current_mode_ = new_mode;
        }

        return current_mode_;
    }

    // 从 TF 中获取机器人坐标，用于独立决策（可选）
    bool get_robot_pose(double &x, double &y)
    {
        try
        {
            geometry_msgs::msg::TransformStamped trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            x = trans.transform.translation.x;
            y = trans.transform.translation.y;
            return true;
        }
        catch (const std::exception &e)
        {
            return false;
        }
    }

    ZoneMode current_mode() const { return current_mode_; }
    uint8_t current_mode_byte() const { return static_cast<uint8_t>(current_mode_); }

private:
    rclcpp::Node *node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    RectZone attack_zone_;
    RectZone defense_zone_;
    ZoneMode current_mode_;

    // 新增：用于计算变化率的状态变量
    uint16_t last_blood_;
    uint32_t last_match_time_;
};

} // namespace rm_communication

#endif