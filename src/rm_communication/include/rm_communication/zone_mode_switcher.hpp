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

enum class ZoneMode : uint8_t { ATTACK = 1, MOVE = 2, DEFENSE = 3 };

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
    uint16_t enemy_sentry_blood{260}; 
};

struct DecisionResult
{
    ZoneMode mode{ZoneMode::MOVE};
    bool has_nav_goal{false};
    double goal_x{0.0};
    double goal_y{0.0};
};

class ZoneModeSwitcher
{
public:
    ZoneModeSwitcher(rclcpp::Node *node,
                                         RectZone defense = RectZone{2.6, 5.9, -5.65, 2.3})
        : node_(node), 
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
          tf_listener_(*tf_buffer_), 
          defense_zone_(defense),
          last_blood_(400), // 初始血量，根据实际车型调整
          last_match_time_(300)
    {
        current_mode_ = ZoneMode::MOVE;
        RCLCPP_INFO(node_->get_logger(), "ZoneModeSwitcher initialized");
    }

    DecisionResult update(const ModeDecisionParams &params)
    {
        DecisionResult result;
        ZoneMode new_mode = current_mode_;
        // constexpr double attack_x = 4.32;
        // constexpr double attack_y = -1.81;
        // constexpr double defense_x = -1.05;
        // constexpr double defense_y = 1.44;
        // constexpr double arrive_dist = 0.35;
        constexpr double attack_x = 2.36;
        constexpr double attack_y = 0.517;
        constexpr double defense_x = -1.69;
        constexpr double defense_y = 0.2333;
        constexpr double arrive_dist = 0.35;
        const double arrive_dist_sq = arrive_dist * arrive_dist;

        const auto dist_sq = [](double x1, double y1, double x2, double y2) {
            const double dx = x1 - x2;
            const double dy = y1 - y2;
            return dx * dx + dy * dy;
        };

        const bool at_attack_point =
            dist_sq(params.robot_x, params.robot_y, attack_x, attack_y) <= arrive_dist_sq;
        const bool at_defense_point =
            dist_sq(params.robot_x, params.robot_y, defense_x, defense_y) <= arrive_dist_sq;

        // 规则1：血量 > 350 -> 导航进攻点，姿态=1
        if (params.our_sentry_blood > 350)
        {
            // 规则4：当处于进攻点且无导航目标时，姿态=2
            if (at_attack_point)
            {
                new_mode = ZoneMode::MOVE;
                result.has_nav_goal = false;
            }
            else
            {
                new_mode = ZoneMode::ATTACK;
                result.has_nav_goal = true;
                result.goal_x = attack_x;
                result.goal_y = attack_y;
            }
        }
        // 规则2：血量 < 100 -> 导航防御点，姿态=3
        else if (params.our_sentry_blood < 100)
        {
            new_mode = ZoneMode::DEFENSE;
            if (at_defense_point)
            {
                result.has_nav_goal = false;
            }
            else
            {
                result.has_nav_goal = true;
                result.goal_x = defense_x;
                result.goal_y = defense_y;
            }
        }
        // 其它情况：不发导航目标，姿态=2
        else
        {
            new_mode = ZoneMode::ATTACK;
            result.has_nav_goal = false;
        }

        // 更新状态记录
        last_blood_ = params.our_sentry_blood;
        last_match_time_ = params.match_time;

        // 状态变更日志
        if (new_mode != current_mode_)
        {
            RCLCPP_INFO(node_->get_logger(), 
                       "Mode Changed: %d -> %d | Blood: %u",
                       static_cast<int>(current_mode_), static_cast<int>(new_mode), 
                       params.our_sentry_blood);
            current_mode_ = new_mode;
        }
        result.mode = current_mode_;
        return result;
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
    RectZone defense_zone_;
    ZoneMode current_mode_;

    // 新增：用于计算变化率的状态变量
    uint16_t last_blood_;
    uint32_t last_match_time_;
};

} // namespace rm_communication

#endif