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

        // --- 2. 核心逻辑决策（冲突仲裁优先级：A=B=C=D > E > F） ---
        struct StrategyCandidate
        {
            bool active{false};
            int priority{0};
            ZoneMode mode{ZoneMode::MOVE};
            bool has_nav_goal{false};
            double goal_x{0.0};
            double goal_y{0.0};
        };

        auto choose_better = [](const StrategyCandidate &best, const StrategyCandidate &cand) {
            if (!cand.active)
                return false;
            if (!best.active)
                return true;
            return cand.priority > best.priority;
        };

        // 同级优先级：A/B/C/D 均为 3；E 为 2；F 为 1
        StrategyCandidate best;

        // 策略 A: 比赛前10秒，开启移动模式
        StrategyCandidate cand_a;
        cand_a.active = (params.match_time >= 290);
        cand_a.priority = 3;
        cand_a.mode = ZoneMode::MOVE;
        cand_a.has_nav_goal = true;
        cand_a.goal_x = 4.32;
        cand_a.goal_y = -1.81;
        if (choose_better(best, cand_a)) best = cand_a;

        // 策略 B: 血量低于100，紧急防御（回家）
        StrategyCandidate cand_b;
        cand_b.active = (params.our_sentry_blood < 100);
        cand_b.priority = 3;
        cand_b.mode = ZoneMode::DEFENSE;
        cand_b.has_nav_goal = true;
        cand_b.goal_x = -1.05;
        cand_b.goal_y = 1.44;
        if (choose_better(best, cand_b)) best = cand_b;

        // 策略 C: 比赛最后60秒进攻
        StrategyCandidate cand_c;
        cand_c.active = (params.match_time <= 60);
        cand_c.priority = 3;
        cand_c.mode = ZoneMode::ATTACK;
        cand_c.has_nav_goal = true;
        cand_c.goal_x = 4.3;
        cand_c.goal_y = -0.329;
        if (choose_better(best, cand_c)) best = cand_c;

        // 策略 D: 根据血量下降速度
        StrategyCandidate cand_d;
        cand_d.active = (blood_drop_rate > 5.0);
        cand_d.priority = 3;
        cand_d.mode = (blood_drop_rate > 50.0) ? ZoneMode::DEFENSE : ZoneMode::ATTACK;
        if (choose_better(best, cand_d)) best = cand_d;

        // 策略 E：根据热度值（heat）判断是否进入攻击模式
        StrategyCandidate cand_e;
        cand_e.active = (params.enemy_sentry_blood < 60);
        cand_e.priority = 2;
        cand_e.mode = ZoneMode::ATTACK;
        if (choose_better(best, cand_e)) best = cand_e;

        // 策略 F: 默认区域逻辑
        StrategyCandidate cand_f;
        cand_f.active = true;
        cand_f.priority = 1;
        if (defense_zone_.contains(params.robot_x, params.robot_y))
            cand_f.mode = ZoneMode::DEFENSE;
        else
            cand_f.mode = ZoneMode::MOVE;
        if (choose_better(best, cand_f)) best = cand_f;

        // 应用最终策略结果
        new_mode = best.mode;
        result.has_nav_goal = best.has_nav_goal;
        result.goal_x = best.goal_x;
        result.goal_y = best.goal_y;

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