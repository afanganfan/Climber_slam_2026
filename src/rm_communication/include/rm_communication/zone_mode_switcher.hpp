#ifndef ZONE_MODE_SWITCHER_HPP
#define ZONE_MODE_SWITCHER_HPP

#include <cstdint>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/time.h>

namespace rm_communication
{

enum class ZoneMode : uint8_t { ATTACK = 1, MOVE = 2, DEFENSE = 3, SENSITIVE = 4 };

struct RectZone
{
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};
    bool contains(double x, double y) const
    {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

struct ModeZone
{
    ZoneMode mode{ZoneMode::MOVE};
    RectZone rect;
    int priority{0};
    size_t order{0};
};

struct ModeDecisionParams
{
    double robot_x{0.0};
    double robot_y{0.0};
    uint16_t our_sentry_blood{400};
    uint32_t match_time{300};
    uint16_t enemy_sentry_blood{260};
    // 当上位机协议提供热量时置为 true，并填写 sentry_heat / heat_limit。
    bool has_heat{false};
    uint16_t sentry_heat{0};
    uint16_t heat_limit{200};
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
                                         RectZone defense = RectZone{2.6, 5.9, -5.65, 2.3},
                                         const std::string &zone_config_path = "")
        : node_(node), 
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
          tf_listener_(*tf_buffer_), 
          defense_zone_(defense),
                    zone_config_path_(zone_config_path),
          last_blood_(400),
          last_match_time_(300)
    {
        current_mode_ = ZoneMode::MOVE;
                load_zone_config();
        RCLCPP_INFO(node_->get_logger(), "ZoneModeSwitcher initialized");
    }

    DecisionResult update(const ModeDecisionParams &params)
    {
        DecisionResult result;
        ZoneMode new_mode = current_mode_;

        constexpr double attack_x = 4.32;
        constexpr double attack_y = -1.63;
        constexpr double defense_x = -1.05;
        constexpr double defense_y = 1.44;

        // 姿态2巡航点（两个点来回）
        constexpr double patrol_a_x = 3.42;
        constexpr double patrol_a_y = -3.39;
        constexpr double patrol_b_x = 3.46;
        constexpr double patrol_b_y = 0.536;

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

        // 当前巡航目标点
        double patrol_target_x = (patrol_target_index_ == 0) ? patrol_a_x : patrol_b_x;
        double patrol_target_y = (patrol_target_index_ == 0) ? patrol_a_y : patrol_b_y;
        const bool at_patrol_target =
            dist_sq(params.robot_x, params.robot_y, patrol_target_x, patrol_target_y) <= arrive_dist_sq;

        bool state_rule_matched = false;
        int state_rule_priority = -1;

        const auto apply_state_rule = [&](int priority,
                                          ZoneMode mode,
                                          bool has_goal,
                                          double gx,
                                          double gy) {
            if (priority > state_rule_priority)
            {
                state_rule_priority = priority;
                state_rule_matched = true;
                new_mode = mode;
                result.has_nav_goal = has_goal;
                result.goal_x = gx;
                result.goal_y = gy;
            }
        };

        // 优先级 P1000：低血量或高热量，强制防御。
        const bool low_blood = params.our_sentry_blood < 100;
        const bool high_heat = params.has_heat && (params.sentry_heat >= params.heat_limit);
        if (low_blood || high_heat)
        {
            patrol_initialized_ = false;
            attack_reached_ = false;
            if (at_defense_point)
            {
                apply_state_rule(1000, ZoneMode::DEFENSE, false, 0.0, 0.0);
            }
            else
            {
                apply_state_rule(1000, ZoneMode::DEFENSE, true, defense_x, defense_y);
            }
        }

        // 优先级 P900：比赛末段且血量不高，转保守防守。
        if (params.match_time <= 60 && params.our_sentry_blood < 220)
        {
            patrol_initialized_ = false;
            attack_reached_ = false;
            if (at_defense_point)
            {
                apply_state_rule(900, ZoneMode::DEFENSE, false, 0.0, 0.0);
            }
            else
            {
                apply_state_rule(900, ZoneMode::DEFENSE, true, defense_x, defense_y);
            }
        }

        // 优先级 P800：高血量且热量安全，先打进攻点，再做双点巡航。
        const bool heat_safe_for_attack = !params.has_heat ||
                                          (params.sentry_heat + 20u < params.heat_limit);
        if (params.our_sentry_blood >= 350 && heat_safe_for_attack)
        {
            if (!attack_reached_)
            {
                if (at_attack_point)
                {
                    attack_reached_ = true;
                    patrol_initialized_ = true;
                    patrol_target_index_ = 0;
                }
                else
                {
                    apply_state_rule(800, ZoneMode::ATTACK, true, attack_x, attack_y);
                }
            }

            if (attack_reached_)
            {
                // 已到达过进攻点：开始巡航
                if (at_patrol_target)
                {
                    patrol_target_index_ = 1 - patrol_target_index_;
                    patrol_target_x = (patrol_target_index_ == 0) ? patrol_a_x : patrol_b_x;
                    patrol_target_y = (patrol_target_index_ == 0) ? patrol_a_y : patrol_b_y;
                }

                apply_state_rule(800, ZoneMode::MOVE, true, patrol_target_x, patrol_target_y);
            }
        }

        // 优先级 P700：对方血量明显更低时，积极进攻。
        if (params.our_sentry_blood > (params.enemy_sentry_blood + 80u) &&
            params.our_sentry_blood >= 180 &&
            heat_safe_for_attack)
        {
            apply_state_rule(700, ZoneMode::ATTACK, true, attack_x, attack_y);
        }

        // 状态规则未命中时，才进入坐标区域规则（同样按区域优先级解析）。
        if (!state_rule_matched)
        {
            ZoneMode zone_mode = ZoneMode::MOVE;
            if (resolve_zone_mode(params.robot_x, params.robot_y, zone_mode))
            {
                new_mode = zone_mode;
                result.has_nav_goal = false;
                state_rule_matched = true;
            }
        }

        // 最终兜底：状态和区域都未命中，维持攻击姿态但不下发导航点。
        if (!state_rule_matched)
        {
            new_mode = ZoneMode::ATTACK;
            result.has_nav_goal = false;
            patrol_initialized_ = false;
            attack_reached_ = false;
        }

        // 更新状态记录
        last_blood_ = params.our_sentry_blood;
        last_match_time_ = params.match_time;

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
    static std::string trim(const std::string &value)
    {
        const auto start = value.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
            return "";
        const auto end = value.find_last_not_of(" \t\r\n");
        return value.substr(start, end - start + 1);
    }

    static std::string to_lower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return value;
    }

    static bool parse_mode(const std::string &mode_name, ZoneMode &mode)
    {
        const std::string mode_str = to_lower(trim(mode_name));
        if (mode_str == "attack")
        {
            mode = ZoneMode::ATTACK;
            return true;
        }
        if (mode_str == "defense")
        {
            mode = ZoneMode::DEFENSE;
            return true;
        }
        if (mode_str == "sensitive")
        {
            mode = ZoneMode::SENSITIVE;
            return true;
        }
        if (mode_str == "move")
        {
            mode = ZoneMode::MOVE;
            return true;
        }
        return false;
    }

    static int default_priority_for_mode(ZoneMode mode)
    {
        switch (mode)
        {
        case ZoneMode::DEFENSE:
            return 300;
        case ZoneMode::SENSITIVE:
            return 200;
        case ZoneMode::ATTACK:
            return 100;
        case ZoneMode::MOVE:
        default:
            return 0;
        }
    }

    static double rect_area(const RectZone &rect)
    {
        return std::max(0.0, rect.x_max - rect.x_min) * std::max(0.0, rect.y_max - rect.y_min);
    }

    void load_zone_config()
    {
        zones_.clear();
        if (zone_config_path_.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "zone_config_path is empty, using fallback rules only.");
            return;
        }

        std::ifstream input(zone_config_path_);
        if (!input.is_open())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to open zone config: %s, using fallback rules only.",
                        zone_config_path_.c_str());
            return;
        }

        std::string line;
        size_t line_no = 0;
        while (std::getline(input, line))
        {
            ++line_no;
            line = trim(line);
            if (line.empty() || line[0] == '#')
                continue;

            std::string lower = to_lower(line);
            if (line_no == 1 && lower.find("mode") != std::string::npos)
                continue;

            std::stringstream ss(line);
            std::string mode_str;
            std::string x_min_str;
            std::string x_max_str;
            std::string y_min_str;
            std::string y_max_str;
            std::string priority_str;

            if (!std::getline(ss, mode_str, ',') ||
                !std::getline(ss, x_min_str, ',') ||
                !std::getline(ss, x_max_str, ',') ||
                !std::getline(ss, y_min_str, ',') ||
                !std::getline(ss, y_max_str, ','))
            {
                RCLCPP_WARN(node_->get_logger(), "Invalid zone line %zu: %s", line_no, line.c_str());
                continue;
            }

            ZoneMode mode;
            if (!parse_mode(mode_str, mode))
            {
                RCLCPP_WARN(node_->get_logger(), "Unknown mode at line %zu: %s", line_no, mode_str.c_str());
                continue;
            }

            try
            {
                ModeZone zone;
                zone.mode = mode;
                zone.rect.x_min = std::stod(trim(x_min_str));
                zone.rect.x_max = std::stod(trim(x_max_str));
                zone.rect.y_min = std::stod(trim(y_min_str));
                zone.rect.y_max = std::stod(trim(y_max_str));
                zone.order = zones_.size();
                zone.priority = default_priority_for_mode(mode);

                if (std::getline(ss, priority_str, ','))
                {
                    const std::string p = trim(priority_str);
                    if (!p.empty())
                    {
                        zone.priority = std::stoi(p);
                    }
                }

                if (zone.rect.x_min > zone.rect.x_max)
                    std::swap(zone.rect.x_min, zone.rect.x_max);
                if (zone.rect.y_min > zone.rect.y_max)
                    std::swap(zone.rect.y_min, zone.rect.y_max);

                zones_.push_back(zone);
            }
            catch (const std::exception &)
            {
                RCLCPP_WARN(node_->get_logger(), "Invalid numeric value at line %zu: %s", line_no, line.c_str());
            }
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Loaded %zu mode zones from %s",
                    zones_.size(), zone_config_path_.c_str());
    }

    bool resolve_zone_mode(double x, double y, ZoneMode &mode) const
    {
        const ModeZone *best_zone = nullptr;
        for (const auto &zone : zones_)
        {
            if (zone.rect.contains(x, y))
            {
                if (best_zone == nullptr)
                {
                    best_zone = &zone;
                    continue;
                }

                if (zone.priority > best_zone->priority)
                {
                    best_zone = &zone;
                    continue;
                }

                if (zone.priority == best_zone->priority)
                {
                    const double zone_area = rect_area(zone.rect);
                    const double best_area = rect_area(best_zone->rect);

                    // 同优先级时，面积更小的区域更具体，优先命中。
                    if (zone_area < best_area)
                    {
                        best_zone = &zone;
                        continue;
                    }

                    // 若面积也相同，后定义的区域覆盖前定义区域。
                    if (zone_area == best_area && zone.order > best_zone->order)
                    {
                        best_zone = &zone;
                    }
                }
            }
        }

        if (best_zone != nullptr)
        {
            mode = best_zone->mode;
            return true;
        }
        return false;
    }

    rclcpp::Node *node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    RectZone defense_zone_;
    std::string zone_config_path_;
    std::vector<ModeZone> zones_;
    ZoneMode current_mode_;

    uint16_t last_blood_;
    uint32_t last_match_time_;

    int patrol_target_index_{0}; // 0 -> A点, 1 -> B点
    bool patrol_initialized_{false};

    // 是否已到达过进攻点
    bool attack_reached_{false};
};

} // namespace rm_communication

#endif