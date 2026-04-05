#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

std::string trim(const std::string &value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos)
  {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

}  // namespace

class NavMissionManager : public rclcpp::Node
{
public:
  using Action = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  NavMissionManager()
  : Node("nav_mission_manager")
  {
    this->declare_parameter("waypoint_csv_path", "src/rm_communication/config/nav_waypoints.csv");
    this->declare_parameter("frame_id", "map");

    waypoint_csv_path_ = this->get_parameter("waypoint_csv_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (!load_waypoints(waypoint_csv_path_))
    {
      RCLCPP_WARN(this->get_logger(), "Waypoint CSV load failed: %s", waypoint_csv_path_.c_str());
    }

    action_client_ = rclcpp_action::create_client<Action>(this, "navigate_through_poses");
    mission_sub_ = this->create_subscription<std_msgs::msg::String>(
      "nav_mission_event", 10,
      std::bind(&NavMissionManager::on_mission_event, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "nav_mission_manager started, csv=%s, frame_id=%s",
      waypoint_csv_path_.c_str(), frame_id_.c_str());
  }

private:
  bool load_waypoints(const std::string &csv_path)
  {
    routes_.clear();

    std::ifstream input(csv_path);
    if (!input.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint csv: %s", csv_path.c_str());
      return false;
    }

    std::string line;
    size_t line_no = 0;
    while (std::getline(input, line))
    {
      ++line_no;
      line = trim(line);
      if (line.empty() || line[0] == '#')
      {
        continue;
      }

      if (line_no == 1 && line.find("route") != std::string::npos)
      {
        continue;
      }

      std::stringstream ss(line);
      std::string route_name;
      std::string seq_str;
      std::string x_str;
      std::string y_str;
      std::string yaw_str;

      if (!std::getline(ss, route_name, ',') ||
          !std::getline(ss, seq_str, ',') ||
          !std::getline(ss, x_str, ',') ||
          !std::getline(ss, y_str, ',') ||
          !std::getline(ss, yaw_str, ','))
      {
        RCLCPP_WARN(this->get_logger(), "Invalid waypoint line %zu: %s", line_no, line.c_str());
        continue;
      }

      try
      {
        Waypoint wp;
        wp.x = std::stod(trim(x_str));
        wp.y = std::stod(trim(y_str));
        wp.yaw = std::stod(trim(yaw_str));
        routes_[trim(route_name)].push_back(wp);
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN(this->get_logger(), "Invalid numeric waypoint at line %zu", line_no);
      }
    }

    for (const auto &kv : routes_)
    {
      RCLCPP_INFO(this->get_logger(), "route '%s' loaded with %zu waypoints", kv.first.c_str(), kv.second.size());
    }

    return !routes_.empty();
  }

  geometry_msgs::msg::PoseStamped to_pose(double x, double y, double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
  }

  void on_mission_event(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string mission = trim(msg->data);
    if (mission.empty())
    {
      return;
    }

    if (mission == "none")
    {
      if (goal_handle_)
      {
        RCLCPP_INFO(this->get_logger(), "Receive mission 'none', canceling active route");
        action_client_->async_cancel_goal(goal_handle_);
      }
      inflight_ = false;
      current_mission_.clear();
      return;
    }

    if (inflight_ && mission == current_mission_)
    {
      return;
    }

    const auto it = routes_.find(mission);
    if (it == routes_.end() || it->second.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No route found for mission: %s", mission.c_str());
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "NavigateThroughPoses action server not available");
      return;
    }

    Action::Goal goal;
    goal.poses.reserve(it->second.size());
    for (const auto &wp : it->second)
    {
      goal.poses.push_back(to_pose(wp.x, wp.y, wp.yaw));
    }

    inflight_ = true;
    current_mission_ = mission;

    auto options = rclcpp_action::Client<Action>::SendGoalOptions();
    options.goal_response_callback =
      [this, mission](GoalHandle::SharedPtr goal_handle)
      {
        if (!goal_handle)
        {
          RCLCPP_ERROR(this->get_logger(), "Mission '%s' rejected", mission.c_str());
          inflight_ = false;
          return;
        }
        goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "Mission '%s' accepted", mission.c_str());
      };

    options.result_callback =
      [this, mission](const GoalHandle::WrappedResult &result)
      {
        inflight_ = false;
        goal_handle_.reset();
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Mission '%s' finished", mission.c_str());
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Mission '%s' aborted", mission.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Mission '%s' canceled", mission.c_str());
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Mission '%s' unknown result", mission.c_str());
            break;
        }
      };

    action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(this->get_logger(), "Send mission '%s' with %zu waypoints", mission.c_str(), it->second.size());
  }

  std::string waypoint_csv_path_;
  std::string frame_id_;
  std::map<std::string, std::vector<Waypoint>> routes_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;

  bool inflight_{false};
  std::string current_mission_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavMissionManager>());
  rclcpp::shutdown();
  return 0;
}
