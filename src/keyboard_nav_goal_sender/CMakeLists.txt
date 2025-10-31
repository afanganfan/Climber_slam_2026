cmake_minimum_required(VERSION 3.5)
project(keyboard_nav_goal_sender)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(keyboard_nav_goal_sender_node src/keyboard_nav_goal_sender_node.cpp)
ament_target_dependencies(keyboard_nav_goal_sender_node rclcpp rclcpp_action geometry_msgs nav2_msgs std_msgs)

install(TARGETS keyboard_nav_goal_sender_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
