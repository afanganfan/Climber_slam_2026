cmake_minimum_required(VERSION 3.5)
project(ballistic_angle_solver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ballistic_angle_node src/ballistic_angle_node.cpp)
ament_target_dependencies(ballistic_angle_node rclcpp std_srvs nav_msgs tf2_ros geometry_msgs tf2_geometry_msgs Eigen3)

add_executable(robot_pose_echo_node src/robot_pose_echo_node.cpp)
ament_target_dependencies(robot_pose_echo_node rclcpp nav_msgs)

install(TARGETS ballistic_angle_node robot_pose_echo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
ament_package()
