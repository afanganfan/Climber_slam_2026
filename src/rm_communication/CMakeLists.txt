cmake_minimum_required(VERSION 3.14)
project(rm_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖（按功能分组，仅保留必要依赖）
find_package(ament_cmake REQUIRED)

# 公共依赖
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 特定目标依赖
find_package(rclcpp_action REQUIRED)  # 用于action client
find_package(serial REQUIRED)         # 串口通信
find_package(geometry_msgs REQUIRED)  # 涉及Twist等消息
find_package(nav2_msgs REQUIRED)      # 导航 action 接口
find_package(message_filters REQUIRED) # 用于lis/monitor


include_directories(
  include
)
add_executable(talker
  src/talker.cpp # 类的实现文件
)
ament_target_dependencies(talker
  rclcpp
  std_msgs
  geometry_msgs
  serial
  nav2_msgs
  rclcpp_action
)

# 3.2 订阅导航消息节点
add_executable(sub_from_nav src/sub_from_nav.cpp)
ament_target_dependencies(sub_from_nav
  rclcpp
  std_msgs
  rclcpp_action
  nav2_msgs
)

# 3.3 serial_echo: simple node that prints raw serial data to terminal
add_executable(serial_echo src/serial_echo.cpp)
ament_target_dependencies(serial_echo
  rclcpp
  serial
)

# 3.4 监听节点
add_executable(lis src/lis.cpp)
ament_target_dependencies(lis
  rclcpp
  message_filters
  serial
  std_msgs
)

# 4. 安装所有可执行文件（合并为一个install命令，简洁）
install(TARGETS
  talker
  sub_from_nav
  serial_echo
  lis
  DESTINATION lib/${PROJECT_NAME}
)

# 5. 安装头文件 (重要：如果其他包需要使用您的头文件，需安装)
install(DIRECTORY
  include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# 6. 测试与打包
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()