cmake_minimum_required(VERSION 3.14)
project(rm_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖（按功能分组，仅保留必要依赖）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # 用于action相关目标
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)  # 仅lis/monitor用到
find_package(serial REQUIRED)  # 串口相关目标用到
find_package(geometry_msgs REQUIRED)  # 涉及Twist等消息的目标用到
find_package(nav2_msgs REQUIRED)  # 自定义action依赖
find_package(rosidl_default_generators REQUIRED)  # 生成action接口

# 2. 生成自定义接口（action）
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  DEPENDENCIES nav2_msgs  # 依赖nav2的消息
)

# 3. 添加可执行文件（按顺序：先声明，后设置依赖）
# 3.1 receive节点（核心串口接收，支持参数切换）
add_executable(receive src/receive.cpp)
ament_target_dependencies(receive
  rclcpp
  std_msgs
  geometry_msgs  # 若代码中用到Twist，保留；否则可移除
  serial         # 串口通信必需
)
# 链接自定义action接口（若receive用到则保留，否则注释）
rosidl_target_interfaces(receive ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 3.2 订阅导航消息节点
add_executable(sub_from_nav src/sub_from_nav.cpp)
ament_target_dependencies(sub_from_nav
  rclcpp
  rclcpp_action  # 若用到action通信
  nav2_msgs      # 导航相关消息
)
# 若sub_from_nav用到自定义action，链接接口
rosidl_target_interfaces(sub_from_nav ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 3.3 seven/three格式发送节点
add_executable(talker_seven src/talker_seven.cpp)
ament_target_dependencies(talker_seven
  rclcpp
  std_msgs
  geometry_msgs
  serial
  nav2_msgs
  rclcpp_action
)

add_executable(talker_three src/talker_three.cpp)
ament_target_dependencies(talker_three
  rclcpp
  std_msgs
  geometry_msgs
  serial
  nav2_msgs
  rclcpp_action
)

# serial_echo: simple node that prints raw serial data to terminal
add_executable(serial_echo src/serial_echo.cpp)
ament_target_dependencies(serial_echo
  rclcpp
  serial
)

# 3.4 监听节点
add_executable(lis src/lis.cpp)
ament_target_dependencies(lis
  rclcpp
  message_filters  # 消息过滤必需
  serial
  std_msgs
)

# 3.5 监控节点
add_executable(monitor src/monitor.cpp)
ament_target_dependencies(monitor
  rclcpp
  message_filters
  serial
  std_msgs
)

# 4. 安装所有可执行文件（合并为一个install命令，简洁）
install(TARGETS
  receive
  sub_from_nav
  talker_seven
  talker_three
  serial_echo
  lis
  monitor
  DESTINATION lib/${PROJECT_NAME}
)

# 5. 测试与打包
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # 跳过版权检查（按需开启）
  set(ament_cmake_cpplint_FOUND TRUE)    # 跳过cpplint（按需开启）
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()