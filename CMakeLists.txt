cmake_minimum_required(VERSION 3.8)
project(nav_serial)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出编译命令（用于IDE）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#=============================================================================
# 依赖
#=============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

#=============================================================================
# 库：传输层
#=============================================================================

add_library(uart_transporter SHARED
  src/transporter_driver/uart_transporter.cpp
)

target_include_directories(uart_transporter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#=============================================================================
# 库：串口驱动核心（独立于ROS）
#=============================================================================

add_library(serial_driver_core SHARED
  src/driver/serial_driver.cpp
)

target_include_directories(serial_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(serial_driver_core
  uart_transporter
)

#=============================================================================
# 可执行文件：ROS2 节点
#=============================================================================

add_executable(serial_driver_node
  src/ros/serial_driver_node.cpp
)

target_include_directories(serial_driver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(serial_driver_node
  serial_driver_core
)

ament_target_dependencies(serial_driver_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rm_interfaces
)

#=============================================================================
# 安装
#=============================================================================

# 安装库
install(TARGETS
  uart_transporter
  serial_driver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
install(TARGETS
  serial_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

#=============================================================================
# 测试
#=============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 可选：添加单元测试
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_packet_protocol test/test_packet_protocol.cpp)
  # target_link_libraries(test_packet_protocol serial_driver_core)
endif()

#=============================================================================
# 导出
#=============================================================================

ament_export_include_directories(include)
ament_export_libraries(uart_transporter serial_driver_core)

ament_package()
