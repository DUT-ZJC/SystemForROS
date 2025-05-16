cmake_minimum_required(VERSION 3.5)
project(robot_act_move)

# 必须在调用任何 Ament 命令前声明依赖
find_package(ament_cmake REQUIRED)

# 默认使用 C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 导出头文件路径（必须在 find_package(ament_cmake) 之后）
ament_export_include_directories(include)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# 其他依赖包
find_package(rclcpp REQUIRED)
find_package(execute_movement_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(nanoflann REQUIRED)
# MoveIt 相关包
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp_action REQUIRED)                # ← 新增
find_package(control_msgs REQUIRED)   

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED COMPONENTS trajectory_processing)

# 编译可执行文件
add_executable(robot_server
  src/robot_server.cpp
  src/RobotAct.cpp
)

add_executable(move_test
  src/move_test.cpp
)

# 链接依赖
ament_target_dependencies(robot_server
  rclcpp
  execute_movement_msgs
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  yaml-cpp
  sensor_msgs
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  nanoflann
  rclcpp_action
  control_msgs
)

ament_target_dependencies(move_test
  rclcpp
  execute_movement_msgs
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
)

target_link_libraries(robot_server
  ${YAML_CPP_LIBRARIES}
)

# 安装可执行文件
install(TARGETS robot_server
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS move_test
  DESTINATION lib/${PROJECT_NAME}
)


# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 必须放在最后
ament_package()