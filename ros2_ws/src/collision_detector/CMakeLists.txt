cmake_minimum_required(VERSION 3.8)
project(collision_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(duco_msg REQUIRED)
find_package(common_msgs REQUIRED)

# --- 编译 ---
add_executable(collision_detector_node src/collision_detector_node.cpp)
ament_target_dependencies(collision_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  duco_msg
  common_msgs
)

# --- 安装 ---
install(TARGETS
  collision_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch、config、rviz 目录
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
