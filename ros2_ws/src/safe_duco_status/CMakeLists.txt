cmake_minimum_required(VERSION 3.8)
project(safe_duco_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(duco_msg REQUIRED)

include_directories(include)

# Use the Duco SDK library
# Assuming the library is installed in the install/lib of duco_ros_driver or similar
# Or we can link against the static library found in duco_ros2_driver
set(DUCO_LIB_PATH "/home/user/IntelliegntControllerHC/duco_ros2_driver/src/robot_control/lib/libDucoCobotAPI.a")

add_executable(safe_duco_status_node src/safe_duco_status_node.cpp)
target_include_directories(safe_duco_status_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/home/user/IntelliegntControllerHC/duco_ros2_driver/src/duco_ros_driver/include"
  "/home/user/IntelliegntControllerHC/duco_ros2_driver/src/robot_control/include"
)

target_link_libraries(safe_duco_status_node
  ${DUCO_LIB_PATH}
)

ament_target_dependencies(safe_duco_status_node
  rclcpp
  std_msgs
  sensor_msgs
  control_msgs
  duco_msg
)

install(TARGETS safe_duco_status_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
