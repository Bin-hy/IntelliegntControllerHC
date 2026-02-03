cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(rclcpp_action)
find_package(control_msgs)
find_package(sensor_msgs)
find_package(duco_msg)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(geometry_msgs REQUIRED)# uncomment the following section in order to fill in
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(robot_control src/robot_control.cpp)
ament_target_dependencies(robot_control rclcpp rclcpp_action control_msgs sensor_msgs duco_msg
            sensor_msgs
            moveit_ros_planning_interface
            geometry_msgs
            pcl_conversions PCL)

target_link_libraries(robot_control ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)

install (TARGETS robot_control DESTINATION lib/${PROJECT_NAME}
    )
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
