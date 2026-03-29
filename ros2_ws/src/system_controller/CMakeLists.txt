cmake_minimum_required(VERSION 3.8)
project(system_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(duco_msg REQUIRED)
find_package(common_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lhandpro_interfaces REQUIRED)
find_package(vision_server REQUIRED)
find_package(lift_server REQUIRED)

add_executable(system_controller_node src/system_controller_node.cpp)
ament_target_dependencies(system_controller_node rclcpp std_msgs std_srvs duco_msg common_msgs rclcpp_action lhandpro_interfaces vision_server lift_server)

install(TARGETS
  system_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
