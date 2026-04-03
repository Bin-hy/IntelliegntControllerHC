cmake_minimum_required(VERSION 3.8)
project(lift_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(duco_msg REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LiftControl.srv"
)

add_executable(lift_server_node src/lift_server_node.cpp)
ament_target_dependencies(lift_server_node rclcpp duco_msg)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(lift_server_node "${cpp_typesupport_target}")

install(TARGETS lift_server_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
