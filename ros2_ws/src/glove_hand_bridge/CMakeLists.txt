cmake_minimum_required(VERSION 3.8)
project(glove_hand_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_msgs REQUIRED)
find_package(lhandpro_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(glove_hand_bridge
  src/glove_hand_bridge.cpp
)

ament_target_dependencies(glove_hand_bridge
  rclcpp
  common_msgs
  lhandpro_interfaces
  std_msgs
  std_srvs
)

install(TARGETS glove_hand_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
