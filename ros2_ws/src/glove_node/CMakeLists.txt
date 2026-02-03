cmake_minimum_required(VERSION 3.8)
project(glove_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories(include ${JSONCPP_INCLUDE_DIRS})

add_executable(glove_node
  src/glove_node.cpp
  src/UDEServer.cxx
)

target_link_libraries(glove_node
  ${JSONCPP_LIBRARIES}
  jsoncpp
)

ament_target_dependencies(glove_node
  rclcpp
  common_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS glove_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
