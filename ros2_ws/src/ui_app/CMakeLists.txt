cmake_minimum_required(VERSION 3.8)
project(ui_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(duco_msg REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(ui_app 
  src/main.cpp
  src/ros_node.cpp
  src/app_window.cpp
)

target_include_directories(ui_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ui_app
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

ament_target_dependencies(ui_app
  rclcpp
  std_msgs
  duco_msg
)

install(TARGETS ui_app
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
