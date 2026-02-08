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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_server REQUIRED)
find_package(lhandpro_interfaces REQUIRED)
include_directories(/home/user/IntelliegntControllerHC/ros2_ws/install/vision_server/include/vision_server)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL 3DCore 3DRender 3DExtras 3DInput 3DLogic)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(ui_app 
  src/main.cpp
  src/ros_node.cpp
  src/app_window.cpp
  src/point_cloud_widget.cpp
  src/robot_viz_widget.cpp
  include/ui_app/point_cloud_widget.hpp
  include/ui_app/robot_viz_widget.hpp
  resources/resources.qrc
)

target_include_directories(ui_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ui_app
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::3DCore
  Qt5::3DRender
  Qt5::3DExtras
  Qt5::3DInput
  Qt5::3DLogic
  /home/user/IntelliegntControllerHC/ros2_ws/install/vision_server/lib/libvision_server__rosidl_typesupport_cpp.so
)

ament_target_dependencies(ui_app
  rclcpp
  std_msgs
  duco_msg
  sensor_msgs
  cv_bridge
  std_srvs
  image_transport
  OpenCV
  lhandpro_interfaces
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  urdf
  ament_index_cpp
)

install(TARGETS ui_app
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
