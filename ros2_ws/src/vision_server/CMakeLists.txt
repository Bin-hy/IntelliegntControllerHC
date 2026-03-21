cmake_minimum_required(VERSION 3.8)
project(vision_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveImage.srv"
  "srv/SavePointCloud.srv"
)

# Image Saver Node
add_executable(image_saver_node src/image_saver_node.cpp)
target_include_directories(image_saver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(image_saver_node
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(image_saver_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_srvs
  common_msgs
)

# Make sure the node depends on the generated messages in the same package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(image_saver_node "${cpp_typesupport_target}")

# Install executable
install(TARGETS image_saver_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install config files (optional, but good practice if we had any)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
