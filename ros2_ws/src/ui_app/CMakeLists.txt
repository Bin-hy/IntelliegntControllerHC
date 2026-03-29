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
find_package(lift_server REQUIRED)
find_package(lhandpro_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL 3DCore 3DRender 3DExtras 3DInput 3DLogic)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)

  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTORCC ON)

  add_executable(ui_app
    src/main.cpp
    src/ros_node.cpp
    src/app_window.cpp
    src/app_window_task.cpp
    src/task_widget.cpp
    src/task_dialogs.cpp
    src/task_record_manager.cpp
    src/auth_manager.cpp
    src/auth_log_manager.cpp
    src/permission_manager.cpp
    src/login_window.cpp
    src/idle_logout_watcher.cpp
    src/point_cloud_widget.cpp
    src/robot_viz_widget.cpp
    thirdparty/ude_glove_sdk/UDEServer.cxx
    resources/resources.qrc
    include/ui_app/point_cloud_widget.hpp
    include/ui_app/robot_viz_widget.hpp
    include/ui_app/task_widget.hpp
    include/ui_app/task_dialogs.hpp
    include/ui_app/task_record_manager.hpp
    include/ui_app/auth_manager.hpp
    include/ui_app/auth_log_manager.hpp
    include/ui_app/permission_manager.hpp
    include/ui_app/login_window.hpp
    include/ui_app/idle_logout_watcher.hpp
  )

  target_include_directories(ui_app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_BINARY_DIR}/../vision_server/rosidl_generator_cpp
    ${CMAKE_BINARY_DIR}/../lift_server/rosidl_generator_cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ude_glove_sdk
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
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ude_glove_sdk/libjsoncpp.a
  )

  ament_target_dependencies(ui_app
    rclcpp
    std_msgs
    duco_msg
    common_msgs
    sensor_msgs
    cv_bridge
    std_srvs
    image_transport
    OpenCV
    lhandpro_interfaces
    vision_server
    lift_server
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    urdf
    ament_index_cpp
    rclcpp_action
  )

  install(TARGETS ui_app
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
