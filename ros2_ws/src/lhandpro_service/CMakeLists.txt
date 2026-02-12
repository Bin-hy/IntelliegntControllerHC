cmake_minimum_required(VERSION 3.8)
project(lhandpro_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) # -Wpedantic
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lhandpro_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

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


 
# 设置第三方库路径
set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
find_library(LHANDPRO_LIB NAMES LHandProLib PATHS ${THIRDPARTY_DIR}/lib REQUIRED)
# 导出该库给其他包使用
ament_export_libraries(${LHANDPRO_LIB})
ament_export_include_directories(${THIRDPARTY_DIR}/include)


add_subdirectory(src/SOEM)

# Ensure RPATH includes linked library paths (critical for setcap/EtherCAT)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# Explicitly add ROS 2 lib directory to RPATH to ensure capabilities (setcap) don't break library loading
if(DEFINED ENV{ROS_DISTRO})
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:/opt/ros/$ENV{ROS_DISTRO}/lib")
else()
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:/opt/ros/humble/lib")
endif()

# Force RPATH instead of RUNPATH to handle transitive dependencies (like libstatistics_collector)
# when LD_LIBRARY_PATH is ignored due to capabilities (setcap)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

# 添加执行文件
add_executable(lhandpro_service 
  src/lhandpro_service.cpp
  src/hand_control_service.hpp
  src/hand_control_service.cpp
  src/EthercatMaster.cpp
)

ament_target_dependencies(lhandpro_service rclcpp lhandpro_interfaces sensor_msgs)

# 导出头文件路径
target_link_libraries(lhandpro_service ${LHANDPRO_LIB} soem)
target_include_directories(lhandpro_service PUBLIC
    $<BUILD_INTERFACE:${THIRDPARTY_DIR}/include>
)

# 安装可执行文件
install(TARGETS 
  lhandpro_service
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 .so 文件
install(
  DIRECTORY ${THIRDPARTY_DIR}/lib/
  DESTINATION lib
  USE_SOURCE_PERMISSIONS  # 保持文件权限
  PATTERN "*.so*"  # 匹配所有 .so 文件
)

# 安装 launch 文件
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
