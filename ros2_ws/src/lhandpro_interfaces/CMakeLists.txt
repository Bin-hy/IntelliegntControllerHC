cmake_minimum_required(VERSION 3.8)
project(lhandpro_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetEnable.srv"
  "srv/GetAngle.srv"
  "srv/GetAngularVelocity.srv"
  "srv/GetControlMode.srv"
  "srv/GetMaxCurrent.srv"
  "srv/GetNowAngle.srv"
  "srv/GetNowAngularVelocity.srv"
  "srv/GetNowCurrent.srv"
  "srv/GetNowPosition.srv"
  "srv/GetNowPositionVelocity.srv"
  "srv/GetPosition.srv"
  "srv/GetPositionVelocity.srv"
  "srv/HomeMotors.srv"
  "srv/MoveMotors.srv"
  "srv/SetAngle.srv"
  "srv/SetAngularVelocity.srv"
  "srv/SetControlMode.srv"
  "srv/SetMaxCurrent.srv"
  "srv/SetPosition.srv"
  "srv/SetPositionVelocity.srv"
)

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

ament_package()
