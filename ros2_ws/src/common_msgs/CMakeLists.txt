cmake_minimum_required(VERSION 3.10)
project(common_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GloveJoints.msg"
  "msg/HandCmd.msg"
  "msg/TaskDeviceCheck.msg"
  "msg/TaskStep.msg"
  "msg/TaskConfig.msg"
  "msg/DeviceStatus.msg"
  "msg/CollisionStatus.msg"
  "msg/DepthMeasurement.msg"
  "srv/SetCurrentUser.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES std_msgs action_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
