cmake_minimum_required(VERSION 3.5)
project(dh116_r000_a1)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
