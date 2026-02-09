cmake_minimum_required(VERSION 3.5)
project(DH116-L000-A1)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
