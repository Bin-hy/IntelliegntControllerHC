cmake_minimum_required(VERSION 3.10.2)
project(duco_support)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf collision DESTINATION share/${PROJECT_NAME})
