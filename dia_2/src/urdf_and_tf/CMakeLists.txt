cmake_minimum_required(VERSION 3.7)
project(urdf_and_tf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
