cmake_minimum_required(VERSION 3.8)
project(perception_setup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
