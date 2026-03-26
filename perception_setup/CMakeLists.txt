cmake_minimum_required(VERSION 3.8)
project(perception_setup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/camera_info_publisher.py
  scripts/image_undistort.py
  scripts/image_crop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
