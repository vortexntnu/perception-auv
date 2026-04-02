cmake_minimum_required(VERSION 3.8)
project(perception_setup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# image_undistort composable node
add_library(image_undistort_component SHARED
  src/image_undistort.cpp
)

target_include_directories(image_undistort_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(image_undistort_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(image_undistort_component yaml-cpp)

rclcpp_components_register_node(
  image_undistort_component
  PLUGIN "perception_setup::ImageUndistort"
  EXECUTABLE image_undistort_node
)

install(TARGETS image_undistort_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Python scripts (kept for reference)
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
