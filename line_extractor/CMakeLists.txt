cmake_minimum_required(VERSION 3.8)
project(line_extractor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(floorplan_alignment_interfaces REQUIRED)

add_executable(line_extractor src/line_extractor.cpp)
ament_target_dependencies(line_extractor 
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  rosbag2_cpp
  floorplan_alignment_interfaces
)

install(TARGETS
  line_extractor
  DESTINATION lib/${PROJECT_NAME})

ament_package()