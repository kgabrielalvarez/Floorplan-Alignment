cmake_minimum_required(VERSION 3.8)
project(trajectory_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(trajectory_recorder src/trajectory_recorder.cpp)

ament_target_dependencies(trajectory_recorder rclcpp nav_msgs)

install(TARGETS
  trajectory_recorder
  DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS
  src/save_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()