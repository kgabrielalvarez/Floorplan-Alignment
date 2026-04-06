cmake_minimum_required(VERSION 3.8)
project(reconstruct_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(reconstruct_3d 
  src/main.cpp
  src/reconstruct_3d.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(reconstruct_3d
  OpenCV
  rosbag2_cpp
  rclcpp
  nav_msgs
)

install(TARGETS
  reconstruct_3d
  DESTINATION lib/${PROJECT_NAME})

ament_package()