cmake_minimum_required(VERSION 3.8)
project(human_detector)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(human_detector src/human_detector.cpp)

ament_target_dependencies(human_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS human_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

