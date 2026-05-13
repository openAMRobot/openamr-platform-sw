cmake_minimum_required(VERSION 3.8)
project(openamrobot_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(detected_dock_pose_publisher src/detected_dock_pose_publisher.cpp)
ament_target_dependencies(detected_dock_pose_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS detected_dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dock_trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
