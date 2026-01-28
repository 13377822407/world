cmake_minimum_required(VERSION 3.10)
project(world)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_publisher src/odom_publisher.cpp)
add_executable(imu_publisher src/imu_publisher.cpp)
add_executable(keyboard_teleop src/keyboard_teleop.cpp)

ament_target_dependencies(odom_publisher rclcpp geometry_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs tf2 tf2_ros)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs nav_msgs)

install(TARGETS
  odom_publisher
  imu_publisher
  keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()