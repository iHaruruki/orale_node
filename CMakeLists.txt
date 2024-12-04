cmake_minimum_required(VERSION 3.5)
project(orale_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(orale_node src/read_serial.cpp)
ament_target_dependencies(orale_node rclcpp geometry_msgs OpenCV)

install(TARGETS
  orale_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()