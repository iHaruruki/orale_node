cmake_minimum_required(VERSION 3.5)
project(my_serial_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(serial_node src/serial_node.cpp)
ament_target_dependencies(serial_node rclcpp geometry_msgs OpenCV)

install(TARGETS
  serial_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()