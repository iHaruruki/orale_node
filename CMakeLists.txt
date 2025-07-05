cmake_minimum_required(VERSION 3.8)
project(orale_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(orale_serial_node src/read_serial.cpp)
ament_target_dependencies(orale_serial_node
  rclcpp
  geometry_msgs
  OpenCV
)

install(TARGETS
  orale_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
