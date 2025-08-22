cmake_minimum_required(VERSION 3.8)
project(orale_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(orale_serial_node src/read_serial.cpp)
ament_target_dependencies(orale_serial_node
  rclcpp
  std_msgs
)

add_executable(tatto_serial_node src/read_serial_9.cpp)
ament_target_dependencies(tatto_serial_node
  rclcpp
  std_msgs
)

add_executable(calibrator_node src/calibrator.cpp)
ament_target_dependencies(calibrator_node
  rclcpp
  std_msgs
)

install(TARGETS
  orale_serial_node
  tatto_serial_node
  calibrator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
