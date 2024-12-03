cmake_minimum_required(VERSION 3.0)
 
set(PROJECT arduino)
 
project(${PROJECT})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_MAIN}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT}
    read_serial.cpp
)
 
target_link_libraries(${PROJECT}
	${OpenCV_LIBS}
	
)
 
install(TARGETS ${PROJECT}
    DESTINATION bin
)

