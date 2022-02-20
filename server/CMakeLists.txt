cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(main main.cpp server.cpp tracker.cpp movement_primitives.cpp vision.cpp positioning.cpp)
target_link_libraries(main  ${OpenCV_LIBS} pthread)
