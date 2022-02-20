cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package( OpenCV 4.5 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(main main.cpp gazebo_interface.cpp)
target_link_libraries(main ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} pthread)
