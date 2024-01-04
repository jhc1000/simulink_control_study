cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(encoderPlug SHARED encoderPlug.cc)
target_link_libraries(encoderPlug ${GAZEBO_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")