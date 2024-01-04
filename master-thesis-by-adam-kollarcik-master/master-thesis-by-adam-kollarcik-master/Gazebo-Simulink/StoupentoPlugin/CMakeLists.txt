cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(gazebo REQUIRED)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_library(stoupentoPlugin SHARED stoupentoPlugin.cc)
target_link_libraries(stoupentoPlugin stoupentoPlugin_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(stoupentoPlugin stoupentoPlugin_msgs)