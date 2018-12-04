cmake_minimum_required(VERSION 2.8.3)
project(spotless_mini_explorer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spotless_mini_explorer
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(frontier_explorer src/main.cpp src/pathPlanner.cpp)
target_link_libraries(frontier_explorer ${catkin_LIBRARIES})
