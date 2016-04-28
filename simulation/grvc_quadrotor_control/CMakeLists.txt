cmake_minimum_required(VERSION 2.8.3)
project(grvc_quadrotor_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  hector_gazebo_plugins
  hector_uav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} "include")

add_executable(grvc_quadrotor_control src/main.cpp)