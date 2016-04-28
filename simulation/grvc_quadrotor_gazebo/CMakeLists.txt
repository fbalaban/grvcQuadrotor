cmake_minimum_required(VERSION 2.8.3)
project(grvc_quadrotor_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_ros
  hector_gazebo_plugins
  hector_uav_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS gazebo_ros hector_gazebo_plugins
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(grvc_quadrotor_gazebo src/quadrotor_simple_controller.cpp)