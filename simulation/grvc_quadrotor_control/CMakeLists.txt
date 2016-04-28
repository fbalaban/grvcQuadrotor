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

# Compiler config
add_definitions(-std=c++11 -Wall -Werror)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} "include")

# ---- Clasify sources according to folder structure ----
# This macro is a modified version of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir dirLabel)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child} ${dirLabel}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${dirLabel})
			source_group(${groupname} FILES
				${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

file(GLOB_RECURSE CONTROL_SOURCE_FILES "src/*.cpp")
GroupSources(src control)
add_executable(grvc_quadrotor_control ${CONTROL_SOURCE_FILES})
target_link_libraries(grvc_quadrotor_control LINK_PUBLIC ${catkin_LIBRARIES})