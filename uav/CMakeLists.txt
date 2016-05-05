################################################################################
# GRVC Quadrotor UAV
################################################################################
# The MIT License (MIT)
# 
# Copyright (c) 2016 GRVC University of Seville
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
################################################################################
cmake_minimum_required (VERSION 2.6)
project(grvc_quadrotor_uav)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
option(GRVCQUAD_BUILD_TESTS "Enable testing and generate test projects" ON)
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	option(GRVCQUAD_USE_ROS "Enable ros-specific code and interfaces" ON)
else()
	option(GRVCQUAD_USE_ROS "Enable ros-specific code and interfaces" OFF)
endif()

# --- Compiler specific options ---
if(MSVC)
	add_definitions(/W4 /WX) # Enable high level warnings and treat them like errors
endif(MSVC)
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -Wall -Werror)
endif()

if(GRVCQUAD_USE_ROS)
	add_definitions(-DGRVC_USE_ROS)
endif(GRVCQUAD_USE_ROS)

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

# --- Catkin specific configuration for ros ---
if(GRVCQUAD_USE_ROS)
	find_package(catkin REQUIRED COMPONENTS roscpp)

	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES uav_server uav_client
		CATKIN_DEPENDS roscpp)

	include_directories(${catkin_INCLUDE_DIRS})
endif(GRVCQUAD_USE_ROS)

# --- External dependencies ---
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

# --- uav server library. Extend this library to create more advanced servers ---
set(SERVER_DIR src/server)
file(GLOB_RECURSE UAVSERVER_SOURCE_FILES "${SERVER_DIR}/*.cpp")
GroupSources(src/server server)
GroupSources(include/uav_server server)
add_library(uav_server STATIC ${UAVSERVER_SOURCE_FILES})
target_include_directories(uav_server PUBLIC "include")
set_target_properties(uav_server PROPERTIES FOLDER "uav")

# --- Main executable, simple node running the uav service ---
set(NODE_DIR src/node)
file(GLOB_RECURSE UAVNODE_SOURCE_FILES "${NODE_DIR}/*.cpp")
GroupSources(src/node node)
add_executable(uav_node ${UAVNODE_SOURCE_FILES})
target_link_libraries(uav_node LINK_PUBLIC uav_server)
set_target_properties(uav_node PROPERTIES FOLDER "uav")

# --- Client library. Communicates with uav server ---
set(CLIENT_DIR src/client)
file(GLOB_RECURSE UAVCLIENT_SOURCE_FILES "${CLIENT_DIR}/*.cpp")
GroupSources(src/client client)
GroupSources(include/uav_client client)
add_library(uav_client STATIC ${UAVCLIENT_SOURCE_FILES})
target_include_directories(uav_client PUBLIC "include")
set_target_properties(uav_client PROPERTIES FOLDER "uav")