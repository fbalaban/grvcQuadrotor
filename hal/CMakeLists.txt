################################################################################
# GRVC Quadrotor HAL
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
project(grvcQuadrotorHal)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
option(GRVCQUAD_BUILD_TESTS "Enable testing and generate test projects" OFF)

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
		LIBRARIES hal_server hal_client
		CATKIN_DEPENDS roscpp)

	include_directories(${catkin_INCLUDE_DIRS})
endif(GRVCQUAD_USE_ROS)

# --- External dependencies ---
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

SET(COMMON_INCLUDES "include/hal_common/*.h")

# --- uav server library. Extend this library to create more advanced servers ---
set(SERVER_DIR src/server)
file(GLOB_RECURSE HALSERVER_SOURCE_FILES "${SERVER_DIR}/*.cpp" "include/hal_server/*.h" ${COMMON_INCLUDES})
GroupSources(src/server server)
GroupSources(include/hal_server server)
GroupSources(include/hal_common common)
add_library(hal_server STATIC ${HALSERVER_SOURCE_FILES})
target_include_directories(hal_server PUBLIC "include")
set_target_properties(hal_server PROPERTIES FOLDER "hal")

# --- Main executable, simple node running the uav service ---
set(NODE_DIR src/node)
file(GLOB_RECURSE HALNODE_SOURCE_FILES "${NODE_DIR}/*.cpp" ${COMMON_INCLUDES})
GroupSources(src/node node)
add_executable(hal_node ${HALNODE_SOURCE_FILES})
target_include_directories(hal_node PUBLIC "include")
target_link_libraries(hal_node LINK_PUBLIC uav_server)
set_target_properties(hal_node PROPERTIES FOLDER "hal")

# --- Client library. Communicates with uav server ---
set(CLIENT_DIR src/client)
file(GLOB_RECURSE HALCLIENT_SOURCE_FILES "${CLIENT_DIR}/*.cpp" "include/hal_client/*.h" ${COMMON_INCLUDES})
GroupSources(src/client client)
GroupSources(include/hal_client client)
GroupSources(include/hal_common common)
add_library(hal_client STATIC ${HALCLIENT_SOURCE_FILES})
target_include_directories(hal_client PUBLIC "include")
set_target_properties(hal_client PROPERTIES FOLDER "hal")

# --- Test projects
if(GRVCQUAD_BUILD_TESTS)
	add_subdirectory(test)
ENDIF(GRVCQUAD_BUILD_TESTS)