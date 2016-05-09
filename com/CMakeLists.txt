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
project(grvc_quadrotor_com)

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
	find_package(catkin REQUIRED COMPONENTS 
		roscpp
		std_msgs)

	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES grvc_com
		CATKIN_DEPENDS roscpp)

	include_directories(${catkin_INCLUDE_DIRS})
endif(GRVCQUAD_USE_ROS)

# --- external dependencies ---
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake/modules")
FIND_PACKAGE(Eigen3 REQUIRED)

# --- communications library ---
file(GLOB_RECURSE COM_SOURCE_FILES "src/*.cpp" "include/grvc_com/*.h")
GroupSources(src grvc_com)
GroupSources(include/grvc_com grvc_com)
add_library(grvc_com STATIC ${COM_SOURCE_FILES})
target_include_directories(grvc_com PUBLIC "include"  ${EIGEN3_INCLUDE_DIR})
set_target_properties(grvc_com PROPERTIES FOLDER "com")
if(GRVCQUAD_USE_ROS)
	target_link_libraries(grvc_com LINK_PUBLIC ${catkin_LIBRARIES})
endif(GRVCQUAD_USE_ROS)

# --- Test projects
if(GRVCQUAD_BUILD_TESTS)
	add_subdirectory(test)
endif(GRVCQUAD_BUILD_TESTS)