################################################################################
# GRVC Quadrotor
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
project(grvc_quadrotor_gazebo_sample)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	option(GRVCQUAD_USE_ROS "Enable ros-specific code and interfaces" ON)
else()
	option(GRVCQUAD_USE_ROS "Enable ros-specific code and interfaces" OFF)
endif()

# --- Compiler specific options ---
# Visual studio specifics
if(MSVC)
	add_definitions(/W4 /WX) # Enable high level warnings and treat them like errors
endif(MSVC)
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -Wall -Werror)
endif()

if(GRVCQUAD_USE_ROS)
	find_package(catkin REQUIRED COMPONENTS 
		roscpp
		std_msgs
		gazebo_ros
		geometry_msgs)

	catkin_package(
		CATKIN_DEPENDS roscpp
		DEPENDS gazebo_ros)

	find_package(gazebo REQUIRED)

	include_directories(${catkin_INCLUDE_DIRS})
	add_definitions(-DGRVC_USE_ROS)

	add_executable(grvc_quad_gazebo_sample src/main.cpp)

	target_include_directories(grvc_quad_gazebo_sample PRIVATE ${GAZEBO_INCLUDE_DIRS})
	target_link_libraries(grvc_quad_gazebo_sample LINK_PRIVATE ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
endif(GRVCQUAD_USE_ROS)