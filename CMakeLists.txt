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
project(grvcQuadrotor)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ---- Configurable options ----
# Pass this options to CMake in command line to select specific functionality for this project
option(GRVCQUAD_BUILD_TESTS "Enable testing and generate test projects" OFF)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	option(GRVCQUAD_USE_ROS, "Enable ros-specific code and interfaces" ON)
else()
	option(GRVCQUAD_USE_ROS, "Enable ros-specific code and interfaces" OFF)
endif()

# --- Compiler specific options ---
# Visual studio specifics
if(MSVC)
	add_definitions(/W4 /WX) # Enable high level warnings and treat them like errors
endif(MSVC)
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -Wall -Werror)
endif()

if(GRVC_USE_ROS)
	add_definitions(-DGRVC_USE_ROS)
endif(GRVC_USE_ROS)

# Included subprojects
add_subdirectory(uav)

if(GRVCQUAD_USE_ROS) # Only ros versions are available for this projects.
	add_subdirectory(hal)
	add_subdirectory(simulation)
endif(GRVCQUAD_USE_ROS)