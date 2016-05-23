//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor HAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef _GRVCQUADROTOR_HAL_COMMON_TYPES_H_
#define _GRVCQUADROTOR_HAL_COMMON_TYPES_H_

#include <Eigen/Core>
#include <cstdint>
#include <iostream>
#include <grvc_com/subscriber.h> // For deserialization of Vec3
#include <vector>

namespace grvc { namespace hal {
	
	typedef Eigen::Vector3d Vec3;
	typedef double Scalar;

	/// Define a path point to command uavs
	struct Waypoint {
		Vec3 pos;
		Scalar yaw;
	};

	typedef std::vector<Waypoint>	WaypointList;

	/// States in which a requested task can be during (or after) execution.
	enum class TaskState : uint8_t {
		finished = 0, ///< The task has finished with success.
		failed, ///< The task failed during execution.
		aborted, ///< The task was aborted from outside.
		running ///< The task is still running.
	};
	
}}	// namespace grvc::hal

inline std::ostream& operator<<(std::ostream& _os, const grvc::hal::Waypoint& _wp) {
	_os << '{' << _wp.pos << ',' << _wp.yaw << '}';
	return _os;
}

inline std::istream& operator>>(std::istream& _is, grvc::hal::Waypoint& _wp) {
	_is.get(); // Skip {
	_is >> _wp.pos;
	_is.get(); // Skip ,
	_is >> _wp.yaw;
	_is.get(); // Skip }
	return _is;
}

#endif // _GRVCQUADROTOR_HAL_COMMON_TYPES_H_