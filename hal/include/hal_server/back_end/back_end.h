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
#ifndef _GRVCQUADROTOR_HALSERVER_BACKEND_BACKEND_H_
#define _GRVCQUADROTOR_HALSERVER_BACKEND_BACKEND_H_

#include <hal_common/types.h>

namespace grvc { namespace hal {
	
	/// Common interface for back end implementations of hal
	class BackEnd {
	public:
		/// Go to the specified waypoint, following a straight line.
		/// \param _wp goal waypoint.
		virtual void		goToWP			(const Vec3& _wp) = 0;
		/// Perform a take off maneuver
		/// \param _height targer height that must be reached to consider the take off complete.
		virtual void		takeOff			(double _height) = 0;
		/// Land on the current position.
		virtual void		land			() = 0;
		/// Retrieve the state of the last task requested
		virtual TaskState	curTaskState	() const = 0;
		/// Cancel execution of the current task
		virtual void		abortTask		() = 0;
		/// Latest position estimation of the robot
		virtual Vec3		position		() const = 0;

		virtual ~BackEnd() = default; // Ensure proper destructor calling for derived classes

		/// Keep running
		/// \return \c false if the service has stopped. \c true otherwise.
		virtual bool update() = 0;
		/// \brief Create an adequate BackEnd depending on current platform and command arguments.
		/// \param _argc number of arguments in _argv
		/// \param _argv command line arguments passed to the program. This arguments will be parsed
		/// and used to select the best fitting implementation of BackEnd from those available in the
		/// current platform.
		/// \return the newly created BackEnd. Whoever calls this method, is responsible for eventually
		/// destroying the BackEnd.
		static BackEnd* createBackEnd(int _argc, char** _argv);
	};
	
}}	// namespace grvc::hal

#endif // _GRVCQUADROTOR_HALSERVER_BACKEND_BACKEND_H_