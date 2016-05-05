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
#ifndef _GRVCQUADROTOR_HALSERVER_BACKEND_PIDCONTROLLER_H_
#define _GRVCQUADROTOR_HALSERVER_BACKEND_PIDCONTROLLER_H_

#ifdef GRVC_USE_ROS

#include <gazebo/common/PID.hh>
#include <hal_common/types.h>

namespace grvc {

	class PidController {
	public:
		Model();

		double 		yawSpd	() const { return yaw_action_; }
		const Vec3& velocity() const { return spd_action_; }

		const Vec3& pos() const { return cur_pos_; }
		void setPos(const Vec3& _pos)	{ cur_pos_ = _pos; }
		const Vec3& yaw() const { return cur_yaw_; }
		void setYaw(double _yaw)		{ cur_pos_ = _pos; }

		void setReferencePos(const Vec3& _pos)	{ pos_reference_ = _pos; }
		void setReferenceYaw(double _yaw) 		{ yaw_reference_ = _yaw; }

		void updateControlActions(common::Time _dt);

	private:
		// Control
		gazebo::common::PID pid_x, pid_y, pid_z, pid_yaw;

		// Internal state
		double	cur_yaw_, yaw_reference_;
		Vec3	cur_pos_, pos_reference_;

		// Control actions
		double	yaw_action_;
		double  spd_action_;
	};
} // namespace grvc

#endif // GRVC_USE_ROS

#endif // _GRVCQUADROTOR_HALSERVER_BACKEND_PIDCONTROLLER_H_