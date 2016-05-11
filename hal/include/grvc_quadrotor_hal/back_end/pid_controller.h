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
#include <grvc_quadrotor_hal/types.h>

namespace grvc { namespace hal {

	class PidController {
	public:
		PidController();

		double 		yawSpd	() const { return yaw_action_; }
		const Vec3& velocity() const { return vel_action_; }

		const Vec3& pos() const { return cur_pos_; }
		void setPos(const Vec3& _pos)	{ cur_pos_ = _pos; has_pos_ = true;}
		double yaw() const { return cur_yaw_; }
		void setYaw(double _yaw)		{ cur_yaw_ = _yaw; has_yaw_ = true; }

		void setReferencePos(const Vec3& _pos);
		const Vec3& posReference() const { return pos_reference_; }
		void setReferenceYaw(double _yaw);
		double yawReference() const { return yaw_reference_; }

		void updateControlActions(gazebo::common::Time _dt);

	private:
		// Control
		gazebo::common::PID pid_x_, pid_y_, pid_z_, pid_yaw_;

		// Internal state
		double	cur_yaw_, yaw_reference_;
		Vec3	cur_pos_, pos_reference_;
		bool has_yaw_ = false, has_yaw_ref_ = false;
		bool has_pos_ = false, has_pos_ref_ = false;

		// Control actions
		double	yaw_action_;
		Vec3  vel_action_;
	};
}} // namespace grvc::hal

#endif // GRVC_USE_ROS

#endif // _GRVCQUADROTOR_HALSERVER_BACKEND_PIDCONTROLLER_H_