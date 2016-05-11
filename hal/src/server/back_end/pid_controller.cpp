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
#ifdef GRVC_USE_ROS
#include <grvc_quadrotor_hal/back_end/pid_controller.h>
#include <ros/ros.h>

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	PidController::PidController() {
		// Set pid gains and limits
		pid_x_.Init(5,0,2,0.0,0.0,2,-2);
		pid_y_.Init(5,0,2,0.0,0.0,2,-2);
		pid_z_.Init(3.0,1,0.0,0.0,0.0,2,-2);
		pid_yaw_.Init(2.5,0.0,0.0,0.0,0.0,2.0,-2.0);
	}

	//------------------------------------------------------------------------------------------------------------------
	void PidController::setReferencePos(const Vec3& _pos) {
		pos_reference_ = _pos;
		pid_x_.Reset();
		pid_y_.Reset();
		pid_z_.Reset();
		has_pos_ref_ = true;
	}

	//------------------------------------------------------------------------------------------------------------------
	void PidController::setReferenceYaw(double _yaw) {
		yaw_reference_ = _yaw;
		pid_yaw_.Reset();
		has_yaw_ref_ = true;
	}

	//------------------------------------------------------------------------------------------------------------------
	void PidController::updateControlActions(gazebo::common::Time _dt) {
		// Position PIDs
		if(has_pos_ && has_yaw_ref_) {
			Vec3 pos_error = cur_pos_ - pos_reference_;
			vel_action_.x() = pid_x_.Update(pos_error.x(), _dt);
			vel_action_.y() = pid_y_.Update(pos_error.y(), _dt);
			vel_action_.z() = pid_z_.Update(pos_error.z(), _dt);
		}

		// Yaw PID
		if(has_yaw_ && has_yaw_ref_) {
			double yaw_error = cur_yaw_ - yaw_reference_;
			yaw_action_ = pid_yaw_.Update(yaw_error, _dt);
		}
	}

}} // namespace grvc

#endif // GRVC_USE_ROS