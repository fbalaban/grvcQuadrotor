//------------------------------------------------------------------------------
// GRVC Quadrotor Control
//------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------
#include <grvc_quadrotor_control/model.h>

namespace grvc {
	
	//------------------------------------------------------------------------------------------------------------------
	Model::Model() {
		// Set pid gains and limits
		x_pid.Init(5,0,2,0.0,0.0,2,-2);
   		y_pid.Init(5,0,2,0.0,0.0,2,-2);
   		z_pid.Init(1.0,0,0.0,0.0,0.0,2,-2);
   		yaw_pid.Init(2.5,0.0,0.0,0.0,0.0,2.0,-2.0);
	}

	//------------------------------------------------------------------------------------------------------------------
	void Model::updateControlActions(common::Time _dt) {
		// Position PIDs
		Vec3 pos_error = cur_pos_ - pos_reference_;
		cmd_vel_.x = x_pid.Update(pos_error.x, _dt);
		cmd_vel_.y = y_pid.Update(pos_error.y, _dt);
		cmd_vel_.z = z_pid.Update(pos_error.z, _dt);

		// Yaw PID
		Vec3 yaw_error = cur_yaw_ - yaw_reference_;
		cmd_yaw_ = yaw_pid.Update(yaw_error, _dt);
	}

} // namespace grvc