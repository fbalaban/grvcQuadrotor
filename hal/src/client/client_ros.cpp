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

#include <hal_client/client_ros.h>
#include <grvc_quadrotor_hal/command.h>

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	ClientROS::ClientROS(const char* _node_name, int _argc, char** _argv) {
		ros::init(_argc, _argv, _node_name, ros::init_options::AnonymousName);
		ros_handle_ = new ros::NodeHandle(_node_name);
		// Init myself
		setDefaultParams();
		parseArguments(_argc, _argv);
		startRosCommunications();
	}

	//------------------------------------------------------------------------------------------------------------------
	void ClientROS::goToWP(const Vec3& _pos) {
		grvc_quadrotor_hal::command cmd;
		cmd.command = "GoToWP";
		cmd.pos.position.x = _pos.x();
		cmd.pos.position.y = _pos.y();
		cmd.pos.position.z = _pos.z();
		cmd_pub_.publish(cmd);
	}
	
}}	// namespace grvc

#endif // GRVC_USE_ROS