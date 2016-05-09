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

#include <geometry_msgs/Twist.h>
#include <grvc_quadrotor_hal/back_end/back_end_mavros.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <grvc_com/ros/ros_singleton.h>

using namespace std;

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	BackEndMavros::BackEndMavros(const char* _node_name, int _argc, char** _argv)
		: cur_task_state_(TaskState::finished)
	{
		// Init ros
		com::RosSingleton::init(_node_name, _argc, _argv);
		ros_handle_ = com::RosSingleton::get()->handle();
		// Init myself
		setDefaultParams();
		parseArguments(_argc, _argv);
		startRosCommunications();
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::goToWP(const Vec3&) {
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::takeOff(double) {
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::land() {
	}

	//------------------------------------------------------------------------------------------------------------------
	TaskState BackEndMavros::curTaskState() const {
		return TaskState::finished;
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::abortTask() {
	}

	//------------------------------------------------------------------------------------------------------------------
	Vec3 BackEndMavros::position() const {
		return Vec3::Zero();
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::startRosCommunications() {

		//Call to server of mavros to take off
		takeOffClient=n.serviceClient<mavros::CommandTOL>("/mavros/cmd/takeoff",0);
	}



}}	// namespace grvc::hal

#endif // GRVC_USE_ROS
