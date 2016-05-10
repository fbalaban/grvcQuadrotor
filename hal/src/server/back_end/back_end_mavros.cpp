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
	void BackEndMavros::takeOff(double altitude) {
		
		mavros_msgs::SetMode flightModeService;
		flightModeService.base_mode=0;
		flightModeService.custom_mode=FLIGHT_MODE_GUIDED;
		if(!flightModeClient.call(flightModeService))
			cur_task_state_=TaskState::aborted;

		mavros_msgs::CommandBool armedService;
		armedService.value=true;
		if(!armedClient.call(armedService))
			cur_task_state_=TaskState::aborted;

		mavros_msgs::CommandTOL takeOffService;
		takeOffService.altitude=altitude;
		if(!takeOffClient.call(takeOffService))
			cur_task_state_=TaskState::aborted;
		else
			cur_task_state_=TaskState::running;

		
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::land() {
		mavros_msgs::CommandTOL land_service_;
		land_service_.altitude=altitude;
		if(!land_client_.call(land_service_))
			cur_task_state_=TaskState::aborted;
		else
			cur_task_state_=TaskState::running;
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
	void BackEndMavros::startRosCommunications() {

		//Call to server of mavros to take off
		land_client_=ros_handle_.serviceClient<mavros::CommandTOL>("/mavros/cmd/land");
		take_off_Client_=ros_handle_.serviceClient<mavros::CommandTOL>("/mavros/cmd/takeoff");
		armed_client_=ros_handle_.serviceClient<mavros::CommandBool>("/mavros/cmd/arming");
		flight_mode_Client_=ros_handle_.serviceClient<mavros::SetMode>("/mavros/set_mode");

		ros_handle_.subscribe(position_full_topic.c_str(),1000, &BackEndMavros::positionCb,this);
		ros_handle_.subscribe(altitude_full_topic.c_str(),1000, &BackEndMavros::altitudeCb,this);
	}
	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::positionCb(const gemometry_msgs::PoseStamped::ConstPtr& _uav) {
		pos_x_=_uav.pose.position.x;
		pos_y_=_uav.pose.position.y;
		pos_z_=_uav.pose.position.z;
	}
	//------------------------------------------------------------------------------------------------------------------
	void BackEndMavros::altitudeCb(const std_msgs::Float64::ConstPtr& _altitude) {
		altitude_=_altitude.value;
	}



}}	// namespace grvc::hal

#endif // GRVC_USE_ROS
