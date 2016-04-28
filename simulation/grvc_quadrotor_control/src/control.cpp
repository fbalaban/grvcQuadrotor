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
#include <functional>
#include <grvc_quadrotor_control/control.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string>

using namespace std;

namespace grvc {
	
	//------------------------------------------------------------------------------------------------------------------
	QuadrotorControl::QuadrotorControl(const char* _nodeName, int _argc, char** _argv)
		: ros_handle_(_nodeName)
	{
		// Init ros
		ros::init(_argc, _argv, _nodeName, ros::init_options::AnonymousName);
		// Init myself
		parseArguments(_argc, _argv);
		startRosCommunications();
		// Start running async
		publish_timer_ = ros_handle_.createTimer(ros::Duration(1/publish_rate_),
               [this](const ros::TimerEvent& _te) { publishCb(_te); });
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::run() {
		// Keep running asynchronously until some event cancels execution
		ros::AsyncSpinner spinner(0);
		spinner.start();
		ros::waitForShutdown();
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::setDefaultParams() {
		gazebo_ns_ = "gazebo_ns_";
		cmd_vel_topic_ = "cmd_vel";
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::parseArguments(int _argc, char** _argv) {
		const string gazebo_ns_arg = "-gazebo_ns=";
		const string cmd_vel_topic_arg = "-cmd_vel_topic=";

		for(int i = 0; i < _argc; ++i) {
			string arg = _argv[i];
			if(parseArg(arg, gazebo_ns_arg, gazebo_ns_))
				break;
			if(parseArg(arg, cmd_vel_topic_arg, cmd_vel_topic_))
				break;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool QuadrotorControl::parseArg(const string& _arg, const string& _label, string& _dst) {
		if(_arg.substr(0, _label.size()) == _label) {
			_dst = _arg.substr(_label.size());
			return true;
		}
		return false;
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::startRosCommunications() {
		// Topic to set velocity references for gazebo plugin
		auto cmd_vel_full_topic = gazebo_ns_ + "/" + cmd_vel_topic_;
		cmd_vel_pub_ = ros_handle_.advertise<geometry_msgs::Twist>(cmd_vel_full_topic.c_str(), 0);
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::publishCb(const ros::TimerEvent&) {
		geometry_msgs::Twist t;
		t.linear.x = 0.0;
		t.linear.y = 0.0;
		t.linear.z = 0.0;
		t.angular.x = 0.0;
		t.angular.y = 0.0;
		t.angular.z = 1.0;
		cmd_vel_pub_.publish(t);
	}

} // namespace grvc