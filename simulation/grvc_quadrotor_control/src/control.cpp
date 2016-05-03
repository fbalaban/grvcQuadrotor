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
#include <geometry_msgs/Twist.h>
#include <grvc_quadrotor_control/control.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>

using namespace std;

namespace grvc {
	
	//------------------------------------------------------------------------------------------------------------------
	QuadrotorControl::QuadrotorControl(const char* _nodeName, int _argc, char** _argv)
	{
		// Init ros
		ros::init(_argc, _argv, _nodeName, ros::init_options::AnonymousName);
		ros_handle_ = new ros::NodeHandle(_nodeName);
		// Init myself
		setDefaultParams();
		parseArguments(_argc, _argv);
		startRosCommunications();
		// Start running async
		publish_timer_ = ros_handle_->createTimer(ros::Duration(1/publish_rate_),
			[this](const ros::TimerEvent& _te) { publishCb(_te); });
		update_timer_ = ros_handle_->createTimer(ros::Duration(1/update_rate_),
			[this](const ros::TimerEvent& _te) { updateCb(_te); });

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
		odometry_topic_ = "ground_truth/state";
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::parseArguments(int _argc, char** _argv) {
		const string gazebo_ns_arg = "-gazebo_ns=";
		const string cmd_vel_topic_arg = "-cmd_vel_topic=";
		const string odometry_topic_arg = "-odometry_topic=";

		for(int i = 0; i < _argc; ++i) {
			string arg = _argv[i];
			if(parseArg(arg, gazebo_ns_arg, gazebo_ns_))
				break;
			if(parseArg(arg, cmd_vel_topic_arg, cmd_vel_topic_))
				break;
			if(parseArg(arg, odometry_topic_arg, odometry_topic_))
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
		cmd_vel_pub_ = ros_handle_->advertise<geometry_msgs::Twist>(cmd_vel_full_topic.c_str(), 0);

		// Suscribe to odometry messages from gazebo
		auto odometry_full_topic = gazebo_ns_ + "/" + odometry_topic_;
		odometry_sub_ = ros_handle_->subscribe(odometry_full_topic.c_str(),
               1000, &QuadrotorControl::odometryCb,
                        this);
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::odometryCb(const nav_msgs::Odometry::ConstPtr& odom) {
		// -- Update model controller --
		state_controller_.setPos(odom->pose.pose.position);
		// Rotation
		tf::Quaternion orientation(odom->pose.pose.orientation.x,
			odom->pose.pose.orientation.y,
			odom->pose.pose.orientation.z,
			odom->pose.pose.orientation.w);
		double roll, pitch, yaw;
		trix3x3(orientation).getRPY(roll, pitch, yaw);
		state_controller_.setYaw(yaw);
		// -- Init states if necessary
		if(!cur_state_) {
			// Use current state estimation to decide initial state
			
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::publishCb(const ros::TimerEvent&) {
		// We can only publish control information when thre is a State available,
		// generating control references for the PIDs
		assert(cur_state_);
		geometry_msgs::Twist twist;
		twist.linear = state_controller_.velocity();
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = state_controller_.yaw();
		cmd_vel_pub_.publish(t);
	}

	//------------------------------------------------------------------------------------------------------------------
	void QuadrotorControl::updateCb(const ros::TimerEvent&) {
		assert(cur_state_);
		cur_state_->setModelEstimation(state_controller_.yaw(), state_controller_.pos());
		// Update state
		State* next_state = cur_state_->nextState();
		if(next_state != cur_state_) {
			delete cur_state_;
			cur_state_ = next_state;
		}
		// Update control references
		state_controller_.setReferencePos(cur_state_->posReference());
		state_controller_.setReferenceYaw(cur_state_->yawReference());
	}

} // namespace grvc