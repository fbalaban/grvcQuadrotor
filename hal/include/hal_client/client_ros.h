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
#ifndef _GRVCQUADROTOR_HAL_CLIENT_CLIENTROS_H_
#define _GRVCQUADROTOR_HAL_CLIENT_CLIENTROS_H_

#ifdef GRVC_USE_ROS

#include "client.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace grvc { namespace hal {
	/// \brief Cient interface for other modules to communicate with a hal (or hal-derived) node

	/// Different implementations of this interface are responsible of communications with an analogous hal service.
	/// For example, hal::ClientRos (derived from hal::Client), implements communications using topics and services
	/// and expects a hal::SericeRos to be running in some other node in the network. Equivalent, hal::ClientHttp
	/// would require hal::ServiceHttp to be running somewhere, with an accessible IP address.
	class ClientROS : public Client {
	public:
		ClientROS(const char* _node_name, int _argc, char** _argv);

		void		goToWP			(const Vec3& _wp) override;
		void		takeOff			(double _height) override;
		void		land			() override;
		TaskState	curTaskState	() const override;
		void		abortTask		() override;
		Vec3		position		() const override;

	private:
		typedef geometry_msgs::Pose PosMsg;

	private:
		void setDefaultParams();
		void parseArguments(int _argc, char** _argv);
		bool parseArg(const std::string& _arg, const std::string& _label, std::string& _dst);
		void startRosCommunications();

		// Callbacks
		void onOdometry(const nav_msgs::Odometry::ConstPtr& _odom);
		void onTaskProgress(const std_msgs::String::ConstPtr& _state);

	private:
		// Cache state information
		Vec3 last_position_;
		TaskState last_state_;

		// Ros communication channels
		ros::NodeHandle* ros_handle_;

		ros::Publisher	cmd_pub_;
		ros::Subscriber	cmd_state_sub_;
   		ros::Subscriber odometry_sub_;

		// Ros communication topics
		std::string odometry_topic_;
		std::string cmd_state_topic_;
		std::string cmd_topic_;
		std::string gazebo_ns_;
	};
	
}}	// namespace grvc::hal

#endif // GRVC_USE_ROS

#endif // _GRVCQUADROTOR_HAL_CLIENT_CLIENTROS_H_