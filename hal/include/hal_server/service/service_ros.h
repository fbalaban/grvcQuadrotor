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
#ifndef _GRVCQUADROTOR_HAL_SERVER_SERVICE_SERVICEROS_H_
#define _GRVCQUADROTOR_HAL_SERVER_SERVICE_SERVICEROS_H_

#ifndef GRVC_USE_ROS

#include <hal_msgs/command.h>
#include <ros/ros.h>
#include "service.h"

namespace grvc { namespace hal {
	
	/// Common interface for classes advertising the hal service over different channels

	/// Whoever uses this class is responsible for registering callbacks for every event they want to be notified of.
	/// New callbacks override older ones, so only the most recent one will be invoked for any event. If you require
	/// Multiple callbacks to be invoked, you will need to do the dispatching for yourself.
	class ServiceROS {
	public:
		ServiceROS(const char* _nodeName, int _argc, char** _argv);

		// Set callbacks
		void onGoToWP	(GoToWpCb) override;
		void onTakeOff	(TakeOffCb) override;
		void onLand		(LandCb) override;
		void onAbort	(AbortCb) override;

		// Direct interface
		void publishPosition	(const Vec3&) override;
		void publishTaskState	(TaskState) override;

		/// Keep running
		/// \return \c false if the service has stopped. \c true otherwise.
		bool update() override;

	private:
		void setDefaultParams();
		void parseArguments(int _argc, char** _argv);
		bool parseArg(const std::string& _arg, const std::string& _label, std::string& _dst);
		void startRosCommunications();

		void onCmdCallBack(const hal_msgs::command::ConstPtr& _cmd);

	private:
		GoToWpCb go_to_wp_cb_;
		TakeOffCb take_off_cb_;
		LandCb land_cb_;
		AbortCb abort_cb_;

		// Ros communications
		ros::NodeHandle* ros_handle_;

		ros::Publisher odometry_pub_;
		ros::Publisher cmd_state_pub_;
		ros::Subscriber cmd_sub_;

		// Ros communication topics
		std::string odometry_topic_;
		std::string cmd_state_topic_;
		std::string cmd_topic_;
		std::string hal_ns_;
	};
	
}}	// namespace grvc::hal

#endif // GRVC_USE_ROS

#endif // _GRVCQUADROTOR_HAL_SERVER_SERVICE_SERVICEROS_H_