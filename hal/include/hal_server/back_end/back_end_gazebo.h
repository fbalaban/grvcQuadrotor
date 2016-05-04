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
#ifndef _GRVCQUADROTOR_HALSERVER_BACKEND_BACKENDGAZEBO_H_
#define _GRVCQUADROTOR_HALSERVER_BACKEND_BACKENDGAZEBO_H_

#ifdef GRVC_USE_ROS

#include "back_end.h"
#include "pid_controller.h"

namespace grvc { namespace hal {
	
	/// Common interface for back end implementations of hal
	class BackEndGazebo : public BackEnd {
	public:
		BackEndGazebo(const char* _nodeName, int _argc, char** _argv);

		/// Go to the specified waypoint, following a straight line.
		/// \param _wp goal waypoint.
		void		goToWP			(const Vec3& _wp) override;
		/// Perform a take off maneuver
		/// \param _height targer height that must be reached to consider the take off complete.
		void		takeOff			(double _height) override;
		/// Land on the current position.
		void		land			() override;
		/// Retrieve the state of the last task requested
		TaskState	curTaskState	() const override;
		/// Cancel execution of the current task
		void		abortTask		() override;
		/// Latest position estimation of the robot
		Vec3		position		() const override;

	private:
		void run();
		void setDefaultParams();
		void parseArguments(int _argc, char** _argv);
		bool parseArg(const std::string& _arg, const std::string& _label, std::string& _dst);
		void startRosCommunications();
		void updateCb(const ros::TimerEvent& _te);

	private:
		TaskState cur_task_state_;

		ros::NodeHandle* ros_handle_;
		ros::Timer publish_timer_;
		ros::Timer update_timer_;
		float publish_rate_ = 100.f;
		float update_rate_ = 100.f;

		// Control
		PidController state_controller_;
		bool has_odometry_;
		bool has_references_;

		// Ros communication
		ros::Publisher	cmd_vel_pub_;
   		ros::Subscriber odometry_sub_;
		std::string odometry_topic_;
		std::string cmd_vel_topic_;
		std::string gazebo_ns_;
	};
	
}}	// namespace grvc::hal

#endif // GRVC_USE_ROS

#endif // _GRVCQUADROTOR_HALSERVER_BACKEND_BACKENDGAZEBO_H_