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
#ifndef _GRVCQUADROTOR_SIMULATION_GRVCQUADROTORCONTROL_CONTROL_H_
#define _GRVCQUADROTOR_SIMULATION_GRVCQUADROTORCONTROL_CONTROL_H_

#include <ros/ros.h>
#include <gazebo/common/PID.hh>
#include <string>
#include "model.h"

namespace grvc {

	class State;

	class QuadrotorControl {
	public:
		QuadrotorControl(const char* _nodeName, int _argc, char** _argv);
		void run();

	private:
		void setDefaultParams();
		void parseArguments(int _argc, char** _argv);
		bool parseArg(const std::string& _arg, const std::string& _label, std::string& _dst);
		void startRosCommunications();

		// Call backs
		void publishCb(const ros::TimerEvent& _te);
		void updateCb(const ros::TimerEvent& _te);

	private:
		ros::NodeHandle* ros_handle_;
		ros::Timer publish_timer_;
		ros::Timer update_timer_;
		float publish_rate_ = 100.f;
		float update_rate_ = 100.f;

		// Control
		Model state_controller_;

		// State machine
		State* cur_state_ = nullptr;
		double take_off_z_ = 1.0;

		// Ros communication channels
		ros::Publisher	cmd_vel_pub_;
		ros::Subscriber	control_refs_sub_;
   		ros::Subscriber odometry_sub_;

		// Ros communication topics
		std::string odometry_topic_;
		std::string cmd_vel_topic_;
		std::string gazebo_ns_;
	};
} // namespace grvc
#endif // _GRVCQUADROTOR_SIMULATION_GRVCQUADROTORCONTROL_CONTROL_H_