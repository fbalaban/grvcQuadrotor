//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor HAL sample
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
#include <cstdint>
#include <thread>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

geometry_msgs::Point pos;

void odometryCb(const nav_msgs::Odometry::ConstPtr& _odom) {
		// -- Update model controller --
		pos = _odom->pose.pose.position;
	}

int main(int _argc, char** _argv) {
	ros::init(_argc, _argv, "sample", ros::init_options::AnonymousName);
	ros::NodeHandle* ros_handle = new ros::NodeHandle("sample");
	auto spin_thread = std::thread([](){ ros::spin(); });

	// Topic to set velocity references for gazebo plugin
	auto cmd_vel_pub = ros_handle->advertise<geometry_msgs::Twist>("/quad1/cmd_vel", 0);

	// Suscribe to odometry messages from gazebo
	pos.x = 0.0;
	pos.y = 0.0;
	pos.z = 0.0;
	ros_handle->subscribe("/quad1/ground_truth/state", 1000, &odometryCb);

	geometry_msgs::Twist goalSpd;
	goalSpd.angular.x = 0.0;
	goalSpd.angular.y = 0.0;
	goalSpd.angular.z = 0.0;
	goalSpd.linear.x = 0.0;
	goalSpd.linear.y = 0.0;
	goalSpd.linear.z = 0.0;
	
	while(pos.z < 2.0) {
		goalSpd.linear.z = 1.0;
		cmd_vel_pub.publish(goalSpd);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	while(pos.z > 0.5) {
		goalSpd.linear.z = -1.0;
		cmd_vel_pub.publish(goalSpd);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;
}