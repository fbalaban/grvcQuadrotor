//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor COM
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
#ifndef _GRVCQUADROTOR_COM_ROS_SUBSCRIBERBACKENDROS_H_
#define _GRVCQUADROTOR_COM_ROS_SUBSCRIBERBACKENDROS_H_

#ifdef GRVC_USE_ROS

#include "../subscriber_back_end.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>

namespace grvc {
	namespace com {

		class SubscriberBackEndROS : public SubscriberBackEnd {
		public:
			SubscriberBackEndROS(const char* _node_name, const char* _topic, int _argc, char** _argv);
			void onMessage(MsgCallBack _cb) override;
			void onNotification(NotifyCallBack _cb) override;

		private:
			ros::Subscriber ros_sub_;
			void onRosMsg(const std_msgs::String::ConstPtr& _s);
			void onRosNotification(const std_msgs::Int32::ConstPtr& _n);

			bool has_subscriber_ = false;
			std::string topic_;
		};

	}
} // namespace grvc::com

#endif // GRVC_USE_ROS

#endif // _GRVCQUADROTOR_COM_ROS_SUBSCRIBERBACKENDROS_H_