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
#ifdef GRVC_USE_ROS

#include <cassert>
#include <grvc_com/ros/subscriber_back_end_ros.h>
#include <grvc_com/ros/ros_singleton.h>
#include <sstream>

namespace grvc { namespace com {

	//------------------------------------------------------------------------------------------------------------------
	SubscriberBackEndROS::SubscriberBackEndROS(const char* _node_name, const char* _topic, int _argc, char** _argv)
		:topic_(_topic)
	{
		RosSingleton::init(_node_name, _argc, _argv);
	}

	//------------------------------------------------------------------------------------------------------------------
	void SubscriberBackEndROS::onMessage(MsgCallBack _cb) {
		if(!has_subscriber_) {
			ros_sub_ = RosSingleton::get()->handle()->subscribe(topic_.c_str(), 0, &SubscriberBackEndROS::onRosMsg, this);
			has_subscriber_ = true;
		}
		msg_cb_ = _cb;
	}

	//------------------------------------------------------------------------------------------------------------------
	void SubscriberBackEndROS::onNotification(NotifyCallBack _cb) {
		if(!has_subscriber_) {
			ros_sub_ = RosSingleton::get()->handle()->subscribe(topic_.c_str(), 0, &SubscriberBackEndROS::onRosNotification, this);
			has_subscriber_ = true;
		}
		notify_cb_ = _cb;
	}

	//------------------------------------------------------------------------------------------------------------------
	void SubscriberBackEndROS::onRosMsg(const std_msgs::String::ConstPtr& _s) {
		std::stringstream ss;
		ss << _s->data;
		msg_cb_(ss);
	}

	//------------------------------------------------------------------------------------------------------------------
	void SubscriberBackEndROS::onRosNotification(const std_msgs::Int32::ConstPtr& _n) {
		assert(_n->data == 1);
		notify_cb_();
	}

}} // namespace grvc::com

#endif // GRVC_USE_ROS