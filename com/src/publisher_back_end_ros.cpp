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

#include <grvc_com/publisher_back_end_ros.h>
#include <std_msgs/String.h>

namespace grvc { namespace com {
	
	//------------------------------------------------------------------------------------------------------------------
	// Static data definitions
	ros::NodeHandle* PublisherBackEndROS::ros_handle_ = nullptr;

	//------------------------------------------------------------------------------------------------------------------
	PublisherBackEndROS::PublisherBackEndROS(const char* _node_name, const char* _topic, int _argc, char** _argv) {
		init(_node_name, _argc, _argv);
		ros_publisher_ = ros_handle_->advertise<std_msgs::String>(_topic, 0);
	}

	//------------------------------------------------------------------------------------------------------------------
	void PublisherBackEndROS::init(const char* _node_name, int _argc, char** _argv) {
		if(!ros_handle_) {
			ros::init(_argc, _argv, _node_name, ros::init_options::AnonymousName);
			ros_handle_ = new ros::NodeHandle(_node_name);
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	void PublisherBackEndROS::publish (const char* _msg) {
		std_msgs::String m;
		m.data = _msg;
		ros_publisher_.publish(m);
	}
	
}} // namespace grvc::com

#endif // GRVC_USE_ROS