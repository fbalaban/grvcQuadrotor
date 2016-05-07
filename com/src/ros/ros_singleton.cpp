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

#include <grvc_com/ros/ros_singleton.h>
#include <sstream>

namespace grvc { namespace com {

	//------------------------------------------------------------------------------------------------------------------
	// Static data definitions
	RosSingleton* RosSingleton::s_instance_ = nullptr;

	//------------------------------------------------------------------------------------------------------------------
	void RosSingleton::init(const char* _node_name, int _argc, char** _argv) {
		if(!s_instance_) {
			s_instance_ = new RosSingleton(_node_name, _argc, _argv);
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	RosSingleton::RosSingleton(const char* _node_name, int _argc, char** _argv) {
		ros::init(_argc, _argv, _node_name, ros::init_options::AnonymousName);
		ros_handle_ = new ros::NodeHandle(_node_name);
	}

}} // namespace grvc::com

#endif // GRVC_USE_ROS