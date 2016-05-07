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
#include <grvc_com/publisher_back_end.h>

#ifdef GRVC_USE_ROS
//#include <grvc_com/publisher_back_end_ros.h>
#endif // GRVC_USE_ROS

namespace grvc { namespace com {
	
	//------------------------------------------------------------------------------------------------------------------
	PublisherBackEnd* PublisherBackEnd::createBackEnd(const char* _node_name, const char* _topic, int _argc, char** _argv) {
		PublisherBackEnd* be = nullptr; // Default implementation returns no back end.
#ifdef GRVC_USE_ROS
		//be = new PublisherBackEndROS(_node_name, _topic, _argc, _argv);
#else
		_node_name;
		_topic;
		_argc;
		_argv;
#endif // GRVC_USE_ROS
		return be;
	}
	
}} // namespace grvc::com