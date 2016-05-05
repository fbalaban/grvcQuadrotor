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
#include <hal_server/service/service_ros.h>

using namespace std;

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	ServiceROS::ServiceROS(const char* _node_name, int _argc, char** _argv) {
		ros::init(_argc, _argv, _node_name, ros::init_options::AnonymousName);
		ros_handle_ = new ros::NodeHandle(_node_name);
		// Init myself
		setDefaultParams();
		parseArguments(_argc, _argv);
		startRosCommunications();
	}

	//------------------------------------------------------------------------------------------------------------------
	void ServiceROS::onGoToWP(GoToWpCb _cb) {
		go_to_wp_cb_ = _cb;
	}

	//------------------------------------------------------------------------------------------------------------------
	void ServiceROS::setDefaultParams() {
		hal_ns_ = "hal_ns";
		cmd_topic_ = "cmd";
		cmd_state_topic_ = "cmd_state";
		odometry_topic_ = "odometry";
	}

	//------------------------------------------------------------------------------------------------------------------
	void ServiceROS::parseArguments(int _argc, char** _argv) {
		const string hal_ns_arg = "-hal_ns=";
		const string cmd_topic_arg = "-cmd_topic=";
		const string odometry_topic_arg = "-odometry_topic=";
		const string cmd_state_topic_arg = "-cmd_state_topic=";

		for(int i = 0; i < _argc; ++i) {
			string arg = _argv[i];
			if(parseArg(arg, hal_ns_arg, hal_ns_))
				break;
			if(parseArg(arg, cmd_topic_arg, cmd_topic_))
				break;
			if(parseArg(arg, odometry_topic_arg, odometry_topic_))
				break;
			if(parseArg(arg, cmd_state_topic_arg, cmd_state_topic_))
				break;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool ServiceROS::parseArg(const string& _arg, const string& _label, string& _dst) {
		if(_arg.substr(0, _label.size()) == _label) {
			_dst = _arg.substr(_label.size());
			return true;
		}
		return false;
	}

	//------------------------------------------------------------------------------------------------------------------
	void ServiceROS::startRosCommunications() {
		// Suscribe to commands topic
		auto cmd_full_topic = hal_ns_ +  "/" + cmd_topic_;
		cmd_sub_ = ros_handle_->subscribe(cmd_full_topic.c_str(), 1000, &ServiceROS::onCmdCallBack, this);
	}

	//------------------------------------------------------------------------------------------------------------------
	void ServiceROS::onCmdCallBack(const CmdMsg::ConstPtr& _cmd) {
		if(_cmd->command == "goToWp") {
			auto pos = _cmd->pos.position;
			Vec3 waypoint = { pos.x, pos.y, pos.z };
			go_to_wp_cb_(waypoint);
		}
	}

	
}}	// namespace grvc