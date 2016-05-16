//------------------------------------------------------------------------------
// GRVC Quadrotor HAL
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
#include <grvc_quadrotor_hal/server.h>
#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/back_end/back_end.h>
#include <string>
#include <thread>

using namespace std;
using namespace std::chrono;

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	Server::Server(int _argc, char** _argv) {
		// Set up back end implementation
		setDefaultParams();
		parseArguments(_argc, _argv);
		startCommunications(_argc, _argv);
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::run() {
		for (;;) {
			publishStateInfo();
			pos_pub_->publish(platform_impl_->position());
			// Sleep until next update
			if(update_rate_ > 0) {
				auto period = milliseconds(1000/update_rate_);
				this_thread::sleep_until(last_update_ + period);
				last_update_ = high_resolution_clock::now();
			}
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::setDefaultParams() {
		hal_ns_ = "hal_ns";
		wp_topic_ = "go_to_wp";
		take_off_topic_ = "take_off";
		land_topic_ = "land";
		state_topic_ = "state";
		pos_topic_ = "position";
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::parseArguments(int _argc, char** _argv) {
		const string hal_ns_arg = "-hal_ns=";
		const string wp_topic_arg = "-wp_topic=";
		const string take_off_arg = "-take_off_topic=";
		const string land_arg = "-land_topic=";
		const string state_arg = "-state_topic=";
		const string pos_arg = "-position=";

		for (int i = 0; i < _argc; ++i) {
			string arg = _argv[i];
			if (parseArg(arg, hal_ns_arg, hal_ns_))
				continue;
			if (parseArg(arg, wp_topic_arg, wp_topic_))
				continue;
			if (parseArg(arg, take_off_arg, take_off_topic_))
				continue;
			if (parseArg(arg, land_arg, land_topic_))
				continue;
			if (parseArg(arg, state_arg, state_topic_))
				continue;
			if (parseArg(arg, pos_arg, pos_topic_))
				continue;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool Server::parseArg(const string& _arg, const string& _label, string& _dst) {
		if (_arg.substr(0, _label.size()) == _label) {
			_dst = _arg.substr(_label.size());
			return true;
		}
		return false;
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::startCommunications(int _argc, char** _argv) {
		// Connect to back end hal
		const char node_name[] = "hal_node";
		platform_impl_ = BackEnd::createBackEnd(node_name, _argc, _argv);
		// Suscribe to waypoint command topic
		auto wp_full_topic = hal_ns_ + "/" + wp_topic_;
		auto bindGoToWP = [this](const Waypoint& _wp) { platform_impl_->goToWP(_wp); };
		wp_sub_ = new com::Subscriber<Waypoint>(node_name, wp_full_topic.c_str(), _argc, _argv, bindGoToWP);
		// Suscribe to take off topic
		auto take_off_full_topic = hal_ns_ + "/" + take_off_topic_;
		auto bindTakeOff = [this](double _z) { platform_impl_->takeOff(_z); };
		take_off_sub_ = new com::Subscriber<double>(node_name, take_off_full_topic.c_str(), _argc, _argv, bindTakeOff);
		// Suscribe to land topic
		auto land_full_topic = hal_ns_ + "/" + land_topic_;
		auto bindLand = std::bind(&BackEnd::land, platform_impl_);
		land_sub_ = new com::Subscriber<void>(node_name, land_full_topic.c_str(), _argc, _argv, bindLand);
		// Publish to state topic
		auto state_full_topic = hal_ns_ + "/" + state_topic_;
		state_pub_ = new com::Publisher(node_name, state_full_topic.c_str(), _argc, _argv);
		// Publish to position topic
		auto pos_full_topic = hal_ns_ + "/" + pos_topic_;
		pos_pub_ = new com::Publisher(node_name, pos_full_topic.c_str(), _argc, _argv);
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::publishStateInfo() {
		TaskState cur_task_state = platform_impl_->curTaskState();
		switch(cur_task_state) {
			case TaskState::finished:
			{
				state_pub_-> publish("finished");
				break;
			}
			case TaskState::running:
			{
				state_pub_-> publish("running");
				break;
			}
			case TaskState::failed:
			{
				state_pub_-> publish("failed");
				break;
			}
			case TaskState::aborted:
			{
				state_pub_-> publish("aborted");
				break;
			}
		}
	}
	
}}	// namespace grvc