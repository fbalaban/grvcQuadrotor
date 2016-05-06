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
			if(!platform_impl_->update())
				return;
			//publishBackEndInfo();
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
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::parseArguments(int _argc, char** _argv) {
		const string hal_ns_arg = "-hal_ns=";
		const string wp_topic_arg = "-wp_topic=";
		const string take_off_arg = "-take_off_topic=";

		for (int i = 0; i < _argc; ++i) {
			string arg = _argv[i];
			if (parseArg(arg, hal_ns_arg, hal_ns_))
				break;
			if (parseArg(arg, wp_topic_arg, wp_topic_))
				break;
			if (parseArg(arg, take_off_arg, take_off_topic_))
				break;
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
		// Suscribe to waypoint command topic
		auto wp_full_topic = hal_ns_ + "/" + wp_topic_;
		auto bindGoToWP = [this](const Vec3& _v) { platform_impl_->goToWP(_v); };
		wp_sub_ = new com::Subscriber("hal_node", wp_full_topic.c_str(), _argc, _argv);
		wp_sub_->setCallBack<Vec3>(bindGoToWP);
		// Suscribe to take off topic
		auto take_off_topic = hal_ns_ + "/" + take_off_topic_;
		auto bindTakeOff = [this](double _z) { platform_impl_->takeOff(_z); }
		take_off_sub_ = new com::Subscriber("hal_node")
	}
	
}}	// namespace grvc