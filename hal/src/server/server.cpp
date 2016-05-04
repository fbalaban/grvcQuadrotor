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
#include <hal_server/server.h>
#include <hal_server/service/service.h>
#include <hal_server/back_end/back_end.h>
#include <string>
#include <thread>

using namespace std;
using namespace std::chrono;

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	Server::Server(int _argc, char** _argv) {
		// Set up back end implementation
		platform_impl_ = BackEnd::createBackEnd(_argc, _argv);
		// Set up public service
		public_service_ = Service::createService(_argc, _argv);
		// Link public service to implementation
		registerCallBacks();
		// Start time stamp for update cycle
		last_update_ = high_resolution_clock::now();
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::run() {
		for (;;) {
			if(!platform_impl_->update())
				return;
			publishBackEndInfo();
			if (!public_service_->update())
				return;
			// Sleep until next update
			if(update_rate_ > 0) {
				auto period = milliseconds(1000/update_rate_);
				this_thread::sleep_until(last_update_ + period);
				last_update_ = high_resolution_clock::now();
			}
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::registerCallBacks() {
		// Directly map back end implementation to public service call backs
		auto bindGoToWP = [this](const Vec3& _v) { platform_impl_->goToWP(_v); };
		public_service_->onGoToWP(bindGoToWP);
		auto bindTakeOff = [this](double _z){ platform_impl_->takeOff(_z); };
		public_service_->onTakeOff(bindTakeOff);
		// Methods without arguments can simply be passed with std::bind
		public_service_->onLand(std::bind(&BackEnd::land, platform_impl_));
		public_service_->onAbort(std::bind(&BackEnd::abortTask, platform_impl_));
	}

	//------------------------------------------------------------------------------------------------------------------
	void Server::publishBackEndInfo() {
		public_service_->publishPosition(	platform_impl_->position() );
		public_service_->publishTaskState(	platform_impl_->curTaskState() );
	}
	
}}	// namespace grvc