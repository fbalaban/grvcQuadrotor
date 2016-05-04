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
#include <hal_server/server.h>
#include <hal_server/service/service.h>
#include <hal_server/back_end/back_end.h>

using namespace grvc::hal;

// ---- Mock classes ----
// Mock service
class MockService : public Service {
	void onGoToWP(GoToWpCb _cb) override { goToWp = _cb; }
	void onTakeOff(TakeOffCb _cb) override { takeOff = _cb; }
	void onLand(LandCb _cb) override { land = _cb; }
	void onAbort(AbortCb _cb) override { abort = _cb; }
	void publishPosition(const Vec3& _pos) override { position = _pos; }
	void publishTaskState(TaskState _state) override { taskState = _state; }

	GoToWpCb goToWp;
	TakeOffCb takeOff;
	LandCb land;
	AbortCb abort;

	Vec3 position;
	TaskState taskState;
};

Service* Service::createService(int _argc, char** _argv) {
	return new MockService;
}

// Mock Implementation