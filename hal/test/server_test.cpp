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
	void publishPosition(const Vec3& _pos) override { position_ = _pos; }
	void publishTaskState(TaskState _state) override { task_state_ = _state; }

	bool update() override { 
		update_called_ = true;
		return !must_exit_;
	}

	GoToWpCb goToWp;
	TakeOffCb takeOff;
	LandCb land;
	AbortCb abort;

	bool update_called_ = false;
	bool must_exit_ = false;
	Vec3 position_;
	TaskState task_state_;
};

Service* Service::createService(int , char**) {
	return new MockService;
}

// Mock Back End
class MockBackEnd : public BackEnd {
public:
	void		goToWP(const Vec3& _wp) override { position_ = _wp; }
	void		takeOff(double _height) override { height_ = _height; }
	void		land() override {}
	TaskState	curTaskState() const override { return TaskState::running; }
	void		abortTask() override {}
	Vec3		position() const override { return position_; }

	bool update() override {
		update_called_ = true;
		return !must_exit_;
	}

	bool update_called_ = false;
	bool must_exit_ = false;

	Vec3 position_ = Vec3::Zero();
	double height_ = 0.0;
};

BackEnd* BackEnd::createBackEnd(int, char**) {
	return new MockBackEnd;
}

int main(int _argc, char** _argv) {
	_argc; _argv;
	return 0;
}