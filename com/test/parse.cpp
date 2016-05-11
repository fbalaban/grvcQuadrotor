//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor HAL sample
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
#include "grvc_com/subscriber.h"
#include <string>
#include <cassert>
#include <Eigen/Core>

using namespace grvc::com;
using namespace std;

// -- Mock classes --
class MockSubscriberBE : public SubscriberBackEnd {
public:
	// Inherited via PublisherBackEnd
	void receive(const char * _msg)
	{
		stringstream ss;
		ss << _msg;
		msg_cb_(ss);
	}

	void notify() {
		notify_cb_();
	}
} g_sub_back_end;

SubscriberBackEnd* SubscriberBackEnd::createBackEnd(const char*, const char*, int, char**) {
	return &g_sub_back_end;
}

//----------------------------------------------------------------------------------------------------------------------
int main(int, char**) {
	// Notification test
	{
		bool notified = false;
		Subscriber<void> sub("", "", 0, nullptr, [&]() {
			notified = true;
		});
		g_sub_back_end.notify();
		assert(notified);
	}

	// Simple parse tests
	{
		Subscriber<size_t> sub("", "", 0, nullptr, [](const size_t& _i) {
			assert(_i == 42);
		});

		g_sub_back_end.receive("42");
	}

	{
		Subscriber<int> sub("", "", 0, nullptr, [](const int& _i) {
			assert(_i == -3);
		});

		g_sub_back_end.receive("-3");
	}

	{
		Subscriber<double> sub("", "", 0, nullptr, [](const double& _d) {
			assert(_d == 1.5);
		});

		g_sub_back_end.receive("1.5");
	}

	{
		Subscriber<Eigen::Vector3d> sub("", "", 0, nullptr, [](const Eigen::Vector3d& _v) {
			assert(_v == Eigen::Vector3d(0.5,0.5,0.5));
		});

		g_sub_back_end.receive("0.5\n0.5\n0.5");
	}

	return 0;
}