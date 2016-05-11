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
#include "grvc_com/publisher.h"
#include <string>
#include <cassert>
#include <Eigen/Core>

using namespace grvc::com;
using namespace std;

// -- Mock classes --
class MockPublisherBE : public PublisherBackEnd {
public:
	// Inherited via PublisherBackEnd
	void publish(const char * _msg) override
	{
		assert(expected_ == _msg);
	}

	void notify() override { notified_ = true; }

	string expected_;
	bool notified_ = false;
} g_pub_back_end;

PublisherBackEnd* PublisherBackEnd::createBackEnd(const char*, const char*, int, char**) {
	return &g_pub_back_end;
}

//----------------------------------------------------------------------------------------------------------------------
int main(int, char**) {
	Publisher p("","",0,nullptr);

	// Notification
	g_pub_back_end.notified_ = false;
	p.publish();
	assert(g_pub_back_end.notified_);

	// Simple serialization tests
	g_pub_back_end.expected_ = "42";
	size_t answer = 42;
	p.publish(answer);

	g_pub_back_end.expected_ = "-3";
	int negative = -3;
	p.publish(negative);

	g_pub_back_end.expected_ = "1.5";
	double floating = 1.5;
	p.publish(floating);

	// Eigen types
	g_pub_back_end.expected_ = "0.5\n0.5\n0.5";
	Eigen::Vector3d eigenV = 0.5 * Eigen::Vector3d::Ones();
	p.publish(eigenV);

	return 0;
}