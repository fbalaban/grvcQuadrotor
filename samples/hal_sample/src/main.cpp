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
#include <grvc_com/publisher.h>
#include <grvc_quadrotor_hal/server.h>
#include <cstdint>
#include <thread>

using namespace grvc::com;
using namespace grvc::hal;

int main(int _argc, char** _argv) {
	// Use this to send waypoints to hal
	Publisher* wpPub = new Publisher("hal_sample", "/quad1/hal/go_to_wp", _argc, _argv);
	Publisher* takeOffPub  = new Publisher("hal_sample", "/quad1/hal/take_off", _argc, _argv);
	Publisher* landPub  = new Publisher("hal_sample", "/quad1/hal/land", _argc, _argv);
	

	double flyZ = 1.0;
	Vec3 points[2] = { {0.0, 0.0, flyZ}, {3.0, 0.0, flyZ} };

	while()
	for (size_t t = 0; t < 100; ++t) {
		takeOffPub->publish(flyZ);
		std::this_thread::sleep_for(std::chrono::seconds(3));
		for (size_t i = 0; i < 2; ++i) {
			wpPub->publish(points[i]);
			std::this_thread::sleep_for(std::chrono::seconds(3));
		}
		landPub->publish();
		std::this_thread::sleep_for(std::chrono::seconds(3));
	}

	return 0;
}