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
#include "grvc_com/action_client.h"
#include "grvc_com/action_server.h"
#include <string>
#include <cassert>
#include <Eigen/Core>
#include <map>

using namespace grvc::com;
using namespace std;

// -- Mock classes --
//----------------------------------------------------------------------------------------------------------------------
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
};

std::map<std::string, MockSubscriberBE*> g_subs;

SubscriberBackEnd* SubscriberBackEnd::createBackEnd(const char*, const char* _ns, int, char**) {
	g_subs[_ns] = new MockSubscriberBE;
	return g_subs[_ns];
}

//----------------------------------------------------------------------------------------------------------------------
class MockPublisherBE : public PublisherBackEnd {
	// Inherited via PublisherBackEnd
	void publish(const char * _msg) override
	{
		g_subs[ns_]->receive(_msg);
	}

	void notify() override
	{
		g_subs[ns_]->notify();
	}

public:
	std::string ns_;
};

std::map<std::string, MockPublisherBE*> g_pubs;

PublisherBackEnd* PublisherBackEnd::createBackEnd(const char*, const char* _ns, int, char**) {
	g_pubs[_ns] = new MockPublisherBE;
	g_pubs[_ns]->ns_ = _ns;
	return g_pubs[_ns];
}

//----------------------------------------------------------------------------------------------------------------------
int main(int, char**) {
	{
		typedef ActionClient<size_t, size_t> Client;
		Client client("", "a", 0, nullptr);
		ActionServer<size_t,size_t> server("", "a", 0, nullptr);
		server.onRequestedGoal([](size_t _goal) {
			return _goal == 1;
		});
		server.onAbort([](){});

		// 

		assert(client.goalState() == Client::GoalState::success);
	}
	return 0;
}