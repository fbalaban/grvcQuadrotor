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
#ifndef _GRVCQUADROTOR_HALSERVER_SERVER_H_
#define _GRVCQUADROTOR_HALSERVER_SERVER_H_

#include <chrono>
#include <grvc_com/publisher.h>
#include <grvc_com/subscriber.h>
#include <grvc_quadrotor_hal/types.h>

namespace grvc { namespace hal {

	// Forward declarations
	class BackEnd;

	/// Encapsulate all functionality of hal into a single class. Derive from this class if you want to implement
	/// a project specific hal with more functionality
	class Server {
	public:
		Server(int _argc, char** _argv);
		virtual void run();

	private:
		void setDefaultParams();
		void parseArguments(int _argc, char** _argv);
		bool parseArg(const std::string& _arg, const std::string& _label, std::string& _dst);
		void startCommunications(int _argc, char** _argv);
		/// Read published info from back end and re-publish it to the service
		void publishStateInfo();

	private:
		typedef std::chrono::high_resolution_clock::time_point Time;
		Time last_update_;
		// Communication interfaces
		com::Subscriber<Waypoint>* wp_sub_ = nullptr;
		com::Subscriber<double>* take_off_sub_ = nullptr;
		com::Subscriber<void>* land_sub_ = nullptr;
		com::Publisher* state_pub_;
		com::Publisher* pos_pub_;

		// Communication topics
		std::string wp_topic_;
		std::string take_off_topic_;
		std::string land_topic_;
		std::string state_topic_;
		std::string pos_topic_;
		std::string hal_ns_;

		BackEnd* platform_impl_;
		int update_rate_ = 100;
	};
	
}}	// namespace grvc::hal

#endif // _GRVCQUADROTOR_HALSERVER_SERVER_H_