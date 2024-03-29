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
#ifndef _GRVCQUADROTOR_UAVSERVER_SERVER_H_
#define _GRVCQUADROTOR_UAVSERVER_SERVER_H_

namespace grvc { 

	// Forward declarations
	namespace hal { class Client; }
	class Service;

	namespace uav {
	
	/// Encapsulate all functionality of uav into a single class. Derive from this class if you want to implement
	/// a project specific uav with more functionality
	class Server {
	public:
		Server(int _argc, char** _argv);
		virtual void run();

	private:
		// Communication interfaces
		hal::Client* hal_comm_; ///< Communicate with the underlying hal implementation
		Service* public_service_; ///< Communicate with clients requesting my functionality
	};
	
}}	// namespace grvc::uav

#endif // _GRVCQUADROTOR_UAVSERVER_SERVER_H_