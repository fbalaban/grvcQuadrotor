//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor COM
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
#ifndef _GRVCQUADROTOR_COM_SUBSCRIBERBACKEND_H_
#define _GRVCQUADROTOR_COM_SUBSCRIBERBACKEND_H_

#include <functional>
#include <iostream>

namespace grvc {
	namespace com {

		/// Subscribes to a communication channel and invokes a callback whenever a message is received.
		/// Expects incomming data to be serialized as strings
		class SubscriberBackEnd {
		public:
			/// Callback functors that can be invoked by the subscriber
			/// \param T_ data type you expect to receive. This is the type your callback admits
			typedef std::function<void(std::istream& _is)>	CallBack;

			/// Register a callback
			///
			/// Whoever uses this class is responsible for registering callbacks for every event they want to be notified of.
			/// New callbacks override older ones, so only the most recent one will be invoked for any event. If you require
			/// Multiple callbacks to be invoked, you will need to do the dispatching for yourself.
			void onMessage(CallBack _cb) { cb_ = _cb; }

			/// Creates the proper backend depending on current platform and command line arguments provided (argc, argv)
			/// \param _node_mame unique identifier of the executable running this subscriber
			/// \param _topic unique identifier with path/like/syntax that specifies the communication channel
			/// \param _argc number of command line arguments
			/// \param _argv array of command line arguments
			static SubscriberBackEnd* createBackEnd(const char* _node_name, const char* _topic, int _argc, char** _argv);

		private:
			CallBack cb_; ///< Back end implementations must invoke this CallBack.
		};

	}
} // namespace grvc::com

#endif // _GRVCQUADROTOR_COM_SUBSCRIBERBACKEND_H_
