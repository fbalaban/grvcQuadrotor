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
#ifndef _GRVCQUADROTOR_COM_SUBSCRIBER_H_
#define _GRVCQUADROTOR_COM_SUBSCRIBER_H_

#include <functional>
#include "subscriber_back_end.h"

namespace grvc {
	namespace com {

		/// Subscribes to a communication channel and invokes a callback whenever a message is received
		class Subscriber {
		public:
			/// Callback functors that can be invoked by the subscriber
			/// \param T_ data type you expect to receive. This is the type your callback admits
			template<class T_>
			using CallBack = std::function<void(const T_&)>;

			/// \param _node_mame unique identifier of the executable running this publisher
			/// \param _topic unique identifier with path/like/syntax that specifies the communication channel
			/// \param _argc number of command line arguments
			/// \param _argv array of command line arguments
			/// \param _cb Functor to be invoked when a message arrives¡
			Subscriber(const char* _node_name, const char* _topic, int _argc, char** _argv) {
				back_end_ = SubscriberBackEnd::createBackEnd(_node_name, _topic, _argc, _argv);
			}

			/// \param T_ data type you expect to receive. This is the type your callback admits
			template<class T_>
			void setCallBack(CallBack<T_> _cb) {
				back_end_->onMessage(CallBackDeserializer<T_>(_cb));
			}

			/// Specialization for dataless messages (simple notification events or commands)
			void setCallBack(std::function<void()> _cb) {
				auto wrapper = [&](std::istream& _is) {
					_cb();
				};
				back_end_->onMessage(wrapper);
			}

		private:
			/// Internal classes that deserializes incomming messages and subscribes to the back end
			/// subscriber as a functor
			template<class T_>
			struct CallBackDeserializer {
				CallBackDeserializer(CallBack<T_> _cb) 
					:cb_(_cb) {}

				void operator()(std::istream& _is) {
					T_ t;
					_is >> t;
					cb_(t);
				}

				CallBack<T_> cb_;
			};

		private:
			SubscriberBackEnd* back_end_ = nullptr;
		};

	}
} // namespace grvc::com

#endif // _GRVCQUADROTOR_COM_SUBSCRIBER_H_