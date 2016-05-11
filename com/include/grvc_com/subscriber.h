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
#include <Eigen/Core>

namespace grvc {
	namespace com {

		/// Subscribes to a communication channel and invokes a callback whenever a message is received
		/// \param T_ data type you expect to receive. This is the type your callback admits
		template<class T_>
		class Subscriber {
		public:
			/// Callback functors that can be invoked by the subscriber
			typedef std::function<void(const T_&)> MsgCallBack;
			typedef std::function<void(void)> NotifyCallBack;

			/// \param _node_mame unique identifier of the executable running this publisher
			/// \param _topic unique identifier with path/like/syntax that specifies the communication channel
			/// \param _argc number of command line arguments
			/// \param _argv array of command line arguments
			/// \param _cb Functor to be invoked when a message arrives¡
			Subscriber(const char* _node_name, const char* _topic, int _argc, char** _argv, MsgCallBack _cb) {
				back_end_ = SubscriberBackEnd::createBackEnd(_node_name, _topic, _argc, _argv);
				back_end_->onMessage([=](std::istream& _is){
					T_ t;
					deserialize(_is, t);
					_cb(t);
				});
			}

			void deserialize(std::istream& _is, T_& _t) {
				_is >> _t;
			}

		private:
			SubscriberBackEnd* back_end_ = nullptr;
		};

		// Specialization for data-less subscribers
		template<>
		class Subscriber<void> {
		public:
			typedef std::function<void()> CallBack;

			Subscriber(const char* _node_name, const char* _topic, int _argc, char** _argv, CallBack _cb) {
				back_end_ = SubscriberBackEnd::createBackEnd(_node_name, _topic, _argc, _argv);
				back_end_->onNotification([=](){
					_cb();
				});
			}

		private:
			SubscriberBackEnd* back_end_ = nullptr;
		};

	}
} // namespace grvc::com

// -------- Specialized extractor operators
// Eigen vectors
template<class Scalar_, int cols_>
inline std::istream& operator >> (std::istream& _is, Eigen::Matrix<Scalar_,cols_,1>& _v) {
	for (int i = 0; i < _v.rows() - 1; ++i) {
		_is >> _v(i); // get ith component
		_is.get(); // skip \n
	}
	_is >> _v(_v.rows()-1); // get last component
	return _is;
}

#endif // _GRVCQUADROTOR_COM_SUBSCRIBER_H_