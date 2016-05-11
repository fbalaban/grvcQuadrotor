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
#ifndef _GRVCQUADROTOR_COM_PUBLISHER_H_
#define _GRVCQUADROTOR_COM_PUBLISHER_H_

#include <sstream>
#include "publisher_back_end.h"

namespace grvc { namespace com {

	/// Allows publishing information to different nodes in the network
	class Publisher {
	public:
		/// \param _node_mame unique identifier of the executable running this publisher
		/// \param _topic unique identifier with path/like/syntax that specifies the communication channel
		/// \param _argc number of command line arguments
		/// \param _argv array of command line arguments
		Publisher(const char* _node_name, const char* _topic, int _argc, char** _argv);

		/// Publish a message of arbitrary type.
		/// \param T_ requires either a specialized implementation of Publisher::serialize
		/// or operator << to be defined for serialization
		template<class T_>
		void publish(const T_& _msg);

		/// Dataless publish
		///
		/// Sends an empty notification to all subscribers. Useful for simple commands that require no data
		/// or when subscribers already know the data and are just waiting for a start signal.
		void publish() {
			back_end_->notify(); // Dummy, empty message
		}
	private:
		template<class T_>
		void serialize(std::ostream& _os, const T_& _t);

	private:
		PublisherBackEnd* back_end_ = nullptr;
	};

	//------------------------------------------------------------------------------------------------------------------
	// Inline implementation
	//------------------------------------------------------------------------------------------------------------------
	inline Publisher::Publisher(const char* _node_name, const char* _topic, int _argc, char** _argv) {
		back_end_ = PublisherBackEnd::createBackEnd(_node_name, _topic, _argc, _argv);
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class T_>
	void Publisher::publish(const T_& _msg) {
		std::stringstream ss;
		serialize(ss, _msg);
		back_end_->publish(ss.str().c_str());
	}

	//------------------------------------------------------------------------------------------------------------------
	template<class T_>
	void Publisher::serialize(std::ostream& _os, const T_& _t) {
		_os << _t;
	}

}} // namespace grvc::com

#endif // _GRVCQUADROTOR_COM_PUBLISHER_H_