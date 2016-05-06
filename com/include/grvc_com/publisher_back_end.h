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
#ifndef _GRVCQUADROTOR_COM_PUBLISHERBACKEND_H_
#define _GRVCQUADROTOR_COM_PUBLISHERBACKEND_H_

namespace grvc { namespace com {
	
	class PublisherBackEnd {
	public:
		/// Actually send a serialized message throught the communication channel
		virtual void publish (const char* _msg) = 0;

		/// Creates the proper backend depending on current platform and command line arguments provided (argc, argv)
		/// \param _nodeName unique identifier of the executable running this publisher
		/// \param _topic unique identifier with path/like/syntax that specifies the communication channel
		/// \param _argc number of command line arguments
		/// \param _argv array of command line arguments
		static PublisherBackEnd* createBackEnd(const char* _nodeName, const char* _topic, int _argc, char** _argv);
	};
	
}} // 

#endif // _GRVCQUADROTOR_COM_PUBLISHERBACKEND_H_