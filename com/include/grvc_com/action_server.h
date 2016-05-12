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
#ifndef _GRVCQUADROTOR_COM_ACTIONSERVER_H_
#define _GRVCQUADROTOR_COM_ACTIONSERVER_H_

#include <functional>

namespace grvc {
	namespace com {

		/// Allows publishing information to different nodes in the network
		template<class Goal, class FeedBack>
		class ActionServer {
			/// Possible states of a goal request
			enum class GoalState {
				accepted,
				pending,
				rejected,
				success,
				fail,
				cancelled
			};

		public:
			/// \param _node_mame unique identifier of the executable running this server
			/// \param _topic_base unique identifier with path/like/syntax that specifies the base communication channel.
			/// All topics used by the action will be located under this namespace.
			/// \param _argc number of command line arguments
			/// \param _argv array of command line arguments
			ActionServer(const char* _node_name, const char* _topic_base, int _argc, char** _argv);

			/// \return whether the goal has been accepted
			virtual bool onRequestedGoal(const Goal&) = 0;
			virtual void abort() = 0; ///< Abort current goal (if any).

		protected:
			void setGoalState(GoalState);
			void sendFeedBack(const FeedBack&);

		private:
			Subsciber<Goal>* goal_sub_ = nullptr;
			Subscriber<Goal>* abort_sub_ = nullptr;
			Publisher* fb_pub_ = nullptr;
			Publisher* state_pub_ = nullptr;
		};
	}
} // namespace grvc::com

#endif // _GRVCQUADROTOR_COM_ACTIONSERVER_H_