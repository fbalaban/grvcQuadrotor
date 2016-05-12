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
#ifndef _GRVCQUADROTOR_COM_ACTIONCLIENT_H_
#define _GRVCQUADROTOR_COM_ACTIONCLIENT_H_

#include <functional>
#include "publisher.h"
#include "subscriber.h"
#include <string>

namespace grvc {
	namespace com {

		/// Allows publishing information to different nodes in the network
		template<class Goal, class FeedBack>
		class ActionClient {
		public:
			/// Possible states of a goal request
			enum class GoalState {
				accepted,
				pending,
				rejected,
				success,
				fail,
				cancelled
			};

			/// Delegate to be invoked on incomming feedback
			typedef std::function<void(const FeedBack&)>	FeedBackDelegate;
			/// Delegate to be invoked upon completion, cancellation or failure of the current goal
			typedef std::function<void(GoalState)>			GoalDelegate;

		public:
			/// \param _node_mame unique identifier of the executable running this client
			/// \param _topic_base unique identifier with path/like/syntax that specifies the base communication channel.
			/// All topics used by the action will be located under this namespace.
			/// \param _argc number of command line arguments
			/// \param _argv array of command line arguments
			ActionClient(const char* _node_name, const char* _topic_base, int _argc, char** _argv);
			
			void setGoal(const Goal&); ///< Request a new goal. Automatically cancels any previous goals
			void abort(); ///< Abort current goal (if any).
			GoalState goalState() const; ///< Query state of last requested goal
			
			/// Set up a callback to be invoked upon success or fail of goals.
			/// aborted goals will not invoke this callback
			void onFinish(GoalDelegate _cb) { goal_cb_ = _cb; }
			void onFeedBack(FeedBackDelegate _cb) { feedback_cb_ = _cb; } ///< Set up callbacks to process feedback information

		private:
			GoalState cur_state_ = GoalState::success;

			Publisher* goal_pub_ = nullptr;
			Publisher* abort_pub_ = nullptr;
			Subscriber<FeedBack>* fb_sub_ = nullptr;
			Subscriber<std::string>* state_sub_ = nullptr;

			GoalDelegate goal_cb_;
			FeedBackDelegate feedback_cb_;
		};

		//--------------------------------------------------------------------------------------------------------------
		template<class Goal_, class FeedBack_>
		ActionClient<Goal_, FeedBack_>::ActionClient(const char* _node_name, const char* _topic_base, int _argc, char** _argv) {
			std::string ns = std::string(_topic_base) + "/";
			// Create publishers
			goal_pub_ = new Publisher(_node_name, (ns + "goal").c_str(), _argc, _argv);
			abort_pub_ = new Publisher(_node_name, (ns + "abort").c_str(), _argc, _argv);
			// Goal state subscription
			state_sub_ = new Subscriber<std::string>(_node_name, (ns + "state").c_str(), _argc, _argv, [this](const std::string& _state) {
				if(_state == "pending")
					cur_state_ = GoalState::pending;
				else if(_state == "accepted")
					cur_state_ = GoalState::accepted;
				else if(_state == "rejected")
					cur_state_ = GoalState::rejected;
				else if(_state =="success")
					cur_state_ = GoalState::success;
				else if (_state == "fail")
					cur_state_ = GoalState::fail;
				else if (_state == "cancelled")
					cur_state_ = GoalState::cancelled;
				if (cur_state_ == GoalState::success || cur_state_ == GoalState::fail) {
					goal_cb_(cur_state_);
				}
			});
			// Feedback subscription
			fb_sub_ = new Subscriber<FeedBack_>(_node_name, (ns + "feedback").c_str(), _argc, _argv, [this](const FeedBack_& _fb) {
				feedback_cb_(_fb);
			});
		}

		//--------------------------------------------------------------------------------------------------------------
		template<class Goal_, class FeedBack_>
		void ActionClient<Goal_, FeedBack_>::setGoal(const Goal_& _goal) {
			cur_state_ = GoalState::pending;
			goal_pub_->publish(_goal);
		}

		//--------------------------------------------------------------------------------------------------------------
		template<class Goal_, class FeedBack_>
		void ActionClient<Goal_, FeedBack_>::abort() {
			abort_pub_->publish();
		}

		//--------------------------------------------------------------------------------------------------------------
		template<class Goal_, class FeedBack_>
		typename ActionClient<Goal_, FeedBack_>::GoalState ActionClient<Goal_, FeedBack_>::goalState() const {
			return cur_state_;
		}
	}
} // namespace grvc::com

#endif // _GRVCQUADROTOR_COM_ACTIONCLIENT_H_