//------------------------------------------------------------------------------
// GRVC Quadrotor Control
//------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------
#ifndef _GRVCQUADROTOR_SIMULATION_GRVCQUADROTORCONTROL_STATE_H_
#define _GRVCQUADROTOR_SIMULATION_GRVCQUADROTORCONTROL_STATE_H_

#include <geometry_msgs/Twist.h>

namespace grvc {

	class State {
	public:
		typedef geometry_msgs::Vector3	Vec3;

	public:
		double 		yawReference () const { return yaw_action_; }
		const Vec3& posReference () const { return spd_action_; }

		virtual void setModelEstimation(double _yaw, const Vec3& _pos) = 0;

		virtual State* nextState() const = 0;

		// Action callbacks
		virtual void onFlyRequest(const Vec3& _goalPos) = 0;
		virtual void onTakeOffRequest() = 0;
		virtual void onLandRequest() = 0;

		virtual bool completedAction() const = 0;

	protected:
		// Internal state
		double	yaw_reference_;
		Vec3	pos_reference_;
	};

} // namespace grvc

#endif // _GRVCQUADROTOR_SIMULATION_GRVCQUADROTORCONTROL_STATE_H_