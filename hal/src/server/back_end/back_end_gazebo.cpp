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
#ifdef GRVC_USE_ROS

#include <geometry_msgs/Twist.h>
#include <grvc_quadrotor_hal/back_end/back_end_gazebo.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <grvc_com/ros/ros_singleton.h>

using namespace std;

namespace grvc { namespace hal {
	
	//------------------------------------------------------------------------------------------------------------------
	BackEndGazebo::BackEndGazebo(const char* _node_name, int _argc, char** _argv)
		: cur_task_state_(TaskState::running)
	{
		// Init ros
		com::RosSingleton::init(_node_name, _argc, _argv);
		ros_handle_ = com::RosSingleton::get()->handle();
		// Init myself
		setDefaultParams();
		parseArguments(_argc, _argv);
		startRosCommunications();
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::goToWP(const Waypoint& _wp) {
		ROS_INFO_STREAM("GoToWp " << _wp.pos.transpose() << ", yaw = " << _wp.yaw);
		cur_path_.clear(); // Cancel path tracking
		state_controller_.setReferencePos(_wp.pos);
		state_controller_.setReferenceYaw(_wp.yaw);
		cur_task_state_ = TaskState::running;
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::trackPath(const WaypointList& _path) {
		cur_path_.clear();
		cur_path_.reserve(_path.size());
		cur_path_.insert(cur_path_.begin(), _path.rbegin(), _path.rend()); // Store the path in reverse order (like a stack)
		state_controller_.setReferencePos(cur_path_.back().pos);
		state_controller_.setReferenceYaw(cur_path_.back().yaw);
		cur_path_.pop_back();
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::takeOff(double _z) {
		if(!has_odometry_)
			return; // Can't figure out take off destination
		cur_task_state_ = TaskState::running;
		ROS_INFO_STREAM("Take off curPos = " << state_controller_.pos().transpose());
		Vec3 ref_pos = state_controller_.pos();
		ref_pos.z() = _z;
		ROS_INFO_STREAM("Take off refPos = " << ref_pos.transpose());
		state_controller_.setReferencePos(ref_pos);
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::land() {
		if(!has_odometry_)
			return; // Don't know where to land
		cur_task_state_ = TaskState::running;
		ROS_INFO_STREAM("land curPos = " << state_controller_.pos().transpose());
		Vec3 ref_pos = state_controller_.pos();
		ref_pos.z() = 0.0;
		state_controller_.setReferencePos(ref_pos);
	}

	//------------------------------------------------------------------------------------------------------------------
	TaskState BackEndGazebo::curTaskState() const {
		return cur_task_state_;
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::abortTask() {
		state_controller_.setReferencePos(state_controller_.pos());
		cur_path_.clear(); // Cancel current path, if any
		if(cur_task_state_ == TaskState::running)
			cur_task_state_ = TaskState::aborted;
	}

	//------------------------------------------------------------------------------------------------------------------
	Vec3 BackEndGazebo::position() const {
		return state_controller_.pos();
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::setDefaultParams() {
		gazebo_ns_ = "gazebo_ns";
		cmd_vel_topic_ = "cmd_vel";
		odometry_topic_ = "ground_truth/state";
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::parseArguments(int _argc, char** _argv) {
		const string gazebo_ns_arg = "-gazebo_ns=";
		const string cmd_vel_topic_arg = "-cmd_vel_topic=";
		const string odometry_topic_arg = "-odometry_topic=";

		for(int i = 0; i < _argc; ++i) {
			string arg = _argv[i];
			if(parseArg(arg, gazebo_ns_arg, gazebo_ns_))
				continue;
			if(parseArg(arg, cmd_vel_topic_arg, cmd_vel_topic_))
				continue;
			if(parseArg(arg, odometry_topic_arg, odometry_topic_))
				continue;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool BackEndGazebo::parseArg(const string& _arg, const string& _label, string& _dst) {
		if(_arg.substr(0, _label.size()) == _label) {
			_dst = _arg.substr(_label.size());
			return true;
		}
		return false;
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::startRosCommunications() {
		// Topic to set velocity references for gazebo plugin
		auto cmd_vel_full_topic = gazebo_ns_ + "/" + cmd_vel_topic_;
		cmd_vel_pub_ = ros_handle_->advertise<geometry_msgs::Twist>(cmd_vel_full_topic.c_str(), 0);

		// Suscribe to odometry messages from gazebo
		auto odometry_full_topic = gazebo_ns_ + "/" + odometry_topic_;
		odometry_sub_ = ros_handle_->subscribe(odometry_full_topic.c_str(),
               1000, &BackEndGazebo::odometryCb,
                        this);
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::odometryCb(const nav_msgs::Odometry::ConstPtr& _odom) {
		// -- Update model controller --
		auto gazeboPos = _odom->pose.pose.position;
		state_controller_.setPos({gazeboPos.x, gazeboPos.y, gazeboPos.z});
		// Rotation
		tf::Quaternion orientation(_odom->pose.pose.orientation.x,
			_odom->pose.pose.orientation.y,
			_odom->pose.pose.orientation.z,
			_odom->pose.pose.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
		state_controller_.setYaw(yaw);
		// -- Init control loop if necessary
		initControlReferences();
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::initControlReferences() {
		if(!has_odometry_) {
			// Set current state as initial reference for control
			state_controller_.setReferencePos(state_controller_.pos());
			state_controller_.setReferenceYaw(state_controller_.yaw());
			// Now that we have odometry, we can start running the control loop
			update_timer_ = ros_handle_->createTimer(ros::Duration(1/update_rate_),
				[this](const ros::TimerEvent& _te) { updateCb(_te); });
			publish_timer_ = ros_handle_->createTimer(ros::Duration(1/publish_rate_),
				[this](const ros::TimerEvent& _te) { publishCb(_te); });
			cur_task_state_ = TaskState::finished; // Finished initialization
			has_odometry_ = true;
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::publishCb(const ros::TimerEvent&) {
		// We can only publish control information when there is a state estimation available,
		// generating control references for the PIDs
		geometry_msgs::Twist twist;
		auto vel = state_controller_.velocity();
		twist.linear.x = vel.x();
		twist.linear.y = vel.y();
		twist.linear.z = vel.z();
		twist.angular.x = 0;
		twist.angular.y = 0;
		twist.angular.z = state_controller_.yaw();
		cmd_vel_pub_.publish(twist);
	}

	//------------------------------------------------------------------------------------------------------------------
	void BackEndGazebo::updateCb(const ros::TimerEvent& _te) {
		// Compute real elapsed time. Might not exactly match the update rate
		ros::Duration deltaT = _te.current_real - _te.last_real;
		// Update control actions
		state_controller_.updateControlActions(gazebo::common::Time(deltaT.sec, deltaT.nsec));
		if(has_odometry_ && reachedGoal()) {
			if(cur_path_.size()) {
				state_controller_.setReferencePos(cur_path_.back().pos);
				state_controller_.setReferenceYaw(cur_path_.back().yaw);
				cur_path_.pop_back();
			}else {
				cur_task_state_ = TaskState::finished;
			}
		}
	}

	//------------------------------------------------------------------------------------------------------------------
	bool BackEndGazebo::reachedGoal() const {
		double yawDiff = state_controller_.yawReference() - state_controller_.yaw();
		if(yawDiff*yawDiff > 0.1)
			return false;
		double posDiff = (state_controller_.posReference() - state_controller_.pos()).norm();
		if(posDiff > 0.1)
			return false;
		return true;
	}
}}	// namespace grvc::hal

#endif // GRVC_USE_ROS