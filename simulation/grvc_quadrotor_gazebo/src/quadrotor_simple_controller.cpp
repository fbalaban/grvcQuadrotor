//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <grvc_quadrotor_gazebo/quadrotor_simple_controller.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <cmath>
#include <cassert>

namespace gazebo {

GazeboQuadrotorSimpleController::GazeboQuadrotorSimpleController()
  : velocity_command_(0.0, 0.0, 0.0)
  , yaw_command_(0.0)
  , cur_vel_(0.0, 0.0, 0.0)
  , cur_yaw_spd_(0.0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorSimpleController::~GazeboQuadrotorSimpleController()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

//----------------------------------------------------------------------------------------
void GazeboQuadrotorSimpleController::parseSdfParams(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // load parameters
  if (!_sdf->HasElement("robotNamespace") || !_sdf->GetElement("robotNamespace")->GetValue())
   namespace_.clear();
  else
   namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("topicName") || !_sdf->GetElement("topicName")->GetValue())
   velocity_topic_ = "cmd_vel";
  else
   velocity_topic_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("stateTopic") || !_sdf->GetElement("stateTopic")->GetValue())
   state_topic_.clear();
  else
   state_topic_ = _sdf->GetElement("stateTopic")->Get<std::string>();

  if (!_sdf->HasElement("bodyName") || !_sdf->GetElement("bodyName")->GetValue())
  {
   link_ = _model->GetLink();
   link_name_ = link_->GetName();
  }
  else {
   link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
   link_ = _model->GetLink(link_name_);
  }
  
  if (!link_)
  {
   ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
   return;
  }

  loadPID(_sdf, "yaw", controllers_.yaw);
  loadPID(_sdf, "velocityXY", controllers_.vx);
  loadPID(_sdf, "velocityXY", controllers_.vy);
  loadPID(_sdf, "velocityZ", controllers_.vz);
}

//--------------------------------------------------------------------------------------------------
void GazeboQuadrotorSimpleController::loadPID(sdf::ElementPtr _sdf, const char* _paramName, common::PID& _pid) {
  if (!_sdf)
    return;

  double gain_p = 0.0;
  double gain_i = 0.0;
  double gain_d = 0.0;
  double limit = 0.0;
  std::string prefix(_paramName);

  if (_sdf->HasElement(prefix + "ProportionalGain")) gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>();
  if (_sdf->HasElement(prefix + "DifferentialGain")) gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>();
  if (_sdf->HasElement(prefix + "IntegralGain"))     gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>();
  if (_sdf->HasElement(prefix + "Limit"))            limit = _sdf->GetElement(prefix + "Limit")->Get<double>();

  _pid = common::PID(gain_p, gain_i, gain_d, 0.0, 0.0, limit, -limit);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world_ = _model->GetWorld();
  parseSdfParams(_model, _sdf);
  link_->SetGravityMode(false);
  // Init mass properties of the quad
  auto inertia = link_->GetInertial();
  if(inertia->GetMass() > 0.0)
    invMass = 1.0 / inertia->GetMass();
  else
    invMass = 1.0; // Default mass
  invZInertia = 1.0 / inertia->GetIZZ();

  node_handle_ = new ros::NodeHandle(namespace_);
  subscribeTopics();

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  control_timer_.Load(world_, _sdf);
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&GazeboQuadrotorSimpleController::update, this));
}

//--------------------------------------------------------------------------------------------------
void GazeboQuadrotorSimpleController::subscribeTopics() {
  // subscribe command
  if (!velocity_topic_.empty()) {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      velocity_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::velocityCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    velocity_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe state
  if (!state_topic_.empty()) {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::stateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorSimpleController::velocityCallback(const geometry_msgs::TwistConstPtr& _velocity)
{
  velocity_command_ = math::Vector3(_velocity->linear.x, _velocity->linear.y, _velocity->linear.z);
}

//------------------------------------------------------------------------------------------------------
void GazeboQuadrotorSimpleController::stateCallback(const nav_msgs::OdometryConstPtr& _state)
{
  auto abs_vel = math::Vector3(_state->twist.twist.linear.x, _state->twist.twist.linear.y, _state->twist.twist.linear.z);
  // Transform velocity to local coordinates
  auto msg_orientation = _state->pose.pose.orientation;
  math::Quaternion rotation (msg_orientation.w, msg_orientation.x, msg_orientation.y, msg_orientation.z);
  cur_vel_ = rotation.RotateVectorReverse(cur_vel_);

  cur_yaw_spd_ = _state->twist.twist.angular.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorSimpleController::update()
{
  // Get new commands/state
  callback_queue_.callAvailable();

  double dt;
  if (control_timer_.update(dt) && dt > 0.0) {
    updatePIDs(dt);

    math::Vector3 force;
    force.x = controllers_.vx.GetCmd();
    force.y = controllers_.vy.GetCmd();
    force.z = controllers_.vz.GetCmd();
    link_->SetForce(force * invMass);

    math::Vector3 torque(0.0, 0.0, controllers_.yaw.GetCmd());
    link_->SetTorque(torque * invZInertia);
  }
}

//---------------------------------------------------------------------------------------------------
void GazeboQuadrotorSimpleController::updatePIDs(double _dt) {
  // Linear speeds
  auto vel_error = velocity_command_ - cur_vel_;
  controllers_.vx.Update(vel_error.x, _dt);
  controllers_.vy.Update(vel_error.y, _dt);
  controllers_.vz.Update(vel_error.z, _dt);

  // Yaw speed
  double yaw_error = yaw_command_ - cur_yaw_spd_;
  controllers_.yaw.Update(yaw_error, _dt);
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorSimpleController::reset()
{
  controllers_.yaw.Reset();
  controllers_.vx.Reset();
  controllers_.vy.Reset();
  controllers_.vz.Reset();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorSimpleController)

} // namespace gazebo
