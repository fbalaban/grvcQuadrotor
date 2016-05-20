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

#ifndef HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
#define HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H

#include <gazebo/common/Plugin.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <hector_gazebo_plugins/update_timer.h>
#include <gazebo/common/PID.hh>

namespace gazebo
{

/// Gazebo plugin for quadrotor control. Expects to receive commands in a velocity commands topic specified inside the urdf
/// \remark Velocity commands are expected in local axes.
/// \remark Yaw actually controls yaw speed. The quad doesn't have any absolute reference for position or yaw
class GazeboQuadrotorSimpleController : public ModelPlugin
{
public:
  GazeboQuadrotorSimpleController();
  virtual ~GazeboQuadrotorSimpleController();

protected:
  void parseSdfParams(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void loadPID(sdf::ElementPtr _sdf, const char* _paramName, common::PID& _pid);
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void subscribeTopics();
  void update();
  void updatePIDs(double _dt);
  void reset();

private:
  // Gazebo stuff
  physics::WorldPtr world_; ///< \brief The parent World
  physics::LinkPtr link_; ///< \brief The link referred to by this plugin
  UpdateTimer control_timer_;
  event::ConnectionPtr  update_connection_;

  // ROS Stuff
  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber state_subscriber_;

  std::string link_name_;
  std::string namespace_;
  std::string velocity_topic_;
  std::string state_topic_;

  // Callbacks
  void velocityCallback(const geometry_msgs::TwistConstPtr&);
  void stateCallback(const nav_msgs::OdometryConstPtr&);

  // Control
  struct Controllers {
    common::PID yaw; ///< Controller for yaw speed. Absolute yaw is not known by the quad
    common::PID vx;
    common::PID vy;
    common::PID vz;
  } controllers_;

  math::Vector3 velocity_command_;
  double yaw_command_;

  double invMass;
  double invZInertia;

  // Internal state
  math::Vector3 cur_vel_;
  double cur_yaw_spd_;
};

}

#endif // HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
