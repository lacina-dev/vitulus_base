/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.h
*/

/*
    Modified for VITULUS and 4 wheels: lacina.dev
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
// ostringstream
#include <sstream>
#include <cmath>

const unsigned int NUM_JOINTS = 4;  // 2

/// \brief Hardware interface for a robot
class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
  MyRobotHWInterface();

  /*
   *
   */
  void write() {
    double diff_ang_speed_front_left = cmd[0];
    double diff_ang_speed_front_right = cmd[1];
    double diff_ang_speed_rear_left = cmd[2];
    double diff_ang_speed_rear_right = cmd[3];

    limitDifferentialSpeed(diff_ang_speed_front_left, diff_ang_speed_front_right);
    limitDifferentialSpeed(diff_ang_speed_rear_left, diff_ang_speed_rear_right);

	std_msgs::Float32 front_left_wheel_vel_msg;
	std_msgs::Float32 front_right_wheel_vel_msg;
	std_msgs::Float32 rear_left_wheel_vel_msg;
	std_msgs::Float32 rear_right_wheel_vel_msg;
	front_left_wheel_vel_msg.data = diff_ang_speed_front_left;
	front_right_wheel_vel_msg.data = diff_ang_speed_front_right;
	rear_left_wheel_vel_msg.data = diff_ang_speed_rear_left;
	rear_right_wheel_vel_msg.data = diff_ang_speed_rear_right;

	front_left_wheel_vel_pub_.publish(front_left_wheel_vel_msg);
	front_right_wheel_vel_pub_.publish(front_right_wheel_vel_msg);
	rear_left_wheel_vel_pub_.publish(rear_left_wheel_vel_msg);
	rear_right_wheel_vel_pub_.publish(rear_right_wheel_vel_msg);
  }

  /**
   * Reading encoder values and setting position and velocity of enconders 
   */
  void read(const ros::Duration &period) {
    // Continuity guard against motor-enable encoder discontinuities. On motor
    // power-up the wheel controllers re-initialise their absolute angle, so
    // _wheel_angle takes a one-cycle non-physical jump; assigning it straight to
    // pos[] made diff_drive_controller derive a huge odom velocity spike (observed
    // -193..-550 m/s, growing with distance driven) that teleported the fused
    // pose. Fix: integrate pos[] from per-cycle angle deltas and DROP any delta
    // beyond the physical maximum (max wheel speed * period). Normal motion is far
    // below the cap, so real odometry is unaffected (zero impact); only the
    // power-up glitch is rejected, keeping pos[] continuous (no pose jump).
    double dt = period.toSec();
    if (dt < 0.02) dt = 0.02;                    // floor to nominal 50 Hz cycle
    const double max_delta = _max_speed * dt * 3.0;   // generous physical cap [rad]

    for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
      double raw = _wheel_angle[i];
      if (!_angle_init) {
        pos[i] = raw;
      } else {
        double d = raw - _prev_wheel_angle[i];
        if (std::fabs(d) > max_delta) {
          ROS_WARN_THROTTLE(1.0,
            "[base_hw] rejected non-physical wheel %u angle jump %.2f rad (cap %.2f) "
            "- likely motor-enable encoder reset", i, d, max_delta);
          d = 0.0;                               // freeze this cycle, re-sync below
        }
        pos[i] += d;
      }
      _prev_wheel_angle[i] = raw;
      vel[i] = _wheel_real_vel[i];
    }
    _angle_init = true;
  }

  ros::Time get_time() {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period() {
    return curr_update_time - prev_update_time;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];
  double _wheel_real_vel[NUM_JOINTS];
  double _prev_wheel_angle[NUM_JOINTS];   // continuity-guard reference (see read())
  bool _angle_init;

  ros::Time curr_update_time, prev_update_time;

  ros::Subscriber front_left_wheel_angle_sub_;
  ros::Subscriber front_right_wheel_angle_sub_;
  ros::Subscriber rear_left_wheel_angle_sub_;
  ros::Subscriber rear_right_wheel_angle_sub_;

  ros::Subscriber front_left_wheel_real_vel_sub_;
  ros::Subscriber front_right_wheel_real_vel_sub_;
  ros::Subscriber rear_left_wheel_real_vel_sub_;
  ros::Subscriber rear_right_wheel_real_vel_sub_;

  ros::Publisher front_left_wheel_vel_pub_;
  ros::Publisher front_right_wheel_vel_pub_;
  ros::Publisher rear_left_wheel_vel_pub_;
  ros::Publisher rear_right_wheel_vel_pub_;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  { 
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

  void frontLeftWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[0] = msg.data;
  }

  void frontRightWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[1] = msg.data;
  }

  void rearLeftWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[2] = msg.data;
  }

  void rearRightWheelAngleCallback(const std_msgs::Float32& msg) {
    _wheel_angle[3] = msg.data;
  }



  void frontLeftWheelRealVelCallback(const std_msgs::Float32& msg) {
    _wheel_real_vel[0] = msg.data;
  }

  void frontRightWheelRealVelCallback(const std_msgs::Float32& msg) {
    _wheel_real_vel[1] = msg.data;
  }

  void rearLeftWheelRealVelCallback(const std_msgs::Float32& msg) {
    _wheel_real_vel[2] = msg.data;
  }

  void rearRightWheelRealVelCallback(const std_msgs::Float32& msg) {
    _wheel_real_vel[3] = msg.data;
  }


  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
	double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
	if (speed > _max_speed) {
		diff_speed_left *= _max_speed / speed;
		diff_speed_right *= _max_speed / speed;
	}
  }

};  // class

MyRobotHWInterface::MyRobotHWInterface()
: running_(true)
  , private_nh("~")
  , start_srv_(nh.advertiseService("start", &MyRobotHWInterface::start_callback, this))
  , stop_srv_(nh.advertiseService("stop", &MyRobotHWInterface::stop_callback, this)) 
  {
    private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.204);
    private_nh.param<double>("max_speed", _max_speed, 35);
  
    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);
    std::fill_n(_prev_wheel_angle, NUM_JOINTS, 0.0);
    _angle_init = false;   // first read() seeds pos[] from the raw angle

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << "wheel_" << i << "_joint";

      hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
      jnt_state_interface.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(os.str()), &cmd[i]);
      jnt_vel_interface.registerHandle(vel_handle);
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

	// Initialize publishers and subscribers
	front_left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("base/front_left_wheel_vel", 1);
	front_right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("base/front_right_wheel_vel", 1);
	rear_left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("base/rear_left_wheel_vel", 1);
	rear_right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("base/rear_right_wheel_vel", 1);

	front_left_wheel_angle_sub_ = nh.subscribe("base/front_left_wheel_angle", 1, &MyRobotHWInterface::frontLeftWheelAngleCallback, this);
	front_right_wheel_angle_sub_ = nh.subscribe("base/front_right_wheel_angle", 1, &MyRobotHWInterface::frontRightWheelAngleCallback, this);
	rear_left_wheel_angle_sub_ = nh.subscribe("base/rear_left_wheel_angle", 1, &MyRobotHWInterface::rearLeftWheelAngleCallback, this);
	rear_right_wheel_angle_sub_ = nh.subscribe("base/rear_right_wheel_angle", 1, &MyRobotHWInterface::rearRightWheelAngleCallback, this);

	front_left_wheel_real_vel_sub_ = nh.subscribe("base/front_left_wheel_real_vel", 1, &MyRobotHWInterface::frontLeftWheelRealVelCallback, this);
	front_right_wheel_real_vel_sub_ = nh.subscribe("base/front_right_wheel_real_vel", 1, &MyRobotHWInterface::frontRightWheelRealVelCallback, this);
	rear_left_wheel_real_vel_sub_ = nh.subscribe("base/rear_left_wheel_real_vel", 1, &MyRobotHWInterface::rearLeftWheelRealVelCallback, this);
	rear_right_wheel_real_vel_sub_ = nh.subscribe("base/rear_right_wheel_real_vel", 1, &MyRobotHWInterface::rearRightWheelRealVelCallback, this);
}
