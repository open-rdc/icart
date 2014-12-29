/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Daiki Maekawa and Robot Design and Control Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

extern "C"{
#include <unistd.h>
#include <math.h>
#include <ypspur.h>
}

class TFrog : public hardware_interface::RobotHW
{
public:
  TFrog(){
    if(Spur_init() < 0){
      ROS_WARN_STREAM("ERROR : cannot open spur.\n");
    }

    YP_set_wheel_vel(13.0, 13.0);
    YP_set_wheel_accel(13.0, 13.0);

    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("right_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("left_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("right_wheel_hinge"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("left_wheel_hinge"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
  }

  ~TFrog(){
    Spur_stop();
    ros::Duration(1);
    //usleep(40000000);
    Spur_free();
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}
  
  void reopen(){
    YP_wheel_vel(0, 0);
    Spur_free();
    Spur_init();
    YP_set_wheel_vel(13.0, 13.0);
    YP_set_wheel_accel(13.0, 13.0);
  }

  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << -cmd_[1]);
    int ret = YP_wheel_vel(cmd_[1], -cmd_[0]);
  }

  void write(){
    double yp_vel[2];
    yp_vel[0] = 0;
    yp_vel[1] = 0;
    YP_get_wheel_vel(&yp_vel[1], &yp_vel[0]);
    yp_vel[0] = -yp_vel[0];
    //ROS_INFO_STREAM("YPSpur vel: " << yp_vel[0] << ", " << -yp_vel[1]);

    for (unsigned int i = 0; i < 2; ++i)
    {
      pos_[i] += yp_vel[i] * getPeriod().toSec();
      vel_[i] = yp_vel[i];
    }
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
};

int main(int argc, char **argv)
{
  double x, y, theta;
  
  ros::init(argc, argv, "icart_mini");
  ros::NodeHandle nh;
    
  TFrog robot;
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    int state = YP_get_error_state();
    
    if(state == 0){
        robot.read();
        robot.write();
    }else{
        ROS_WARN("Disconnected T-frog driver");
        robot.reopen();
    }
    
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
