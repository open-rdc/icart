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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

#include <gazebo_ros_control/robot_hw_sim.h>

#include <urdf/model.h>

//#include <safety_interface/safety_interface.h>

namespace icart_mini_gazebo
{

    class ICartMiniHWSim : public gazebo_ros_control::RobotHWSim
    {
    private:
        static const double max_drive_joint_torque_ = 20.0;
        
        double cmd_[2];
        double pos_[2];
        double vel_[2];
        double eff_[2];
        
        gazebo::physics::JointPtr joint_[2];
        
        hardware_interface::JointStateInterface js_interface_;
        hardware_interface::VelocityJointInterface vj_interface_;
        //safety_interface::SafetyInterface safety_interface_;

    public:
        bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
            const urdf::Model* const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions)
        {
            pos_[0] = 0.0; pos_[1] = 0.0;
            vel_[0] = 0.0; vel_[1] = 0.0;
            eff_[0] = 0.0; eff_[1] = 0.0;
            cmd_[0] = 0.0; cmd_[1] = 0.0;

            std::string joint_namespace = robot_namespace.substr(1); //remove leading slash
            
            std::cout << "joint_namespace = " << joint_namespace << std::endl;

            joint_[0] = parent_model->GetJoint("right_wheel_hinge");
            joint_[1] = parent_model->GetJoint("left_wheel_hinge");

            //joint_[0] = parent_model->GetJoint(joint_namespace + "/right_wheel_hinge");
            //joint_[1] = parent_model->GetJoint(joint_namespace + "/left_wheel_hinge");
            
            js_interface_.registerHandle(
                //hardware_interface::JointStateHandle(joint_namespace + "/right_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]));
                hardware_interface::JointStateHandle("right_wheel_hinge", &pos_[0], &vel_[0], &eff_[0]));
            js_interface_.registerHandle(
                //hardware_interface::JointStateHandle(joint_namespace + "/left_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]));
                hardware_interface::JointStateHandle("left_wheel_hinge", &pos_[1], &vel_[1], &eff_[1]));
            
            vj_interface_.registerHandle(
                //hardware_interface::JointHandle(js_interface_.getHandle(joint_namespace + "/right_wheel_hinge"), &cmd_[0]));
                hardware_interface::JointHandle(js_interface_.getHandle("right_wheel_hinge"), &cmd_[0]));
            
            vj_interface_.registerHandle(
                //hardware_interface::JointHandle(js_interface_.getHandle(joint_namespace + "/right_wheel_hinge"), &cmd_[1]));
                hardware_interface::JointHandle(js_interface_.getHandle("left_wheel_hinge"), &cmd_[1]));

            registerInterface(&js_interface_);
            registerInterface(&vj_interface_);

            //registerInterface(&safety_interface_);
            
            return true;
        }

        void readSim(ros::Time time, ros::Duration period){
            for(int i=0; i < 2; i++){
                pos_[i] += angles::shortest_angular_distance(pos_[i], joint_[i]->GetAngle(0).Radian());
                vel_[i] = joint_[i]->GetVelocity(0);
                eff_[i] = joint_[i]->GetForce((unsigned int)(0));
            }
        }

        void writeSim(ros::Time time, ros::Duration period){
            for(int i=0; i < 2; i++){
                joint_[i]->SetVelocity(0, cmd_[i]);
                joint_[i]->SetMaxForce(0, max_drive_joint_torque_);
            }
            
            /*
            if(safety_interface_.get_state() == safety_interface::safety_state::OK){
                for(int i=0; i < 2; i++){
                    joint_[i]->SetVelocity(0, cmd_[i]);
                    joint_[i]->SetMaxForce(0, max_drive_joint_torque_);
                }
            }else{
                for(int i=0; i < 2; i++) joint_[i]->SetVelocity(0, 0);
            }
            */
        }
    };

    typedef boost::shared_ptr<ICartMiniHWSim> ICartMiniHWSimPtr;
}

PLUGINLIB_EXPORT_CLASS(icart_mini_gazebo::ICartMiniHWSim, gazebo_ros_control::RobotHWSim)

