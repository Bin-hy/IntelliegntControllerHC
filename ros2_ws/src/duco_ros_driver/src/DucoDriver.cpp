/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, SIASUN Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "duco_ros_driver/DucoDriver.h"
#include "duco_ros_driver/DucoCobot.h"
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
using namespace DucoRPC;

namespace duco_driver {

const unsigned int  IOMAXNUM = 16;
const unsigned int  TOOLIOMAXNUM = 2;


DucoDriver::DucoDriver(const rclcpp::NodeOptions &options):Node("DucoDriver", options),
    emergency_stopped_(false)
    ,protective_stopped_(false)
    ,normal_stopped_(false)
    ,controller_connected_flag_(false)
    ,start_move_(false)
    ,collision_class_(6)
{
    /** subscribe topics **/
    /*订阅来自DucoRobotStatus的消息*/
    robot_states_subs_ = this->create_subscription<duco_msg::msg::DucoRobotState>("/duco_cobot/robot_state", 10, std::bind(&DucoDriver::robotStateCallback, this,std::placeholders::_1));

    /*订阅来自MoveGroup的消息*/
    trajectory_execution_subs_ =this->create_subscription<std_msgs::msg::String>("trajectory_execution_event", 1, std::bind(&DucoDriver::trajectoryExecutionCallback,this,std::placeholders::_1));
    moveit_controller_subs_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 2000, std::bind(&DucoDriver::moveItPosCallback,this,std::placeholders::_1));

    int arm_num_str;
    this->declare_parameter("arm_num",0);
    bool ret=this->get_parameter("arm_num",arm_num_str);
    if(ret)
        this->arm_num=arm_num_str;
    for(int i=0;i<arm_num;i++)
    {
        std::string host="server_host_"+std::to_string(i+1);
        std::string ip="127.0.0.1";
        this->declare_parameter(host,"127.0.0.1");
        this->get_parameter(host,ip);
        std::shared_ptr<DucoRPC::DucoCobot >duco_robot = std::make_shared<DucoRPC::DucoCobot>(ip, 7003);
        RCLCPP_INFO(this->get_logger(), "DucoDriver::moveItPosCallback msg is empty %s",ip.c_str());

        duco_robots.push_back(duco_robot);
    }
    joint_track_.resize(arm_num);
    joint_names_.resize(arm_num);
    joint_receive_finish_.resize(arm_num);
    for(int i=0;i<arm_num;i++)
    {
        joint_receive_finish_[i]=false;
        std::string pre="arm_"+std::to_string(i+1);
        for(int j=0;j<6;j++)
        {
            joint_names_[i].push_back(pre+"_"+"joint_"+std::to_string(j+1));
        }
    }
}

DucoDriver::~DucoDriver()
{
    for(int i=0;i<duco_robots.size();i++)
    {
        duco_robots[i]->close();
    }
}

bool DucoDriver::connectToRobotController()
{
    for(int i=0;i<arm_num;i++)
    {
        if(duco_robots[i]->open() != 0)
        {
            controller_connected_flag_=false;
            return false;
        }
    }
    controller_connected_flag_=true;
    for(int i=0;i<arm_num;i++)
    {

        duco_robots[i]->power_on(true);
        usleep(1000000);
        duco_robots[i]->enable( true );
        usleep(1000000);
    }
    return true;
}


bool DucoDriver::getRobotStates(duco_msg::srv::DucoRobotStates::Request &req,duco_msg::srv::DucoRobotStates::Response &res)
{
    res.robot_states = robot_state_;
    return true;
}

bool DucoDriver::setRobotJointsByMoveIt()
{
    bool ret = true;
    for(int arm=0;arm<arm_num;arm++)
    {
        if(!joint_track_[arm].empty()&& joint_receive_finish_[arm] )
        {

            joint_receive_finish_[arm] = false;
            if(  2==joint_track_[arm].size() )
            {
                std::vector<double> joint2;
                for (size_t i = 0; i < 6; i++)
                {
                    double tmp = (joint_track_[arm][0][i] + joint_track_[arm][1][i])/2;
                    joint2.push_back(tmp);
                }
                std::vector<double>joint1 = joint_track_[arm][0];
                std::vector<double>joint3 = joint_track_[arm][1];

                joint_track_[arm].clear();
                joint_track_[arm].push_back(joint1);
                joint_track_[arm].push_back(joint2);
                joint_track_[arm].push_back(joint3);
            }

            if( controller_connected_flag_ )      // actually no need this judgment
            {
                joint_track_[arm].erase(joint_track_[arm].begin());
                for(int i=0;i<joint_track_[arm].size();i++)
                {
                    for(int j=0;j<joint_track_[arm][i].size();j++)
                    {
                        std::cout<<joint_track_[arm][i][j]<<" ";

                    }
                    std::cout<<std::endl;
                }
                duco_robots[arm]->trackClearQueue();
                duco_robots[arm]->trackEnqueue( joint_track_[arm], true);
                if( !start_move_ )
                {
                    RCLCPP_INFO(this->get_logger(), "Start Move robot_speed_ %.6f", robot_speed_);
                    duco_robots[arm]->trackJointMotion(robot_speed_, 1,false);
                }
                joint_track_[arm].clear();
            }
        }

    }
    return ret;
}


void DucoDriver::moveItPosCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if(msg->points.size() == 0 )
    {
        RCLCPP_INFO(this->get_logger(), "DucoDriver::moveItPosCallback msg is empty");
    }
    else
    {
        if( !controller_connected_flag_ )
        {
            RCLCPP_ERROR(this->get_logger(), "DucoDriver::moveItPosCallback error,Robot not connected!" );
            return;
        }

        RCLCPP_INFO(this->get_logger(), "DucoDriver::moveItPosCallback size %d", static_cast<int>(msg->points.size()) );
        for(int arm=0;arm<arm_num;arm++)
        {
            for(int i = 0; i < msg->points.size(); i++)
            {
                // first remaps point to controller joint order, the add the point to the controller.
                trajectory_msgs::msg::JointTrajectoryPoint point = msg->points[i];
                std::vector<double> joints;
                joints.clear();

                for(int j = 0; j < 6; j++)
                {
                    for(int k = 0; k < msg->joint_names.size(); k++)
                    {
                        if(joint_names_[arm][j] == msg->joint_names[k])
                        {
                            RCLCPP_INFO(this->get_logger(), "-----------------%s,%s",joint_names_[arm][j].c_str(),msg->joint_names[k].c_str());

                            joints.push_back(msg->points[i].positions[k]);
                            break;
                        }
                    }
                }
                if(!joints.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "DucoDriver::moveItPosCallback joints i %d %.6f %.6f %.6f  %.6f %.6f %.6f ", i,  joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
                    joint_track_[arm].push_back(joints) ;
                }
            }

            if(!joint_track_[arm].empty())
            {
                joint_receive_finish_[arm]=true;
            }
        }
    }
}

void DucoDriver::robotStateCallback(const duco_msg::msg::DucoRobotState& msg )
{

    std::lock_guard<std::mutex> lock( robot_status_mutex_ );
    robot_state_ = msg;
}



void DucoDriver::trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "stop")
    {
        RCLCPP_INFO(this->get_logger(), "trajectory execution status: stop");
        normal_stopped_ = true;
    }
}

void DucoDriver::updateControlStatus()
{
    if( !controller_connected_flag_ )
    {
        connectToRobotController();
    }

    if( controller_connected_flag_ )
    {
        setRobotJointsByMoveIt();
    }

    if( emergency_stopped_ )
    {
        RCLCPP_INFO(this->get_logger(), "DucoDriver::updateControlStatus emergency_stopped_  %d" ,  emergency_stopped_);
        //clear the buffer, there will be a jerk
        joint_track_.clear();
    }
    else if( normal_stopped_ )
    {
        RCLCPP_INFO(this->get_logger(), "DucoDriver::updateControlStatus emergency_stopped_  %d" ,  normal_stopped_);

        if( controller_connected_flag_ )
        {
            for(int i=0;i<arm_num;i++)            {
                duco_robots[i]->stop(false);
            }
        }

        joint_track_.clear();
        normal_stopped_ = false;
        protective_stopped_ = false;
    }
}



}

