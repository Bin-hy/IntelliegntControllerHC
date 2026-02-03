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
#ifndef DUCODRIVER_H
#define DUCODRIVER_H

#include <string>
#include <sys/timeb.h>
#include <queue>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "duco_msg/msg/duco_robot_state.hpp"
#include "duco_msg/srv/duco_robot_states.hpp"

#include <trajectory_msgs/msg/joint_trajectory.h>
#include <std_msgs/msg/string.hpp>

namespace DucoRPC
{
class DucoCobot;
};


namespace duco_driver
{
class DucoDriver: public rclcpp::Node
{
    /*定时器频率*/
    const int TIMER_SPAN_ = 50;
    /*关节比较阈值*/
    const double THRESHHOLD = 0.000001;

public:

    /*构造函数*/
    DucoDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /*析构函数*/
    ~DucoDriver();

    /*更新机器人控制状态，比如机器人是否启动运行*/
    void updateControlStatus();

protected:

    /*RPC连接服务*/
    bool  connectToRobotController();

    /*根据MoveIt下发机器人关节数据*/
    bool setRobotJointsByMoveIt();

private:

    /*获取当前机器人状态服务*/
    bool getRobotStates(duco_msg::srv::DucoRobotStates::Request &req,duco_msg::srv::DucoRobotStates::Response &res);

private:
    /* 获取机器人当前位置状态 */
    void robotStateCallback(const duco_msg::msg::DucoRobotState &msg );


    /*根据MoveIt规划执行*/
    void moveItPosCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) ;

    /*监控机器人停止信号*/
    void trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_subs_;

private://服务节点
    /*robot status server*/
    //rclcpp::ServiceServer duco_states_srv_;


private://订阅节点

    /*moveItController_cmd 机器人 通过Moveit控制 订阅节点*/
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr moveit_controller_subs_;

    /*robot_states_subs_ 机器人状态 订阅节点*/
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr robot_states_subs_;

    double                   robot_speed_=1.5;

private:

    /*互*/
    std::mutex     io_mutex_;

    std::mutex     robot_status_mutex_;

    std::vector<bool>                  joint_receive_finish_ ;

    /*主机IP地址*/
    std::string server_host_="192.168.120.138";



    /*碰撞等级*/
    int collision_class_;

    /*机器人RPC接口*/
    std::vector<std::shared_ptr<DucoRPC::DucoCobot> > duco_robots;
    //DucoRPC::DucoCobot       *duco_robot_ = nullptr;


    std::vector<std::vector<std::vector<double> > > joint_track_;

    std::vector<std::vector<std::string> >     joint_names_;


    /*机器人状态*/
    duco_msg::msg::DucoRobotState  robot_state_;


    /*机器人急停标志*/
    bool emergency_stopped_=false;

    /*机器人保护停止标志*/
    bool protective_stopped_=false;

    /*机器人正常停止规划标志 OTG*/
    bool normal_stopped_=false;

    /*当前机器人是否连接标志*/
    bool controller_connected_flag_=false;

    /*当前机器人是否启动运行标志*/
    bool start_move_=false;

    /*机器人关节名称*/

    int arm_num=1;

};
}

#endif /* DUCODRIVER_H */
