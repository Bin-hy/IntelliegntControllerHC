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
#include "rclcpp/rclcpp.hpp"
#include "duco_ros_driver/DucoTrajectoryAction.h"
#include "duco_ros_driver/utils.h"
#include "duco_ros_driver/utils_.h"
#include "duco_ros_driver/param_utils.h"
#include "duco_ros_driver/utils.h"
#include <map>
#include <functional>

using namespace std;
namespace duco_driver
{

const double DucoTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
const double DucoTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.002;

DucoTrajectoryAction::DucoTrajectoryAction(std::string controller_name,const rclcpp::NodeOptions & options ):Node("DucoTrajectoryAction", options),
    controller_alive_(false)
{
    int arm_num_str;
    this->declare_parameter("arm_num",0);
    bool ret=this->get_parameter("arm_num",arm_num_str);
    if(ret)
        this->arm_num=arm_num_str;
    using namespace std::placeholders;

    active_goal_.resize(arm_num+1,nullptr);
    current_traj_.resize(arm_num+1);
    for(int i=0;i<arm_num;i++)
    {
        std::string server_name="arm_"+std::to_string(i+1)+"_controller/follow_joint_trajectory";
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server=rclcpp_action::create_server<FollowJointTrajectory>(
                    this,
                    server_name,
                    std::bind(&DucoTrajectoryAction::goalCB, this, std::placeholders::_1, std::placeholders::_2,i+1),
                    std::bind(&DucoTrajectoryAction::cancelCB, this, std::placeholders::_1,i+1),
                    std::bind(&DucoTrajectoryAction::handle_accepted, this, std::placeholders::_1,i+1));
        this->action_server_.push_back(server);
    }

    pub_trajectory_command_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 100);
    sub_trajectory_state_ = this->create_subscription<control_msgs::action::FollowJointTrajectory_Feedback>("feedback_states", 100, std::bind(&DucoTrajectoryAction::controllerStateCB, this,_1));
    //sub_robot_status_ = node_.subscribe("robot_status", 1, &DucoTrajectoryAction::robotStatusCB, this);
    trajectory_execution_subs_ =this->create_subscription<std_msgs::msg::String>("trajectory_execution_event", 1, std::bind(&DucoTrajectoryAction::trajectoryExecutionCallback,this,std::placeholders::_1));
    //watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &DucoTrajectoryAction::watchdog, this, true);
}

DucoTrajectoryAction::~DucoTrajectoryAction()
{
}

void DucoTrajectoryAction::trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "stop")
    {
        RCLCPP_INFO(this->get_logger(),"trajectory execution status: stop");
        // Marks the current goal as canceled.

        for(int i=0;i<active_goal_.size();i++)
        {
            if(active_goal_[i])
            {
                abortGoal(i+1);
            }
        }

    }
}



//void DucoTrajectoryAction::watchdog(const ros::TimerEvent &e)
//{
//  // Some debug logging
//  if (!last_trajectory_state_)
//  {
//    RCLCPP_DEBUG(this->get_logger(),("Waiting for subscription to joint trajectory state");
//  }

//  RCLCPP_WARN(this->get_logger(),"Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
//  controller_alive_ = false;

//  // Aborts the active goal if the controller does not appear to be active.
//  if (has_active_goal_)
//  {
//    // last_trajectory_state_ is null if the subscriber never makes a connection
//    if (!last_trajectory_state_)
//    {
//      RCLCPP_WARN(this->get_logger(),"Aborting goal because we have never heard a controller state message.");
//    }
//    else
//    {
//      ROS_WARN_STREAM("Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
//    }
//    abortGoal();
//  }
//}

rclcpp_action::GoalResponse DucoTrajectoryAction::goalCB(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal,int num)
{
    RCLCPP_INFO(this->get_logger(),"Received new goal");
    std::shared_ptr<const FollowJointTrajectory::Goal>  tmp_gh = goal;

    // reject all goals as long as we haven't heard from the remote controller
    if (!controller_alive_)
    {
        RCLCPP_ERROR(this->get_logger(),"Joint trajectory action rejected: waiting for (initial) feedback from controller");
        control_msgs::action::FollowJointTrajectory_Result  rslt;
        rslt.error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_GOAL;
        // no point in continuing: already rejected
        return rclcpp_action::GoalResponse::REJECT;

    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;


}

rclcpp_action::CancelResponse DucoTrajectoryAction::cancelCB(const std::shared_ptr<GoalHandleTrajectory> goal_handle,int num)
{
    std::shared_ptr<GoalHandleTrajectory>  tmp_gh = goal_handle;
    RCLCPP_DEBUG(this->get_logger(),"Received action cancel request");

    if(active_goal_[num-1]==goal_handle)
    {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_trajectory_command_->publish(empty);
        active_goal_[num-1]=nullptr;
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

void DucoTrajectoryAction::handle_accepted(const std::shared_ptr<DucoTrajectoryAction::GoalHandleTrajectory> goal_handle,int num)
{
    std::shared_ptr<const FollowJointTrajectory::Goal>  tmp_gh = goal_handle->get_goal();
    if (!tmp_gh->trajectory.points.empty())
    {
        if (true)
        {
            // Cancels the currently active goal.
            if (active_goal_[num-1])
            {
                RCLCPP_WARN(this->get_logger(),"Received new goal, canceling current goal");
                abortGoal(num);
            }

            active_goal_[num-1] = goal_handle;

            RCLCPP_INFO(this->get_logger(),"Publishing trajectory");

            current_traj_[num-1] = tmp_gh->trajectory;
            pub_trajectory_command_->publish(current_traj_[num-1]);

        }

    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Joint trajectory action failed on empty trajectory");
        auto  rslt=std::make_shared<control_msgs::action::FollowJointTrajectory_Result>();
        rslt->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_GOAL;
        goal_handle->canceled(rslt);

    }
}

void DucoTrajectoryAction::controllerStateCB(control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(),"Checking controller state feedback");
    control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr  last_trajectory_state= msg;
    controller_alive_ = true;

    for(int i=0;i<arm_num+1;i++)
    {
        if(!active_goal_[i])
        {
            continue;
        }

        if(current_traj_[i].points.empty())
        {
            continue;
        }
        if (withinGoalConstraints(last_trajectory_state, current_traj_[i]))
        {
            RCLCPP_INFO(this->get_logger(),"Inside goal constraints, stopped moving, return success for action");
            auto  rslt=std::make_shared<control_msgs::action::FollowJointTrajectory_Result>();
            rslt->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
            active_goal_[i]->succeed(rslt);
            active_goal_[i]=nullptr;
        }
    }

    //RCLCPP_DEBUG(this->get_logger(),"Checking goal constraints");

}

void DucoTrajectoryAction::abortGoal(int num)
{
    // Marks the current goal as aborted.
    auto  rslt=std::make_shared<control_msgs::action::FollowJointTrajectory_Result>();
    rslt->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_GOAL;
    active_goal_[num-1]->abort(rslt);
}

bool mapInsert(const std::string & key, double value, std::map<std::string, double> & mappings)
{
    bool rtn = false;

    std::pair<std::map<std::string, double>::iterator, bool> insert_rtn;

    insert_rtn = mappings.insert(std::make_pair(key, value));

    // The second value returned form insert is a boolean (true for success)
    if (!insert_rtn.second)
    {
        rtn = false;
    }
    else
    {
        rtn = true;
    }
    return rtn;

}


bool toMap(const std::vector<std::string> & keys, const std::vector<double> & values,
           std::map<std::string, double> & mappings)
{
    bool rtn;

    mappings.clear();

    if (keys.size() == values.size())
    {
        rtn = true;

        for (size_t i = 0; i < keys.size(); ++i)
        {
            rtn = mapInsert(keys[i], values[i], mappings);
            if (!rtn)
            {
                break;
            }
        }

    }
    else
    {
        rtn = false;
    }

    return rtn;
}

bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, double full_range)
{
    bool rtn = false;

    if ((keys.size() != rhs.size()) || (keys.size() != lhs.size()))
    {

        rtn = false;
    }
    else
    {
        // Calculating the half range causes some precision loss, but it's good enough
        double half_range = full_range / 2.0;
        rtn = true; // Assume within range, catch exception in loop below

        // This loop will not run for empty vectors, results in return of true
        for (size_t i = 0; i < keys.size(); ++i)
        {
            if (fabs(lhs.at(keys[i]) - rhs.at(keys[i])) > fabs(half_range))
            {
                rtn = false;
                break;
            }
        }

    }

    return rtn;
}


bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values, double full_range)
{
    bool rtn = false;
    std::map<std::string, double> lhs_map;
    std::map<std::string, double> rhs_map;

    if (toMap(lhs_keys, lhs_values, lhs_map) && toMap(rhs_keys, rhs_values, rhs_map))
    {
        rtn = isWithinRange(lhs_keys, lhs_map, rhs_map, full_range);
    }


    return rtn;
}

bool DucoTrajectoryAction::withinGoalConstraints(const control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg,
                                                  const trajectory_msgs::msg::JointTrajectory & traj )
{
    bool rtn = false;
    if (traj.points.empty())
    {
        RCLCPP_WARN(this->get_logger(),"Empty joint trajectory passed to check goal constraints, return false");
        rtn = false;
    }
    else
    {
        int last_point = traj.points.size() - 1;

        //    std::cout<<"last_trajectory_state"<<last_trajectory_state_->joint_names[0]<<","<<last_trajectory_state_->joint_names[1]<<","<<last_trajectory_state_->joint_names[2]
        //                                                    <<","<<last_trajectory_state_->joint_names[3]<<","<<last_trajectory_state_->joint_names[4]
        //                                                    <<","<<last_trajectory_state_->joint_names[5]<<","<<last_trajectory_state_->joint_names[6]<<std::endl;
        //    std::cout<<"traj.joint_names"<<traj.joint_names[0]<<","<<traj.joint_names[1]<<","<<traj.joint_names[2]
        //                                                    <<","<<traj.joint_names[3]<<","<<traj.joint_names[4]
        //                                                    <<","<<traj.joint_names[5]<<","<<traj.joint_names[6]<<std::endl;


        //    RCLCPP_INFO(this->get_logger(),"laset position,%f,%f,%f,%f,%f,%f,%f",last_trajectory_state_->actual.positions[0],last_trajectory_state_->actual.positions[1],last_trajectory_state_->actual.positions[2],last_trajectory_state_->actual.positions[3],
        //            last_trajectory_state_->actual.positions[4],last_trajectory_state_->actual.positions[5],last_trajectory_state_->actual.positions[6]);

        //    RCLCPP_INFO(this->get_logger(),"current position,%f,%f,%f,%f,%f,%f,%f",traj.points[last_point].positions[0],traj.points[last_point].positions[1],traj.points[last_point].positions[2],traj.points[last_point].positions[3],
        //            traj.points[last_point].positions[4],traj.points[last_point].positions[5],traj.points[last_point].positions[6]);

        if (isWithinRange(msg->joint_names,
                          msg->actual.positions, traj.joint_names,
                          traj.points[last_point].positions, goal_threshold_))
        {
            rtn = true;
        }
        else
        {
            rtn = false;
        }
    }
    return rtn;
}

} //industrial_robot_client

using namespace duco_driver;
/** This node should be loaded after the robot description**/
int main(int argc, char** argv)
{
    // initialize node
    rclcpp::init(argc, argv);
    std::shared_ptr<DucoTrajectoryAction> action=std::make_shared<DucoTrajectoryAction>("gcr5");
    action->run();

    return 0;
}
