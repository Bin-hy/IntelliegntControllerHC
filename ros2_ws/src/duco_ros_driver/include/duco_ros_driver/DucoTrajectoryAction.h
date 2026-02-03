/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#ifndef DUCOTRAJECTORYACTION_H
#define DUCOTRAJECTORYACTION_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "duco_msg/msg/duco_robot_state.hpp"
#include "duco_msg/srv/duco_robot_states.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <thread>
#include <std_msgs/msg/string.hpp>


namespace duco_driver
{

class DucoTrajectoryAction: public rclcpp::Node
{

public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  /**
   * \brief Constructor
   *
   */
  DucoTrajectoryAction(std::string controller_name,const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * \brief Destructor
   *
   */
  ~DucoTrajectoryAction();

  /**
     * \brief Begin processing messages and publishing topics.
     */
    void run() { rclcpp::spin(this->shared_from_this()); }
    void abort() { rclcpp::shutdown();}

private:



  /**
   * \brief Internal action server
   */
  std::vector<rclcpp_action::Server<FollowJointTrajectory>::SharedPtr> action_server_;

  /**
   * \brief Publishes desired trajectory (typically to the robot driver)
   */
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_trajectory_command_;

  /**
   * \brief Subscribes to trajectory feedback (typically published by the
   * robot driver).
   */
  rclcpp::Subscription<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr sub_trajectory_state_;

  /**
   * \brief Subscribes to the robot status (typically published by the
   * robot driver).
   */
  rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_robot_status_;

  /**
   * \brief Subscribes to the trajectory execution status (typically published by the
   * user).
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_subs_;

  /**
   * \brief Watchdog time used to fail the action request if the robot
   * driver is not responding.
   */
  //rclcpp::Timer watchdog_timer_;

  /**
    * \brief Controller was alive during the last watchdog interval
    */
  bool controller_alive_;



  /**
   * \brief Cache of the current active goal
   */
  std::vector<std::shared_ptr<GoalHandleTrajectory> > active_goal_;
  /**
   * \brief Cache of the current active trajectory
   */
  std::vector<trajectory_msgs::msg::JointTrajectory> current_traj_;

  /**
   * \brief The default goal joint threshold see(goal_threshold). Unit
   * are joint specific (i.e. radians or meters).
   */
  static const double DEFAULT_GOAL_THRESHOLD_;// = 0.01;

  /**
   * \brief The goal joint threshold used for determining if a robot
   * is near it final destination.  A single value is used for all joints
   *
   * NOTE: This value is used in conjunction with the robot inMotion
   * status (see industrial_msgs::RobotStatus) if it exists.
   */
  double goal_threshold_=0.01;

  /**
   * \brief The joint names associated with the robot the action is
   * interfacing with.  The joint names must be the same as expected
   * by the robot driver.
   */
  std::vector<std::string> joint_names_;

  /**
   * \brief Cache of the last subscribed feedback message
   */
  control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr last_trajectory_state_;

  /**
   * \brief Time at which to start checking for completion of current 
   * goal, if one is active
   */
  rclcpp::Time time_to_check_;

  /**
   * \brief The watchdog period (seconds)
   */
  static const double WATCHDOG_PERIOD_;// = 1.0;

  /**
   * \brief Watch dog callback, used to detect robot driver failures
   *
   * \param e time event information
   *
   */
  //void watchdog(const rclcpp::TimerEvent &e);

  /**
   * \brief Action server goal callback method
   *
   * \param gh goal handle
   *
   */
  rclcpp_action::GoalResponse goalCB(const rclcpp_action::GoalUUID & uuid,
               std::shared_ptr<const FollowJointTrajectory::Goal> goal, int num);

  /**
   * \brief Action server cancel callback method
   *
   * \param gh goal handle
   *
   */

  rclcpp_action::CancelResponse cancelCB(const std::shared_ptr<GoalHandleTrajectory> goal_handle, int num);

  void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle, int num);

  /**
   * \brief Controller state callback (executed when feedback message
   * received)
   *
   * \param msg joint trajectory feedback message
   *
   */
  void controllerStateCB(control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg);


  /**
   * @brief trajectoryExecutionCallback
   * @param msg trajectory execution status
   */
  void trajectoryExecutionCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * \brief Aborts the current action goal and sends a stop command
   * (empty message) to the robot driver.
   *
   *
   */
  void abortGoal(int num);

  /**
   * \brief Controller status callback (executed when robot status
   *  message received)
   *
   * \param msg trajectory feedback message
   * \param traj trajectory to test against feedback
   *
   * \return true if all joints are within goal contraints
   *
   */
  bool withinGoalConstraints(const control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg,
                             const trajectory_msgs::msg::JointTrajectory & traj );

  int arm_num=1;

};

} //duco_driver

#endif /* JOINT_TRAJTORY_ACTION_H */

