// src/hand_control_service.hpp
#pragma once

#include <LHandProLib/LHandProLib.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lhandpro_interfaces/srv/get_control_mode.hpp"
#include "lhandpro_interfaces/srv/get_max_current.hpp"
#include "lhandpro_interfaces/srv/get_now_angle.hpp"
#include "lhandpro_interfaces/srv/get_now_current.hpp"
#include "lhandpro_interfaces/srv/get_now_position.hpp"
#include "lhandpro_interfaces/srv/get_now_position_velocity.hpp"
#include "lhandpro_interfaces/srv/get_position.hpp"
#include "lhandpro_interfaces/srv/get_position_velocity.hpp"
#include "lhandpro_interfaces/srv/home_motors.hpp"
#include "lhandpro_interfaces/srv/move_motors.hpp"
#include "lhandpro_interfaces/srv/set_control_mode.hpp"
#include "lhandpro_interfaces/srv/set_enable.hpp"
#include "lhandpro_interfaces/srv/set_max_current.hpp"
#include "lhandpro_interfaces/srv/set_position.hpp"
#include "lhandpro_interfaces/srv/set_position_velocity.hpp"

// 服务名称宏
#define SRV_NAME_SET_ENABLE "set_enable"
#define SRV_NAME_SET_POSITION "set_position"
#define SRV_NAME_GET_POSITION "get_position"
#define SRV_NAME_GET_NOW_ANGLE "get_now_angle"
#define SRV_NAME_GET_NOW_POSITION "get_now_position"
#define SRV_NAME_GET_NOW_POSITION_VELOCITY "get_now_position_velocity"
#define SRV_NAME_GET_NOW_CURRENT "get_now_current"
#define SRV_NAME_GET_POSITION_VELOCITY "get_position_velocity"
#define SRV_NAME_SET_POSITION_VELOCITY "set_position_velocity"
#define SRV_NAME_GET_MAX_CURRENT "get_max_current"
#define SRV_NAME_SET_MAX_CURRENT "set_max_current"
#define SRV_NAME_GET_CONTROL_MODE "get_control_mode"
#define SRV_NAME_SET_CONTROL_MODE "set_control_mode"
#define SRV_NAME_HOME_MOTORS "home_motors"
#define SRV_NAME_MOVE_MOTORS "move_motors"

// 服务注册宏
#define REGISTER_SERVICE(SrvType, SrvName, Callback)          \
  this->create_service<lhandpro_interfaces::srv::SrvType>(    \
      SrvName, std::bind(&HandControlService::Callback, this, \
                         std::placeholders::_1, std::placeholders::_2))

class EthercatMaster;

class HandControlService : public rclcpp::Node {
 public:
  HandControlService();
  ~HandControlService();

  void init_ethercat(int channel = 0);
  void cleanup_resources();
  void init_service();

  void check_and_reconnect();
  bool is_alive();

 private:
  bool check_joint_validity(int joint_id, const std::string& service_name);
  void start_monitor();
  void stop_monitor();

 private:
  void set_enable_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::SetEnable::Request> req,
      std::shared_ptr<lhandpro_interfaces::srv::SetEnable::Response> res);

  void set_position_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::SetPosition::Request> req,
      std::shared_ptr<lhandpro_interfaces::srv::SetPosition::Response> res);

  void get_position_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::GetPosition::Request> req,
      std::shared_ptr<lhandpro_interfaces::srv::GetPosition::Response> res);

  void get_now_angle_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::GetNowAngle::Request> req,
      std::shared_ptr<lhandpro_interfaces::srv::GetNowAngle::Response> res);

  void get_now_position_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::GetNowPosition::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::GetNowPosition::Response> res);

  void get_now_position_velocity_callback(
      const std::shared_ptr<
          lhandpro_interfaces::srv::GetNowPositionVelocity::Request>
          req,
      std::shared_ptr<
          lhandpro_interfaces::srv::GetNowPositionVelocity::Response>
          res);

  void get_now_current_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::GetNowCurrent::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::GetNowCurrent::Response> res);

  void get_position_velocity_callback(
      const std::shared_ptr<
          lhandpro_interfaces::srv::GetPositionVelocity::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::GetPositionVelocity::Response>
          res);

  void set_position_velocity_callback(
      const std::shared_ptr<
          lhandpro_interfaces::srv::SetPositionVelocity::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::SetPositionVelocity::Response>
          res);

  void get_max_current_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::GetMaxCurrent::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::GetMaxCurrent::Response> res);

  void set_max_current_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::SetMaxCurrent::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::SetMaxCurrent::Response> res);

  void get_control_mode_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::GetControlMode::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::GetControlMode::Response> res);

  void set_control_mode_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::SetControlMode::Request>
          req,
      std::shared_ptr<lhandpro_interfaces::srv::SetControlMode::Response> res);

  void home_motors_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::HomeMotors::Request> req,
      std::shared_ptr<lhandpro_interfaces::srv::HomeMotors::Response> res);

  void move_motors_callback(
      const std::shared_ptr<lhandpro_interfaces::srv::MoveMotors::Request> req,
      std::shared_ptr<lhandpro_interfaces::srv::MoveMotors::Response> res);

 private:
  std::shared_ptr<lhplib::LHandProLib> lhp_lib_;
  std::shared_ptr<EthercatMaster> ec_master_;
  std::function<bool(const unsigned char*, unsigned int)> send_function_;
  int active_dof_{0};
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  std::atomic<bool> is_connected_;
  int current_channel_{0};
  std::atomic<bool> stop_flag_{false};
  std::thread monitor_thread_;

  rclcpp::Service<lhandpro_interfaces::srv::SetEnable>::SharedPtr
      set_enable_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::SetPosition>::SharedPtr
      set_position_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetPosition>::SharedPtr
      get_position_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetNowAngle>::SharedPtr
      get_now_angle_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetNowPosition>::SharedPtr
      get_now_position_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetNowPositionVelocity>::SharedPtr
      get_now_position_velocity_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetNowCurrent>::SharedPtr
      get_now_current_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetPositionVelocity>::SharedPtr
      get_position_velocity_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::SetPositionVelocity>::SharedPtr
      set_position_velocity_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetMaxCurrent>::SharedPtr
      get_max_current_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::SetMaxCurrent>::SharedPtr
      set_max_current_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::GetControlMode>::SharedPtr
      get_control_mode_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::SetControlMode>::SharedPtr
      set_control_mode_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::HomeMotors>::SharedPtr
      home_motors_srv_;
  rclcpp::Service<lhandpro_interfaces::srv::MoveMotors>::SharedPtr
      move_motors_srv_;
};