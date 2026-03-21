#include "hand_control_service.hpp"

#include <functional>
#include <vector>

#include "DucoTransport.h"

HandControlService::HandControlService() : Node("lhandpro_service") {
  RCLCPP_INFO(this->get_logger(), "LHandPro控制服务已创建");
  lhp_lib_ = std::make_shared<lhplib::LHandProLib>();
  transport_ = std::make_shared<DucoTransport>();

  is_connected_ = false;

  // 声明并获取参数
  this->declare_parameter("robot_ip", "192.168.1.10");
  this->declare_parameter("protocol", "tool_rs485");
  robot_ip_ = this->get_parameter("robot_ip").as_string();
  protocol_str_ = this->get_parameter("protocol").as_string();

  // Publisher for DeviceStatus
  pub_device_status_ = this->create_publisher<common_msgs::msg::DeviceStatus>("/system/device_status", 10);
  now_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lhandpro_service/now_angles", 10);

  RCLCPP_INFO(this->get_logger(), "使用DUCO透传, IP: %s, 协议: %s",
              robot_ip_.c_str(), protocol_str_.c_str());

  init_transport(robot_ip_);
  init_service();

  now_angles_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), [this]() { this->publish_now_angles(); });

  // 每5秒检查一次连接状态
  reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), [this]() { this->check_and_reconnect(); });
}

HandControlService::~HandControlService() {
  // 先停监控线程，再关库，最后关传输层（避免线程访问已关闭的 socket）
  stop_monitor();
  if (lhp_lib_) {
    lhp_lib_->close();
  }
  if (transport_) {
    transport_->stop();
  }
}

void HandControlService::init_transport(const std::string& robot_ip) {
  cleanup_resources();

  // 解析协议参数
  DucoTransport::Protocol proto = DucoTransport::Protocol::ToolRS485;
  if (protocol_str_ == "rs485") proto = DucoTransport::Protocol::RS485;
  else if (protocol_str_ == "can") proto = DucoTransport::Protocol::CAN;

  RCLCPP_INFO(get_logger(), "连接 DUCO 控制器: %s, 协议: %s",
              robot_ip.c_str(), protocol_str_.c_str());

  if (!transport_->init(robot_ip, 7003, proto)) {
    RCLCPP_ERROR(get_logger(), "DUCO 控制器连接失败");
    return;
  }

  /****** LHandProLib的初始化 ******/
  // 不使用回调机制 — 直接在监控线程中手动 send+recv，避免重入/双重发送问题
  // 回调在没有真实EtherCAT硬件时不会被触发 (initial(LCN_ECAT)扫描超时)
  // send_function_ 仍保留以备将来使用，但不向库注册

  // 开启日志，辅助调试
  lhp_lib_->log_on(true, 5000);

  // 等待一会儿, 再初始化
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  int init_ret = lhp_lib_->initial(lhplib::LCN_ECAT);
  RCLCPP_INFO(get_logger(), "LHandProLib initial(LCN_ECAT) 返回: %d", init_ret);

  // 监控线程, 刷新TPDO数据
  start_monitor();

  int total = 0, active = 0;
  lhp_lib_->get_dof(&total, &active);
  RCLCPP_INFO(get_logger(), "连接成功，总自由度: %d，主动自由度: %d",
              total, active);
  active_dof_ = active;
  is_connected_ = true;
}

void HandControlService::cleanup_resources() {
  RCLCPP_INFO(this->get_logger(), "清理资源...");

  // 1. 停止监控线程
  stop_monitor();

  // 2. 停止传输层
  if (transport_) {
    transport_->stop();
  }

  // 3. 关闭LHandPro库
  if (lhp_lib_) {
    lhp_lib_->close();
  }

  // 4. 重置状态
  is_connected_ = false;
}

void HandControlService::check_and_reconnect() {
  common_msgs::msg::DeviceStatus status_msg;
  status_msg.device_type = "lhand";
  status_msg.device_usage = "left_hand";
  status_msg.device_model = "DH116-L000-A1";
  status_msg.device_name = "DH116 左手";
  status_msg.device_sn = "lhand_1";

  if (is_connected_) {
    // 检查是否仍然有效
    if (!is_alive()) {
      RCLCPP_WARN(this->get_logger(), "检测到连接异常，尝试重新连接...");
      is_connected_ = false;
      status_msg.status = "error";
    } else {
      status_msg.status = "ready";
    }
  } else {
    status_msg.status = "disconnected";
  }
  
  pub_device_status_->publish(status_msg);

  if (!is_connected_) {
    RCLCPP_INFO(this->get_logger(), "正在尝试重新连接设备...");
    init_transport(robot_ip_);
  }
}

void HandControlService::publish_now_angles() {
  if (!lhp_lib_) return;
  if (!is_alive()) return;
  if (last_angles_.size() != 6) {
    last_angles_.assign(6, 0.0);
  }
  int count = active_dof_;
  if (count > 6) count = 6;
  for (int i = 0; i < count; ++i) {
    float angle = 0.0f;
    int retn = lhp_lib_->get_now_angle(i + 1, &angle);
    if (retn == 0) {
      last_angles_[i] = angle;
    }
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data = last_angles_;
  now_angles_pub_->publish(msg);
}

bool HandControlService::is_alive() {
  if (!transport_) return false;
  return transport_->getState() == DucoTransport::State::Operational;
}

bool HandControlService::check_joint_validity(int joint_id,
                                              const std::string& service_name) {
  if (!lhp_lib_) {
    RCLCPP_ERROR(this->get_logger(), "[%s] LHP库未初始化",
                 service_name.c_str());
    return false;
  }

  if (!is_alive()) {
    RCLCPP_ERROR(this->get_logger(), "[%s] 设备未连接或通信异常",
                 service_name.c_str());
    return false;
  }

  if (joint_id < 0 || joint_id > active_dof_) {
    RCLCPP_ERROR(this->get_logger(), "[%s] 无效的关节ID: %d",
                 service_name.c_str(), joint_id);
    return false;
  }

  return true;
}

void HandControlService::start_monitor() {
  if (monitor_thread_.joinable()) return;  // 已经在运行

  stop_flag_ = false;
  send_count_ = 0;
  recv_count_ = 0;
  monitor_thread_ = std::thread([this]() {
    bool use_can = (protocol_str_ == "can");
    int cycle = 0;
    int recv_ok = 0;
    int send_fail = 0;

    // 直接手动 send+recv，不依赖回调机制
    // 原因：initial(LCN_ECAT) 在无真实EtherCAT硬件时会超时(~37s)，
    // 回调不会被 get_pre_send_rpdo_data() 触发 (cb_sent=0 confirmed in log)
    while (!stop_flag_) {
      // 1. 获取 RPDO 数据
      int rpdo_size = 0;
      if (use_can) {
        lhp_lib_->get_pre_send_canfd_data(nullptr, &rpdo_size);
      } else {
        lhp_lib_->get_pre_send_rpdo_data(nullptr, &rpdo_size);
      }

      if (rpdo_size > 0) {
        std::vector<unsigned char> rpdo(rpdo_size);
        if (use_can) {
          lhp_lib_->get_pre_send_canfd_data(rpdo.data(), &rpdo_size);
        } else {
          lhp_lib_->get_pre_send_rpdo_data(rpdo.data(), &rpdo_size);
        }

        // 2. 手动发送 RPDO 并接收 TPDO
        unsigned char recv_buf[256];
        int recv_len = transport_->sendAndRecv(rpdo.data(), rpdo_size,
                                               recv_buf, sizeof(recv_buf));
        send_count_++;
        if (recv_len > 0) {
          // 3. 解码 TPDO
          if (use_can) {
            lhp_lib_->set_canfd_data_decode(recv_buf, recv_len);
          } else {
            lhp_lib_->set_tpdo_data_decode(recv_buf, recv_len);
          }
          recv_ok++;
        } else {
          send_fail++;
        }
      }

      // 诊断日志 (每 2 秒)
      cycle++;
      if (cycle % 200 == 0) {
        RCLCPP_INFO(get_logger(),
                    "透传状态: rpdo=%d sent=%d recv=%d fail=%d transport=%s",
                    rpdo_size, send_count_.load(), recv_ok, send_fail,
                    transport_->getState() == DucoTransport::State::Operational
                        ? "OK" : "ERROR");
        send_count_ = 0;
        recv_count_ = 0;
        recv_ok = 0;
        send_fail = 0;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

void HandControlService::stop_monitor() {
  stop_flag_ = true;
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
}

void HandControlService::init_service() {
  // 使用宏注册所有服务
  set_enable_srv_ =
      REGISTER_SERVICE(SetEnable, SRV_NAME_SET_ENABLE, set_enable_callback);
  set_position_srv_ = REGISTER_SERVICE(SetPosition, SRV_NAME_SET_POSITION,
                                       set_position_callback);
  set_all_position_srv_ = REGISTER_SERVICE(SetAllPosition, SRV_NAME_SET_ALL_POSITION,
                                       set_all_position_callback);
  get_position_srv_ = REGISTER_SERVICE(GetPosition, SRV_NAME_GET_POSITION,
                                       get_position_callback);
  get_now_angle_srv_ = REGISTER_SERVICE(GetNowAngle, SRV_NAME_GET_NOW_ANGLE,
                                        get_now_angle_callback);
  get_now_position_srv_ = REGISTER_SERVICE(
      GetNowPosition, SRV_NAME_GET_NOW_POSITION, get_now_position_callback);
  get_now_position_velocity_srv_ = REGISTER_SERVICE(
      GetNowPositionVelocity, SRV_NAME_GET_NOW_POSITION_VELOCITY,
      get_now_position_velocity_callback);
  get_now_current_srv_ = REGISTER_SERVICE(
      GetNowCurrent, SRV_NAME_GET_NOW_CURRENT, get_now_current_callback);
  get_position_velocity_srv_ =
      REGISTER_SERVICE(GetPositionVelocity, SRV_NAME_GET_POSITION_VELOCITY,
                       get_position_velocity_callback);
  set_position_velocity_srv_ =
      REGISTER_SERVICE(SetPositionVelocity, SRV_NAME_SET_POSITION_VELOCITY,
                       set_position_velocity_callback);
  get_max_current_srv_ = REGISTER_SERVICE(
      GetMaxCurrent, SRV_NAME_GET_MAX_CURRENT, get_max_current_callback);
  set_max_current_srv_ = REGISTER_SERVICE(
      SetMaxCurrent, SRV_NAME_SET_MAX_CURRENT, set_max_current_callback);
  get_control_mode_srv_ = REGISTER_SERVICE(
      GetControlMode, SRV_NAME_GET_CONTROL_MODE, get_control_mode_callback);
  set_control_mode_srv_ = REGISTER_SERVICE(
      SetControlMode, SRV_NAME_SET_CONTROL_MODE, set_control_mode_callback);
  home_motors_srv_ =
      REGISTER_SERVICE(HomeMotors, SRV_NAME_HOME_MOTORS, home_motors_callback);
  move_motors_srv_ =
      REGISTER_SERVICE(MoveMotors, SRV_NAME_MOVE_MOTORS, move_motors_callback);
}

void HandControlService::set_enable_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::SetEnable::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::SetEnable::Response> res) {
  const char* service_name = SRV_NAME_SET_ENABLE;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }
  int retn = lhp_lib_->set_enable(req->joint_id, req->enable);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置使能失败，错误码：%d",
                service_name, retn);
  }
  res->result = retn;
}

void HandControlService::set_position_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::SetPosition::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::SetPosition::Response> res) {
  const char* service_name = SRV_NAME_SET_POSITION;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "关节位置信息: 关节ID=%d, 目标位置=%d", req->joint_id, req->position);
  
  int retn = lhp_lib_->set_target_position(req->joint_id, req->position);

  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置位置失败，错误码：%d",
                service_name, retn);
  }
  res->result = retn;
}

void HandControlService::set_all_position_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::SetAllPosition::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::SetAllPosition::Response> res) {
  const char* service_name = SRV_NAME_SET_ALL_POSITION;
  
  if (!is_alive()) {
    RCLCPP_ERROR(this->get_logger(), "[%s] 设备未连接或通信异常", service_name);
    res->result = -1;
    return;
  }

  // positions is int32[6]
  for (int i = 0; i < 6; ++i) {
      int joint_id = i + 1;
      if (joint_id > active_dof_) break; // Stop if exceeding active DOF
      
      int pos = req->positions[i];
      int retn = lhp_lib_->set_target_position(joint_id, pos);
      if (retn != 0) {
          RCLCPP_WARN(this->get_logger(), "[%s] 设置关节%d位置失败，错误码：%d",
                      service_name, joint_id, retn);
          // Continue setting others? Or return error?
          // Let's continue but mark error.
          res->result = retn; 
      }
  }
  
  // Usually we trigger move automatically or wait for move_motors call?
  // The existing pattern seems to be: set_position sets target, move_motors executes it?
  // Or does set_target_position just update internal state?
  // Based on existing code: set_position just calls set_target_position.
  
  if (res->result == 0) {
       RCLCPP_INFO(this->get_logger(), "设置所有关节位置成功");
  }
}

void HandControlService::get_position_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::GetPosition::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::GetPosition::Response> res) {
  const char* service_name = SRV_NAME_GET_POSITION;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->position = 0;
    return;
  }
  int position = 0;
  int retn = lhp_lib_->get_target_position(req->joint_id, &position);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取目标位置失败，错误码：%d",
                service_name, retn);
  }
  res->position = position;
}

void HandControlService::get_now_angle_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::GetNowAngle::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::GetNowAngle::Response> res) {
  const char* service_name = SRV_NAME_GET_NOW_ANGLE;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->angle = 0.0f;
    return;
  }
  float angle = 0.0f;
  int retn = lhp_lib_->get_now_angle(req->joint_id, &angle);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取当前角度失败，错误码：%d",
                service_name, retn);
  }
  res->angle = angle;
}

void HandControlService::get_now_position_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::GetNowPosition::Request>
        req,
    std::shared_ptr<lhandpro_interfaces::srv::GetNowPosition::Response> res) {
  const char* service_name = SRV_NAME_GET_NOW_POSITION;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->position = 0;
    return;
  }
  int position = 0;
  int retn = lhp_lib_->get_now_position(req->joint_id, &position);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取当前位置失败，错误码：%d",
                service_name, retn);
  }
  res->position = position;
}

void HandControlService::get_now_position_velocity_callback(
    const std::shared_ptr<
        lhandpro_interfaces::srv::GetNowPositionVelocity::Request>
        req,
    std::shared_ptr<lhandpro_interfaces::srv::GetNowPositionVelocity::Response>
        res) {
  const char* service_name = SRV_NAME_GET_NOW_POSITION_VELOCITY;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->velocity = 0;
    return;
  }
  int velocity = 0;
  int retn = lhp_lib_->get_now_position_velocity(req->joint_id, &velocity);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取当前速度失败，错误码：%d",
                service_name, retn);
  }
  res->velocity = velocity;
}

void HandControlService::get_now_current_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::GetNowCurrent::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::GetNowCurrent::Response> res) {
  const char* service_name = SRV_NAME_GET_NOW_CURRENT;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->current = 0.0;
    return;
  }
  int current = 0;
  int retn = lhp_lib_->get_now_current(req->joint_id, &current);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取当前电流失败，错误码：%d",
                service_name, retn);
  }
  res->current = (float)current;
}

void HandControlService::get_position_velocity_callback(
    const std::shared_ptr<
        lhandpro_interfaces::srv::GetPositionVelocity::Request>
        req,
    std::shared_ptr<lhandpro_interfaces::srv::GetPositionVelocity::Response>
        res) {
  const char* service_name = SRV_NAME_GET_POSITION_VELOCITY;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->velocity = 0;
    return;
  }
  int velocity = 0;
  int retn = lhp_lib_->get_position_velocity(req->joint_id, &velocity);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取目标速度失败，错误码：%d",
                service_name, retn);
  }
  res->velocity = velocity;
}

void HandControlService::set_position_velocity_callback(
    const std::shared_ptr<
        lhandpro_interfaces::srv::SetPositionVelocity::Request>
        req,
    std::shared_ptr<lhandpro_interfaces::srv::SetPositionVelocity::Response>
        res) {
  const char* service_name = SRV_NAME_SET_POSITION_VELOCITY;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }
  int retn = lhp_lib_->set_position_velocity(req->joint_id, req->velocity);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置速度失败，错误码：%d",
                service_name, retn);
  }
  res->result = retn;
}

void HandControlService::get_max_current_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::GetMaxCurrent::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::GetMaxCurrent::Response> res) {
  const char* service_name = SRV_NAME_GET_MAX_CURRENT;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->current = 0.0;
    return;
  }
  int current = 0;
  int retn = lhp_lib_->get_max_current(req->joint_id, &current);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取最大电流失败，错误码：%d",
                service_name, retn);
  }
  res->current = (float)current;
}

void HandControlService::set_max_current_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::SetMaxCurrent::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::SetMaxCurrent::Response> res) {
  const char* service_name = SRV_NAME_SET_MAX_CURRENT;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }
  int retn = lhp_lib_->set_max_current(req->joint_id, req->current);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置最大电流失败，错误码：%d",
                service_name, retn);
  }
  res->result = retn;
}

void HandControlService::get_control_mode_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::GetControlMode::Request>
        req,
    std::shared_ptr<lhandpro_interfaces::srv::GetControlMode::Response> res) {
  const char* service_name = SRV_NAME_GET_CONTROL_MODE;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->mode = -1;
    return;
  }
  int mode = 0;
  int retn = lhp_lib_->get_control_mode(req->joint_id, &mode);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 获取控制模式失败，错误码：%d",
                service_name, retn);
  }
  res->mode = mode;
}

void HandControlService::set_control_mode_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::SetControlMode::Request>
        req,
    std::shared_ptr<lhandpro_interfaces::srv::SetControlMode::Response> res) {
  const char* service_name = SRV_NAME_SET_CONTROL_MODE;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }
  int retn = lhp_lib_->set_control_mode(req->joint_id, req->mode);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置控制模式失败，错误码：%d",
                service_name, retn);
  }
  res->result = retn;
}

void HandControlService::home_motors_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::HomeMotors::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::HomeMotors::Response> res) {
  const char* service_name = SRV_NAME_HOME_MOTORS;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }

  int retn = lhp_lib_->home_motors(req->joint_id);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置回零电机失败，错误码：%d",
                service_name, retn);
    res->result = retn;
    return;
  }
  if (req->joint_id == 0) {
    RCLCPP_INFO(this->get_logger(), "回零关节全部运动 : %d", retn);
  } else {
    RCLCPP_INFO(this->get_logger(), "回零关节 %d 运动 : %d", req->joint_id,
                retn);
  }
  res->result = retn;
}

void HandControlService::move_motors_callback(
    const std::shared_ptr<lhandpro_interfaces::srv::MoveMotors::Request> req,
    std::shared_ptr<lhandpro_interfaces::srv::MoveMotors::Response> res) {
  const char* service_name = SRV_NAME_MOVE_MOTORS;
  if (!check_joint_validity(req->joint_id, service_name)) {
    res->result = -1;
    return;
  }

  int retn = lhp_lib_->move_motors(req->joint_id);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置驱动电机失败，错误码：%d",
                service_name, retn);
    res->result = retn;
    return;
  }
  if (req->joint_id == 0) {
    
    RCLCPP_INFO(this->get_logger(), "驱动关节全部运动 : %d", retn);
  } else {
    RCLCPP_INFO(this->get_logger(), "驱动关节 %d 运动 : %d", req->joint_id,
                retn);
  }
  res->result = retn;
}
