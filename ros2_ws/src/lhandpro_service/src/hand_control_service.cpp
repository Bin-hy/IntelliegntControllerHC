#include "hand_control_service.hpp"

#include <functional>
#include <vector>

#include "EthercatMaster.h"

HandControlService::HandControlService() : Node("lhandpro_service") {
  RCLCPP_INFO(this->get_logger(), "LHandPro控制服务已创建");
  lhp_lib_ = std::make_shared<lhplib::LHandProLib>();
  ec_master_ = std::make_shared<EthercatMaster>();

  is_connected_ = false;
  current_channel_ = 0;  // 手动修改默认通道

  init_ethercat(current_channel_);
  init_service();

  // 每5秒检查一次连接状态
  reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), [this]() { this->check_and_reconnect(); });
}

HandControlService::~HandControlService() {
  if (ec_master_) {
    ec_master_->stop();
  }
  if (lhp_lib_) {
    lhp_lib_->close();
  }
}

void HandControlService::init_ethercat(int channel) {
  cleanup_resources();

  int target = channel;
  auto channels = ec_master_->scanNetworkInterfaces();
  if (channels.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "没有端口");
    return;
  }

  for (size_t i = 0; i < channels.size(); ++i)
    RCLCPP_INFO(this->get_logger(), "扫描到:%d --- %s", (int)i,
                channels[i].c_str());

  RCLCPP_INFO(this->get_logger(), "正在连接:%d --- %s", target,
              channels[target].c_str());

  // 初始化
  if (ec_master_->init(channel) == false) {
    RCLCPP_ERROR(this->get_logger(), "连接失败");
    return;
  }
  // 启动到OP
  if (!ec_master_->start()) {
    RCLCPP_ERROR(this->get_logger(), "OP失败");
    return;
  }
  // 启动后台通信（非阻塞）
  ec_master_->run();

  /****** LHandProLib的初始化 ******/
  // EtherCAT的发送函数
  auto send_func = [this](const unsigned char* data, unsigned int size) {
    return ec_master_->setOutputs(data, size);
  };
  // 使用std::function包装
  send_function_ = send_func;

  // 设置回调
  lhp_lib_->set_send_rpdo_callback_ex(&send_function_);

  // 等待一会儿, 再初始化
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  lhp_lib_->initial(lhplib::LCN_ECAT);

  // 监控线程, 刷新TPDO数据
  start_monitor();  // 启动监控

  int total = 0, active = 0;
  lhp_lib_->get_dof(&total, &active);
  RCLCPP_INFO(this->get_logger(), "连接成功，总自由度: %d，主动自由度: %d",
              total, active);
  active_dof_ = active;
  is_connected_ = true;
  current_channel_ = target;
}

void HandControlService::cleanup_resources() {
  RCLCPP_INFO(this->get_logger(), "清理资源...");

  // 1. 停止监控线程
  stop_monitor();

  // 2. 停止EtherCAT主站
  if (ec_master_) {
    ec_master_->stop();
  }

  // 3. 关闭LHandPro库
  if (lhp_lib_) {
    lhp_lib_->close();
  }

  // 4. 重置状态
  is_connected_ = false;
}

void HandControlService::check_and_reconnect() {
  if (is_connected_) {
    // 检查是否仍然有效
    if (!is_alive()) {
      RCLCPP_WARN(this->get_logger(), "检测到连接异常，尝试重新连接...");
      is_connected_ = false;
    }
  }

  if (!is_connected_) {
    RCLCPP_INFO(this->get_logger(), "正在尝试重新连接设备...");
    init_ethercat(current_channel_);
  }
}

bool HandControlService::is_alive() {
  if (!ec_master_) return false;
  return ec_master_->getState() == EthercatMaster::EthercatState::Operational;
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
  monitor_thread_ = std::thread([this]() {
    while (!stop_flag_) {
      // 监控逻辑
      int input_size = ec_master_->getInputSize();
      std::vector<uint8_t> in_buffer(input_size);
      if (ec_master_->getInputs(in_buffer.data(), input_size)) {
        lhp_lib_->set_tpdo_data_decode(in_buffer.data(), input_size);
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
  int retn = lhp_lib_->set_target_position(req->joint_id, req->position);
  if (retn != 0) {
    RCLCPP_WARN(this->get_logger(), "[%s] 设置位置失败，错误码：%d",
                service_name, retn);
  }
  res->result = retn;
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