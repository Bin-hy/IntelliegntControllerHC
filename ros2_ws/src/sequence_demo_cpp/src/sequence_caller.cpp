#include <chrono>
#include <thread>
#include <vector>

#include "lhandpro_interfaces/srv/home_motors.hpp"
#include "lhandpro_interfaces/srv/move_motors.hpp"
#include "lhandpro_interfaces/srv/set_enable.hpp"
#include "lhandpro_interfaces/srv/set_position.hpp"
#include "lhandpro_interfaces/srv/set_position_velocity.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SequenceCaller : public rclcpp::Node {
 public:
  using SetEnable = lhandpro_interfaces::srv::SetEnable;
  using SetPosition = lhandpro_interfaces::srv::SetPosition;
  using SetPositionVelocity = lhandpro_interfaces::srv::SetPositionVelocity;
  using MoveMotors = lhandpro_interfaces::srv::MoveMotors;
  using HomeMotors = lhandpro_interfaces::srv::HomeMotors;

  SequenceCaller() : Node("sequence_caller") {
    RCLCPP_INFO(this->get_logger(), "Sequence Caller 启动...");

    // 创建客户端
    set_enable_client_ = this->create_client<SetEnable>("set_enable");
    set_position_client_ = this->create_client<SetPosition>("set_position");
    set_position_velocity_client_ =
        this->create_client<SetPositionVelocity>("set_position_velocity");
    move_motors_client_ = this->create_client<MoveMotors>("move_motors");
    home_motors_client_ = this->create_client<HomeMotors>("home_motors");

    // 等待服务上线
    while (!set_enable_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "等待 set_enable 服务...");
    }

    while (!set_position_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "等待 set_position 服务...");
    }

    while (!set_position_velocity_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "等待 set_position_velocity 服务...");
    }

    while (!move_motors_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "等待 move_motors 服务...");
    }

    while (!home_motors_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "等待 home_motors 服务...");
    }

    // 使用线程启动轨迹执行
    std::thread([this]() {
      rclcpp::NodeOptions options;
      auto context = std::make_shared<rclcpp::Context>();
      context->init(0, nullptr);

      // 创建一个独立节点用于执行服务调用
      auto sub_node = rclcpp::Node::make_shared("sequence_sub_node");

      // 执行轨迹
      this->execute_trajectory();
    }).detach();  // 自动释放线程资源（注意内存安全）
  }

 public:
  void execute_trajectory() {
    // 设置默认参数
    std::vector<int> joint_ids = {1, 2, 3, 4, 5, 6};
    int default_velocity_ = 20000;

    // 内嵌动作数组
    const std::vector<std::vector<int>> positions = {
        {0, 0, 0, 0, 0, 0},     {10000, 0, 0, 0, 0, 0}, {0, 10000, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},     {0, 0, 10000, 0, 0, 0}, {0, 0, 0, 10000, 0, 0},
        {0, 0, 0, 0, 10000, 0}, {0, 0, 0, 0, 0, 10000}, {0, 0, 0, 0, 0, 0},
    };

    send_set_enable(0, 1);
    RCLCPP_INFO(this->get_logger(), "正在使能");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    send_home_motors(0);
    RCLCPP_INFO(this->get_logger(), "正在回零");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    for (size_t i = 0; i < positions.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "正在执行第 %lu 步动作...", i + 1);

      for (size_t j = 0; j < joint_ids.size(); ++j) {
        int joint_id = joint_ids[j];
        int position = positions[i][j];

        send_set_position(joint_id, position);
        send_set_position_velocity(joint_id, default_velocity_);
      }

      // 触发所有关节运动
      send_move_motors(0);
      RCLCPP_INFO(this->get_logger(), "第 %lu 步动作", i + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
  }

  void send_set_position(int joint_id, int position) {
    auto req = std::make_shared<SetPosition::Request>();
    req->joint_id = joint_id;
    req->position = position;

    auto future = set_position_client_->async_send_request(req);
    future.wait();
    if (future.get()->result == 0) {
      RCLCPP_DEBUG(this->get_logger(), "设置位置成功: %d %d", joint_id,
                   position);
    } else {
      RCLCPP_ERROR(this->get_logger(), "设置位置失败: %d", joint_id);
    }
  }

  void send_set_position_velocity(int joint_id, int velocity) {
    auto req = std::make_shared<SetPositionVelocity::Request>();
    req->joint_id = joint_id;
    req->velocity = velocity;

    auto future = set_position_velocity_client_->async_send_request(req);
    future.wait();
    if (future.get()->result == 0) {
      RCLCPP_DEBUG(this->get_logger(), "设置速度成功: %d %d", joint_id,
                   velocity);
    } else {
      RCLCPP_ERROR(this->get_logger(), "设置速度失败: %d", joint_id);
    }
  }

  void send_set_enable(int joint_id, int enable) {
    auto req = std::make_shared<SetEnable::Request>();
    req->joint_id = joint_id;
    req->enable = enable;

    auto future = set_enable_client_->async_send_request(req);
    future.wait();
    if (future.get()->result == 0) {
      RCLCPP_DEBUG(this->get_logger(), "设置使能成功: %d %d", joint_id, enable);
    } else {
      RCLCPP_ERROR(this->get_logger(), "设置使能失败: %d", joint_id);
    }
  }

  void send_move_motors(int joint_id) {
    auto req = std::make_shared<MoveMotors::Request>();
    req->joint_id = joint_id;

    auto future = move_motors_client_->async_send_request(req);
    future.wait();
    if (future.get()->result != 0) {
      RCLCPP_ERROR(this->get_logger(), "驱动关节失败: %d", joint_id);
    }
  }

  void send_home_motors(int joint_id) {
    auto req = std::make_shared<HomeMotors::Request>();
    req->joint_id = joint_id;

    auto future = home_motors_client_->async_send_request(req);
    future.wait();
    if (future.get()->result != 0) {
      RCLCPP_ERROR(this->get_logger(), "回零关节失败: %d", joint_id);
    }
  }

  rclcpp::Client<SetEnable>::SharedPtr set_enable_client_;
  rclcpp::Client<SetPosition>::SharedPtr set_position_client_;
  rclcpp::Client<SetPositionVelocity>::SharedPtr set_position_velocity_client_;
  rclcpp::Client<MoveMotors>::SharedPtr move_motors_client_;
  rclcpp::Client<HomeMotors>::SharedPtr home_motors_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceCaller>());
  rclcpp::shutdown();
  return 0;
}