#include "ui_app/ros_node.hpp"
#include <sstream>
#include <chrono>

using namespace std::chrono_literals;

RosNode::RosNode() : rclcpp::Node("ui_ros_node"), count_(0) {
    // Publisher
    pub_ = create_publisher<std_msgs::msg::String>("/ui/heartbeat", 10);
    
    // Subscriber: Duco Robot State
    sub_robot_state_ = create_subscription<duco_msg::msg::DucoRobotState>(
      "/duco_robot/robot_state", 30, std::bind(&RosNode::robot_state_callback, this, std::placeholders::_1));

    // Service Clients
    // Refactored to point to SystemController instead of direct Duco Driver
    client_control_ = create_client<duco_msg::srv::RobotControl>("/ui/request_control");
    client_io_ = create_client<duco_msg::srv::RobotIoControl>("/ui/request_io");
    client_move_ = create_client<duco_msg::srv::RobotMove>("/ui/request_move");

    // Timer
    timer_ = create_wall_timer(std::chrono::seconds(1), [this](){
      std_msgs::msg::String msg;
      msg.data = "ok";
      pub_->publish(msg);
      ++count_;
    });
}

void RosNode::call_robot_control(const std::string& command) {
    if (!client_control_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Robot Control service not available");
      return;
    }
    auto request = std::make_shared<duco_msg::srv::RobotControl::Request>();
    request->command = command;
    request->arm_num = 0;
    request->block = true; // Request blocking, SystemController will enforce it anyway
    
    auto future = client_control_->async_send_request(request, 
      [this, command](rclcpp::Client<duco_msg::srv::RobotControl>::SharedFuture future) {
        try {
          auto response = future.get();
          RCLCPP_INFO(get_logger(), "Control '%s' result: %s", command.c_str(), response->response.c_str());
        } catch (const std::exception &e) {
          RCLCPP_ERROR(get_logger(), "Control '%s' failed: %s", command.c_str(), e.what());
        }
      });
}

void RosNode::call_robot_move(const std::string& command, 
                       const std::vector<float>& p, 
                       const std::vector<float>& q, 
                       float v, float a, float r, 
                       const std::string& tool, const std::string& wobj) {
     if (!client_move_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Robot Move service not available");
      return;
    }

    auto request = std::make_shared<duco_msg::srv::RobotMove::Request>();
    request->command = command;
    request->arm_num = 0;
    request->p = p;
    request->q = q;
    request->v = v;
    request->a = a;
    request->r = r;
    request->tool = tool;
    request->wobj = wobj;
    request->block = true; // Request blocking, SystemController will enforce it anyway

    RCLCPP_INFO(get_logger(), "Sending Move Command to SystemController: %s, V: %.2f, A: %.2f", command.c_str(), v, a);

    auto future = client_move_->async_send_request(request, 
      [this, command](rclcpp::Client<duco_msg::srv::RobotMove>::SharedFuture future) {
        try {
          auto response = future.get();
          RCLCPP_INFO(get_logger(), "Move '%s' result: %s", command.c_str(), response->response.c_str());
        } catch (const std::exception &e) {
          RCLCPP_ERROR(get_logger(), "Move '%s' failed: %s", command.c_str(), e.what());
        }
      });
}

void RosNode::call_robot_io(const std::string& command, int type, int port, bool value) {
      if (!client_io_->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_WARN(get_logger(), "Robot IO service not available");
          return;
      }
      auto request = std::make_shared<duco_msg::srv::RobotIoControl::Request>();
      request->command = command;
      request->arm_num = 0;
      request->type = type;
      request->port = port;
      request->value = value;
      request->block = true; // Request blocking, SystemController will enforce it anyway

      auto future = client_io_->async_send_request(request,
          [this, command, port, value](rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedFuture future) {
              try {
                  auto response = future.get();
                  RCLCPP_INFO(get_logger(), "IO '%s' (Port %d -> %d) result: %s", command.c_str(), port, value, response->response.c_str());
              } catch (const std::exception &e) {
                  RCLCPP_ERROR(get_logger(), "IO Call failed: %s", e.what());
              }
          });
}

void RosNode::robot_state_callback(const duco_msg::msg::DucoRobotState::SharedPtr msg)
{
      std::lock_guard<std::mutex> lock(data_mutex_);
      std::stringstream ss;
      ss << "State Code: " << (int)msg->robot_state << "\n";
      ss << "Error Code: " << msg->robot_error << "\n";
      ss << "Collision: " << (msg->collision ? "YES" : "NO") << "\n";
      
      ss << "Joint Pos: [";
      current_joints_.clear();
      for(size_t i=0; i<7; ++i) {
          ss << msg->joint_actual_position[i] << (i<6?", ":"");
          current_joints_.push_back(msg->joint_actual_position[i]);
      }
      ss << "]\n";

      // Store cartesian if available
      current_cart_pos_.clear();
      for(size_t i=0; i<6; ++i) { // Usually 6 for Cartesian
           current_cart_pos_.push_back(msg->cart_actual_position[i]);
      }
      
      last_robot_state_str_ = ss.str();
}
