#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <duco_msg/srv/robot_control.hpp>
#include <duco_msg/srv/robot_io_control.hpp>
#include <duco_msg/srv/robot_move.hpp>
#include <duco_msg/msg/duco_robot_state.hpp>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>

class RosNode : public rclcpp::Node {
public:
  RosNode();

  void call_robot_control(const std::string& command);
  void call_robot_move(const std::string& command, 
                       const std::vector<float>& p, 
                       const std::vector<float>& q, 
                       float v, float a, float r, 
                       const std::string& tool, const std::string& wobj);
  void call_robot_io(const std::string& command, int type, int port, bool value);

  // Data storage
  std::atomic<int> count_;
  std::string last_robot_state_str_;
  
  std::vector<double> current_joints_; 
  std::vector<double> current_cart_pos_;
  
  std::mutex data_mutex_;

private:
  void robot_state_callback(const duco_msg::msg::DucoRobotState::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_robot_state_;
  
  rclcpp::Client<duco_msg::srv::RobotControl>::SharedPtr client_control_;
  rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedPtr client_io_;
  rclcpp::Client<duco_msg::srv::RobotMove>::SharedPtr client_move_;

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ROS_NODE_HPP
