#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/glove_joints.hpp>
#include <common_msgs/msg/device_status.hpp>
#include "UDEServer.h"
#include <vector>
#include <string>

using namespace std::chrono_literals;

class GloveNode : public rclcpp::Node
{
public:
  GloveNode() : Node("glove_node")
  {
    publisher_ = this->create_publisher<common_msgs::msg::GloveJoints>("glove/joints", 10);
    pub_device_status_ = this->create_publisher<common_msgs::msg::DeviceStatus>("/system/device_status", 10);
    
    // 初始化 SDK
    sdk_.SetPortNum(5555);
    sdk_.StartListening();
    
    RCLCPP_INFO(this->get_logger(), "Glove SDK started listening on port 5555");

    timer_ = this->create_wall_timer(
      20ms, std::bind(&GloveNode::timer_callback, this)); // 50Hz
      
    // Status Timer (1Hz)
    status_timer_ = this->create_wall_timer(
      1000ms, std::bind(&GloveNode::status_timer_callback, this));
  }

  ~GloveNode()
  {
    sdk_.EndListening();
  }

private:
  void status_timer_callback()
  {
      common_msgs::msg::DeviceStatus status;
      status.device_type = "glove";
      status.device_usage = "capture";
      status.device_model = "UDE_Glove";
      status.device_name = "UDE_Glove";
      
      auto role_list = sdk_.GetRoleNameList();
      if (role_list.empty()) {
          status.status = "disconnected";
          status.device_sn = "glove_default";
      } else {
          status.status = "ready";
          // Use the first detected device ID as SN
          status.device_sn = role_list[0];
      }
      
      pub_device_status_->publish(status);
  }

  void timer_callback()
  {
    auto role_list = sdk_.GetRoleNameList();
    if (role_list.empty()) {
      return;
    }

    // Only process the first connected glove for now
    const auto& role = role_list[0];
    auto finger_data = sdk_.GetVecFingerData(role);
    
    if (finger_data.empty()) return;

    auto msg = common_msgs::msg::GloveJoints();
    
    // UDE Glove returns 15 joints per hand (if single hand mode) or more
    // Mapping: 0-4 thumb, 5-8 index, etc. (Just an example, depends on SDK)
    // Here we just flatten the data
    
    for (size_t i = 0; i < finger_data.size(); ++i) {
        // Simple naming convention: joint_0_x, joint_0_y, etc.
        // In reality, you might want semantic names like "thumb_mcp_flexion"
        msg.name.push_back("joint_" + std::to_string(i) + "_x");
        msg.position.push_back(finger_data[i].x);
        
        msg.name.push_back("joint_" + std::to_string(i) + "_y");
        msg.position.push_back(finger_data[i].y);
        
        msg.name.push_back("joint_" + std::to_string(i) + "_z");
        msg.position.push_back(finger_data[i].z);
    }
    
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Publisher<common_msgs::msg::GloveJoints>::SharedPtr publisher_;
  rclcpp::Publisher<common_msgs::msg::DeviceStatus>::SharedPtr pub_device_status_;
  UDEGloveSDK sdk_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GloveNode>());
  rclcpp::shutdown();
  return 0;
}
