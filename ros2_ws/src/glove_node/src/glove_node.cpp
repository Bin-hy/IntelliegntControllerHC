#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/glove_joints.hpp>
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
    
    // 初始化 SDK
    sdk_.SetPortNum(5555);
    sdk_.StartListening();
    
    RCLCPP_INFO(this->get_logger(), "Glove SDK started listening on port 5555");

    timer_ = this->create_wall_timer(
      20ms, std::bind(&GloveNode::timer_callback, this)); // 50Hz
  }

  ~GloveNode()
  {
    sdk_.EndListening();
  }

private:
  void timer_callback()
  {
    auto role_list = sdk_.GetRoleNameList();
    if (role_list.empty()) {
      // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No glove connected");
      return;
    }

    // 假设我们只关心第一个连接的角色，或者可以遍历
    for (const auto& role : role_list) {
      auto finger_data = sdk_.GetVecFingerData(role);
      
      // GetVecFingerData 返回 30 个 Vector3Float
      // 左手 15 个关节 (0-14)，右手 15 个关节 (15-29)
      // 每个 Vector3Float 有 x, y, z
      
      auto msg = common_msgs::msg::GloveJoints();
      // 这里我们需要定义 msg 的结构。假设 GloveJoints.msg 有 name[], position[]
      // 为了简单演示，我们只填充 position
      
      // 注意：GloveVecRes 的大小固定为 30
      if (finger_data.size() < 30) continue;

      // 填充数据
      // 简单映射：将所有关节的 x, y, z 展平或者只取 x (弯曲度)
      // 这里需要根据实际 msg 定义来填充。
      // 假设 common_msgs/msg/GloveJoints.msg 定义为:
      // string[] name
      // float64[] position
      
      // 我们简单地把 30 个关节的 x 分量放进去作为演示
      for (size_t i = 0; i < finger_data.size(); ++i) {
          msg.name.push_back("joint_" + std::to_string(i) + "_x");
          msg.position.push_back(finger_data[i].x);
          msg.name.push_back("joint_" + std::to_string(i) + "_y");
          msg.position.push_back(finger_data[i].y);
          msg.name.push_back("joint_" + std::to_string(i) + "_z");
          msg.position.push_back(finger_data[i].z);
      }
      
      publisher_->publish(msg);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<common_msgs::msg::GloveJoints>::SharedPtr publisher_;
  UDEGloveSDK sdk_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GloveNode>());
  rclcpp::shutdown();
  return 0;
}
