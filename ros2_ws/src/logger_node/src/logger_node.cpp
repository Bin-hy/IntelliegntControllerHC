#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <common_msgs/msg/glove_joints.hpp>
#include <common_msgs/msg/hand_cmd.hpp>
#include <fstream>
#include <chrono>
#include <string>
#include <filesystem>
using namespace std::chrono_literals;
class LoggerNode : public rclcpp::Node {
public:
  LoggerNode() : rclcpp::Node("logger_node") {
    declare_parameter<std::string>("log_dir", "logs");
    declare_parameter<std::string>("session_prefix", "session");
    auto dir = get_parameter("log_dir").as_string();
    auto prefix = get_parameter("session_prefix").as_string();
    std::filesystem::create_directories(dir);
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    filename_ = dir + "/" + prefix + "_" + std::to_string(t) + ".csv";
    file_.open(filename_, std::ios::out | std::ios::app);
    file_ << "ts,source,fields\n";
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/arm/state", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg){ write_joint_state(*msg); });
    sub_glove_ = create_subscription<common_msgs::msg::GloveJoints>(
      "/glove/joints", rclcpp::SensorDataQoS(),
      [this](common_msgs::msg::GloveJoints::SharedPtr msg){ write_glove(*msg); });
    sub_hand_cmd_ = create_subscription<common_msgs::msg::HandCmd>(
      "/hand/cmd", rclcpp::ServicesQoS(),
      [this](common_msgs::msg::HandCmd::SharedPtr msg){ write_hand_cmd(*msg); });
    timer_ = create_wall_timer(5s, [this](){ if(file_) file_.flush(); });
  }
private:
  void write_joint_state(const sensor_msgs::msg::JointState & msg) {
    auto ts = rclcpp::Clock().now();
    file_ << ts.nanoseconds() << ",arm.state,";
    for(size_t i=0;i<msg.name.size();++i){
      file_ << msg.name[i] << ":" << (i<msg.position.size()?msg.position[i]:0.0);
      if(i+1<msg.name.size()) file_ << "|";
    }
    file_ << "\n";
  }
  void write_glove(const common_msgs::msg::GloveJoints & msg) {
    auto ts = rclcpp::Clock().now();
    file_ << ts.nanoseconds() << ",glove.joints,";
    for(size_t i=0;i<msg.name.size();++i){
      double p = i<msg.position.size()?msg.position[i]:0.0;
      file_ << msg.name[i] << ":" << p;
      if(i+1<msg.name.size()) file_ << "|";
    }
    file_ << "\n";
  }
  void write_hand_cmd(const common_msgs::msg::HandCmd & msg) {
    auto ts = rclcpp::Clock().now();
    file_ << ts.nanoseconds() << ",hand.cmd,";
    for(size_t i=0;i<msg.name.size();++i){
      double t = i<msg.target.size()?msg.target[i]:0.0;
      file_ << msg.name[i] << ":" << t;
      if(i+1<msg.name.size()) file_ << "|";
    }
    file_ << ";speed:" << msg.speed << ";effort:" << msg.effort << "\n";
  }
  std::string filename_;
  std::ofstream file_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<common_msgs::msg::GloveJoints>::SharedPtr sub_glove_;
  rclcpp::Subscription<common_msgs::msg::HandCmd>::SharedPtr sub_hand_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
