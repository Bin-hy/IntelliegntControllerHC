#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <duco_msg/srv/robot_control.hpp>
#include <duco_msg/srv/robot_io_control.hpp>
#include <duco_msg/srv/robot_move.hpp>
#include <duco_msg/msg/duco_robot_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include "vision_server/srv/save_image.hpp"
#include "lhandpro_interfaces/srv/set_enable.hpp"
#include "lhandpro_interfaces/srv/set_position.hpp"
#include "lhandpro_interfaces/srv/set_all_position.hpp"
#include "lhandpro_interfaces/srv/set_position_velocity.hpp"
#include "lhandpro_interfaces/srv/move_motors.hpp"
#include "lhandpro_interfaces/srv/home_motors.hpp"
#include "lhandpro_interfaces/srv/get_now_position.hpp"
// #include "vision_server/vision_server/srv/save_image.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include "common_msgs/action/execute_task.hpp"
#include "common_msgs/msg/device_status.hpp"
#include "common_msgs/msg/collision_status.hpp"
#include "common_msgs/msg/depth_measurement.hpp"
#include "common_msgs/srv/set_current_user.hpp"
#include "common_msgs/srv/trigger_grasp.hpp"
#include "common_msgs/srv/calibrate_hand_eye.hpp"
#include "lift_server/srv/lift_control.hpp"

class RosNode : public rclcpp::Node {
public:
  using ExecuteTask = common_msgs::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  RosNode();

  void call_robot_control(const std::string& command);
  void call_robot_move_joint(const std::vector<double>& joints);
  void call_execute_task(const common_msgs::msg::TaskConfig& task_config, 
                         std::function<void(const GoalHandleExecuteTask::WrappedResult&)> result_callback,
                         std::function<void(const std::shared_ptr<const ExecuteTask::Feedback>)> feedback_callback);
  
  void cancel_current_task();

  void call_robot_move(const std::string& command, 
                       const std::vector<float>& p, 
                       const std::vector<float>& q, 
                       float v, float a, float r, 
                       const std::string& tool, const std::string& wobj);
  void call_robot_io(const std::string& command, int type, int port, bool value,
                     std::function<void(const std::string&)> callback = nullptr);
  void call_pause_task(bool pause);
  void set_collision_camera(const std::string& camera_ns);
  void set_user_context(const std::string& username, const std::string& role, const std::string& session_id);
  // void save_image(); // Deprecated in favor of multi-camera
  void save_snapshot(const std::string& camera_ns, bool color, bool depth, bool ir_left, bool ir_right, bool point_cloud, std::function<void(bool, std::string)> callback = nullptr);

  // LHand Control
  // Lift platform control
  void call_lift_control(const std::string& command, int speed_rpm = 100,
                         std::function<void(bool, const std::string&, int)> callback = nullptr,
                         int target_pulses = 0, int accel_ms = 0, int decel_ms = 0);

  // Hand-eye calibration TF (UI preview — for permanent calibration use call_hand_eye_calibrate)
  void publish_hand_eye_tf(double tx, double ty, double tz,
                           double rx_deg, double ry_deg, double rz_deg,
                           const std::string& child_frame = "cam_305_link");

  // Hand-eye calibration service (delegates to vision_grasp/hand_eye_calibration node)
  void call_hand_eye_calibrate(const std::string& command, double select_u = -1, double select_v = -1,
                                std::function<void(bool, const std::string&, int, double, double)> callback = nullptr);

  // Vision grasp trigger
  void call_trigger_grasp(double u, double v,
                          std::function<void(bool, const std::string&)> callback = nullptr);

  // Update grasp offset compensation in running coordinator (via set_parameters service)
  void set_grasp_offset(double ox, double oy, double oz);

  void call_lhand_enable(bool enable); // Use global enable for now, or default joint_id if needed
  void call_lhand_home(int joint_id, std::function<void(int)> callback = nullptr);
  void call_lhand_set_position(int joint_id, int position);
  void call_lhand_set_all_position(const std::array<int, 6>& positions);
  void call_lhand_set_velocity(int joint_id, int velocity);
  void call_lhand_move(int joint_id);
  void call_lhand_get_position(int joint_id, std::function<void(int)> callback);

  // Right hand (rhandpro_service)
  void call_rhand_set_all_position(const std::array<int, 6>& positions);
  void call_rhand_move(int joint_id);
  
  std::vector<std::string> scan_cameras();
  std::vector<std::string> scan_point_clouds();
  
  struct CameraCapabilities {
      bool has_color = false;
      bool has_depth = false;
      bool has_ir_left = false;
      bool has_ir_right = false;
      bool has_point_cloud = false;
      std::string ir_left_topic;      // e.g. "/SN/left_ir/image_raw" or "/SN/ir/image_raw"
      std::string ir_right_topic;     // e.g. "/SN/right_ir/image_raw"
      std::string point_cloud_topic;  // e.g. "/SN/depth_registered/points" or "/SN/depth/points"
  };
  CameraCapabilities get_camera_capabilities(std::string camera_ns);
  CameraCapabilities last_caps_;

  void update_camera_subscriptions(const std::string& camera_ns, bool color, bool depth, bool ir_left, bool ir_right, bool point_cloud, std::string pc_topic = "");

  bool check_device_availability(const common_msgs::msg::TaskDeviceCheck& check);
  bool is_task_action_ready() const;
  std::vector<common_msgs::msg::DeviceStatus> get_connected_devices();
  bool is_hand_connected(const std::string& side);

  // Getters for URDF paths
  std::string get_robot_urdf_path() const { return robot_urdf_path_; }
  std::string get_left_hand_urdf_path() const { return left_hand_urdf_path_; }
  std::string get_right_hand_urdf_path() const { return right_hand_urdf_path_; }

  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() { return tf_buffer_; }

  // Data storage
  std::atomic<int> count_;
  duco_msg::msg::DucoRobotState::SharedPtr last_robot_state_;
  std::string last_robot_state_str_;
  std::string last_collision_str_;        // protected by data_mutex_
  std::string last_depth_measure_str_;    // protected by data_mutex_

  std::vector<double> current_joints_;
  std::vector<double> current_cart_pos_;
  
  // Image storage
  cv::Mat last_color_image_;
  cv::Mat last_depth_image_;     // 8-bit normalized for display
  cv::Mat last_depth_raw_;       // 16UC1 original millimeter values
  cv::Mat last_ir_left_image_;
  cv::Mat last_ir_right_image_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_point_cloud_;

  // Returns depth in mm at image pixel (u,v).
  // MUST be called while holding image_mutex_ (caller's responsibility).
  uint16_t get_depth_mm_locked(int u, int v) const {
      if (last_depth_raw_.empty()) return 0;
      if (u < 0 || v < 0 || u >= last_depth_raw_.cols || v >= last_depth_raw_.rows) return 0;
      return last_depth_raw_.at<uint16_t>(v, u);
  }
  
  std::mutex data_mutex_;
  std::mutex image_mutex_;

private:
  void robot_state_callback(const duco_msg::msg::DucoRobotState::SharedPtr msg);
  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void ir_left_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void ir_right_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void device_status_callback(const common_msgs::msg::DeviceStatus::SharedPtr msg);
  void collision_status_callback(const common_msgs::msg::CollisionStatus::SharedPtr msg);
  void depth_measure_callback(const common_msgs::msg::DepthMeasurement::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_robot_state_;
  rclcpp::Subscription<common_msgs::msg::DeviceStatus>::SharedPtr sub_device_status_;
  
  std::map<std::string, common_msgs::msg::DeviceStatus> connected_devices_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir_right_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Subscription<common_msgs::msg::CollisionStatus>::SharedPtr sub_collision_;
  rclcpp::Subscription<common_msgs::msg::DepthMeasurement>::SharedPtr sub_depth_measure_;
  
  rclcpp::Client<duco_msg::srv::RobotControl>::SharedPtr client_control_;
  rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedPtr client_io_;
  rclcpp::Client<duco_msg::srv::RobotMove>::SharedPtr client_move_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_pause_task_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_collision_topic_;
  rclcpp::Client<common_msgs::srv::SetCurrentUser>::SharedPtr client_set_user_;

  // URDF Paths
  std::string robot_urdf_path_;
  std::string left_hand_urdf_path_;
  std::string right_hand_urdf_path_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_save_image_;
  rclcpp::Client<vision_server::srv::SaveImage>::SharedPtr client_save_image_;

  // LHand Clients
  rclcpp::Client<lhandpro_interfaces::srv::SetEnable>::SharedPtr client_lhand_enable_;
  rclcpp::Client<lhandpro_interfaces::srv::SetPosition>::SharedPtr client_lhand_pos_;
  rclcpp::Client<lhandpro_interfaces::srv::SetAllPosition>::SharedPtr client_lhand_all_pos_;
  rclcpp::Client<lhandpro_interfaces::srv::SetPositionVelocity>::SharedPtr client_lhand_vel_;
  rclcpp::Client<lhandpro_interfaces::srv::MoveMotors>::SharedPtr client_lhand_move_;
  rclcpp::Client<lhandpro_interfaces::srv::HomeMotors>::SharedPtr client_lhand_home_;
  rclcpp::Client<lhandpro_interfaces::srv::GetNowPosition>::SharedPtr client_lhand_get_now_pos_;

  // RHand Clients (mirror of LHand, under /rhandpro_service namespace)
  rclcpp::Client<lhandpro_interfaces::srv::SetEnable>::SharedPtr client_rhand_enable_;
  rclcpp::Client<lhandpro_interfaces::srv::SetPosition>::SharedPtr client_rhand_pos_;
  rclcpp::Client<lhandpro_interfaces::srv::SetAllPosition>::SharedPtr client_rhand_all_pos_;
  rclcpp::Client<lhandpro_interfaces::srv::SetPositionVelocity>::SharedPtr client_rhand_vel_;
  rclcpp::Client<lhandpro_interfaces::srv::MoveMotors>::SharedPtr client_rhand_move_;
  rclcpp::Client<lhandpro_interfaces::srv::HomeMotors>::SharedPtr client_rhand_home_;
  rclcpp::Client<lhandpro_interfaces::srv::GetNowPosition>::SharedPtr client_rhand_get_now_pos_;

  // Lift platform client
  rclcpp::Client<lift_server::srv::LiftControl>::SharedPtr client_lift_;

  // Vision grasp client
  rclcpp::Client<common_msgs::srv::TriggerGrasp>::SharedPtr client_trigger_grasp_;

  // Hand-eye calibration client
  rclcpp::Client<common_msgs::srv::CalibrateHandEye>::SharedPtr client_hand_eye_calibrate_;

  rclcpp_action::Client<ExecuteTask>::SharedPtr client_execute_task_;
  GoalHandleExecuteTask::SharedPtr current_goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ROS_NODE_HPP
