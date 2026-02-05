#include "ui_app/ros_node.hpp"
#include <sstream>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

RosNode::RosNode() : rclcpp::Node("ui_ros_node"), count_(0) {
    // Publisher
    pub_ = create_publisher<std_msgs::msg::String>("/ui/heartbeat", 10);
    
    // Parameters
    this->declare_parameter<std::string>("robot_ip", "192.168.1.10");
    std::string robot_ip;
    this->get_parameter("robot_ip", robot_ip);
    RCLCPP_INFO(this->get_logger(), "UI Configured with Robot IP: %s", robot_ip.c_str());

    // Subscriber: Duco Robot State
    sub_robot_state_ = create_subscription<duco_msg::msg::DucoRobotState>(
      "/duco_robot/robot_state", 30, std::bind(&RosNode::robot_state_callback, this, std::placeholders::_1));

    // Service Clients
    client_control_ = create_client<duco_msg::srv::RobotControl>("/ui/request_control");
    client_io_ = create_client<duco_msg::srv::RobotIoControl>("/ui/request_io");
    client_move_ = create_client<duco_msg::srv::RobotMove>("/ui/request_move");
    // New Save Image Service (Vision Server)
    client_save_image_ = create_client<vision_server::srv::SaveImage>("save_image");

    // Default Camera Subscriptions (camera, Color+Depth)
    update_camera_subscriptions("camera", true, true, false, false);

    // Timer
    timer_ = create_wall_timer(std::chrono::seconds(1), [this](){
      std_msgs::msg::String msg;
      msg.data = "ok";
      pub_->publish(msg);
      ++count_;
    });
}

std::vector<std::string> RosNode::scan_cameras() {
    std::vector<std::string> cameras;
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    for (const auto& [name, types] : topic_names_and_types) {
        // Look for topics ending in /color/image_raw
        if (name.find("/color/image_raw") != std::string::npos) {
            // Extract namespace
            // e.g. /camera/color/image_raw -> /camera
            // e.g. /camera_01/color/image_raw -> /camera_01
            std::string suffix = "/color/image_raw";
            if (name.length() > suffix.length()) {
                std::string ns = name.substr(0, name.length() - suffix.length());
                // Remove leading slash if strictly relative? No, keep absolute.
                // If it is empty (root), handle gracefully.
                if (ns.empty()) ns = "/"; // unlikely for standard usage
                cameras.push_back(ns);
            }
        }
    }
    // Sort and remove duplicates
    std::sort(cameras.begin(), cameras.end());
    cameras.erase(std::unique(cameras.begin(), cameras.end()), cameras.end());
    
    if (cameras.empty()) {
        // Fallback if no cameras found
        cameras.push_back("camera");
    }
    return cameras;
}

void RosNode::update_camera_subscriptions(std::string camera_ns, bool color, bool depth, bool ir_left, bool ir_right) {
    // Unsubscribe all
    sub_color_.reset();
    sub_depth_.reset();
    sub_ir_left_.reset();
    sub_ir_right_.reset();

    // Remove trailing slash if user provided one (though we extracted without it)
    if (camera_ns.back() == '/') camera_ns.pop_back();
    // Ensure leading slash
    if (camera_ns.front() != '/') camera_ns = "/" + camera_ns;

    RCLCPP_INFO(get_logger(), "Updating subs for camera: %s [C:%d D:%d L:%d R:%d]", 
        camera_ns.c_str(), color, depth, ir_left, ir_right);

    if (color) {
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            camera_ns + "/color/image_raw", 10, std::bind(&RosNode::color_callback, this, std::placeholders::_1));
    }
    if (depth) {
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            camera_ns + "/depth/image_raw", 10, std::bind(&RosNode::depth_callback, this, std::placeholders::_1));
    }
    if (ir_left) {
        // Standard Orbbec mapping often uses "left_ir" or "ir/left" depending on launch.
        // Based on config "leftir", it is likely "left_ir" or "ir_left".
        // Let's assume "left_ir" as it is common.
        sub_ir_left_ = create_subscription<sensor_msgs::msg::Image>(
            camera_ns + "/left_ir/image_raw", 10, std::bind(&RosNode::ir_left_callback, this, std::placeholders::_1));
    }
    if (ir_right) {
        sub_ir_right_ = create_subscription<sensor_msgs::msg::Image>(
            camera_ns + "/right_ir/image_raw", 10, std::bind(&RosNode::ir_right_callback, this, std::placeholders::_1));
    }
}

void RosNode::save_snapshot(std::string camera_ns, bool color, bool depth, bool ir_left, bool ir_right) {
    if (!client_save_image_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "Save Image service not available");
        return;
    }

    if (camera_ns.back() == '/') camera_ns.pop_back();
    if (camera_ns.front() != '/') camera_ns = "/" + camera_ns;

    auto send_request = [this](std::string topic, std::string tag) {
        auto request = std::make_shared<vision_server::srv::SaveImage::Request>();
        request->topic_name = topic;
        request->file_tag = tag;
        
        client_save_image_->async_send_request(request, 
            [this, topic](rclcpp::Client<vision_server::srv::SaveImage>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(get_logger(), "Saved %s: %s", topic.c_str(), response->message.c_str());
                    } else {
                        RCLCPP_WARN(get_logger(), "Failed to save %s: %s", topic.c_str(), response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(get_logger(), "Service call failed for %s: %s", topic.c_str(), e.what());
                }
            });
    };

    if (color) send_request(camera_ns + "/color/image_raw", "Color");
    if (depth) send_request(camera_ns + "/depth/image_raw", "Depth");
    if (ir_left) send_request(camera_ns + "/left_ir/image_raw", "IR_Left");
    if (ir_right) send_request(camera_ns + "/right_ir/image_raw", "IR_Right");
}

void RosNode::color_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    try {
        last_color_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception (color): %s", e.what());
    }
}

void RosNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    try {
        // Normalize 16UC1 to 8UC1 for display
        cv::Mat depth_raw = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        cv::normalize(depth_raw, last_depth_image_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception (depth): %s", e.what());
    }
}

void RosNode::ir_left_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    try {
        // IR is usually 8UC1 or 16UC1. If 16, normalize.
        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv::Mat ir_raw = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            cv::normalize(ir_raw, last_ir_left_image_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        } else {
            last_ir_left_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception (ir_left): %s", e.what());
    }
}

void RosNode::ir_right_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    try {
         if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            cv::Mat ir_raw = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            cv::normalize(ir_raw, last_ir_right_image_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        } else {
            last_ir_right_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception (ir_right): %s", e.what());
    }
}

void RosNode::call_robot_control(const std::string& command) {
    if (!client_control_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Robot Control service not available");
      return;
    }
    auto request = std::make_shared<duco_msg::srv::RobotControl::Request>();
    request->command = command;
    request->arm_num = 0;
    request->block = true;
    
    client_control_->async_send_request(request, 
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
    request->block = true;

    RCLCPP_INFO(get_logger(), "Sending Move Command: %s", command.c_str());

    client_move_->async_send_request(request, 
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
      request->block = true;

      client_io_->async_send_request(request,
          [this, command, port, value](rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedFuture future) {
              try {
                  auto response = future.get();
                  RCLCPP_INFO(get_logger(), "IO '%s' result: %s", command.c_str(), response->response.c_str());
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

      current_cart_pos_.clear();
      for(size_t i=0; i<6; ++i) {
           current_cart_pos_.push_back(msg->cart_actual_position[i]);
      }
      
      last_robot_state_str_ = ss.str();
}
