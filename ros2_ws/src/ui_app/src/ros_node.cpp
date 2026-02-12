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
    this->declare_parameter<std::string>("robot_urdf_path", "");
    this->declare_parameter<std::string>("left_hand_urdf_path", "");
    this->declare_parameter<std::string>("right_hand_urdf_path", "");

    std::string robot_ip;
    this->get_parameter("robot_ip", robot_ip);
    this->get_parameter("robot_urdf_path", robot_urdf_path_);
    this->get_parameter("left_hand_urdf_path", left_hand_urdf_path_);
    this->get_parameter("right_hand_urdf_path", right_hand_urdf_path_);
    
    RCLCPP_INFO(this->get_logger(), "UI Configured with Robot IP: %s", robot_ip.c_str());
    if (!robot_urdf_path_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Robot URDF Path: %s", robot_urdf_path_.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Robot URDF Path is empty! Visualization may not work.");
    }
    if (!left_hand_urdf_path_.empty()) RCLCPP_INFO(this->get_logger(), "Left Hand URDF: %s", left_hand_urdf_path_.c_str());
    if (!right_hand_urdf_path_.empty()) RCLCPP_INFO(this->get_logger(), "Right Hand URDF: %s", right_hand_urdf_path_.c_str());

    // Subscriber: Duco Robot State
    sub_robot_state_ = create_subscription<duco_msg::msg::DucoRobotState>(
      "/duco_cobot/robot_state", 30, std::bind(&RosNode::robot_state_callback, this, std::placeholders::_1));

    // Service Clients
    client_control_ = create_client<duco_msg::srv::RobotControl>("/ui/request_control");
    client_io_ = create_client<duco_msg::srv::RobotIoControl>("/ui/request_io");
    client_move_ = create_client<duco_msg::srv::RobotMove>("/ui/request_move");
    // New Save Image Service (Vision Server)
    client_save_image_ = create_client<vision_server::srv::SaveImage>("save_image");

    // LHand Clients
    client_lhand_enable_ = create_client<lhandpro_interfaces::srv::SetEnable>("/lhandpro_service/set_enable");
    client_lhand_pos_ = create_client<lhandpro_interfaces::srv::SetPosition>("/lhandpro_service/set_position");
    client_lhand_all_pos_ = create_client<lhandpro_interfaces::srv::SetAllPosition>("/lhandpro_service/set_all_position");
    client_lhand_vel_ = create_client<lhandpro_interfaces::srv::SetPositionVelocity>("/lhandpro_service/set_position_velocity");
    client_lhand_move_ = create_client<lhandpro_interfaces::srv::MoveMotors>("/lhandpro_service/move_motors");
    client_lhand_home_ = create_client<lhandpro_interfaces::srv::HomeMotors>("/lhandpro_service/home_motors");

    // Default Camera Subscriptions (camera, Color+Depth)
    update_camera_subscriptions("camera", true, true, false, false, false);

    // TF Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    
    RCLCPP_INFO(this->get_logger(), "Scanning for cameras...");
    for (const auto& [name, types] : topic_names_and_types) {
        // Debug log all topics
        // RCLCPP_DEBUG(this->get_logger(), "Topic: %s", name.c_str());

        // Look for topics ending in /color/image_raw
        if (name.find("/color/image_raw") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Found potential camera topic: %s", name.c_str());
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
                RCLCPP_INFO(this->get_logger(), "Added camera: %s", ns.c_str());
            }
        }
    }
    // Sort and remove duplicates
    std::sort(cameras.begin(), cameras.end());
    cameras.erase(std::unique(cameras.begin(), cameras.end()), cameras.end());
    
    if (cameras.empty()) {
        // Fallback if no cameras found
        // cameras.push_back("camera"); // Removed hardcoded fallback that causes confusion
    }
    return cameras;
}

std::vector<std::string> RosNode::scan_point_clouds() {
    std::vector<std::string> topics;
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    for (const auto& [name, types] : topic_names_and_types) {
        for (const auto& type : types) {
            if (type == "sensor_msgs/msg/PointCloud2") {
                topics.push_back(name);
                break;
            }
        }
    }
    std::sort(topics.begin(), topics.end());
    return topics;
}

RosNode::CameraCapabilities RosNode::get_camera_capabilities(std::string camera_ns) {
    CameraCapabilities caps;
    
    // Normalize namespace
    if (camera_ns.back() == '/') camera_ns.pop_back();
    if (camera_ns.front() != '/') camera_ns = "/" + camera_ns;

    auto topic_names_and_types = this->get_topic_names_and_types();
    auto topics = topic_names_and_types; // Copy map

    // Helper to check existence
    auto has_topic = [&](std::string suffix) -> bool {
        std::string full_name = camera_ns + suffix;
        return topics.find(full_name) != topics.end();
    };

    caps.has_color = has_topic("/color/image_raw");
    caps.has_depth = has_topic("/depth/image_raw");
    caps.has_ir_left = has_topic("/left_ir/image_raw");
    caps.has_ir_right = has_topic("/right_ir/image_raw");
    caps.has_point_cloud = has_topic("/depth/color/points"); // Default guess

    // Fallback/Enhancement logic
    if (!caps.has_ir_left) {
        // Check for mono IR
        if (has_topic("/ir/image_raw")) {
            caps.has_ir_left = true; // Map mono IR to Left
            // caps.has_ir_right remains false
        }
    }

    return caps;
}

void RosNode::update_camera_subscriptions(std::string camera_ns, bool color, bool depth, bool ir_left, bool ir_right, bool point_cloud, std::string pc_topic) {
    // Unsubscribe all
    sub_color_.reset();
    sub_depth_.reset();
    sub_ir_left_.reset();
    sub_ir_right_.reset();
    sub_point_cloud_.reset();

    // Remove trailing slash if user provided one (though we extracted without it)
    if (camera_ns.back() == '/') camera_ns.pop_back();
    // Ensure leading slash
    if (camera_ns.front() != '/') camera_ns = "/" + camera_ns;

    RCLCPP_INFO(get_logger(), "Updating subs for camera: %s [C:%d D:%d L:%d R:%d P:%d Topic:%s]", 
        camera_ns.c_str(), color, depth, ir_left, ir_right, point_cloud, pc_topic.c_str());

    if (color) {
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            camera_ns + "/color/image_raw", rclcpp::SensorDataQoS(), std::bind(&RosNode::color_callback, this, std::placeholders::_1));
    }
    if (depth) {
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            camera_ns + "/depth/image_raw", rclcpp::SensorDataQoS(), std::bind(&RosNode::depth_callback, this, std::placeholders::_1));
    }
    if (ir_left) {
        std::string ir_topic = camera_ns + "/left_ir/image_raw";
        // Check if "ir/image_raw" exists instead (for 210 series)
        auto topics = this->get_topic_names_and_types();
        std::string mono_ir = camera_ns + "/ir/image_raw";
        if (topics.find(mono_ir) != topics.end()) {
            ir_topic = mono_ir;
            RCLCPP_INFO(get_logger(), "Using mono IR topic: %s", ir_topic.c_str());
        }

        sub_ir_left_ = create_subscription<sensor_msgs::msg::Image>(
            ir_topic, rclcpp::SensorDataQoS(), std::bind(&RosNode::ir_left_callback, this, std::placeholders::_1));
    }
    if (ir_right) {
        std::string ir_topic = camera_ns + "/right_ir/image_raw";
        // Check if "ir/image_raw" exists instead (for 210 series fallback or similar mono cam)
        // If "left_ir" was mapped to "ir", we shouldn't map "right_ir" to the same unless desired.
        // But for 210 series which is mono IR, usually users expect IR Left to show the IR stream.
        // IR Right is simply not available.
        // However, if the user explicitly checks IR Right, we can check if a dedicated topic exists.
        
        // Let's just subscribe standard right_ir. If it doesn't exist, it won't receive data.
        // Or we can be smart: if left_ir mapped to /ir/image_raw, maybe right is not needed.
        
        sub_ir_right_ = create_subscription<sensor_msgs::msg::Image>(
            ir_topic, rclcpp::SensorDataQoS(), std::bind(&RosNode::ir_right_callback, this, std::placeholders::_1));
    }
    if (point_cloud) {
        std::string topic = pc_topic;
        if (topic.empty()) {
             // Fallback default
             topic = camera_ns + "/depth/color/points";
        }
        sub_point_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(), std::bind(&RosNode::point_cloud_callback, this, std::placeholders::_1));
    }
}

void RosNode::save_snapshot(std::string camera_ns, bool color, bool depth, bool ir_left, bool ir_right, std::function<void(bool, std::string)> callback) {
    if (!client_save_image_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "Save Image service not available");
        if(callback) callback(false, "Service not available");
        return;
    }

    if (camera_ns.back() == '/') camera_ns.pop_back();
    if (camera_ns.front() != '/') camera_ns = "/" + camera_ns;

    auto send_request = [this, callback](std::string topic, std::string tag) {
        auto request = std::make_shared<vision_server::srv::SaveImage::Request>();
        request->topic_name = topic;
        request->file_tag = tag;
        
        client_save_image_->async_send_request(request, 
            [this, topic, callback](rclcpp::Client<vision_server::srv::SaveImage>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(get_logger(), "Saved %s: %s", topic.c_str(), response->message.c_str());
                        if(callback) callback(true, response->message);
                    } else {
                        RCLCPP_WARN(get_logger(), "Failed to save %s: %s", topic.c_str(), response->message.c_str());
                        if(callback) callback(false, "Failed: " + response->message);
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(get_logger(), "Service call failed for %s: %s", topic.c_str(), e.what());
                    if(callback) callback(false, std::string("Exception: ") + e.what());
                }
            });
    };

    if (color) send_request(camera_ns + "/color/image_raw", "Color");
    if (depth) send_request(camera_ns + "/depth/image_raw", "Depth");
    // Handle IR: if "left" is requested but might be single IR
    if (ir_left) {
        // Try the standard left_ir first, if we want to be strict.
        // But user said "camera_Axxx/ir" is the one.
        // We will prioritize checking if "/ir/image_raw" exists or just default to it if it is camera_Axxx?
        // Simpler approach: Send request to "ir/image_raw" if "left_ir/image_raw" is not the convention for single IR.
        // However, we don't know for sure which camera it is dynamically here without checking topics.
        // Let's trust the user's specific request: "one is camera_Axxx/ir".
        // We can try to support both or just switch based on the user's report.
        // Given the user said "Handle it" implying a fix for their current setup:
        // We will change the logic to use "ir/image_raw" if it's the specific single-IR camera case, 
        // OR we can send to both or try one then the other.
        // But the "SaveImage" service just waits for a topic. If we send the wrong topic, it timeouts.
        
        // Let's assume if it is "camera_Axxx", use "ir".
        if (camera_ns.find("camera_A") != std::string::npos) {
             send_request(camera_ns + "/ir/image_raw", "IR");
        } else {
             send_request(camera_ns + "/left_ir/image_raw", "IR_Left");
        }
    }
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

void RosNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Just store the pointer, processing happens in UI thread or worker
    std::lock_guard<std::mutex> lock(data_mutex_); // Reuse data_mutex or use image_mutex? Let's use image_mutex for all visual data
    last_point_cloud_ = msg;
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

// LHand Implementation
void RosNode::call_lhand_enable(int joint_id, int enable) {
    if (!client_lhand_enable_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand Enable service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::SetEnable::Request>();
    request->joint_id = joint_id;
    request->enable = enable;
    
    using ServiceT = lhandpro_interfaces::srv::SetEnable;
    client_lhand_enable_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand Enable result: %d", response->result);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand Enable failed: %s", e.what());
            }
        });
}

void RosNode::call_lhand_home(int joint_id) {
    if (!client_lhand_home_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand Home service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::HomeMotors::Request>();
    request->joint_id = joint_id;
    
    using ServiceT = lhandpro_interfaces::srv::HomeMotors;
    client_lhand_home_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand Home result: %d", response->result);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand Home failed: %s", e.what());
            }
        });
}

void RosNode::call_lhand_set_position(int joint_id, int position) {
    if (!client_lhand_pos_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand SetPosition service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::SetPosition::Request>();
    request->joint_id = joint_id;
    request->position = position;
    
    using ServiceT = lhandpro_interfaces::srv::SetPosition;
    client_lhand_pos_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand SetPosition result: %d", response->result);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand SetPosition failed: %s", e.what());
            }
        });
}

void RosNode::call_lhand_set_all_position(const std::array<int, 6>& positions) {
    if (!client_lhand_all_pos_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand SetAllPosition service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::SetAllPosition::Request>();
    // Copy array to request
    for(size_t i=0; i<6; ++i) {
        request->positions[i] = positions[i];
    }
    
    using ServiceT = lhandpro_interfaces::srv::SetAllPosition;
    client_lhand_all_pos_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand SetAllPosition result: %d", response->result);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand SetAllPosition failed: %s", e.what());
            }
        });
}

void RosNode::call_lhand_set_velocity(int joint_id, int velocity) {
    if (!client_lhand_vel_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand SetVelocity service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::SetPositionVelocity::Request>();
    request->joint_id = joint_id;
    request->velocity = velocity;
    
    using ServiceT = lhandpro_interfaces::srv::SetPositionVelocity;
    client_lhand_vel_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand SetVelocity result: %d", response->result);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand SetVelocity failed: %s", e.what());
            }
        });
}

void RosNode::call_lhand_move(int joint_id) {
    if (!client_lhand_move_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand Move service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::MoveMotors::Request>();
    request->joint_id = joint_id;
    
    using ServiceT = lhandpro_interfaces::srv::MoveMotors;
    client_lhand_move_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand Move result: %d", response->result);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand Move failed: %s", e.what());
            }
        });
}
