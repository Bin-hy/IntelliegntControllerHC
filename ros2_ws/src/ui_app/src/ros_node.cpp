#include "ui_app/ros_node.hpp"
#include <sstream>
#include <chrono>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

RosNode::RosNode() : rclcpp::Node("ui_ros_node"), count_(0) {
    pub_ = create_publisher<std_msgs::msg::String>("/ui/heartbeat", 10);
    
    this->declare_parameter<std::string>("robot_ip", "192.168.1.10");
    this->declare_parameter<std::string>("robot_urdf_path", "");
    this->declare_parameter<std::string>("left_hand_urdf_path", "");
    this->declare_parameter<std::string>("right_hand_urdf_path", "");
    this->declare_parameter<std::string>("robot_model", "gcr16_960");

    std::string robot_ip;
    std::string robot_model;
    this->get_parameter("robot_ip", robot_ip);
    this->get_parameter("robot_urdf_path", robot_urdf_path_);
    this->get_parameter("left_hand_urdf_path", left_hand_urdf_path_);
    this->get_parameter("right_hand_urdf_path", right_hand_urdf_path_);
    this->get_parameter("robot_model", robot_model);

    if (robot_urdf_path_.empty()) {
        try {
            std::string duco_share = ament_index_cpp::get_package_share_directory("duco_support");
            robot_urdf_path_ = duco_share + "/urdf/duco_" + robot_model + ".urdf";
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to resolve duco_support share directory: %s", e.what());
        }
    }
    
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

    // Subscriber: Device Status
    sub_device_status_ = create_subscription<common_msgs::msg::DeviceStatus>(
      "/system/device_status", 10, std::bind(&RosNode::device_status_callback, this, std::placeholders::_1));

    // Service Clients
    client_control_ = create_client<duco_msg::srv::RobotControl>("/ui/request_control");
    client_io_ = create_client<duco_msg::srv::RobotIoControl>("/ui/request_io");
    client_move_ = create_client<duco_msg::srv::RobotMove>("/ui/request_move");
    client_pause_task_ = create_client<std_srvs::srv::SetBool>("/system/pause_task");
    client_set_user_ = create_client<common_msgs::srv::SetCurrentUser>("/system/set_current_user");
    // New Save Image Service (Vision Server)
    client_save_image_ = create_client<vision_server::srv::SaveImage>("save_image");

    // LHand Clients
    client_lhand_enable_ = create_client<lhandpro_interfaces::srv::SetEnable>("/lhandpro_service/set_enable");
    client_lhand_pos_ = create_client<lhandpro_interfaces::srv::SetPosition>("/lhandpro_service/set_position");
    client_lhand_all_pos_ = create_client<lhandpro_interfaces::srv::SetAllPosition>("/lhandpro_service/set_all_position");
    client_lhand_vel_ = create_client<lhandpro_interfaces::srv::SetPositionVelocity>("/lhandpro_service/set_position_velocity");
    client_lhand_move_ = create_client<lhandpro_interfaces::srv::MoveMotors>("/lhandpro_service/move_motors");
    client_lhand_home_ = create_client<lhandpro_interfaces::srv::HomeMotors>("/lhandpro_service/home_motors");
    client_lhand_get_now_pos_ = create_client<lhandpro_interfaces::srv::GetNowPosition>("/lhandpro_service/get_now_position");

    // Task Execution Action Client
    client_execute_task_ = rclcpp_action::create_client<ExecuteTask>(this, "execute_task");

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
                // Avoid duplicates
                if (std::find(cameras.begin(), cameras.end(), ns) == cameras.end()) {
                    cameras.push_back(ns);
                }
            }
        }
    }
    if (cameras.empty()) {
        RCLCPP_WARN(this->get_logger(), "No cameras found via topic scanning. Defaulting to 'camera'.");
        cameras.push_back("camera");
    }
    return cameras;
}

void RosNode::update_camera_subscriptions(const std::string& camera_ns, bool color, bool depth, bool ir_left, bool ir_right, bool point_cloud, std::string pc_topic) {
    // Unsubscribe existing
    sub_color_.reset();
    sub_depth_.reset();
    sub_ir_left_.reset();
    sub_ir_right_.reset();
    sub_point_cloud_.reset();

    // Using simple create_subscription with specific types to avoid template issues
    if (color) {
        std::string topic = camera_ns + "/color/image_raw";
        RCLCPP_INFO(this->get_logger(), "Subscribing to Color: %s", topic.c_str());
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&RosNode::color_callback, this, std::placeholders::_1));
    }
    if (depth) {
        std::string topic = camera_ns + "/depth/image_raw"; // Or aligned_depth_to_color
        RCLCPP_INFO(this->get_logger(), "Subscribing to Depth: %s", topic.c_str());
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&RosNode::depth_callback, this, std::placeholders::_1));
    }
    if (ir_left) {
        // Some cameras use "ir/image_raw", others "left_ir/image_raw"
        // Try to guess or just use standard convention for Orbbec/Realsense
        std::string topic = camera_ns + "/ir/image_raw"; 
        if (camera_ns.find("camera_A") != std::string::npos) { // Custom logic based on user hint
             topic = camera_ns + "/ir/image_raw";
        } else {
             topic = camera_ns + "/left_ir/image_raw"; 
        }
        RCLCPP_INFO(this->get_logger(), "Subscribing to IR Left: %s", topic.c_str());
        sub_ir_left_ = create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&RosNode::ir_left_callback, this, std::placeholders::_1));
    }
    if (ir_right) {
        std::string topic = camera_ns + "/right_ir/image_raw";
        RCLCPP_INFO(this->get_logger(), "Subscribing to IR Right: %s", topic.c_str());
        sub_ir_right_ = create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&RosNode::ir_right_callback, this, std::placeholders::_1));
    }
    if (point_cloud) {
        std::string topic = pc_topic.empty() ? (camera_ns + "/depth/color/points") : pc_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribing to PointCloud: %s", topic.c_str());
        sub_point_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic, 10, std::bind(&RosNode::point_cloud_callback, this, std::placeholders::_1));
    }
}

void RosNode::robot_state_callback(const duco_msg::msg::DucoRobotState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    last_robot_state_ = msg;

    std::stringstream ss;
    ss << "State Code: " << static_cast<int>(msg->robot_state) << "\n";
    ss << "Error Code: " << msg->robot_error << "\n";
    ss << "Collision: " << (msg->collision ? "YES" : "NO") << "\n";

    current_joints_.clear();
    ss << "Joint Pos: [";
    const auto joints_size = msg->joint_actual_position.size();
    for (size_t i = 0; i < joints_size; ++i) {
        ss << msg->joint_actual_position[i];
        if (i + 1 < joints_size) ss << ", ";
        current_joints_.push_back(msg->joint_actual_position[i]);
    }
    ss << "] (" << joints_size << " DOF)\n";

    current_cart_pos_.clear();
    const auto cart_size = msg->cart_actual_position.size();
    const size_t cart_to_copy = cart_size < 6 ? cart_size : 6;
    for (size_t i = 0; i < cart_to_copy; ++i) {
        current_cart_pos_.push_back(msg->cart_actual_position[i]);
    }

    last_robot_state_str_ = ss.str();

    common_msgs::msg::DeviceStatus status;
    status.device_type = "duco";
    status.device_model = "unknown";
    status.status = (msg->robot_state == 6) ? "ready" : "connected";
    status.device_sn = "duco_arm_1";
    
    connected_devices_["duco"] = status;
}

void RosNode::device_status_callback(const common_msgs::msg::DeviceStatus::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Use device_type as key for simplicity in this context, or device_sn if available for multiples
    std::string key = msg->device_type;
    if (!msg->device_sn.empty()) {
        key += ":" + msg->device_sn;
    }
    connected_devices_[key] = *msg;
}

std::vector<common_msgs::msg::DeviceStatus> RosNode::get_connected_devices() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<common_msgs::msg::DeviceStatus> devices;
    for (const auto& kv : connected_devices_) {
        devices.push_back(kv.second);
    }
    return devices;
}

bool RosNode::check_device_availability(const common_msgs::msg::TaskDeviceCheck& check) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& kv : connected_devices_) {
        const auto& status = kv.second;
        // Check if device matches requirements
        if (status.device_type == check.device_type ||
            ((check.device_type == "orbbec" || check.device_type == "camera" || check.device_type == "vision_system") &&
             (status.device_type == "orbbec" || status.device_type == "camera_server" || status.device_type == "vision_system"))) {
            // If SN is specified, check it
            if (!check.device_sn.empty() && status.device_sn != check.device_sn) {
                continue;
            }
            // Check status
            if (status.status == "ready" || status.status == "running") {
                return true;
            }
        }
    }
    if (check.device_type == "orbbec" || check.device_type == "camera" || check.device_type == "vision_system") {
        auto topic_names_and_types = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_names_and_types) {
            if (name.find("/color/image_raw") != std::string::npos ||
                name.find("/depth/image_raw") != std::string::npos) {
                if (check.device_sn.empty() || name.find(check.device_sn) != std::string::npos) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool RosNode::is_task_action_ready() const {
    if (!client_execute_task_) return false;
    return client_execute_task_->action_server_is_ready();
}

void RosNode::call_execute_task(const common_msgs::msg::TaskConfig& task_config, 
                       std::function<void(const GoalHandleExecuteTask::WrappedResult&)> result_callback,
                       std::function<void(const std::shared_ptr<const ExecuteTask::Feedback>)> feedback_callback) {
    
    if (!client_execute_task_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = ExecuteTask::Goal();
    goal_msg.task_config = task_config;

    RCLCPP_INFO(get_logger(), "Sending task execution goal...");

    auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        [this](const GoalHandleExecuteTask::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                current_goal_handle_ = goal_handle;
            }
        };

    send_goal_options.feedback_callback =
        [this, feedback_callback](
            GoalHandleExecuteTask::SharedPtr,
            const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
            if (feedback_callback) feedback_callback(feedback);
        };

    send_goal_options.result_callback =
        [this, result_callback](const GoalHandleExecuteTask::WrappedResult & result) {
            current_goal_handle_.reset();
            if (result_callback) result_callback(result);
            
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Task succeeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Task was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Task was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
            }
        };

    client_execute_task_->async_send_goal(goal_msg, send_goal_options);
}

void RosNode::cancel_current_task() {
    if (current_goal_handle_) {
        RCLCPP_INFO(get_logger(), "Canceling current task...");
        client_execute_task_->async_cancel_goal(current_goal_handle_);
    } else {
        RCLCPP_WARN(get_logger(), "No active task to cancel");
    }
}


void RosNode::save_snapshot(const std::string& camera_ns, bool color, bool depth, bool ir_left, bool ir_right, bool point_cloud, std::function<void(bool, std::string)> callback) {
    // Use vision_server service
    if (!client_save_image_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "Vision Server SaveImage service not available");
        if(callback) callback(false, "Service not available");
        return;
    }

    auto send_request = [&](std::string topic, std::string tag) {
        auto request = std::make_shared<vision_server::srv::SaveImage::Request>();
        request->topic_name = topic;
        request->file_tag = tag;
        
        using ServiceT = vision_server::srv::SaveImage;
        client_save_image_->async_send_request(request, 
            [this, topic, callback](rclcpp::Client<ServiceT>::SharedFuture future) {
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
    if (ir_left) {
        if (camera_ns.find("camera_A") != std::string::npos) {
             send_request(camera_ns + "/ir/image_raw", "IR");
        } else {
             send_request(camera_ns + "/left_ir/image_raw", "IR_Left");
        }
    }
    if (ir_right) send_request(camera_ns + "/right_ir/image_raw", "IR_Right");
    if (point_cloud) send_request(camera_ns + "/depth/color/points", "PointCloud");
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
    std::lock_guard<std::mutex> lock(data_mutex_); 
    last_point_cloud_ = msg;
}

void RosNode::call_robot_control(const std::string& command) {
    if (!client_control_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Robot Control service not available");
      return;
    }
    auto request = std::make_shared<duco_msg::srv::RobotControl::Request>();
    request->command = command; // E.g., "power_on", "enable"
    
    using ServiceT = duco_msg::srv::RobotControl;
    client_control_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Control Result: %s", response->response.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Control service call failed: %s", e.what());
            }
        });
}

void RosNode::call_robot_move_joint(const std::vector<double>& joints) {
    if (!client_move_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "Robot Move service not available");
        return;
    }
    auto request = std::make_shared<duco_msg::srv::RobotMove::Request>();
    request->command = "movej"; 
    // Convert double to float
    std::vector<float> q_float(joints.begin(), joints.end());
    request->q = q_float;
    request->v = 20.0; // Default velocity % or value
    request->a = 100.0; // Default accel
    request->r = 0.0;
    request->arm_num = 0;
    
    using ServiceT = duco_msg::srv::RobotMove;
    client_move_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Move Result: %s", response->response.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Move service call failed: %s", e.what());
            }
        });
}

void RosNode::call_robot_io(const std::string& command, int type, int port, bool value) {
    if (!client_io_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "IO Service not available");
        return;
    }
    auto request = std::make_shared<duco_msg::srv::RobotIoControl::Request>();
    request->command = command;
    request->type = type;
    request->port = port;
    request->value = value;
    request->arm_num = 0;
    request->block = false;

    using ServiceT = duco_msg::srv::RobotIoControl;
    client_io_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "IO Result: %s", response->response.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "IO service call failed: %s", e.what());
            }
        });
}

void RosNode::call_pause_task(bool pause) {
    if (!client_pause_task_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Pause Task Service not available");
        return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = pause;
    client_pause_task_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Pause Task Request Result: %s (%s)", response->success ? "Success" : "Failed", response->message.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Pause Task Service call failed: %s", e.what());
        }
    });
}

void RosNode::set_user_context(const std::string& username,
                               const std::string& role,
                               const std::string& session_id) {
    if (!client_set_user_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "SetCurrentUser service not available");
        return;
    }
    auto request = std::make_shared<common_msgs::srv::SetCurrentUser::Request>();
    request->username = username;
    request->role = role;
    request->session_id = session_id;
    client_set_user_->async_send_request(request,
        [this](rclcpp::Client<common_msgs::srv::SetCurrentUser>::SharedFuture future) {
            try {
                auto response = future.get();
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(), "SetCurrentUser failed: %s", response->message.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "SetCurrentUser ok: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "SetCurrentUser call failed: %s", e.what());
            }
        });
}


// LHand Implementation
void RosNode::call_lhand_enable(bool enable) {
    if (!client_lhand_enable_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand Enable service not available");
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::SetEnable::Request>();
    request->joint_id = 0; // 0 = all joints
    request->enable = enable ? 1 : 0;
    
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

void RosNode::call_lhand_get_position(int joint_id, std::function<void(int)> callback) {
    if (!client_lhand_get_now_pos_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "LHand GetNowPosition service not available");
        if(callback) callback(-1); 
        return;
    }
    auto request = std::make_shared<lhandpro_interfaces::srv::GetNowPosition::Request>();
    request->joint_id = joint_id;
    
    using ServiceT = lhandpro_interfaces::srv::GetNowPosition;
    client_lhand_get_now_pos_->async_send_request(request, 
        [this, callback](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "LHand GetNowPosition result: %d", response->position);
                if(callback) callback(response->position);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "LHand GetNowPosition failed: %s", e.what());
                if(callback) callback(-1);
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
    request->p = p;
    request->q = q;
    request->v = v;
    request->a = a;
    request->r = r;
    request->tool = tool;
    request->wobj = wobj;
    request->arm_num = 0;
    request->block = false;

    using ServiceT = duco_msg::srv::RobotMove;
    client_move_->async_send_request(request, 
        [this](rclcpp::Client<ServiceT>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "Move Result: %s", response->response.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Move service call failed: %s", e.what());
            }
        });
}

std::vector<std::string> RosNode::scan_point_clouds() {
    std::vector<std::string> topics;
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    RCLCPP_INFO(this->get_logger(), "Scanning for point clouds...");
    for (const auto& [name, types] : topic_names_and_types) {
        if (name.find("points") != std::string::npos || name.find("PointCloud") != std::string::npos) {
             topics.push_back(name);
        }
    }
    return topics;
}

RosNode::CameraCapabilities RosNode::get_camera_capabilities(std::string camera_ns) {
    CameraCapabilities caps;
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    for (const auto& [name, types] : topic_names_and_types) {
        if (name.find(camera_ns) != 0) continue; // Must start with ns
        
        if (name.find("color/image_raw") != std::string::npos) caps.has_color = true;
        if (name.find("depth/image_raw") != std::string::npos) caps.has_depth = true;
        if (name.find("ir/image_raw") != std::string::npos) caps.has_ir_left = true; 
        if (name.find("left_ir/image_raw") != std::string::npos) caps.has_ir_left = true;
        if (name.find("right_ir/image_raw") != std::string::npos) caps.has_ir_right = true;
        if (name.find("points") != std::string::npos) caps.has_point_cloud = true;
    }
    return caps;
}
