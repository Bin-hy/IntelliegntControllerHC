#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <duco_msg/srv/robot_move.hpp>
#include <duco_msg/srv/robot_control.hpp>
#include <duco_msg/srv/robot_io_control.hpp>
#include <duco_msg/msg/duco_robot_state.hpp>
#include <duco_msg/srv/robot_task_state_rquest.hpp>
#include <common_msgs/action/execute_task.hpp>
#include <common_msgs/msg/device_status.hpp>
#include <common_msgs/srv/set_current_user.hpp>
#include <lhandpro_interfaces/srv/set_all_position.hpp>
#include <lhandpro_interfaces/srv/set_position.hpp>
#include <vision_server/srv/save_image.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <future>
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <sstream>

using namespace std::chrono_literals;

class SystemController : public rclcpp::Node {
public:
    using ExecuteTask = common_msgs::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using SetCurrentUser = common_msgs::srv::SetCurrentUser;

    SystemController() : Node("system_controller_node"), is_busy_(false) {
        // Subscribers
        sub_state_ = this->create_subscription<duco_msg::msg::DucoRobotState>(
            "/duco_cobot/robot_state", 10,
            std::bind(&SystemController::robot_state_callback, this, std::placeholders::_1));

        sub_device_status_ = this->create_subscription<common_msgs::msg::DeviceStatus>(
            "/system/device_status", 10,
            std::bind(&SystemController::device_status_callback, this, std::placeholders::_1));

        // Clients to Duco Driver
        client_move_ = this->create_client<duco_msg::srv::RobotMove>("/duco_robot/robot_move");
        client_control_ = this->create_client<duco_msg::srv::RobotControl>("/duco_robot/robot_control");
        client_io_ = this->create_client<duco_msg::srv::RobotIoControl>("/duco_robot/robot_io_control");
        client_task_state_ = this->create_client<duco_msg::srv::RobotTaskStateRquest>("/duco_robot/robot_task_state_request");

        // Clients for LHand (Assuming standard names, might need config)
        // These clients will be created on demand or stored in a map if multiple hands exist
        
        // Services for UI
        srv_move_ = this->create_service<duco_msg::srv::RobotMove>(
            "/ui/request_move",
            std::bind(&SystemController::handle_move_request, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_control_ = this->create_service<duco_msg::srv::RobotControl>(
            "/ui/request_control",
            std::bind(&SystemController::handle_control_request, this, std::placeholders::_1, std::placeholders::_2));

        srv_io_ = this->create_service<duco_msg::srv::RobotIoControl>(
            "/ui/request_io",
            std::bind(&SystemController::handle_io_request, this, std::placeholders::_1, std::placeholders::_2));

        srv_pause_ = this->create_service<std_srvs::srv::SetBool>(
            "/system/pause_task",
            std::bind(&SystemController::handle_pause_request, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_user_ = this->create_service<SetCurrentUser>(
            "/system/set_current_user",
            std::bind(&SystemController::handle_set_current_user, this, std::placeholders::_1, std::placeholders::_2));

        // Action Server for Task Execution
        action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this,
            "execute_task",
            std::bind(&SystemController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SystemController::handle_cancel, this, std::placeholders::_1),
            std::bind(&SystemController::handle_accepted, this, std::placeholders::_1));

        // Timer to publish this controller's status or aggregate status?
        // Maybe not needed for now.

        RCLCPP_INFO(this->get_logger(), "SystemController Node Started.");
    }

private:
    // Internal State
    std::atomic<bool> is_busy_;
    std::atomic<bool> is_paused_;
    std::condition_variable pause_cv_;
    std::mutex pause_mutex_;
    duco_msg::msg::DucoRobotState current_state_;
    std::map<std::string, common_msgs::msg::DeviceStatus> connected_devices_; // Key: Device SN or Type+ID
    std::mutex state_mutex_;
    std::mutex devices_mutex_;
    bool state_received_ = false;

    // Constants
    const int STATE_ENABLE = 6;

    // Action Server
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

    // Subscribers
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_state_;
    rclcpp::Subscription<common_msgs::msg::DeviceStatus>::SharedPtr sub_device_status_;

    // Clients
    rclcpp::Client<duco_msg::srv::RobotMove>::SharedPtr client_move_;
    rclcpp::Client<duco_msg::srv::RobotControl>::SharedPtr client_control_;
    rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedPtr client_io_;
    rclcpp::Client<duco_msg::srv::RobotTaskStateRquest>::SharedPtr client_task_state_;

    // Services
    rclcpp::Service<duco_msg::srv::RobotMove>::SharedPtr srv_move_;
    rclcpp::Service<duco_msg::srv::RobotControl>::SharedPtr srv_control_;
    rclcpp::Service<duco_msg::srv::RobotIoControl>::SharedPtr srv_io_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pause_;
    rclcpp::Service<SetCurrentUser>::SharedPtr srv_set_user_;

    struct CurrentUser {
        std::string username;
        std::string role;
        std::string session_id;
    };

    CurrentUser current_user_;
    std::mutex user_mutex_;

    void handle_pause_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        bool pause = request->data;
        if (pause) {
            if (!is_paused_) {
                is_paused_ = true;
                RCLCPP_INFO(this->get_logger(), "Task Paused Request");
                response->success = true;
                response->message = "Task Paused";
            } else {
                response->success = false;
                response->message = "Already Paused";
            }
        } else {
            if (is_paused_) {
                is_paused_ = false;
                pause_cv_.notify_all();
                RCLCPP_INFO(this->get_logger(), "Task Resumed Request");
                response->success = true;
                response->message = "Task Resumed";
            } else {
                response->success = false;
                response->message = "Not Paused";
            }
        }
    }

    void handle_set_current_user(const std::shared_ptr<SetCurrentUser::Request> request,
                                 std::shared_ptr<SetCurrentUser::Response> response) {
        std::lock_guard<std::mutex> lock(user_mutex_);
        if (request->username.empty() || request->role.empty() || request->session_id.empty()) {
            response->success = false;
            response->message = "invalid user information";
            return;
        }
        current_user_.username = request->username;
        current_user_.role = request->role;
        current_user_.session_id = request->session_id;
        response->success = true;
        response->message = "user context updated";
        RCLCPP_INFO(this->get_logger(), "Current user set to %s (%s)", current_user_.username.c_str(), current_user_.role.c_str());
    }

    bool has_permission(const std::string& role, const std::string& action) {
        if (role == "admin") {
            return true;
        }
        if (role == "maintainer") {
            if (action == "power_on" || action == "power_off" ||
                action == "start_task" || action == "stop_task" ||
                action == "modify_param" || action == "calibrate_system") {
                return true;
            }
            return false;
        }
        if (role == "operator") {
            if (action == "power_on" || action == "power_off" ||
                action == "start_task" || action == "stop_task") {
                return true;
            }
            return false;
        }
        return false;
    }

    bool check_user_permission(const std::string& action, std::string& message) {
        std::lock_guard<std::mutex> lock(user_mutex_);
        if (current_user_.username.empty()) {
            message = "no authenticated user";
            return false;
        }
        std::string role_lower = current_user_.role;
        for (char& c : role_lower) {
            c = static_cast<char>(::tolower(c));
        }
        if (!has_permission(role_lower, action)) {
            message = "permission denied";
            return false;
        }
        return true;
    }

    // Callbacks
    void robot_state_callback(const duco_msg::msg::DucoRobotState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = *msg;
        state_received_ = true;
        
        // Also update connected_devices_ for Duco
        // Assuming we only have one Duco arm for now, or we need to extract SN from somewhere.
        // DucoRobotState doesn't seem to have SN. We might need another way or just assume type "duco" is present.
        common_msgs::msg::DeviceStatus status;
        status.device_type = "duco";
        status.device_model = "unknown"; // Extract if possible
        status.status = "connected"; // msg->robot_state == STATE_ENABLE ? "ready" : "connected";
        status.device_sn = "duco_arm_1"; // Placeholder
        
        std::lock_guard<std::mutex> dev_lock(devices_mutex_);
        connected_devices_["duco"] = status;
    }

    void device_status_callback(const common_msgs::msg::DeviceStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        // Use SN as key if available, otherwise type
        std::string key = msg->device_sn.empty() ? msg->device_type : msg->device_sn;
        connected_devices_[key] = *msg;
        RCLCPP_DEBUG(this->get_logger(), "Device updated: %s (%s)", key.c_str(), msg->status.c_str());
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteTask::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request for task: %s", goal->task_config.task_name.c_str());
        (void)uuid;
        // Check if busy?
        // if (is_busy_) return rclcpp_action::GoalResponse::REJECT; 
        // We can queue tasks or reject. Let's reject for simplicity.
        // However, is_busy_ is for atomic check. 
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&SystemController::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing task...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        auto result = std::make_shared<ExecuteTask::Result>();

        // 1. Device Check
        RCLCPP_INFO(this->get_logger(), "Checking devices...");
        feedback->current_status = "Checking Devices";
        goal_handle->publish_feedback(feedback);

        {
            std::lock_guard<std::mutex> lock(devices_mutex_);
            for (const auto& check : goal->task_config.device_checks) {
                bool found = false;
                // Look for device by SN or Type
                for (const auto& [key, status] : connected_devices_) {
                    if (!check.device_sn.empty()) {
                         if (status.device_sn == check.device_sn) {
                             found = true;
                             break;
                         }
                    } else {
                        // If SN is empty in check, match by type
                        if (status.device_type == check.device_type) {
                            found = true;
                            break;
                        }
                    }
                }
                
                if (!found) {
                    RCLCPP_ERROR(this->get_logger(), "Missing device: %s (%s)", check.device_type.c_str(), check.device_sn.c_str());
                    result->success = false;
                    result->message = "Missing device: " + check.device_type;
                    goal_handle->abort(result);
                    return;
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "All devices found.");

        // 2. Execute Steps
        int step_index = 0;
        is_paused_ = false; // Reset pause state
        for (const auto& step : goal->task_config.task_seqs) {
            // Check for pause
            if (is_paused_) {
                feedback->current_status = "Paused";
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Task Paused...");
                std::unique_lock<std::mutex> lock(pause_mutex_);
                pause_cv_.wait(lock, [this, &goal_handle]{ return !is_paused_ || goal_handle->is_canceling(); });
                if (goal_handle->is_canceling()) {
                     // Handled below
                } else {
                     RCLCPP_INFO(this->get_logger(), "Task Resumed.");
                }
            }

            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Canceled";
                goal_handle->canceled(result);
                return;
            }

            feedback->current_step_index = step_index;
            feedback->current_status = "Executing Step " + std::to_string(step_index);
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Executing Step %d: Name: %s, Type %s", step_index, step.name.c_str(), step.type.c_str());

            bool step_success = true;
            if (step.type == "arm") {
                step_success = execute_arm_step(step, goal_handle);
            } else if (step.type == "lhand" || step.type == "rhand") {
                step_success = execute_hand_step(step, goal_handle);
            } else if (step.type == "camera") {
                step_success = execute_camera_step(step, goal_handle);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown step type: %s", step.type.c_str());
            }
            
            // Check cancel again after step execution (if long running)
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Canceled";
                goal_handle->canceled(result);
                return;
            }

            if (!step_success) {
                result->success = false;
                result->message = "Step " + std::to_string(step_index) + " failed";
                goal_handle->abort(result);
                return;
            }

            step_index++;
        }

        if (rclcpp::ok()) {
            result->success = true;
            result->message = "Task Completed Successfully";
            goal_handle->succeed(result);
        }
    }

    bool execute_arm_step(const common_msgs::msg::TaskStep& step, std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        // Convert step.arm_pos to Duco command
        auto request = std::make_shared<duco_msg::srv::RobotMove::Request>();
        
        request->command = "movej";
        
        // Convert double (TaskStep) to float (RobotMove)
        for (double val : step.arm_pos) {
            request->q.push_back(static_cast<float>(val));
        }
        
        // Default parameters (can be tuned or added to TaskStep)
        request->v = 20.0; // % velocity? or rad/s? Duco usually uses % or specific units. RosNode used 20.0
        request->a = 100.0; // % accel?
        request->r = 0.0; // blend radius
        request->arm_num = 1;
        request->block = true; // Wait for completion

        auto future = client_move_->async_send_request(request);
        
        while (rclcpp::ok()) {
             auto status = future.wait_for(100ms);
             if (status == std::future_status::ready) {
                 break;
             }
             
             if (goal_handle->is_canceling()) {
                 RCLCPP_WARN(this->get_logger(), "Task canceled during arm move");
                 return false;
             }
        }

        try {
            auto response = future.get();
            // Check response->response string? 
            // "OK" or similar?
            RCLCPP_INFO(this->get_logger(), "Arm move result: %s", response->response.c_str());
            return true; 
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Arm move exception: %s", e.what());
            return false;
        }
    }

    bool execute_hand_step(const common_msgs::msg::TaskStep& step, std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        std::string service_name = "/lhandpro_service/set_all_position"; 
        if (step.type == "rhand") service_name = "/rhandpro_service/set_all_position"; 

        auto client = this->create_client<lhandpro_interfaces::srv::SetAllPosition>(service_name);
        if (!client->wait_for_service(2s)) {
             RCLCPP_ERROR(this->get_logger(), "Hand service not available: %s", service_name.c_str());
             return false;
        }

        auto request = std::make_shared<lhandpro_interfaces::srv::SetAllPosition::Request>();
        if (step.hand_pos.size() >= 6) {
            for(int i=0; i<6; ++i) request->positions[i] = step.hand_pos[i];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Hand positions size mismatch (expected 6, got %zu)", step.hand_pos.size());
            return false;
        }

        auto future = client->async_send_request(request);
        while (rclcpp::ok()) {
             auto status = future.wait_for(100ms);
             if (status == std::future_status::ready) break;
             if (goal_handle->is_canceling()) {
                 RCLCPP_WARN(this->get_logger(), "Task canceled during hand move");
                 return false;
             }
        }
        
        try {
            auto response = future.get();
            if (response->result != 0) {
                 RCLCPP_ERROR(this->get_logger(), "Hand service returned error code: %d", response->result);
                 return false;
            }
        } catch (const std::exception& e) {
             RCLCPP_ERROR(this->get_logger(), "Hand service failed: %s", e.what());
             return false;
        }

        return true;
    }

    bool execute_camera_step(const common_msgs::msg::TaskStep& step, std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        auto client = this->create_client<vision_server::srv::SaveImage>("/vision_server/save_image");
        if (!client->wait_for_service(2s)) {
             RCLCPP_ERROR(this->get_logger(), "Vision service not available");
             return false;
        }

        // Auto-detect camera topics
        auto topic_names_and_types = this->get_topic_names_and_types();
        std::string color_topic, depth_topic, ir_topic;

        // Prefer topics starting with /camera or /ob_camera, but accept any matching suffix
        for (const auto& [name, types] : topic_names_and_types) {
            if (name.find("color/image_raw") != std::string::npos) {
                if (color_topic.empty() || name.find("/camera/") != std::string::npos) color_topic = name;
            }
            else if (name.find("depth/image_raw") != std::string::npos) {
                if (depth_topic.empty() || name.find("/camera/") != std::string::npos) depth_topic = name;
            }
            else if (name.find("ir/image_raw") != std::string::npos || name.find("left_ir/image_raw") != std::string::npos) {
                 if (ir_topic.empty() || name.find("/camera/") != std::string::npos) ir_topic = name;
            }
        }

        if (color_topic.empty()) color_topic = "/camera/color/image_raw"; // Fallback
        if (depth_topic.empty()) depth_topic = "/camera/depth/image_raw"; // Fallback
        if (ir_topic.empty()) ir_topic = "/camera/ir/image_raw"; // Fallback

        auto call_save = [&](std::string topic, std::string tag) {
            auto request = std::make_shared<vision_server::srv::SaveImage::Request>();
            request->topic_name = topic;
            request->file_tag = tag;
            
            auto future = client->async_send_request(request);
            auto start_time = std::chrono::steady_clock::now();
            while (rclcpp::ok()) {
                auto status = future.wait_for(100ms);
                if (status == std::future_status::ready) break;
                
                if (goal_handle->is_canceling()) {
                    RCLCPP_WARN(this->get_logger(), "Task canceled during camera capture");
                    return false;
                }
                
                if (std::chrono::steady_clock::now() - start_time > 5s) {
                    RCLCPP_ERROR(this->get_logger(), "Vision service timeout for %s", tag.c_str());
                    return false;
                }
            }

            try {
                auto response = future.get();
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(), "Vision service failed for %s: %s", tag.c_str(), response->message.c_str());
                    return false;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Vision service exception: %s", e.what());
                return false;
            }
            return true;
        };

        bool success = true;
        if (step.camera_type.empty()) {
            // Default behavior: Capture Color and Depth
            if (!call_save(color_topic, "Color")) success = false;
            if (!call_save(depth_topic, "Depth")) success = false;
        } else {
            for (const auto& type : step.camera_type) {
                if (type == "color") {
                    if (!call_save(color_topic, "Color")) success = false;
                } else if (type == "depth") {
                    if (!call_save(depth_topic, "Depth")) success = false;
                } else if (type == "ir") {
                     if (!call_save(ir_topic, "IR")) success = false;
                }
            }
        }
        
        return success;
    }

    // Keep existing handlers...
    void handle_move_request(const std::shared_ptr<duco_msg::srv::RobotMove::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotMove::Response> response) {
        (void)request;
        std::string msg;
        if (!check_user_permission("start_task", msg)) {
            response->response = "DENY:" + msg;
            RCLCPP_WARN(this->get_logger(), "Move request denied: %s", msg.c_str());
            return;
        }
        response->response = "OK";
    }
    
    void handle_control_request(const std::shared_ptr<duco_msg::srv::RobotControl::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotControl::Response> response) {
        (void)request;
        std::string msg;
        if (!check_user_permission("power_on", msg)) {
            response->response = "DENY:" + msg;
            RCLCPP_WARN(this->get_logger(), "Control request denied: %s", msg.c_str());
            return;
        }
        response->response = "OK";
    }

    void handle_io_request(const std::shared_ptr<duco_msg::srv::RobotIoControl::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotIoControl::Response> response) {
        (void)request;
        std::string msg;
        if (!check_user_permission("modify_param", msg)) {
            response->response = "DENY:" + msg;
            RCLCPP_WARN(this->get_logger(), "IO request denied: %s", msg.c_str());
            return;
        }
        response->response = "OK";
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
