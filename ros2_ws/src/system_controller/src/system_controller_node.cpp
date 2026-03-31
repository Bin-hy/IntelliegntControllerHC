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
#include <lhandpro_interfaces/srv/move_motors.hpp>
#include <lhandpro_interfaces/srv/set_enable.hpp>
#include <lhandpro_interfaces/srv/home_motors.hpp>
#include <vision_server/srv/save_image.hpp>
#include <vision_server/srv/measure_depth.hpp>
#include <vision_server/srv/capture_baseline.hpp>
#include <vision_server/srv/measure_earphone.hpp>
#include <lift_server/srv/lift_control.hpp>
#include <common_msgs/msg/collision_status.hpp>
#include <std_msgs/msg/string.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <cmath>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <future>
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <sstream>
#include <iomanip>
#include <ctime>

using namespace std::chrono_literals;

class SystemController : public rclcpp::Node {
public:
    using ExecuteTask = common_msgs::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    SystemController() : Node("system_controller_node"), is_busy_(false), is_paused_(false) {
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

        srv_set_user_ = this->create_service<common_msgs::srv::SetCurrentUser>(
            "/system/set_current_user",
            std::bind(&SystemController::handle_set_user_request, this, std::placeholders::_1, std::placeholders::_2));

        // Action Server for Task Execution
        action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this,
            "execute_task",
            std::bind(&SystemController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SystemController::handle_cancel, this, std::placeholders::_1),
            std::bind(&SystemController::handle_accepted, this, std::placeholders::_1));

        // Collision detector integration
        sub_collision_ = this->create_subscription<common_msgs::msg::CollisionStatus>(
            "/collision_detector/status", 10,
            [this](common_msgs::msg::CollisionStatus::SharedPtr msg) {
                if (msg->status == "emergency" && is_busy_ && !is_paused_) {
                    RCLCPP_ERROR(this->get_logger(), "Collision emergency: %s", msg->message.c_str());
                    is_paused_ = true;
                    pause_reason_ = "碰撞检测紧急停止: " + msg->message;
                }
            });
        pub_collision_topic_ = this->create_publisher<std_msgs::msg::String>(
            "/collision_detector/set_topic", 10);

        // Depth measure client + camera switch publisher
        pub_depth_camera_ = this->create_publisher<std_msgs::msg::String>(
            "/depth_measure/set_camera", 10);
        client_depth_measure_ = this->create_client<vision_server::srv::MeasureDepth>(
            "/depth_measure/measure");
        client_vision_baseline_ = this->create_client<vision_server::srv::CaptureBaseline>(
            "/earphone_inspector/capture_baseline");
        client_vision_measure_ = this->create_client<vision_server::srv::MeasureEarphone>(
            "/earphone_inspector/measure");

        // Lift platform client
        client_lift_ = this->create_client<lift_server::srv::LiftControl>("/lift_server/lift_control");

        RCLCPP_INFO(this->get_logger(), "SystemController Node Started.");
    }

private:
    // Internal State
    std::atomic<bool> is_busy_;
    std::atomic<bool> is_paused_;
    std::condition_variable pause_cv_;
    std::mutex pause_mutex_;
    std::string pause_reason_;
    std::string last_step_error_;   // Detailed error reason from the last failed step
    duco_msg::msg::DucoRobotState current_state_;
    std::map<std::string, common_msgs::msg::DeviceStatus> connected_devices_; // Key: Device SN or Type+ID
    std::mutex state_mutex_;
    std::mutex devices_mutex_;
    bool state_received_ = false;
    std::string current_user_ = "guest";
    std::string current_role_ = "operator";

    // Constants
    const int STATE_ENABLE = 6;

    // Action Server
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

    // Subscribers
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_state_;
    rclcpp::Subscription<common_msgs::msg::DeviceStatus>::SharedPtr sub_device_status_;
    rclcpp::Subscription<common_msgs::msg::CollisionStatus>::SharedPtr sub_collision_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_collision_topic_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_depth_camera_;

    // Clients
    rclcpp::Client<duco_msg::srv::RobotMove>::SharedPtr client_move_;
    rclcpp::Client<duco_msg::srv::RobotControl>::SharedPtr client_control_;
    rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedPtr client_io_;
    rclcpp::Client<duco_msg::srv::RobotTaskStateRquest>::SharedPtr client_task_state_;
    rclcpp::Client<vision_server::srv::MeasureDepth>::SharedPtr client_depth_measure_;
    rclcpp::Client<vision_server::srv::CaptureBaseline>::SharedPtr client_vision_baseline_;
    rclcpp::Client<vision_server::srv::MeasureEarphone>::SharedPtr client_vision_measure_;
    rclcpp::Client<lift_server::srv::LiftControl>::SharedPtr client_lift_;

    // Services
    rclcpp::Service<duco_msg::srv::RobotMove>::SharedPtr srv_move_;
    rclcpp::Service<duco_msg::srv::RobotControl>::SharedPtr srv_control_;
    rclcpp::Service<duco_msg::srv::RobotIoControl>::SharedPtr srv_io_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_pause_;
    rclcpp::Service<common_msgs::srv::SetCurrentUser>::SharedPtr srv_set_user_;

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

    void handle_set_user_request(const std::shared_ptr<common_msgs::srv::SetCurrentUser::Request> request,
                                 std::shared_ptr<common_msgs::srv::SetCurrentUser::Response> response) {
        current_user_ = request->username.empty() ? "guest" : request->username;
        current_role_ = request->role.empty() ? "operator" : request->role;
        response->success = true;
        response->message = "user_context_updated";
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
        status.device_name = "DUCO 协作机械臂";
        status.device_model = "GCR5-910";
        status.device_usage = "arm";
        status.status = (msg->robot_state == STATE_ENABLE) ? "ready" : "connected";
        status.device_sn = "duco_arm_1";
        
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

    bool is_status_ready(const std::string& status) {
        return status == "ready" || status == "running";
    }

    std::string sanitize_component(const std::string& input) {
        std::string out;
        out.reserve(input.size());
        for (char c : input) {
            if ((c >= 'a' && c <= 'z') ||
                (c >= 'A' && c <= 'Z') ||
                (c >= '0' && c <= '9') ||
                c == '_' || c == '-') {
                out.push_back(c);
            } else {
                out.push_back('_');
            }
        }
        if (out.empty()) {
            return "unnamed";
        }
        return out;
    }

    std::string current_run_time_tag() {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm_time;
#ifdef _WIN32
        localtime_s(&tm_time, &tt);
#else
        localtime_r(&tt, &tm_time);
#endif
        std::ostringstream ss;
        ss << std::put_time(&tm_time, "%Y%m%d_%H%M%S");
        return ss.str();
    }

    bool match_device(const common_msgs::msg::DeviceStatus& status, const common_msgs::msg::TaskDeviceCheck& check) {
        if (!check.device_sn.empty() && status.device_sn != check.device_sn) return false;
        if (!check.device_type.empty() && status.device_type != check.device_type) return false;
        return true;
    }

    // Resolve camera SN to a namespace string (e.g., "/CV2R1610004H")
    std::string resolve_camera_ns(const std::string& camera_sn) {
        if (camera_sn.empty()) return {};
        if (camera_sn.front() == '/') return camera_sn;
        {
            std::lock_guard<std::mutex> lock(devices_mutex_);
            auto it = connected_devices_.find(camera_sn);
            if (it != connected_devices_.end() && !it->second.topic_prefix.empty()) {
                return it->second.topic_prefix;
            }
        }
        return "/" + camera_sn;
    }

    bool check_vision_topics(const common_msgs::msg::TaskDeviceCheck& check) {
        auto topic_names_and_types = this->get_topic_names_and_types();
        // First try to resolve topic_prefix from connected_devices_ using SN
        if (!check.device_sn.empty()) {
            auto it = connected_devices_.find(check.device_sn);
            if (it != connected_devices_.end() && !it->second.topic_prefix.empty()) {
                std::string prefix = it->second.topic_prefix;
                std::string color_t = prefix + "/color/image_raw";
                return topic_names_and_types.count(color_t) > 0;
            }
        }
        // Fallback: any color/depth topic exists
        for (const auto& [name, types] : topic_names_and_types) {
            if (name.find("color/image_raw") != std::string::npos ||
                name.find("depth/image_raw") != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    bool are_devices_ready(const std::vector<common_msgs::msg::TaskDeviceCheck>& checks, std::string& reason) {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        for (const auto& check : checks) {
            bool found = false;
            bool ready = false;
            for (const auto& [key, status] : connected_devices_) {
                if (!match_device(status, check)) continue;
                found = true;
                if (is_status_ready(status.status)) {
                    ready = true;
                    break;
                }
            }
            if (!found) {
                if (check.device_type == "orbbec" || check.device_type == "camera" || check.device_type == "vision_system") {
                    if (check_vision_topics(check)) {
                        continue;
                    }
                }
                reason = "Missing device: " + check.device_type;
                return false;
            }
            if (!ready) {
                reason = "Device not ready: " + check.device_type;
                return false;
            }
        }
        reason.clear();
        return true;
    }

    bool wait_for_devices_ready(const std::shared_ptr<GoalHandleExecuteTask> goal_handle,
                                const std::shared_ptr<ExecuteTask::Feedback>& feedback,
                                const std::vector<common_msgs::msg::TaskDeviceCheck>& checks) {
        std::string reason;
        if (are_devices_ready(checks, reason)) {
            return true;
        }
        is_paused_ = true;
        pause_reason_ = reason;
        feedback->current_status = "Paused: " + reason;
        if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return false;
            }
            std::unique_lock<std::mutex> lock(pause_mutex_);
            pause_cv_.wait(lock, [this, &goal_handle]{ return !is_paused_ || goal_handle->is_canceling(); });
            if (goal_handle->is_canceling()) {
                return false;
            }
            if (are_devices_ready(checks, reason)) {
                pause_reason_.clear();
                return true;
            }
            is_paused_ = true;
            pause_reason_ = reason;
            feedback->current_status = "Paused: " + reason;
            if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
        }
        return false;
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
        if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);

        if (!wait_for_devices_ready(goal_handle, feedback, goal->task_config.device_checks)) {
            result->success = false;
            result->message = "Canceled";
            goal_handle->canceled(result);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "All devices found.");

        // Notify collision_detector and depth_measure of task camera
        if (!goal->task_config.collision_camera_sn.empty()) {
            std::string ns = resolve_camera_ns(goal->task_config.collision_camera_sn);
            if (!ns.empty()) {
                auto msg = std_msgs::msg::String();
                msg.data = ns + "/depth/points";
                pub_collision_topic_->publish(msg);
                // Also set depth_measure camera to the same
                auto msg2 = std_msgs::msg::String();
                msg2.data = ns;
                pub_depth_camera_->publish(msg2);
                RCLCPP_INFO(this->get_logger(), "Task camera set: %s", ns.c_str());
            }
        }

        int rounds = goal->task_config.exec_rounds > 0 ? goal->task_config.exec_rounds : 1;
        std::string run_time = current_run_time_tag();
        int step_index = 0;
        is_paused_ = false;
        for (int round = 0; round < rounds; ++round) {
            for (const auto& step : goal->task_config.task_seqs) {
                if (!wait_for_devices_ready(goal_handle, feedback, goal->task_config.device_checks)) {
                    result->success = false;
                    result->message = "Canceled";
                    goal_handle->canceled(result);
                    return;
                }

                if (is_paused_) {
                    feedback->current_status = pause_reason_.empty() ? "Paused" : pause_reason_;
                    goal_handle->publish_feedback(feedback);
                    std::unique_lock<std::mutex> lock(pause_mutex_);
                    pause_cv_.wait(lock, [this, &goal_handle]{ return !is_paused_ || goal_handle->is_canceling(); });
                    if (!goal_handle->is_canceling()) {
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
                feedback->current_status = "Executing Step " + std::to_string(step_index) + " Round " + std::to_string(round + 1) + "/" + std::to_string(rounds);
                if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);

                RCLCPP_INFO(this->get_logger(), "Executing Step %d: Name: %s, Type %s (Round %d/%d)", step_index, step.name.c_str(), step.type.c_str(), round + 1, rounds);

                last_step_error_.clear();
                bool step_success = true;
                if (step.type == "arm") {
                    step_success = execute_arm_step(step, goal_handle);
                } else if (step.type == "lhand" || step.type == "rhand") {
                    step_success = execute_hand_step(step, goal_handle);
                } else if (step.type == "camera") {
                    step_success = execute_camera_step(step, goal_handle, goal->task_config.task_name, run_time, round, step_index);
                } else if (step.type == "io") {
                    step_success = execute_io_step(step, goal_handle);
                } else if (step.type == "lift") {
                    step_success = execute_lift_step(step, goal_handle);
                } else if (step.type == "control") {
                    step_success = execute_control_step(step, goal_handle);
                } else if (step.type == "vision_baseline") {
                    // Capture depth baseline for earphone inspection
                    if (!client_vision_baseline_->wait_for_service(2s)) {
                        RCLCPP_ERROR(this->get_logger(), "CaptureBaseline service not available");
                        last_step_error_ = "BASELINE_SERVICE_UNAVAILABLE";
                        step_success = false;
                    } else {
                        auto req = std::make_shared<vision_server::srv::CaptureBaseline::Request>();
                        auto future = client_vision_baseline_->async_send_request(req);
                        auto t0 = std::chrono::steady_clock::now();
                        while (rclcpp::ok()) {
                            if (future.wait_for(100ms) == std::future_status::ready) break;
                            if (goal_handle->is_canceling()) { step_success = false; break; }
                            if (std::chrono::steady_clock::now() - t0 > 5s) {
                                RCLCPP_ERROR(this->get_logger(), "CaptureBaseline timeout");
                                last_step_error_ = "BASELINE_TIMEOUT";
                                step_success = false;
                                break;
                            }
                        }
                        if (step_success) {
                            try {
                                auto res = future.get();
                                step_success = res->success;
                                if (res->success) {
                                    RCLCPP_INFO(this->get_logger(), "Baseline captured: %s", res->message.c_str());
                                    feedback->current_step_index = step_index;
                                    feedback->current_status = "VISION_BASELINE_OK:" + res->message;
                                    if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "Baseline failed: %s", res->message.c_str());
                                    last_step_error_ = "BASELINE_FAILED:" + res->message;
                                }
                            } catch (const std::exception& e) {
                                RCLCPP_ERROR(this->get_logger(), "Baseline exception: %s", e.what());
                                last_step_error_ = "BASELINE_EXCEPTION:" + std::string(e.what());
                                step_success = false;
                            }
                        }
                    }
                } else if (step.type == "vision_measure") {
                    // Measure earphone angle + depth (soft-fail: don't abort task on failure)
                    if (!client_vision_measure_->wait_for_service(2s)) {
                        RCLCPP_WARN(this->get_logger(), "MeasureEarphone service not available — skipping");
                        feedback->current_step_index = step_index;
                        feedback->current_status = "VISION_RESULT:0,0,0,";
                        if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                    } else {
                        auto req = std::make_shared<vision_server::srv::MeasureEarphone::Request>();
                        req->file_tag = sanitize_component(goal->task_config.task_name) + "/" + run_time;
                        auto future = client_vision_measure_->async_send_request(req);
                        auto t0 = std::chrono::steady_clock::now();
                        bool timed_out = false;
                        while (rclcpp::ok()) {
                            if (future.wait_for(100ms) == std::future_status::ready) break;
                            if (goal_handle->is_canceling()) break;
                            if (std::chrono::steady_clock::now() - t0 > 10s) {
                                RCLCPP_WARN(this->get_logger(), "MeasureEarphone timeout — skipping");
                                timed_out = true;
                                break;
                            }
                        }
                        if (timed_out || goal_handle->is_canceling()) {
                            feedback->current_step_index = step_index;
                            feedback->current_status = "VISION_RESULT:0,0,0,";
                            if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                        } else {
                            try {
                                auto res = future.get();
                                if (res->success) {
                                    RCLCPP_INFO(this->get_logger(),
                                        "耳机测量: 角度=%.1f° 深度=%.1fmm 置信度=%.2f 路径=%s",
                                        res->angle_deg, res->depth_mm, res->confidence, res->saved_path.c_str());
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "耳机测量失败: %s", res->message.c_str());
                                }
                                // Always send result via feedback (soft-fail)
                                std::ostringstream oss;
                                oss << "VISION_RESULT:"
                                    << res->angle_deg << "," << res->depth_mm << ","
                                    << res->confidence << "," << res->saved_path;
                                feedback->current_step_index = step_index;
                                feedback->current_status = oss.str();
                                if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                            } catch (const std::exception& e) {
                                RCLCPP_ERROR(this->get_logger(), "MeasureEarphone exception: %s", e.what());
                                feedback->current_step_index = step_index;
                                feedback->current_status = "VISION_RESULT:0,0,0,";
                                if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                            }
                        }
                    }
                    // vision_measure is soft-fail — never abort the task
                    step_success = true;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unknown step type: %s", step.type.c_str());
                }

                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "Canceled";
                    goal_handle->canceled(result);
                    return;
                }

                if (!step_success) {
                    if (last_step_error_.empty()) last_step_error_ = "UNKNOWN_ERROR";
                    // Send error detail via feedback before aborting
                    feedback->current_step_index = step_index;
                    feedback->current_status = "STEP_ERROR:" + step.name + ":" + step.type + ":" + last_step_error_;
                    if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);

                    result->success = false;
                    result->message = "Step " + std::to_string(step_index) + " [" + step.name + "] (" + step.type + ") failed: " + last_step_error_;
                    goal_handle->abort(result);
                    return;
                }

                if (step.delay_ms > 0) {
                    RCLCPP_INFO(this->get_logger(), "Delaying for %d ms", step.delay_ms);
                    feedback->current_status = "Delaying " + std::to_string(step.delay_ms) + " ms";
                    if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                    
                    auto start_time = std::chrono::steady_clock::now();
                    while (rclcpp::ok()) {
                        if (goal_handle->is_canceling()) {
                            result->success = false;
                            result->message = "Canceled during delay";
                            goal_handle->canceled(result);
                            return;
                        }
                        
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                        if (elapsed >= step.delay_ms) {
                            break;
                        }
                        
                        if (is_paused_) {
                            feedback->current_status = pause_reason_.empty() ? "Paused" : pause_reason_;
                            if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                            std::unique_lock<std::mutex> lock(pause_mutex_);
                            pause_cv_.wait(lock, [this, &goal_handle]{ return !is_paused_ || goal_handle->is_canceling(); });
                            // Resume the delay
                            start_time = std::chrono::steady_clock::now() - std::chrono::milliseconds(elapsed);
                            feedback->current_status = "Delaying " + std::to_string(step.delay_ms) + " ms";
                            if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
                        }
                        
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }

                step_index++;
            }
        }

        result->success = true;
        result->message = "Task Completed Successfully";
        goal_handle->succeed(result);
    }

    bool execute_arm_step(const common_msgs::msg::TaskStep& step, std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        auto request = std::make_shared<duco_msg::srv::RobotMove::Request>();

        std::string cmd = step.arm_command.empty() ? "movej" : step.arm_command;

        if (cmd == "movel") {
            request->command = "movel";
            // arm_cart_pos: [X,Y,Z (mm), RX,RY,RZ (deg)] → Duco: [X,Y,Z (m), RX,RY,RZ (rad)]
            request->p.clear();
            for (int i = 0; i < static_cast<int>(step.arm_cart_pos.size()) && i < 6; ++i) {
                float val = static_cast<float>(step.arm_cart_pos[i]);
                if (i < 3) val = val / 1000.0f;                              // mm → m
                else       val = val * static_cast<float>(M_PI) / 180.0f;   // deg → rad
                request->p.push_back(val);
            }
            // movel requires q (q_near) for IK reference — use current joint positions
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int i = 0; i < 7; ++i) {
                    request->q.push_back(static_cast<float>(current_state_.joint_actual_position[i]));
                }
            }
            // MoveL: v in m/s [0.01, 5], a in m/s²
            double vel_mms = step.arm_velocity > 0 ? step.arm_velocity : 100.0;
            double acc_mms = step.arm_accel   > 0 ? step.arm_accel   : 500.0;
            request->v = static_cast<float>(std::max(0.01, std::min(5.0, vel_mms / 1000.0)));
            request->a = static_cast<float>(std::max(0.01, acc_mms / 1000.0));
            request->tool = "default";
            request->wobj = "default";
        } else {
            // MoveJ: try movej2 first (v in rad/s), fallback to movej (v in %) if rejected
            request->command = "movej2";
            request->q.clear();
            for (double val : step.arm_pos) {
                request->q.push_back(static_cast<float>(val * M_PI / 180.0));
            }
            // Do NOT pad to 7 DOF — send exactly 6 joints for 6-axis robot
            // deg/s → rad/s; clamp to movej2 valid range
            double vel_degs = step.arm_velocity > 0 ? step.arm_velocity : 30.0;
            double acc_degs = step.arm_accel    > 0 ? step.arm_accel    : 60.0;
            const double v_min = 0.01 * M_PI / 180.0;
            const double v_max = 1.25 * M_PI;
            request->v = static_cast<float>(std::max(v_min, std::min(v_max, vel_degs * M_PI / 180.0)));
            request->a = static_cast<float>(std::max(v_min, acc_degs * M_PI / 180.0));
        }

        request->r = 0.0;
        request->arm_num = 0;
        request->block = true;

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
            int result_code = 0;
            try { result_code = std::stoi(response->response); } catch (...) {}
            RCLCPP_INFO(this->get_logger(), "Arm move result: %d", result_code);
            // ST_Finished=4, ST_Interrupt=5 → success; ST_Illegal=7 → error
            if (result_code == 7 && current_state_.robot_state == STATE_ENABLE
                && request->command == "movej2") {
                // movej2 rejected — fallback to movej (v in % instead of rad/s)
                // Convert rad/s velocity to percentage: max movej2 speed = 1.25*PI rad/s ≈ 225°/s
                double v_pct = (request->v / (1.25 * M_PI)) * 100.0;
                v_pct = std::max(1.0, std::min(100.0, v_pct));
                double a_pct = (request->a / (12.5 * M_PI)) * 100.0;
                a_pct = std::max(1.0, std::min(100.0, a_pct));

                RCLCPP_WARN(this->get_logger(),
                    "movej2 ST_Illegal(7) — falling back to movej with v=%.1f%%, a=%.1f%%",
                    v_pct, a_pct);

                auto req2 = std::make_shared<duco_msg::srv::RobotMove::Request>();
                req2->command = "movej";
                req2->q = request->q;
                req2->v = static_cast<float>(v_pct);
                req2->a = static_cast<float>(a_pct);
                req2->r = 0.0;
                req2->arm_num = 0;
                req2->block = true;

                auto future2 = client_move_->async_send_request(req2);
                while (rclcpp::ok()) {
                    if (future2.wait_for(100ms) == std::future_status::ready) break;
                    if (goal_handle->is_canceling()) return false;
                }
                auto res2 = future2.get();
                int rc2 = 0;
                try { rc2 = std::stoi(res2->response); } catch (...) {}
                RCLCPP_INFO(this->get_logger(), "Arm move (movej fallback) result: %d", rc2);
                if (rc2 == 4 || rc2 == 5) return true;
                RCLCPP_ERROR(this->get_logger(), "movej fallback also failed: %d", rc2);
                last_step_error_ = "ARM_MOVE_REJECTED:fallback_code=" + std::to_string(rc2);
                return false;
            }
            if (result_code == 7) {
                RCLCPP_ERROR(this->get_logger(), "Arm move ST_Illegal(7): robot may not be enabled. Current robot_state=%d",
                             current_state_.robot_state);
                last_step_error_ = "ARM_NOT_ENABLED:robot_state=" + std::to_string(current_state_.robot_state);
                return false;
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Arm move exception: %s", e.what());
            last_step_error_ = "ARM_EXCEPTION:" + std::string(e.what());
            return false;
        }
    }

    bool execute_hand_step(const common_msgs::msg::TaskStep& step, std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        std::string setpos_srv = "/lhandpro_service/set_all_position"; 
        std::string movem_srv = "/lhandpro_service/move_motors";
        if (step.type == "rhand") {
            setpos_srv = "/rhandpro_service/set_all_position";
            movem_srv = "/rhandpro_service/move_motors";
        }

        auto client_setpos = this->create_client<lhandpro_interfaces::srv::SetAllPosition>(setpos_srv);
        if (!client_setpos->wait_for_service(2s)) {
             RCLCPP_ERROR(this->get_logger(), "Hand service not available: %s", setpos_srv.c_str());
             last_step_error_ = "HAND_SERVICE_UNAVAILABLE:" + setpos_srv;
             return false;
        }

        auto request = std::make_shared<lhandpro_interfaces::srv::SetAllPosition::Request>();
        if (step.hand_pos.size() >= 6) {
            for(int i=0; i<6; ++i) request->positions[i] = step.hand_pos[i];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Hand positions size mismatch (expected 6, got %zu)", step.hand_pos.size());
            last_step_error_ = "HAND_POS_MISMATCH:expected=6,got=" + std::to_string(step.hand_pos.size());
            return false;
        }

        auto future = client_setpos->async_send_request(request);
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
                 last_step_error_ = "HAND_SET_ERROR:code=" + std::to_string(response->result);
                 return false;
            }
        } catch (const std::exception& e) {
             RCLCPP_ERROR(this->get_logger(), "Hand service failed: %s", e.what());
             last_step_error_ = "HAND_EXCEPTION:" + std::string(e.what());
             return false;
        }

        auto client_move = this->create_client<lhandpro_interfaces::srv::MoveMotors>(movem_srv);
        if (!client_move->wait_for_service(2s)) {
             RCLCPP_ERROR(this->get_logger(), "Hand move service not available: %s", movem_srv.c_str());
             last_step_error_ = "HAND_MOVE_UNAVAILABLE:" + movem_srv;
             return false;
        }
        auto move_req = std::make_shared<lhandpro_interfaces::srv::MoveMotors::Request>();
        auto move_future = client_move->async_send_request(move_req);
        while (rclcpp::ok()) {
             auto status = move_future.wait_for(100ms);
             if (status == std::future_status::ready) break;
             if (goal_handle->is_canceling()) {
                 RCLCPP_WARN(this->get_logger(), "Task canceled during hand move execution");
                 return false;
             }
        }
        try {
            auto move_res = move_future.get();
            if (move_res->result != 0) {
                RCLCPP_ERROR(this->get_logger(), "Move motors returned error code: %d", move_res->result);
                last_step_error_ = "HAND_MOVE_ERROR:code=" + std::to_string(move_res->result);
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Move motors call failed: %s", e.what());
            last_step_error_ = "HAND_MOVE_EXCEPTION:" + std::string(e.what());
            return false;
        }

        return true;
    }

    bool execute_camera_step(const common_msgs::msg::TaskStep& step,
                             std::shared_ptr<GoalHandleExecuteTask> goal_handle,
                             const std::string& task_name,
                             const std::string& run_time,
                             int round_index,
                             int step_index) {
        auto client_primary = this->create_client<vision_server::srv::SaveImage>("/image_saver/save_image");
        auto client_fallback = this->create_client<vision_server::srv::SaveImage>("save_image");
        rclcpp::Client<vision_server::srv::SaveImage>::SharedPtr client;
        if (client_primary->wait_for_service(2s)) {
            client = client_primary;
        } else if (client_fallback->wait_for_service(2s)) {
            client = client_fallback;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Vision service not available (/image_saver/save_image or save_image)");
            last_step_error_ = "CAMERA_SERVICE_UNAVAILABLE";
            return false;
        }

        std::string cam_ns;
        {
            std::lock_guard<std::mutex> lock(devices_mutex_);
            if (!step.device_sn.empty()) {
                if (!step.device_sn.empty() && step.device_sn.front() == '/') {
                    cam_ns = step.device_sn;
                } else {
                    auto it = connected_devices_.find(step.device_sn);
                    if (it != connected_devices_.end() && !it->second.topic_prefix.empty()) {
                        cam_ns = it->second.topic_prefix;
                    } else {
                        cam_ns = "/" + step.device_sn;
                    }
                }
            } else {
                for (const auto& [key, dev] : connected_devices_) {
                    if (dev.device_type == "vision_system" && !dev.topic_prefix.empty()) {
                        cam_ns = dev.topic_prefix;
                        break;
                    }
                }
            }
        }
        if (cam_ns.empty()) {
            std::string suffix = "/color/image_raw";
            auto topic_names_and_types = this->get_topic_names_and_types();
            for (const auto& [name, types] : topic_names_and_types) {
                if (name.size() > suffix.size() && name.rfind(suffix) == name.size() - suffix.size()) {
                    cam_ns = name.substr(0, name.size() - suffix.size());
                    break;
                }
            }
        }
        if (cam_ns.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No camera namespace available for capture");
            last_step_error_ = "CAMERA_NS_MISSING:sn=" + step.device_sn;
            return false;
        }
        if (cam_ns.front() != '/') {
            cam_ns = "/" + cam_ns;
        }

        std::string color_topic = cam_ns + "/color/image_raw";
        std::string depth_topic = cam_ns + "/depth/image_raw";
        std::string ir_topic    = cam_ns + "/left_ir/image_raw";

        std::string task_tag = sanitize_component(task_name);
        std::string user_tag = sanitize_component(current_user_);
        std::string round_tag = "round_" + std::to_string(round_index + 1);
        std::string base_tag = task_tag + "/" + run_time + "/" + round_tag + "/" + user_tag;

        auto call_save = [&](std::string topic, std::string tag) -> std::string {
            auto request = std::make_shared<vision_server::srv::SaveImage::Request>();
            request->topic_name = topic;
            request->file_tag = base_tag + "/" + tag;

            auto future = client->async_send_request(request);
            auto start_time = std::chrono::steady_clock::now();
            while (rclcpp::ok()) {
                auto status = future.wait_for(100ms);
                if (status == std::future_status::ready) break;

                if (goal_handle->is_canceling()) {
                    RCLCPP_WARN(this->get_logger(), "Task canceled during camera capture");
                    return {};
                }

                if (std::chrono::steady_clock::now() - start_time > 5s) {
                    RCLCPP_ERROR(this->get_logger(), "Vision service timeout for %s", tag.c_str());
                    last_step_error_ = "CAMERA_TIMEOUT:" + tag;
                    return {};
                }
            }

            try {
                auto response = future.get();
                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(), "Vision service failed for %s: %s", tag.c_str(), response->message.c_str());
                    last_step_error_ = "CAMERA_SAVE_FAILED:" + tag + ":" + response->message;
                    return {};
                }
                // Send saved file path via feedback
                std::string saved_path = response->message;
                auto fb = std::make_shared<ExecuteTask::Feedback>();
                fb->current_step_index = step_index;
                fb->current_status = "SAVED_FILE:" + saved_path;
                if (!goal_handle->is_canceling()) goal_handle->publish_feedback(fb);
                return saved_path;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Vision service exception: %s", e.what());
                last_step_error_ = "CAMERA_EXCEPTION:" + std::string(e.what());
                return {};
            }
        };

        bool success = true;
        std::string step_tag = sanitize_component(step.name);
        if (step.camera_type.empty()) {
            if (call_save(color_topic, step_tag + "_Color").empty()) success = false;
            if (call_save(depth_topic, step_tag + "_Depth").empty()) success = false;
        } else {
            for (const auto& type : step.camera_type) {
                if (type == "color") {
                    if (call_save(color_topic, step_tag + "_Color").empty()) success = false;
                } else if (type == "depth") {
                    if (call_save(depth_topic, step_tag + "_Depth").empty()) success = false;
                } else if (type == "ir" || type == "ir_left") {
                     if (call_save(ir_topic, step_tag + "_IRLeft").empty()) success = false;
                } else if (type == "ir_right") {
                     std::string right_ir = cam_ns + "/right_ir/image_raw";
                     if (call_save(right_ir, step_tag + "_IRRight").empty()) success = false;
                } else if (type == "depth_measure") {
                     // Snapshot depth measurement and save
                     if (!client_depth_measure_->wait_for_service(2s)) {
                         RCLCPP_ERROR(this->get_logger(), "MeasureDepth service not available");
                         last_step_error_ = "DEPTH_SERVICE_UNAVAILABLE";
                         success = false;
                     } else {
                         auto dm_req = std::make_shared<vision_server::srv::MeasureDepth::Request>();
                         dm_req->file_tag = base_tag + "/" + step_tag + "_DepthMeasure";
                         auto dm_future = client_depth_measure_->async_send_request(dm_req);
                         auto dm_start = std::chrono::steady_clock::now();
                         while (rclcpp::ok()) {
                             auto st = dm_future.wait_for(100ms);
                             if (st == std::future_status::ready) break;
                             if (goal_handle->is_canceling()) { success = false; break; }
                             if (std::chrono::steady_clock::now() - dm_start > 5s) {
                                 RCLCPP_ERROR(this->get_logger(), "MeasureDepth timeout");
                                 last_step_error_ = "DEPTH_MEASURE_TIMEOUT";
                                 success = false;
                                 break;
                             }
                         }
                         if (success) {
                             try {
                                 auto dm_res = dm_future.get();
                                 if (dm_res->success) {
                                     RCLCPP_INFO(this->get_logger(),
                                         "DepthMeasure: %.1fmm  Pos=(%.4f,%.4f,%.4f) | %s",
                                         dm_res->depth_mm, dm_res->pos_x, dm_res->pos_y, dm_res->pos_z,
                                         dm_res->message.c_str());
                                 } else {
                                     RCLCPP_WARN(this->get_logger(), "DepthMeasure failed: %s", dm_res->message.c_str());
                                     last_step_error_ = "DEPTH_MEASURE_FAILED:" + dm_res->message;
                                     success = false;
                                 }
                             } catch (const std::exception& e) {
                                 RCLCPP_ERROR(this->get_logger(), "DepthMeasure exception: %s", e.what());
                                 last_step_error_ = "DEPTH_EXCEPTION:" + std::string(e.what());
                                 success = false;
                             }
                         }
                     }
                }
            }
        }
        
        return success;
    }

    bool execute_io_step(const common_msgs::msg::TaskStep& step, std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        if (!client_io_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "IO service not available");
            last_step_error_ = "IO_SERVICE_UNAVAILABLE";
            return false;
        }

        auto request = std::make_shared<duco_msg::srv::RobotIoControl::Request>();
        request->command = "setIo";
        request->arm_num = 0;
        request->type = step.io_type;  // 0=standard, 1=tool
        request->port = step.io_port;
        request->value = step.io_value;
        request->block = true;

        std::string type_name = (step.io_type == 1) ? "Tool" : "Standard";
        RCLCPP_INFO(this->get_logger(), "IO Step: %s type=%s port=%d value=%s",
            step.name.c_str(), type_name.c_str(), step.io_port,
            step.io_value ? "HIGH" : "LOW");

        auto future = client_io_->async_send_request(request);
        while (rclcpp::ok()) {
            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready) break;
            if (goal_handle->is_canceling()) {
                RCLCPP_WARN(this->get_logger(), "Task canceled during IO step");
                return false;
            }
        }

        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "IO step result: %s", response->response.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IO step exception: %s", e.what());
            last_step_error_ = "IO_EXCEPTION:" + std::string(e.what());
            return false;
        }
    }

    bool call_lift_service(const std::string& command, int speed_rpm,
                           std::shared_ptr<GoalHandleExecuteTask> goal_handle,
                           int target_pulses = 0, int accel_ms = 0, int decel_ms = 0) {
        if (!client_lift_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Lift service not available");
            last_step_error_ = "LIFT_SERVICE_UNAVAILABLE";
            return false;
        }
        auto request = std::make_shared<lift_server::srv::LiftControl::Request>();
        request->command = command;
        request->speed_rpm = speed_rpm;
        request->target_pulses = target_pulses;
        request->accel_ms = accel_ms;
        request->decel_ms = decel_ms;

        auto future = client_lift_->async_send_request(request);
        while (rclcpp::ok()) {
            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready) break;
            if (goal_handle->is_canceling()) {
                RCLCPP_WARN(this->get_logger(), "Task canceled during lift step");
                return false;
            }
        }
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Lift result: %s", response->message.c_str());
            return response->success;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Lift service exception: %s", e.what());
            last_step_error_ = "LIFT_EXCEPTION:" + std::string(e.what());
            return false;
        }
    }

    bool call_io_sync(int io_type, int port, bool value,
                      std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        if (!client_io_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "IO service not available for brake control");
            last_step_error_ = "LIFT_BRAKE_IO_UNAVAILABLE";
            return false;
        }
        auto request = std::make_shared<duco_msg::srv::RobotIoControl::Request>();
        request->command = "setIo";
        request->arm_num = 0;
        request->type = io_type;
        request->port = port;
        request->value = value;
        request->block = true;

        auto future = client_io_->async_send_request(request);
        while (rclcpp::ok()) {
            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready) break;
            if (goal_handle->is_canceling()) return false;
        }
        try {
            future.get();
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "IO call failed: %s", e.what());
            last_step_error_ = "LIFT_BRAKE_IO_EXCEPTION:" + std::string(e.what());
            return false;
        }
    }

    bool execute_lift_step(const common_msgs::msg::TaskStep& step,
                           std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        const std::string& cmd = step.lift_command;
        int speed = step.lift_speed_rpm > 0 ? step.lift_speed_rpm : 1000;

        RCLCPP_INFO(this->get_logger(), "Lift Step: %s cmd=%s speed=%d",
                    step.name.c_str(), cmd.c_str(), speed);

        if (cmd == "disable") {
            // Disable servo, deactivate DIO 10, then re-engage brake (DO4=LOW)
            bool ok = call_lift_service("disable", 0, goal_handle);
            call_io_sync(0, 10, false, goal_handle);  // DIO 10 LOW
            std::this_thread::sleep_for(200ms);
            call_io_sync(0, 4, false, goal_handle);  // DO4 LOW = engage brake
            return ok;
        }

        if (cmd == "enable") {
            // Release brake (DO4=HIGH), then enable (triggers SI3 homing)
            if (!call_io_sync(0, 4, true, goal_handle)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to release brake (DO4)");
                last_step_error_ = "LIFT_BRAKE_RELEASE_FAILED";
                return false;
            }
            std::this_thread::sleep_for(200ms);
            if (!call_lift_service("enable", speed, goal_handle)) {
                last_step_error_ = "LIFT_SERVO_ENABLE_FAILED";
                return false;
            }
            // Wait 1s then activate DIO 10
            std::this_thread::sleep_for(1000ms);
            call_io_sync(0, 10, true, goal_handle);
            return true;
        }

        if (cmd == "set_target") {
            return call_lift_service("set_target", 0, goal_handle,
                                     step.lift_target_pulses);
        }

        if (cmd == "trigger_step") {
            return call_lift_service("trigger_step", 0, goal_handle);
        }

        // Convenience: combined set_target + trigger_step
        if (cmd == "move_to_target") {
            if (!call_lift_service("set_target", 0, goal_handle, step.lift_target_pulses)) {
                last_step_error_ = "LIFT_SET_TARGET_FAILED";
                return false;
            }
            return call_lift_service("trigger_step", 0, goal_handle);
        }

        // Convenience: trigger_step for return to origin
        if (cmd == "return_home") {
            return call_lift_service("trigger_step", 0, goal_handle);
        }

        // Direct passthrough for get_position/status
        return call_lift_service(cmd, speed, goal_handle);
    }

    bool execute_control_step(const common_msgs::msg::TaskStep& step,
                              std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
        const std::string& target = step.control_target;
        const std::string& cmd    = step.control_command;

        RCLCPP_INFO(this->get_logger(), "Control Step: %s target=%s cmd=%s",
                    step.name.c_str(), target.c_str(), cmd.c_str());

        // ── Arm (Duco) ──
        if (target == "arm") {
            if (!client_control_->wait_for_service(2s)) {
                RCLCPP_ERROR(this->get_logger(), "RobotControl service not available");
                last_step_error_ = "CTRL_ARM_SERVICE_UNAVAILABLE";
                return false;
            }
            auto req = std::make_shared<duco_msg::srv::RobotControl::Request>();
            req->arm_num = 0;
            req->block   = true;
            if (cmd == "poweron")       req->command = "poweron";
            else if (cmd == "enable")   req->command = "enable";
            else if (cmd == "disable")  req->command = "disable";
            else if (cmd == "poweroff") req->command = "poweroff";
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown arm control command: %s", cmd.c_str());
                last_step_error_ = "CTRL_UNKNOWN_CMD:" + cmd;
                return false;
            }
            auto future = client_control_->async_send_request(req);
            while (rclcpp::ok()) {
                if (future.wait_for(100ms) == std::future_status::ready) break;
                if (goal_handle->is_canceling()) return false;
            }
            try {
                auto res = future.get();
                RCLCPP_INFO(this->get_logger(), "Arm control result: %s", res->response.c_str());
                if (cmd == "enable") std::this_thread::sleep_for(2000ms);
                return true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Arm control exception: %s", e.what());
                last_step_error_ = "CTRL_ARM_EXCEPTION:" + std::string(e.what());
                return false;
            }
        }

        // ── Hand (LHand / RHand) ──
        if (target == "lhand" || target == "rhand") {
            std::string ns = (target == "lhand") ? "/lhandpro_service" : "/rhandpro_service";

            if (cmd == "enable" || cmd == "disable") {
                auto client = this->create_client<lhandpro_interfaces::srv::SetEnable>(ns + "/set_enable");
                if (!client->wait_for_service(2s)) {
                    RCLCPP_ERROR(this->get_logger(), "Hand set_enable service not available: %s", ns.c_str());
                    last_step_error_ = "CTRL_HAND_SERVICE_UNAVAILABLE:" + ns;
                    return false;
                }
                auto req = std::make_shared<lhandpro_interfaces::srv::SetEnable::Request>();
                req->joint_id = 0;
                req->enable   = (cmd == "enable");
                auto future = client->async_send_request(req);
                while (rclcpp::ok()) {
                    if (future.wait_for(100ms) == std::future_status::ready) break;
                    if (goal_handle->is_canceling()) return false;
                }
                try {
                    auto res = future.get();
                    RCLCPP_INFO(this->get_logger(), "Hand enable result: %d", res->result);
                    if (res->result != 0) last_step_error_ = "CTRL_HAND_ENABLE_ERROR:code=" + std::to_string(res->result);
                    return (res->result == 0);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Hand enable exception: %s", e.what());
                    last_step_error_ = "CTRL_HAND_EXCEPTION:" + std::string(e.what());
                    return false;
                }
            }

            if (cmd == "home") {
                auto client = this->create_client<lhandpro_interfaces::srv::HomeMotors>(ns + "/home_motors");
                if (!client->wait_for_service(2s)) {
                    RCLCPP_ERROR(this->get_logger(), "Hand home_motors service not available: %s", ns.c_str());
                    last_step_error_ = "CTRL_HAND_HOME_UNAVAILABLE:" + ns;
                    return false;
                }
                auto req = std::make_shared<lhandpro_interfaces::srv::HomeMotors::Request>();
                req->joint_id = 0;
                auto future = client->async_send_request(req);
                const auto t0 = std::chrono::steady_clock::now();
                while (rclcpp::ok()) {
                    if (future.wait_for(100ms) == std::future_status::ready) break;
                    if (goal_handle->is_canceling()) return false;
                    if (std::chrono::steady_clock::now() - t0 > 30s) {
                        RCLCPP_ERROR(this->get_logger(), "Hand home_motors timeout");
                        last_step_error_ = "CTRL_HAND_HOME_TIMEOUT";
                        return false;
                    }
                }
                try {
                    auto res = future.get();
                    RCLCPP_INFO(this->get_logger(), "Hand home result: %d", res->result);
                    if (res->result != 0) last_step_error_ = "CTRL_HAND_HOME_ERROR:code=" + std::to_string(res->result);
                    return (res->result == 0);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Hand home exception: %s", e.what());
                    last_step_error_ = "CTRL_HAND_HOME_EXCEPTION:" + std::string(e.what());
                    return false;
                }
            }

            RCLCPP_WARN(this->get_logger(), "Unknown hand control command: %s", cmd.c_str());
            last_step_error_ = "CTRL_UNKNOWN_HAND_CMD:" + cmd;
            return false;
        }

        // ── Lift ──
        if (target == "lift") {
            if (cmd == "enable")  return call_lift_service("enable",  1000, goal_handle);
            if (cmd == "disable") return call_lift_service("disable", 0,    goal_handle);
            RCLCPP_WARN(this->get_logger(), "Unknown lift control command: %s", cmd.c_str());
            last_step_error_ = "CTRL_UNKNOWN_LIFT_CMD:" + cmd;
            return false;
        }

        RCLCPP_WARN(this->get_logger(), "Unknown control target: %s", target.c_str());
        last_step_error_ = "CTRL_UNKNOWN_TARGET:" + target;
        return false;
    }

    void handle_move_request(const std::shared_ptr<duco_msg::srv::RobotMove::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotMove::Response> response) {
        if (is_paused_) {
            response->response = "System paused";
            return;
        }

        if (!client_move_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "DucoRobot move service not available");
            response->response = "Service unavailable";
            return;
        }

        auto future = client_move_->async_send_request(request,
            [this](rclcpp::Client<duco_msg::srv::RobotMove>::SharedFuture f) {
                try {
                    auto res = f.get();
                    RCLCPP_INFO(this->get_logger(), "Forwarded move result: %s", res->response.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Forwarded move failed: %s", e.what());
                }
            });

        (void)future;
        response->response = "Forwarded";
    }
    
    void handle_control_request(const std::shared_ptr<duco_msg::srv::RobotControl::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotControl::Response> response) {
        if (!client_control_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "DucoRobot control service not available");
            response->response = "Service unavailable";
            return;
        }

        auto future = client_control_->async_send_request(request,
            [this](rclcpp::Client<duco_msg::srv::RobotControl>::SharedFuture f) {
                try {
                    auto res = f.get();
                    RCLCPP_INFO(this->get_logger(), "Forwarded control result: %s", res->response.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Forwarded control failed: %s", e.what());
                }
            });

        (void)future;
        response->response = "Forwarded";
    }

    void handle_io_request(const std::shared_ptr<duco_msg::srv::RobotIoControl::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotIoControl::Response> response) {
        if (!client_io_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "DucoRobot IO service not available");
            response->response = "Service unavailable";
            return;
        }

        // Note: single-threaded executor cannot wait synchronously for inner service calls.
        // UI now calls /duco_robot/robot_io_control directly for IO operations.
        // This handler is kept for task execution compatibility.
        auto future = client_io_->async_send_request(request,
            [this](rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedFuture f) {
                try {
                    auto res = f.get();
                    RCLCPP_INFO(this->get_logger(), "Forwarded IO result: %s", res->response.c_str());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Forwarded IO failed: %s", e.what());
                }
            });

        (void)future;
        response->response = "Forwarded";
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
