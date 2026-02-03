#include <rclcpp/rclcpp.hpp>
#include <duco_msg/srv/robot_move.hpp>
#include <duco_msg/srv/robot_control.hpp>
#include <duco_msg/srv/robot_io_control.hpp>
#include <duco_msg/msg/duco_robot_state.hpp>
#include <mutex>
#include <atomic>
#include <future>
#include <chrono>

using namespace std::chrono_literals;

class SystemController : public rclcpp::Node {
public:
    SystemController() : Node("system_controller_node"), is_busy_(false) {
        // Subscribers
        sub_state_ = this->create_subscription<duco_msg::msg::DucoRobotState>(
            "/duco_robot/robot_state", 10,
            std::bind(&SystemController::robot_state_callback, this, std::placeholders::_1));

        // Clients to Duco Driver
        client_move_ = this->create_client<duco_msg::srv::RobotMove>("/duco_robot/robot_move");
        client_control_ = this->create_client<duco_msg::srv::RobotControl>("/duco_robot/robot_control");
        client_io_ = this->create_client<duco_msg::srv::RobotIoControl>("/duco_robot/robot_io_control");

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

        RCLCPP_INFO(this->get_logger(), "SystemController Node Started. Waiting for robot state...");
    }

private:
    // Internal State
    std::atomic<bool> is_busy_;
    duco_msg::msg::DucoRobotState current_state_;
    std::mutex state_mutex_;
    bool state_received_ = false;

    // Constants
    const int STATE_ENABLE = 6;

    // Callbacks
    void robot_state_callback(const duco_msg::msg::DucoRobotState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_ = *msg;
        state_received_ = true;
    }

    bool check_safety(std::string& error_msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        if (!state_received_) {
            error_msg = "System not ready: No robot state received yet.";
            return false;
        }

        if (current_state_.collision) {
            error_msg = "Safety Stop: Robot Collision Detected!";
            return false;
        }

        if (current_state_.robot_error != 0) {
            error_msg = "Safety Stop: Robot Error Code " + std::to_string(current_state_.robot_error);
            return false;
        }

        // For motion, we usually require Enable state (6). 
        // But for PowerOn/Enable commands, we might be in other states.
        // So this check might need to be specific to the command type.
        // I will split this into check_motion_safety and check_general_safety.
        return true;
    }

    bool check_motion_safety(std::string& error_msg) {
        if (!check_safety(error_msg)) return false;
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_state_.robot_state != STATE_ENABLE) {
            error_msg = "Motion Rejected: Robot not in ENABLE state (Current: " + std::to_string(current_state_.robot_state) + ")";
            return false;
        }
        return true;
    }

    void handle_move_request(const std::shared_ptr<duco_msg::srv::RobotMove::Request> request,
                             std::shared_ptr<duco_msg::srv::RobotMove::Response> response) {
        
        std::string error_msg;
        
        // 1. Busy Check
        bool expected = false;
        if (!is_busy_.compare_exchange_strong(expected, true)) {
            response->response = "REJECTED: System is BUSY";
            RCLCPP_WARN(this->get_logger(), "Move request rejected: System BUSY");
            return;
        }

        // 2. Safety Check
        if (!check_motion_safety(error_msg)) {
            is_busy_ = false;
            response->response = "REJECTED: " + error_msg;
            RCLCPP_ERROR(this->get_logger(), "Move request rejected: %s", error_msg.c_str());
            return;
        }

        // 3. Enforce Blocking
        request->block = true;

        // 4. Forward Request
        RCLCPP_INFO(this->get_logger(), "Executing Move Command: %s", request->command.c_str());
        
        if (!client_move_->wait_for_service(1s)) {
            is_busy_ = false;
            response->response = "ERROR: RobotMove service not available";
            return;
        }

        auto future = client_move_->async_send_request(request);
        
        // Wait for result with timeout (Move can be long, e.g. 120s)
        if (future.wait_for(120s) == std::future_status::timeout) {
            is_busy_ = false; // Reset busy state on timeout to allow recovery
            response->response = "ERROR: RobotMove Service Timeout (120s)";
            RCLCPP_ERROR(this->get_logger(), "Move Timeout: Service did not respond in 120s");
            // Ideally we should trigger a STOP here, but no stop interface is standard.
            return;
        }
        
        try {
            auto result = future.get();
            response->response = result->response;
            RCLCPP_INFO(this->get_logger(), "Move Complete: %s", result->response.c_str());
        } catch (const std::exception &e) {
            response->response = std::string("ERROR: Exception during service call: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Move Exception: %s", e.what());
        }

        is_busy_ = false;
    }

    void handle_control_request(const std::shared_ptr<duco_msg::srv::RobotControl::Request> request,
                                std::shared_ptr<duco_msg::srv::RobotControl::Response> response) {
        // Similar logic, but maybe less strict on "ENABLE" state for power on commands.
        bool expected = false;
        if (!is_busy_.compare_exchange_strong(expected, true)) {
            response->response = "REJECTED: System is BUSY";
            return;
        }

        std::string error_msg;
        if (!check_safety(error_msg)) {
             // Allow recovery from error via poweroff or disable?
             // User said: "robot_error != 0 -> 禁止一切动作" (Prohibit all actions).
             // But usually we need to clear error. 
             // If we stick to strict instruction, we can't even power off?
             // I will assume "Action" means "Motion". "Control" (Power) might be allowed?
             // Let's allow "poweroff" and "disable" even in error to be safe.
             if (current_state_.robot_error != 0 && request->command != "poweroff" && request->command != "disable") {
                  is_busy_ = false;
                  response->response = "REJECTED: " + error_msg; 
                  return;
             }
             // If Collision, maybe same?
             if (current_state_.collision && request->command != "poweroff" && request->command != "disable") {
                  is_busy_ = false;
                  response->response = "REJECTED: " + error_msg;
                  return;
             }
        }
        
        request->block = true;
        
        auto future = client_control_->async_send_request(request);
        
        // Control commands should be fast (5s)
        if (future.wait_for(5s) == std::future_status::timeout) {
            is_busy_ = false;
            response->response = "ERROR: RobotControl Service Timeout";
            RCLCPP_ERROR(this->get_logger(), "Control Timeout");
            return;
        }

        try {
            auto result = future.get();
            response->response = result->response;
            RCLCPP_INFO(this->get_logger(), "Control '%s' Result: %s", request->command.c_str(), result->response.c_str());
        } catch (const std::exception &e) {
            response->response = "ERROR: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Control Exception: %s", e.what());
        }
        
        is_busy_ = false;
    }

    void handle_io_request(const std::shared_ptr<duco_msg::srv::RobotIoControl::Request> request,
                           std::shared_ptr<duco_msg::srv::RobotIoControl::Response> response) {
        // IO might not be blocking, but we treat it as such for consistency
        bool expected = false;
        if (!is_busy_.compare_exchange_strong(expected, true)) {
            response->response = "REJECTED: System is BUSY";
            return;
        }
        
        // IO requires safety check? 
        // User said "robot_state 非 READY → 禁止运动" (Motion). 
        // "collision == true → 禁止一切动作" (All actions).
        // "robot_error != 0 → 禁止一切动作" (All actions).
        std::string error_msg;
        if (!check_safety(error_msg)) {
             is_busy_ = false;
             response->response = "REJECTED: " + error_msg;
             return;
        }

        request->block = true;
        
        auto future = client_io_->async_send_request(request);
        
        if (future.wait_for(5s) == std::future_status::timeout) {
            is_busy_ = false;
            response->response = "ERROR: RobotIoControl Service Timeout";
            RCLCPP_ERROR(this->get_logger(), "IO Timeout");
            return;
        }

        try {
            auto result = future.get();
            response->response = result->response;
            RCLCPP_INFO(this->get_logger(), "IO Result: %s", result->response.c_str());
        } catch (const std::exception &e) {
             response->response = "ERROR: " + std::string(e.what());
             RCLCPP_ERROR(this->get_logger(), "IO Exception: %s", e.what());
        }
        
        is_busy_ = false;
    }

    // Member variables
    rclcpp::Subscription<duco_msg::msg::DucoRobotState>::SharedPtr sub_state_;
    
    rclcpp::Client<duco_msg::srv::RobotMove>::SharedPtr client_move_;
    rclcpp::Client<duco_msg::srv::RobotControl>::SharedPtr client_control_;
    rclcpp::Client<duco_msg::srv::RobotIoControl>::SharedPtr client_io_;
    
    rclcpp::Service<duco_msg::srv::RobotMove>::SharedPtr srv_move_;
    rclcpp::Service<duco_msg::srv::RobotControl>::SharedPtr srv_control_;
    rclcpp::Service<duco_msg::srv::RobotIoControl>::SharedPtr srv_io_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemController>();
    // Use MultiThreadedExecutor to prevent deadlocks when waiting for services in callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
