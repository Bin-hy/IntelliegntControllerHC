#include <rclcpp/rclcpp.hpp>
#include <duco_ros_driver/DucoCobot.h>
#include <duco_msg/msg/duco_robot_state.hpp>
#include <duco_msg/srv/robot_control.hpp>
#include <duco_msg/srv/robot_move.hpp>
#include <duco_msg/srv/robot_io_control.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>

using namespace DucoRPC;

class SafeDucoStatusNode : public rclcpp::Node
{
public:
    SafeDucoStatusNode() : Node("safe_duco_status")
    {
        // Parameters
        this->declare_parameter("robot_ip", "192.168.192.10");
        this->declare_parameter("robot_port", 7003);
        this->declare_parameter("arm_dof", 6);

        std::string ip;
        int port;
        this->get_parameter("robot_ip", ip);
        this->get_parameter("robot_port", port);
        this->get_parameter("arm_dof", dof_);

        RCLCPP_INFO(this->get_logger(), "Connecting to robot at %s:%d with DOF %d", ip.c_str(), port, dof_);

        // Initialize SDK wrapper
        duco_robot_ = std::make_shared<DucoRPC::DucoCobot>(ip, port);

        // Publishers
        robot_state_pub_ = this->create_publisher<duco_msg::msg::DucoRobotState>("/duco_robot/robot_state", 30);
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 300);
        
        // Callback Groups for Multi-threading
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Services
        srv_control_ = this->create_service<duco_msg::srv::RobotControl>(
            "/duco_robot/robot_control",
            std::bind(&SafeDucoStatusNode::handle_robot_control, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_cb_group_);
            
        srv_move_ = this->create_service<duco_msg::srv::RobotMove>(
            "/duco_robot/robot_move",
            std::bind(&SafeDucoStatusNode::handle_robot_move, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_cb_group_);
            
        srv_io_ = this->create_service<duco_msg::srv::RobotIoControl>(
            "/duco_robot/robot_io_control",
            std::bind(&SafeDucoStatusNode::handle_robot_io_control, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_cb_group_);

        // Timer for polling (Reduced frequency to 10Hz to prevent starvation)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&SafeDucoStatusNode::timer_callback, this),
            timer_cb_group_);
            
        // Attempt initial connection
        connect_to_robot();
    }

    ~SafeDucoStatusNode()
    {
        if (duco_robot_) {
            duco_robot_->close();
        }
    }

private:
    void connect_to_robot()
    {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        if (duco_robot_->open() == 0) {
            connected_ = true;
            RCLCPP_INFO(this->get_logger(), "Connected to robot controller successfully");
        } else {
            connected_ = false;
            RCLCPP_WARN(this->get_logger(), "Failed to connect to robot controller, will retry...");
        }
    }

    void timer_callback()
    {
        if (!connected_) {
            // Simple retry logic
            static int retry_count = 0;
            if (++retry_count > 100) { // Retry every 2 seconds roughly
                connect_to_robot();
                retry_count = 0;
            }
            return;
        }

        RobotStatusList status;
        
        auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(sdk_mutex_);
            duco_robot_->getRobotStatus(status);
        }
        auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (diff > 50) {
            RCLCPP_WARN(this->get_logger(), "getRobotStatus took %ld ms", diff);
        }

        // CRITICAL FIX: Check sizes before accessing
        if (status.jointExpectPosition.size() < (size_t)dof_ ||
            status.jointActualPosition.size() < (size_t)dof_ ||
            status.jointActualVelocity.size() < (size_t)dof_ ||
            status.jointActualCurrent.size() < (size_t)dof_) 
        {
            // Data not ready yet
            return;
        }

        if (status.robotState == 0) { // Disconnected or Error
             // Optionally handle reconnection logic
        }

        publish_robot_state(status);
        publish_joint_states(status);
    }

    // --- Service Callbacks ---

    void handle_robot_control(const std::shared_ptr<duco_msg::srv::RobotControl::Request> request,
                              std::shared_ptr<duco_msg::srv::RobotControl::Response> response)
    {
        bool is_connected = false;
        {
            std::lock_guard<std::mutex> lock(sdk_mutex_);
            is_connected = connected_;
        }
        
        if (!is_connected) { 
            response->response = "Error: Robot not connected"; 
            return; 
        }

        RCLCPP_INFO(this->get_logger(), "Executing Control Command: %s (Block: %d)", request->command.c_str(), request->block);
        
        int ret = -1;
        // Release lock for blocking operations to allow status updates
        if (request->command == "poweron") ret = duco_robot_->power_on(request->block);
        else if (request->command == "enable") ret = duco_robot_->enable(request->block);
        else if (request->command == "disable") ret = duco_robot_->disable(request->block);
        else if (request->command == "poweroff") ret = duco_robot_->power_off(request->block);
        else RCLCPP_ERROR(this->get_logger(), "Invalid control command: %s", request->command.c_str());

        RCLCPP_INFO(this->get_logger(), "Control Command %s returned: %d", request->command.c_str(), ret);
        response->response = std::to_string(ret);
    }

    void handle_robot_move(const std::shared_ptr<duco_msg::srv::RobotMove::Request> request,
                           std::shared_ptr<duco_msg::srv::RobotMove::Response> response)
    {
        bool is_connected = false;
        {
            std::lock_guard<std::mutex> lock(sdk_mutex_);
            is_connected = connected_;
        }
        
        if (!is_connected) { 
            response->response = "Error: Robot not connected"; 
            return; 
        }
        
        RCLCPP_INFO(this->get_logger(), "Executing Move Command: %s (Block: %d)", request->command.c_str(), request->block);

        int ret = -1;
        // Release lock for blocking operations
        if (request->command == "movej") {
            if (request->q.size() < (size_t)dof_) {
                RCLCPP_ERROR(this->get_logger(), "Invalid Joints Position Dof");
            } else {
                std::vector<double> joints(request->q.begin(), request->q.begin() + dof_);
                ret = duco_robot_->movej(joints, request->v, request->a, request->r, request->block);
            }
        }
        else if (request->command == "movej2") {
            if (request->q.size() < (size_t)dof_) {
                RCLCPP_ERROR(this->get_logger(), "Invalid Joints Position Dof");
            } else {
                std::vector<double> joints(request->q.begin(), request->q.begin() + dof_);
                ret = duco_robot_->movej2(joints, request->v, request->a, request->r, request->block);
            }
        }
        else if (request->command == "movel") {
            if (request->p.size() < 6 || request->q.size() < (size_t)dof_) {
                 RCLCPP_ERROR(this->get_logger(), "Invalid Data Dof for MoveL");
            } else {
                std::vector<double> pose(request->p.begin(), request->p.begin() + 6);
                std::vector<double> joints(request->q.begin(), request->q.begin() + dof_);
                ret = duco_robot_->movel(pose, request->v, request->a, request->r, joints, request->tool, request->wobj, request->block);
            }
        }
        else {
             RCLCPP_ERROR(this->get_logger(), "Unsupported move command: %s", request->command.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Move Command %s returned: %d", request->command.c_str(), ret);
        response->response = std::to_string(ret);
    }

    void handle_robot_io_control(const std::shared_ptr<duco_msg::srv::RobotIoControl::Request> request,
                                 std::shared_ptr<duco_msg::srv::RobotIoControl::Response> response)
    {
        bool is_connected = false;
        {
            std::lock_guard<std::mutex> lock(sdk_mutex_);
            is_connected = connected_;
        }

        if (!is_connected) { 
            response->response = "Error: Robot not connected"; 
            return; 
        }

        int ret = -1;
        if (request->command == "setIo" || request->command == "SetIo") {
            if (request->type == 0) ret = duco_robot_->set_standard_digital_out(request->port, request->value, request->block);
            else if (request->type == 1) ret = duco_robot_->set_tool_digital_out(request->port, request->value, request->block);
        }
        else if (request->command == "getIo" || request->command == "GetIo") {
             if (request->type == 0) ret = duco_robot_->get_standard_digital_in(request->port);
             else if (request->type == 1) ret = duco_robot_->get_tool_digital_in(request->port);
        }
        
        response->response = std::to_string(ret);
    }

    void publish_robot_state(const RobotStatusList& status)
    {
        auto msg = duco_msg::msg::DucoRobotState();
        
        // Safely copy data
        for(int i=0; i<dof_ && i<7; ++i) { // Fixed size 7 arrays in msg
            if (i < (int)status.jointExpectPosition.size()) msg.joint_expect_position[i] = status.jointExpectPosition[i];
            if (i < (int)status.jointExpectVelocity.size()) msg.joint_expect_velocity[i] = status.jointExpectVelocity[i];
            if (i < (int)status.jointExpectAccelera.size()) msg.joint_expect_accelera[i] = status.jointExpectAccelera[i];
            if (i < (int)status.jointActualPosition.size()) msg.joint_actual_position[i] = status.jointActualPosition[i];
            if (i < (int)status.jointActualVelocity.size()) msg.joint_actual_velocity[i] = status.jointActualVelocity[i];
            if (i < (int)status.jointActualAccelera.size()) msg.joint_actual_accelera[i] = status.jointActualAccelera[i];
            if (i < (int)status.jointActualCurrent.size())  msg.joint_actual_current[i]  = status.jointActualCurrent[i];
            
            // Map other fields as needed...
        }

        // Cartesian actual position
        for(int i=0; i<6 && i<(int)status.cartActualPosition.size(); ++i) {
             msg.cart_actual_position[i] = status.cartActualPosition[i];
        }
        
        msg.collision = status.collision;
        msg.collision_axis = status.collisionAxis;
        msg.robot_state = status.robotState;
        msg.robot_error = status.robotError;
        
        robot_state_pub_->publish(msg);
    }

    void publish_joint_states(const RobotStatusList& status)
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        
        for(int i=0; i<dof_; ++i) {
            msg.name.push_back("joint_" + std::to_string(i+1));
            if (i < (int)status.jointActualPosition.size()) msg.position.push_back(status.jointActualPosition[i]);
            if (i < (int)status.jointActualVelocity.size()) msg.velocity.push_back(status.jointActualVelocity[i]);
            if (i < (int)status.jointActualCurrent.size())  msg.effort.push_back(status.jointActualCurrent[i]); // Using current as effort approximation
        }
        
        joint_states_pub_->publish(msg);
    }

    std::shared_ptr<DucoRPC::DucoCobot> duco_robot_;
    rclcpp::Publisher<duco_msg::msg::DucoRobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    rclcpp::Service<duco_msg::srv::RobotControl>::SharedPtr srv_control_;
    rclcpp::Service<duco_msg::srv::RobotMove>::SharedPtr srv_move_;
    rclcpp::Service<duco_msg::srv::RobotIoControl>::SharedPtr srv_io_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    bool connected_ = false;
    int dof_ = 6;
    std::mutex sdk_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafeDucoStatusNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
