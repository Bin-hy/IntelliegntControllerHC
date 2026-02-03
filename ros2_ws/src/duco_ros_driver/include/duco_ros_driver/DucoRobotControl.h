#ifndef DUCOROBOTCONTROL_H
#define DUCOROBOTCONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "duco_msg/srv/robot_control.hpp"
#include "duco_msg/srv/robot_io_control.hpp"
#include "duco_msg/srv/robot_move.hpp"
#include "duco_msg/srv/robot_task_state_rquest.hpp"
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include "DucoRobotStatus.h"

namespace DucoRPC{
class DucoCobot;
};

namespace duco_driver
{

class DucoRobotControl: public rclcpp::Node
{
public:
    DucoRobotControl(const rclcpp::NodeOptions & option = rclcpp::NodeOptions());

    ~DucoRobotControl();

    void run();

protected:
    /*连接机器人*/
    bool connectToRobotController();
    /**
     * @brief handle_robot_control
     * @param request
     * @param response
     */
    void handle_robot_control(const std::shared_ptr<duco_msg::srv::RobotControl::Request> request, std::shared_ptr<duco_msg::srv::RobotControl::Response> response);
    /**
     * @brief handle_robot_move
     * @param request
     * @param response
     */
    void handle_robot_move(const std::shared_ptr<duco_msg::srv::RobotMove::Request> request, std::shared_ptr<duco_msg::srv::RobotMove::Response> response);
    /**
     * @brief handle_robot_io_control
     * @param request
     * @param response
     */
    void handle_robot_io_control(const std::shared_ptr<duco_msg::srv::RobotIoControl::Request> request, std::shared_ptr<duco_msg::srv::RobotIoControl::Response> response);
    /**
     * @brief handle_robot_task_state_request
     * @param request
     * @param response
     */
    void handle_robot_task_state_request(const std::shared_ptr<duco_msg::srv::RobotTaskStateRquest::Request> request, std::shared_ptr<duco_msg::srv::RobotTaskStateRquest::Response> response);

private:
    /*机器人IP地址*/
    std::string server_host_="127.0.0.1";
    /*RPC连接标志*/
    bool controller_connected_flag_=false;
    /*机器人开始运动标志*/
    bool _start_move_;

    /*机器人急停标志*/
    bool _emergency_stopped_;

    std::vector<std::shared_ptr<DucoRPC::DucoCobot>> _duco_robots;
    std::vector<std::shared_ptr<EdgeTrigger<int>>> _edge_triggers;

    rclcpp::Service<duco_msg::srv::RobotControl>::SharedPtr _robot_control_srv;
    rclcpp::Service<duco_msg::srv::RobotMove>::SharedPtr _robot_move_srv;
    rclcpp::Service<duco_msg::srv::RobotIoControl>::SharedPtr _robot_io_control_srv;
    rclcpp::Service<duco_msg::srv::RobotTaskStateRquest>::SharedPtr _robot_task_state_request_srv;

    uint _arm_num = 1;
    uint _dof = 6;

    std::recursive_mutex _mu;
};

}
#endif
