#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "duco_msg/msg/duco_robot_state.hpp"
#include <mutex>
#include <duco_msg/srv/robot_control.hpp>
#include <std_msgs/msg/string.hpp>
#include <duco_msg/srv/grippers.hpp>
namespace DucoRPC
{
class DucoCobot;
};


class RobotControl: public rclcpp::Node
{
public :
    RobotControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RobotControl();

    /*运行*/
    void        run();

protected:
    /*连接机器人*/
    bool connectToRobotController();

    /*定时器信号，用于反馈机器人当前状态*/
    void getRobotStatus();

private:

    /*机器人IP地址*/
    std::string                             server_host_="192.168.120.138";

    /*机器人当前位置，用于失去连接备份*/
    std::vector<double>  current_joints_;

    /*RPC连接标志*/
    bool                       controller_connected_flag_=false;

    std::vector<std::shared_ptr<DucoRPC::DucoCobot> > duco_robots;
    std::vector<std::shared_ptr<DucoRPC::DucoCobot> > duco_robots_move_control;
    std::vector<std::shared_ptr<DucoRPC::DucoCobot> > duco_robots_gripper;


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   trajectory_execution_pub_ ;

    /*机器人轴名称*/
    std::vector<std::vector<std::string> >     joint_names_;

    /*机器人轴数量*/
    unsigned int                       axis_number_=6;

    rclcpp::Service<duco_msg::srv::RobotControl>::SharedPtr robot_service;
    rclcpp::Service<duco_msg::srv::Grippers>::SharedPtr gripper_service;


    void handleRobotCommand(const duco_msg::srv::RobotControl_Request::SharedPtr request,duco_msg::srv::RobotControl_Response::SharedPtr response);


    void handleGripperCommand(const duco_msg::srv::Grippers_Request::SharedPtr request,duco_msg::srv::Grippers_Response::SharedPtr response);






    std::recursive_mutex _mu;

    int arm_num=1;

};


#endif
