#include "duco_ros_driver/DucoCobot.h"
#include "duco_ros_driver/DucoRobotControl.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace DucoRPC;

namespace duco_driver
{

DucoRobotControl::DucoRobotControl(const rclcpp::NodeOptions &option)
 : Node("DucoRobotControl", option)
{
    uint arm_num_str=0;
    this->declare_parameter("arm_num", 0);
    if (this->get_parameter("arm_num",arm_num_str))
        this->_arm_num = arm_num_str;

    uint dof_num_str=0;
    this->declare_parameter("arm_dof",0);
    if (this->get_parameter("arm_dof"),dof_num_str)
        this->_dof = dof_num_str;

    for (uint i=0;i<_arm_num;++i)
    {
        std::string host="server_host_"+std::to_string(i+1);
        std::string ip="127.0.0.1";
        this->declare_parameter(host,"127.0.0.1");
        this->get_parameter(host,ip);
        std::shared_ptr<DucoRPC::DucoCobot> duco_cobot = std::make_shared<DucoRPC::DucoCobot>(ip,7003);
        _duco_robots.push_back(duco_cobot);
        std::shared_ptr<EdgeTrigger<int>> edg = std::make_shared<EdgeTrigger<int>>(Rising|Drop);
        _edge_triggers.push_back(edg);
    }

    _robot_control_srv = this->create_service<duco_msg::srv::RobotControl>(
                "/duco_robot/robot_control",
                std::bind(&DucoRobotControl::handle_robot_control, this,
                          std::placeholders::_1, std::placeholders::_2));
    _robot_move_srv = this->create_service<duco_msg::srv::RobotMove>(
                "/duco_robot/robot_move",
                std::bind(&DucoRobotControl::handle_robot_move, this,
                          std::placeholders::_1, std::placeholders::_2));
    _robot_io_control_srv = this->create_service<duco_msg::srv::RobotIoControl>(
                "/duco_robot/robot_io_control",
                std::bind(&DucoRobotControl::handle_robot_io_control, this,
                          std::placeholders::_1, std::placeholders::_2));
    _robot_task_state_request_srv = this->create_service<duco_msg::srv::RobotTaskStateRquest>(
                "/duco_robot/robot_task_state_request",
                std::bind(&DucoRobotControl::handle_robot_task_state_request, this,
                          std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Duco Robot Control Init Done");
}

DucoRobotControl::~DucoRobotControl()
{
    for(int i=0;i<_arm_num;i++)
    {
        _duco_robots[i]->close();
    }
}

void DucoRobotControl::run()
{
    std::lock_guard<std::recursive_mutex> lock(_mu);
    //RCLCPP_INFO(this->get_logger(), "Start the Robot Status!");
    /** connect to the robot controller **/
    if ( !controller_connected_flag_  )
    {
        if( connectToRobotController() )
        {
            /** Switches to ros-controller **/
            RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),"Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
        }
    }
}

bool DucoRobotControl::connectToRobotController()
{
    for(int i=0;i<_arm_num;i++)
    {
        if(_duco_robots[i]->open() != 0)
        {
            controller_connected_flag_=false;
            return false;
        }
    }
    controller_connected_flag_=true;
    return true;
}

void DucoRobotControl::handle_robot_control(const std::shared_ptr<duco_msg::srv::RobotControl::Request> request, std::shared_ptr<duco_msg::srv::RobotControl::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Handle Robot Control");
    RCLCPP_INFO(this->get_logger(),"Request:");
    RCLCPP_INFO(this->get_logger(),"    command: %s", request->command.c_str());
    RCLCPP_INFO(this->get_logger(),"    arm_num: %i", request->arm_num);
    RCLCPP_INFO(this->get_logger(),"    block: %s", std::to_string(request->block).c_str());

    int32_t ret = -1;
    if (request->arm_num >= this->_arm_num)
    {
        RCLCPP_ERROR(this->get_logger(),"Receive wrong arm index");
        response->response = std::to_string(ret);
        return;
    }
    if ("poweron"==request->command)
    {
        ret = _duco_robots[request->arm_num]->power_on(request->block);
    }
    else if ("enable"==request->command)
    {
        ret = _duco_robots[request->arm_num]->enable(request->block);
    }
    else if ("disable"==request->command)
    {
        ret = _duco_robots[request->arm_num]->disable(request->block);
    }
    else if("poweroff"==request->command)
    {
        ret =_duco_robots[request->arm_num]->power_off(request->block);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Receive invalid command");
    }
    response->response = std::to_string(ret);
}

void DucoRobotControl::handle_robot_move(const std::shared_ptr<duco_msg::srv::RobotMove::Request> request, std::shared_ptr<duco_msg::srv::RobotMove::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Handle Robot Move");
    RCLCPP_INFO(this->get_logger(),"Request:");
    RCLCPP_INFO(this->get_logger(),"    command: %s", request->command.c_str());
    RCLCPP_INFO(this->get_logger(),"    arm_num: %i", request->arm_num);
    RCLCPP_INFO(this->get_logger(),"    p size: %i", request->p.size());
    RCLCPP_INFO(this->get_logger(),"    q size: %i", request->q.size());
    std::string msg = "";
    if (request->p.size() > 0)
    {
        msg = "p: {";
        for (int i=0;i<request->p.size()-1;++i)
        {
            msg += std::to_string(request->p[i]);
            msg += ",";
        }
        msg += std::to_string(request->p[request->p.size()-1]) + "}";
        RCLCPP_INFO(this->get_logger(),"    %s", msg.c_str());
    }
    if (request->q.size() > 0)
    {
        msg = "q: {";
        for (int i=0;i<request->q.size()-1;++i)
        {
            msg += std::to_string(request->q[i]);
            msg += ",";
        }
        msg += std::to_string(request->q[request->q.size()-1]) + "}";
        RCLCPP_INFO(this->get_logger(),"    %s", msg.c_str());
    }
    RCLCPP_INFO(this->get_logger(),"    v: %f", request->v);
    RCLCPP_INFO(this->get_logger(),"    a: %f", request->a);
    RCLCPP_INFO(this->get_logger(),"    r: %f", request->r);
    RCLCPP_INFO(this->get_logger(),"    tool: %s", request->tool.c_str());
    RCLCPP_INFO(this->get_logger(),"    wobj: %s", request->wobj.c_str());
    RCLCPP_INFO(this->get_logger(),"    block: %s", std::to_string(request->block).c_str());

    int32_t ret = -1;
    if (request->arm_num >= this->_arm_num)
    {
        RCLCPP_ERROR(this->get_logger(),"Receive wrong arm index");
        response->response = std::to_string(ret);
        return;
    }
    if ("movej"==request->command)
    {
        if (request->q.size()<this->_dof)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Joints Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        std::vector<double> joints;
        joints.resize(this->_dof);
        for (int i=0;i<_dof;++i)
        {
            joints[i] = request->q[i];
        }
        ret = _duco_robots[request->arm_num]->movej(
                    joints,
                    request->v,request->a,
                    request->r,
                    request->block);
    }
    else if ("movej2"==request->command)
    {
        if (request->q.size()<this->_dof)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Joints Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        std::vector<double> joints;
        joints.resize(this->_dof);
        for (int i=0;i<_dof;++i)
        {
            joints[i] = request->q[i];
        }
        ret = _duco_robots[request->arm_num]->movej2(
                    joints,
                    request->v,request->a,
                    request->r,
                    request->block);
    }
    else if ("movejpose"==request->command)
    {
        if (request->p.size()<6)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Cartesian Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        if (request->q.size()<this->_dof)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Joints Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        std::vector<double> joints;
        joints.resize(this->_dof);
        joints.clear();
        for (int i=0;i<_dof;++i)
        {
            joints.push_back(request->q[i]);
        }
        std::vector<double> pose;
        pose.resize(6);
        pose.clear();
        for (int i=0;i<6;++i)
        {
            pose.push_back(request->p[i]);
        }
        ret = _duco_robots[request->arm_num]->movej_pose(
                    pose,
                    request->v,request->a,
                    request->r,
                    joints,
                    request->tool,request->wobj,
                    request->block);
    }
    else if ("movejpose2"==request->command)
    {
        if (request->p.size()<6)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Cartesian Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        if (request->q.size()<this->_dof)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Joints Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        std::vector<double> joints;
        joints.resize(this->_dof);
        joints.clear();
        for (int i=0;i<_dof;++i)
        {
            joints.push_back(request->q[i]);
        }
        std::vector<double> pose;
        pose.resize(6);
        pose.clear();
        for (int i=0;i<6;++i)
        {
            pose.push_back(request->p[i]);
        }
        ret = _duco_robots[request->arm_num]->movej_pose(
                    pose,
                    request->v,request->a,
                    request->r,
                    joints,
                    request->tool,request->wobj,
                    request->block);
    }
    else if ("movel"==request->command)
    {
        if (request->p.size()<6)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Cartesian Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        if (request->q.size()<this->_dof)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Joints Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        std::vector<double> joints;
        joints.resize(this->_dof);
        joints.clear();
        for (int i=0;i<_dof;++i)
        {
            joints.push_back(request->q[i]);
        }
        std::vector<double> pose;
        pose.resize(6);
        pose.clear();
        for (int i=0;i<6;++i)
        {
            pose.push_back(request->p[i]);
        }
        ret = _duco_robots[request->arm_num]->movel(
                    pose,
                    request->v,request->a,
                    request->r,
                    joints,
                    request->tool,request->wobj,
                    request->block);
    }
    else if ("movetcp"==request->command)
    {
        if (request->p.size()<6)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Cartesian Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        if (request->q.size()<this->_dof)
        {
            RCLCPP_ERROR(this->get_logger(),"Invalid Joints Position Dof");
            response->response = std::to_string(ret);
            return;
        }
        std::vector<double> joints;
        joints.resize(this->_dof);
        joints.clear();
        for (int i=0;i<_dof;++i)
        {
            joints.push_back(request->q[i]);
        }
        std::vector<double> pose;
        pose.resize(6);
        pose.clear();
        for (int i=0;i<6;++i)
        {
            pose.push_back(request->p[i]);
        }
        ret = _duco_robots[request->arm_num]->tcp_move(
                    pose,
                    request->v,request->a,
                    request->r,
                    request->tool,
                    request->block);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Receive invalid command");
    }
    response->response = std::to_string(ret);
}

void DucoRobotControl::handle_robot_io_control(const std::shared_ptr<duco_msg::srv::RobotIoControl::Request> request, std::shared_ptr<duco_msg::srv::RobotIoControl::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Handle Robot Io Control");
    RCLCPP_INFO(this->get_logger(),"Request:");
    RCLCPP_INFO(this->get_logger(),"    command: %s", request->command.c_str());
    RCLCPP_INFO(this->get_logger(),"    arm_num: %i", request->arm_num);
    RCLCPP_INFO(this->get_logger(),"    type: %i", request->type);
    RCLCPP_INFO(this->get_logger(),"    port: %i", request->port);
    RCLCPP_INFO(this->get_logger(),"    value: %s", std::to_string(request->value).c_str());
    RCLCPP_INFO(this->get_logger(),"    block: %s", std::to_string(request->block).c_str());

    int32_t ret = -1;
    if (request->arm_num >= this->_arm_num)
    {
        RCLCPP_ERROR(this->get_logger(),"Receive wrong arm index");
        response->response = std::to_string(ret);
        return;
    }
    if ("setIo"==request->command || "SetIo"==request->command)
    {
        if (0==request->type)
        {
            if (request->port<=0 || request->port>16)
                RCLCPP_ERROR(this->get_logger(),"Invalid io port");
            else
                ret = _duco_robots[request->arm_num]->set_standard_digital_out(request->port,request->value,request->block);
        }
        else if(1==request->type)
        {
            if (request->port<=0 || request->port>2)
                RCLCPP_ERROR(this->get_logger(),"Invalid io port");
            else
                ret = _duco_robots[request->arm_num]->set_tool_digital_out(request->port,request->value,request->block);
        }
        else
            RCLCPP_ERROR(this->get_logger(),"Invalid Io Type");
    }
    else if ("getIo"==request->command || "GetIo"==request->command)
    {
        if (0==request->type)
        {
            if (request->port<=0 || request->port>16)
                RCLCPP_ERROR(this->get_logger(),"Invalid io port");
            else
                ret = _duco_robots[request->arm_num]->get_standard_digital_in(request->port);
        }
        else if(1==request->type)
        {
            if (request->port<=0 || request->port>2)
                RCLCPP_ERROR(this->get_logger(),"Invalid io port");
            else
                ret = _duco_robots[request->arm_num]->get_tool_digital_in(request->port);
        }
        else
            RCLCPP_ERROR(this->get_logger(),"Invalid Io Type");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Receive invalid command");
    }
    response->response = std::to_string(ret);
}

void DucoRobotControl::handle_robot_task_state_request(const std::shared_ptr<duco_msg::srv::RobotTaskStateRquest::Request> request, std::shared_ptr<duco_msg::srv::RobotTaskStateRquest::Response> response)
{
    RCLCPP_INFO(this->get_logger(),"Handle Robot Task State Request");
    RCLCPP_INFO(this->get_logger(),"Request:");
    RCLCPP_INFO(this->get_logger(),"    id: %s", request->id.c_str());
    RCLCPP_INFO(this->get_logger(),"    arm_num: %i", request->arm_num);

    int32_t ret = -1;
    if (request->arm_num >= this->_arm_num)
    {
        RCLCPP_ERROR(this->get_logger(),"Receive wrong arm index");
        response->response = std::to_string(ret);
        return;
    }

    ret =_duco_robots[request->arm_num]->get_noneblock_taskstate(std::stoi(request->id));
    response->response = std::to_string(ret);
}

}

using namespace duco_driver;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<DucoRobotControl> n=std::make_shared<DucoRobotControl>();

    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(n);
    while(rclcpp::ok())
    {
        executor->spin_some();  // 处理当前已经准备好的所有回调
        // 这里可以执行其他需要周期性执行的代码
        // ...
        // 让出 CPU 时间或执行其他非回调相关任务
        n->run();

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 模拟工作负载
    }

    RCLCPP_WARN(n->get_logger(),"Exiting duco robot control");
    return(0);
}
