#include "duco_ros_driver/RobotNode.h"
#include "duco_ros_driver/DucoCobot.h"



RobotNode::RobotNode(const std::string &ip)
{
    this->server_host=ip;
    this->duco_robot=std::make_shared<DucoRPC::DucoCobot>(this->server_host,PORT);
}

RobotNode::~RobotNode()
{
    disconnect();
}

bool RobotNode::connect()
{
    if(0==duco_robot->open())
    {
        this->is_connected=true;
        return true;
    }
    return false;
}

bool RobotNode::disconnect()
{
    this->is_connected=false;
    duco_robot->close();
    return true;
}


