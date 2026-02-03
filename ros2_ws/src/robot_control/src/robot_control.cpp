#include "robot_control/robot_control.h"
#include "robot_control/DucoCobot.h"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/display_robot_state.h>
// #include <moveit_msgs/msg/display_trajectory.h>
// #include <moveit_msgs/msg/attached_collision_object.h>
// #include <moveit_msgs/msg/collision_object.h>
#include <geometry_msgs/msg/pose.hpp>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/conversions.h>
// #include <moveit/kinematic_constraints/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <moveit_msgs/msg/orientation_constraint.hpp>
// #include <moveit_msgs/msg/constraints.hpp>

geometry_msgs::msg::Pose createPose(double x, double y, double z, double roll, double pitch, double yaw) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    pose.orientation = tf2::toMsg(q);

    return pose;
}
std::vector<uint8_t> calculateAndAppendCRC(std::vector<uint8_t> data) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < data.size(); i++) {
        crc ^= data[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    // 将CRC的低字节和高字节附加到数据的末尾
    data.push_back(crc & 0xFF);        // 低字节
    data.push_back((crc >> 8) & 0xFF); // 高字节

    return data;
}

std::vector<int8_t> vu2vi(std::vector<uint8_t> data) {
    std::vector<int8_t>tmp;
    for(int i=0;i<data.size();i++)
    {
        tmp.push_back(data[i]);
    }
    return tmp;
}

RobotControl::RobotControl(const rclcpp::NodeOptions &options): Node("RobotControl", options)
{

    std::string s;
    int arm_num_str;
    this->declare_parameter("arm_num",0);
    bool ret=this->get_parameter("arm_num",arm_num_str);
    if(ret)
        this->arm_num=arm_num_str;
    for(int i=0;i<arm_num;i++)
    {
        std::string host="server_host_"+std::to_string(i+1);
        std::string ip="127.0.0.1";
        this->declare_parameter(host,"127.0.0.1");
        this->get_parameter(host,ip);
        std::shared_ptr<DucoRPC::DucoCobot >duco_robot = std::make_shared<DucoRPC::DucoCobot>(ip, 7003);
        std::shared_ptr<DucoRPC::DucoCobot >duco_robot_move_control = std::make_shared<DucoRPC::DucoCobot>(ip, 7003);
        std::shared_ptr<DucoRPC::DucoCobot >duco_robot_gripper = std::make_shared<DucoRPC::DucoCobot>(ip, 7003);


        duco_robots.push_back(duco_robot);
        duco_robots_move_control.push_back(duco_robot_move_control);
        duco_robots_gripper.push_back(duco_robot_gripper);

    }
    for(int i=0;i<arm_num;i++)
    {
        if(duco_robots[i]->open() != 0 || duco_robots_move_control[i]->open()!=0 || duco_robots_gripper[i]->open()!=0)
        {
            controller_connected_flag_=false;
            RCLCPP_ERROR(this->get_logger(),"Cann't connect to the robot controller!");
            return ;
        }
    }
    controller_connected_flag_=true;
    robot_service=this->create_service<duco_msg::srv::RobotControl>("robot_service",std::bind(&RobotControl::handleRobotCommand,this,std::placeholders::_1,std::placeholders::_2));
    gripper_service=this->create_service<duco_msg::srv::Grippers>("gripper_service",std::bind(&RobotControl::handleGripperCommand,this,std::placeholders::_1,std::placeholders::_2));
    trajectory_execution_pub_=this->create_publisher<std_msgs::msg::String>("trajectory_execution_event",100);
}

RobotControl::~RobotControl()
{
    for(int i=0;i<duco_robots.size();i++)
    {
        duco_robots[i]->close();
        duco_robots_gripper[i]->close();
        duco_robots_move_control[i]->close();
    }
}

void RobotControl::handleRobotCommand(const duco_msg::srv::RobotControl_Request::SharedPtr request, duco_msg::srv::RobotControl_Response::SharedPtr response)
{
//    std_msgs::msg::String msg;
//    int move_ret=0;
//    if(request->command=="stop")
//    {
//        RCLCPP_ERROR(this->get_logger(),"----------------stop!");

//        for(int i=0;i<arm_num;i++)
//        {
//            duco_robots_move_control[i]->stop(false);
//        }
//        msg.data="stop";
//        trajectory_execution_pub_->publish(msg);
//        response->response ="success";
//        RCLCPP_ERROR(this->get_logger(),"----------------stop!");

//        return;

//    }else if(request->command=="pause")
//    {
//        for(int i=0;i<arm_num;i++)
//        {
//            duco_robots_move_control[i]->pause(false);
//        }
//        msg.data="pause";
//        trajectory_execution_pub_->publish(msg);
//    }else if(request->command=="resume")
//    {
//        for(int i=0;i<arm_num;i++)
//        {
//            duco_robots_move_control[i]->resume(false);
//        }
//        msg.data="resume";
//        trajectory_execution_pub_->publish(msg);
//    }else if(request->command=="movej")
//    {
//        std::string  pre="arm_";
//        std::string num_str=request->arm_group.data();
//        num_str=num_str.substr(pre.length());
//        int num=std::stoi(num_str);
//        if(num<=0 || num>arm_num)
//        {
//            response->response ="failed";
//            return ;
//        }
//        std::vector<double > joints;
//        for(int i=0;i<request->p.size();i++)
//        {
//            joints.push_back(request->p[i]);
//        }
//        move_ret=duco_robots[num-1]->movej2(joints,request->v,request->a,request->r,true);

//    }else if(request->command=="movel")
//    {
//        std::string  pre="arm_";
//        std::string num_str=request->arm_group.data();
//        num_str=num_str.substr(pre.length());
//        int num=std::stoi(num_str);
//        if(num<=0 || num>arm_num)
//        {
//            response->response ="failed";
//            return ;
//        }
//        std::vector<double > p;
//        std::vector<double > q_near;
//        for(int i=0;i<request->p.size();i++)
//        {
//            p.push_back(request->p[i]);
//        }
//        for(int i=0;i<request->q_near.size();i++)
//        {
//            q_near.push_back(request->p[i]);
//        }
//        move_ret=duco_robots[num-1]->movel(p,request->v,request->a,request->r,q_near,request->tool,request->wobj,true);
//    }else if(request->command=="movej_pose")
//    {
//        std::string  pre="arm_";
//        std::string num_str=request->arm_group.data();
//        num_str=num_str.substr(pre.length());
//        int num=std::stoi(num_str);
//        if(num<=0 || num>arm_num)
//        {
//            response->response ="failed";
//            return ;
//        }
//        std::vector<double > p;
//        std::vector<double > q_near;
//        for(int i=0;i<request->p.size();i++)
//        {
//            p.push_back(request->p[i]);
//        }
//        for(int i=0;i<request->q_near.size();i++)
//        {
//            q_near.push_back(request->p[i]);
//        }
//        move_ret=duco_robots[num-1]->movej_pose2(p,request->v,request->a,request->r,q_near,request->tool,request->wobj,true);
//    }else if(request->command=="servoj_pose")
//    {
//        std::string  pre="arm_";
//        std::string num_str=request->arm_group.data();
//        num_str=num_str.substr(pre.length());
//        int num=std::stoi(num_str);
//        if(num<=0 || num>arm_num)
//        {
//            response->response ="failed";
//            return ;
//        }
//        std::vector<double > p;
//        std::vector<double > q_near;
//        for(int i=0;i<request->p.size();i++)
//        {
//            p.push_back(request->p[i]);
//        }
//        for(int i=0;i<request->q_near.size();i++)
//        {
//            q_near.push_back(request->p[i]);
//        }
//        duco_robots[num-1]->servoj_pose(p,request->v,request->a,q_near,request->tool,request->wobj,false);
//    }else if(request->command=="plan_joints")
//    {
//        std::string PLANNING_GROUP =request->arm_group;  // 根据你的机器人模型调整
//        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
//        std::vector<double> joints;
//        if(PLANNING_GROUP=="arm_1")
//        {
//            for(int i=0;i<request->arm_1.size();i++)
//            {
//                joints.push_back(request->arm_1[i]);
//            }
//        }else if(PLANNING_GROUP=="arm_2")
//        {
//            for(int i=0;i<request->arm_2.size();i++)
//            {
//                joints.push_back(request->arm_2[i]);
//            }
//        }
//        move_group.setJointValueTarget(joints);
//        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//        bool success =false;
//        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//        if(success)
//        {
//            RCLCPP_INFO(this->get_logger(),"plan success " );
//            move_group.move();
//        }
//        else {
//            RCLCPP_INFO(this->get_logger(),"plan error " );
//            response->response ="failed";
//            return ;
//        }
//    }else if(request->command=="plan_pose")
//    {
//        std::string PLANNING_GROUP =request->arm_group;  // 根据你的机器人模型调整
//        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
//        std::vector<double> pose;
//        move_group.setPlanningTime(10.0); // 设置规划时间为10秒
//        move_group.setNumPlanningAttempts(10); // 设置规划尝试次数为5次

//        // 设置不同的规划器
//        move_group.setPlannerId("RRTConnect");
//        if(PLANNING_GROUP=="arm_1")
//        {
//            for(int i=0;i<request->arm_1.size();i++)
//            {
//                pose.push_back(request->arm_1[i]);
//            }
//        }else if(PLANNING_GROUP=="arm_2")
//        {
//            for(int i=0;i<request->arm_2.size();i++)
//            {
//                pose.push_back(request->arm_2[i]);
//            }
//        }
//        geometry_msgs::msg::Pose pose_target=createPose(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);

//        move_group.clearPathConstraints();
//        // 添加运动约束
//        moveit_msgs::msg::OrientationConstraint ocm;
//        ocm.link_name = "end_1";
//        ocm.header.frame_id = "base_link";
//        ocm.orientation=pose_target.orientation;
//        ocm.absolute_x_axis_tolerance = 3.1415;
//        ocm.absolute_y_axis_tolerance = 3.1415;
//        ocm.absolute_z_axis_tolerance = 3.1415;
//        ocm.weight = 1.0;

//        moveit_msgs::msg::Constraints test_constraints;
//        test_constraints.orientation_constraints.push_back(ocm);
//        move_group.setPathConstraints(test_constraints);

//        std::vector<double> tmp_p={pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]};
//        std::vector<double> tmp_q;
//        std::vector<double> ret_joints;
//        std::vector<double> tool;
//        std::vector<double> wobj;

//        //duco_robots_move_control[0]->get_actual_joints_position(tmp_q);

//        //duco_robots_move_control[0]->cal_ikine(ret_joints,tmp_p,tmp_q,tool,wobj);

//        //move_group.setJointValueTarget(ret_joints);

//        move_group.setPoseTarget(pose_target);
//        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//        bool success =false;
//        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//        if(success)
//        {
//            RCLCPP_INFO(this->get_logger(),"plan success " );
//            move_group.move();
//        }
//        else {
//            RCLCPP_INFO(this->get_logger(),"plan error " );
//            response->response ="failed";
//            return ;
//        }
//    }
//    response->response ="success";
    return ;

}

void RobotControl::handleGripperCommand(const duco_msg::srv::Grippers_Request::SharedPtr request, duco_msg::srv::Grippers_Response::SharedPtr response)
{
    if(request->command=="abs_pos")
    {
        std::vector<uint8_t> comm;
        comm.push_back(1);
        comm.push_back(0x10);
        comm.push_back(0x13);
        comm.push_back(0x8A);
        comm.push_back(0);
        comm.push_back(2);
        comm.push_back(4);

        uint32_t pos=request->pose*1000;
        comm.push_back((pos>>24) & 0xFF);
        comm.push_back((pos>>16) & 0xFF);
        comm.push_back((pos>>8) & 0xFF);
        comm.push_back(pos & 0xFF);

        comm=calculateAndAppendCRC(comm);

        std::shared_ptr<DucoRPC::DucoCobot>  gripper;
        if(request->name=="arm_1")
        {
            gripper =duco_robots_gripper[0];
        }else if(request->name=="arm_2")
        {
            gripper =duco_robots_gripper[1];
        }else
        {
            response->response="failed";
            return;
        }
        gripper->write_raw_data_485(vu2vi(comm));
        usleep(30000);
        comm.clear();
        comm.push_back(1);
        comm.push_back(5);
        comm.push_back(5);
        comm.push_back(0x80);
        comm.push_back(0);
        comm.push_back(0);
        comm=calculateAndAppendCRC(comm);
        gripper->write_raw_data_485(vu2vi(comm));
        usleep(30000);

        comm.clear();
        comm.push_back(1);
        comm.push_back(5);
        comm.push_back(5);
        comm.push_back(0x80);
        comm.push_back(0xFF);
        comm.push_back(0);
        comm=calculateAndAppendCRC(comm);
        gripper->write_raw_data_485(vu2vi(comm));
        usleep(30000);
        response->response="success";



    }else if(request->command=="force_pos")
    {
        std::vector<uint8_t> comm;
        std::shared_ptr<DucoRPC::DucoCobot>  gripper;
        if(request->name=="arm_1")
        {
            gripper =duco_robots_gripper[0];
        }else if(request->name=="arm_2")
        {
            gripper =duco_robots_gripper[1];
        }else
        {
            response->response="failed";
            return;
        }
        gripper->write_raw_data_485(vu2vi(comm));
        usleep(30000);
        comm.clear();
        comm.push_back(1);
        comm.push_back(5);
        comm.push_back(5);
        comm.push_back(0x81);
        comm.push_back(0);
        comm.push_back(0);
        comm=calculateAndAppendCRC(comm);
        gripper->write_raw_data_485(vu2vi(comm));
        usleep(30000);

        comm.clear();
        comm.push_back(1);
        comm.push_back(5);
        comm.push_back(5);
        comm.push_back(0x81);
        comm.push_back(0xFF);
        comm.push_back(0);
        comm=calculateAndAppendCRC(comm);
        gripper->write_raw_data_485(vu2vi(comm));
        usleep(30000);
        response->response="success";

    }else
    {
        response->response="failed";
    }

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<RobotControl> n=std::make_shared<RobotControl>();

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(n);
    executor->spin();
    rclcpp::shutdown();
    return(0);
}


