#include "duco_ros_driver/DucoCobot.h"
#include "duco_ros_driver/DucoRobotStatus.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace DucoRPC;

namespace duco_driver
{

DucoRobotStatus::DucoRobotStatus(const rclcpp::NodeOptions & options ) : Node("DucoRobotStatus", options)
{
    //decalare total arm number
    uint arm_num_str;
    this->declare_parameter("arm_num",0);
    bool ret=this->get_parameter("arm_num",arm_num_str);
    if(ret)
        this->arm_num=arm_num_str;

    //decalare dof of each arm
    uint dof_num_str;
    this->declare_parameter("arm_dof",0);
    ret = this->get_parameter("arm_dof",dof_num_str);
    if(ret)
        this->_dof = dof_num_str;

    //decalare the robot rpc client object
    for(uint i=0;i<arm_num;i++)
    {
        std::string host="server_host_"+std::to_string(i+1);
        std::string ip="127.0.0.1";
        this->declare_parameter(host,"127.0.0.1");
        this->get_parameter(host,ip);
        std::shared_ptr<DucoRPC::DucoCobot >duco_robot = std::make_shared<DucoRPC::DucoCobot>(ip, 7003);
        duco_robots.push_back(duco_robot);
        std::shared_ptr<EdgeTrigger<int> > edg=std::make_shared<EdgeTrigger<int> >(Rising | Drop);
        edge_triggers.push_back(edg);
    }

    //declare the joint name for each robot
    joint_names_.resize(arm_num);
    _last_joints_actual_pos.resize(arm_num);
    _last_joints_actual_vel.resize(arm_num);
    _last_joints_actual_acc.resize(arm_num);
    _last_joints_actual_torq.resize(arm_num);
    for (int i=0;i<arm_num;++i)
    {
        for (int j=0;j<_dof;++j)
        {
            joint_names_[i].push_back("arm_"+std::to_string(i+1)+"_joint_"+std::to_string(j+1));
            _last_joints_actual_pos[i].push_back(0);
            _last_joints_actual_vel[i].push_back(0);
            _last_joints_actual_acc[i].push_back(0);
            _last_joints_actual_torq[i].push_back(0);
        }
    }

    /*通知MoveGroup机器人的状态*/
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 300);
    joint_feedback_pub_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>("feedback_states", 100);
    /*通知客户端机器人的状态*/
    robot_state_pub_ = this->create_publisher<duco_msg::msg::DucoRobotState> ("/duco_cobot/robot_state",30);
    stop_pub_=this->create_publisher<std_msgs::msg::String> ("trajectory_execution_event",30);
    RCLCPP_INFO(this->get_logger(),"Duco Robot Status Init Done");
}

DucoRobotStatus::~DucoRobotStatus()
{
    for(int i=0;i<arm_num;i++)
    {
        duco_robots[i]->close();
    }
}

void DucoRobotStatus::getRobotStatus()
{
    std::lock_guard<std::recursive_mutex> lock(_mu);
    std::vector<std::vector<double> > robot_joint_actual_pos;
    std::vector<std::vector<double> > robot_joint_actual_vel;
    std::vector<std::vector<double> > robot_joint_actual_acc;
    std::vector<std::vector<double> > robot_joint_actual_torq;
    robot_joint_actual_pos.resize(arm_num);
    robot_joint_actual_vel.resize(arm_num);
    robot_joint_actual_acc.resize(arm_num);
    robot_joint_actual_torq.resize(arm_num);
    //获取机器人控制器状态数据并转发给ROS
    if( controller_connected_flag_ )
    {
        for(int axis=0;axis<arm_num;axis++)
        {
            duco_robots[axis]->get_actual_joints_position(robot_joint_actual_pos[axis]);
            if (robot_joint_actual_pos[axis].size() < _dof)
            {
                RCLCPP_ERROR(this->get_logger(),"Actual joint position data size incorrect!");
                return;
            }
            duco_robots[axis]->get_actual_joints_speed(robot_joint_actual_vel[axis]);
            if (robot_joint_actual_vel[axis].size() < _dof)
            {
                RCLCPP_ERROR(this->get_logger(),"Actual joint velocity data size incorrect!");
                return;
            }
            duco_robots[axis]->get_actual_joints_acceleration(robot_joint_actual_acc[axis]);
            if (robot_joint_actual_vel[axis].size() < _dof)
            {
                RCLCPP_ERROR(this->get_logger(),"Actual joint acceleration data size incorrect!");
                return;
            }
            duco_robots[axis]->get_actual_joints_torque(robot_joint_actual_torq[axis]);
            if (robot_joint_actual_vel[axis].size() < _dof)
            {
                RCLCPP_ERROR(this->get_logger(),"Actual joint torque data size incorrect!");
                return;
            }

            for(int i = 0;i<arm_num;i++)
            {
                _last_joints_actual_pos[i].clear();
                _last_joints_actual_vel[i].clear();
                _last_joints_actual_acc[i].clear();
                _last_joints_actual_torq[i].clear();
                for (uint j=0;j<_dof;++j)
                {
                    _last_joints_actual_pos[i].push_back(robot_joint_actual_pos[axis][i]);
                    _last_joints_actual_vel[i].push_back(robot_joint_actual_vel[axis][i]);
                    _last_joints_actual_acc[i].push_back(robot_joint_actual_acc[axis][i]);
                    _last_joints_actual_torq[i].push_back(robot_joint_actual_torq[axis][i]);
                }
            }

            //RCLCPP_INFO(this->get_logger(), "duco_robot_->robotmoving before " );
            /** Get the buff size of thr rib **/
            start_move_ = duco_robots[axis]->robotmoving();

            RobotStatusList  status;
            // this is controlled by Robot Controller
            duco_robots[axis]->getRobotStatus(status);

            // Check if status vectors have sufficient size
            if (status.jointExpectPosition.size() < _dof ||
                status.jointExpectVelocity.size() < _dof ||
                status.jointExpectAccelera.size() < _dof ||
                status.jointActualPosition.size() < _dof ||
                status.jointActualVelocity.size() < _dof ||
                status.jointActualAccelera.size() < _dof ||
                status.jointActualCurrent.size() < _dof ||
                status.jointTemperature.size() < _dof ||
                status.driverTemperature.size() < _dof ||
                status.cartExpectPosition.size() < _dof ||
                status.cartExpectVelocity.size() < _dof ||
                status.cartExpectAccelera.size() < _dof ||
                status.cartActualPosition.size() < _dof ||
                status.cartActualVelocity.size() < _dof ||
                status.cartActualAccelera.size() < _dof ||
                status.slaveReady.size() < _dof)
            {
                // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot status data incomplete or invalid size. Skipping update.");
                return;
            }

            //status.robotState==5 disable
            if((*edge_triggers[axis])(status.robotState)==Drop && status.robotState==5)
            {
                std_msgs::msg::String msg;
                msg.data="stop";
                stop_pub_->publish(msg);
            }


            // publish robot_status information to the controller action server.
            //                    robot_status_.mode.val            = (int8_t)status.robotState;
            //                    robot_status_.e_stopped.val       = (int8_t)(status.emcStopSignal );
            //                    robot_status_.drives_powered.val  = (int8_t)1;
            //                    robot_status_.motion_possible.val = (int)(!start_move_);
            //                    robot_status_.in_motion.val       = (int)start_move_;
            //                    robot_status_.in_error.val        = (int)0;   //used for protective stop.
            //                    robot_status_.error_code          = (int32_t)status.robotError;

            uint count = robot_joint_actual_pos[axis].size();

            //RCLCPP_INFO(this->get_logger(), "duco_robot_->getRobotStatus count %d",count);

            for( uint i=0;i<_dof;++i)
            {
                robot_state_.joint_expect_position[i] = status.jointExpectPosition[i];
                robot_state_.joint_expect_velocity[i] = status.jointExpectVelocity[i];
                robot_state_.joint_expect_accelera[i] = status.jointExpectAccelera[i];
                robot_state_.joint_actual_position[i] = status.jointActualPosition[i];
                robot_state_.joint_actual_velocity[i] = status.jointActualVelocity[i];
                robot_state_.joint_actual_accelera[i] = status.jointActualAccelera[i];
                robot_state_.joint_actual_current[i] = status.jointActualCurrent[i];

                robot_state_.cart_expect_position[i] = status.cartExpectPosition[i];
                robot_state_.cart_expect_velocity[i] = status.cartExpectVelocity[i];
                robot_state_.cart_expect_accelera[i] = status.cartExpectAccelera[i];
                robot_state_.cart_actual_position[i]  = status.cartActualPosition[i];
                robot_state_.cart_actual_velocity[i] = status.cartActualAccelera[i];
                robot_state_.cart_actual_accelera[i] = status.cartActualAccelera[i];
                robot_state_.slave_ready[i] = status.slaveReady[i];

                robot_state_.joint_temperature[i] = status.jointTemperature[i];
                robot_state_.driver_temperature[i] = status.driverTemperature[i];
            }

            robot_state_.collision = status.collision;
            robot_state_.collision_axis = status.collisionAxis;

            robot_state_.emc_stop_signal = status.emcStopSignal;
            robot_state_.robot_state = status.robotState;
            robot_state_.robot_error = status.robotError;

            //RCLCPP_DEBUG(this->get_logger(), "duco_robot_->getRobotStatus ");

            robot_state_pub_->publish(robot_state_);
        }
    }

    {
        //joint_states_pub_
        //joint_feedback_pub_
        sensor_msgs::msg::JointState joint_state;

        uint total_dof = arm_num*_dof;
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name.resize(total_dof);
        joint_state.position.resize(total_dof);
        joint_state.velocity.resize(total_dof);
        joint_state.effort.resize(total_dof);
        for(int i=0;i<arm_num;i++)
        {
            for (int j=0;j<_dof;++j)
            {
                joint_state.name[i*_dof+j] = joint_names_[i][j];
                if(controller_connected_flag_)
                {
                    joint_state.position[i*_dof+j] = robot_joint_actual_pos[i][j];
                    joint_state.velocity[i*_dof+j] = robot_joint_actual_vel[i][j];
                    joint_state.effort[i*_dof+j] = robot_joint_actual_torq[i][j];
                }
                else
                {
                    joint_state.position[i*_dof+j] = _last_joints_actual_pos[i][j];
                    joint_state.velocity[i*_dof+j] = _last_joints_actual_vel[i][j];
                    joint_state.effort[i*_dof+j] = _last_joints_actual_torq[i][j];
                }
            }
        }

        joint_states_pub_->publish(joint_state);

        control_msgs::action::FollowJointTrajectory_Feedback joint_feedback;
        joint_feedback.joint_names.resize(total_dof);
        joint_feedback.actual.positions.resize(total_dof);
        joint_feedback.actual.velocities.resize(total_dof);
        joint_feedback.actual.accelerations.resize(total_dof);
        joint_feedback.actual.effort.resize(total_dof);
        for(uint i=0;i<arm_num;i++)
        {
            for (uint j=0;j<_dof;++j)
            {
                joint_feedback.joint_names[i*_dof+j] = joint_names_[i][j];
                if(controller_connected_flag_)
                {
                    joint_feedback.actual.positions[i*_dof+j] = robot_joint_actual_pos[i][j];
                    joint_feedback.actual.velocities[i*_dof+j] = robot_joint_actual_vel[i][j];
                    joint_feedback.actual.accelerations[i*_dof+j] = robot_joint_actual_acc[i][j];
                    joint_feedback.actual.effort[i*_dof+j] = robot_joint_actual_torq[i][j];
                }
                else
                {
                    joint_feedback.actual.positions[i*_dof+j] = _last_joints_actual_pos[i][j];
                    joint_feedback.actual.velocities[i*_dof+j] = _last_joints_actual_vel[i][j];
                    joint_feedback.actual.accelerations[i*_dof+j] = _last_joints_actual_acc[i][j];
                    joint_feedback.actual.effort[i*_dof+j] = _last_joints_actual_torq[i][j];
                }
            }
        }
        joint_feedback_pub_->publish(joint_feedback);
    }//joint_states_pub_   joint_feedback_pub_
}

bool DucoRobotStatus::connectToRobotController()
{
    for(int i=0;i<arm_num;i++)
    {
        if(duco_robots[i]->open() != 0)
        {
            controller_connected_flag_=false;
            return false;
        }
    }
    controller_connected_flag_=true;
    return true;
}

void DucoRobotStatus::run()
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
    getRobotStatus();
}

}

