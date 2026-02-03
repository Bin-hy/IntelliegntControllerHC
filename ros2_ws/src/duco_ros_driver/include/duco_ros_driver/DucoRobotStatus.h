#ifndef DUCOROBOTSTATUS_H
#define DUCOROBOTSTATUS_H


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "duco_msg/msg/duco_robot_state.hpp"
#include <mutex>
#include <std_msgs/msg/string.hpp>
namespace DucoRPC
{
class DucoCobot;
};

namespace duco_driver
{

#define Rising 1
#define Drop 1<<1
template<class T>
/**
 * @brief The default comparison function
 *
 */
class defaultCmpFun
{
public:
    /**
     * @brief comparison function
     *
     * @param t1 value t1
     * @param t2 value t2
     * @return int operator
     */
    int operator()(const T &t1,const T &t2)
    {
        if(t1>t2)
            return 1;
        else if(t1==t2)
            return 0;
        else
            return -1;
    }
};

template<>
class defaultCmpFun<std::string>
{
public:
    /**
     * @brief comparison function
     *
     * @param t1 value t1
     * @param t2 value t2
     * @return int operator
     */
    int operator()(const std::string &t1,const std::string &t2)
    {
        if(t1.length()>t2.length())
            return 1;
        else if(t1.length()==t2.length())
            return 0;
        else
            return -1;
    }
};

template <class T,class CmpFun=defaultCmpFun<T> >
/**
 * @brief Rising edge falling edge class
 * Can provide different comparison functions to achieve comparison of different data structures
 *
 */
class EdgeTrigger
{
public:
    /**
     * @brief Constructor
     *
     */
    EdgeTrigger(){}
    /**
     * @brief Constructor
     *
     * @param rising_drop Rising edge falling edge mark
     * Rising 1
     * Drop 1<<1
     */
    EdgeTrigger(unsigned int rising_drop,unsigned int rising_anti_shake=1,unsigned int drop_anti_shake=1):_rising_anti_shake(rising_anti_shake),
        _drop_anti_shake(drop_anti_shake)
    {
        _rising_drop=rising_drop;
    }
    /**
     * @brief Determine whether it is a rising or falling edge
     *
     * @param value Data assigned every cycle
     * @return int operator  Rising 1 Drop 1<<1
     */
    int operator()(const T & value)
    {
        int ret=0;
        if((_rising_drop & Rising) && CmpFun()(value,_value)>0 )
        {
            _rising_count++;
            _drop_count=0;
            if(_rising_count>=_rising_anti_shake)
            {
                _value=value;
                _rising_count=0;
                ret=Rising;
                return ret;
            }
        }else if((_rising_drop & Drop) && CmpFun()(value,_value)<0 )
        {
            _rising_count=0;
            _drop_count++;
            if(_drop_count>=_drop_anti_shake)
            {
                _value=value;
                _drop_count=0;
                ret=Drop;
                return ret;
            }
        }else {
            _value=value;
            _drop_count=0;
            _rising_count=0;
            return 0;
        }
        return 0;
    }
    /**
     * @brief Set edge monitor mark
     *
     * @param rising_drop  Rising 1 Drop 1<<1
     */
    void setEdge(int rising_drop)
    {
        _rising_drop=rising_drop;
    }
    void setAntiShakeTime(int time)
    {
        _rising_anti_shake=time;
        _drop_anti_shake=time;
    }
public:
    unsigned int _rising_drop=0; /* edge monitor mark */
    typename std::remove_reference<T>::type _value; /*monitor value */
    unsigned int _rising_count=0;
    unsigned int _drop_count=0;
    unsigned int _rising_anti_shake=1;
    unsigned int _drop_anti_shake=1;


};


class DucoRobotStatus: public rclcpp::Node
{
public :
    DucoRobotStatus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~DucoRobotStatus();

    /*运行*/
    void        run();

protected:
    /*连接机器人*/
    bool connectToRobotController();

    /*定时器信号，用于反馈机器人当前状态*/
    void getRobotStatus();

private:

    /*机器人IP地址*/
    std::string server_host_="127.0.0.1";

    /*机器人当前位置，用于失去连接备份*/
    std::vector<std::vector<double>>  _last_joints_actual_pos;
    std::vector<std::vector<double>>  _last_joints_actual_vel;
    std::vector<std::vector<double>>  _last_joints_actual_acc;
    std::vector<std::vector<double>>  _last_joints_actual_torq;

    /*RPC连接标志*/
    bool                       controller_connected_flag_=false;

    /*机器人开始运动标志*/
    bool                       start_move_;

    /*机器人急停标志*/
    bool                        emergency_stopped_;

    std::vector<std::shared_ptr<DucoRPC::DucoCobot> > duco_robots;
    std::vector<std::shared_ptr<EdgeTrigger<int> > > edge_triggers;

    /*机器人轴名称*/
    std::vector<std::vector<std::string> >     joint_names_;

    /*机器人轴数量*/
    unsigned int                       axis_number_=6;

    /* /duco_driver/rib_status 机器人状态发布节点 */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_states_pub_;

    /*用于发布给MoveGroup机器人关节数据*/
    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr  joint_feedback_pub_;

    /*用于发布给客户端机器人状态*/
    rclcpp::Publisher<duco_msg::msg::DucoRobotState>::SharedPtr  robot_state_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  stop_pub_;

    /*用于反馈给客户端机器人数据*/
    duco_msg::msg::DucoRobotState    robot_state_;





    std::recursive_mutex _mu;

    uint arm_num=1;

    uint _dof =6;

};



}

#endif 
