#ifndef _DUCOCOBOTRPC
#define _DUCOCOBOTRPC
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <list>

namespace apache {
	namespace thrift {
		namespace transport {
			class TSocket;
			class TTransport;
		}
	}
}
namespace apache {
	namespace thrift {
		namespace protocol {
			class TProtocol;
		}
	}
}
class RPCRobotClient;
namespace DucoRPC {
/**
 * @brief 机器人状态信息
 */
enum StateRobot {
    SR_Start = 0,
    SR_Initialize = 1,
    SR_Logout = 2,
    SR_Login = 3,
    SR_PowerOff = 4,
    SR_Disable = 5,
    SR_Enable = 6
};

/**
 * @brief 程序状态信息
 */
enum StateProgram {
    SP_Stopped = 0,
    SP_Stopping = 1,
    SP_Running = 2,
    SP_Paused = 3,
    SP_Pausing = 4
};


/**
 * @brief 机器人操作模式信息
 */
enum OperationMode {
    kManual = 0,
    kAuto = 1,
    kRemote = 2
};

/**
 * @brief 任务状态信息
 */
enum TaskState {
    ST_Idle = 0,
    ST_Running = 1,
    ST_Paused = 2,
    ST_Stopped = 3,
    ST_Finished = 4,
    ST_Interrupt = 5,
    ST_Error = 6,
    ST_Illegal = 7,
    ST_ParameterMismatch = 8
};

/**
 * @brief 安全控制器状态信息
 */
enum SafetyState {
    SS_INIT = 0,
    SS_WAIT = 2,
    SS_CONFIG = 3,
    SS_POWER_OFF = 4,
    SS_RUN = 5,
    SS_RECOVERY = 6,
    SS_STOP2 = 7,
    SS_STOP1 = 8,
    SS_STOP0 = 9,
    SS_MODEL = 10,
    SS_REDUCE = 12,
    SS_BOOT = 13,
    SS_FAIL = 14,
    SS_UPDATE = 99
};

// 机器人移动过程中对控制柜的IO操作数据
struct OP{
    char time_or_dist_1 = 0;       // 轨迹起始点触发类型, 0:不启用, 1:时间触发, 2:距离触发
    char trig_io_1 = 1;            // 轨迹触发控制柜IO的输出序号, 范围1-16
    bool trig_value_1 = 0;         // 轨迹触发控制柜IO的电平高低, false:低电平, true:高电平
    double trig_time_1 = 0;        // 当time_or_dist_1为1时, 代表轨迹运行多少时间长度触发IO,单位: ms
    double trig_dist_1 = 0;        // 当time_or_dist_1为2时, 代表轨迹运行多少距离长度触发IO,单位: m
    std::string trig_event_1 = ""; // 轨迹触发的用户自定义事件名称
    char time_or_dist_2 = 0;       // 轨迹结束点触发类型, 0:不启用, 1:时间触发, 2:距离触发
    char trig_io_2 = 1;            // 轨迹触发控制柜IO的输出序号, 范围1-16
    bool trig_value_2 = 0;         // 轨迹触发控制柜IO的电平高低, false:低电平, true:高电平
    double trig_time_2 = 0;        // 当time_or_dist_2为1时, 当trig_time_2 >= 0时, 代表轨迹运行剩余多少时间长度触发IO,单位: ms; 当trig_time_2 < 0时, 代表代表轨迹运行结束后多少时间长度触发IO
    double trig_dist_2 = 0;        // 当time_or_dist_2为2时, 当trig_ dist _2 >= 0时, 代表轨迹运行剩余多少距离长度触发IO,单位: m;当trig_ dist _2 < 0时, 代表代表轨迹运行结束后多少距离长度触发IO
    std::string trig_event_2 = ""; // 轨迹触发的用户自定义事件名称
    char time_or_dist_3 = 0;       // 轨迹暂停或停止触发类型，0：不启用，1：时间触发。
    char trig_io_3 = 1;            // 轨迹触发控制柜IO的输出序号, 范围1-16
    bool trig_value_3 = 0;         // 轨迹触发控制柜IO的电平高低, false:低电平, true:高电平
    double trig_time_3 = 0;        // 当time_or_dist_3为1时, 当trig_time_3 >= 0时, 代表任务暂停或停止多少时间触发IO,单位: ms;
    double trig_dist_3 = 0;        // 不适用
    std::string trig_event_3 = ""; // 轨迹触发的用户自定义事件名称
};

// 机器人相关信息
struct RobotStatusList{
    std::vector<double>  jointExpectPosition;   // 目标关节位置
    std::vector<double>  jointExpectVelocity;   // 目标角速度
    std::vector<double>  jointExpectAccelera;   // 目标角加速度
    std::vector<double>  jointActualPosition;   // 实际关节位置
    std::vector<double>  jointActualVelocity;   // 实际角速度
    std::vector<double>  jointActualAccelera;   // 实际角加速度
    std::vector<double>  jointActualCurrent;    // 实际关节电流
    std::vector<double>  jointTemperature;      // 实际关节温度
    std::vector<double>  driverTemperature;     // 未使用
    std::vector<double>  cartExpectPosition;    // 目标末端位姿
    std::vector<double>  cartExpectVelocity;    // 目标末端速度
    std::vector<double>  cartExpectAccelera;    // 目标末端加速度
    std::vector<double>  cartActualPosition;    // 实际末端位姿
    std::vector<double>  cartActualVelocity;    // 实际末端速度
    std::vector<double>  cartActualAccelera;    // 实际末端加速度
    std::vector<bool>    slaveReady;            // 从站状态
    bool    collision;       // 是否发生碰撞
    int8_t  collisionAxis;   // 发生碰撞的关节
    bool    emcStopSignal;   // 未使用
    int8_t  robotState;      // 机器人状态
    int32_t robotError;      // 机器人错误码
    std::vector<double>  jointAuxiliaryPosition;
    std::vector<double>  jointDynamicTorque;
    std::vector<double>  jointGravityTorque;
    std::vector<double>  jointActualTorque;
    std::vector<double>  jointExtraTorque;
};

// IO和寄存器相关信息
struct IOStatusList{
    std::vector<double>  analogCurrentOutputs;  // 模拟电流输出
    std::vector<double>  analogVoltageOutputs;  // 模拟电压输出
    std::vector<double>  analogCurrentInputs;   // 模拟电流输入
    std::vector<double>  analogVoltageInputs;   // 模拟电压输入
    std::vector<bool>    digitalInputs;         // 通用数字输入
    std::vector<bool>    digitalOutputs;        // 通用数字输出
    std::vector<bool>    toolIOIn;              // 工具数字输入
    std::vector<bool>    toolIOOut;             // 工具数字输出
    std::vector<bool>    toolButton;            // 工具末端按键
    std::vector<bool>    funRegisterInputs;     // 功能寄存器输入
    std::vector<bool>    funRegisterOutputs;    // 功能寄存器输出
    std::vector<bool>    boolRegisterInputs;    // bool寄存器输入
    std::vector<bool>    boolRegisterOutputs;   // bool寄存器输出
    std::vector<int16_t> wordRegisterInputs;    // word寄存器输入
    std::vector<int16_t> wordRegisterOutputs;   // word寄存器输出
    std::vector<double>  floatRegisterInputs;   // float寄存器输入
    std::vector<double>  floatRegisterOutputs;  // float寄存器输出
};

// 机器人点动相关信息
struct MoveJogTaskParams
{
    int32_t jog_direction;  // 运动方向, 1:负方向, 0:正方向
    int32_t jog_type;       // 1: 空间点动, 2: 关节点动
    int32_t axis_num;       // 关节索引号, 0--5
    double vel;             // 速度百分比  (未使用)
    int32_t jog_coordinate; // 参考坐标系, 0: 世界, 1: 基座, 2: 工具, 3: 工件
    bool use_step;          // 是否步进模式, true: 步进, false: 连续
    double step_jointValue; // 关节步进距离, 单位: m、rad
    double step_cartvalue;  // 末端步进距离, 单位: m、rad
};

// 可达性检测相关信息
struct ReachabilityParams
{
    bool result;                                  // 可达性确认结果
    std::vector<std::vector<double> > joints_pos; // 可达性检测成功时所对应的所有关节位置
};

// 外部轴反馈信息
struct EAxisInfo
{
    std::string scheme_name;    // 外部轴方案名称
    int32_t status;             // 激活状态
    double pos;                 // 当前位置
};

// 实时数据
struct RealTimeData
{
    std::vector<double>  joint_pos_cmd;           // 关节位置实时指令, 仅在实时控制模式为关节位置时生效
    std::vector<double>  joint_vel_cmd;           // 关节速度实时指令, 仅在实时控制模式为关节速度时生效
    std::vector<double>  joint_torq_cmd;          // 关节力矩控制指令, 预留
    std::vector<double>  cart_pos_tool_wobj_cmd;  // 笛卡尔位置实时指令, 对应当前机器人工具坐标系在工件坐标系下的位置, 仅在实时控制模式未笛卡尔位置时生效
    std::vector<double>  cart_vel_tool_wobj_cmd;  // 笛卡尔速度实时指令, 对应当前机器人工具坐标系在工件坐标系下的速度, 仅在实时控制模式未笛卡尔速度时生效
    std::vector<double>  cart_ft_cmd;             // 笛卡尔力和力矩控制指令, 预留
    bool status;                                  // 控制指令刷新状态, 更新机器人控制指令时给true
};

// 路径点和OP操作
struct PointOP
{
    std::vector<double>  pos;  // 笛卡尔位置
    OP op;                     // 该位置对应的OP操作
};


class DucoCobot
{
public:
	/**
     * @brief 创建机器人远程连接的实例
     * @param ip 机器人IP
     * @param port 机器人端口(7003)
     */
	DucoCobot(std::string ip, unsigned int port);
	/**
     * @brief 连接机器人
     * @return -1:打开失败; 0:打开成功
     */
    int32_t open();
    /**
     * @brief 断开机器人连接
     * @return -1:打开失败; 0:打开成功
     */
    int32_t close();
    /**
     * @brief 机器人上电
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t power_on(bool block) ;
	/**
     * @brief 机器人下电
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t power_off(bool block) ;
    /**
     * @brief 机器人上使能
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t enable(bool block) ;
    /**
     * @brief 机器人下使能
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t disable(bool block) ;
    /**
     * @brief 机器人关机
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t shutdown(bool block) ;
    /**
     * @brief 停止所有任务
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t stop(bool block) ;
    /**
     * @brief 暂停所有任务
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t pause(bool block) ;
    /**
     * @brief 恢复所有暂停的任务
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t resume(bool block) ;
    /**
     * @brief 运行程序脚本
     * @param name  脚本程序名称
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t run_program(const std::string& name, bool block) ;
    /**
     * @brief 设置工具末端相对于法兰面坐标系的偏移, 设置成功后, 
	 *        后续运动函数中的TCP设置为该TCP或者为空时,使用该TCP参数
     * @param name           工具坐标系的名字    
     * @param tool_offset    工具TCP偏移量 [x_off, y_off,z_off,rx,ry,rz], 单位: m, rad
     * @param payload        末端负载质量, 质心, [mass,x_cog,y_cog,z_cog], 单位: kg, m
     * @param inertia_tensor 末端工具惯量矩阵参数, 参数1-6分别对应矩阵xx、xy、xz、yy、yz、zz元素, 单位: kg*m^2
     * @return 返回当前任务结束时的状态
     */
    int32_t set_tool_data(const std::string& name, const std::vector<double> & tool_offset, const std::vector<double> & payload, const std::vector<double>& inertia_tensor) ;
    /**
     * @brief 获取当前设置工具的负载质量及质心位置
     * @param _return 质量单位: kg,质心位置单位: m,[mass,x_cog,y_cog,z_cog]
     */
    void get_tool_load(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂有效的末端工具的偏移量
     * @param _return TCP偏移量信息,单位: m, rad
     */
    void get_tcp_offset(std::vector<double> & _return) ;
    /**
     * @brief 获取当前设置的工件坐标系的值
     * @param _return [x, y, z, rx, ry, rz]工件坐标系相对于基坐标系的位移, 单位: m, rad
     */
    void get_wobj(std::vector<double> & _return) ;
    /**
     * @brief 设置工件坐标系
     * @param name 工件坐标系名称
     * @param wobj 工件坐标系
     * @return 返回当前任务结束时的状态
     */
    int32_t set_wobj(const std::string& name, const std::vector<double> & wobj) ;
    /**
     * @brief 基于当前的工件坐标系设置一个偏移量, 后续的move类脚本的参考工件坐标系上都将添加这个偏移量
     * @param wobj 工件坐标系相对于基坐标系的位移, 单位: m, rad
     * @param active 是否启用
     * @return 返回当前任务结束时的状态
     */
    int32_t set_wobj_offset(const std::vector<double> & wobj, bool active) ;
    /**
     * @brief 计算机械臂的正运动学, 求解给定TCP在给定wobj下的值
     * @param _return         末端姿态列表[x,y,z,rx,ry,rz]
     * @param joints_position 需要计算正解的关节角, 单位: rad
     * @param tool            工具坐标系信息,tcp偏移量[x_off,y_off,z_off,rx,ry,rz], 单位:m, rad, 为空使用当前tcp值
     * @param wobj            工件坐标系相对于基坐标系的位移[x, y, z, rx, ry, rz], 单位:m, rad, 为空使用当前wobj
     */
    void cal_fkine(std::vector<double> & _return, const std::vector<double> & joints_position, const std::vector<double> & tool, const std::vector<double> & wobj) ;
    /**
     * @brief 计算运动学逆解, 在求解过程中, 会选取靠近当前机械臂关节位置的解
     * @param _return   关节位置列表[q1,q2,q3,q4,q5,q6]
     * @param p      需要计算的末端位姿在设置工件坐标系的值,包含当前有效的工具偏移量, 单位:m,rad
     * @param q_near 用于计算逆运动学的参考关节位置,为空使用当前关节值
     * @param tool   工具坐标系信息,tcp偏移量[x_off,y_off,z_off,rx,ry,rz], 单位:m, rad, 为空使用当前tcp值
     * @param wobj   工件坐标系相对于基坐标系的位移[x, y, z, rx, ry, rz], 单位:m, rad, 为空使用当前wobj
     */
    void cal_ikine(std::vector<double> & _return, const std::vector<double> & p, const std::vector<double> & q_near, const std::vector<double> & tool, const std::vector<double> & wobj) ;
    /**
     * @brief 设置通用IO输出的信号类型
     * @param num         控制柜上的IO输出口序号, 范围从1-16
     * @param type        输出的信号类型, 0: 高低电平, 1: 脉冲
     * @param freq        脉冲频率(Hz)
     * @param duty_cycle  脉冲占空比(%)
     * @return 返回当前任务结束时的状态
     */
    int32_t set_digital_output_mode(const int16_t num, const int16_t type, const int32_t freq, const int32_t duty_cycle);
    /**
     * @brief 该函数可控制控制柜上的IO输出口的高低电平
     * @param num   控制柜上的IO输出口序号, 范围从1-16
     * @param value true为高电平, false为低电平
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_standard_digital_out(int16_t num, bool value, bool block) ;
    /**
     * @brief set_tool_digital_out
     * @param num   机械臂末端的IO输出口序号, 范围从1-2
     * @param value true为高电平, false为低电平
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_tool_digital_out(int16_t num, bool value, bool block) ;
    /**
     * @brief 获取控制柜上通用IO输出口的高低电平, 返回true为高电平, false为低电平
     * @param num 控制柜上的IO输出口序号, 范围从1-16
     * @return true为高电平, false为低电平
     */
    bool get_standard_digital_out(int16_t num) ;
    /**
     * @brief 读取控制柜上的用户IO输入口的高低电平, 返回true为高电平, false为低电平
     * @param num 控制柜上的IO输入口序号, 范围从1-16
     * @return true为高电平, false为低电平
     */
    bool get_standard_digital_in(int16_t num) ;
    /**
     * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
     * @param num 机械臂末端的IO输出口序号, 范围从1-2
     * @return true为高电平, false为低电平
     */
    bool get_tool_digital_in(int16_t num) ;
    /**
     * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
     * @param num 机械臂末端的IO输出口序号, 范围从1-2
     * @return true为高电平, false为低电平
     */
    bool get_tool_digital_out(int16_t num) ;
    /**
     * @brief 读取控制柜上的模拟电压输入
     * @param num 控制柜上的模拟电压通道序号, 范围从1-4
     * @return 对应通道的模拟电压值
     */
    double get_standard_analog_voltage_in(int16_t num) ;
    /**
     * @brief 读取机械臂末端的模拟电压输入
     * @param num 机械臂末端的模拟电压通道序号, 范围从1-2
     * @return 对应通道的模拟电压值
     */
    double get_tool_analog_voltage_in(int16_t num) ;
    /**
     * @brief 读取控制柜上的模拟电流输入
     * @param num 控制柜上的模拟电流通道序号, 范围从1-4
     * @return 对应通道的模拟电流值
     */
    double get_standard_analog_current_in(int16_t num) ;
    /**
     * @brief 读取控制柜功能输入IO高低电平, 返回true为高电平, false为低电平
     * @param num 控制柜功能IO输入口序号, 范围从1-8
     * @return true为高电平, false为低电平
     */
    bool get_function_digital_in(int16_t num) ;
    /**
     * @brief 读取控制柜功能输出IO高低电平, 返回true为高电平, false为低电平
     * @param num 控制柜功能IO输出口序号, 范围从1-8
     * @return true为高电平, false为低电平
     */
    bool get_function_digital_out(int16_t num) ;
    /**
     * @brief 读取功能输入寄存器的值
     * @param num 内部寄存器序号, 范围从1-16
     * @return bool寄存器的值
     */
    bool get_function_reg_in(int16_t num);
    /**
     * @brief 读取功能输出寄存器的值
     * @param num 内部寄存器序号, 范围从1-16
     * @return bool寄存器的值
     */
    bool get_function_reg_out(int16_t num);
    /**
     * @brief 485端口读取长度为len的字节数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param len 读取的长度
     */
    void read_raw_data_485(std::vector<int8_t> & _return, int32_t len) ;
    /**
     * @brief 匹配头head和尾tail读取到一帧匹配的数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param head 需要匹配的头数据
     * @param tail 需要匹配的尾数据
     */
    void read_raw_data_485_ht(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;
    /**
     * @brief 匹配头head后读取到长度为len的一帧数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param head 需要匹配的头数据
     * @param len  需要读取的长度
     */
    void read_raw_data_485_h(std::vector<int8_t> & _return, const std::vector<int8_t> & head, int32_t len) ;
    /**
     * @brief 485写原生数据, 将表value中的数据写入485端口
     * @param data 需要写入的数据列表
     * @return true:成功, false:失败
     */
    bool write_raw_data_485(const std::vector<int8_t> & data) ;
    /**
     * @brief 485写原生数据, 将列表value中的数据加上head写入485端口
     * @param data 需要写入的数据列表
     * @param head 需要添加的头
     * @return true:成功, false:失败
     */
    bool write_raw_data_485_h(const std::vector<int8_t> & data, const std::vector<int8_t> & head) ;
    /**
     * @brief 485写原生数据, 将列表value中的数据加上头head和尾tail写入485端口
     * @param data 写入的数据列表
     * @param head 需要添加的头
     * @param tail 需要添加的尾
     * @return true:成功, false:失败
     */
    bool write_raw_data_485_ht(const std::vector<int8_t> & data, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;
    /**
     * @brief 末端485端口读取长度为len的字节数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param len  需要读取的长度
     */
    void tool_read_raw_data_485(std::vector<int8_t> & _return, int32_t len) ;
    /**
     * @brief 末端485匹配头head后读取到长度为len的一帧数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param head 需要匹配的头数据
     * @param len  需要读取的长度
     */
    void tool_read_raw_data_485_h(std::vector<int8_t> & _return, const std::vector<int8_t> & head, int32_t len) ;
    /**
     * @brief 末端485匹配头head和尾tail读取到一帧匹配的数据
     * @param data 读取到的数据, 未读到数据返回空列表
     * @param head 需要匹配的头数据
     * @param tail 需要匹配的尾数据
     */
    void tool_read_raw_data_485_ht(std::vector<int8_t> & _return, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;
    /**
     * @brief 末端485写原生数据, 将data写入485端口
     * @param data 写入的数据列表
     * @return true:成功, false:失败
     */
    bool tool_write_raw_data_485(const std::vector<int8_t> & data) ;
    /**
     * @brief 末端485写原生数据, 将data中的数据加上head写入485端口
     * @param data 写入的数据列表
     * @param head 添加的头
     * @return true:成功, false:失败
     */
    bool tool_write_raw_data_485_h(const std::vector<int8_t> & data, const std::vector<int8_t> & head) ;
    /**
     * @brief 末端485写原生数据, 将value中的数据加上头head和尾tail写入485端口
     * @param data 写入的数据列表
     * @param head 添加的头
     * @param tail 添加的尾
     * @return true:成功, false:失败
     */
    bool tool_write_raw_data_485_ht(const std::vector<int8_t> & data, const std::vector<int8_t> & head, const std::vector<int8_t> & tail) ;
    /**
     * @brief 读取一帧can的字节数据
     * @param data 读取到的数据, 未读到数据返回空列表, 读到数据时, 列表的第一个数据为发送端的can帧id
     */
    void read_raw_data_can(std::vector<int8_t> & _return) ;
    /**
     * @brief can写帧为id, 数据为data的原生数据
     * @param id   数据帧的id
     * @param data 发送的数据列表
     * @return true:成功, false:失败
     */
    bool write_raw_data_can(int32_t id, const std::vector<int8_t> & data) ;
    /**
     * @brief 读取bool寄存器的值
     * @param num 寄存器序号, num范围为1-64
     * @return true or false
     */
    bool read_bool_reg(int16_t num) ;
    /**
     * @brief 读取word寄存器的值
     * @param num 寄存器序号, num范围为1-32
     * @return 寄存器的值
     */
    int32_t read_word_reg(int16_t num) ;
    /**
     * @brief 读取float寄存器的值
     * @param num 寄存器序号, num范围为1-32
     * @return 寄存器的值
     */
    double read_float_reg(int16_t num) ;
    /**
     * @brief 修改bool寄存器的值
     * @param num   寄存器序号, num范围为1-64
     * @param value true or false
     * @return 返回当前任务结束时的状态
     */
    int32_t write_bool_reg(int16_t num, bool value) ;
    /**
     * @brief 修改word寄存器的值
     * @param num   寄存器序号, num范围为1-32
     * @param value 寄存器的值
     * @return 返回当前任务结束时的状态
     */
    int32_t write_word_reg(int16_t num, int32_t value) ;
    /**
     * @brief 修改float寄存器的值
     * @param num   寄存器序号, num范围为1-32
     * @param value 寄存器的值
     * @return 返回当前任务结束时的状态
     */
    int32_t write_float_reg(int16_t num, double value) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态
     * @param joints_list 1-6关节的目标关节角度, 单位: rad
     * @param v 关节角速度, 单位: 系统设定速度的百分比%, 取值范围(0,100]
     * @param a 关节角加速度, 单位: 系统设定加速度的百分比%, 取值范围(0,100]
     * @param r 融合半径, 单位: 系统设定最大融合半径的百分比%, 默认值为 0, 表示无融合, 取值范围[0,50)
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej(const std::vector<double> & joints_list, double v, double a, double r, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态
     * @param joints_list 1-6关节的目标关节角度, 单位: rad
     * @param v 关节角速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 关节角加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej2(const std::vector<double> & joints_list, double v, double a, double r, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 关节角速度, 单位: 系统设定速度的百分比%, 取值范围(0,100]
     * @param a 关节加速度, 单位: 系统设定加速度的百分比%, 取值范围(0,100]
     * @param r 融合半径, 单位: 系统设定最大融合半径的百分比%, 默认值为 0, 表示无融合, 取值范围[0,50)
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej_pose(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 关节角速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej_pose2(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂末端从当前状态按照直线路径移动到目标状态
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 单位: rad
     * @param v 末端速度, 范围[0.00001, 5]，单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movel(const std::vector<double> & p, double v, double a, double r, const std::vector<double> & q_near, const std::string& tool = "default", const std::string& wobj = "default", bool block = true, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂做圆弧运动, 起始点为当前位姿点, 途径p1点, 终点为p2点
     * @param p1 圆弧运动中间点
     * @param p2 圆弧运动结束点
     * @param v  末端速度, 范围[0.00001, 5]，单位: m/s
     * @param a  末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param mode   姿态控制模式  0:姿态与终点保持一致;1:姿态与起点保持一致;2:姿态受圆心约束
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movec(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, int mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂做圆周运动, 起始点为当前位姿点, 途径p1点和p2点
     * @param p1 圆周运动经过点
     * @param p2 圆周运动经过点
     * @param v  末端速度, 范围[0.00001, 5]，单位: m/s
     * @param a  末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param mode   姿态控制模式   1:姿态与终点保持一致;  2:姿态受圆心约束
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t move_circle(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, bool mode, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂沿工具坐标系直线移动一个增量
     * @param pose_offset 工具坐标系下的位姿偏移量
     * @param v 直线移动的速度, 范围[0.00001, 5]，单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a 加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool   设置使用的工具坐标系的名称, 为空时默认为当前使用的工具坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t tcp_move(const std::vector<double> & pose_offset, double v, double a, double r, const std::string& tool, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机器人沿工具坐标系直线移动一个增量, 增量为p1与p2点之间的差, 运动的目标点为:当前点*p1^-1*p2
     * @param p1 工具坐标系下的位姿偏移量计算点1
     * @param p2 工具坐标系下的位姿偏移量计算点2
     * @param v  直线移动的速度, 范围[0.00001, 5]，单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a  加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool  设置使用的工具坐标系的名称, 为空时默认为当前使用的工具坐标系
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t tcp_move_2p(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, bool def_acc = false) ;
    /**
     * @brief 控制机械臂沿工件坐标系直线移动一个增量
     * @param pose_offset 工件坐标系下的位姿偏移量
     * @param v 直线移动的速度, 范围[0.00001, 5]，单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a 加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r 融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t wobj_move(const std::vector<double> & pose_offset, double v, double a, double r, const std::string& wobj, bool block, const OP& op = _op, bool def_acc = false);
    /**
     * @brief 控制机器人沿工件坐标系直线移动一个增量, 增量为p1与p2点之间的差, 运动的目标点为:当前点*p1^-1*p2
     * @param p1 工件坐标系下的位姿偏移量计算点1
     * @param p2 工件坐标系下的位姿偏移量计算点2
     * @param v  直线移动的速度, 范围[0.00001, 5]，单位: m/s, 当x、y、z均为0时, 线速度按比例换算成角速度
     * @param a  加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合.当数值大于0时表示与下一条运动融合
     * @param tool  设置使用的工具坐标系的名称, 为空时默认为当前使用的工具坐标系
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t wobj_move_2p(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double r, const std::string& tool, const std::string& wobj, bool block, const OP& op = _op, bool def_acc = false);
    /**
     * @brief 样条运动函数, 控制机器人按照空间样条进行运动
     * @param pose_list 在设置工件坐标系下的末端位姿列表,最多不超过50个点
     * @param v 末端速度, 范围[0.00001, 5]，单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t spline(const std::vector<std::vector<double> > & pose_list, double v, double a, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, double r = 0, bool def_acc = false) ;
    /**
     * @brief 控制机械臂每个关节按照给定的速度一直运动, 函数执行后会直接运行后续指令.
	 *        运行speedj函数后, 机械臂会持续运动并忽略后续运动指令, 直到接收到speed_stop()函数后停止
     * @param joints_list 每个关节的速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 主导轴的关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param time  运行时间, 到达时间后会停止运动,单位: ms.默认-1表示一直运行
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t speedj(const std::vector<double> & joints_list, double a, int32_t time, bool block) ;
    /**
     * @brief 控制机械臂末端按照给定的速度一直运动, 函数执行后会直接运行后续指令.
	 *        运行speedl函数后, 机械臂会持续运动并忽略后续运动指令, 直到接收到speed_stop()函数后停止
     * @param pose_list 末端速度向量, 范围[0.00001, 5]，线速度单位: m/s,角速度单位: rad/s
     * @param a 末端的线性加速度, 范围[0.00001, ∞]，单位: rad/s^2
     * @param time  运行时间, 到达时间会停止运动, 单位: ms.默认-1表示一直运行
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t speedl(const std::vector<double> & pose_list, double a, int32_t time, bool block) ;
    /**
     * @brief 停止speedj及speedl函数的运动
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t speed_stop(bool block) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态,运动过程中不考虑笛卡尔空间路径
     * @param joints_list 目标关节角度, 单位: rad
     * @param v 最大关节角速度, 范围[0.01*PI/180, 1.25*PI]，单位: m/s
     * @param a 最大关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: m/s^2
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 可缺省, 默认为非阻塞
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数已保证速度的平滑度，否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数以保证加速度的平滑度，否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servoj(const std::vector<double> & joints_list, double v, double a, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动到目标笛卡尔状态,通过关节空间运动
     * @param pose_list 目标工件坐标系下的末端位姿, 单位: m, rad
     * @param v 关节速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: m/s^2
     * @param q_near 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool   设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block  是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 可缺省, 默认为非阻塞
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数已保证速度的平滑度，否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数以保证加速度的平滑度，否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servoj_pose(const std::vector<double> & pose_list, double v, double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 控制机械臂从当前状态, 按照关节运动的方式移动一个增量,通过关节空间运动
     * @param pose_offset 目标工件坐标系下的末端位姿, 单位: m, rad
     * @param v 关节速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 默认值false, 可缺省
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数已保证速度的平滑度，否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数以保证加速度的平滑度，否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servo_tcp(const std::vector<double> & pose_offset, double v, double a, const std::string& tool, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;
    /**
     * @brief 控制机械臂末端从当前状态按照直线路径移动到目标状态
     * @param pose_list 目标工件坐标系下的末端位姿, 单位: m, rad
     * @param v 末端速度, 范围[0.00001, 5]，单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 默认值false, 可缺省
     * @param kp 比例参数, 默认值200, 可缺省, 建议使用默认参数
     * @param kd 微分参数, 默认值25, 可缺省, 建议使用默认参数
     * @param smooth_vel 速度平滑参数, 默认值10, 可缺省, 范围[1-10]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数已保证速度的平滑度，否则可根据实际需要降低参数提高精度
     * @param smooth_acc 加速度平滑参数, 默认值10, 可缺省, 范围[0-1]，当快速连续发送密集离散点位且无法保证点位间隔均匀性时，推荐使用较大参数以保证加速度的平滑度，否则可根据实际需要降低参数提高跟踪精度
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t servol(const std::vector<double> & pose_list, double v, double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block=false, double kp=200, double kd=25, double smooth_vel=10, double smooth_acc=1) ;

    /**
     * @brief 控制机器人进入牵引示教模式
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t teach_mode(bool block) ;
    /**
     * @brief 控制机器人退出牵引示教模式
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t end_teach_mode(bool block) ;
    /**
     * @brief 读取modbus节点的数据, 返回值为double类型
     * @param signal_name modbus节点名
     * @return 节点返回值
     */
    int32_t modbus_read(const std::string& signal_name) ;
    /**
     * @brief 对modbus节点进行写操作
     * @param signal_name modbus节点名
     * @param value 写入的数值, 寄存器节点取值为0-65535内的整数, 线圈节点取值为0或1
     * @return 返回当前任务结束时的状态
     */
    int32_t modbus_write(const std::string& signal_name, int32_t value) ;
    /**
     * @brief 修改modbus节点的刷新频率, 默认频率为10Hz
     * @param signal_name modbus节点名
     * @param frequence 频率值, 取值范围:1~100Hz
     */
    void modbus_set_frequency(const std::string& signal_name, int32_t frequence) ;
    /**
     * @brief 读取机器人最新的错误列表
     * @param _return 错误列表
     */
    void get_last_error(std::vector<std::string> & _return) ;
    /**
     * @brief 根据id查询当前的任务状态
     * @param id 任务的id
     * @return 任务的当前执行状态
     */
    int32_t get_noneblock_taskstate(int32_t id) ;
    /**
     * @brief 插入log日志, 记录运行问题
     * @param message 日志描述
     */
    void log_info(const std::string& message) ;
    /**
     * @brief 在运行过程中产生弹窗, 并暂停当前所有任务
     * @param message 弹窗描述
     */
    void log_error(const std::string& message) ;
    /**
     * @brief 切换机器人到仿真或者真机模式
     * @param sim   true:仿真, false:真机
     * @param block
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t simulation(bool sim, bool block) ;
    /**
     * @brief 设置机器人全局速度
     * @param val 机器人全局速度, 范围[1,100]
     * @return 任务结束时状态
     */
    int32_t speed(double val) ;
    /**
     * @brief 获取当前机器人状态
     * @param _return 机器人状态信息列表, data[0]表示机器人状态, data [1]表示程序状态, 
	 *             data [2]表示安全控制器状态, data [3]表示操作模式
     */
    void get_robot_state(std::vector<int8_t> & _return) ;
    /**
     * @brief 获取当前状态下机械臂末端法兰在基坐标系下的位姿
     * @param _return 末端法兰位置
     */
    void get_flange_pose(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂末端法兰在基坐标系下的速度
     * @param _return 末端法兰速度列表, 单位: m/s,rad/s
     */
    void get_flange_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂末端法兰在基坐标系下的加速度
     * @param _return 末端法兰加速度列表, 单位: m/s^2, rad/s^2
     */
    void get_flange_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂工具末端点在基坐标系下的位姿
     * @param _return 末端位姿
     */
    void get_tcp_pose(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂工具末端点的速度
     * @param _return 末端速度列表, 单位: m/s,rad/s
     */
    void get_tcp_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂工具末端点的加速度
     * @param _return 末端加速度列表, 单位: m/s^2, rad/s^2
     */
    void get_tcp_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 获取当前末端的力矩信息
     * @param _return 末端力矩信息, [Fx,Fy,Fz,Mx,My,Mz],单位: N、N.m
     */
    void get_tcp_force(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节的角度
     * @param _return 1-6轴关节角度列表, 单位: rad
     */
    void get_actual_joints_position(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节的规划角度
     * @param _return 1-6轴目标关节角度列表, 单位: rad
     */
    void get_target_joints_position(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节角速度
     * @param _return 1-6轴关节速度列表, 单位: rad/s
     */
    void get_actual_joints_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节规划角速度
     * @param _return 1-6轴目标关节速度列表, 单位: rad/s
     */
    void get_target_joints_speed(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节角加速度
     * @param _return 1-6轴关节加速度列表, 单位: rad/s^2
     */
    void get_actual_joints_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 取当前状态下机械臂各关节角规划加速度
     * @param _return 1-6轴关节加速度列表, 单位: rad/s^2
     */
    void get_target_joints_acceleration(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节力矩
     * @param _return 1-6轴关节力矩列表, 单位: N.m
     */
    void get_actual_joints_torque(std::vector<double> & _return) ;
    /**
     * @brief 获取当前状态下机械臂各关节目标力矩
     * @param _return 1-6轴关节加速度列表, 单位: rad/s^2
     */
    void get_target_joints_torque(std::vector<double> & _return) ;
    /**
     * @brief 停止轨迹记录
     * @return 任务结束时状态
     */
    int32_t stop_record_track() ;
    /**
     * @brief 开启轨迹记录,当超过允许记录的轨迹长度(针对基于位置记录)或允许记录的时长时(针对基于时间记录),
     *        会自动停止文件记录,并且暂停当前运行的程序.
	 *        文件会记录机器人的各关节弧度值和选定工具、工件坐标系下的笛卡尔空间位姿
     * @param name 轨迹名称
     * @param mode 轨迹类型, mode=0基于位置记录(与上一记录点所有关节偏移总量到达5°时记录新点);
	 *                       mode=1基于时间记录(与上一记录点间隔250ms记录新点)
     * @param tool 工具坐标系名称
     * @param wobj 工件坐标系名称
     * @return 当前任务的id
     */
    int32_t start_record_track(const std::string& name, int32_t mode, const std::string& tool, const std::string& wobj) ;
    /**
     * @brief 设置碰撞检测等级
     * @param value 0:关闭碰撞检测, 1-5:对应设置碰撞检测等级1到等级5
     * @return 任务结束时状态
     */
    int32_t collision_detect(int32_t value) ;
    /**
     * @brief 对记录的轨迹基于关节空间(或基于笛卡尔空间)复现
     * @param name 轨迹名称
     * @param value 轨迹速度, (系统设定速度的百分比%), 取值范围(0,100]
     * @param mode 复现方式, 0:基于关节空间, 1:基于笛卡尔空间
     * @return 任务结束时状态
     */
    int32_t replay(const std::string& name, int32_t value, int32_t mode) ;
    /**
     * @brief 设置抓取负载.可以在程序运行过程中设置机器人当前的负载(质量、质心)
     * @param value 末端工具抓取负载质量, 质心, {mass,x_cog,y_cog,z_cog}, 
	 *              相对于工具坐标系, 质量范围[0, 35], 单位: kg, m
     * @return 任务结束时状态
     */
    int32_t set_load_data(const std::vector<double> & value) ;
    /**
     * @brief 控制机械臂开启末端力控.开启末端力控后所有运动函数除正常运动外,
	 *        会额外基于已配置的末端力控参数进行末端力控运动
     * @return 返回值代表当前任务的id信息
     */
    int32_t fc_start() ;
    /**
     * @brief 控制机械臂退出末端力控
     * @return 当前任务的id信息
     */
    int32_t fc_stop() ;
    /**
     * @brief 修改并配置机器人末端力控参数
     * @param direction 6个笛卡尔空间方向末端力控开关, 开为true, 关为false
     * @param ref_ft 6个笛卡尔空间方向末端力控参考力, 范围[-1000, 1000], X/Y/Z方向单位: N, 
	                 RX/RY/RZ方向单位: Nm, 方向符号参考末端力控参考坐标系方向
     * @param damp  6个笛卡尔空间方向末端力控阻尼, 范围[-10000, 10000], 
	 *              X/Y/Z方向单位: N/(m/s), RX/RY/RZ方向单位: Nm/(°/s)
     * @param max_vel 6个笛卡尔空间方向末端力控最大调整速度, 范围[-5, 5], X/Y/Z方向单位: m/s, 
	 *                范围[-2*PI, 2*PI], RX/RY/RZ方向单位: rad/s
     * @param dead_zone 6个笛卡尔空间方向末端与环境接触力死区, 范围[-1000, 1000],
	 *                     X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param toolname 设置使用的末端力控工具的名称, 默认为当前使用的工具
     * @param wobjname 设置使用的末端力控工件坐标系的名称, 默认为当前使用的工件坐标系
     * @param value 末端力控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
     * @return 当前任务的id信息
     */
    int32_t fc_config(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::vector<double> & damp, const std::vector<double> & max_vel, const std::vector<double> & dead_zone, const std::string& toolname, const std::string& wobjname, int32_t value) ;
    /**
     * @brief 控制机械臂仅产生末端力控运动, 为阻塞型指令
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回, 默认为false
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t fc_move(bool block = false) ;
    /**
     * @brief 控制机械臂在末端力控过程中进行力安全监控
     * @param direction 6个笛卡尔空间方向末端力安全监控开关, 开为true, 关为false
     * @param ref_ft 6个笛卡尔空间方向末端力安全监控参考力, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm, 
	 *               方向符号参考末端力安全监控参考坐标系方向
     * @param toolname 设置使用的末端力安全监控工具的名称
     * @param wobjname 设置使用的末端力安全监控工件坐标系的名称
     * @param type 末端力安全监控参考坐标系选择标志位, 0为参考工具坐标系, 1位参考工件坐标系
     * @param force_property 监控力属性, 0为末端负载力及外力, 1为末端外力(不含负载),可缺省, 默认为0
     * @return 当前任务的id信息
     */
    int32_t fc_guard_act(const std::vector<bool> & direction, const std::vector<double> & ref_ft, const std::string& toolname, const std::string& wobjname, int32_t type, int32_t force_property = 0) ;
    /**
     * @brief 控制机械臂在末端力控过程中禁用力安全监控
     * @return 当前任务的id信息
     */
    int32_t fc_guard_deact() ;
    /**
     * @brief 控制机械臂末端力传感器读数设置为指定值
     * @param direction 6个末端力传感器输出力设置标志位, 需要设置为true, 不需要设置为false
     * @param ref_ft 6个末端力传感器输出力设置目标值, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @return 当前任务的id信息
     */
    int32_t fc_force_set_value(const std::vector<bool> & direction, const std::vector<double> & ref_ft) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定位置判断条件时自动停止当前运动函数并调过后续运动函数, 
	 *        直到fc_stop()函数被执行停止末端力控
     * @param middle 位置判断条件绝对值, X/Y/Z方向单位: m, RX/RY/RZ方向单位: rad
     * @param range  位置判断条件偏移范围大小, X/Y/Z方向单位: m, RX/RY/RZ方向单位: rad
     * @param absolute 绝对/增量条件判断标志位, true为绝对位置判断, false为增量位置判断
     * @param duration 条件满足触发保持时间, 单位: ms
     * @param timeout  条件满足触发超时时间, 单位: ms
     * @return 当前任务的id信息
     */
    int32_t fc_wait_pos(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定速度判断条件时自动停止当前运动函数并跳过后续运动函数, 
	 *        直到fc_stop()函数被执行停止末端力控
     * @param middle 速度判断条件绝对值, X/Y/Z方向范围[-5, 5], 单位: m/s, RX/RY/RZ方向范围[-2*PI, 2*PI], 单位: rad/s
     * @param range  速度判断条件偏移范围大小, X/Y/Z方向单位: m/s, RX/RY/RZ方向单位: rad/s
     * @param absolute 绝对/增量条件判断标志位, true为绝对速度判断, false为增量速度判断
     * @param duration 条件满足触发保持时间, 单位: ms
     * @param timeout  条件满足触发超时时间, 单位: ms
     * @return 当前任务的id信息
     */
    int32_t fc_wait_vel(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中满足指定力判断条件时自动停止当前运动函数并跳过后续运动函数, 
	 *        直到fc_stop()函数被执行停止末端力控
     * @param middle 力判断条件绝对值, 范围[-1000, 1000], X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param range  力判断条件偏移范围大小, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     * @param absolute 绝对/增量条件判断标志位, true为绝对力判断, false为增量力判断
     * @param duration 条件满足触发保持时间, 单位: ms
     * @param timeout  条件满足触发超时时间, 单位: ms
     * @return 当前任务的id信息
     */
    int32_t fc_wait_ft(const std::vector<double> & middle, const std::vector<double> & range, bool absolute, int32_t duration, int32_t timeout) ;
    /**
     * @brief 控制机械臂在执行fc_start()函数后的末端力控过程中位置条件判断、速度条件判断与力条件判断间的逻辑关系.不配置时默认三个条件判断都禁用
     * @param value 三维整形列表, 0代表不启用, 1代表与逻辑, 2代表或逻辑.例如开启位置条件判断, 禁用速度条件判断, 开启力条件判断, 并且位置与力的关系为或, 则输入[1,0,2]
     * @return 当前任务的id信息
     */
    int32_t fc_wait_logic(const std::vector<int32_t> & value) ;
    /**
     * @brief 获取当前机器人末端传感器的反馈读数
     * @param _return 6自由度末端力读数, X/Y/Z方向单位: N, RX/RY/RZ方向单位: Nm
     */
    void fc_get_ft(std::vector<double> & _return) ;
    /**
     * @brief 获取当前机器人末端力控功能启用状态
     * @return 机器人末端力控启用返回true, 未启用返回false
     */
    bool fc_mode_is_active() ;
    /**
     * @brief 控制机械臂开启速度优化功能.开启该功能后, 在满足系统约束前提下,
	 *        机械臂以尽可能高的速度跟踪路径
     * @return 当前任务结束时的状态
     */
    int32_t enable_speed_optimization();
    /**
     * @brief 控制机械臂退出速度优化
     * @return 当前任务结束时的状态
     */
    int32_t disable_speed_optimization();
    /**
     * @brief 被废弃
     * @param name
     * @param value
     * @return
     */
    int32_t set_system_value_bool(const std::string& name, bool value);
    /**
     * @brief 被废弃
     * @param name
     * @param value
     * @return
     */
    int32_t set_system_value_double(const std::string& name, double value);
    /**
     * @brief 被废弃
     * @param name
     * @param value
     * @return
     */
    int32_t set_system_value_str(const std::string& name, const std::string& value);
    /**
     * @brief 被废弃
     * @param name
     * @param value
     * @return
     */
    int32_t set_system_value_list(const std::string& name, const std::vector<double> & value);
    /**
     * @brief 被废弃
     * @param name
     * @return
     */
    bool get_system_value_bool(const std::string& name);
    /**
     * @brief 被废弃
     * @param name
     * @return
     */
    double get_system_value_double(const std::string& name);
    /**
     * @brief 被废弃
     * @param _return
     * @param name
     */
    void get_system_value_str(std::string& _return, const std::string& name);
    /**
     * @brief 被废弃
     * @param _return
     * @param name
     */
    void get_system_value_list(std::vector<double> & _return, const std::string& name);
    /**
     * @brief 将一组points点位信息输入到机器人控制器中的轨迹池
     * @param track 一组points点位信息.每个point以6个double类型数据构成
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackEnqueue(const std::vector<std::vector<double> > & track, bool block);
    /**
     * @brief 将机器人控制器中的轨迹池清空
     * @return 当前任务结束时的状态
     */
    int32_t trackClearQueue();
    /**
     * @brief 获取机器人控制器中的当前轨迹池大小
     * @return 当前轨迹池大小
     */
    int32_t getQueueSize();
    /**
     * @brief 执行时, 机器人的各关节将顺序到达轨迹池中的点位值直到轨迹池中无新的点位.
     *        执行过程中, 主导关节(关节位置变化最大的关节)将以speed与acc规划运动, 其他关节按比例缩放.
     *        注:如果已经开始执行停止规划, 将不再重新获取轨迹池中的数据, 直到完成停止.
     *            停止后如果轨迹池中有新的点位, 将重新执行跟随.为保证运动连续性, 建议至少保证轨迹池中有10个数据
     * @param speed 最大关节速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param acc   最大关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackJointMotion(double speed, double acc, bool block);
    /**
     * @brief 执行时, 机器人的工具末端tool将顺序到达轨迹池中的点位值直到轨迹池中无新的点位.
     *        执行过程中, 工具末端tool将以speed与acc在工件坐标系wobj下规划运动.
     *        注:如果已经开始执行停止规划, 将不再重新获取轨迹池中的数据, 直到完成停止.
                  停止后如果轨迹池中有新的点位, 将重新执行跟随.为保证运动连续性, 建议至少保证轨迹池中有10个数据
     * @param speed 最大末端速度, 范围[0.00001, 5], 单位: m/s
     * @param acc   最大末端加速度, 范围[0.00001, ∞], 单位: m/s^2
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param tool  设置使用的工件坐标系的名称, 为空字符串时默认为当前使用的工件坐标系
     * @param wobj  设置使用的工具的名称,为空字符串时默认为当前使用的工具
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackCartMotion(double speed, double acc, bool block, const std::string& tool, const std::string& wobj);
    /**
     * @brief 确保机器人远程连接断开时, 机器人自动产生一条stop指令以停止当前运动.使用该函数需要单独创建一个线程周期性调用
     * @param time 心跳延时时间, 单位: ms
     */
    void rpc_heartbeat(int32_t time=1000);
    /**
     * @brief 通过参数或者结束点两种设置方式, 在笛卡尔空间做螺旋轨迹运动
     * @param p1   螺旋线中心点位姿
     * @param p2   螺旋线的目标点位姿, 参数设置模式时不参考此参数
     * @param rev  总旋转圈数, rev < 0, 表示顺时针旋转;rev > 0, 表示逆时针旋转
     * @param len  轴向移动距离, 正负号遵循右手定则, 结束点设置模式时不参考此参数, 单位: m
     * @param r    目标点半径, 结束点设置模式时不参考此参数, 单位: m
     * @param mode 螺旋线示教模式, 0:参数设置, 1:结束点设置
     * @param v    末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a    末端加速度, 范围[0.00001, ∞], 单位: m/s^2
     * @param q_near 目标点位置对应的关节角度, 用于确定逆运动学选解, 单位: rad
     * @param tool   设置使用的工具的名称, 为空字符串时默认为当前使用的工具
     * @param wobj   设置使用的工件坐标系的名称, 为空字符串默认为当前使用的工件坐标系
     * @param block  指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op     详见上方Op特殊类型说明,可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t move_spiral(const std::vector<double> & p1, const std::vector<double> & p2, double rev, double len, double r, int32_t mode, double v, double a, const std::vector<double> & q_near, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, bool def_acc = false);
    /**
     * @brief 控制机械臂开启加速度优化功能.开启该功能后, 系统会根据机器人动力学模型、电功率模型计算得到最优加速度大小,
     *        在满足速度约束前提下, 机械臂以尽可能高的加速度进行规划.当速度优化同时打开后, 该函数不起作用
     * @return 当前任务结束时的状态
     */
    int32_t enable_acc_optimization();
    /**
     * @brief 控制机械臂退出加速度优化
     * @return 当前任务结束时的状态
     */
    int32_t disable_acc_optimization();
    /**
     * @brief 设置控制柜上的模拟电压输出
     * @param num   控制柜上的模拟电压通道序号, 范围从1-4
     * @param value 设置的模拟电压值
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_standard_analog_voltage_out(int16_t num, double value, bool block);
    /**
     * @brief 设置控制柜上的模拟电流输出
     * @param num   控制柜上的模拟电流通道序号, 范围从1-4
     * @param value 设置的模拟电流值
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_standard_analog_current_out(int16_t num, double value, bool block);
    /**
     * @brief 设置485的波特率
     * @param value 波特率
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_baudrate_485(int32_t value, bool block);
    /**
     * @brief 设置CAN的波特率
     * @param value 波特率
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return  阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_baudrate_can(int32_t value, bool block);
    /**
     * @brief set_analog_output_mode
     * @param num
     * @param mode
     * @param block
     * @return
     */
    int32_t set_analog_output_mode(int16_t num, int32_t mode, bool block);
    /**
     * @brief 判断机器人是否在运动
     * @return True:机器人在运动, False:机器人没有运动
     */
    bool robotmoving();
    /**
     * @brief 对多线圈进行写操作
     * @param slave_num modbus节点号
     * @param name modbus节点名
     * @param len 需要写入数据的线圈长度
     * @param byte_list 需要写入的数据
     * @return 任务结束时状态
     */
    int32_t modbus_write_multiple_coils(int32_t slave_num, const std::string& name, int32_t len, const std::vector<int8_t> & byte_list);
    /**
     * @brief 对多寄存器进行写操作
     * @param slave_num modbus节点号
     * @param name modbus节点名
     * @param len 需要写入数据的寄存器长度
     * @param word_list 需要写入的数据
     * @return 任务结束时状态
     */
    int32_t modbus_write_multiple_regs(int32_t slave_num, const std::string& name, int32_t len, const std::vector<int16_t> & word_list);
    /**
     * @brief 获取当前工程的路径
     * @param project_path 当前工程路径
     */
    void get_current_project(std::string& project_path);
    /**
     * @brief 获取指定路径下的文件列表
     * @param fileslist 文件列表和类型;0:文件夹;1:文件
     * @param path 当前工程路径
     */
    void get_files_list(std::map<std::string, int32_t> & fileslist, const std::string& path);
    /**
     * @brief 获取当前RPC库的版本号
     * @return 当前RPC库的版本号
     */
    std::string get_version();
    /**
     * @brief 获取机器人当前的位姿等信息
     * @param status 机器人位姿等信息
     */
    void getRobotStatus(RobotStatusList& status);
    /**
     * @brief 获取机器人当前IO和寄存器信息
     * @param status 当前IO和寄存器信息
     */
    void getRobotIOStatus(IOStatusList& status);
    /**
     * @brief 获取末端法兰在工具坐标系和工件坐标系下的位姿
     * @param _return 末端法兰的位姿
     * @param tool 工具坐标系名称, 为空字符串默认为当前使用的坐标系
     * @param wobj 工件坐标系名称, 为空字符串默认为当前使用的坐标系
     */
    void get_tcp_pose_coord(std::vector<double> & _return, const std::string& tool, const std::string& wobj);
    /**
     * @brief 获取机械臂工具末端在工具坐标系下的力矩信息
     * @param _return 末端力矩信息, [Fx,Fy,Fz,Mx,My,Mz],单位: N、N.m
     * @param tool 工具坐标系名称, 默认为当前使用的坐标系
     */
    void get_tcp_force_tool(std::vector<double> & _return, const std::string& tool);
    /**
     * @brief 修改伺服参数
     * @param axis_num 关节索引号
     * @param id       参数的ID号
     * @param value    要设置的值
     * @param qfmt     要设置的qfmt值
     * @param block    指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t set_servo_config(int32_t axis_num, int32_t id, int32_t value,int32_t qfmt, bool block);
    /**
     * @brief 将伺服参数应用到实际控制
     * @param axis_num 关节索引号
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t apply_servo_config(int32_t axis_num, bool block);
    /**
     * @brief 获取电机极对数
     * @param _return 电机极对数
     */
    void get_motor_pole_pair_number(std::vector<int16_t> & _return);
    /**
     * @brief 获取机器人轴电机定子插槽编号
     * @param _return 轴电机定子插槽编号
     */
    void get_motor_stator_slots(std::vector<int16_t> & _return);
    /**
     * @brief 获取机器人轴减速器比率
     * @param _return 轴减速器比率
     */
    void get_axis_ratio(std::vector<int16_t> & _return);
    /**
     * @brief 重置碰撞检测警告
     * @return 任务结束时状态
     */
    int32_t collision_detection_reset();
    /**
     * @brief 修改底层伺服配置文件的参数
     * @param axis_num 关节索引号
     * @param id       参数的ID号
     * @param name     参数的名称
     * @param value    要设置的值
     * @param qfmt     要设置的qfmt值
     * @return 任务结束时状态
     */
    int32_t set_servo_file_params(int32_t axis_num, int32_t id, const std::string& name, double value, double qfmt);
    /**
     * @brief 控制机械臂在关节或者笛卡尔空间做点动
     * @param param Jog运动的相关参数, 参考MoveJogTaskParams
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t move_jog(const MoveJogTaskParams& param, bool block);
    /**
     * @brief 结束机械臂的关节或者笛卡尔Jog
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t stop_manual_move(bool block);
    /**
     * @brief 获取机器人控制器的软件版本号
     * @param _return 机器人控制器的软件版本号
     */
    void get_robot_version(std::string& _return);
    /**
     * @brief 启用或禁用示教器的物理按键
     * @param enable true:启动示教器物理按键, false:禁用示教器物理按键
     * @return 任务结束时状态
     */
    int32_t set_teach_pendant(bool enable);
    /**
     * @brief 获取示教速度的百分比
     * @return 示教速度的百分比
     */
    int32_t get_teach_speed();
    /**
     * @brief 获取全局速度的百分比
     * @return 全局速度的百分比
     */
    int32_t get_global_speed();
    /**
     * @brief 设置示教速度的百分比
     * @param v 示教速度的百分比, 范围[1,100]
     * @return 任务结束时状态
     */
    int32_t set_teach_speed(int32_t v);
    /**
     * @brief 设置复合运动的相关参数
     * @param type 复合运动类型.1:平面三角形轨迹, 2:平面正旋轨迹,
     *                           3:平面圆形轨迹, 4:平面梯形轨迹, 5:平面8字形轨迹
     * @param ref_plane  参考平面, 0:工具XOY, 1:工具XOZ
     * @param fq 频率, 单位: Hz
     * @param amp 振幅, 单位: m
     * @param el_offset 仰角偏移, 单位: m.(参数预留)
     * @param az_offset 方向角偏移, 单位: m.(参数预留)
     * @param up_height 中心隆起高度, 单位: m.(参数预留)
     * @param time 左右停留时间
     * @param path_dwell 主路径同步停留, 可缺省, 默认参数是false
     * @param op_list 二维的OP参数列表, 可缺省, 默认参数是空vector
     * @return 任务结束时状态
     */
    int32_t combine_motion_config(int32_t type, int32_t ref_plane, double fq, double amp, double el_offset, double az_offset, double up_height, const std::vector<int32_t> & time, bool path_dwell = false, const std::vector<OP> & op_list = {});
    /**
     * @brief 开启复合运动
     * @return 任务结束时状态
     */
    int32_t enable_combine_motion();
    /**
     * @brief 结束复合运动
     * @return 任务结束时状态
     */
    int32_t disable_combine_motion();
    /**
     * @brief 开启机械臂奇异点规避功能
     * @return 任务结束时状态
     */
    int32_t enable_singularity_control();
    /**
     * @brief 关闭机械臂奇异点规避功能
     * @return 任务结束时状态
     */
    int32_t disable_singularity_control();
    /**
     * @brief 开启末端震动抑制功能
     * @return 任务结束时状态
     */
    int32_t enable_vibration_control();
    /**
     * @brief 关闭末端震动抑制功能
     * @return 任务结束时状态
     */
    int32_t disable_vibration_control();
    /**
     * @brief 控制外部轴移动
     * @param scheme_name 目标外部轴方案名称
     * @param epose 目标外部轴方案所对应自由度位置(三维), 
	 *              记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
     * @param v     外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op    详见上方Op特殊类型说明(距离触发无效),可缺省参数
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t move_eaxis(const std::string& scheme_name, const std::vector<double> & epose, double v, bool block, const OP& op = _op);
    /**
     * @brief 控制外部轴和机器人执行关节运动
     * @param joints_list 目标关节位置, 单位: rad
     * @param v 关节角速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param rad  融合半径, 单位: m
     * @param scheme_name 目标外部轴方案名称
     * @param epose 目标外部轴方案所对应自由度位置(三维), 
	 *              记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
     * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
     * @param block   指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op      详见上方Op特殊类型说明(距离触发无效), 可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej2_eaxis(const std::vector<double> & joints_list, double v, double a, double rad, const std::string& scheme_name, const std::vector<double> & epose, double eaxis_v, bool block, const OP& op = _op, bool def_acc = false);
    /**
     * @brief 控制外部轴和机器人从当前状态, 按照关节运动的方式移动到末端目标位置
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 范围[-2*PI, 2*PI], 单位: rad
     * @param v 关节角速度, 范围[0.01*PI/180, 1.25*PI]，单位: rad/s
     * @param a 关节加速度, 范围[0.01*PI/180, 12.5*PI]，单位: rad/s^2
     * @param rad 融合半径, 单位: m
     * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param scheme_name 目标外部轴方案名称
     * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
     * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movej2_pose_eaxis(const std::vector<double> & p, double v, double a, double rad, const std::vector<double> & qnear, const std::string& tool, const std::string& wobj, const std::string& scheme_name, const std::vector<double> & epose, double eaxis_v, bool block, const OP& op = _op, bool def_acc = false);
    /**
     * @brief 控制外部轴和机器人从当前状态按照直线路径移动到目标状态
     * @param p 对应末端的位姿, 位置单位: m, 姿态以Rx、Ry、Rz表示, 范围[-2*PI, 2*PI], 单位: rad
     * @param v 末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param rad 融合半径, 单位: m
     * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param scheme_name 目标外部轴方案名称
     * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
     * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movel_eaxis(const std::vector<double> & p, double v, double a, double rad, const std::vector<double> & qnear, const std::string& tool, const std::string& wobj, const std::string& scheme_name, const std::vector<double> & epose, double eaxis_v, bool block, const OP& op = _op, bool def_acc = false);
    /**
     * @brief 控制外部轴和机器人做圆弧运动, 起始点为当前位姿点, 途径p1点, 终点为p2点
     * @param p1 圆弧运动中间点位姿
     * @param p2 圆弧运动结束点位姿
     * @param v 末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param rad 融合半径, 单位: m
     * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param scheme_name 目标外部轴方案名称
     * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
     * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @param mode  姿态控制模式  0:姿态与终点保持一致;1:姿态与起点保持一致;2:姿态受圆心约束, 可缺省参数, 默认是0
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
	 *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t movec_eaxis(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double rad, const std::vector<double> & qnear, const std::string& tool, const std::string& wobj, const std::string& scheme_name, const std::vector<double> & epose, double eaxis_v, bool block, const OP& op = _op, bool def_acc = false, int32_t mode = 0);
    /**
     * @brief 控制机械臂做圆周运动, 起始点为当前位姿点, 途径p1点和p2点
     * @param p1 圆周运动经过点
     * @param p2 圆周运动经过点
     * @param v 末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param rad 融合半径, 单位: m
     * @param mode  姿态控制模式  0:姿态与终点保持一致;1:姿态与起点保持一致;2:姿态受圆心约束
     * @param qnear 目标点附近位置对应的关节角度, 用于确定逆运动学选解, 为空时使用当前位置
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param scheme_name 目标外部轴方案名称
     * @param epose 目标外部轴方案所对应自由度位置(三维), 记录位置自由度及单位根据外部轴方案所设置自由度及外部轴方案类型改变, 单位: rad, m
     * @param eaxis_v 外部轴最大规划速度, 根据对应外部轴方案类型改变, 单位: rad/s, m/s
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 详见上方Op特殊类型说明(距离触发无效),可缺省参数
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态, 若无融合为Finished, 若有融合为Interrupt.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t move_circle_eaxis(const std::vector<double> & p1, const std::vector<double> & p2, double v, double a, double rad, int32_t mode, const std::vector<double> & qnear, const std::string& tool, const std::string& wobj, const std::string& scheme_name, const std::vector<double> & epose, double eaxis_v, bool block, const OP& op = _op, bool def_acc = false);
    /**
     * @brief 可达性检查
     * @param _return 可达性确认结果
     * @param base 基坐标在世界坐标系中的位置
     * @param wobj 工件坐标系在世界坐标系中的位置
     * @param tool 工具坐标系在法兰坐标系中的描述
     * @param ref_pos 机器人关节参考角度
     * @param check_points 需要确认可达性检查的点位列表
     */
    void reach_check(ReachabilityParams& _return, const std::vector<double> & base, const std::vector<double> & wobj, const std::vector<double> & tool, const std::vector<double> & ref_pos, const std::vector<std::vector<double> > & check_points);
    /**
     * @brief 控制外部轴和机器人执行点动
     * @param name 目标外部轴方案名称
     * @param direction 运动方向, -1:负方向, 1:正方向
     * @param vel 速度百分比
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t move_jog_eaxis(const std::string& name, int32_t direction, double vel, bool block);
    /**
     * @brief 获取外部轴当前位置和激活状态信息
     * @param info 当前位置和激活状态信息
     */
    void get_eaxis_info(std::vector<EAxisInfo> & info);
    /**
     * @brief 启动外部轴方案
     * @param scheme_name 外部轴方案名称
     * @return 任务结束时的状态
     */
    int32_t enable_eaxis_scheme(const std::string& scheme_name);
    /**
     * @brief 结束外部轴方案
     * @param scheme_name 外部轴方案名称
     * @return 任务结束时的状态
     */
    int32_t disable_eaxis_scheme(const std::string& scheme_name);
    /**
     * @brief 设置牵引时的参数
     * @param space        牵引类型, 0: 关节空间, 1: 笛卡尔空间
     * @param joint_scale  关节柔顺度, 元素数量必须为机器人关节数量
     * @param cart_scale   笛卡尔柔顺度, 元素数量必须为6个
     * @param coord_type   笛卡尔示教参考坐标系类型, 0: 世界, 1: 基座, 2: 工具, 3: 工件
     * @param direction    牵引方向激活, true: 激活, false: 不激活
     * @return 任务结束时的状态
     */
    int32_t set_hand_teach_parameter(const int32_t space, const std::vector<int32_t> joint_scale, const std::vector<int32_t> cart_scale, const int32_t coord_type, const std::vector<bool> & direction);
    /**
     * @brief 示教器按键的jog类型
     * @param type 1:关节, 2:笛卡尔
     * @return 任务结束时的状态
     */
    int32_t set_pendant_type(const int32_t type);
    /**
     * @brief 设置融合预读取配置
     * @param per 百分比(%)
     * @param num 预读取运动脚本数量
     * @return 任务结束时的状态
     */
    int32_t set_blend_ahead(const int32_t per, int num = 1);
    /**
     * @brief 开启实时控制模式
     * @param mode  实时控制模式, 0：关节位置, 1：关节速度, 2：关节力矩, 3：空间位置, 4：空间速度
     * @param filter_bandwidth 实时控制指令滤波器带宽, 单位Hz, 默认100Hz
     * @param com_lost_time 实时控制通讯数据丢失监控保护时间, 单位s, 默认0.02s
     * @return 任务结束时的状态
     */
    int32_t start_realtime_mode(int32_t mode, double filter_bandwidth, double com_lost_time);
    /**
     * @brief 结束实时控制模式
     * @return 任务结束时的状态
     */
    int32_t end_realtime_mode();
    /**
     * @brief 实时数据入队
     * @param realtime_data 实时数据
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 任务结束时的状态
     */
    int32_t realtime_data_enqueue(const std::vector<RealTimeData> &realtime_data, bool block);
    /**
     * @brief 清空实时数据队列
     * @return 任务结束时的状态
     */
    int32_t clear_realtime_data_queue();
    /**
     * @brief 获取当前实时队列池数据的数量
     * @return 当前实时队列池数据的数量
     */
    int32_t get_realtime_data_queue_size();
    /**
     * @brief 样条运动函数, 控制机器人按照空间样条进行运动, 在运动过程中触发对应点位的OP操作
     * @param pose_list 在设置工件坐标系下的末端位姿和OP列表, 最多不超过50个点
     * @param v 末端速度, 范围[0.00001, 5], 单位: m/s
     * @param a 末端加速度, 范围[0.00001, ∞]，单位: m/s^2
     * @param tool  设置使用的工具的名称, 为空时默认为当前使用的工具
     * @param wobj  设置使用的工件坐标系的名称, 为空时默认为当前使用的工件坐标系
     * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
     * @param op 可缺省参数
     * @param r  融合半径, 单位: m, 默认值为 0, 表示无融合. 当数值大于0时表示与下一条运动融合
     * @param def_acc 是否使用系统默认加速度, false表示使用自定义的加速度值, true表示使用系统自动规划的加速度值, 可缺省, 默认为false
     * @return 当配置为阻塞执行, 返回值代表当前任务结束时的状态.
     *         当配置为非阻塞执行, 返回值代表当前任务的id, 用户可以调用get_noneblock_taskstate(id)函数查询当前任务的执行状态
     */
    int32_t spline_op(const std::vector<PointOP> & pose_list, double v, double a, const std::string& tool, const std::string& wobj, bool block, const OP &op = _op, double r = 0, bool def_acc = false);
    /**
     * @brief 将一组points点位和该点位下的OP信息输入到机器人控制器中的轨迹池, 在运动过程中触发对应点位的OP操作
     * @param track 点位信息和该点位下的OP列表.
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t trackEnqueueOp(const std::vector<PointOP> & track, bool block);
    /**
     * @brief 手自动模式切换
     * @param mode 0:手动模式, 1:自动模式.
     * @return 阻塞执行代表任务结束时状态
    */
    int32_t switch_mode(const int32_t mode);
    /**
     * @brief 获取外接编码器的CNT值
     * @return 外接编码器的CNT值
     */
    int64_t read_encoder_count();
    /**
     * @brief 当前机器人型号原始DH参数
     * @param dh 原始DH参数, 参数顺序a, alpha, d, theta, 单位m/rad
     */
    void get_origin_DH(std::vector<std::vector<double> > & dh);
    /**
     * @brief 当前机器人型号标定补偿后DH参数
     * @param calib_dh 补偿后DH参数, 参数顺序a, alpha, d, theta, 单位m/rad
     */
    void get_calib_DH(std::vector<std::vector<double> > & calib_dh);
    /**
     * @brief 获取机器人系列号, 型号, ext, SN
     * @param type 机器人系列号, 型号, ext, SN
     */
    void get_robot_type(std::vector<std::string> & type);
    /**
     * @brief 获取关节外力矩
     * @param torque 关节外力矩
     */
    void get_ext_torque(std::vector<double> & torque);
    /**
     * @brief 修改机器人DH参数
     * @param params 机器人DH参数 a alpha d beta
     * @return 任务结束时的状态
     */
    int32_t set_kinematic_calibration_params(const std::vector<std::vector<double> > & params);
    /**
     * @brief 获取机器人零位信息
     * @param bias
     */
    void get_pos_bias(std::vector<double> & bias);
    /**
     * @brief 获取机器人pose_list类型全局变量
     * @param value pose_list类型全局变量的值
     * @param name 全局变量的名称
     */
    void get_system_value_lists(std::vector<std::vector<double> > & value, const std::string& name);
    /**
     * @brief 修改SJxx-xx-x.json文件的Fric值和零位
     * @param params Fric值和零位
     * @return 任务结束时的状态
     */
    int32_t set_dynamic_calibration_params(const std::vector<std::vector<double> > & params);
    /**
     * @brief 获取SJxx-xx-x.json文件的Fric值和零位
     * @param params Fric值和零位
     * @return
     */
    void get_dynamic_calibration_params(std::vector<std::vector<double> > & params);
    /**
     * @brief 同步机器人参数到末端
     * @param passwd 密码
     * @return 任务结束时的状态
     */
    int32_t upload_robot_param_to_toolboard(const std::string& passwd);
    /**
     * @brief 设置运动学标定识别码
     * @param passwd 密码
     * @param version 版本号
     * @return 任务结束时的状态
     */
    int32_t set_kinematic_calibration_info(const std::string& passwd, const std::string& version);
    /**
     * @brief 设置动力学标定识别码
     * @param passwd 密码
     * @param version 版本号
     * @return 任务结束时的状态
     */
    int32_t set_dynamic_calibration_info(const std::string& passwd, const std::string& version);
    /**
     * @brief 设置震动标定识别码
     * @param passwd  密码
     * @param version 版本号
     * @return 任务结束时的状态
     */
    int32_t set_vibration_calibration_info(const std::string& passwd, const std::string& version);
    /**
     * @brief 获取机器人轴减速器比率
     * @param rated 轴减速器比率
     */
    void get_axis_motor_rated_current(std::vector<double> & rated);
    /**
     * @brief 获取机器人轴减速器比率
     * @param kt 轴减速器比率
     */
    void get_axis_motor_kt(std::vector<double> & kt);
    /**
     * @brief 中止当前正在执行的运动任务. 如果还提前预读取了下一条或多条运动指令进行融合，此时预读取的指令同样会被中止
     * @param block 指令是否阻塞型指令, 如果为false表示非阻塞指令, 指令会立即返回
     * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
     */
    int32_t abort(const bool block);
    /**
     * @brief 获取震动参数
     * @param params
     */
    void get_vibration_calibration_params(std::vector<std::vector<double> > & params);
    /**
     * @brief 保存运动学参数到文件
     * @param passwd 密码
     * @return 任务结束时的状态
     */
    int32_t save_kinematic_calibration_params(const std::string& passwd);
    /**
     * @brief 保存动力学参数到文件
     * @param passwd 密码
     * @return 任务结束时的状态
     */
    int32_t save_dynamic_calibration_params(const std::string& passwd);
    /**
     * @brief 获取机器人仿真状态
     * @return true: 仿真, false: 真机
     */
    bool get_simulation_state();
    /**
     * @brief save_stiffness_calibration_params
     * @param passwd
     * @return
     */
    int32_t save_stiffness_calibration_params(const std::string& passwd);
    /**
     * @brief get_stiffness_calibration_params
     * @param _return
     */
    void get_stiffness_calibration_params(std::vector<std::vector<double> > & _return);
    /**
     * @brief set_stiffness_calibration_params
     * @param params
     * @return
     */
    int32_t set_stiffness_calibration_params(const std::vector<std::vector<double> > & params);
    /**
     * @brief 获取机械臂各关节默认速度加速度
     * @param _return 关节默认速度加速度列表, 单位: rad/s, rad/s^2
     */
    void get_joint_motion_params(std::vector<std::vector<double> > & _return);

private:
    std::shared_ptr<apache::thrift::transport::TSocket> _socket;        // SDK内部使用, 开发人员无需关心
    std::shared_ptr<apache::thrift::transport::TTransport> _transport;  // SDK内部使用, 开发人员无需关心
    std::shared_ptr<apache::thrift::protocol::TProtocol> _protocol;     // SDK内部使用, 开发人员无需关心
    std::shared_ptr<RPCRobotClient> _robot;                             // SDK内部使用, 开发人员无需关心
    std::string _serverIP;                                              // 机器人IP
    bool _idle;                                                         // SDK内部使用, 开发人员无需关心

    static struct OP _op;
};
}
#endif // _DUCOCOBOTRPC
