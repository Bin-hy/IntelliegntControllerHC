[官方文档](https://docs.ducorobots.cn/zh/develop/3.8/ROS2%E5%BC%80%E5%8F%91%E8%AF%B4%E6%98%8E/06%E6%8C%87%E4%BB%A4%E6%8E%A7%E5%88%B6%E6%9C%BA%E6%A2%B0%E8%87%82%E5%8F%8A%E7%9B%B8%E5%85%B3%E6%8E%A5%E5%8F%A3%E8%B0%83%E7%94%A8%E8%AF%B4%E6%98%8E/index.html#id2)

指令控制机械臂及相关接口调用说明
# 方案二：在终端中使用指令对机械臂进行控制
我们可以通过RobotControl，RobotIoControl和RobotMove的指令，分别对机械臂的电源使能，输入输出和移动进行控制。

注意：
```
1. 以下指令中的‘:’后都需要有空格

2. 指令中的参数可以从src/duco_msg/srv中寻找相关的srv文件查看所需参数和参数类型
```

我们使用指令前，需要完成之前章节的配置过程及功能包的编译。

完成之前准备工作后，我们来到src资源包的目录下，打开目录下终端，将所需指令输入终端进行运行。

下面我们将详细介绍可用指令：

## RobotControl
对于机器的电源和使能进行控制。

输入参数：
```
string command: 指令

int8 arm_num: 机械臂编号，设置为0

bool block: 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回，默认为阻塞

string response: 返回值为String
```

command列表：
```
poweron（上电）

enable（使能）

disable（断使能）

poweroff（断电）
```
指令模版：
```
ros2 service call /duco_robot/robot_control duco_msg/srv/RobotControl "{command: "", arm_num: , block: }"
```
指令示例：
```
ros2 service call /duco_robot/robot_control duco_msg/srv/RobotControl "{command: "poweron", arm_num: 0, block: true}"
```

## RobotIoControl
对于通用输出和输入分别进行设置和获取。

输入参数：
```
string command: 指令

int8 arm_num: 机械臂编号，设置为0

int8 type: IO 类型, 0 为 gen io, 1 为 tool io

int8 port: IO 端口，GEN IO 范围 1-16 TOOL IO 范围 0-1

bool value: SetIO 值

bool block: 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回，默认为阻塞

string response: 返回值为String
```
command列表：
```
setIo（设置通用输出）

getIo（获取通用输入）
```
指令模版：
```
ros2 service call /duco_robot/robot_io_control duco_msg/srv/RobotIoControl "{command: "", arm_num: , type: , port: , value: , block: }"
```
指令示例：
```
ros2 service call /duco_robot/robot_io_control duco_msg/srv/RobotIoControl "{command: "setIo", arm_num: 0, type: 0, port: 1, value: true, block: true}"
```

## RobotMove
控制机械臂的移动。

输入参数：
```
string command: 指令

int8 arm_num: 机械臂编号，设置为0

float32[] p: 笛卡尔目标位置

float32[] q: 目标机器人关节角位置，单位(rad)

float32 v: 最大末端线速度，范围[0.01, 5]，单位m/s，当x、y、z均为0时，线速度按比例换算成角速度

float32 a: 最大末端线加速度，范围[0.01, ∞]，单位（m/s2）

float32 r: 轨迹融合半径，单位m，默认值为 0，表示无融合。当数值大于0时表示与下一条运动融合

string tool: 设置使用的工具的名称，默认为当前使用的工具("default")

string wobj: 设置使用的工件坐标系的名称，默认为当前使用的工件坐标系("default")

bool block: 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回，默认为阻塞

string response: 返回值为String
```
command列表：
```
movej（控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态）

movej2(控制机械臂从当前状态，按照各关节相位同步运动的方式移动到目标关节角状态)

movejpose（控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置）

movejpose2（控制机械臂从当前状态，按照各关节相位同步运动的方式移动到末端目标位置）

movel（控制机械臂末端从当前状态按照直线路径移动到目标位姿）

movetcp（控制机械臂沿工具坐标系直线移动一个增量）
```
指令模版：
```
ros2 service call /duco_robot/robot_move duco_msg/srv/RobotMove "{command: "", arm_num: , v: , a: , r: , tool: "", wobj: "", block: , q: [,,,,,], p: [,,,,,]}"
```
指令示例：
```
ros2 service call /duco_robot/robot_move duco_msg/srv/RobotMove "{command: "movejpose",arm_num: 0,v: 1,a: 1,r: 0,tool: "default", wobj: "default", block: true, q: [1,1,1,1,1,1], p: [0.49,0.14,0.44,-1.14,0,-1.57]}"
```

# API说明
## RobotControl
``` cpp
/**
 * @brief 机器人上电
 * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
 */
int32_t power_on(bool block)
// 例子:
power_on(true)

/**
 * @brief 机器人下电
 * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
 */
int32_t power_off(bool block)
// 例子:
power_off(true)

/**
 * @brief 机器人上使能
 * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
 */
int32_t enable(bool block) ;
// 例子:
enable(true)

/**
 * @brief 机器人下使能
 * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
 */
int32_t disable(bool block) ;
// 例子:
disable(true)
```

## RobotIoControl
```cpp
 /**
 * @brief 读取机械臂末端的IO输入口的高低电平, 返回true为高电平, false为低电平
 * @param num 机械臂末端的IO输出口序号, 范围从1-2
 * @return true为高电平, false为低电平
 */
bool get_tool_digital_in(int16_t num) ;
//例子:
get_tool_digital_in(1)

/**
 * @brief 读取控制柜上的用户IO输入口的高低电平, 返回true为高电平, false为低电平
 * @param num 控制柜上的IO输入口序号, 范围从1-16
 * @return true为高电平, false为低电平
 */
bool get_standard_digital_in(int16_t num) ;
// 例子:
get_standard_digital_in(1)

 /**
 * @brief set_tool_digital_out
 * @param num   机械臂末端的IO输出口序号, 范围从1-2
 * @param value true为高电平, false为低电平
 * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
 */
int32_t set_tool_digital_out(int16_t num, bool value, bool block) ;
//例子:
set_tool_digital_out(1, true, true)

/**
 * @brief 该函数可控制控制柜上的IO输出口的高低电平
 * @param num   控制柜上的IO输出口序号, 范围从1-16
 * @param value true为高电平, false为低电平
 * @param block 是否阻塞, 如果为false表示非阻塞指令, 指令会立即返回
 * @return 阻塞执行代表任务结束时状态, 非阻塞执行代表任务的ID
 */
int32_t set_standard_digital_out(int16_t num, bool value, bool block) ;
// 例子:
set_standard_digital_out(1, true, true)
```

## RobotMove
```cpp
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
// 例子：
movej([1,1,1,1,1,1], 1, 1, 0, true)

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
// 例子：
 movej2([1,1,1,1,1,1], 1, 1, 0, true)

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
// 例子：
movej_pose([0.49,0.14,0.44,-1.14,0,-1.57], 1, 1, 0, [1,1,1,1,1,1], "default", "default", true)

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
// 例子：
movej_pose2([0.49,0.14,0.44,-1.14,0,-1.57], 1, 1, 0, [1,1,1,1,1,1], "default", "default", true)

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
// 例子：
movel([0.49,0.14,0.44,-1.14,0,-1.57], 1, 1, 0, [1,1,1,1,1,1])

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
// 例子：
movetcp([0.49,0.14,0.44,-1.14,0,-1.57], 1, 1, 0, "default", "default")
```