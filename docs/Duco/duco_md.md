指令控制机械臂及相关接口调用说明
方案二：在终端中使用指令对机械臂进行控制
我们可以通过RobotControl，RobotIoControl和RobotMove的指令，分别对机械臂的电源使能，输入输出和移动进行控制。

注意：

以下指令中的‘:’后都需要有空格

指令中的参数可以从src/duco_msg/srv中寻找相关的srv文件查看所需参数和参数类型

我们使用指令前，需要完成之前章节的配置过程及功能包的编译。

完成之前准备工作后，我们来到src资源包的目录下，打开目录下终端，将所需指令输入终端进行运行。

下面我们将详细介绍可用指令：

RobotControl
对于机器的电源和使能进行控制。

输入参数：

string command: 指令

int8 arm_num: 机械臂编号，设置为0

bool block: 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回，默认为阻塞

string response: 返回值为String

command列表：

poweron（上电）

enable（使能）

disable（断使能）

poweroff（断电）

指令模版：

ros2 service call /duco_robot/robot_control duco_msg/srv/RobotControl "{command: "", arm_num: , block: }"
Copy code
指令示例：

ros2 service call /duco_robot/robot_control duco_msg/srv/RobotControl "{command: "poweron", arm_num: 0, block: true}"
Copy code
../../_images/RC.jpg
RobotIoControl
对于通用输出和输入分别进行设置和获取。

输入参数：

string command: 指令

int8 arm_num: 机械臂编号，设置为0

int8 type: IO 类型, 0 为 gen io, 1 为 tool io

int8 port: IO 端口，GEN IO 范围 1-16 TOOL IO 范围 0-1

bool value: SetIO 值

bool block: 指令是否阻塞型指令，如果为false表示非阻塞指令，指令会立即返回，默认为阻塞

string response: 返回值为String

command列表：

setIo（设置通用输出）

getIo（获取通用输入）

指令模版：

ros2 service call /duco_robot/robot_io_control duco_msg/srv/RobotIoControl "{command: "", arm_num: , type: , port: , value: , block: }"
Copy code
指令示例：

ros2 service call /duco_robot/robot_io_control duco_msg/srv/RobotIoControl "{command: "setIo", arm_num: 0, type: 0, port: 1, value: true, block: true}"
重试
  
错误原因
Copy code
../../_images/RIC.jpg
RobotMove
控制机械臂的移动。

输入参数：

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

command列表：

movej（控制机械臂从当前状态, 按照关节运动的方式移动到目标关节角状态）

movej2(控制机械臂从当前状态，按照各关节相位同步运动的方式移动到目标关节角状态)

movejpose（控制机械臂从当前状态, 按照关节运动的方式移动到末端目标位置）

movejpose2（控制机械臂从当前状态，按照各关节相位同步运动的方式移动到末端目标位置）

movel（控制机械臂末端从当前状态按照直线路径移动到目标位姿）

movetcp（控制机械臂沿工具坐标系直线移动一个增量）

指令模版：

ros2 service call /duco_robot/robot_move duco_msg/srv/RobotMove "{command: "", arm_num: , v: , a: , r: , tool: "", wobj: "", block: , q: [,,,,,], p: [,,,,,]}"
Copy code
指令示例：

ros2 service call /duco_robot/robot_move duco_msg/srv/RobotMove "{command: "movejpose",arm_num: 0,v: 1,a: 1,r: 0,tool: "default", wobj: "default", block: true, q: [1,1,1,1,1,1], p: [0.49,0.14,0.44,-1.14,0,-1.57]}"
Copy code
