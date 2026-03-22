# UDEXREAL HandDriver JSON C++/Python SDK关节角度使用手册

# 1. 概述

## 1.1 功能

本SDK基于JSON传输格式获取手部各关节角度数据，支持Windows和Linux两种环境。

## 1.3 HandDriver端设置

- 勾选“数据发送”，即打开数据发送开关。
- 如需传输IMU数据（仅支持带IMU的手套），点选IMU选项，否则不传输IMU数据。
- 发送帧率支持120Hz、90Hz、60Hz三种帧率，默认为120Hz。
- 点击目标地址中的“+”图标，填入接收端的IP地址和端口号，方可进行数据发送（本地发送填写127.0.0.1）。可添加多个目标地址。
- 端口号默认为5555，如有修改，请保持与SDK端一致。
- 设置完成后，点击“应用”按钮保存设置。

### 1.3.1 HandDriver2.2.2之后

传输格式请选择JSON，使用本SDK时数据内容请选择“欧拉角”，请勿勾选旧插件。

# 2. 坐标系及数据定义

## 2.2 数据格式

发送的数据格式如下图所示，包括角色名、标定状态、手指节旋转角度、IMU四元数（当手套带有IMU并勾选IMU发送时才有）、控制器（摇杆按键）等数据。

```json
{
    "Udhand": { //对应HandDriver中的角色名称
        "Bones": [
            {
                "Name": "Head", 
                "Parent": 0,
                "Location": [ 0,0,0 ],
                "Rotation": [ 0,0,0 ],
                "Scale": [ 0,0,0 ]
            }
        ],
        "Parameter": [
                { "Name": "L_CalibrationStatus", "Value": 3 },//左手标定状态 -1:未标定，0:握拳标定中，1:五指并拢标定中，2:五指分开标定中，3:标定完成     
                { "Name": "l0", "Value": 0 },//左手 拇指第三关节俯仰角                        
                { "Name": "l1", "Value": 0 },//左手 拇指第二关节俯仰角                        
                { "Name": "l2", "Value": 0 },//左手 拇指第一关节俯仰角                        
                { "Name": "l3", "Value": 0 },//左手 拇指第一关节偏航角                        
                { "Name": "l4", "Value": 0 },//左手 食指第三关节俯仰角                        
                { "Name": "l5", "Value": 0 },//左手 食指第二关节俯仰角
                { "Name": "l6", "Value": 0 },//左手 食指第一关节俯仰角
                { "Name": "l7", "Value": 0 },//左手 食指第一关节偏航角
                { "Name": "l8", "Value": 0 },//左手 中指第三关节俯仰角
                { "Name": "l9", "Value": 0 },//左手 中指第二关节俯仰角
                { "Name": "l10", "Value": 0 },//左手 中指第一关节俯仰角
                { "Name": "l11", "Value": 0 },//左手 中指第一关节偏航角
                { "Name": "l12", "Value": 0 },//左手 无名指第三关节俯仰角
                { "Name": "l13", "Value": 0 },//左手 无名指第二关节俯仰角
                { "Name": "l14", "Value": 0 },//左手 无名指第一关节俯仰角
                { "Name": "l15", "Value": 0 },//左手 无名指第一关节偏航角
                { "Name": "l16", "Value": 0 },//左手 小指第三关节俯仰角
                { "Name": "l17", "Value": 0 },//左手 小指第二关节俯仰角
                { "Name": "l18", "Value": 0 },//左手 小指第一关节俯仰角
                { "Name": "l19", "Value": 0 },//左手 小指第一关节偏航角
                { "Name": "l20", "Value": 0 },//左手 拇指第一关节旋转角
                { "Name": "l21", "Value": 0 },//左手 食指第一关节旋转角
                { "Name": "l22", "Value": 0 },//左手 小指第一关节旋转角
                { "Name": "l23", "Value": -1 },//左手 手势识别预留位
                { "Name": "l24", "Value": -1 },//左手 IMU四元数W
                { "Name": "l25", "Value": -1 },//左手 IMU四元数X
                { "Name": "l26", "Value": -1 },//左手 IMU四元数Y
                { "Name": "l27", "Value": -1 },//左手 IMU四元数Z
                { "Name": "l_joyX", "Value": 0 },
                { "Name": "l_joyY", "Value": 0 },
                { "Name": "l_aButton", "Value": false },
                { "Name": "l_bButton", "Value": false },
                { "Name": "l_joyButton", "Value": false },
                { "Name": "l_menu", "Value": false },
                { "Name": "R_CalibrationStatus", "Value": 3 },//右手标定状态 -1:未标定，0:握拳标定中，1:五指并拢标定中，2:五指分开标定中，3:标定完成
                { "Name": "r0", "Value": 0 },//右手 拇指第三关节俯仰角
                { "Name": "r1", "Value": 0 },//右手 拇指第二关节俯仰角
                { "Name": "r2", "Value": 0 },//右手 拇指第一关节俯仰角
                { "Name": "r3", "Value": 0 },//右手 拇指第一关节偏航角
                { "Name": "r4", "Value": 0 },//右手 食指第三关节俯仰角
                { "Name": "r5", "Value": 0 },//右手 食指第二关节俯仰角
                { "Name": "r6", "Value": 0 },//右手 食指第一关节俯仰角
                { "Name": "r7", "Value": 0 },//右手 食指第一关节偏航角
                { "Name": "r8", "Value": 0 },//右手 中指第三关节俯仰角
                { "Name": "r9", "Value": 0 },//右手 中指第二关节俯仰角
                { "Name": "r10", "Value": 0 },//右手 中指第一关节俯仰角
                { "Name": "r11", "Value": 0 },//右手 中指第一关节偏航角
                { "Name": "r12", "Value": 0 },//右手 无名指第三关节俯仰角
                { "Name": "r13", "Value": 0 },//右手 无名指第二关节俯仰角
                { "Name": "r14", "Value": 0 },//右手 无名指第一关节俯仰角
                { "Name": "r15", "Value": 0 },//右手 无名指第一关节偏航角
                { "Name": "r16", "Value": 0 },//右手 小指第三关节俯仰角
                { "Name": "r17", "Value": 0 },//右手 小指第二关节俯仰角
                { "Name": "r18", "Value": 0 },//右手 小指第一关节俯仰角
                { "Name": "r19", "Value": 0 },//右手 小指第一关节偏航角
                { "Name": "r20", "Value": 0 },//右手 拇指第一关节旋转角
                { "Name": "r21", "Value": 0 },//右手 食指第一关节旋转角
                { "Name": "r22", "Value": 0 },//右手 小指第一关节旋转角
                { "Name": "r23", "Value": -1 },//右手 手势识别预留位
                { "Name": "r24", "Value": -1 },//右手 IMU四元数W
                { "Name": "r25", "Value": -1 },//右手 IMU四元数X
                { "Name": "r26", "Value": -1 },//右手 IMU四元数Y
                { "Name": "r27", "Value": -1 },//右手 IMU四元数Z
                { "Name": "r_joyX", "Value": 0 },
                { "Name": "r_joyY", "Value": 0 },
                { "Name": "r_aButton", "Value": false },
                { "Name": "r_bButton", "Value": false },
                { "Name": "r_joyButton", "Value": false },
                { "Name": "r_menu", "Value": false }
            ]
    }
}
```

## 2.3 接收的角度说明

关于接收的角度值，可参考HandDriver端数据页，此页数据与SDK接收数据一致：

- 正负一致
- 具体数值在HandDriver中显示时以整数表达
- 详细说明请查看这里：UDEXREAL HandDriver手部模型关节说明

# 4. C++ SDK - Linux版

## 4.1 文件目录

Linux Cpp SDK文件夹内包含：include文件夹（包含Json解析所需头文件），lib文件夹（包含libjsoncpp.so, libjsoncpp.so.27 必要Json动态库），UDEServer.h（SDK头文件），UDEServer.cxx（方法实现）main.cxx（运用实例），build.sh（示例build文件），CMakeLists.txt，ReadMe.txt

## 4.2 数据结构定义

与前文相同

## 4.3 数据调用

需要先引用UDEServer.h的头文件，并完成服务端的初始化。
随后可以获取接收到的角色名列表方法如下：

```cpp
#include "UDEServer.h"

//创建新SDK实例
UDEGloveSDK sdk;
//初始化方法
sdk.Initialize();

//开启服务器监听
sdk.StartListening();

//调用GetRoleNameList()方法取得接收到角色名列表
vector<string> list = sdk.GetRoleNameList();
```

服务端的默认端口号为5555，可以通过方法获取当前端口号或设置端口：

```cpp
int port = sdk.GetPortNum();

int newPort = 1234;
sdk.SetPortNum(newPort);
```

服务器监听中可以关闭数据接收或是获取当前状态：

```cpp
//服务器状态枚举参数
enum ServerStatus
{
    NO_INIT, 
    READY, 
    IN_LISTENING, 
    END
};

int status = sdk.GetStatus()

//关闭服务器监听
sdk.EndListening();
```

根据指定角色名，能够获取所有指关节数据，类型为vector<Vector3Float>，根据GloveDataHeaders内的索引次序对应取得手指关节名称以及欧拉角数组。同时也可以获取控制器数据，根据ControllerHeaders内的索引次序取得类型为float的数组，对应为控制器参数。

```cpp
//Vector3Float 结构体
struct Vector3Float
{
    float x;
    float y;
    float z;

    Vector3Float(float X, float Y, float Z)
    {
        this->x = X;
        this->y = Y;
        this->z = Z;
    }
};

//指关节返回值对应的名称索引
string GloveDataHeaders[30] = 
{
    "LeftThumb1", 
    "LeftThumb2", 
    "LeftThumb3", 
    "LeftIndex1", 
    "LeftIndex2", 
    "LeftIndex3", 
    "LeftMiddle1",
    "LeftMiddle2",
    "LeftMiddle3",
    "LeftRing1",
    "LeftRing2",
    "LeftRing3",
    "LeftPinky1", 
    "LeftPinky2", 
    "LeftPinky3", 
    "RightThumb1", 
    "RightThumb2", 
    "RightThumb3", 
    "RightIndex1", 
    "RightIndex2", 
    "RightIndex3",
    "RightMiddle1",
    "RightMiddle2",
    "RightMiddle3",
    "RightRing1",
    "RightRing2",
    "RightRing3",
    "RightPinky1", 
    "RightPinky2", 
    "RightPinky3", 
};

//控制器返回值对应的名称索引
string ControllerHeaders[12] = 
{
    "Left Joy X",
    "Left Joy Y",
    "Left A Button",
    "Left B Button",
    "Left Joy Button",
    "Left Menu Button",
    "Right Joy X",
    "Right Joy Y",
    "Right A Button",
    "Right B Button",
    "Right Joy Button",
    "Right Menu Button"
};

vector<string> list = sdk.GetRoleNameList();
if(list.size() > 0)
{
    //默认获取第一位角色的数据
    auto FingerData = sdk.GetVecFingerData(list[0]);
    
    //控制器参数
    auto ControllerData = sdk.GetVecControllerData(list[0]);
}
```

取得的对应条目示例如下：

```cpp
{
    "LeftThumb1", {0, 0, 0};
    "LeftThumb2", {0, 0, 0};
    "LeftThumb3", {0, 0, 0};
    ...
    "RightThumb1", {0, 0, 0};
    "RightThumb2", {0, 0, 0};
    "RightThumb3", {0, 0, 0};
    ...
    "Left Joy X", 0;
    "Left Joy Y", 0;
    "Left A Button", 0;
    "Left B Button", 0;
    ...
}
```

指关节名称规则为Left/Right（左手/右手）+ Thumb/Index/Middle/Ring/Pinky（拇指/食指/中指/无名指/小指）+ 1/2/3（第一关节/第二关节/第三关节），角度数组顺序为{俯仰角，偏航角，旋转角}。
控制器数据中Joy X和Joy Y对应为摇杆参数，范围是（-1，1）的浮点数，其余所有按钮的参数为了统一也定为浮点数0或1，对应为未触发或触发。

## 4.4 接口函数定义

- 初始化

```cpp
int Initialize();
    函数说明：Socket初始化，包含创建服务器所需声明及调用。
    返回值说明：若成功则返回0，失败返回-1。
```

- 设置端口号

```cpp
void SetPortNum(int port);
    函数说明：设置服务器监听端口号。
    参数说明：int整数型端口号
```

- 获取端口号

```cpp
int GetPortNum();
```

函数说明：获取服务器监听端口号。
返回值说明：int整数型端口号

- 开启服务器监听

```cpp
void StartListening();
```

函数说明：开启服务器监听方法，能在循环中获取参数。

- 关闭服务器监听

```cpp
void EndListening();
```

```
函数说明：关闭服务器监听方法。
```

- 获取服务器状态

```cpp
ServerStatus GetStatus();
    函数说明：取得当前服务器状态。
    返回值说明：ServerStatus枚举参数，0对应 NO_INIT 未初始化，1对应 READY 准备就绪，2对应 IN_LISTENING 正在监听，3对应 END 监听终止。
```

- 获取角色名列表

```cpp
vector<string> GetRoleNameList();
    函数说明：取得接收到的所有角色名的字符串列表。
    返回值说明：名称字符串数组
```

- 获取指关节数据

```cpp
vector<Vector3Float> GetVecFingerData(string RoleName);
    函数说明：取得指定角色的指关节数据。
    参数说明：字符串类型角色名
    返回值说明：与GloveDataHeaders中字符索引相同排序的欧拉角数组
```

- 获取控制器数据

```cpp
float* GetVecControllerData(string RoleName);
```

函数说明：取得指定角色的控制器数据。
参数说明：字符串类型角色名
返回值说明：与ControllerHeaders中字符索引相同排序的float数组

## 4.5 范例

main.cxx中提供了一个在主方法内调用SDK接口接收并打印数据的范例，可以通过运行build.sh后生成可执行文件。

1. 启动终端，进入SDK文件夹根目录，运行如下build命令：

```bash
$ sudo sh ./build.sh
```

1. 由此能生成可执行范例ServerSample，在同一目录下，用如下命令运行服务器范例：

```bash
$ ./ServerSample
```

