# Intelligent Controller HC 软件交付说明书

## 1. 项目概述

**项目名称**：Intelligent Controller HC 智能协作机器人控制系统 (V0.1.0)
**项目背景**：在复杂作业场景下，需要对机械臂、灵巧手、3D视觉等多种异构设备进行统一的调度、监控和可视化控制。传统单设备的控制软件难以实现设备间的联动与任务级编排。
**系统目标**：基于 ROS 2 和 Qt5，打造一套安全、稳定、易操作的综合性机器人上位机软件平台，实现从视觉感知、任务编排到机械臂及灵巧手底层执行的全链路控制。
**功能概述**：
- 支持多可 (DUCO) 全系列单/双臂系统的姿态同步与运动控制。
- 支持赛雷 DH116 灵巧手控制与状态反馈。
- 支持 Orbbec 3D 深度相机图像获取及 3D 点云渲染可视化。
- 具备完善的用户权限管理、系统状态监控与日志记录。
- 提供基于 Action Server 的复杂作业序列 (Sequence Demo) 编排能力。

## 2. 系统架构

**前端交互层 (UI App)**
- **技术栈**：C++ / Qt 5 / Qt3D / OpenGL
- **核心模块**：`ui_app`（含 `app_window`, `point_cloud_widget`, `robot_viz_widget`, `task_widget` 等）

**系统控制层 (Controller)**
- **技术栈**：C++ / ROS 2 Jazzy / rclcpp_action
- **核心模块**：
  - `system_controller`：中央调度与任务序列执行枢纽
  - `safe_duco_status`：独立的安全监控与防碰撞检测
  - `logger_node`：全系统日志收集

**感知与执行驱动层 (Hardware Drivers)**
- **视觉**：`OrbbecSDK_ROS2`, `vision_server`
- **手臂**：`duco_ros_driver`, `robot_control`, `duco_support` (及各型号 `moveit_config`)
- **末端**：`lhandpro_service`, `lhandpro_description`
- **外设**：`glove_node` (数据手套)

## 3. 功能说明

### 3.1 机械臂控制模块 (DUCO Arm)
- **状态监控**：实时获取并显示各关节角度、笛卡尔坐标、运行状态。
- **运动控制**：支持关节空间 (Joint) 和笛卡尔空间 (Cartesian) 的点动及连续轨迹运动控制。
- **3D 可视化**：结合 URDF 模型，在软件界面内实时同步并渲染机器人的 3D 姿态。

### 3.2 灵巧手控制模块 (LHand Pro)
- **多协议支持**：兼容 `tool_rs485`, `rs485`, `can` 三种通信方式。
- **精细控制**：支持各手指关节独立的位置和速度控制。
- **一键复位**：支持灵巧手回零 (Home) 和指定手势宏调用。

### 3.3 视觉与感知模块 (Vision System)
- **多模态显示**：支持彩色 (Color)、深度 (Depth)、红外 (IR) 图像的实时拉流与展示。
- **3D 点云**：支持解析 `sensor_msgs::msg::PointCloud2` 并在 UI 内通过 Qt3D 渲染点云。
- **图像保存**：提供一键保存图像及点云数据的服务接口。

### 3.4 任务编排与系统管理
- **序列任务**：支持下发多步骤组合任务（如：移动到A点 -> 灵巧手闭合 -> 拍照 -> 移动到B点）。
- **用户权限**：内置多级权限管理 (Admin/User)，支持超时自动登出。
- **任务记录**：记录并回溯任务执行日志。

## 4. 部署说明

### 4.1 环境要求
- **硬件**：x86_64 架构 PC 或工控机，至少 8GB 内存，具备以太网及 USB 3.0 接口。
- **操作系统**：Ubuntu 24.04 (Noble)
- **基础依赖**：ROS 2 Jazzy, Qt 5 (含 `qt3d5-dev`, `libqt5-opengl-dev` 等)

### 4.2 安装步骤
建议将代码克隆至 `~/IntelliegntControllerHC` 目录。
执行一键部署脚本：
```bash
cd ~/IntelliegntControllerHC
chmod +x *.bash *.sh
# 执行主安装脚本（自动包含 ROS2、依赖、驱动的安装）
sudo ./install.bash
```

### 4.3 编译工作空间
```bash
cd ~/IntelliegntControllerHC/ros2_ws
source /opt/ros/jazzy/setup.bash
# 使用 colcon 编译所有模块
colcon build --symlink-install
```

### 4.4 启动方式
```bash
cd ~/IntelliegntControllerHC/ros2_ws
source install/setup.bash

# 使用集成 launch 文件一键拉起所有节点及 UI
ros2 launch ui_app unified.launch.py \
    robot_ip:=192.168.1.10 \
    model:=gcr5_910 \
    protocol:=tool_rs485
```

## 5. 使用说明 (简易操作流程)
1. **设备通电与连接**：确保机械臂控制柜、相机、灵巧手均已上电，且与主控机网络/串口连接正常。
2. **启动软件**：按 `4.4` 章节命令启动后，软件主界面会自动弹出。
3. **用户登录**：在弹出的登录窗口输入账号密码（默认管理员账号及密码请参阅 `auth_manager.cpp` 初始设定）。
4. **状态确认**：
   - 观察 3D 模型区，确认模型已加载。
   - 检查界面右上角/左侧状态栏，确认机械臂和灵巧手连接状态正常。
5. **控制操作**：
   - 可在“手动操作”面板中拖动滑块或输入数值，对机械臂及灵巧手进行点动测试。
   - 切换至“视觉面板”，选择相应的 Topic 即可实时查看相机画面与点云。
6. **任务执行**：在“任务编排”面板，加载预设好的任务序列，点击“执行”即可自动完成连贯动作。

## 6. 核心接口说明

系统基于 ROS 2 话题 (Topic)、服务 (Service) 及动作 (Action) 进行通信。

### 6.1 核心 Topic
| Topic 名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/duco_cobot/robot_state` | `duco_msg/msg/DucoRobotState` | 机械臂实时位姿及状态反馈 |
| `/device_status` | `common_msgs/msg/DeviceStatus` | 系统内各硬件设备健康状态 |
| `/*/color/image_raw` | `sensor_msgs/msg/Image` | 相机彩色图像流 |
| `/*/depth/color/points` | `sensor_msgs/msg/PointCloud2` | 3D点云数据流 |

### 6.2 核心 Service
| Service 名称 | 服务类型 | 说明 |
| :--- | :--- | :--- |
| `/ui/request_move` | `duco_msg/srv/RobotMove` | 机械臂移动控制指令 |
| `/lhandpro_service/set_position` | `lhandpro_interfaces/srv/SetPosition` | 灵巧手单指位置控制 |
| `/lhandpro_service/move_motors` | `lhandpro_interfaces/srv/MoveMotors` | 灵巧手预设动作调用 |
| `/image_saver/save_image` | `vision_server/srv/SaveImage` | 触发保存当前图像/点云 |

### 6.3 核心 Action
| Action 名称 | 动作类型 | 说明 |
| :--- | :--- | :--- |
| `/execute_task` | `common_msgs/action/ExecuteTask` | 下发包含多步骤的复杂序列任务 |

## 7. 交付清单

1. **软件源代码**：`~/IntelliegntControllerHC/ros2_ws/src/` 目录下所有功能包源码。
2. **自动化安装脚本**：`install.bash` 及子系统安装脚本集合。
3. **配置文件与模型文件**：
   - URDF 模型：位于 `duco_support`, `DH116_*` 包。
   - MoveIt 配置：位于各 `*_moveit_config` 包。
4. **说明文档**：本《软件交付说明书》。

## 8. 验收标准

1. **编译安装验收**：在符合要求的纯净 Ubuntu 24.04 系统上，运行 `install.bash` 后可一次性编译通过，无致命报错。
2. **通信链路验收**：软件启动后，能稳定获取机械臂状态数据，UI 界面的 3D 模型与实体机器人姿态保持同步，延迟 < 500ms。
3. **控制功能验收**：通过 UI 下发移动指令或灵巧手抓取指令，实体硬件能够准确响应。
4. **视觉功能验收**：UI 能够清晰显示彩色、深度图像，且点云画面无明显撕裂、卡顿。
5. **任务流程验收**：能够成功执行包含“运动+抓取+视觉”的综合 Action 序列，并在执行完毕后正确反馈成功状态。