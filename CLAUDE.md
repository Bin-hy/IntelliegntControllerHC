# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Industrial robotic upper-computer system (机器臂手上位机) integrating a DUCO collaborative robot arm, DH116 L-shaped gripper (16 joints, controlled via DUCO controller passthrough), and Orbbec Gemini 330 depth camera. Built on ROS 2 Jazzy with a Qt5 GUI.

## Build Commands

```bash
# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Build entire workspace
cd ros2_ws
colcon build

# Build single package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash
```

## Running the System

```bash
cd ~/IntelliegntControllerHC/ros2_ws
source install/setup.bash
# Specify robot IP and hand communication protocol
ros2 launch ui_app unified.launch.py robot_ip:=192.168.1.10 protocol:=tool_rs485
```

Launch arguments: `robot_ip` (default: 192.168.1.10), `protocol` (default: tool_rs485, options: tool_rs485/rs485/can), `model` (default: gcr5_910).

## First-Time Setup

```bash
sudo ./install.bash   # all-in-one, or run individual scripts:
sudo ./install_ros2.bash        # ROS 2 Jazzy
sudo ./install_rosdep.sh        # rosdep
sudo ./install_orbbec_env.bash  # Orbbec SDK
sudo ./install_lhand.bash       # DH116 hand libraries
sudo ./install_ui_app.bash      # Qt5 + UI deps
```

## Architecture

The system follows a layered architecture with `system_controller` as the central orchestrator:

```
UI App (Qt5)
    ↓ /ui/request_move, /ui/request_control, /ui/request_io  (services)
    ↓ execute_task  (action)
System Controller
    ├─→ /duco_robot/robot_move|robot_control|robot_io_control  → duco_ros_driver → DUCO Arm
    ├─→ lhandpro_interfaces services (21 services)            → lhandpro_service → DUCO Controller → DH116 Hand
    ├─→ /image_saver/save_image                               → vision_server → Orbbec Camera (图片)
    └─→ /image_saver/save_point_cloud                         → vision_server → Orbbec Camera (点云+位置)
```

**Node startup order** (from `unified.launch.py`):
1. `vision_server` — Orbbec camera + image saver
2. `duco_ros_driver` — Robot arm driver (publishes `/duco_cobot/robot_state`)
3. `robot_state_publisher` — TF from combined arm+hand URDF
4. `lhandpro_service` — Hand driver via DUCO passthrough (21 services via `lhandpro_interfaces`)
5. `lhandpro_description` — Hand joint state publisher (`/joint_states`)
6. `system_controller` — Centralized orchestrator (proxies UI↔hardware, hosts `execute_task` action)
7. `ui_app` — Qt5 GUI (delayed 3s)

## Key Packages

| Package | Role |
|---|---|
| `common_msgs` | Shared message/action definitions: `ExecuteTask` action, `DeviceStatus`, `TaskConfig` |
| `lhandpro_interfaces` | 21 service definitions for DH116 hand control |
| `duco_msg` | DUCO arm message/service definitions (`DucoRobotState`, `RobotMove`, etc.) |
| `system_controller` | Central orchestrator; proxies UI requests, manages task state (pause/resume) |
| `ui_app` | Qt5 GUI with 3D robot viz (Qt3D/OpenGL), point cloud view, task management |
| `duco_ros_driver` | DUCO arm hardware driver |
| `lhandpro_service` | DH116 hand driver via DUCO controller passthrough (uses `DucoTransport` + `LHandProLib`) |
| `lhandpro_description` | Maps 6 motor angles → 16 joint positions for TF |
| `duco_support` | URDF models for all DUCO arm variants + combined `duco_gcr5_910_with_dh116_lhand.urdf` |
| `vision_server` | `SaveImage` + `SavePointCloud` services, Orbbec Gemini 330 integration |

## Key Topics & Services

- `/duco_cobot/robot_state` — Robot arm state (joints, cartesian, temp, current)
- `/joint_states` — All joints (arm + hand)
- `/tf`, `/tf_static` — Transform frames
- `/camera/color/image_raw`, `/camera/depth/image_raw` — Camera streams
- `/ui/request_move|request_control|request_io` — UI → system_controller proxy
- `/system/pause_task` (`std_srvs/SetBool`) — Pause/resume task execution
- `execute_task` — `common_msgs/action/ExecuteTask` action server
- `/image_saver/save_point_cloud` — `vision_server/srv/SavePointCloud` captures PointCloud2, returns centroid + PLY file

## Notable Implementation Details

- **Hand passthrough**: `lhandpro_service` communicates with DH116 hand via DUCO controller's RS-485/CAN passthrough API (no longer requires direct EtherCAT or `cap_net_raw`)
- **Protocol selection**: `protocol` parameter controls passthrough mode — `tool_rs485` (end-effector RS-485, default), `rs485` (controller-side RS-485), or `can` (CAN bus)
- **Qt rendering**: If 3D view fails, try `export QT_OPENGL=software`
- **UI executor**: Uses `SingleThreadedExecutor` with a 10ms QTimer calling `spin_some()` to integrate ROS and Qt event loops
- **System Controller state**: `is_busy_` and `is_paused_` are thread-safe atomics; `STATE_ENABLE = 6` is the required robot state for motion
- **URDF selection**: For `model=gcr5_910`, the combined `duco_gcr5_910_with_dh116_lhand.urdf` is used; other models use `duco_<model>.urdf`

## Task Step Types

任务系统支持以下步骤类型（`TaskStep.type`）：

| Type | Description | Device | Key Fields |
|---|---|---|---|
| `arm` | 机械臂关节运动 (movej) | duco | `arm_pos[6]` — 6 个关节角度 (rad) |
| `lhand` / `rhand` | 灵巧手运动 | lhand / rhand | `hand_pos[6]` — 6 个手指位置 (0–100000) |
| `camera` | 相机拍照/点云采集 | orbbec | `camera_type[]` — color, depth, ir_left, ir_right, point_cloud |
| `io` | 控制柜 IO 输出（高低电平） | duco（同 arm） | `io_port` (1=清洗机, 2=吹风机), `io_value` (true=HIGH/false=LOW) |

**IO 不是独立设备类型，属于机械臂 (duco) 设备**。`arm` 和 `io` 步骤共用同一个 duco 设备。UI 中 arm/io 步骤的设备选择优先从任务设备列表取 duco 设备，如果没有则 fallback 到已连接设备扫描。

### 点云采集 (point_cloud)

- 当 `camera_type` 包含 `point_cloud` 时，`system_controller` 调用 `SavePointCloud` 服务
- 订阅 `{cam_ns}/depth/points` (PointCloud2) 话题，采集一帧
- 计算质心位置 (centroid_x, centroid_y, centroid_z)
- 保存 PLY 文件 + 位置 CSV 文件到 `~/.ros/task_photos/{task}/{time}/{round}/{user}/`
- CSV 格式: `centroid_x,centroid_y,centroid_z,num_points`

### IO 控制 (io)

- 通过 `RobotIoControl` 服务调用 `set_standard_digital_out(port, value, block=true)`
- command="setIo", type=0 (控制柜 gen IO), port=io_port
- 端口映射: 1=清洗机, 2=吹风机
- 可在任务中自由组合 IO 开/关步骤，控制点胶机、吹风机等外设

## Debugging

```bash
ros2 topic echo /duco_cobot/robot_state    # Robot arm state
ros2 topic echo /joint_states              # All joint positions
ros2 topic echo /lhandpro_service/now_angles  # Hand motor angles
ros2 service list                           # All available services
```
