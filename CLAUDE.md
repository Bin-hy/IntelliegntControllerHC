# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Industrial robotic upper-computer system (机器臂手上位机) integrating a DUCO collaborative robot arm, DH116 L-shaped gripper (16 joints via EtherCAT), and Orbbec Gemini 330 depth camera. Built on ROS 2 Jazzy with a Qt5 GUI.

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
# Specify robot IP and EtherCAT interface index
ros2 launch ui_app unified.launch.py robot_ip:=192.168.192.10 ethercat_channel:=0
```

Launch arguments: `robot_ip` (default: 192.168.192.10), `ethercat_channel` (default: 0), `model` (default: gcr5_910).

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
    ├─→ lhandpro_interfaces services (21 services)            → lhandpro_service → DH116 Hand (EtherCAT)
    └─→ /image_saver/save_image                               → vision_server → Orbbec Camera
```

**Node startup order** (from `unified.launch.py`):
1. `vision_server` — Orbbec camera + image saver
2. `duco_ros_driver` — Robot arm driver (publishes `/duco_cobot/robot_state`)
3. `robot_state_publisher` — TF from combined arm+hand URDF
4. `lhandpro_service` — EtherCAT hand driver (21 services via `lhandpro_interfaces`)
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
| `lhandpro_service` | DH116 hand EtherCAT driver (uses SOEM + proprietary `LHandProLib`) |
| `lhandpro_description` | Maps 6 motor angles → 16 joint positions for TF |
| `duco_support` | URDF models for all DUCO arm variants + combined `duco_gcr5_910_with_dh116_lhand.urdf` |
| `vision_server` | `SaveImage` service, Orbbec Gemini 330 integration |

## Key Topics & Services

- `/duco_cobot/robot_state` — Robot arm state (joints, cartesian, temp, current)
- `/joint_states` — All joints (arm + hand)
- `/tf`, `/tf_static` — Transform frames
- `/camera/color/image_raw`, `/camera/depth/image_raw` — Camera streams
- `/ui/request_move|request_control|request_io` — UI → system_controller proxy
- `/system/pause_task` (`std_srvs/SetBool`) — Pause/resume task execution
- `execute_task` — `common_msgs/action/ExecuteTask` action server

## Notable Implementation Details

- **EtherCAT capabilities**: `lhandpro_service` requires `cap_net_raw` socket capabilities. If it fails to connect, run: `setcap cap_net_raw=ep install/lhandpro_service/lib/lhandpro_service/lhandpro_service`
- **Library path**: May need `export LD_LIBRARY_PATH=$PWD/install/lhandpro_service/lib:$LD_LIBRARY_PATH`
- **Qt rendering**: If 3D view fails, try `export QT_OPENGL=software`
- **UI executor**: Uses `SingleThreadedExecutor` with a 10ms QTimer calling `spin_some()` to integrate ROS and Qt event loops
- **System Controller state**: `is_busy_` and `is_paused_` are thread-safe atomics; `STATE_ENABLE = 6` is the required robot state for motion
- **URDF selection**: For `model=gcr5_910`, the combined `duco_gcr5_910_with_dh116_lhand.urdf` is used; other models use `duco_<model>.urdf`

## Debugging

```bash
ros2 topic echo /duco_cobot/robot_state    # Robot arm state
ros2 topic echo /joint_states              # All joint positions
ros2 topic echo /lhandpro_service/now_angles  # Hand motor angles
ros2 service list                           # All available services
```
