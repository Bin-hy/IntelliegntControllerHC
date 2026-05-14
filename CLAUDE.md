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

# Build single package + its dependencies
colcon build --packages-up-to <package_name>

# Build single package only (skip deps)
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash
```

## Testing

```bash
# Run all tests
colcon test

# Run tests for a single package
colcon test --packages-select <package_name>

# Show test output
colcon test-result --all --verbose

# Run tests and show output inline
colcon test --event-handlers console_direct+
```

## Running the System

```bash
cd ~/IntelliegntControllerHC/ros2_ws
source install/setup.bash
ros2 launch ui_app unified.launch.py robot_ip:=192.168.1.10 ethercat_channel:=0
```

### Full Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | 192.168.1.10 | DUCO robot arm IP |
| `ethercat_channel` | 1 | Ethernet interface index for DH116 hand |
| `model` | gcr5_910 | DUCO robot model (selects matching URDF) |
| `hand_side` | left | DH116 hand side: `left` or `right` |
| `camera1_serial` | (empty) | Serial of primary Orbbec camera (auto-detect if empty) |
| `camera2_serial` | (empty) | Serial of secondary Orbbec camera (empty = single-camera mode) |
| `depth_camera_serial` | (empty) | Serial of depth measurement camera (Gemini 215 for earphone inspection) |
| `lift_serial_port` | /dev/ttyUSB0 | RS485 serial port for lift platform |
| `grasp_camera_ns` | /CV2R1610004H | Namespace for vision grasp camera (empty = disabled) |

### Orbbec Camera Setup

```bash
sudo cp ~/IntelliegntControllerHC/ros2_ws/install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

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
| `vision_server` | Camera services: `SaveImage`, `MeasureDepth`, `CaptureBaseline`, `MeasureEarphone` (earphone angle/depth inspection) |
| `lift_server` | RS485 Modbus RTU lift platform control (`LiftControl` service: move_up/down, position mode, status) |
| `vision_grasp` | Vision-guided grasping using Orbbec camera (bottle detection, grasp planning) |
| `glove_node` | UDEXREAL motion capture glove reader (UDP port 5555, publishes `glove/joints`) |
| `glove_hand_bridge` | Maps glove joint data → DH116 6-motor positions with EMA smoothing, deadzone, enable/disable |
| `collision_detector` | Collision detection between robot arm and environment |
| `safe_duco_status` | Safety monitoring for robot state |
| `logger_node` | Centralized logging node |

## Key Topics & Services

- `/duco_cobot/robot_state` — Robot arm state (joints, cartesian, temp, current)
- `/joint_states` — All joints (arm + hand)
- `/tf`, `/tf_static` — Transform frames
- `/camera/color/image_raw`, `/camera/depth/image_raw` — Camera streams
- `/ui/request_move|request_control|request_io` — UI → system_controller proxy
- `/system/pause_task` (`std_srvs/SetBool`) — Pause/resume task execution
- `execute_task` — `common_msgs/action/ExecuteTask` action server
- `/earphone_inspector/capture_baseline` — Capture baseline image for earphone inspection
- `/earphone_inspector/measure` — Measure earphone angle/depth from baseline
- `/lift_server/lift_control` — RS485 lift platform control (move_up, move_down, position, status, debug)
- `/glove_hand_bridge/enable` (`std_srvs/SetBool`) — Enable/disable glove→hand teleoperation

## Services by Hardware Device

### Vision Server (earphone inspection)
```bash
# Capture baseline (no earphone in place)
ros2 service call /earphone_inspector/capture_baseline vision_server/srv/CaptureBaseline
# Measure earphone position after placement
ros2 service call /earphone_inspector/measure vision_server/srv/MeasureEarphone "{file_tag: 'test1'}"
# Tune detection sensitivity
ros2 param set /earphone_inspector/earphone_inspector_node noise_sigma_scale 2.0
```

### Lift Server (RS485 Modbus RTU)
```bash
ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'move_up', speed_rpm: 1000}"
ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'status'}"
ros2 service call /lift_server/lift_control lift_server/srv/LiftControl "{command: 'debug', speed_rpm: 0}"
```

### Glove-Hand Bridge (motion capture teleoperation)
```bash
ros2 service call /glove_hand_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

## Task System

Tasks are JSON-persisted sequences of steps executed by `system_controller` via the `ExecuteTask` action. Step types defined in `common_msgs/msg/TaskStep.msg`:

| Step Type | Description |
|---|---|
| `arm` | DUCO robot arm movement (MoveJ with joint velocity, MoveL with end-effector speed) |
| `lhand` | DH116 hand 6-motor positions (0-10000 range) |
| `camera` | Capture baseline or measure earphone via vision_server |
| `io` | DIO digital output control (DO1-DO10 for cleaning, drying, lights, etc.) |
| `lift` | RS485 lift platform position/speed control |
| `control` | Flow control (wait, delay in ms) |

Task execution history records step-level results including depth/angle measurements and saved image paths.

## UI Auth & Permissions

Three-tier role system: **Operator** (basic run/stop/view), **Maintainer** (full manual control, task editing, calibration), **Admin** (user management, log/photo deletion). Login/logout via user avatar dropdown. Idle auto-logout via `idle_logout_watcher.cpp`.

## Notable Implementation Details

- **UI executor**: Uses `SingleThreadedExecutor` with a 10ms QTimer calling `spin_some()` to integrate ROS and Qt event loops
- **System Controller state**: `is_busy_` and `is_paused_` are thread-safe atomics; `STATE_ENABLE = 6` is the required robot state for motion
- **URDF selection**: For `model=gcr5_910`, the combined `duco_gcr5_910_with_dh116_lhand.urdf` is used; other models use `duco_<model>.urdf`
- **Glove integration**: UDEXREAL motion capture data is received via UDE C++ SDK (UDP, port 6321). The glove tab in ui_app displays 30 finger joints real-time. `glove_hand_bridge` maps glove data to DH116 motor positions with EMA smoothing and deadzone filtering.
- **Serial protocol**: Lift platform uses RS485 Modbus RTU (19200 baud, 8 data bits, 1 stop bit, even parity, slave ID 1). Supports both velocity mode and position mode control.

## Debugging

```bash
# Robot state
ros2 topic echo /duco_cobot/robot_state
ros2 topic echo /joint_states
ros2 topic echo /lhandpro_service/now_angles    # Hand motor angles

# Camera
ros2 topic echo /camera/color/image_raw --no-arr
ros2 topic hz /camera/color/image_raw          # Check frame rate

# Services & params
ros2 service list
ros2 param list /earphone_inspector/earphone_inspector_node
ros2 param dump /earphone_inspector/earphone_inspector_node

# TF tree
ros2 run tf2_tools view_frames

# Node graph
ros2 run rqt_graph rqt_graph
```

### Common Issues

- **EtherCAT connection fails**: `setcap cap_net_raw=ep install/lhandpro_service/lib/lhandpro_service/lhandpro_service`
- **Library path**: `export LD_LIBRARY_PATH=$PWD/install/lhandpro_service/lib:$LD_LIBRARY_PATH`
- **Qt 3D rendering fails**: `export QT_OPENGL=software`
- **HD service for glove**: `sudo systemctl restart hdservice` (glove data relay, web UI at http://localhost:8088/)
