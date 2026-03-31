---
name: duco-robot-control
description: >
  DUCO collaborative robot arm control patterns, API reference, and common pitfalls for this project.
  Trigger when working with DUCO robot arm motion (movej, movej2, movel), robot state management,
  DucoRobotControl driver, system_controller task execution, DH116 LHand gripper EtherCAT control,
  or debugging DucoTaskState errors (ST_Illegal, ST_Error, ST_Interrupt). Also trigger for unit
  conversions (degrees/radians, mm/meters), ExecuteTask action, robot enable/disable/poweron/poweroff,
  or any DUCO cobot ROS 2 integration in this workspace.
---

# DUCO Robot Control Skill

## When to Use This Skill
- Implementing or modifying robot arm motion commands (movej, movej2, movel)
- Debugging DucoTaskState errors (ST_Illegal=7, ST_Error=6, etc.)
- Working with system_controller task execution logic
- Integrating DH116 LHand gripper (EtherCAT) control
- Unit conversions between UI (degrees/mm) and ROS (radians/meters)
- Robot state management (enable, disable, poweron, poweroff)
- Writing or modifying ExecuteTask action server/client code
- Troubleshooting motion failures or communication issues

## DUCO Robot Model

### GCR5-910 Specifications
- **DOF**: 6-axis (CRITICAL: never pad to 7 joints)
- **Driver node**: `duco_ros_driver` (`DucoRobotControl`)
- **State topic**: `/duco_cobot/robot_state` (DucoRobotState)
- **`_dof`**: Defaults to 6 in driver header

### DucoTaskState Constants
```cpp
// From DucoCobot.h
ST_Finished  = 4   // Motion completed successfully
ST_Interrupt = 5   // Motion interrupted (e.g. new command)
ST_Error     = 6   // Runtime error during motion
ST_Illegal   = 7   // Illegal command (bad params, not enabled, e-stop)
```

### Robot State Constants
```cpp
STATE_ENABLE = 6   // Robot is enabled and ready for motion
// Check: current_state_.robot_state == STATE_ENABLE before sending motion
```

## Motion Commands API

### movej — Joint Motion (Percentage Velocity)
```cpp
// DucoRobotControl::movej
// velocity: 0-100 (percentage of max speed)
// joints: exactly 6 elements (radians)
request->command = "movej";
request->joints = {j1, j2, j3, j4, j5, j6};  // radians, 6 DOF only
request->v = 50.0;    // percentage 0-100
request->a = 0.0;
request->r = 0.0;
request->block = true;
// Does NOT use tool, wobj, or q_near
```

### movej2 — Joint Motion (rad/s Velocity)
```cpp
// DucoRobotControl::movej2 (line ~200-218 in DucoRobotControl.cpp)
// velocity: rad/s (typical range: 0.1 ~ 1.25*PI)
// CRITICAL: Does NOT take tool or wobj parameters
request->command = "movej2";
request->joints = {j1, j2, j3, j4, j5, j6};  // radians, 6 DOF only
request->v = 1.0;     // rad/s
request->a = 1.0;     // rad/s²
request->r = 0.0;
request->block = true;
// NO tool, NO wobj — sending these causes ST_Illegal(7)
```

### movel — Linear (Cartesian) Motion
```cpp
// DucoRobotControl::movel (line ~292-327 in DucoRobotControl.cpp)
// velocity: m/s
// REQUIRES: tool, wobj, q_near (joints for IK seed)
request->command = "movel";
request->pose = {x, y, z, rx, ry, rz};  // meters + radians
request->v = 0.1;     // m/s
request->a = 0.5;     // m/s²
request->r = 0.0;
request->block = true;
request->tool = "default";   // REQUIRED
request->wobj = "default";   // REQUIRED
request->q_near = current_joints;  // 6 elements, for IK seed
```

### Velocity Conversion: movej2 (rad/s) → movej (percentage)
```cpp
// Max joint speed ≈ 1.25 * PI rad/s
double v_pct = (v_rad_s / (1.25 * M_PI)) * 100.0;
v_pct = std::clamp(v_pct, 1.0, 100.0);
```

## Fallback Pattern: movej2 → movej

When movej2 returns ST_Illegal(7) but robot is enabled, fall back to movej:

```cpp
if (result_code == 7 && current_state_.robot_state == STATE_ENABLE
    && request->command == "movej2") {
    RCLCPP_WARN(get_logger(), "movej2 returned ST_Illegal, falling back to movej");
    auto fallback = std::make_shared<duco_msg::srv::RobotMove::Request>();
    fallback->command = "movej";
    fallback->joints = request->joints;  // same 6 joints
    double v_pct = (request->v / (1.25 * M_PI)) * 100.0;
    fallback->v = std::clamp(v_pct, 1.0, 100.0);
    fallback->a = 0.0;
    fallback->r = 0.0;
    fallback->block = true;
    // Send fallback request...
}
```

## Control Commands

### Arm Control
```cpp
// Via /duco_robot/robot_control service (RobotControl.srv)
// Commands: "poweron", "enable", "disable", "poweroff"
request->command = "enable";

// IMPORTANT: Wait ~2000ms after enable for robot settling
// before sending any motion commands
```

### LHand (DH116) Control
```cpp
// 21 services via lhandpro_interfaces
// Key services:
//   set_enable    — enable/disable motors
//   home_motors   — MUST call before move_motors (otherwise LER_NOT_HOME=11)
//   move_motors   — move to target angles

// Error codes:
//   LER_NOT_HOME = 11  — motors not homed, call home_motors first
//   0 = success

// EtherCAT requires cap_net_raw:
// sudo setcap cap_net_raw=ep install/lhandpro_service/lib/lhandpro_service/lhandpro_service
```

### Lift Control
```cpp
// Via RS485 serial, services: lift_enable, lift_disable, lift_move
// Similar enable/disable pattern as arm
```

## Unit Conversions

### UI ↔ ROS Conventions
```
UI (display)          ROS (internal)        Conversion
─────────────────────────────────────────────────────────
Joint angles: °       Joint angles: rad     * M_PI / 180.0
Cart XYZ: mm          Cart XYZ: m           / 1000.0
Cart RxRyRz: °        Cart RxRyRz: rad      * M_PI / 180.0
movej velocity: °/s   movej velocity: %     (v_deg / max_deg) * 100
movej2 velocity: °/s  movej2 velocity: rad/s  * M_PI / 180.0
movel velocity: mm/s  movel velocity: m/s   / 1000.0
```

### Code Examples
```cpp
// Radians → Degrees (for display)
double deg = rad * 180.0 / M_PI;

// Degrees → Radians (for ROS commands)
double rad = deg * M_PI / 180.0;

// mm → meters
double m = mm / 1000.0;

// meters → mm
double mm = m * 1000.0;
```

## Task Execution (ExecuteTask Action)

### Action Definition
```
# common_msgs/action/ExecuteTask
# Goal
common_msgs/TaskConfig config
---
# Result
bool success
string message
---
# Feedback
int32 current_step
int32 total_steps
string step_description
string status
```

### publish_feedback Safety
```cpp
// CRITICAL: Always guard publish_feedback with is_canceling() check
// Calling publish_feedback after goal cancellation causes SIGSEGV/terminate
if (!goal_handle->is_canceling()) {
    auto feedback = std::make_shared<ExecuteTask::Feedback>();
    feedback->current_step = step_index;
    feedback->total_steps = total;
    feedback->step_description = description;
    feedback->status = "executing";
    goal_handle->publish_feedback(feedback);
}
```

### block=true for Sequential Execution
```cpp
// IMPORTANT: Always use block=true for task step execution
// block=false returns immediately, causing steps to overlap
request->block = true;
```

## Common Pitfalls & Debugging

### ST_Illegal(7) Causes (Priority Order)
1. **Emergency stop pressed** — most common, check physical button first
2. **Robot not enabled** — verify `robot_state == 6` (STATE_ENABLE)
3. **Wrong DOF count** — must send exactly 6 joints for GCR5-910
4. **tool/wobj sent to movej2** — movej2 does NOT accept these
5. **Invalid joint values** — out of range or NaN
6. **movel missing tool/wobj/q_near** — all three required

### Debugging Commands
```bash
# Check robot state (robot_state should be 6 for enabled)
ros2 topic echo /duco_cobot/robot_state --once

# Check all joint positions
ros2 topic echo /joint_states --once

# Check hand motor angles
ros2 topic echo /lhandpro_service/now_angles --once

# List all services
ros2 service list | grep duco

# Check if robot_move service is available
ros2 service type /duco_robot/robot_move
```

### Qt + ROS 2 Integration
```cpp
// UI uses SingleThreadedExecutor with 10ms QTimer
// NEVER use QPointer<AppWindow> in lambdas — causes "incomplete type" error
// Use raw pointer: AppWindow* self = this;

// tr_ui() is a GLOBAL inline function, NOT a class member
// Call: tr_ui("text")  NOT: this->tr_ui("text")
```

## Key File Locations

```
ros2_ws/src/duco_ros_driver/src/DucoRobotControl.cpp   — Driver implementation
ros2_ws/src/duco_ros_driver/include/.../DucoCobot.h     — API & constants
ros2_ws/src/system_controller/src/system_controller_node.cpp — Task executor
ros2_ws/src/ui_app/src/app_window.cpp                   — Manual control UI
ros2_ws/src/ui_app/src/task_dialogs.cpp                 — Task step editor
ros2_ws/src/ui_app/src/task_widget.cpp                  — Task list & JSON persistence
ros2_ws/src/ui_app/src/ros_node.cpp                     — ROS service call wrappers
ros2_ws/src/common_msgs/msg/TaskStep.msg                — Step message definition
ros2_ws/src/common_msgs/action/ExecuteTask.action       — Task action definition
```

## TaskStep Message Fields
```
# common_msgs/msg/TaskStep.msg
string type              # "arm", "lhand", "delay", "io", "control"
string arm_command       # "movej"/"movej2"/"movel" (for type=arm)
float64[] arm_pos        # Joint positions in degrees (6 elements)
float64[] arm_cart_pos   # Cartesian pose [x,y,z,rx,ry,rz] mm/degrees (for movel)
float64 arm_velocity     # Velocity (units depend on command)
float64 arm_accel        # Acceleration
string control_target    # "arm"/"lhand"/"rhand"/"lift" (for type=control)
string control_command   # "poweron"/"enable"/"disable"/"poweroff"/"home"
int32[] lhand_angles     # LHand motor angles (for type=lhand)
int32 delay_ms           # Delay in milliseconds (for type=delay)
# ... io fields
```

## Step JSON Serialization (.steps.json)
```json
{
  "source_task": "任务名称",
  "steps": [
    {
      "type": "arm",
      "arm_command": "movej2",
      "arm_pos": [0.0, -90.0, 0.0, 0.0, 90.0, 0.0],
      "arm_velocity": 1.0,
      "arm_accel": 1.0
    },
    {
      "type": "control",
      "control_target": "arm",
      "control_command": "enable"
    }
  ]
}
```
