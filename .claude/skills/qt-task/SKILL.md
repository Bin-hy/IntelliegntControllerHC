---
name: qt-task-system
description: >
  Qt5 task management system for this industrial robot upper-computer project. Covers task lifecycle
  (create/edit/run/history), step types (arm/lhand/camera/io/lift/control), JSON persistence,
  drag-drop reordering, step import/export, ExecuteTask action execution with pause/resume/cancel,
  and task run feedback parsing. Trigger when working with TaskWidget, TaskConfigDialog, StepAddDialog,
  TaskRunDialog, task_widget.cpp, task_dialogs.cpp, TaskStep.msg, ExecuteTask.action, or any task
  step creation/editing/execution logic. Also trigger for task JSON save/load, .steps.json format,
  step drag-drop, step import/export, task history/records, or task device checks.
---

# Qt Task Management System Skill

## When to Use This Skill
- Creating or modifying task step types
- Working with TaskConfigDialog (step list, drag-drop, import/export)
- Working with StepAddDialog (step type UI, capture current position)
- Modifying task JSON persistence (tasks.json, .steps.json)
- Working with TaskRunDialog (execution, feedback, pause/resume)
- Adding new step types to the system
- Debugging task execution flow
- Working with task history or photo management

## Architecture Overview

```
TaskWidget (task_widget.cpp)
  ├── TaskConfigDialog (task_dialogs.cpp) — Edit task: devices + steps
  │     ├── DeviceAddDialog — Add device with scan
  │     └── StepAddDialog — Add/edit single step (type-specific UI)
  ├── TaskRunDialog (task_dialogs.cpp) — Run task: device check → execute → feedback
  └── History / Photos / CSV Export
```

### Data Flow
```
UI (TaskWidget)
  → tasks.json (QStandardPaths::AppConfigLocation)
  → TaskConfig msg (ROS 2)
  → ExecuteTask action (system_controller)
  → execute_*_step() per type
  → Feedback → TaskRunDialog → TaskRecordManager
```

## Key Files

```
ros2_ws/src/ui_app/src/task_widget.cpp          — Task list CRUD, JSON save/load, history, photos
ros2_ws/src/ui_app/include/ui_app/task_widget.hpp
ros2_ws/src/ui_app/src/task_dialogs.cpp          — All 4 dialogs: Device/Step/Config/Run
ros2_ws/src/ui_app/include/ui_app/task_dialogs.hpp
ros2_ws/src/common_msgs/msg/TaskConfig.msg       — Task definition (name, rounds, devices, steps)
ros2_ws/src/common_msgs/msg/TaskStep.msg         — Step definition (all fields for all types)
ros2_ws/src/common_msgs/msg/TaskDeviceCheck.msg  — Device check entry
ros2_ws/src/common_msgs/action/ExecuteTask.action — Action: goal=TaskConfig, feedback=step+status
ros2_ws/src/system_controller/src/system_controller_node.cpp — Step execution logic
```

## Message Definitions

### TaskConfig.msg
```
string task_name
int32 task_id
int64 created_time
int32 exec_rounds
string collision_camera_sn
TaskDeviceCheck[] device_checks
TaskStep[] task_seqs
```

### TaskStep.msg — All Step Types in One Message
```
string name
string type                # "arm"|"lhand"|"rhand"|"camera"|"io"|"lift"|"control"
string device_sn
# Arm
string arm_command         # "movej"|"movel"
float64[] arm_pos          # Joint degrees (6 elements, MoveJ)
float64[] arm_cart_pos     # [X,Y,Z mm, RX,RY,RZ degrees] (MoveL)
float64 arm_velocity       # MoveJ: deg/s, MoveL: mm/s
float64 arm_accel          # MoveJ: deg/s², MoveL: mm/s²
# Hand
int32[] hand_pos           # 6 finger positions
# Camera
string[] camera_type       # ["color"|"depth"|"ir_left"|"ir_right"|"point_cloud"|...]
# IO
int8 io_type               # 0=standard(DIO 1-16), 1=tool(1-2)
int8 io_port
bool io_value              # true=HIGH, false=LOW
# Delay
int32 delay_ms
# Lift
string lift_command        # move_up/move_down/stop/position_move/position_next/position_stop
int32 lift_speed_rpm
int32 lift_target_pulses
int32 lift_accel_ms
int32 lift_decel_ms
# Control
string control_target      # "arm"|"lhand"|"rhand"|"lift"
string control_command     # "poweron"|"enable"|"disable"|"poweroff"|"home"
```

### ExecuteTask.action
```
# Goal
TaskConfig task_config
---
# Result
bool success
string message
---
# Feedback
int32 current_step_index
string current_status      # Plain text or structured prefix (SAVED_FILE:, VISION_RESULT:, etc.)
```

## Step Types Reference

| Type | UI Widget | Key Fields | Execution |
|------|-----------|------------|-----------|
| arm | QStackedWidget: MoveJ (6 joint spins) / MoveL (6 cart spins) + velocity/accel | arm_command, arm_pos/arm_cart_pos, arm_velocity, arm_accel | execute_arm_step → /duco_robot/robot_move |
| lhand/rhand | 6 QSpinBox finger positions | hand_pos[6] | execute_hand_step → lhand services |
| camera | QComboBox camera type | camera_type[] | execute_camera_step → /image_saver/save_image |
| io | Group combo (Std 1-8/9-16/Tool) + port + HIGH/LOW | io_type, io_port, io_value | execute_io_step → /duco_robot/robot_io_control |
| lift | Command combo + speed/target/accel/decel | lift_command, lift_speed_rpm, lift_target_pulses, lift_accel_ms, lift_decel_ms | execute_lift_step → lift services |
| control | Target combo (arm/lhand/rhand/lift) + command combo (dynamic) | control_target, control_command | execute_control_step → poweron/enable/disable/poweroff/home |

### Control Step Command Matrix
```
Target    Available Commands
────────────────────────────────
arm       poweron, enable, disable, poweroff
lhand     enable, disable, home
rhand     enable, disable, home
lift      enable, disable
```

## StepAddDialog Patterns

### Type Switching with QStackedWidget
```cpp
auto * stack = new QStackedWidget();
stack->addWidget(widget_arm_);      // index 0
stack->addWidget(widget_hand_);     // index 1
stack->addWidget(widget_camera_);   // index 2
stack->addWidget(widget_io_);       // index 3
stack->addWidget(widget_lift_);     // index 4
stack->addWidget(widget_control_);  // index 5

// In onTypeChanged:
if (type == "arm") stack->setCurrentWidget(widget_arm_);
else if (type == "lhand" || type == "rhand") stack->setCurrentWidget(widget_hand_);
// ... etc
```

### Arm MoveJ/MoveL Toggle
```cpp
// combo_arm_command_ data: "movej" or "movel"
connect(combo_arm_command_, &QComboBox::currentTextChanged,
        this, &StepAddDialog::onArmCommandChanged);

void onArmCommandChanged(const QString&) {
    bool is_movej = (combo_arm_command_->currentData().toString() == "movej");
    widget_arm_movej_->setVisible(is_movej);
    widget_arm_movel_->setVisible(!is_movej);
    // Update velocity/accel ranges and suffixes
    if (is_movej) {
        spin_arm_velocity_->setRange(0.1, 180.0);  // deg/s
        spin_arm_velocity_->setSuffix(" °/s");
    } else {
        spin_arm_velocity_->setRange(0.1, 500.0);  // mm/s
        spin_arm_velocity_->setSuffix(" mm/s");
    }
}
```

### Capture Current Joint Position
```cpp
void onCaptureCurrent() {
    std::lock_guard<std::mutex> lock(node_->data_mutex_);
    if (node_->current_joints_.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            // ROS radians → UI degrees
            spin_arm_pos_[i]->setValue(node_->current_joints_[i] * 180.0 / M_PI);
        }
    }
}
```

### Dynamic Control Commands (onControlTargetChanged)
```cpp
void onControlTargetChanged(const QString&) {
    combo_control_command_->clear();
    QString t = combo_control_target_->currentData().toString();
    if (t == "arm") {
        combo_control_command_->addItem(tr_ui("上电", "Power On"), "poweron");
        combo_control_command_->addItem(tr_ui("使能", "Enable"), "enable");
        // ...
    } else if (t == "lhand" || t == "rhand") {
        // enable, disable, home
    } else if (t == "lift") {
        // enable, disable only
    }
}
```

### IO Group/Port Pattern
```cpp
// 3 groups: Standard DIO 1-8, Standard DIO 9-16, Tool IO 1-2
// Named ports for group 0 (industrial-specific):
// 1=清洗机, 2=左侧吹风机, 3=右侧烘干机, 4=升降平台解锁,
// 5=黄灯, 6=绿灯, 7=红灯, 8=蜂鸣器
combo_io_group_->addItem(tr_ui("标准 DIO (1-8)"), 0);
combo_io_group_->addItem(tr_ui("标准 DIO (9-16)"), 1);
combo_io_group_->addItem(tr_ui("工具 IO (1-2)"), 2);
// onIOGroupChanged populates combo_io_port_ based on group
```

## TaskConfigDialog Patterns

### Drag-Drop Step Reordering
```cpp
// Enable InternalMove drag-drop on QListWidget
list_steps_->setDragDropMode(QAbstractItemView::InternalMove);
list_steps_->setDefaultDropAction(Qt::MoveAction);

// Sync steps_ vector when items are reordered
connect(list_steps_->model(), &QAbstractItemModel::rowsMoved,
    this, [this](const QModelIndex&, int start, int, const QModelIndex&, int dest) {
    int to = (dest > start) ? dest - 1 : dest;
    auto step = steps_[start];
    steps_.erase(steps_.begin() + start);
    steps_.insert(steps_.begin() + to, step);
    // CRITICAL: Update numbering IN-PLACE (no clear/rebuild during drag!)
    for (int i = 0; i < list_steps_->count(); ++i) {
        auto * item = list_steps_->item(i);
        item->setText(QString("%1. [%2] %3")
            .arg(i + 1)
            .arg(QString::fromStdString(steps_[i].type))
            .arg(QString::fromStdString(steps_[i].name)));
        item->setData(Qt::UserRole, i);
    }
});
```

### Step List Display Format
```
"1. [arm] 回原点"
"2. [control] 使能机械臂"
"3. [lhand] 张开手指"
```

### Step Import/Export (.steps.json)
```json
{
  "source_task": "入耳式",
  "steps": [
    {
      "step_name": "回原点",
      "step_type": "arm",
      "device_sn": "",
      "arm_command": "movej",
      "arm_pos": [0.0, -90.0, 0.0, 0.0, 90.0, 0.0],
      "arm_cart_pos": [],
      "arm_velocity": 30.0,
      "arm_accel": 60.0,
      "hand_pos": [],
      "camera_type": [],
      "io_type": 0,
      "io_port": 0,
      "io_value": false,
      "delay_ms": 0,
      "lift_command": "",
      "lift_speed_rpm": 0,
      "lift_target_pulses": 0,
      "lift_accel_ms": 0,
      "lift_decel_ms": 0,
      "control_target": "",
      "control_command": ""
    }
  ]
}
```

### Three Import Sources
1. **From other tasks** (onImportSteps): Dialog with task selector + multi-select step list
2. **From .steps.json file** (onImportFromFile): QFileDialog open
3. **Export to file** (onExportSteps): QFileDialog save

### stepToJson / stepFromJson (Static Methods)
```cpp
// Serializes ALL TaskStep fields to QJsonObject — used by both
// .steps.json export AND could be reused for clipboard/network transfer
static QJsonObject stepToJson(const common_msgs::msg::TaskStep& step);
static common_msgs::msg::TaskStep stepFromJson(const QJsonObject& obj);
```

## Task JSON Persistence (tasks.json)

### Storage Location
```cpp
QDir dir(QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation));
QString path = dir.filePath("tasks.json");
// Typically: ~/.config/<app>/tasks.json
```

### Format: Array of TaskConfig
```json
[
  {
    "task_name": "入耳式",
    "exec_rounds": 3,
    "device_checks": [
      { "device_type": "duco", "device_name": "GCR5-910", "device_sn": "...", "device_usage": "主臂" }
    ],
    "task_seqs": [
      { "step_name": "...", "step_type": "arm", ... }
    ]
  }
]
```

### Save/Load Lifecycle
```
TaskWidget::loadTasks()  — on construction, reads tasks.json → tasks_ vector → updateTaskList()
TaskWidget::saveTasks()  — after every add/edit/delete, writes tasks_ → tasks.json
TaskWidget::onAddTask()  — TaskConfigDialog → dlg.setAllTasks(tasks_) → push_back → save
TaskWidget::onEditTask() — TaskConfigDialog → dlg.setAllTasks(tasks_) → setTask → replace → save
```

## Task Execution Flow

### TaskRunDialog Lifecycle
```
1. Constructor → checkDevices() after 500ms delay
2. checkDevices() → node_->is_task_action_ready() + check_device_availability()
3. onStart() → node_->call_execute_task(task_, result_cb, feedback_cb)
4. onTaskFeedback() → update progress bar + parse structured prefixes
5. onTaskResult() → show success/failure + save record
6. onPause() → node_->call_pause_task(true/false) toggle
7. onStop() → node_->cancel_current_task() + reject()
```

### system_controller Execution Loop
```cpp
for (int round = 0; round < rounds; ++round) {
    for (const auto& step : goal->task_config.task_seqs) {
        // 1. Wait for devices ready (cancel check)
        // 2. Handle pause (condition variable wait)
        // 3. Check canceling
        // 4. Publish feedback with step index
        // 5. Dispatch by step.type:
        if (step.type == "arm")          execute_arm_step(step, goal_handle);
        else if (step.type == "lhand")   execute_hand_step(step, goal_handle);
        else if (step.type == "camera")  execute_camera_step(...);
        else if (step.type == "io")      execute_io_step(step, goal_handle);
        else if (step.type == "lift")    execute_lift_step(step, goal_handle);
        else if (step.type == "control") execute_control_step(step, goal_handle);
        // 6. Handle delay_ms if > 0
        // 7. On failure → abort with error message
    }
}
```

### Feedback Structured Prefixes
```cpp
// Parsed in TaskRunDialog::onTaskFeedback()
"SAVED_FILE:/path/to/file.png"           — Camera step saved a photo
"VISION_RESULT:angle,depth,confidence,path" — Vision measurement result
"VISION_BASELINE_OK:message"             — Baseline capture success
// All other strings shown as plain status text
```

## Common Pitfalls

### Drag-Drop: Never clear/rebuild during drag
```cpp
// WRONG: updateStepList() calls list_steps_->clear() which breaks mid-drag state
// RIGHT: Update item text and UserRole data in-place within rowsMoved handler
```

### setAllTasks() Must Be Called Before setTask()
```cpp
TaskConfigDialog dlg(node_, this);
dlg.setAllTasks(tasks_);  // FIRST — enables "Import from Task" feature
dlg.setTask(tasks_[row]); // THEN — populate the dialog
```

### publish_feedback Guard
```cpp
// ALWAYS check before publishing — calling after cancel causes SIGSEGV
if (!goal_handle->is_canceling()) goal_handle->publish_feedback(feedback);
```

### Step delay_ms Is Per-Step, Not a Separate Type
```cpp
// Every step has delay_ms field (default 0)
// Applied AFTER step execution in system_controller
// UI: "步骤后延时 (ms)" spin box in StepAddDialog (range 0-3600000)
```

### i18n: Use tr_ui() Global Function
```cpp
// tr_ui is a GLOBAL inline function, NOT a QObject member
// Usage: tr_ui("中文", "English")
// WRONG: this->tr_ui(...)
// RIGHT: tr_ui(...)
```

### QComboBox Data Pattern
```cpp
// Store machine-readable value in itemData, display human-readable text
combo->addItem(tr_ui("关节运动 (MoveJ)", "Joint Move (MoveJ)"), "movej");
// Read: combo->currentData().toString()  → "movej"
// NOT:  combo->currentText()             → "关节运动 (MoveJ)"
```

## Adding a New Step Type Checklist

1. **TaskStep.msg**: Add new fields for the step type
2. **StepAddDialog** (task_dialogs.hpp/cpp):
   - Add widget member (`QWidget* widget_newtype_`)
   - Add input widgets (spins, combos, etc.)
   - Add to QStackedWidget in constructor
   - Handle in `onTypeChanged()` — set stack + populate device combo
   - Handle in `getStep()` — read widget values into TaskStep
   - Handle in `setStep()` — populate widgets from TaskStep
3. **combo_type_**: Add the new type string
4. **TaskConfigDialog**: stepToJson/stepFromJson — serialize new fields
5. **task_widget.cpp**: saveTasks/loadTasks — serialize new fields in tasks.json
6. **system_controller_node.cpp**:
   - Add `execute_newtype_step()` method
   - Add dispatch in execution loop: `else if (step.type == "newtype")`
7. **Rebuild**: `colcon build --packages-select common_msgs system_controller ui_app`
