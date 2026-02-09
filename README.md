#  初次clone安装依赖
```bash
cd ros2_ws
rosdep init 
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# 安装orbbec依赖
```bash

```
# 启动所有
```bash
cd ~/IntelliegntControllerHC/ros2_ws
source install/setup.bash
ros2 launch ui_app unified.launch.py
```
1.  **Qt 多线程冲突 (`QSocketNotifier`)**:
    ROS2 的 `rclcpp::spin()` 默认会监听 DDS 的 socket 文件描述符。你之前的代码在主线程创建了 Qt Application，却在一个独立的 `std::thread` 中运行 ROS `spin()`。这会导致 ROS 的 socket 信号和 Qt 的事件循环（Event Loop）发生冲突，特别是当两者都试图管理底层的 I/O 事件时。

    **修复方案**: 我修改了 `ui_app/src/main.cpp`，去掉了独立的线程，改用 Qt 原生的 `QTimer` 在主线程中定期调用 `exec.spin_some()`。这是一种经典的、线程安全的 ROS2 + Qt 集成模式。

2.  **Service not available**:
    UI 仅仅是一个前端界面，它并不直接控制机器臂，而是向 `system_controller` 发送请求。报错说明 **你没有启动系统控制器**。

### 下一步操作

请按照以下顺序，在 **4 个不同的终端** 中分别启动各个模块，即可正常工作：

**终端 1: 启动安全状态节点 (Safe Wrapper)**
这是我刚才为你写的、不会崩溃的新驱动节点。
```bash
cd /home/user/IntelliegntControllerHC/ros2_ws
source /home/user/IntelliegntControllerHC/duco_ros2_driver/install/setup.bash
source install/setup.bash
# 必须指定你的控制器IP
ros2 run safe_duco_status safe_duco_status_node --ros-args -p robot_ip:="192.168.192.10"
```

**终端 2: 启动官方控制服务**
用于执行具体的运动指令（Service Server）。
```bash
cd /home/user/IntelliegntControllerHC/duco_ros2_driver
source install/setup.bash
# 只启动控制服务，不启动状态反馈（避免冲突）
ros2 run duco_ros_driver DucoRobotControl --ros-args -p server_host_1:="192.168.192.10" -p arm_num:=1
```

**终端 3: 启动流程控制器 (System Controller)**
这是系统的“大脑”，负责接收 UI 指令并转发给驱动，同时进行安全检查。
```bash
cd /home/user/IntelliegntControllerHC/ros2_ws
source install/setup.bash
ros2 run system_controller system_controller_node
```

**终端 4: 启动 UI (重新编译后)**
我已经修复了多线程问题，请重新编译并运行。
```bash
cd /home/user/IntelliegntControllerHC/ros2_ws
source install/setup.bash
ros2 run ui_app ui_app
```

现在点击按钮应该就能正常控制了，且不会再报 `QSocketNotifier` 错误。